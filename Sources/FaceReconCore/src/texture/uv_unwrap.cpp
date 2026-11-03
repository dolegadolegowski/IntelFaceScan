// License: Apache 2.0. See LICENSE file in root directory.
//
// UV atlas generation with xatlas: charts, packing into one texture_size x texture_size atlas, seam vertex split.

#include "fr_texture.h"
#include "../mesh/mesh_internal.h"

#include "xatlas.h"

#include <atomic>
#include <memory>
#include <mutex>

using namespace fr::mesh;

namespace
{

struct progress_bridge
{
    fr_progress_cb callback;
    void * user;
    std::mutex mutex;  // xatlas reports from its worker threads; the C callback is serialized
    std::atomic< bool > cancelled{ false };
};

bool xatlas_progress( xatlas::ProgressCategory category, int percent, void * user )
{
    auto * bridge = static_cast< progress_bridge * >( user );
    if( bridge->cancelled.load() )
        return false;
    if( ! bridge->callback )
        return true;
    float lo = 0, hi = 0.05f;
    const char * stage = "uv: adding mesh";
    switch( category )
    {
    case xatlas::ProgressCategory::AddMesh: break;
    case xatlas::ProgressCategory::ComputeCharts:
        lo = 0.05f;
        hi = 0.7f;
        stage = "uv: computing charts";
        break;
    case xatlas::ProgressCategory::PackCharts:
        lo = 0.7f;
        hi = 0.95f;
        stage = "uv: packing charts";
        break;
    case xatlas::ProgressCategory::BuildOutputMeshes:
        lo = 0.95f;
        hi = 1.f;
        stage = "uv: building output";
        break;
    }
    std::lock_guard< std::mutex > lock( bridge->mutex );
    if( ! fr::report( bridge->callback, bridge->user, lo + ( hi - lo ) * float( percent ) / 100.f, stage ) )
        bridge->cancelled = true;
    return ! bridge->cancelled.load();
}

struct atlas_deleter
{
    void operator()( xatlas::Atlas * a ) const { xatlas::Destroy( a ); }
};

}  // namespace

extern "C" {

void fr_uv_params_default( fr_uv_params * p )
{
    if( ! p )
        return;
    std::memset( p, 0, sizeof( *p ) );
    p->texture_size = 4096;
    p->padding_px = 4;
}

fr_status fr_mesh_uv_unwrap( fr_mesh * mesh, const fr_uv_params * params, fr_progress_cb progress, void * user )
{
    return fr::guarded( "fr_mesh_uv_unwrap", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_mesh_uv_unwrap", true );
        if( s != FR_OK )
            return s;
        if( ! params || params->texture_size < 16 || params->texture_size > 32768 || params->padding_px < 0
            || params->padding_px > params->texture_size / 8 || ! std::isfinite( params->max_chart_area )
            || ! std::isfinite( params->max_cost ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_uv_unwrap: invalid parameters" );

        // xatlas prints warnings with printf by default; its print hook is global, so it is silenced once.
        static std::once_flag quiet;
        std::call_once( quiet, [] { xatlas::SetPrint( nullptr, false ); } );

        std::unique_ptr< xatlas::Atlas, atlas_deleter > atlas( xatlas::Create() );
        if( ! atlas )
            return fr::fail( FR_ERR_OUT_OF_MEMORY, "fr_mesh_uv_unwrap: xatlas::Create" );
        progress_bridge bridge{ progress, user, {}, { false } };
        xatlas::SetProgressCallback( atlas.get(), xatlas_progress, &bridge );

        xatlas::MeshDecl decl;
        decl.vertexPositionData = mesh->positions;
        decl.vertexPositionStride = sizeof( float ) * 3;
        decl.vertexCount = mesh->vertex_count;
        if( mesh->normals )
        {
            decl.vertexNormalData = mesh->normals;
            decl.vertexNormalStride = sizeof( float ) * 3;
        }
        decl.indexData = mesh->indices;
        decl.indexCount = mesh->triangle_count * 3;
        decl.indexFormat = xatlas::IndexFormat::UInt32;
        const xatlas::AddMeshError added = xatlas::AddMesh( atlas.get(), decl );
        if( added != xatlas::AddMeshError::Success )
            return fr::fail( FR_ERR_INVALID_ARGUMENT,
                             std::string( "fr_mesh_uv_unwrap: " ) + xatlas::StringForEnum( added ) );

        xatlas::ChartOptions charts;
        if( params->max_chart_area > 0 )
            charts.maxChartArea = params->max_chart_area;
        if( params->max_cost > 0 )
            charts.maxCost = params->max_cost;
        charts.fixWinding = true;
        xatlas::ComputeCharts( atlas.get(), charts );
        if( bridge.cancelled )
            return fr::fail( FR_ERR_CANCELLED, "fr_mesh_uv_unwrap: cancelled" );

        // With only a resolution, xatlas estimates the texel density assuming 75% packing efficiency and returns a
        // single atlas of whatever size the charts need. Re-pack with a corrected density until it fits the
        // requested size closely; any remaining overshoot is absorbed by scaling the UVs.
        const int size = params->texture_size;
        xatlas::PackOptions pack;
        pack.padding = uint32_t( params->padding_px );
        pack.resolution = uint32_t( size );
        pack.bilinear = true;
        pack.blockAlign = false;
        xatlas::PackCharts( atlas.get(), pack );
        for( int pass = 0; pass < 4 && ! bridge.cancelled && atlas->meshCount == 1; ++pass )
        {
            const float extent = float( std::max( atlas->width, atlas->height ) );
            if( extent <= 0 || atlas->texelsPerUnit <= 0 )
                break;
            const float ratio = float( size ) / extent;
            if( ratio >= 0.97f && ratio <= 1.f )
                break;
            pack.resolution = 0;
            pack.texelsPerUnit = atlas->texelsPerUnit * ratio * ( ratio > 1.f ? 0.99f : 0.985f );
            xatlas::PackCharts( atlas.get(), pack );
        }
        if( bridge.cancelled )
            return fr::fail( FR_ERR_CANCELLED, "fr_mesh_uv_unwrap: cancelled" );
        if( atlas->meshCount != 1 || atlas->atlasCount != 1 || ! atlas->meshes || atlas->width == 0 )
            return fr::fail( FR_ERR_FAILED, "fr_mesh_uv_unwrap: xatlas produced no atlas" );

        const xatlas::Mesh & om = atlas->meshes[0];
        if( om.indexCount != mesh->triangle_count * 3 )
            return fr::fail( FR_ERR_FAILED, "fr_mesh_uv_unwrap: unexpected xatlas output" );
        const mesh_buffers in = load( *mesh );
        mesh_buffers out = in.empty_like();
        out.has_uvs = true;
        out.reserve_vertices( om.vertexCount );
        // UVs: xatlas texel coordinates have their origin at the top-left of the image; OBJ UVs at the bottom-left.
        const float scale = 1.f / float( std::max( { atlas->width, atlas->height, uint32_t( size ) } ) );
        for( uint32_t v = 0; v < om.vertexCount; ++v )
        {
            const xatlas::Vertex & xv = om.vertexArray[v];
            if( xv.xref >= in.vertex_count() )
                return fr::fail( FR_ERR_FAILED, "fr_mesh_uv_unwrap: xatlas vertex reference out of range" );
            const size_t src = xv.xref;
            out.positions.insert( out.positions.end(), &in.positions[src * 3], &in.positions[src * 3] + 3 );
            if( out.has_normals )
                out.normals.insert( out.normals.end(), &in.normals[src * 3], &in.normals[src * 3] + 3 );
            if( out.has_colors )
                out.colors.insert( out.colors.end(), &in.colors[src * 3], &in.colors[src * 3] + 3 );
            out.uvs.push_back( xv.uv[0] * scale );
            out.uvs.push_back( 1.f - xv.uv[1] * scale );
        }
        out.indices.assign( om.indexArray, om.indexArray + om.indexCount );
        for( uint32_t i : out.indices )
            if( i >= om.vertexCount )
                return fr::fail( FR_ERR_FAILED, "fr_mesh_uv_unwrap: xatlas index out of range" );
        return store( out, mesh );
    } );
}

}  // extern "C"
