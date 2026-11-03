// License: Apache 2.0. See LICENSE file in root directory.
//
// UV atlas generation with xatlas: charts, packing into one texture_size x texture_size atlas, seam vertex split.

#include "fr_mesh.h"
#include "fr_texture.h"
#include "../mesh/mesh_internal.h"

#include "xatlas.h"

#include <atomic>
#include <cfloat>
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

// Faces per chart group: xatlas computes the charts of each connected same-material face group on its own worker
// thread, and its chart growth gets slow on large groups (seconds per 100k faces).
constexpr uint32_t faces_per_group = 10000;

// For each corner edge a->b of face f, the face containing b->a (-1 on boundaries).
std::vector< int32_t > face_neighbors( const fr_mesh & m )
{
    const csr vf = vertex_faces( m.vertex_count, m.indices, m.triangle_count );
    std::vector< int32_t > nb( size_t( m.triangle_count ) * 3, -1 );
    parallel_for( m.triangle_count, 8192, [&]( size_t begin, size_t end ) {
        for( size_t f = begin; f < end; ++f )
            for( int k = 0; k < 3; ++k )
            {
                const uint32_t a = m.indices[f * 3 + k], b = m.indices[f * 3 + ( k + 1 ) % 3];
                for( uint32_t i = vf.begin( b ); i < vf.end( b ); ++i )
                {
                    const uint32_t g = vf.data[i];
                    const uint32_t * t = m.indices + size_t( g ) * 3;
                    if( g != f && ( ( t[0] == b && t[1] == a ) || ( t[1] == b && t[2] == a ) || ( t[2] == b && t[0] == a ) ) )
                    {
                        nb[f * 3 + k] = int32_t( g );
                        break;
                    }
                }
            }
    } );
    return nb;
}

// Splits the faces into `groups` compact, connected clusters (k-means on centroid and normal, small islands merged
// into their neighbors). Used as xatlas materials: the groups are charted in parallel, and no chart can span a
// whole closed component.
std::vector< uint32_t > partition_faces( const fr_mesh & m, const float * positions, uint32_t groups )
{
    const uint32_t t = m.triangle_count;
    std::vector< uint32_t > label( t, 0 );
    if( groups <= 1 )
        return label;
    struct feature
    {
        float v[6];
    };
    // Positions are in atlas units (1000 across); normals weigh like ~150 units, so clusters follow the surface
    // orientation as well as its position and their borders tend to run along curved areas.
    constexpr float normal_weight = 150.f;
    std::vector< feature > features( t );
    parallel_for( t, 8192, [&]( size_t begin, size_t end ) {
        for( size_t f = begin; f < end; ++f )
        {
            const uint32_t * tri = m.indices + f * 3;
            const vec3 a = vec3::load( positions + size_t( tri[0] ) * 3 ), b = vec3::load( positions + size_t( tri[1] ) * 3 ),
                       c = vec3::load( positions + size_t( tri[2] ) * 3 );
            const vec3 centroid = ( a + b + c ) * ( 1.f / 3.f ), n = normalized( cross( b - a, c - a ) ) * normal_weight;
            features[f] = { { centroid.x, centroid.y, centroid.z, n.x, n.y, n.z } };
        }
    } );
    auto distance2 = []( const feature & a, const feature & b ) {
        float d = 0;
        for( int k = 0; k < 6; ++k )
            d += ( a.v[k] - b.v[k] ) * ( a.v[k] - b.v[k] );
        return d;
    };

    // Deterministic farthest-point seeding and Lloyd iterations on a subsample.
    const uint32_t stride = std::max< uint32_t >( 1, t / 50000 );
    std::vector< uint32_t > sample;
    for( uint32_t f = 0; f < t; f += stride )
        sample.push_back( f );
    std::vector< feature > centers{ features[sample[0]] };
    std::vector< float > nearest( sample.size(), FLT_MAX );
    while( centers.size() < groups )
    {
        size_t best = 0;
        for( size_t i = 0; i < sample.size(); ++i )
        {
            nearest[i] = std::min( nearest[i], distance2( features[sample[i]], centers.back() ) );
            if( nearest[i] > nearest[best] )
                best = i;
        }
        centers.push_back( features[sample[best]] );
    }
    auto closest = [&]( const feature & x ) {
        uint32_t best = 0;
        float d = FLT_MAX;
        for( uint32_t c = 0; c < centers.size(); ++c )
        {
            const float dc = distance2( x, centers[c] );
            if( dc < d )
            {
                d = dc;
                best = c;
            }
        }
        return best;
    };
    std::vector< uint32_t > sample_label( sample.size() );
    for( int it = 0; it < 12; ++it )
    {
        parallel_for( sample.size(), 4096, [&]( size_t b, size_t e ) {
            for( size_t i = b; i < e; ++i )
                sample_label[i] = closest( features[sample[i]] );
        } );
        std::vector< double > sum( centers.size() * 6, 0.0 );
        std::vector< uint32_t > count( centers.size(), 0 );
        for( size_t i = 0; i < sample.size(); ++i )
        {
            for( int k = 0; k < 6; ++k )
                sum[sample_label[i] * 6 + k] += features[sample[i]].v[k];
            ++count[sample_label[i]];
        }
        for( size_t c = 0; c < centers.size(); ++c )
            if( count[c] )
                for( int k = 0; k < 6; ++k )
                    centers[c].v[k] = float( sum[c * 6 + k] / count[c] );
    }
    parallel_for( t, 8192, [&]( size_t b, size_t e ) {
        for( size_t f = b; f < e; ++f )
            label[f] = closest( features[f] );
    } );

    // Every group should be one connected patch: smaller pieces join the neighboring group they share most edges
    // with (two passes settle pieces whose neighbor moved in the first one).
    const std::vector< int32_t > nb = face_neighbors( m );
    std::vector< int32_t > component( t );
    for( int pass = 0; pass < 3; ++pass )
    {
        std::fill( component.begin(), component.end(), -1 );
        std::vector< std::vector< uint32_t > > pieces;
        std::vector< uint32_t > stack;
        for( uint32_t f = 0; f < t; ++f )
        {
            if( component[f] >= 0 )
                continue;
            pieces.emplace_back();
            stack.assign( 1, f );
            component[f] = int32_t( pieces.size() - 1 );
            while( ! stack.empty() )
            {
                const uint32_t g = stack.back();
                stack.pop_back();
                pieces.back().push_back( g );
                for( int k = 0; k < 3; ++k )
                {
                    const int32_t h = nb[size_t( g ) * 3 + k];
                    if( h >= 0 && component[h] < 0 && label[h] == label[g] )
                    {
                        component[h] = component[f];
                        stack.push_back( uint32_t( h ) );
                    }
                }
            }
        }
        std::vector< size_t > largest( groups, 0 );
        std::vector< int32_t > largest_piece( groups, -1 );
        for( size_t p = 0; p < pieces.size(); ++p )
        {
            const uint32_t g = label[pieces[p][0]];
            if( pieces[p].size() > largest[g] )
            {
                largest[g] = pieces[p].size();
                largest_piece[g] = int32_t( p );
            }
        }
        bool moved = false;
        for( size_t p = 0; p < pieces.size(); ++p )
        {
            const uint32_t g = label[pieces[p][0]];
            if( largest_piece[g] == int32_t( p ) )
                continue;
            std::vector< uint32_t > shared( groups, 0 );
            for( uint32_t f : pieces[p] )
                for( int k = 0; k < 3; ++k )
                {
                    const int32_t h = nb[size_t( f ) * 3 + k];
                    if( h >= 0 && label[h] != g )
                        ++shared[label[h]];
                }
            const uint32_t target = uint32_t( std::max_element( shared.begin(), shared.end() ) - shared.begin() );
            if( shared[target] == 0 )
                continue;  // a separate component of the mesh: stays a group of its own
            for( uint32_t f : pieces[p] )
                label[f] = target;
            moved = true;
        }
        if( ! moved )
            break;
    }
    return label;
}

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

        // xatlas drops faces with an area below FLT_EPSILON as degenerate, which in meters includes ordinary
        // sub-millimeter triangles of a scan (and broken charts made of them can crash its packer). The atlas does
        // not depend on the scale, so the positions go in centered and scaled to a 1000 unit bounding box.
        float lo[3], hi[3];
        if( fr_mesh_bounds( mesh, lo, hi ) != FR_OK )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_uv_unwrap: the mesh has no finite vertices" );
        const vec3 center( 0.5f * ( lo[0] + hi[0] ), 0.5f * ( lo[1] + hi[1] ), 0.5f * ( lo[2] + hi[2] ) );
        const float extent = std::max( { hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2] } );
        if( ! ( extent > 0 ) || ! std::isfinite( extent ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_uv_unwrap: degenerate mesh extent" );
        const float to_atlas_units = 1000.f / extent;
        // xatlas also turns every set of faces connected through edges whose normals differ by less than ~0.8 degrees
        // into one "planar" chart, projected orthographically. On a dense smooth scan that chain can swallow a whole
        // curved component (overlapping UVs; a closed one crashes the packer). A deterministic jitter of 3% of the
        // edge length along the normals (a few degrees of normal noise) breaks those chains. Only the charting sees
        // it: the output keeps the exact positions, and the parameterization changes by a negligible amount.
        std::vector< float > normals( size_t( mesh->vertex_count ) * 3 );
        compute_vertex_normals( mesh->positions, mesh->vertex_count, mesh->indices, mesh->triangle_count, normals.data() );
        const float jitter
            = 0.03f * float( mean_edge_length( mesh->positions, mesh->indices, mesh->triangle_count ) ) * to_atlas_units;
        std::vector< float > positions( size_t( mesh->vertex_count ) * 3 );
        for( uint32_t v = 0; v < mesh->vertex_count; ++v )
        {
            const vec3 p = vec3::load( mesh->positions + size_t( v ) * 3 );
            if( ! finite( p ) )
                return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_uv_unwrap: non-finite vertex position" );
            uint32_t h = v * 0x9E3779B1u;
            h ^= h >> 15;
            h *= 0x2C1B3C6Du;
            h ^= h >> 12;
            const float r = float( h & 0xFFFF ) / 32767.5f - 1.f;  // [-1, 1]
            ( ( p - center ) * to_atlas_units + vec3::load( &normals[size_t( v ) * 3] ) * ( jitter * r ) )
                .store( &positions[size_t( v ) * 3] );
        }

        xatlas::MeshDecl decl;
        decl.vertexPositionData = positions.data();
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
        const uint32_t groups = std::clamp< uint32_t >( ( mesh->triangle_count + faces_per_group / 2 ) / faces_per_group, 1, 64 );
        const std::vector< uint32_t > materials = partition_faces( *mesh, positions.data(), groups );
        if( groups > 1 )
            decl.faceMaterialData = materials.data();
        const xatlas::AddMeshError added = xatlas::AddMesh( atlas.get(), decl );
        if( added != xatlas::AddMeshError::Success )
            return fr::fail( FR_ERR_INVALID_ARGUMENT,
                             std::string( "fr_mesh_uv_unwrap: " ) + xatlas::StringForEnum( added ) );

        xatlas::ChartOptions charts;
        if( params->max_chart_area > 0 )
            charts.maxChartArea = params->max_chart_area * to_atlas_units * to_atlas_units;  // m^2 -> atlas units
        if( params->max_cost > 0 )
            charts.maxCost = params->max_cost;
        charts.fixWinding = true;
        xatlas::ComputeCharts( atlas.get(), charts );
        if( bridge.cancelled )
            return fr::fail( FR_ERR_CANCELLED, "fr_mesh_uv_unwrap: cancelled" );

        // With only a resolution, xatlas estimates the texel density assuming 75% packing efficiency and returns a
        // single atlas of whatever size the charts need. One re-pack with the corrected density brings it close to
        // the requested size (packing is randomized, iterating further does not converge); the UVs are then scaled
        // so that the atlas spans the texture.
        const int size = params->texture_size;
        xatlas::PackOptions pack;
        pack.padding = uint32_t( params->padding_px );
        pack.resolution = uint32_t( size );
        pack.bilinear = true;
        xatlas::PackCharts( atlas.get(), pack );
        if( ! bridge.cancelled && atlas->meshCount == 1 && atlas->texelsPerUnit > 0 )
        {
            const float extent = float( std::max( atlas->width, atlas->height ) );
            const float ratio = extent > 0 ? float( size ) / extent : 1.f;
            if( ratio < 0.97f || ratio > 1.1f )
            {
                pack.resolution = 0;
                pack.texelsPerUnit = atlas->texelsPerUnit * ratio * 0.985f;
                xatlas::PackCharts( atlas.get(), pack );
            }
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
        // The atlas spans the texture along its longer side (the re-pack keeps the scale change, and with it the
        // change of the padding, within a few percent).
        const float scale = 1.f / float( std::max( atlas->width, atlas->height ) );
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
