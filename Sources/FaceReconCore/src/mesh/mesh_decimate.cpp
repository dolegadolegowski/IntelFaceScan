// License: Apache 2.0. See LICENSE file in root directory.
//
// Quadric edge-collapse decimation through meshoptimizer (meshopt_simplifyWithAttributes) with locked borders.

#include "fr_mesh.h"
#include "mesh_internal.h"

#include "meshoptimizer.h"

#include <cfloat>

using namespace fr::mesh;

namespace
{
// Attribute weights relative to the position error, which meshoptimizer measures relative to the mesh extent
// (~0.3 m for a head): a 10% color change costs about as much as a 0.3 mm deviation; a UV shift of one texel of a
// 4096 atlas about as much as 0.07 mm.
constexpr float color_weight = 0.01f;
constexpr float normal_weight = 0.01f;
constexpr float uv_weight = 1.0f;
}  // namespace

extern "C" {

fr_status fr_mesh_decimate( fr_mesh * mesh, uint32_t target_triangles, float max_error_m )
{
    return fr::guarded( "fr_mesh_decimate", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_mesh_decimate" );
        if( s != FR_OK )
            return s;
        if( std::isnan( max_error_m ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_decimate: max_error_m is NaN" );
        if( target_triangles == 0 && ! ( max_error_m > 0 ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_decimate: give a triangle target or an error limit" );
        if( mesh->triangle_count == 0 || ( target_triangles >= mesh->triangle_count && target_triangles > 0 ) )
            return FR_OK;

        mesh_buffers b = load( *mesh );
        const size_t n = b.vertex_count();
        size_t attribute_count = 0;
        float weights[8];
        if( b.has_colors )
            for( int k = 0; k < 3; ++k )
                weights[attribute_count++] = color_weight;
        if( b.has_normals )
            for( int k = 0; k < 3; ++k )
                weights[attribute_count++] = normal_weight;
        if( b.has_uvs )
            for( int k = 0; k < 2; ++k )
                weights[attribute_count++] = uv_weight;
        std::vector< float > attributes( std::max< size_t >( n * attribute_count, 1 ), 0.f );
        for( size_t v = 0; v < n && attribute_count; ++v )
        {
            float * a = &attributes[v * attribute_count];
            if( b.has_colors )
                for( int k = 0; k < 3; ++k )
                    *a++ = b.colors[v * 3 + k] / 255.f;
            if( b.has_normals )
                for( int k = 0; k < 3; ++k )
                    *a++ = b.normals[v * 3 + k];
            if( b.has_uvs )
                for( int k = 0; k < 2; ++k )
                    *a++ = b.uvs[v * 2 + k];
        }

        std::vector< uint32_t > out( b.indices.size() );
        const size_t target_indices = size_t( target_triangles ) * 3;
        const float target_error = max_error_m > 0 ? max_error_m : FLT_MAX;
        const unsigned options = meshopt_SimplifyLockBorder | meshopt_SimplifyErrorAbsolute;
        float result_error = 0;
        size_t count;
        if( attribute_count )
            count = meshopt_simplifyWithAttributes( out.data(), b.indices.data(), b.indices.size(), b.positions.data(), n,
                                                    sizeof( float ) * 3, attributes.data(), sizeof( float ) * attribute_count,
                                                    weights, attribute_count, nullptr, target_indices, target_error,
                                                    options, &result_error );
        else
            count = meshopt_simplify( out.data(), b.indices.data(), b.indices.size(), b.positions.data(), n,
                                      sizeof( float ) * 3, target_indices, target_error, options, &result_error );
        out.resize( count );
        b.indices.swap( out );
        b.remove_unreferenced();
        if( b.has_normals )
            compute_vertex_normals( b.positions.data(), b.vertex_count(), b.indices.data(), b.triangle_count(),
                                    b.normals.data(), b.has_uvs );
        return store( b, mesh );
    } );
}

}  // extern "C"
