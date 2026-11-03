// License: Apache 2.0. See LICENSE file in root directory.
//
// Screened Poisson surface reconstruction of the registered, oriented cloud (comparison method, see
// docs/RECONSTRUCTION.md), with density trimming of the surface Poisson invents far from the samples.

#include "fr_fusion.h"
#include "../registration/recon_cloud.h"
#include "../third_party/poisson_recon.h"
#include "../common/fr_internal.h"

#include <algorithm>
#include <cstring>
#include <numeric>

extern "C" {

void fr_poisson_params_default( fr_poisson_params * p )
{
    if( ! p )
        return;
    std::memset( p, 0, sizeof( *p ) );
    p->depth = 10;
    p->point_weight = 4.f;
    p->samples_per_node = 1.5f;
    p->density_trim_quantile = 0.02f;
    p->threads = 0;
}

// threads: 0 = all cores, 1 = single-threaded; PoissonRecon's pool always uses every core otherwise.
fr_status fr_poisson_reconstruct( const fr_cloud * cloud, const fr_poisson_params * params, fr_mesh ** out,
                                  fr_progress_cb progress, void * user )
{
    return fr::guarded( "fr_poisson_reconstruct", [&]() -> fr_status {
        if( ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_poisson_reconstruct: out is NULL" );
        *out = nullptr;
        if( ! cloud )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_poisson_reconstruct: cloud is NULL" );
        fr_poisson_params p;
        if( params )
            p = *params;
        else
            fr_poisson_params_default( &p );
        if( p.depth < 5 || p.depth > 13 || ! ( p.point_weight >= 0 ) || ! ( p.samples_per_node > 0 )
            || ! ( p.density_trim_quantile >= 0 && p.density_trim_quantile < 0.5f ) || p.threads < 0 )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_poisson_reconstruct: invalid parameters" );
        if( ! cloud->has_normals() )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_poisson_reconstruct: the cloud needs oriented normals" );
        if( cloud->size() < 100 )
            return fr::fail( FR_ERR_NOT_ENOUGH_DATA, "fr_poisson_reconstruct: too few points" );
        if( ! fr::report( progress, user, 0.f, "poisson: solving" ) )
            return fr::fail( FR_ERR_CANCELLED, "fr_poisson_reconstruct: cancelled" );

        fr::poisson::Input input;
        input.points = cloud->points.data();
        input.normals = cloud->normals.data();
        input.colors = cloud->has_colors() ? cloud->colors.data() : nullptr;
        input.count = cloud->size();
        fr::poisson::Options options;
        options.depth = p.depth;
        options.point_weight = p.point_weight;
        options.samples_per_node = p.samples_per_node;
        options.parallel = p.threads != 1;
        fr::poisson::Output result;
        fr::poisson::reconstruct( input, options, result );
        if( ! fr::report( progress, user, 0.9f, "poisson: trimming" ) )
            return fr::fail( FR_ERR_CANCELLED, "fr_poisson_reconstruct: cancelled" );

        const size_t n = result.density.size();
        if( n == 0 || result.indices.empty() )
            return fr::fail( FR_ERR_FAILED, "fr_poisson_reconstruct: empty surface" );
        // Drop the least supported vertices (and their triangles): Poisson closes every hole with a smooth,
        // hallucinated membrane whose sample density is low.
        float threshold = -std::numeric_limits< float >::infinity();
        if( p.density_trim_quantile > 0 )
        {
            std::vector< float > sorted = result.density;
            const size_t k = std::min( n - 1, size_t( double( p.density_trim_quantile ) * double( n ) ) );
            std::nth_element( sorted.begin(), sorted.begin() + std::ptrdiff_t( k ), sorted.end() );
            threshold = sorted[k];
        }
        std::vector< uint32_t > remap( n, UINT32_MAX );
        std::vector< uint32_t > kept_triangles;
        kept_triangles.reserve( result.indices.size() );
        for( size_t t = 0; t + 2 < result.indices.size(); t += 3 )
        {
            const uint32_t a = result.indices[t], b = result.indices[t + 1], c = result.indices[t + 2];
            if( a >= n || b >= n || c >= n )
                return fr::fail( FR_ERR_FAILED, "fr_poisson_reconstruct: invalid triangle index" );
            if( result.density[a] < threshold || result.density[b] < threshold || result.density[c] < threshold )
                continue;
            kept_triangles.insert( kept_triangles.end(), { a, b, c } );   // already counter-clockwise from outside
        }
        uint32_t vertex_count = 0;
        for( uint32_t v : kept_triangles )
            if( remap[v] == UINT32_MAX )
                remap[v] = vertex_count++;

        fr_mesh * mesh = nullptr;
        const bool colored = ! result.colors.empty();
        fr_status status = fr_mesh_create( vertex_count, uint32_t( kept_triangles.size() / 3 ),
                                           FR_MESH_NORMALS | ( colored ? FR_MESH_COLORS : 0u ), &mesh );
        if( status != FR_OK )
            return status;
        std::unique_ptr< fr_mesh, void ( * )( fr_mesh * ) > owner( mesh, fr_mesh_free );
        for( size_t v = 0; v < n; ++v )
        {
            const uint32_t m = remap[v];
            if( m == UINT32_MAX )
                continue;
            std::memcpy( &mesh->positions[3 * size_t( m )], &result.positions[3 * v], 3 * sizeof( float ) );
            if( colored )
                for( int k = 0; k < 3; ++k )
                    mesh->colors[3 * size_t( m ) + k] = uint8_t( std::lround( std::clamp( result.colors[3 * v + k], 0.f, 1.f ) * 255.f ) );
        }
        for( size_t i = 0; i < kept_triangles.size(); ++i )
            mesh->indices[i] = remap[kept_triangles[i]];
        // Area-weighted vertex normals (cross products are proportional to the triangle area).
        std::vector< Eigen::Vector3f > normals( vertex_count, Eigen::Vector3f::Zero() );
        for( size_t t = 0; t < kept_triangles.size(); t += 3 )
        {
            const uint32_t a = mesh->indices[t], b = mesh->indices[t + 1], c = mesh->indices[t + 2];
            const Eigen::Map< const Eigen::Vector3f > pa( &mesh->positions[3 * size_t( a )] ), pb( &mesh->positions[3 * size_t( b )] ),
                pc( &mesh->positions[3 * size_t( c )] );
            const Eigen::Vector3f fn = ( pb - pa ).cross( pc - pa );
            normals[a] += fn;
            normals[b] += fn;
            normals[c] += fn;
        }
        for( uint32_t v = 0; v < vertex_count; ++v )
        {
            const float len = normals[v].norm();
            const Eigen::Vector3f nv = len > 0 ? Eigen::Vector3f( normals[v] / len ) : Eigen::Vector3f::UnitZ();
            std::memcpy( &mesh->normals[3 * size_t( v )], nv.data(), 3 * sizeof( float ) );
        }
        fr::report( progress, user, 1.f, "poisson: finished" );
        *out = owner.release();
        return FR_OK;
    } );
}

}  // extern "C"
