// License: Apache 2.0. See LICENSE file in root directory.
//
// Detail-preserving smoothing: bilateral normal filtering with vertex position update (Zheng et al. 2011,
// "Bilateral Normal Filtering for Mesh Denoising"), followed by Taubin lambda/mu smoothing whose Laplacian weights
// fall off across sharp features.

#include "fr_mesh.h"
#include "mesh_internal.h"

using namespace fr::mesh;

namespace
{

// Vertex updates per bilateral iteration: enough to bring the vertices close to the filtered normal field.
constexpr int vertex_update_steps = 10;

struct face_data
{
    std::vector< vec3 > normal, centroid;
    std::vector< float > area;
};

void compute_faces( const float * p, const uint32_t * idx, uint32_t t, face_data & fd, bool need_normals )
{
    parallel_for( t, 8192, [&]( size_t begin, size_t end ) {
        for( size_t f = begin; f < end; ++f )
        {
            const uint32_t * tri = idx + f * 3;
            const vec3 a = vec3::load( p + size_t( tri[0] ) * 3 ), b = vec3::load( p + size_t( tri[1] ) * 3 ),
                       c = vec3::load( p + size_t( tri[2] ) * 3 );
            fd.centroid[f] = ( a + b + c ) * ( 1.f / 3.f );
            if( need_normals )
            {
                const vec3 cr = cross( b - a, c - a );
                const float l = length( cr );
                fd.area[f] = 0.5f * l;
                fd.normal[f] = l > 0 ? cr / l : vec3();
            }
        }
    } );
}

inline bool contains( const uint32_t * tri, uint32_t v ) { return tri[0] == v || tri[1] == v || tri[2] == v; }

// Calls fn(g) once for every face sharing at least one vertex with face f (including f itself).
template< typename F >
inline void for_each_ring_face( const csr & vf, const uint32_t * idx, uint32_t f, F && fn )
{
    const uint32_t * tri = idx + size_t( f ) * 3;
    for( int k = 0; k < 3; ++k )
        for( uint32_t i = vf.begin( tri[k] ); i < vf.end( tri[k] ); ++i )
        {
            const uint32_t g = vf.data[i];
            const uint32_t * other = idx + size_t( g ) * 3;
            // Visit g only from the first of f's vertices it contains.
            if( k >= 1 && contains( other, tri[0] ) )
                continue;
            if( k == 2 && contains( other, tri[1] ) )
                continue;
            fn( g );
        }
}

void bilateral_filter( fr_mesh * mesh, const csr & vf, int iterations, float sigma_s, float sigma_r )
{
    const uint32_t n = mesh->vertex_count, t = mesh->triangle_count;
    const uint32_t * idx = mesh->indices;
    face_data fd;
    fd.normal.resize( t );
    fd.centroid.resize( t );
    fd.area.resize( t );
    std::vector< vec3 > filtered( t );
    std::vector< float > next( size_t( n ) * 3 );
    const float inv_2ss = 1.f / ( 2.f * sigma_s * sigma_s ), inv_2sr = 1.f / ( 2.f * sigma_r * sigma_r );

    for( int it = 0; it < iterations; ++it )
    {
        compute_faces( mesh->positions, idx, t, fd, true );
        parallel_for( t, 4096, [&]( size_t begin, size_t end ) {
            for( size_t f = begin; f < end; ++f )
            {
                const vec3 nf = fd.normal[f], cf = fd.centroid[f];
                vec3 sum;
                for_each_ring_face( vf, idx, uint32_t( f ), [&]( uint32_t g ) {
                    const float ws = std::exp( -length2( fd.centroid[g] - cf ) * inv_2ss );
                    const float wr = std::exp( -length2( fd.normal[g] - nf ) * inv_2sr );
                    sum += fd.normal[g] * ( fd.area[g] * ws * wr );
                } );
                filtered[f] = normalized( sum, nf );
            }
        } );

        // Move every vertex toward the planes of its faces given by the filtered normals (Sun et al. 2007 update,
        // as used by Zheng et al.); the centroids follow the moving vertices.
        for( int step = 0; step < vertex_update_steps; ++step )
        {
            if( step > 0 )
                compute_faces( mesh->positions, idx, t, fd, false );
            const float * p = mesh->positions;
            parallel_for( n, 4096, [&]( size_t begin, size_t end ) {
                for( size_t v = begin; v < end; ++v )
                {
                    const vec3 x = vec3::load( p + v * 3 );
                    vec3 delta;
                    const uint32_t count = vf.size( uint32_t( v ) );
                    for( uint32_t i = vf.begin( uint32_t( v ) ); i < vf.end( uint32_t( v ) ); ++i )
                    {
                        const uint32_t g = vf.data[i];
                        delta += filtered[g] * dot( filtered[g], fd.centroid[g] - x );
                    }
                    const vec3 y = count ? x + delta / float( count ) : x;
                    ( finite( y ) ? y : x ).store( &next[v * 3] );
                }
            } );
            std::copy( next.begin(), next.end(), mesh->positions );
        }
    }
}

void taubin_smooth( fr_mesh * mesh, int iterations, float lambda, float mu, bool feature_weights, float sigma_r )
{
    const uint32_t n = mesh->vertex_count, t = mesh->triangle_count;
    const uint32_t * idx = mesh->indices;
    const csr nb = vertex_neighbors( n, idx, t );
    const csr out = outgoing_edges( n, idx, t );

    // Boundary vertices are smoothed along the boundary only (a 1D Laplacian), so open cuts neither shrink nor get
    // pulled inward; vertices where several boundaries meet stay fixed.
    constexpr uint32_t none = ~0u;
    std::vector< uint32_t > boundary_count( n, 0 ), boundary_nb( size_t( n ) * 2, none );
    auto add_boundary = [&]( uint32_t a, uint32_t b ) {
        if( boundary_count[a] < 2 )
            boundary_nb[size_t( a ) * 2 + boundary_count[a]] = b;
        ++boundary_count[a];
    };
    for( uint32_t a = 0; a < n; ++a )
        for( uint32_t k = out.begin( a ); k < out.end( a ); ++k )
        {
            const uint32_t b = out.data[k];
            if( ! has_edge( out, b, a ) )
            {
                add_boundary( a, b );
                add_boundary( b, a );
            }
        }

    std::vector< float > weight( nb.data.size(), 1.f );
    if( feature_weights )
    {
        std::vector< float > vn( size_t( n ) * 3 );
        compute_vertex_normals( mesh->positions, n, idx, t, vn.data() );
        const float inv_2sr = 1.f / ( 2.f * sigma_r * sigma_r );
        parallel_for( n, 4096, [&]( size_t begin, size_t end ) {
            for( size_t v = begin; v < end; ++v )
            {
                const vec3 ni = vec3::load( &vn[v * 3] );
                for( uint32_t i = nb.begin( uint32_t( v ) ); i < nb.end( uint32_t( v ) ); ++i )
                    weight[i] = std::exp( -length2( vec3::load( &vn[size_t( nb.data[i] ) * 3] ) - ni ) * inv_2sr );
            }
        } );
    }

    std::vector< float > next( size_t( n ) * 3 );
    auto step = [&]( float factor ) {
        const float * p = mesh->positions;
        parallel_for( n, 8192, [&]( size_t begin, size_t end ) {
            for( size_t v = begin; v < end; ++v )
            {
                const vec3 x = vec3::load( p + v * 3 );
                vec3 avg;
                float wsum = 0;
                const uint32_t bc = boundary_count[v];
                if( bc == 0 )
                {
                    for( uint32_t i = nb.begin( uint32_t( v ) ); i < nb.end( uint32_t( v ) ); ++i )
                    {
                        avg += vec3::load( p + size_t( nb.data[i] ) * 3 ) * weight[i];
                        wsum += weight[i];
                    }
                }
                else if( bc == 2 )
                {
                    avg = vec3::load( p + size_t( boundary_nb[v * 2] ) * 3 ) + vec3::load( p + size_t( boundary_nb[v * 2 + 1] ) * 3 );
                    wsum = 2;
                }
                vec3 y = x;
                if( wsum > 1e-12f )
                    y = x + ( avg / wsum - x ) * factor;
                ( finite( y ) ? y : x ).store( &next[v * 3] );
            }
        } );
        std::copy( next.begin(), next.end(), mesh->positions );
    };
    for( int it = 0; it < iterations; ++it )
    {
        step( lambda );
        step( mu );
    }
}

}  // namespace

extern "C" {

void fr_smooth_params_default( fr_smooth_params * p )
{
    if( ! p )
        return;
    std::memset( p, 0, sizeof( *p ) );
    p->taubin_iterations = 5;
    p->taubin_lambda = 0.5f;
    p->taubin_mu = -0.53f;
    p->bilateral_iterations = 2;
    p->bilateral_sigma_spatial_m = 0.0015f;
    p->bilateral_sigma_normal = 0.3f;
}

fr_status fr_mesh_smooth( fr_mesh * mesh, const fr_smooth_params * params )
{
    return fr::guarded( "fr_mesh_smooth", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_mesh_smooth" );
        if( s != FR_OK )
            return s;
        if( ! params )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_smooth: params is NULL" );
        const fr_smooth_params & p = *params;
        if( p.taubin_iterations < 0 || p.bilateral_iterations < 0 || ! std::isfinite( p.taubin_lambda )
            || ! std::isfinite( p.taubin_mu ) || ! std::isfinite( p.bilateral_sigma_spatial_m )
            || ! std::isfinite( p.bilateral_sigma_normal ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_smooth: invalid parameters" );
        if( p.taubin_iterations > 0
            && ( p.taubin_lambda <= 0 || p.taubin_lambda >= 1 || p.taubin_mu >= 0 || p.taubin_mu <= -1 ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT,
                             "fr_mesh_smooth: Taubin needs 0 < lambda < 1 and -1 < mu < 0 (e.g. 0.5 / -0.53)" );
        if( mesh->triangle_count == 0 )
            return FR_OK;

        // Sigmas <= 0 select automatic values: the spatial one from the mesh resolution.
        const float sigma_r = p.bilateral_sigma_normal > 0 ? p.bilateral_sigma_normal : 0.3f;
        if( p.bilateral_iterations > 0 )
        {
            float sigma_s = p.bilateral_sigma_spatial_m;
            if( sigma_s <= 0 )
                sigma_s = float( 1.5 * mean_edge_length( mesh->positions, mesh->indices, mesh->triangle_count ) );
            if( sigma_s > 0 )
            {
                const csr vf = vertex_faces( mesh->vertex_count, mesh->indices, mesh->triangle_count );
                bilateral_filter( mesh, vf, p.bilateral_iterations, sigma_s, sigma_r );
            }
        }
        if( p.taubin_iterations > 0 )
            taubin_smooth( mesh, p.taubin_iterations, p.taubin_lambda, p.taubin_mu, p.bilateral_iterations > 0, sigma_r );
        if( mesh->normals )
            compute_vertex_normals( mesh->positions, mesh->vertex_count, mesh->indices, mesh->triangle_count,
                                    mesh->normals );
        return FR_OK;
    } );
}

}  // extern "C"
