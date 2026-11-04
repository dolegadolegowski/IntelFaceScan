// License: Apache 2.0. See LICENSE file in root directory.
//
// Internal helpers of the mesh, texture and export code (not part of the public C API): small vector math, a
// std::vector based working copy of fr_mesh, adjacency structures and a parallel loop over GCD.

#pragma once

#include "fr_types.h"
#include "../common/fr_internal.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <mutex>
#include <vector>

#include <dispatch/dispatch.h>

namespace fr
{
namespace mesh
{

// ---------------------------------------------------------------------------------------------------------------
// Vector math

struct vec3
{
    float x = 0, y = 0, z = 0;

    vec3() = default;
    constexpr vec3( float x_, float y_, float z_ )
        : x( x_ )
        , y( y_ )
        , z( z_ )
    {
    }
    static vec3 load( const float * p ) { return { p[0], p[1], p[2] }; }
    void store( float * p ) const
    {
        p[0] = x;
        p[1] = y;
        p[2] = z;
    }

    vec3 operator+( const vec3 & o ) const { return { x + o.x, y + o.y, z + o.z }; }
    vec3 operator-( const vec3 & o ) const { return { x - o.x, y - o.y, z - o.z }; }
    vec3 operator-() const { return { -x, -y, -z }; }
    vec3 operator*( float s ) const { return { x * s, y * s, z * s }; }
    vec3 operator/( float s ) const { return { x / s, y / s, z / s }; }
    vec3 & operator+=( const vec3 & o )
    {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
    vec3 & operator-=( const vec3 & o )
    {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }
    vec3 & operator*=( float s )
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    float operator[]( int i ) const { return i == 0 ? x : ( i == 1 ? y : z ); }
};

inline vec3 operator*( float s, const vec3 & v ) { return v * s; }
inline float dot( const vec3 & a, const vec3 & b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline vec3 cross( const vec3 & a, const vec3 & b )
{
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
inline float length2( const vec3 & v ) { return dot( v, v ); }
inline float length( const vec3 & v ) { return std::sqrt( dot( v, v ) ); }
inline vec3 normalized( const vec3 & v, const vec3 & fallback = vec3( 0, 0, 1 ) )
{
    float l = length( v );
    return l > 1e-30f && std::isfinite( l ) ? v / l : fallback;
}
inline bool finite( const vec3 & v ) { return std::isfinite( v.x ) && std::isfinite( v.y ) && std::isfinite( v.z ); }

// Affine transform y = A x + t, from a column-major float[16].
struct affine
{
    float m[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };  // m[row][col]
    vec3 t;

    static affine from_column_major( const float * c )
    {
        affine a;
        for( int r = 0; r < 3; ++r )
            for( int k = 0; k < 3; ++k )
                a.m[r][k] = c[k * 4 + r];
        a.t = { c[12], c[13], c[14] };
        return a;
    }
    vec3 apply( const vec3 & p ) const { return rotate( p ) + t; }
    vec3 rotate( const vec3 & v ) const
    {
        return { m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                 m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z };
    }
    double det() const
    {
        return double( m[0][0] ) * ( double( m[1][1] ) * m[2][2] - double( m[1][2] ) * m[2][1] )
             - double( m[0][1] ) * ( double( m[1][0] ) * m[2][2] - double( m[1][2] ) * m[2][0] )
             + double( m[0][2] ) * ( double( m[1][0] ) * m[2][1] - double( m[1][1] ) * m[2][0] );
    }
    // Inverse of the linear part; false when singular.
    bool inverse_linear( float out[3][3] ) const
    {
        double d = det();
        if( ! std::isfinite( d ) || std::fabs( d ) < 1e-20 )
            return false;
        double inv = 1.0 / d;
        out[0][0] = float( ( double( m[1][1] ) * m[2][2] - double( m[1][2] ) * m[2][1] ) * inv );
        out[0][1] = float( ( double( m[0][2] ) * m[2][1] - double( m[0][1] ) * m[2][2] ) * inv );
        out[0][2] = float( ( double( m[0][1] ) * m[1][2] - double( m[0][2] ) * m[1][1] ) * inv );
        out[1][0] = float( ( double( m[1][2] ) * m[2][0] - double( m[1][0] ) * m[2][2] ) * inv );
        out[1][1] = float( ( double( m[0][0] ) * m[2][2] - double( m[0][2] ) * m[2][0] ) * inv );
        out[1][2] = float( ( double( m[0][2] ) * m[1][0] - double( m[0][0] ) * m[1][2] ) * inv );
        out[2][0] = float( ( double( m[1][0] ) * m[2][1] - double( m[1][1] ) * m[2][0] ) * inv );
        out[2][1] = float( ( double( m[0][1] ) * m[2][0] - double( m[0][0] ) * m[2][1] ) * inv );
        out[2][2] = float( ( double( m[0][0] ) * m[1][1] - double( m[0][1] ) * m[1][0] ) * inv );
        return true;
    }
    bool inverse( affine & out ) const
    {
        if( ! inverse_linear( out.m ) )
            return false;
        out.t = -out.rotate( t );
        return true;
    }
    // Matrix for normals (inverse transpose of the linear part); false when singular.
    bool normal_matrix( affine & out ) const
    {
        float inv[3][3];
        if( ! inverse_linear( inv ) )
            return false;
        for( int r = 0; r < 3; ++r )
            for( int k = 0; k < 3; ++k )
                out.m[r][k] = inv[k][r];
        out.t = {};
        return true;
    }
    bool is_finite() const
    {
        for( auto & row : m )
            for( float v : row )
                if( ! std::isfinite( v ) )
                    return false;
        return finite( t );
    }
};

// ---------------------------------------------------------------------------------------------------------------
// Parallel loop

// Calls f(begin, end) for chunks of [0, count) on the GCD thread pool. An exception thrown by f is rethrown on the
// calling thread after all chunks finished (the remaining chunks still run).
template< typename F >
void parallel_for( size_t count, size_t grain, F && f )
{
    if( count == 0 )
        return;
    grain = std::max< size_t >( grain, 1 );
    const size_t chunks = ( count + grain - 1 ) / grain;
    if( chunks == 1 )
    {
        f( size_t( 0 ), count );
        return;
    }
    struct context
    {
        F * f;
        size_t count, grain;
        std::mutex mutex;
        std::exception_ptr error;
    } ctx{ &f, count, grain, {}, nullptr };
    dispatch_apply_f( chunks, DISPATCH_APPLY_AUTO, &ctx, []( void * p, size_t i ) {
        auto * c = static_cast< context * >( p );
        const size_t begin = i * c->grain, end = std::min( c->count, begin + c->grain );
        try
        {
            ( *c->f )( begin, end );
        }
        catch( ... )
        {
            std::lock_guard< std::mutex > lock( c->mutex );
            if( ! c->error )
                c->error = std::current_exception();
        }
    } );
    if( ctx.error )
        std::rethrow_exception( ctx.error );
}

// ---------------------------------------------------------------------------------------------------------------
// Working copy of a mesh

// fr_mesh with std::vector storage. Mesh operations that change the vertex or triangle count work on this and
// swap the result into the caller's fr_mesh (store), which leaves the input untouched when anything fails.
struct mesh_buffers
{
    std::vector< float > positions;   // 3 per vertex
    std::vector< float > normals;     // 3 per vertex or empty
    std::vector< uint8_t > colors;    // 3 per vertex or empty
    std::vector< float > uvs;         // 2 per vertex or empty
    std::vector< uint32_t > indices;  // 3 per triangle
    bool has_normals = false, has_colors = false, has_uvs = false;

    uint32_t vertex_count() const { return uint32_t( positions.size() / 3 ); }
    uint32_t triangle_count() const { return uint32_t( indices.size() / 3 ); }
    vec3 position( uint32_t v ) const { return vec3::load( &positions[size_t( v ) * 3] ); }
    void set_position( uint32_t v, const vec3 & p ) { p.store( &positions[size_t( v ) * 3] ); }

    // Appends a vertex whose attributes interpolate the given vertices with weights (summing to 1). Normals are
    // renormalized, colors rounded.
    uint32_t add_blend( const uint32_t * src, const float * w, int n );
    // Same attribute layout, no vertices.
    mesh_buffers empty_like() const;
    // Appends vertex src of other (same attribute layout).
    uint32_t add_copy( const mesh_buffers & other, uint32_t src );
    void reserve_vertices( size_t n );
    // Drops vertices no triangle references and renumbers the indices.
    void remove_unreferenced();
};

mesh_buffers load( const fr_mesh & mesh );
// Replaces the arrays of mesh with copies of b (allocated with malloc, as fr_mesh_free expects). On failure the
// mesh is unchanged.
fr_status store( const mesh_buffers & b, fr_mesh * mesh );

// Checks pointers and index ranges of a C mesh.
fr_status validate( const fr_mesh * mesh, const char * where, bool require_triangles = false );

// ---------------------------------------------------------------------------------------------------------------
// Adjacency

// Compressed rows: items of row i are data[offsets[i] .. offsets[i + 1]).
struct csr
{
    std::vector< uint32_t > offsets;
    std::vector< uint32_t > data;
    uint32_t begin( uint32_t i ) const { return offsets[i]; }
    uint32_t end( uint32_t i ) const { return offsets[i + 1]; }
    uint32_t size( uint32_t i ) const { return offsets[i + 1] - offsets[i]; }
};

// Triangles incident to each vertex.
csr vertex_faces( uint32_t vertex_count, const uint32_t * indices, uint32_t triangle_count );
// Unique neighbor vertices of each vertex (sorted).
csr vertex_neighbors( uint32_t vertex_count, const uint32_t * indices, uint32_t triangle_count );
// Targets of the directed edges a->b of the triangles, per source vertex a (one per incident corner, unsorted).
csr outgoing_edges( uint32_t vertex_count, const uint32_t * indices, uint32_t triangle_count );
// True when the directed edge a->b belongs to a triangle.
inline bool has_edge( const csr & out, uint32_t a, uint32_t b )
{
    for( uint32_t k = out.begin( a ); k < out.end( a ); ++k )
        if( out.data[k] == b )
            return true;
    return false;
}

// Mean edge length (each undirected edge counted once per incident triangle; 0 for an empty mesh).
double mean_edge_length( const float * positions, const uint32_t * indices, uint32_t triangle_count );

// For each vertex the lowest-numbered vertex at exactly the same position. Meshes with UVs have their vertices split
// on seams; topology-based operations use this to see through the seams.
std::vector< uint32_t > coincident_representatives( const float * positions, uint32_t vertex_count );
// indices with every vertex replaced by its representative.
std::vector< uint32_t > remap_indices( const uint32_t * indices, size_t count, const std::vector< uint32_t > & rep );

// Area-weighted vertex normals into normals (3 * vertex_count floats). share_coincident: vertices at exactly the same
// position (UV seam splits) get one common normal, so seams do not show in the shading.
void compute_vertex_normals( const float * positions, uint32_t vertex_count, const uint32_t * indices,
                             uint32_t triangle_count, float * normals, bool share_coincident = false );

// On every edge used twice in one direction or by more than two triangles, only the largest triangle per direction
// stays: keep[f] = 0 for the others (keep: one flag per triangle, 0 = already removed). Removing triangles never makes
// another edge non-manifold. Returns the number of triangles removed. Welding collapses short edges without regard to
// the topology (four marching-cubes vertices within a micrometer of a voxel corner merged into one left an edge with
// three triangles on a head scan), and so does quadric decimation.
uint32_t remove_nonmanifold( const mesh_buffers & b, std::vector< uint8_t > & keep );

// Fills the boundary loops with at most max_edges edges that pass through a marked vertex (mesh_holes.cpp: minimal
// triangulation, refined and faired, the variant with the fewest folds), the longest one included: the small holes an
// operation tore into a surface around the marked vertices. Returns the number of loops filled.
uint32_t fill_small_holes( mesh_buffers & b, uint32_t max_edges, const std::vector< uint8_t > & marked );

}  // namespace mesh
}  // namespace fr
