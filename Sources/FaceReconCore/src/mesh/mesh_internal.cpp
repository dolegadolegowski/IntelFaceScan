// License: Apache 2.0. See LICENSE file in root directory.
//
// Working copies of meshes, adjacency structures and vertex normals shared by the mesh/texture/export code.

#include "mesh_internal.h"

#include <cstdlib>
#include <string>

namespace fr
{
namespace mesh
{

uint32_t mesh_buffers::add_blend( const uint32_t * src, const float * w, int n )
{
    const uint32_t v = vertex_count();
    vec3 p, nrm;
    float c[3] = { 0, 0, 0 }, uv[2] = { 0, 0 };
    for( int i = 0; i < n; ++i )
    {
        const size_t s = src[i];
        p += vec3::load( &positions[s * 3] ) * w[i];
        if( has_normals )
            nrm += vec3::load( &normals[s * 3] ) * w[i];
        if( has_colors )
            for( int k = 0; k < 3; ++k )
                c[k] += float( colors[s * 3 + k] ) * w[i];
        if( has_uvs )
            for( int k = 0; k < 2; ++k )
                uv[k] += uvs[s * 2 + k] * w[i];
    }
    positions.insert( positions.end(), { p.x, p.y, p.z } );
    if( has_normals )
    {
        nrm = normalized( nrm );
        normals.insert( normals.end(), { nrm.x, nrm.y, nrm.z } );
    }
    if( has_colors )
        for( float x : c )
            colors.push_back( uint8_t( std::clamp( std::lround( x ), 0L, 255L ) ) );
    if( has_uvs )
        uvs.insert( uvs.end(), { uv[0], uv[1] } );
    return v;
}

mesh_buffers mesh_buffers::empty_like() const
{
    mesh_buffers b;
    b.has_normals = has_normals;
    b.has_colors = has_colors;
    b.has_uvs = has_uvs;
    return b;
}

uint32_t mesh_buffers::add_copy( const mesh_buffers & o, uint32_t src )
{
    const uint32_t v = vertex_count();
    const size_t s = src;
    positions.insert( positions.end(), &o.positions[s * 3], &o.positions[s * 3] + 3 );
    if( has_normals )
        normals.insert( normals.end(), &o.normals[s * 3], &o.normals[s * 3] + 3 );
    if( has_colors )
        colors.insert( colors.end(), &o.colors[s * 3], &o.colors[s * 3] + 3 );
    if( has_uvs )
        uvs.insert( uvs.end(), &o.uvs[s * 2], &o.uvs[s * 2] + 2 );
    return v;
}

void mesh_buffers::reserve_vertices( size_t n )
{
    positions.reserve( n * 3 );
    if( has_normals )
        normals.reserve( n * 3 );
    if( has_colors )
        colors.reserve( n * 3 );
    if( has_uvs )
        uvs.reserve( n * 2 );
}

void mesh_buffers::remove_unreferenced()
{
    const uint32_t n = vertex_count();
    constexpr uint32_t unused = ~0u;
    // Kept vertices stay in their original order (callers and tests rely on stable numbering).
    std::vector< uint32_t > remap( n, unused );
    for( uint32_t i : indices )
        remap[i] = 0;
    uint32_t next = 0;
    for( uint32_t & r : remap )
        if( r != unused )
            r = next++;
    if( next == n )
        return;
    for( uint32_t & i : indices )
        i = remap[i];
    auto gather = [&]( auto & array, size_t per_vertex ) {
        using T = typename std::decay_t< decltype( array ) >::value_type;
        if( array.empty() )
            return;
        std::vector< T > out( size_t( next ) * per_vertex );
        for( uint32_t v = 0; v < n; ++v )
            if( remap[v] != unused )
                std::memcpy( &out[size_t( remap[v] ) * per_vertex], &array[size_t( v ) * per_vertex], per_vertex * sizeof( T ) );
        array.swap( out );
    };
    gather( positions, 3 );
    gather( normals, 3 );
    gather( colors, 3 );
    gather( uvs, 2 );
}

mesh_buffers load( const fr_mesh & m )
{
    mesh_buffers b;
    const size_t n = m.vertex_count, t = m.triangle_count;
    if( n )
        b.positions.assign( m.positions, m.positions + n * 3 );
    if( t )
        b.indices.assign( m.indices, m.indices + t * 3 );
    b.has_normals = m.normals != nullptr;
    b.has_colors = m.colors != nullptr;
    b.has_uvs = m.uvs != nullptr;
    if( b.has_normals && n )
        b.normals.assign( m.normals, m.normals + n * 3 );
    if( b.has_colors && n )
        b.colors.assign( m.colors, m.colors + n * 3 );
    if( b.has_uvs && n )
        b.uvs.assign( m.uvs, m.uvs + n * 2 );
    return b;
}

namespace
{
template< typename T >
bool copy_out( const std::vector< T > & v, T ** out )
{
    *out = nullptr;
    if( v.empty() )
        return true;
    *out = static_cast< T * >( std::malloc( v.size() * sizeof( T ) ) );
    if( ! *out )
        return false;
    std::memcpy( *out, v.data(), v.size() * sizeof( T ) );
    return true;
}
}  // namespace

fr_status store( const mesh_buffers & b, fr_mesh * mesh )
{
    const size_t n = b.vertex_count();
    float *positions = nullptr, *normals = nullptr, *uvs = nullptr;
    uint8_t * colors = nullptr;
    uint32_t * indices = nullptr;
    // Attribute flags survive an empty result (the arrays are NULL then, as fr_mesh_create does for 0 vertices).
    bool ok = copy_out( b.positions, &positions ) && copy_out( b.indices, &indices )
           && ( ! b.has_normals || copy_out( b.normals, &normals ) ) && ( ! b.has_colors || copy_out( b.colors, &colors ) )
           && ( ! b.has_uvs || copy_out( b.uvs, &uvs ) );
    if( ok && ( ( b.has_normals && b.normals.size() != n * 3 ) || ( b.has_colors && b.colors.size() != n * 3 )
                || ( b.has_uvs && b.uvs.size() != n * 2 ) ) )
    {
        std::free( positions );
        std::free( normals );
        std::free( colors );
        std::free( uvs );
        std::free( indices );
        return fail( FR_ERR_FAILED, "internal error: inconsistent attribute arrays" );
    }
    if( ! ok )
    {
        std::free( positions );
        std::free( normals );
        std::free( colors );
        std::free( uvs );
        std::free( indices );
        return fail( FR_ERR_OUT_OF_MEMORY, "mesh storage" );
    }
    std::free( mesh->positions );
    std::free( mesh->normals );
    std::free( mesh->colors );
    std::free( mesh->uvs );
    std::free( mesh->indices );
    mesh->positions = positions;
    mesh->normals = normals;
    mesh->colors = colors;
    mesh->uvs = uvs;
    mesh->indices = indices;
    mesh->vertex_count = uint32_t( n );
    mesh->triangle_count = b.triangle_count();
    return FR_OK;
}

fr_status validate( const fr_mesh * m, const char * where, bool require_triangles )
{
    if( ! m )
        return fail( FR_ERR_INVALID_ARGUMENT, std::string( where ) + ": mesh is NULL" );
    if( ( m->vertex_count && ! m->positions ) || ( m->triangle_count && ! m->indices ) )
        return fail( FR_ERR_INVALID_ARGUMENT, std::string( where ) + ": missing positions or indices" );
    if( require_triangles && ( m->triangle_count == 0 || m->vertex_count < 3 ) )
        return fail( FR_ERR_NOT_ENOUGH_DATA, std::string( where ) + ": the mesh has no triangles" );
    const size_t count = size_t( m->triangle_count ) * 3;
    const uint32_t n = m->vertex_count;
    for( size_t i = 0; i < count; ++i )
        if( m->indices[i] >= n )
            return fail( FR_ERR_INVALID_ARGUMENT,
                         std::string( where ) + ": vertex index " + std::to_string( m->indices[i] ) + " out of range" );
    return FR_OK;
}

csr vertex_faces( uint32_t n, const uint32_t * idx, uint32_t t )
{
    csr c;
    c.offsets.assign( size_t( n ) + 1, 0 );
    for( size_t i = 0; i < size_t( t ) * 3; ++i )
        ++c.offsets[idx[i] + 1];
    for( uint32_t v = 0; v < n; ++v )
        c.offsets[v + 1] += c.offsets[v];
    c.data.resize( size_t( t ) * 3 );
    std::vector< uint32_t > fill( c.offsets.begin(), c.offsets.end() - 1 );
    for( uint32_t f = 0; f < t; ++f )
        for( int k = 0; k < 3; ++k )
            c.data[fill[idx[size_t( f ) * 3 + k]]++] = f;
    return c;
}

csr outgoing_edges( uint32_t n, const uint32_t * idx, uint32_t t )
{
    csr c;
    c.offsets.assign( size_t( n ) + 1, 0 );
    for( size_t i = 0; i < size_t( t ) * 3; ++i )
        ++c.offsets[idx[i] + 1];
    for( uint32_t v = 0; v < n; ++v )
        c.offsets[v + 1] += c.offsets[v];
    c.data.resize( size_t( t ) * 3 );
    std::vector< uint32_t > fill( c.offsets.begin(), c.offsets.end() - 1 );
    for( uint32_t f = 0; f < t; ++f )
    {
        const uint32_t * tri = idx + size_t( f ) * 3;
        for( int k = 0; k < 3; ++k )
            c.data[fill[tri[k]]++] = tri[( k + 1 ) % 3];
    }
    return c;
}

csr vertex_neighbors( uint32_t n, const uint32_t * idx, uint32_t t )
{
    // Every corner contributes both of its edges; duplicates (from the neighboring triangle) are removed per row.
    csr raw;
    raw.offsets.assign( size_t( n ) + 1, 0 );
    for( size_t i = 0; i < size_t( t ) * 3; ++i )
        raw.offsets[idx[i] + 1] += 2;
    for( uint32_t v = 0; v < n; ++v )
        raw.offsets[v + 1] += raw.offsets[v];
    raw.data.resize( size_t( t ) * 6 );
    std::vector< uint32_t > fill( raw.offsets.begin(), raw.offsets.end() - 1 );
    for( uint32_t f = 0; f < t; ++f )
    {
        const uint32_t * tri = idx + size_t( f ) * 3;
        for( int k = 0; k < 3; ++k )
        {
            raw.data[fill[tri[k]]++] = tri[( k + 1 ) % 3];
            raw.data[fill[tri[k]]++] = tri[( k + 2 ) % 3];
        }
    }
    std::vector< uint32_t > unique_count( n );
    parallel_for( n, 4096, [&]( size_t b, size_t e ) {
        for( size_t v = b; v < e; ++v )
        {
            auto first = raw.data.begin() + raw.offsets[v], last = raw.data.begin() + raw.offsets[v + 1];
            std::sort( first, last );
            unique_count[v] = uint32_t( std::unique( first, last ) - first );
        }
    } );
    csr c;
    c.offsets.assign( size_t( n ) + 1, 0 );
    for( uint32_t v = 0; v < n; ++v )
        c.offsets[v + 1] = c.offsets[v] + unique_count[v];
    c.data.resize( c.offsets[n] );
    for( uint32_t v = 0; v < n; ++v )
        std::copy_n( raw.data.begin() + raw.offsets[v], unique_count[v], c.data.begin() + c.offsets[v] );
    return c;
}

double mean_edge_length( const float * p, const uint32_t * idx, uint32_t t )
{
    if( t == 0 )
        return 0;
    double sum = 0;
    for( uint32_t f = 0; f < t; ++f )
    {
        const uint32_t * tri = idx + size_t( f ) * 3;
        for( int k = 0; k < 3; ++k )
            sum += length( vec3::load( p + size_t( tri[k] ) * 3 ) - vec3::load( p + size_t( tri[( k + 1 ) % 3] ) * 3 ) );
    }
    return sum / ( 3.0 * t );
}

void compute_vertex_normals( const float * p, uint32_t n, const uint32_t * idx, uint32_t t, float * normals )
{
    // The cross product's length is twice the triangle area, which gives the area weighting for free.
    std::vector< vec3 > acc( n );
    for( uint32_t f = 0; f < t; ++f )
    {
        const uint32_t * tri = idx + size_t( f ) * 3;
        const vec3 a = vec3::load( p + size_t( tri[0] ) * 3 ), b = vec3::load( p + size_t( tri[1] ) * 3 ),
                   c = vec3::load( p + size_t( tri[2] ) * 3 );
        const vec3 fn = cross( b - a, c - a );
        if( ! finite( fn ) )
            continue;
        acc[tri[0]] += fn;
        acc[tri[1]] += fn;
        acc[tri[2]] += fn;
    }
    parallel_for( n, 16384, [&]( size_t b, size_t e ) {
        for( size_t v = b; v < e; ++v )
            normalized( acc[v] ).store( normals + v * 3 );
    } );
}

}  // namespace mesh
}  // namespace fr
