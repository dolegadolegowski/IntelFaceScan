// License: Apache 2.0. See LICENSE file in root directory.
//
// Status strings, per-thread error messages and allocation of the plain C mesh/texture structs.

#include "fr_types.h"
#include "fr_internal.h"

#include <cstdlib>
#include <cstring>
#include <string>
#include <type_traits>

namespace
{
thread_local std::string g_last_error;

template< typename T >
T * alloc_array( size_t count )
{
    if( count == 0 )
        return nullptr;
    return static_cast< T * >( std::malloc( count * sizeof( T ) ) );
}
}  // namespace

namespace fr
{
fr_status fail( fr_status status, const std::string & message )
{
    g_last_error = message;
    return status;
}

void clear_error()
{
    g_last_error.clear();
}
}  // namespace fr

extern "C" {

const char * fr_status_string( fr_status status )
{
    switch( status )
    {
    case FR_OK: return "ok";
    case FR_ERR_INVALID_ARGUMENT: return "invalid argument";
    case FR_ERR_NOT_IMPLEMENTED: return "not implemented";
    case FR_ERR_NO_GPU: return "no Metal GPU available";
    case FR_ERR_OUT_OF_MEMORY: return "out of memory";
    case FR_ERR_CANCELLED: return "cancelled";
    case FR_ERR_IO: return "I/O error";
    case FR_ERR_NOT_ENOUGH_DATA: return "not enough data";
    case FR_ERR_FAILED: return "failed";
    }
    return "unknown status";
}

const char * fr_last_error( void )
{
    return g_last_error.c_str();
}

fr_status fr_mesh_create( uint32_t vertex_count, uint32_t triangle_count, uint32_t attributes, fr_mesh ** out )
{
    if( ! out )
        return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_create: out is NULL" );
    *out = nullptr;
    auto * mesh = static_cast< fr_mesh * >( std::calloc( 1, sizeof( fr_mesh ) ) );
    if( ! mesh )
        return fr::fail( FR_ERR_OUT_OF_MEMORY, "fr_mesh_create" );
    mesh->vertex_count = vertex_count;
    mesh->triangle_count = triangle_count;
    mesh->positions = alloc_array< float >( size_t( vertex_count ) * 3 );
    mesh->indices = alloc_array< uint32_t >( size_t( triangle_count ) * 3 );
    if( ( vertex_count && ! mesh->positions ) || ( triangle_count && ! mesh->indices ) )
    {
        fr_mesh_free( mesh );
        return fr::fail( FR_ERR_OUT_OF_MEMORY, "fr_mesh_create" );
    }
    fr_status status = fr_mesh_set_attributes( mesh, attributes );
    if( status != FR_OK )
    {
        fr_mesh_free( mesh );
        return status;
    }
    *out = mesh;
    return FR_OK;
}

fr_status fr_mesh_set_attributes( fr_mesh * mesh, uint32_t attributes )
{
    if( ! mesh )
        return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_set_attributes: mesh is NULL" );
    const size_t n = mesh->vertex_count;
    auto update = [&]( auto *& array, bool wanted, size_t per_vertex ) -> bool {
        using T = std::remove_reference_t< decltype( *array ) >;
        if( wanted && ! array && n )
        {
            array = alloc_array< T >( n * per_vertex );
            return array != nullptr;
        }
        if( ! wanted && array )
        {
            std::free( array );
            array = nullptr;
        }
        return true;
    };
    if( ! update( mesh->normals, attributes & FR_MESH_NORMALS, 3 ) || ! update( mesh->colors, attributes & FR_MESH_COLORS, 3 )
        || ! update( mesh->uvs, attributes & FR_MESH_UVS, 2 ) )
        return fr::fail( FR_ERR_OUT_OF_MEMORY, "fr_mesh_set_attributes" );
    return FR_OK;
}

fr_status fr_mesh_clone( const fr_mesh * mesh, fr_mesh ** out )
{
    if( ! mesh || ! out )
        return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_clone: NULL argument" );
    uint32_t attributes = ( mesh->normals ? FR_MESH_NORMALS : 0 ) | ( mesh->colors ? FR_MESH_COLORS : 0 )
                        | ( mesh->uvs ? FR_MESH_UVS : 0 );
    fr_status status = fr_mesh_create( mesh->vertex_count, mesh->triangle_count, attributes, out );
    if( status != FR_OK )
        return status;
    fr_mesh * copy = *out;
    const size_t n = mesh->vertex_count, t = mesh->triangle_count;
    if( n )
        std::memcpy( copy->positions, mesh->positions, n * 3 * sizeof( float ) );
    if( t )
        std::memcpy( copy->indices, mesh->indices, t * 3 * sizeof( uint32_t ) );
    if( mesh->normals && n )
        std::memcpy( copy->normals, mesh->normals, n * 3 * sizeof( float ) );
    if( mesh->colors && n )
        std::memcpy( copy->colors, mesh->colors, n * 3 );
    if( mesh->uvs && n )
        std::memcpy( copy->uvs, mesh->uvs, n * 2 * sizeof( float ) );
    return FR_OK;
}

void fr_mesh_free( fr_mesh * mesh )
{
    if( ! mesh )
        return;
    std::free( mesh->positions );
    std::free( mesh->normals );
    std::free( mesh->colors );
    std::free( mesh->uvs );
    std::free( mesh->indices );
    std::free( mesh );
}

fr_status fr_texture_create( int32_t width, int32_t height, fr_texture ** out )
{
    if( ! out || width <= 0 || height <= 0 || width > 32768 || height > 32768 )
        return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_texture_create: invalid size" );
    auto * texture = static_cast< fr_texture * >( std::calloc( 1, sizeof( fr_texture ) ) );
    if( ! texture )
        return fr::fail( FR_ERR_OUT_OF_MEMORY, "fr_texture_create" );
    texture->width = width;
    texture->height = height;
    texture->rgba = static_cast< uint8_t * >( std::calloc( size_t( width ) * size_t( height ), 4 ) );
    if( ! texture->rgba )
    {
        std::free( texture );
        return fr::fail( FR_ERR_OUT_OF_MEMORY, "fr_texture_create" );
    }
    *out = texture;
    return FR_OK;
}

void fr_texture_free( fr_texture * texture )
{
    if( ! texture )
        return;
    std::free( texture->rgba );
    std::free( texture );
}

}  // extern "C"
