// License: Apache 2.0. See LICENSE file in root directory.
//
// Binary little-endian PLY export (positions, optional normals, uchar colors and per-vertex texture coordinates).

#include "fr_export.h"
#include "export_internal.h"

#include <sstream>

using namespace fr::mesh;
using namespace fr::io;

static_assert( __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "the PLY writer emits the host byte order" );

extern "C" {

fr_status fr_export_ply( const fr_mesh * mesh, const char * ply_path, const fr_export_options * options )
{
    return fr::guarded( "fr_export_ply", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_export_ply" );
        if( s != FR_OK )
            return s;
        if( ! ply_path || ! *ply_path || ! options )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_export_ply: NULL path or options" );
        export_frame frame;
        if( ( s = make_export_frame( *options, frame, "fr_export_ply" ) ) != FR_OK )
            return s;
        const bool normals = options->write_normals && mesh->normals;
        const bool colors = options->write_colors && mesh->colors;
        const bool uvs = options->write_uvs && mesh->uvs;

        std::ostringstream h;
        h << "ply\nformat binary_little_endian 1.0\ncomment IntelFaceScan\n";
        if( options->comment && *options->comment )
        {
            std::istringstream lines( options->comment );
            for( std::string line; std::getline( lines, line ); )
                h << "comment " << line << "\n";
        }
        h << "comment " << units_comment( frame.scale ) << "\n"
          << "element vertex " << mesh->vertex_count << "\n"
          << "property float x\nproperty float y\nproperty float z\n";
        if( normals )
            h << "property float nx\nproperty float ny\nproperty float nz\n";
        if( colors )
            h << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
        if( uvs )
            h << "property float texture_u\nproperty float texture_v\n";
        h << "element face " << mesh->triangle_count << "\n"
          << "property list uchar int vertex_indices\nend_header\n";

        atomic_file file( ply_path );
        if( ! file.is_open() )
            return fr::fail( FR_ERR_IO, std::string( "fr_export_ply: cannot create " ) + ply_path );
        bool ok = file.write( h.str() );
        const size_t vertex_bytes = 12 + ( normals ? 12 : 0 ) + ( colors ? 3 : 0 ) + ( uvs ? 8 : 0 );
        const fr_mesh & m = *mesh;
        if( ok )
            ok = write_formatted( file, m.vertex_count, vertex_bytes, [&]( size_t v, char * p ) {
                const vec3 x = frame.transform.apply( vec3::load( m.positions + v * 3 ) ) * frame.scale;
                std::memcpy( p, &x.x, 4 );
                std::memcpy( p + 4, &x.y, 4 );
                std::memcpy( p + 8, &x.z, 4 );
                p += 12;
                if( normals )
                {
                    const vec3 n = normalized( frame.normal_matrix.rotate( vec3::load( m.normals + v * 3 ) ) );
                    std::memcpy( p, &n.x, 4 );
                    std::memcpy( p + 4, &n.y, 4 );
                    std::memcpy( p + 8, &n.z, 4 );
                    p += 12;
                }
                if( colors )
                {
                    std::memcpy( p, m.colors + v * 3, 3 );
                    p += 3;
                }
                if( uvs )
                {
                    std::memcpy( p, m.uvs + v * 2, 8 );
                    p += 8;
                }
                return p;
            } );
        if( ok )
            ok = write_formatted( file, m.triangle_count, 13, [&]( size_t f, char * p ) {
                int32_t tri[3] = { int32_t( m.indices[f * 3] ), int32_t( m.indices[f * 3 + 1] ), int32_t( m.indices[f * 3 + 2] ) };
                if( frame.mirrored )
                    std::swap( tri[1], tri[2] );
                *p++ = 3;
                std::memcpy( p, tri, 12 );
                return p + 12;
            } );
        if( ! ok || ! file.commit() )
            return fr::fail( FR_ERR_IO, std::string( "fr_export_ply: write failed: " ) + ply_path );
        return FR_OK;
    } );
}

}  // extern "C"
