// License: Apache 2.0. See LICENSE file in root directory.
//
// Wavefront OBJ (+MTL +PNG) export. Vertices carry all attributes (UVs are per vertex), so faces use the same index
// for v/vt/vn. Numbers are formatted in parallel chunks with std::to_chars (shortest round-trip floats).

#include "fr_export.h"
#include "export_internal.h"

#include <memory>
#include <sstream>

using namespace fr::mesh;
using namespace fr::io;

namespace
{

constexpr size_t max_line = 192;

bool write_mtl( const std::string & path, const std::string & material, const std::string & texture_file )
{
    std::ostringstream s;
    s << "# IntelFaceScan material\n"
      << "newmtl " << material << "\n"
      << "Ka 0.000000 0.000000 0.000000\n"  // no ambient term (some importers map Ka to metallic)
      << "Kd 1.000000 1.000000 1.000000\n"  // the texture is the albedo
      << "Ks 0.000000 0.000000 0.000000\n"  // scanned skin: no specular baked in, none added
      << "d 1.000000\n"
      << "illum 1\n"
      << "map_Kd " << texture_file << "\n";
    atomic_file file( path );
    return file.is_open() && file.write( s.str() ) && file.commit();
}

fr_status write_texture_png( const fr_texture & texture, const std::string & path )
{
    // RGB only: the atlas is opaque, and an alpha channel makes some viewers (SceneKit, QuickLook) blend it.
    const size_t count = size_t( texture.width ) * size_t( texture.height );
    std::vector< uint8_t > rgb( count * 3 );
    parallel_for( count, 1 << 16, [&]( size_t b, size_t e ) {
        for( size_t i = b; i < e; ++i )
        {
            rgb[i * 3] = texture.rgba[i * 4];
            rgb[i * 3 + 1] = texture.rgba[i * 4 + 1];
            rgb[i * 3 + 2] = texture.rgba[i * 4 + 2];
        }
    } );
    return fr_write_png( path.c_str(), rgb.data(), texture.width, texture.height, 3 );
}

}  // namespace

extern "C" {

fr_status fr_export_obj( const fr_mesh * mesh, const fr_texture * texture, const char * obj_path,
                         const fr_export_options * options )
{
    return fr::guarded( "fr_export_obj", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_export_obj" );
        if( s != FR_OK )
            return s;
        if( ! obj_path || ! *obj_path || ! options )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_export_obj: NULL path or options" );
        export_frame frame;
        if( ( s = make_export_frame( *options, frame, "fr_export_obj" ) ) != FR_OK )
            return s;
        const bool uvs = options->write_uvs && mesh->uvs;
        const bool normals = options->write_normals && mesh->normals;
        const bool colors = options->write_colors && mesh->colors;
        const bool textured = texture && options->write_uvs;
        if( textured && ! mesh->uvs )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_export_obj: a texture needs a mesh with UVs" );
        if( textured && ( texture->width <= 0 || texture->height <= 0 || ! texture->rgba ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_export_obj: invalid texture" );
        for( size_t i = 0; i < size_t( mesh->vertex_count ) * 3; ++i )
            if( ! std::isfinite( mesh->positions[i] ) )
                return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_export_obj: non-finite vertex position" );

        const std::string path( obj_path ), dir = directory_of( path ), stem = stem_of( path );
        const std::string file_stem = safe_name( stem );
        const std::string material = safe_name( options->material_name && *options->material_name ? options->material_name
                                                                                                  : "material0" );
        const std::string mtl_name = file_stem + ".mtl", png_name = file_stem + ".png";

        // Texture and material first: the OBJ is the last file to appear, once its references exist.
        if( textured )
        {
            if( ( s = write_texture_png( *texture, dir + png_name ) ) != FR_OK )
                return s;
            if( ! write_mtl( dir + mtl_name, material, png_name ) )
                return fr::fail( FR_ERR_IO, "fr_export_obj: cannot write " + dir + mtl_name );
        }

        atomic_file file( path );
        if( ! file.is_open() )
            return fr::fail( FR_ERR_IO, "fr_export_obj: cannot create " + path );
        std::ostringstream header;
        header << "# IntelFaceScan OBJ\n";
        if( options->comment && *options->comment )
        {
            std::istringstream lines( options->comment );
            for( std::string line; std::getline( lines, line ); )
                header << "# " << line << "\n";
        }
        header << "# " << units_comment( frame.scale ) << "\n"
               << "# vertices: " << mesh->vertex_count << ", triangles: " << mesh->triangle_count << "\n";
        if( textured )
            header << "mtllib " << mtl_name << "\n";
        header << "o " << file_stem << "\n";
        if( ! file.write( header.str() ) )
            return fr::fail( FR_ERR_IO, "fr_export_obj: write failed: " + path );

        const fr_mesh & m = *mesh;
        bool ok = write_formatted( file, m.vertex_count, max_line, [&]( size_t v, char * p ) {
            const vec3 x = frame.transform.apply( vec3::load( m.positions + v * 3 ) ) * frame.scale;
            p = put_str( p, "v " );
            p = put_float( p, x.x );
            *p++ = ' ';
            p = put_float( p, x.y );
            *p++ = ' ';
            p = put_float( p, x.z );
            if( colors )
                for( int k = 0; k < 3; ++k )
                {
                    *p++ = ' ';
                    p = put_fixed( p, m.colors[v * 3 + k] / 255.f, 4 );
                }
            *p++ = '\n';
            return p;
        } );
        if( ok && uvs )
            ok = write_formatted( file, m.vertex_count, max_line, [&]( size_t v, char * p ) {
                p = put_str( p, "vt " );
                p = put_fixed( p, m.uvs[v * 2], 6 );
                *p++ = ' ';
                p = put_fixed( p, m.uvs[v * 2 + 1], 6 );
                *p++ = '\n';
                return p;
            } );
        if( ok && normals )
            ok = write_formatted( file, m.vertex_count, max_line, [&]( size_t v, char * p ) {
                const vec3 n = normalized( frame.normal_matrix.rotate( vec3::load( m.normals + v * 3 ) ) );
                p = put_str( p, "vn " );
                p = put_fixed( p, n.x, 6 );
                *p++ = ' ';
                p = put_fixed( p, n.y, 6 );
                *p++ = ' ';
                p = put_fixed( p, n.z, 6 );
                *p++ = '\n';
                return p;
            } );
        if( ok && textured )
            ok = file.write( "usemtl " + material + "\n" );
        if( ok )
            ok = write_formatted( file, m.triangle_count, max_line, [&]( size_t f, char * p ) {
                uint32_t tri[3] = { m.indices[f * 3], m.indices[f * 3 + 1], m.indices[f * 3 + 2] };
                if( frame.mirrored )
                    std::swap( tri[1], tri[2] );
                *p++ = 'f';
                for( uint32_t i : tri )
                {
                    *p++ = ' ';
                    p = put_uint( p, i + 1 );
                    if( uvs || normals )
                    {
                        *p++ = '/';
                        if( uvs )
                            p = put_uint( p, i + 1 );
                        if( normals )
                        {
                            *p++ = '/';
                            p = put_uint( p, i + 1 );
                        }
                    }
                }
                *p++ = '\n';
                return p;
            } );
        if( ! ok || ! file.commit() )
            return fr::fail( FR_ERR_IO, "fr_export_obj: write failed: " + path );
        return FR_OK;
    } );
}

}  // extern "C"
