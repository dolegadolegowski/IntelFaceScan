// License: Apache 2.0. See LICENSE file in root directory.
//
// Wavefront OBJ import (v / vt / vn / f, polygons fanned into triangles, optional "v x y z r g b" colors). Corners
// with distinct (v, vt, vn) combinations become distinct vertices; files whose faces use one index for all three
// (like ours) keep their vertex numbering.

#include "fr_export.h"
#include "export_internal.h"

#include <string_view>
#include <unordered_map>

using namespace fr::mesh;

namespace
{

struct cursor
{
    const char * p;
    const char * end;

    void skip_space()
    {
        while( p < end && ( *p == ' ' || *p == '\t' || *p == '\r' ) )
            ++p;
    }
    bool at_line_end() const { return p >= end || *p == '\n' || *p == '#'; }
    void next_line()
    {
        while( p < end && *p != '\n' )
            ++p;
        if( p < end )
            ++p;
    }
    bool number( float & out )
    {
        skip_space();
        if( p < end && *p == '+' )
            ++p;
        auto r = std::from_chars( p, end, out );
        if( r.ec != std::errc() )
            return false;
        p = r.ptr;
        return true;
    }
    bool integer( long & out )
    {
        if( p < end && *p == '+' )
            ++p;
        auto r = std::from_chars( p, end, out );
        if( r.ec != std::errc() )
            return false;
        p = r.ptr;
        return true;
    }
    // The statement keyword ("v", "vt", "f", ...).
    std::string_view keyword()
    {
        skip_space();
        const char * start = p;
        while( p < end && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n' )
            ++p;
        return std::string_view( start, size_t( p - start ) );
    }
};

struct corner
{
    long v, vt, vn;  // 0-based, -1 = absent
    bool operator==( const corner & o ) const { return v == o.v && vt == o.vt && vn == o.vn; }
};

struct corner_hash
{
    size_t operator()( const corner & c ) const
    {
        uint64_t h = uint64_t( c.v ) * 0x9E3779B97F4A7C15ull;
        h ^= uint64_t( c.vt + 1 ) * 0xC2B2AE3D27D4EB4Full + ( h << 6 ) + ( h >> 2 );
        h ^= uint64_t( c.vn + 1 ) * 0x165667B19E3779F9ull + ( h << 6 ) + ( h >> 2 );
        return size_t( h );
    }
};

}  // namespace

extern "C" {

fr_status fr_import_obj( const char * obj_path, fr_mesh ** out )
{
    if( out )
        *out = nullptr;
    return fr::guarded( "fr_import_obj", [&]() -> fr_status {
        if( ! obj_path || ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_import_obj: NULL argument" );
        FILE * f = std::fopen( obj_path, "rb" );
        if( ! f )
            return fr::fail( FR_ERR_IO, std::string( "fr_import_obj: cannot open " ) + obj_path );
        std::string text;
        {
            char buf[1 << 16];
            size_t n;
            while( ( n = std::fread( buf, 1, sizeof( buf ), f ) ) > 0 )
                text.append( buf, n );
            const bool error = std::ferror( f );
            std::fclose( f );
            if( error )
                return fr::fail( FR_ERR_IO, std::string( "fr_import_obj: read error: " ) + obj_path );
        }

        std::vector< float > v, vt, vn, vc;
        std::vector< corner > corners;  // 3 per triangle
        bool any_color = false;
        float max_color = 0;
        cursor c{ text.data(), text.data() + text.size() };
        size_t line = 0;
        std::vector< corner > poly;
        auto malformed = [&]() {
            return fr::fail( FR_ERR_INVALID_ARGUMENT,
                             std::string( "fr_import_obj: malformed line " ) + std::to_string( line ) + " in " + obj_path );
        };
        while( c.p < c.end )
        {
            ++line;
            const std::string_view kw = c.keyword();
            if( kw == "v" )
            {
                float x[7];
                int count = 0;
                while( count < 7 && ! ( c.skip_space(), c.at_line_end() ) && c.number( x[count] ) )
                    ++count;
                if( count < 3 )
                    return malformed();
                v.insert( v.end(), x, x + 3 );
                if( count >= 6 )
                {
                    vc.insert( vc.end(), x + 3, x + 6 );
                    any_color = true;
                    max_color = std::max( { max_color, x[3], x[4], x[5] } );
                }
                else
                    vc.insert( vc.end(), { 1.f, 1.f, 1.f } );
            }
            else if( kw == "vt" )
            {
                float x[2];
                if( ! c.number( x[0] ) )
                    return malformed();
                if( ! c.number( x[1] ) )
                    x[1] = 0;
                vt.insert( vt.end(), x, x + 2 );
            }
            else if( kw == "vn" )
            {
                float x[3];
                if( ! c.number( x[0] ) || ! c.number( x[1] ) || ! c.number( x[2] ) )
                    return malformed();
                vn.insert( vn.end(), x, x + 3 );
            }
            else if( kw == "f" )
            {
                poly.clear();
                for( ;; )
                {
                    c.skip_space();
                    if( c.at_line_end() )
                        break;
                    long idx[3] = { 0, 0, 0 };
                    if( ! c.integer( idx[0] ) )
                        return malformed();
                    for( int k = 1; k < 3 && c.p < c.end && *c.p == '/'; ++k )
                    {
                        ++c.p;
                        if( c.p < c.end && *c.p != '/' && *c.p != ' ' && *c.p != '\t' && *c.p != '\r' && *c.p != '\n'
                            && ! c.integer( idx[k] ) )
                            return malformed();
                    }
                    // 1-based; negative values count back from the latest element.
                    auto resolve = [&]( long i, size_t count ) -> long {
                        if( i > 0 )
                            return i - 1 < long( count ) ? i - 1 : -2;
                        if( i < 0 )
                            return long( count ) + i >= 0 ? long( count ) + i : -2;
                        return -1;
                    };
                    const corner cr{ resolve( idx[0], v.size() / 3 ), resolve( idx[1], vt.size() / 2 ),
                                     resolve( idx[2], vn.size() / 3 ) };
                    if( cr.v < 0 || cr.vt == -2 || cr.vn == -2 )
                        return malformed();
                    poly.push_back( cr );
                }
                if( poly.size() < 3 )
                    return malformed();
                for( size_t k = 1; k + 1 < poly.size(); ++k )
                    corners.insert( corners.end(), { poly[0], poly[k], poly[k + 1] } );
            }
            c.next_line();
        }

        bool has_vt = false, has_vn = false, identity = true;
        for( const corner & cr : corners )
        {
            has_vt |= cr.vt >= 0;
            has_vn |= cr.vn >= 0;
            identity &= ( cr.vt < 0 || cr.vt == cr.v ) && ( cr.vn < 0 || cr.vn == cr.v );
        }
        const size_t nv = v.size() / 3;
        if( identity && ( ( has_vt && vt.size() / 2 < nv ) || ( has_vn && vn.size() / 3 < nv ) ) )
            identity = false;

        mesh_buffers b;
        b.has_uvs = has_vt;
        b.has_normals = has_vn;
        b.has_colors = any_color;
        const float color_scale = max_color > 1.f ? 1.f : 255.f;
        auto add_vertex = [&]( const corner & cr ) {
            const size_t i = size_t( cr.v );
            b.positions.insert( b.positions.end(), &v[i * 3], &v[i * 3] + 3 );
            if( b.has_colors )
                for( int k = 0; k < 3; ++k )
                    b.colors.push_back( uint8_t( std::clamp( std::lround( vc[i * 3 + k] * color_scale ), 0L, 255L ) ) );
            if( b.has_uvs )
            {
                const float * t = cr.vt >= 0 ? &vt[size_t( cr.vt ) * 2] : nullptr;
                b.uvs.push_back( t ? t[0] : 0.f );
                b.uvs.push_back( t ? t[1] : 0.f );
            }
            if( b.has_normals )
            {
                const vec3 n = cr.vn >= 0 ? vec3::load( &vn[size_t( cr.vn ) * 3] ) : vec3( 0, 0, 1 );
                b.normals.insert( b.normals.end(), { n.x, n.y, n.z } );
            }
        };
        b.indices.reserve( corners.size() );
        if( identity )
        {
            b.reserve_vertices( nv );
            for( size_t i = 0; i < nv; ++i )
                add_vertex( corner{ long( i ), has_vt ? long( i ) : -1, has_vn ? long( i ) : -1 } );
            for( const corner & cr : corners )
                b.indices.push_back( uint32_t( cr.v ) );
        }
        else
        {
            std::unordered_map< corner, uint32_t, corner_hash > unique;
            unique.reserve( corners.size() );
            for( const corner & cr : corners )
            {
                auto [it, inserted] = unique.try_emplace( cr, b.vertex_count() );
                if( inserted )
                    add_vertex( cr );
                b.indices.push_back( it->second );
            }
        }
        if( b.has_normals )
            for( size_t i = 0; i < b.normals.size(); i += 3 )
                normalized( vec3::load( &b.normals[i] ) ).store( &b.normals[i] );

        fr_mesh * mesh = nullptr;
        const uint32_t flags = ( b.has_normals ? FR_MESH_NORMALS : 0u ) | ( b.has_colors ? FR_MESH_COLORS : 0u )
                             | ( b.has_uvs ? FR_MESH_UVS : 0u );
        if( b.positions.size() / 3 > UINT32_MAX || b.indices.size() / 3 > UINT32_MAX )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_import_obj: mesh too large" );
        fr_status s = fr_mesh_create( 0, 0, flags, &mesh );
        if( s != FR_OK )
            return s;
        s = store( b, mesh );
        if( s != FR_OK )
        {
            fr_mesh_free( mesh );
            return s;
        }
        *out = mesh;
        return FR_OK;
    } );
}

}  // extern "C"
