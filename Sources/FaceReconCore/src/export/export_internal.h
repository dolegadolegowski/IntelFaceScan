// License: Apache 2.0. See LICENSE file in root directory.
//
// Internal helpers of the exporters: atomic file output, parallel chunked formatting, file names, the export
// transform.

#pragma once

#include "fr_export.h"
#include "../mesh/mesh_internal.h"

#include <charconv>
#include <cstdio>
#include <string>

namespace fr
{
namespace io
{

// Writes to "<path>.partial" and renames it over <path> on commit(), so a failed export never leaves a truncated
// file behind (the partial file is removed).
class atomic_file
{
public:
    explicit atomic_file( std::string path );
    ~atomic_file();
    atomic_file( const atomic_file & ) = delete;
    atomic_file & operator=( const atomic_file & ) = delete;

    bool is_open() const { return file_ != nullptr; }
    bool write( const void * data, size_t size );
    bool write( const std::string & s ) { return write( s.data(), s.size() ); }
    // Flushes, closes and renames; false (with the partial file removed) on any error.
    bool commit();
    const std::string & path() const { return path_; }

private:
    std::string path_, temp_;
    FILE * file_ = nullptr;
    bool ok_ = true;
};

// Formats items [0, count) into the file in parallel: chunks of items are formatted into separate buffers by
// format(i, out) -> end (at most max_bytes per item), then written in order. Memory stays bounded by processing a
// limited number of chunks at a time.
template< typename F >
bool write_formatted( atomic_file & file, size_t count, size_t max_bytes, F && format )
{
    constexpr size_t chunk_items = 16384;
    constexpr size_t chunks_per_batch = 64;
    const size_t chunks = ( count + chunk_items - 1 ) / chunk_items;
    std::vector< std::vector< char > > buffers( std::min( chunks, chunks_per_batch ) );
    std::vector< size_t > used( buffers.size() );
    for( size_t first = 0; first < chunks; first += chunks_per_batch )
    {
        const size_t batch = std::min( chunks_per_batch, chunks - first );
        mesh::parallel_for( batch, 1, [&]( size_t b, size_t e ) {
            for( size_t c = b; c < e; ++c )
            {
                const size_t begin = ( first + c ) * chunk_items, end = std::min( count, begin + chunk_items );
                std::vector< char > & buf = buffers[c];
                buf.resize( ( end - begin ) * max_bytes );
                char * p = buf.data();
                for( size_t i = begin; i < end; ++i )
                    p = format( i, p );
                used[c] = size_t( p - buf.data() );
            }
        } );
        for( size_t c = 0; c < batch; ++c )
            if( ! file.write( buffers[c].data(), used[c] ) )
                return false;
    }
    return true;
}

// Shortest decimal representation that reads back to the same float.
inline char * put_float( char * p, float v )
{
    if( ! std::isfinite( v ) )
        v = 0;
    return std::to_chars( p, p + 32, v ).ptr;
}
inline char * put_fixed( char * p, float v, int precision )
{
    if( ! std::isfinite( v ) )
        v = 0;
    return std::to_chars( p, p + 48, v, std::chars_format::fixed, precision ).ptr;
}
inline char * put_uint( char * p, uint32_t v ) { return std::to_chars( p, p + 16, v ).ptr; }
inline char * put_str( char * p, const char * s )
{
    while( *s )
        *p++ = *s++;
    return p;
}

// Directory part of a path including the trailing '/' ("" when there is none).
std::string directory_of( const std::string & path );
// File name without directory and extension.
std::string stem_of( const std::string & path );
// File-name-safe version of a name: characters other than A-Z a-z 0-9 . _ - become '_' (MTL statements and many
// OBJ readers split file names at spaces). Empty names become "model".
std::string safe_name( const std::string & name );

// The export transform: positions -> scale * (T * p), normals -> normalized N * n, mirrored when det(T) < 0.
struct export_frame
{
    mesh::affine transform;
    mesh::affine normal_matrix;
    float scale = 1;
    bool mirrored = false;
};

fr_status make_export_frame( const fr_export_options & options, export_frame & out, const char * where );

// "units: millimeters" etc. for the header comments.
std::string units_comment( float scale );

}  // namespace io
}  // namespace fr
