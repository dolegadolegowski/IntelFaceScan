// License: Apache 2.0. See LICENSE file in root directory.
//
// Shared export helpers (see export_internal.h) and the default export options.

#include "export_internal.h"

#include <cerrno>
#include <cstring>
#include <unistd.h>

namespace fr
{
namespace io
{

atomic_file::atomic_file( std::string path )
    : path_( std::move( path ) )
    , temp_( path_ + ".partial" )
{
    file_ = std::fopen( temp_.c_str(), "wb" );
    if( file_ )
        std::setvbuf( file_, nullptr, _IOFBF, 1 << 20 );
}

atomic_file::~atomic_file()
{
    if( file_ )
    {
        std::fclose( file_ );
        ::unlink( temp_.c_str() );
    }
}

bool atomic_file::write( const void * data, size_t size )
{
    if( ! file_ || ! ok_ )
        return false;
    if( size && std::fwrite( data, 1, size, file_ ) != size )
        ok_ = false;
    return ok_;
}

bool atomic_file::commit()
{
    if( ! file_ )
        return false;
    const bool flushed = std::fflush( file_ ) == 0;
    const bool closed = std::fclose( file_ ) == 0;
    file_ = nullptr;
    if( ! ok_ || ! flushed || ! closed || std::rename( temp_.c_str(), path_.c_str() ) != 0 )
    {
        ::unlink( temp_.c_str() );
        return false;
    }
    return true;
}

std::string directory_of( const std::string & path )
{
    const size_t slash = path.find_last_of( '/' );
    return slash == std::string::npos ? std::string() : path.substr( 0, slash + 1 );
}

std::string stem_of( const std::string & path )
{
    const size_t slash = path.find_last_of( '/' );
    std::string name = slash == std::string::npos ? path : path.substr( slash + 1 );
    const size_t dot = name.find_last_of( '.' );
    if( dot != std::string::npos && dot > 0 )
        name.resize( dot );
    return name;
}

std::string safe_name( const std::string & name )
{
    std::string out;
    out.reserve( name.size() );
    for( unsigned char c : name )
        out.push_back( ( c >= 'a' && c <= 'z' ) || ( c >= 'A' && c <= 'Z' ) || ( c >= '0' && c <= '9' ) || c == '.' || c == '_'
                               || c == '-'
                           ? char( c )
                           : '_' );
    return out.empty() ? std::string( "model" ) : out;
}

fr_status make_export_frame( const fr_export_options & o, export_frame & out, const char * where )
{
    if( ! std::isfinite( o.scale ) || ! ( o.scale > 0 ) )
        return fail( FR_ERR_INVALID_ARGUMENT, std::string( where ) + ": scale must be positive" );
    out.scale = o.scale;
    out.transform = o.has_transform ? mesh::affine::from_column_major( o.transform ) : mesh::affine();
    if( ! out.transform.is_finite() || ! out.transform.normal_matrix( out.normal_matrix ) )
        return fail( FR_ERR_INVALID_ARGUMENT, std::string( where ) + ": singular or non-finite transform" );
    out.mirrored = out.transform.det() < 0;
    return FR_OK;
}

std::string units_comment( float scale )
{
    if( scale == 1000.f )
        return "units: millimeters";
    if( scale == 100.f )
        return "units: centimeters";
    if( scale == 1.f )
        return "units: meters";
    char buf[64];
    std::snprintf( buf, sizeof( buf ), "units: 1 unit = %g m", 1.0 / double( scale ) );
    return buf;
}

}  // namespace io
}  // namespace fr

extern "C" {

void fr_export_options_default( fr_export_options * o )
{
    if( ! o )
        return;
    std::memset( o, 0, sizeof( *o ) );
    o->scale = 1000.f;
    for( int i = 0; i < 4; ++i )
        o->transform[i * 5] = 1.f;
    o->write_normals = 1;
    o->write_colors = 1;
    o->write_uvs = 1;
}

}  // extern "C"
