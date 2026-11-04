// License: Apache 2.0. See LICENSE file in root directory.

#include "security_recording.h"

#include <sys/param.h>
#include <sys/stat.h>

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <stdexcept>
#include <unistd.h>
#include <vector>

namespace ifs
{

namespace
{
[[noreturn]] void fail( const std::string & message )
{
    throw std::runtime_error( message );
}

std::string errno_text()
{
    return std::strerror( errno );
}

// A directory nobody but the effective user can use. Checked through a descriptor opened without following a
// link at the last component.
bool private_directory( const std::string & path )
{
    UniqueFd fd( ::open( path.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC ) );
    if( ! fd.valid() )
        return false;
    struct stat st;
    return ::fstat( fd.get(), &st ) == 0 && S_ISDIR( st.st_mode ) && st.st_uid == ::geteuid()
        && ( st.st_mode & 077 ) == 0;
}

std::string private_base()
{
    char temp[PATH_MAX];
    size_t n = ::confstr( _CS_DARWIN_USER_TEMP_DIR, temp, sizeof( temp ) );
    if( n > 0 && n <= sizeof( temp ) )
    {
        char real[PATH_MAX];
        if( ::realpath( temp, real ) && private_directory( real ) )
            return real;
    }
    if( ::geteuid() == 0 )
    {
        // Only root can create entries in /private/var/run, so nobody can have prepared this path.
        const std::string fallback = "/private/var/run/IntelFaceScanHelper";
        if( ::mkdir( fallback.c_str(), 0700 ) != 0 && errno != EEXIST )
            fail( "cannot create " + fallback + ": " + errno_text() );
        if( private_directory( fallback ) )
            return fallback;
    }
    fail( "no private temporary directory for recordings" );
}

void remove_tree( int parent, const std::string & name, int depth )
{
    if( ::unlinkat( parent, name.c_str(), 0 ) == 0 || errno == ENOENT )
        return;
    if( depth > 0 )
    {
        int fd = ::openat( parent, name.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC );
        if( fd >= 0 )
        {
            DIR * d = ::fdopendir( fd );
            if( ! d )
                ::close( fd );
            else
            {
                std::vector< std::string > entries;
                while( dirent * entry = ::readdir( d ) )
                    if( std::strcmp( entry->d_name, "." ) != 0 && std::strcmp( entry->d_name, ".." ) != 0 )
                        entries.emplace_back( entry->d_name );
                for( const std::string & entry : entries )
                    remove_tree( ::dirfd( d ), entry, depth - 1 );
                ::closedir( d );
            }
        }
    }
    ::unlinkat( parent, name.c_str(), AT_REMOVEDIR );
}

bool regular_file( const std::string & path )
{
    struct stat st;
    return ::lstat( path.c_str(), &st ) == 0 && S_ISREG( st.st_mode );
}

bool has_suffix( const std::string & s, const std::string & suffix )
{
    return s.size() >= suffix.size() && s.compare( s.size() - suffix.size(), suffix.size(), suffix ) == 0;
}

// The engine is asked to write file_path(), but recorders may add a folder level or a suffix; accept the one
// regular file with the format's extension the spool directory then contains (it is private, so whatever is in it
// was written by the helper).
std::string find_recording( const std::string & dir, const std::string & expected )
{
    if( regular_file( expected ) )
        return expected;
    const std::string ext = expected.substr( expected.rfind( '.' ) );
    std::vector< std::string > found;
    std::vector< std::pair< std::string, int > > pending{ { dir, 0 } };
    while( ! pending.empty() )
    {
        auto [path, depth] = pending.back();
        pending.pop_back();
        DIR * d = ::opendir( path.c_str() );
        if( ! d )
            continue;
        while( dirent * entry = ::readdir( d ) )
        {
            if( ! std::strcmp( entry->d_name, "." ) || ! std::strcmp( entry->d_name, ".." ) )
                continue;
            const std::string child = path + "/" + entry->d_name;
            struct stat st;
            if( ::lstat( child.c_str(), &st ) != 0 )
                continue;
            if( S_ISREG( st.st_mode ) && has_suffix( child, ext ) )
                found.push_back( child );
            else if( S_ISDIR( st.st_mode ) && depth < 2 )
                pending.emplace_back( child, depth + 1 );
        }
        ::closedir( d );
    }
    if( found.size() != 1 )
        fail( found.empty() ? "the recording produced no file" : "the recording produced more than one file" );
    return found.front();
}
}  // namespace

std::string check_recording_destination( int fd )
{
    struct stat st;
    if( ::fstat( fd, &st ) != 0 )
        return "cannot inspect the recording destination: " + errno_text();
    if( ! S_ISREG( st.st_mode ) )
        return "the recording destination is not a regular file";
    // The kernel clears these bits when anyone but root writes to a file; root writing through the app's descriptor
    // would keep them (e.g. on a set-id file the app's user may write to but should not be able to fill as root).
    if( st.st_mode & ( S_ISUID | S_ISGID ) )
        return "the recording destination has the set-user-ID or set-group-ID bit";
    int flags = ::fcntl( fd, F_GETFL );
    if( flags < 0 )
        return "cannot inspect the recording destination: " + errno_text();
    if( ( flags & O_ACCMODE ) != O_WRONLY && ( flags & O_ACCMODE ) != O_RDWR )
        return "the recording destination is not open for writing";
    return {};
}

std::unique_ptr< RecordingSpool > RecordingSpool::create( const std::string & format )
{
    if( format != "db3" && format != "bag" )
        fail( "unsupported recording format '" + format + "'" );
    std::string templ = private_base() + "/ifs-recording.XXXXXX";
    std::vector< char > buf( templ.begin(), templ.end() );
    buf.push_back( '\0' );
    if( ! ::mkdtemp( buf.data() ) )
        fail( "cannot create the recording directory: " + errno_text() );
    std::string dir( buf.data() );
    return std::unique_ptr< RecordingSpool >( new RecordingSpool( dir, dir + "/recording." + format ) );
}

RecordingSpool::~RecordingSpool()
{
    remove();
}

uint64_t RecordingSpool::copy_into( int destination ) const
{
    const std::string path = find_recording( dir_, file_ );
    UniqueFd src( ::open( path.c_str(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC ) );
    if( ! src.valid() )
        fail( "cannot open the finished recording: " + errno_text() );
    struct stat st;
    if( ::fstat( src.get(), &st ) != 0 || ! S_ISREG( st.st_mode ) )
        fail( "the finished recording is not a regular file" );

    std::vector< char > buf( 1 << 20 );
    uint64_t total = 0;
    for( ;; )
    {
        ssize_t n = ::read( src.get(), buf.data(), buf.size() );
        if( n < 0 )
        {
            if( errno == EINTR )
                continue;
            fail( "reading the recording failed: " + errno_text() );
        }
        if( n == 0 )
            break;
        size_t written = 0;
        while( written < size_t( n ) )
        {
            ssize_t w = ::write( destination, buf.data() + written, size_t( n ) - written );
            if( w < 0 )
            {
                if( errno == EINTR )
                    continue;
                fail( "writing the recording failed: " + errno_text() );
            }
            written += size_t( w );
        }
        total += uint64_t( n );
    }
    // Some file systems (network shares, FUSE) cannot fsync; the data has still been handed to them.
    if( ::fsync( destination ) != 0 && errno != EINVAL && errno != ENOTSUP )
        fail( "flushing the recording failed: " + errno_text() );
    return total;
}

void RecordingSpool::remove()
{
    if( removed_ )
        return;
    removed_ = true;
    remove_tree( AT_FDCWD, dir_, 4 );
}

}  // namespace ifs
