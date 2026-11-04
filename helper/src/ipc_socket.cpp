// License: Apache 2.0. See LICENSE file in root directory.

#include "ipc_socket.h"

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#include <cerrno>
#include <cstring>
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

std::vector< std::string > split_path( const std::string & path )
{
    std::vector< std::string > parts;
    size_t start = 1;
    while( start <= path.size() )
    {
        size_t end = path.find( '/', start );
        if( end == std::string::npos )
            end = path.size();
        parts.push_back( path.substr( start, end - start ) );
        start = end + 1;
    }
    return parts;
}
}  // namespace

UniqueFd connect_app_socket( const std::string & path, uid_t uid )
{
    if( path.empty() || path[0] != '/' )
        fail( "socket path must be absolute" );
    if( path.find( '\0' ) != std::string::npos )
        fail( "socket path contains a NUL byte" );

    std::vector< std::string > parts = split_path( path );
    if( parts.size() < 2 )
        fail( "socket path must name a file inside a directory" );
    for( const std::string & part : parts )
        if( part.empty() || part == "." || part == ".." )
            fail( "socket path must be canonical (no empty, '.' or '..' components)" );
    const std::string name = parts.back();
    parts.pop_back();
    sockaddr_un addr{};
    if( name.size() >= sizeof( addr.sun_path ) )
        fail( "socket name too long" );

    // Walk the directories without following links: a link anywhere could point the root helper elsewhere.
    UniqueFd dir( ::open( "/", O_RDONLY | O_DIRECTORY | O_CLOEXEC ) );
    if( ! dir.valid() )
        fail( "cannot open /: " + errno_text() );
    for( const std::string & part : parts )
    {
        UniqueFd next( ::openat( dir.get(), part.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC ) );
        if( ! next.valid() )
        {
            if( errno == ELOOP || errno == ENOTDIR )
                fail( "socket path component '" + part + "' is not a directory (symbolic links are not allowed)" );
            fail( "cannot open socket directory component '" + part + "': " + errno_text() );
        }
        dir = std::move( next );
    }

    struct stat st;
    if( ::fstat( dir.get(), &st ) != 0 )
        fail( "cannot stat the socket directory: " + errno_text() );
    if( ! S_ISDIR( st.st_mode ) )
        fail( "socket directory is not a directory" );
    if( st.st_uid != uid )
        fail( "socket directory is not owned by uid " + std::to_string( uid ) );
    if( ( st.st_mode & 07777 ) != 0700 )
        fail( "socket directory must have mode 0700" );

    if( ::fstatat( dir.get(), name.c_str(), &st, AT_SYMLINK_NOFOLLOW ) != 0 )
        fail( "socket does not exist: " + errno_text() );
    if( ! S_ISSOCK( st.st_mode ) )
        fail( "socket path is not a socket" );
    if( st.st_uid != uid )
        fail( "socket is not owned by uid " + std::to_string( uid ) );

    UniqueFd sock( ::socket( AF_UNIX, SOCK_STREAM, 0 ) );
    if( ! sock.valid() )
        fail( "socket() failed: " + errno_text() );
    configure_socket( sock.get() );

    // There is no connectat() in the macOS SDK: connect relative to the verified directory instead. This runs
    // before the helper starts any thread, so changing the working directory affects nobody else.
    addr.sun_family = AF_UNIX;
    std::memcpy( addr.sun_path, name.c_str(), name.size() + 1 );
    addr.sun_len = uint8_t( SUN_LEN( &addr ) );
    if( ::fchdir( dir.get() ) != 0 )
        fail( "cannot enter the socket directory: " + errno_text() );
    int rc;
    do
        rc = ::connect( sock.get(), reinterpret_cast< sockaddr * >( &addr ), socklen_t( addr.sun_len ) );
    while( rc != 0 && errno == EINTR );
    const int connect_errno = errno;
    if( ::chdir( "/" ) != 0 )
        fail( "cannot leave the socket directory: " + errno_text() );
    if( rc != 0 )
        fail( "connect() failed: " + std::string( std::strerror( connect_errno ) ) );
    return sock;
}

void verify_app_peer( int sock, uid_t uid, pid_t pid )
{
    uid_t peer_uid = 0;
    gid_t peer_gid = 0;
    if( ::getpeereid( sock, &peer_uid, &peer_gid ) != 0 )
        fail( "cannot read the peer credentials: " + errno_text() );
    if( peer_uid != uid )
        fail( "socket peer runs as uid " + std::to_string( peer_uid ) + ", expected " + std::to_string( uid ) );

    pid_t peer_pid = 0;
    socklen_t len = sizeof( peer_pid );
    if( ::getsockopt( sock, SOL_LOCAL, LOCAL_PEERPID, &peer_pid, &len ) != 0 )
        fail( "cannot read the peer pid: " + errno_text() );
    if( peer_pid != pid )
        fail( "socket peer is process " + std::to_string( peer_pid ) + ", expected " + std::to_string( pid ) );
}

}  // namespace ifs
