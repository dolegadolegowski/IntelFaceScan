// License: Apache 2.0. See LICENSE file in root directory.

#include "security_process.h"

#include <sys/resource.h>
#include <sys/stat.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <libproc.h>
#include <pwd.h>
#include <string>
#include <unistd.h>
#include <vector>

extern char ** environ;

namespace ifs
{

namespace
{
void ensure_standard_descriptors()
{
    // A closed 0/1/2 would be reused by the next open() and receive stray output (or input) meant for the terminal.
    int null_fd = ::open( "/dev/null", O_RDWR );
    if( null_fd < 0 )
        return;
    ::dup2( null_fd, STDIN_FILENO );
    for( int fd = STDOUT_FILENO; fd <= STDERR_FILENO; ++fd )
        if( ::fcntl( fd, F_GETFD ) < 0 )
            ::dup2( null_fd, fd );
    if( null_fd > STDERR_FILENO )
        ::close( null_fd );
}

void close_inherited_descriptors()
{
    // The kernel's list of this process's descriptors: a loop up to the soft RLIMIT_NOFILE alone misses descriptors
    // the parent opened before it lowered the limit (they stay open and usable above it).
    const int needed = ::proc_pidinfo( ::getpid(), PROC_PIDLISTFDS, 0, nullptr, 0 );
    if( needed > 0 )
    {
        std::vector< proc_fdinfo > fds( size_t( needed ) / sizeof( proc_fdinfo ) + 16 );
        const int got = ::proc_pidinfo( ::getpid(), PROC_PIDLISTFDS, 0, fds.data(),
                                        int( fds.size() * sizeof( proc_fdinfo ) ) );
        for( size_t i = 0, n = got > 0 ? size_t( got ) / sizeof( proc_fdinfo ) : 0; i < n; ++i )
            if( fds[i].proc_fd > STDERR_FILENO )
                ::close( fds[i].proc_fd );
    }
    // And the classic sweep, in case the list could not be read.
    rlimit rl{};
    int limit = 4096;
    if( ::getrlimit( RLIMIT_NOFILE, &rl ) == 0 && rl.rlim_cur != RLIM_INFINITY )
        limit = int( std::min< rlim_t >( rl.rlim_cur, 65536 ) );
    for( int fd = STDERR_FILENO + 1; fd < limit; ++fd )
        ::close( fd );
}

void reset_environment()
{
    std::vector< std::string > names;
    for( char ** e = environ; e && *e; ++e )
    {
        const char * eq = std::strchr( *e, '=' );
        names.emplace_back( *e, eq ? size_t( eq - *e ) : std::strlen( *e ) );
    }
    for( const std::string & name : names )
        ::unsetenv( name.c_str() );

    ::setenv( "PATH", "/usr/bin:/bin:/usr/sbin:/sbin", 1 );
    if( passwd * pw = ::getpwuid( ::geteuid() ) )
        if( pw->pw_dir && pw->pw_dir[0] == '/' )
            ::setenv( "HOME", pw->pw_dir, 1 );
}
}  // namespace

void harden_process()
{
    ::umask( 077 );
    ensure_standard_descriptors();
    close_inherited_descriptors();
    reset_environment();
}

}  // namespace ifs
