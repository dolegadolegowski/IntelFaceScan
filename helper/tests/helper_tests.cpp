// License: Apache 2.0. See LICENSE file in root directory.
//
// Tests of the helper outside the capture module, all as the current user and without a camera:
// * the recording spool (security_recording.h), which helpers of one user share since the live helper can run as the
//   user (the RealSense camera service path, docs/HELPER.md);
// * the helper process itself: its command line and exit status, and a whole live session as the user - the helper
//   takes the app's socket, says HELLO with mode "live" and the user's uid, reports why it has no camera, and leaves
//   on QUIT without touching USB.
//
// The process tests run the helper binary named by IFS_HELPER_UNDER_TEST (ctest sets it when this folder is built
// with the helper, helper/tests/CMakeLists.txt) and are skipped without it. Nothing here needs the camera service
// or root; a Mac that has a camera or the service passes as well.

#include "ifs_protocol.h"
#include "ipc_wire.h"
#include "security_recording.h"
#include "test_framework.h"

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <sys/wait.h>

#include <cerrno>
#include <climits>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

using nlohmann::json;
using namespace ifs;
using namespace ifs::test;

namespace
{

constexpr int kTimeoutMs = 30000;

bool exists( const std::string & path )
{
    struct stat st;
    return ::lstat( path.c_str(), &st ) == 0;
}

// Makes a file or folder look as if nothing touched it for 'seconds'.
void set_age( const std::string & path, time_t seconds )
{
    timeval tv[2];
    tv[0].tv_sec = tv[1].tv_sec = ::time( nullptr ) - seconds;
    tv[0].tv_usec = tv[1].tv_usec = 0;
    if( ::utimes( path.c_str(), tv ) != 0 )
        throw std::runtime_error( "utimes " + path + ": " + std::strerror( errno ) );
}

void write_file( const std::string & path, const std::string & content )
{
    UniqueFd fd( ::open( path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600 ) );
    if( ! fd.valid() || ::write( fd.get(), content.data(), content.size() ) != ssize_t( content.size() ) )
        throw std::runtime_error( "cannot write " + path );
}

std::string make_directory( const std::string & pattern )
{
    std::string path = pattern;
    if( ! ::mkdtemp( path.data() ) )
        throw std::runtime_error( "mkdtemp " + pattern + ": " + std::strerror( errno ) );
    return path;
}

std::string parent_of( const std::string & path )
{
    return path.substr( 0, path.rfind( '/' ) );
}

// --- The helper process --------------------------------------------------------------------------------------------

std::string helper_binary()
{
    const char * path = std::getenv( "IFS_HELPER_UNDER_TEST" );
    if( ! path || ! *path )
        SKIP( "IFS_HELPER_UNDER_TEST is not set (the IntelFaceScanHelper to run)" );
    if( ::access( path, X_OK ) != 0 )
        throw std::runtime_error( std::string( "IFS_HELPER_UNDER_TEST is not an executable: " ) + path );
    return path;
}

struct Child
{
    pid_t pid = -1;
    UniqueFd out, err;
};

struct Exit
{
    int status = -1;  // exit status; 128 + signal when a signal ended the helper; -1 when it had to be killed here
    std::string out, err;
};

Child spawn( const std::string & binary, const std::vector< std::string > & args )
{
    int out[2], err[2];
    if( ::pipe( out ) != 0 || ::pipe( err ) != 0 )
        throw std::runtime_error( "pipe failed" );
    std::vector< std::string > words{ binary };
    words.insert( words.end(), args.begin(), args.end() );
    std::vector< char * > argv;
    for( std::string & w : words )
        argv.push_back( w.data() );
    argv.push_back( nullptr );
    std::fflush( nullptr );
    Child child;
    child.pid = ::fork();
    if( child.pid < 0 )
        throw std::runtime_error( "fork failed" );
    if( child.pid == 0 )
    {
        ::dup2( out[1], STDOUT_FILENO );
        ::dup2( err[1], STDERR_FILENO );
        ::execv( binary.c_str(), argv.data() );
        ::_exit( 127 );
    }
    ::close( out[1] );
    ::close( err[1] );
    child.out.reset( out[0] );
    child.err.reset( err[0] );
    return child;
}

// Collects the helper's output until it closed both streams, then its exit status. A helper that is still running
// after the timeout is killed.
Exit finish( Child & child )
{
    Exit result;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds( kTimeoutMs );
    bool timed_out = false;
    while( child.out.valid() || child.err.valid() )
    {
        pollfd fds[2] = { { child.out.get(), POLLIN, 0 }, { child.err.get(), POLLIN, 0 } };
        const auto left = std::chrono::duration_cast< std::chrono::milliseconds >(
            deadline - std::chrono::steady_clock::now() );
        if( left.count() <= 0 || ::poll( fds, 2, int( left.count() ) ) == 0 )
        {
            timed_out = true;
            break;
        }
        for( int i = 0; i < 2; ++i )
        {
            UniqueFd & fd = i == 0 ? child.out : child.err;
            if( ! fd.valid() || ! ( fds[i].revents & ( POLLIN | POLLHUP | POLLERR ) ) )
                continue;
            char buf[4096];
            ssize_t n = ::read( fd.get(), buf, sizeof( buf ) );
            if( n > 0 )
                ( i == 0 ? result.out : result.err ).append( buf, size_t( n ) );
            else if( n == 0 || errno != EINTR )
                fd.reset();
        }
    }
    if( timed_out )
        ::kill( child.pid, SIGKILL );
    int status = 0;
    while( ::waitpid( child.pid, &status, 0 ) < 0 && errno == EINTR )
    {
    }
    if( ! timed_out )
        result.status = WIFEXITED( status ) ? WEXITSTATUS( status ) : 128 + WTERMSIG( status );
    return result;
}

Exit run_helper( const std::vector< std::string > & args )
{
    Child child = spawn( helper_binary(), args );
    return finish( child );
}

bool contains( const std::string & text, const std::string & part )
{
    return text.find( part ) != std::string::npos;
}

// The arguments every session needs. No socket exists at the path: a helper that accepts the command line and its
// privileges gets as far as the socket and ends with status 69.
std::vector< std::string > session_args( uid_t uid, std::initializer_list< const char * > more = {} )
{
    std::vector< std::string > args{ "--socket", "/nonexistent-ifs-helper-tests/camera.sock", "--uid",
                                     std::to_string( uid ), "--parent-pid", std::to_string( ::getpid() ) };
    args.insert( args.end(), more.begin(), more.end() );
    return args;
}

constexpr int kExitUsage = 64, kExitUnavailable = 69, kExitNoPermission = 77;

// --- The app's side of a session -----------------------------------------------------------------------------------

// What the app sets up for a helper: a folder only the user can enter, in the user's own temporary directory (no
// symbolic links in its path), with a listening socket.
struct AppSocket
{
    std::string dir, path;
    UniqueFd listener;

    AppSocket()
    {
        char temp[PATH_MAX], real[PATH_MAX];
        size_t n = ::confstr( _CS_DARWIN_USER_TEMP_DIR, temp, sizeof( temp ) );
        if( n == 0 || n > sizeof( temp ) || ! ::realpath( temp, real ) )
            throw std::runtime_error( "no temporary directory" );
        dir = make_directory( std::string( real ) + "/ifs-helper-tests.XXXXXX" );
        path = dir + "/camera.sock";
        sockaddr_un addr{};
        if( path.size() >= sizeof( addr.sun_path ) )
            throw std::runtime_error( "socket path too long: " + path );
        addr.sun_family = AF_UNIX;
        std::memcpy( addr.sun_path, path.c_str(), path.size() + 1 );
        listener.reset( ::socket( AF_UNIX, SOCK_STREAM, 0 ) );
        if( ! listener.valid() || ::bind( listener.get(), reinterpret_cast< sockaddr * >( &addr ), sizeof( addr ) ) != 0
            || ::listen( listener.get(), 1 ) != 0 )
            throw std::runtime_error( std::string( "cannot listen on " ) + path + ": " + std::strerror( errno ) );
    }

    ~AppSocket()
    {
        ::unlink( path.c_str() );
        ::rmdir( dir.c_str() );
    }

    // The helper's connection, or an invalid descriptor when none arrived in time.
    UniqueFd accept()
    {
        pollfd fd{ listener.get(), POLLIN, 0 };
        if( ::poll( &fd, 1, kTimeoutMs ) <= 0 )
            return UniqueFd();
        UniqueFd conn( ::accept( listener.get(), nullptr, nullptr ) );
        if( conn.valid() )
        {
            timeval tv{ kTimeoutMs / 1000, 0 };
            ::setsockopt( conn.get(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof( tv ) );
            int on = 1;
            ::setsockopt( conn.get(), SOL_SOCKET, SO_NOSIGPIPE, &on, sizeof( on ) );
        }
        return conn;
    }
};

struct Message
{
    uint16_t type = 0;
    json body;
};

// Reads exactly 'size' bytes; false when the helper closed the connection before the first one.
bool read_exact( int fd, void * data, size_t size )
{
    size_t done = 0;
    while( done < size )
    {
        ssize_t n = ::recv( fd, static_cast< char * >( data ) + done, size - done, 0 );
        if( n == 0 )
        {
            if( done == 0 )
                return false;
            throw std::runtime_error( "the helper closed the connection in the middle of a message" );
        }
        if( n < 0 )
        {
            if( errno == EINTR )
                continue;
            if( errno == EAGAIN )
                throw std::runtime_error( "no message from the helper in time" );
            throw std::runtime_error( std::string( "socket read failed: " ) + std::strerror( errno ) );
        }
        done += size_t( n );
    }
    return true;
}

// One message of docs/PROTOCOL.md from the helper; false at the end of the connection.
bool read_message_from_helper( int fd, Message & out )
{
    ifs_wire_header header;
    if( ! read_exact( fd, &header, sizeof( header ) ) )
        return false;
    if( ifs_wire_header_valid( &header ) != 0 )
        throw std::runtime_error( "invalid message header from the helper" );
    std::string text( header.json_bytes, '\0' );
    if( header.json_bytes && ! read_exact( fd, text.data(), text.size() ) )
        throw std::runtime_error( "the helper closed the connection in the middle of a message" );
    std::vector< char > payload( header.payload_bytes );
    if( header.payload_bytes && ! read_exact( fd, payload.data(), payload.size() ) )
        throw std::runtime_error( "the helper closed the connection in the middle of a message" );
    out.type = header.type;
    out.body = json::parse( text );
    return true;
}

void send_command( int fd, uint16_t type, const json & body )
{
    const std::string text = body.dump();
    ifs_wire_header header;
    ifs_wire_header_init( &header, type, uint32_t( text.size() ), 0, 0 );
    std::string bytes( reinterpret_cast< const char * >( &header ), sizeof( header ) );
    bytes += text;
    if( ::send( fd, bytes.data(), bytes.size(), 0 ) != ssize_t( bytes.size() ) )
        throw std::runtime_error( std::string( "socket write failed: " ) + std::strerror( errno ) );
}

}  // namespace

// --- Recording spool -----------------------------------------------------------------------------------------------

TEST_CASE( spool_is_private_and_delivers_the_recording )
{
    std::unique_ptr< RecordingSpool > spool = RecordingSpool::create( "db3" );
    const std::string dir = spool->directory();
    struct stat st;
    REQUIRE( ::lstat( dir.c_str(), &st ) == 0 );
    CHECK( S_ISDIR( st.st_mode ) );
    CHECK_MSG( ( st.st_mode & 07777 ) == 0700, std::oct << ( st.st_mode & 07777 ) );
    CHECK( st.st_uid == ::geteuid() );
    CHECK( spool->file_path() == dir + "/recording.db3" );
    // The folder it is in belongs to the user running the helper and nobody else can enter it: the same rule for
    // root (the administrator path) and for the user (the camera service path, the fake camera).
    REQUIRE( ::stat( parent_of( dir ).c_str(), &st ) == 0 );
    CHECK( st.st_uid == ::geteuid() );
    CHECK_MSG( ( st.st_mode & 077 ) == 0, std::oct << ( st.st_mode & 07777 ) );

    // The engine writes the recording there; it reaches the app through the descriptor the app passed.
    const std::string recorded = "the recording\n";
    write_file( spool->file_path(), recorded );
    const std::string app_dir = make_directory( parent_of( dir ) + "/ifs-helper-tests.XXXXXX" );
    const std::string app_file = app_dir + "/scan.db3";
    {
        UniqueFd destination( ::open( app_file.c_str(), O_RDWR | O_CREAT | O_EXCL, 0600 ) );
        REQUIRE( destination.valid() );
        CHECK( check_recording_destination( destination.get() ).empty() );
        CHECK( spool->copy_into( destination.get() ) == recorded.size() );
        char buf[64] = {};
        CHECK( ::pread( destination.get(), buf, sizeof( buf ), 0 ) == ssize_t( recorded.size() ) );
        CHECK( recorded == buf );
    }
    ::unlink( app_file.c_str() );
    ::rmdir( app_dir.c_str() );

    spool->remove();
    CHECK( ! exists( dir ) );
    CHECK_THROWS_WITH( RecordingSpool::create( "mp4" ), "unsupported recording format" );
}

TEST_CASE( spool_sweep_removes_abandoned_spools_but_none_in_use )
{
    const time_t two_days = 2 * 24 * 60 * 60;
    // A recording that has been running for two days: the helper holds the spool, and nothing changed the folder
    // itself since it was created (the recorder only grows the file inside).
    std::unique_ptr< RecordingSpool > in_use = RecordingSpool::create( "bag" );
    write_file( in_use->file_path(), "still recording" );
    set_age( in_use->directory(), two_days );
    const std::string base = parent_of( in_use->directory() );

    // What a helper that was killed two days ago left behind, and what one that ended a moment ago left.
    const std::string abandoned = make_directory( base + "/ifs-recording.XXXXXX" );
    write_file( abandoned + "/recording.db3", "left behind" );
    set_age( abandoned, two_days );
    const std::string recent = make_directory( base + "/ifs-recording.XXXXXX" );

    // The next recording of any helper of this user sweeps the folder.
    std::unique_ptr< RecordingSpool > next = RecordingSpool::create( "db3" );
    CHECK_MSG( ! exists( abandoned ), "an abandoned spool older than a day is removed" );
    CHECK_MSG( exists( in_use->file_path() ), "a spool a running helper holds is never removed" );
    CHECK_MSG( exists( recent ), "a spool younger than a day is left alone" );
    CHECK( exists( next->directory() ) );

    // Once its helper is gone (here: the spool object), the old spool is abandoned too.
    const std::string released = in_use->directory();
    in_use.reset();
    CHECK( ! exists( released ) );
    ::rmdir( recent.c_str() );
}

// --- Command line and privileges -----------------------------------------------------------------------------------

TEST_CASE( cli_version_and_help )
{
    Exit version = run_helper( { "--version" } );
    CHECK_MSG( version.status == 0, version.status << " " << version.err );
    CHECK_MSG( version.out.rfind( "IntelFaceScanHelper (librealsense ", 0 ) == 0, version.out );

    Exit help = run_helper( { "--help" } );
    CHECK( help.status == 0 );
    for( const char * word : { "--socket", "--uid", "--parent-pid", "--gid", "--no-usb-release", "--playback",
                               "--fake", "--probe", "--release", "RealSense camera service" } )
        CHECK_MSG( contains( help.out, word ), word );

    Exit unknown = run_helper( { "--no-such-option" } );
    CHECK( unknown.status == kExitUsage );
    CHECK_MSG( contains( unknown.err, "unknown argument: --no-such-option" ), unknown.err );
}

TEST_CASE( cli_no_usb_release_is_for_the_live_camera_only )
{
    const uid_t me = ::getuid();
    Exit fake = run_helper( session_args( me, { "--fake", "--no-usb-release" } ) );
    CHECK_MSG( fake.status == kExitUsage, fake.status << " " << fake.err );
    CHECK_MSG( contains( fake.err, "--no-usb-release is only for the live camera" ), fake.err );

    Exit playback = run_helper( session_args( me, { "--playback", "/nonexistent.db3", "--no-usb-release" } ) );
    CHECK_MSG( playback.status == kExitUsage, playback.status << " " << playback.err );
    CHECK_MSG( contains( playback.err, "--no-usb-release is only for the live camera" ), playback.err );

    Exit twice = run_helper( session_args( me, { "--no-usb-release", "--no-usb-release" } ) );
    CHECK_MSG( twice.status == kExitUsage, twice.status << " " << twice.err );
    CHECK_MSG( contains( twice.err, "--no-usb-release given twice" ), twice.err );

    Exit value = run_helper( session_args( me, { "--no-usb-release=1" } ) );
    CHECK_MSG( value.status == kExitUsage, value.status << " " << value.err );
}

TEST_CASE( cli_live_mode_runs_as_the_user )
{
    if( ::geteuid() == 0 )
        SKIP( "runs the helper as a user" );
    const uid_t me = ::getuid();
    // The live camera no longer needs root: the helper accepts the command line and its privileges and goes on to
    // the app's socket, which is not there (69, not 77).
    for( auto more : { std::initializer_list< const char * >{},
                       std::initializer_list< const char * >{ "--gid", "20" },
                       std::initializer_list< const char * >{ "--no-usb-release" },
                       std::initializer_list< const char * >{ "--gid", "20", "--no-usb-release" } } )
    {
        Exit live = run_helper( session_args( me, more ) );
        CHECK_MSG( live.status == kExitUnavailable, live.status << " " << live.err );
        CHECK_MSG( contains( live.err, "socket" ), live.err );
        CHECK_MSG( ! contains( live.err, "root" ), live.err );
    }

    // As for a recording and the fake camera: a helper of the user serves that user only.
    for( auto more : { std::initializer_list< const char * >{},
                       std::initializer_list< const char * >{ "--fake" },
                       std::initializer_list< const char * >{ "--playback", "/nonexistent.db3" } } )
    {
        Exit other = run_helper( session_args( me + 1, more ) );
        CHECK_MSG( other.status == kExitNoPermission, other.status << " " << other.err );
        CHECK_MSG( contains( other.err, "--uid must be the user running the helper" ), other.err );
    }
}

// --- A live session as the user ------------------------------------------------------------------------------------

TEST_CASE( live_session_as_the_user )
{
    if( ::geteuid() == 0 )
        SKIP( "runs the helper as a user" );
    const std::string binary = helper_binary();
    AppSocket app;
    Child child = spawn( binary, { "--socket", app.path, "--uid", std::to_string( ::getuid() ), "--gid",
                                   std::to_string( ::getgid() ), "--parent-pid", std::to_string( ::getpid() ) } );
    UniqueFd conn = app.accept();
    if( ! conn.valid() )
    {
        ::kill( child.pid, SIGTERM );
        Exit e = finish( child );
        throw Failure( "the live helper did not connect as the user (status " + std::to_string( e.status )
                       + "): " + e.err );
    }

    std::vector< std::string > logs, errors;
    json hello, reply, bye;
    std::string failure;
    try
    {
        Message m;
        if( ! read_message_from_helper( conn.get(), m ) || m.type != IFS_MSG_HELLO )
            throw std::runtime_error( "the first message is not HELLO" );
        hello = m.body;
        // QUIT right away: the helper answers it after it tried to open the camera.
        send_command( conn.get(), IFS_CMD_QUIT, { { "request_id", 1 } } );
        while( read_message_from_helper( conn.get(), m ) )
        {
            if( m.type == IFS_MSG_LOG )
                logs.push_back( m.body.value( "message", "" ) );
            else if( m.type == IFS_MSG_EVENT && m.body.value( "event", "" ) == "error" )
                errors.push_back( m.body.value( "message", "" ) );
            else if( m.type == IFS_MSG_REPLY )
                reply = m.body;
            else if( m.type == IFS_MSG_BYE )
                bye = m.body;
        }
    }
    catch( const std::exception & e )
    {
        failure = e.what();
        ::kill( child.pid, SIGTERM );
    }
    conn.reset();
    Exit e = finish( child );
    for( const std::string & line : errors )
        std::printf( "    the helper reported: %s\n", line.c_str() );
    if( ! failure.empty() )
        throw Failure( failure + " (helper status " + std::to_string( e.status ) + ", stderr: " + e.err + ")" );

    CHECK_MSG( hello.value( "mode", "" ) == "live", hello.dump() );
    CHECK_MSG( hello.value( "euid", -1 ) == int( ::getuid() ), hello.dump() );
    CHECK_MSG( hello.value( "pid", -1 ) == int( child.pid ), hello.dump() );
    CHECK_MSG( reply.value( "request_id", 0 ) == 1 && reply.value( "ok", false ), reply.dump() );
    CHECK_MSG( bye.value( "reason", "" ) == "the app asked the helper to quit", bye.dump() );
    CHECK_MSG( e.status == 0, e.status << " " << e.err );
    CHECK_MSG( e.err.empty(), e.err );  // after the connection exists everything goes over the socket

    // A helper of the user never re-enumerates USB devices (it could not, and the camera service hands the camera
    // back itself): none of the release's lines (usb_release.cpp) may show up.
    for( const std::string & line : logs )
        for( const char * release_line : { "re-enumerated", "No RealSense device found", "macOS drivers",
                                           "failed to enumerate USB devices", "handed back", "--no-usb-release" } )
            CHECK_MSG( ! contains( line, release_line ), line );
    bool shutting_down = false;
    for( const std::string & line : logs )
        shutting_down = shutting_down || contains( line, "helper shutting down" );
    CHECK( shutting_down );
    // Whatever kept the camera from opening (none connected, or no camera service), nobody is told to use sudo.
    for( const std::string & line : errors )
        CHECK_MSG( ! contains( line, "sudo" ), line );
}

int main( int argc, char ** argv )
{
    return run_tests( argc, argv );
}
