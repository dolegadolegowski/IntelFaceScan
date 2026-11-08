// License: Apache 2.0. See LICENSE file in root directory.
//
// IntelFaceScanHelper - the camera helper process (docs/HELPER.md, docs/PROTOCOL.md).
//
//   IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid>                         live camera (user)
//   IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --playback <file> [--no-real-time]
//   IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --fake [--fake-row-padding <bytes>]
//   IntelFaceScanHelper --probe ...                                                            measurements (live: root only without the camera access service)
//   IntelFaceScanHelper --release [-w]                                                         hand cameras back

#include "capture_engine.h"
#include "fake_engine.h"
#include "ipc_connection.h"
#include "ipc_lifecycle.h"
#include "ipc_service.h"
#include "ipc_socket.h"
#include "probe.h"
#include "security_process.h"
#include "usb_release.h"

#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

namespace
{
// sysexits.h values, so the app can tell the failures apart in the exit status.
constexpr int kExitUsage = 64;        // EX_USAGE: bad command line
constexpr int kExitUnavailable = 69;  // EX_UNAVAILABLE: the app's socket is unusable or the app is gone
constexpr int kExitSoftware = 70;     // EX_SOFTWARE: internal error
constexpr int kExitNoPermission = 77; // EX_NOPERM: wrong privileges for the mode, or the peer is not the app

constexpr int kReleaseWaitMs = 5000;
constexpr auto kByeTimeout = std::chrono::milliseconds( 2000 );

struct Options
{
    std::string socket;
    bool has_uid = false;
    uid_t uid = 0;
    bool has_gid = false;
    gid_t gid = 0;
    pid_t parent_pid = 0;
    ifs::HelperMode mode = ifs::HelperMode::live;
    std::string playback_file;
    bool real_time = true;
    bool real_time_flag = false;
    ifs::FakeOptions fake;
    bool padding_flag = false;
    bool help = false;
    bool version = false;
};

void usage( FILE * out )
{
    std::fprintf( out,
                  "usage: IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid>\n"
                  "       IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --playback <file> "
                  "[--no-real-time]\n"
                  "       IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --fake "
                  "[--fake-row-padding <bytes>]\n"
                  "       IntelFaceScanHelper --probe ...\n"
                  "       IntelFaceScanHelper --release [-w]\n"
                  "Started by IntelFaceScan; see docs/HELPER.md.\n" );
}

bool parse_number( const char * text, unsigned long max, unsigned long & out )
{
    if( ! text || ! *text || *text < '0' || *text > '9' )
        return false;
    errno = 0;
    char * end = nullptr;
    unsigned long v = std::strtoul( text, &end, 10 );
    if( errno != 0 || *end != '\0' || v > max )
        return false;
    out = v;
    return true;
}

bool parse_args( int argc, char ** argv, Options & o, std::string & error )
{
    bool has_socket = false, has_parent = false, has_playback = false, has_fake = false;
    for( int i = 1; i < argc; ++i )
    {
        const std::string arg = argv[i];
        auto value = [&]( const char * name ) -> const char * {
            if( i + 1 >= argc )
            {
                error = std::string( name ) + " needs a value";
                return nullptr;
            }
            return argv[++i];
        };
        auto once = [&]( bool & seen ) {
            if( seen )
            {
                error = arg + " given twice";
                return false;
            }
            seen = true;
            return true;
        };
        unsigned long n = 0;
        if( arg == "--help" || arg == "-h" )
            o.help = true;
        else if( arg == "--version" )
            o.version = true;
        else if( arg == "--socket" )
        {
            const char * v = value( "--socket" );
            if( ! v || ! once( has_socket ) )
                return false;
            o.socket = v;
        }
        else if( arg == "--uid" || arg == "--gid" || arg == "--parent-pid" || arg == "--fake-row-padding" )
        {
            const char * v = value( arg.c_str() );
            if( ! v )
                return false;
            const unsigned long max = arg == "--fake-row-padding" ? 4096 : arg == "--parent-pid" ? INT_MAX : UINT_MAX - 1;
            if( ! parse_number( v, max, n ) )
            {
                error = "invalid value for " + arg + ": " + v;
                return false;
            }
            if( arg == "--uid" )
            {
                if( ! once( o.has_uid ) )
                    return false;
                o.uid = uid_t( n );
            }
            else if( arg == "--gid" )
            {
                if( ! once( o.has_gid ) )
                    return false;
                o.gid = gid_t( n );
            }
            else if( arg == "--parent-pid" )
            {
                if( ! once( has_parent ) )
                    return false;
                o.parent_pid = pid_t( n );
            }
            else
            {
                if( ! once( o.padding_flag ) )
                    return false;
                o.fake.color_row_padding = int( n );
            }
        }
        else if( arg == "--playback" )
        {
            const char * v = value( "--playback" );
            if( ! v || ! once( has_playback ) )
                return false;
            o.playback_file = v;
        }
        else if( arg == "--no-real-time" )
        {
            if( ! once( o.real_time_flag ) )
                return false;
            o.real_time = false;
        }
        else if( arg == "--fake" )
        {
            if( ! once( has_fake ) )
                return false;
        }
        else
        {
            error = "unknown argument: " + arg;
            return false;
        }
    }
    if( o.help || o.version )
        return true;
    if( has_playback && has_fake )
    {
        error = "--playback and --fake are exclusive";
        return false;
    }
    o.mode = has_playback ? ifs::HelperMode::playback : has_fake ? ifs::HelperMode::fake : ifs::HelperMode::live;
    if( o.real_time_flag && o.mode != ifs::HelperMode::playback )
    {
        error = "--no-real-time needs --playback";
        return false;
    }
    if( o.padding_flag && o.mode != ifs::HelperMode::fake )
    {
        error = "--fake-row-padding needs --fake";
        return false;
    }
    if( ! has_socket || ! o.has_uid || ! has_parent )
    {
        error = "--socket, --uid and --parent-pid are required";
        return false;
    }
    if( o.parent_pid <= 1 )
    {
        error = "invalid --parent-pid";
        return false;
    }
    return true;
}

// No mode needs root: the live camera goes through the camera access service of the RealSense tools (a root
// LaunchDaemon that owns the USB device; docs/HELPER.md), everything else reads files or makes frames up.
int check_privileges( const Options & o )
{
    if( ::geteuid() == 0 || ::getuid() == 0 )
    {
        std::fprintf( stderr, "IntelFaceScanHelper: refusing to run %s mode as root\n", ifs::mode_name( o.mode ) );
        return kExitNoPermission;
    }
    if( o.uid != ::getuid() )
    {
        std::fprintf( stderr, "IntelFaceScanHelper: --uid must be the user running the helper\n" );
        return kExitNoPermission;
    }
    return 0;
}

ifs::HelperService::EngineFactory make_factory( const Options & o )
{
    switch( o.mode )
    {
    case ifs::HelperMode::live:
        return []( ifs::CaptureCallbacks cb ) { return ifs::CaptureEngine::create_live( std::move( cb ) ); };
    case ifs::HelperMode::playback:
    {
        // The socket connection changes the working directory, so a relative path is resolved first.
        std::string file = o.playback_file;
        char real[PATH_MAX];
        if( ::realpath( file.c_str(), real ) )
            file = real;
        const bool real_time = o.real_time;
        return [file, real_time]( ifs::CaptureCallbacks cb ) {
            return ifs::CaptureEngine::create_playback( file, real_time, std::move( cb ) );
        };
    }
    case ifs::HelperMode::fake:
    {
        const ifs::FakeOptions fake = o.fake;
        return [fake]( ifs::CaptureCallbacks cb ) { return ifs::create_fake_engine( std::move( cb ), fake ); };
    }
    }
    return nullptr;
}

// Runs commands until something ends the session; returns why.
std::string session_loop( ifs::Lifecycle & life, ifs::Connection & connection, ifs::HelperService & service )
{
    uint64_t reported_drops = 0;
    for( ;; )
    {
        if( life.wait( 5000 ) == ifs::Lifecycle::Event::terminate )
            return life.reason();
        // Nothing more is run once the connection broke: the replies could not be sent, and each command could take
        // a while on the camera (up to kMaxPendingCommands of them may still be queued).
        while( ! connection.broken() )
        {
            auto command = connection.pop_command();
            if( ! command )
                break;
            service.handle( *command );
            if( service.quit_requested() )
                return "the app asked the helper to quit";
            // A command can take seconds (advanced mode, recordings): do not start another one after the app is gone.
            if( life.wait( 0 ) == ifs::Lifecycle::Event::terminate )
                return life.reason();
        }
        if( connection.broken() )
            return connection.broken_reason();
        const uint64_t drops = connection.framesets_dropped();
        if( drops > reported_drops )
        {
            service.log( "debug", std::to_string( drops - reported_drops )
                                      + " framesets dropped because the app read them slower than they arrived" );
            reported_drops = drops;
        }
    }
}

int run_session( const Options & o )
{
    // Signals are ignored and recorded from here on, before anything that could be interrupted.
    ifs::Lifecycle life;
    life.watch_parent( o.parent_pid );
    if( life.terminating() )
    {
        std::fprintf( stderr, "IntelFaceScanHelper: %s\n", life.reason().c_str() );
        return kExitUnavailable;
    }
    ifs::HelperService::EngineFactory factory = make_factory( o );

    ifs::UniqueFd sock;
    try
    {
        sock = ifs::connect_app_socket( o.socket, o.uid );
    }
    catch( const std::exception & e )
    {
        std::fprintf( stderr, "IntelFaceScanHelper: %s\n", e.what() );
        return kExitUnavailable;
    }
    try
    {
        ifs::verify_app_peer( sock.get(), o.uid, o.parent_pid );
    }
    catch( const std::exception & e )
    {
        std::fprintf( stderr, "IntelFaceScanHelper: %s\n", e.what() );
        return kExitNoPermission;
    }

    ifs::Connection connection( std::move( sock ), [&life] { life.wake(); } );
    connection.start();
    ifs::HelperService service( connection, o.mode, std::move( factory ), o.real_time );

    // From here on every way out, an exception included (e.g. bad_alloc while framing a reply), goes through the
    // shutdown sequence: a helper that skipped it would leave the camera leased until the service notices the dead connection.
    std::string reason;
    int status = 0;
    try
    {
        service.send_hello();
        service.create_engine();
        reason = session_loop( life, connection, service );
    }
    catch( const std::exception & e )
    {
        reason = std::string( "internal error: " ) + e.what();
        status = kExitSoftware;
    }
    catch( ... )
    {
        reason = "internal error";
        status = kExitSoftware;
    }

    auto log = [&service]( const char * level, const std::string & line ) {
        try
        {
            service.log( level, line );
        }
        catch( ... )
        {
        }
    };
    log( "info", "helper shutting down: " + reason );
    try
    {
        service.shutdown_engine();
    }
    catch( ... )
    {
        status = kExitSoftware;
    }
    connection.drop_pending_framesets();
    // The camera access service takes the camera back when the last connection of this process is gone (and hands it
    // to macOS after a short grace period): nothing to release here.
    try
    {
        if( ! connection.broken() )
            connection.send( IFS_MSG_BYE, { { "reason", reason } } );
    }
    catch( ... )
    {
    }
    connection.finish( kByeTimeout );
    return status;
}

// `--probe` on the live camera: the camera is captured from the macOS drivers like in a session, so it is handed back
// the same way afterwards, whatever the probe's outcome (Ctrl-C included: the probe stops cleanly on SIGINT).
int run_probe_tool( int argc, char ** argv )
{
    bool live = false;
    int rc = 1;
    try
    {
        rc = ifs::run_probe( argc, argv, &live );
    }
    catch( ... )
    {
        std::fprintf( stderr, "probe: internal error\n" );
    }
    // Without root nothing could be captured (libusb needs it), and a re-enumeration would fail anyway.
    if( live && ::geteuid() == 0 )
    {
        ifs::UsbReleaseResult r;
        try
        {
            r = ifs::release_realsense_devices( kReleaseWaitMs, []( const std::string & line ) {
                std::fprintf( stderr, "%s\n", line.c_str() );
            } );
        }
        catch( ... )
        {
            r.failed = 1;
        }
        if( r.failed )
            std::fprintf( stderr, "The camera could not be handed back to macOS: re-plug it or run "
                                  "'sudo IntelFaceScanHelper --release -w'\n" );
    }
    return rc;
}
}  // namespace

int main( int argc, char ** argv )
{
    ifs::harden_process();

    if( argc > 1 && std::strcmp( argv[1], "--probe" ) == 0 )
        return run_probe_tool( argc - 2, argv + 2 );
    if( argc > 1 && std::strcmp( argv[1], "--release" ) == 0 )
        return ifs::run_release_tool( argc - 2, argv + 2 );

    Options options;
    std::string error;
    if( ! parse_args( argc, argv, options, error ) )
    {
        std::fprintf( stderr, "IntelFaceScanHelper: %s\n", error.c_str() );
        usage( stderr );
        return kExitUsage;
    }
    if( options.help )
    {
        usage( stdout );
        return 0;
    }
    if( options.version )
    {
        std::printf( "IntelFaceScanHelper (librealsense %s)\n", ifs::sdk_version().c_str() );
        return 0;
    }
    if( int rc = check_privileges( options ) )
        return rc;

    try
    {
        return run_session( options );
    }
    catch( const std::exception & e )
    {
        std::fprintf( stderr, "IntelFaceScanHelper: %s\n", e.what() );
        return kExitSoftware;
    }
}
