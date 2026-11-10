// License: Apache 2.0. See LICENSE file in root directory.
//
// IntelFaceScanHelper - the camera helper process (docs/HELPER.md, docs/PROTOCOL.md).
//
//   IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> [--gid <gid>] [--no-usb-release]   live camera
//   IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --playback <file> [--no-real-time]
//   IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --fake [--fake-row-padding <bytes>]
//   IntelFaceScanHelper --probe ...                                                            measurements
//   IntelFaceScanHelper --release [-w]                                                         hand cameras back (root)
//
// The live camera runs in two ways:
// * As the user, started directly by the app. The patched librealsense reaches the camera through the RealSense
//   camera service (realsense-usbd, a root LaunchDaemon shared by RealSense applications), which owns the USB device
//   and hands it back to macOS itself a few seconds after the last program let go of it. Nothing here needs root, and
//   the helper never re-enumerates USB devices.
// * As root, started through the administrator prompt: the path for a Mac where the service cannot be used.
//   librealsense opens the USB device in this process, and the helper re-enumerates it when the session ends so that
//   macOS gets its drivers back. --no-usb-release skips that: the app passes it when the service is running but does
//   not admit this user, because root's librealsense then goes through the service too, and a re-enumeration would
//   pull the camera away from it.

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
    bool no_usb_release = false;
    bool help = false;
    bool version = false;
};

void usage( FILE * out )
{
    std::fprintf( out,
                  "usage: IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> [--gid <gid>] "
                  "[--no-usb-release]\n"
                  "       IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --playback <file> "
                  "[--no-real-time]\n"
                  "       IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --fake "
                  "[--fake-row-padding <bytes>]\n"
                  "       IntelFaceScanHelper --probe ...\n"
                  "       IntelFaceScanHelper --release [-w]\n"
                  "The first form is the live camera. As the user (--uid is the caller) it uses the camera through "
                  "the\n"
                  "RealSense camera service. As root (the administrator path) it opens the USB device itself and "
                  "hands the\n"
                  "camera back to macOS when the session ends (USB re-enumeration), unless --no-usb-release is "
                  "given.\n"
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
        else if( arg == "--no-usb-release" )
        {
            if( ! once( o.no_usb_release ) )
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
    if( o.no_usb_release && o.mode != ifs::HelperMode::live )
    {
        error = "--no-usb-release is only for the live camera";
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

// Only the live camera may run as root (the administrator path); anything that only reads files or makes frames up
// never does. Every helper that is not root, the live one included (the RealSense camera service path), must be run
// by the user named with --uid: the socket checks then tie it to that user's app.
int check_privileges( const Options & o )
{
    if( ::geteuid() == 0 || ::getuid() == 0 )
    {
        if( o.mode != ifs::HelperMode::live )
        {
            std::fprintf( stderr, "IntelFaceScanHelper: refusing to run %s mode as root\n", ifs::mode_name( o.mode ) );
            return kExitNoPermission;
        }
        if( ::geteuid() != 0 )
        {
            // Root that gave up its effective user ID could take it back: neither the user's helper nor root's.
            std::fprintf( stderr, "IntelFaceScanHelper: the real user ID is root but the effective one is not; run "
                                  "the live camera as the user or as root\n" );
            return kExitNoPermission;
        }
        return 0;
    }
    if( o.uid != ::getuid() )
    {
        std::fprintf( stderr, "IntelFaceScanHelper: --uid must be the user running the helper\n" );
        return kExitNoPermission;
    }
    return 0;
}

// True when this live helper hands the camera back to macOS itself at the end (USB re-enumeration): it runs as root,
// so librealsense may have captured the USB device in this process, and the app did not say that the RealSense camera
// service owns the camera (--no-usb-release). A helper of the user never does: its librealsense got the camera from
// the service, which hands it back itself, and only root can re-enumerate a device.
bool releases_usb( const Options & o )
{
    return o.mode == ifs::HelperMode::live && ::geteuid() == 0 && ! o.no_usb_release;
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
    // shutdown sequence: a root helper that skipped it would leave the camera captured until it is re-plugged (a
    // helper of the user leaves that to the camera service, which notices that its client is gone).
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
    if( o.mode == ifs::HelperMode::live && ::geteuid() == 0 && o.no_usb_release )
        log( "info", "the camera is not re-enumerated (--no-usb-release): the RealSense camera service hands it "
                     "back to macOS" );
    if( releases_usb( o ) )
    {
        // After shutdown_engine(): the librealsense context is gone, nothing uses the device any more. Every
        // RealSense device without a macOS driver is released, not only the one captured here: librealsense does not
        // report its capture, and no other sign of it is reliable (docs/HELPER.md, session end).
        ifs::UsbReleaseResult r;
        try
        {
            r = ifs::release_realsense_devices( kReleaseWaitMs,
                                                [&log]( const std::string & line ) { log( "info", line ); } );
        }
        catch( ... )
        {
            r.failed = 1;
        }
        if( r.failed )
            log( "warning", "the camera could not be handed back to macOS; re-plug it or run "
                            "'sudo IntelFaceScanHelper --release'" );
    }
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

// `--probe` on the live camera. As root the camera is captured from the macOS drivers like in a session of the
// administrator path, so it is handed back the same way afterwards, whatever the probe's outcome (Ctrl-C included:
// the probe stops cleanly on SIGINT). As the user the probe works through the RealSense camera service.
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
    // Not root: this process captured nothing. The probe either went through the RealSense camera service, which
    // hands the camera back itself, or could not open the camera at all; and only root can re-enumerate a device.
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
