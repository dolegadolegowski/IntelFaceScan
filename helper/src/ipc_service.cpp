// License: Apache 2.0. See LICENSE file in root directory.

#include "ipc_service.h"

#include <atomic>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <unistd.h>

using nlohmann::json;

namespace ifs
{

namespace
{
constexpr const char * kHelperVersion = "0.1.0";
constexpr size_t kMaxTextBytes = 16 * 1024;

[[noreturn]] void fail( const std::string & message )
{
    throw std::runtime_error( message );
}

std::string clip( const std::string & text )
{
    return text.size() <= kMaxTextBytes ? text : text.substr( 0, kMaxTextBytes ) + "...";
}

// Strict validation: a field the helper does not know means the app and the helper disagree about the protocol.
void check_fields( const json & body, std::initializer_list< const char * > allowed )
{
    for( auto it = body.begin(); it != body.end(); ++it )
    {
        bool known = false;
        for( const char * name : allowed )
            known = known || it.key() == name;
        if( ! known )
            fail( "unknown field '" + it.key() + "'" );
    }
}

const json & require( const json & body, const char * key )
{
    auto it = body.find( key );
    if( it == body.end() || it->is_null() )
        fail( std::string( "missing field '" ) + key + "'" );
    return *it;
}

std::string get_string( const json & body, const char * key )
{
    const json & v = require( body, key );
    if( ! v.is_string() )
        fail( std::string( "field '" ) + key + "' must be a string" );
    return v.get< std::string >();
}

double get_number( const json & body, const char * key )
{
    const json & v = require( body, key );
    if( ! v.is_number() )
        fail( std::string( "field '" ) + key + "' must be a number" );
    double d = v.get< double >();
    if( ! std::isfinite( d ) )
        fail( std::string( "field '" ) + key + "' must be finite" );
    return d;
}

int get_int( const json & body, const char * key )
{
    double d = get_number( body, key );
    if( d != std::floor( d ) || d < double( std::numeric_limits< int >::min() )
        || d > double( std::numeric_limits< int >::max() ) )
        fail( std::string( "field '" ) + key + "' must be an integer" );
    return int( d );
}

json without_request_id( const json & body )
{
    json copy = body;
    copy.erase( "request_id" );
    return copy;
}

json event_body( const std::string & name, const std::string & message, const json & data )
{
    return { { "event", name }, { "message", message }, { "data", data.is_null() ? json::object() : data } };
}
}  // namespace

const char * mode_name( HelperMode mode )
{
    switch( mode )
    {
    case HelperMode::live: return "live";
    case HelperMode::playback: return "playback";
    case HelperMode::fake: return "fake";
    }
    return "unknown";
}

HelperService::HelperService( Connection & connection, HelperMode mode, EngineFactory factory, bool playback_real_time )
    : connection_( connection )
    , mode_( mode )
    , factory_( std::move( factory ) )
    , playback_real_time_( playback_real_time )
{
}

HelperService::~HelperService()
{
    shutdown_engine();
}

void HelperService::send_hello()
{
    connection_.send( IFS_MSG_HELLO, { { "protocol", IFS_PROTOCOL_VERSION },
                                       { "helper_version", kHelperVersion },
                                       { "sdk_version", sdk_version() },
                                       { "pid", ::getpid() },
                                       { "euid", ::geteuid() },
                                       { "mode", mode_name( mode_ ) } } );
}

void HelperService::create_engine()
{
    try
    {
        engine();
    }
    catch( const std::exception & e )
    {
        log( "error", e.what() );
        connection_.send( IFS_MSG_EVENT, event_body( "error", clip( e.what() ), json::object() ) );
    }
}

CaptureEngine & HelperService::engine()
{
    if( ! engine_ )
    {
        try
        {
            engine_ = factory_( callbacks() );
        }
        catch( const std::exception & e )
        {
            engine_error_ = e.what();
            throw;
        }
        if( ! engine_ )
            fail( "the camera engine could not be created" );
        engine_error_.clear();
    }
    return *engine_;
}

CaptureCallbacks HelperService::callbacks()
{
    // Engine threads must never see an exception from the IPC layer (e.g. bad_alloc while framing).
    CaptureCallbacks cb;
    cb.on_frameset = [this]( Frameset && fs ) {
        try
        {
            on_frameset( std::move( fs ) );
        }
        catch( ... )
        {
        }
    };
    cb.on_event = [this]( const std::string & name, const json & data ) {
        try
        {
            on_event( name, data );
        }
        catch( ... )
        {
        }
    };
    cb.on_log = [this]( const std::string & level, const std::string & message ) {
        try
        {
            log( level, message );
        }
        catch( ... )
        {
        }
    };
    return cb;
}

void HelperService::log( const std::string & level, const std::string & message )
{
    const bool known = level == "debug" || level == "info" || level == "warning" || level == "error";
    connection_.send( IFS_MSG_LOG, { { "level", known ? level : "info" }, { "message", clip( message ) } } );
}

void HelperService::on_frameset( Frameset && fs )
{
    static std::atomic< bool > warned{ false };
    OutMessage msg;
    json meta = fs.metadata.is_object() ? std::move( fs.metadata ) : json::object();
    json planes = json::array();
    uint64_t offset = 0;
    for( Plane & p : fs.planes )
    {
        if( p.bytes && ! p.data )
        {
            if( ! warned.exchange( true ) )
                log( "error", "dropped a frameset with a plane without data" );
            return;
        }
        json desc = p.desc.is_object() ? std::move( p.desc ) : json::object();
        desc["offset"] = offset;
        desc["bytes"] = p.bytes;
        planes.push_back( std::move( desc ) );
        if( p.bytes )
            msg.payload.push_back( { const_cast< uint8_t * >( p.data ), p.bytes } );
        msg.keepalive.push_back( std::move( p.keepalive ) );
        offset += p.bytes;
    }
    meta["planes"] = std::move( planes );
    if( ! make_message( msg, IFS_MSG_FRAMESET, meta ) )
    {
        if( ! warned.exchange( true ) )
            log( "error", "dropped a frameset above the protocol size limits" );
        return;
    }
    connection_.send_frameset( std::move( msg ) );
}

void HelperService::on_event( const std::string & name, const json & data )
{
    // The service reports these itself: only it knows how many bytes reached the app's file.
    if( name == "recording_started" || name == "recording_finished" )
        return;
    std::string message;
    if( data.is_object() )
    {
        auto it = data.find( "message" );
        if( it != data.end() && it->is_string() )
            message = it->get< std::string >();
    }
    connection_.send( IFS_MSG_EVENT, event_body( name, clip( message ), data ) );
}

void HelperService::reply( uint64_t request_id, bool ok, const json & data, const std::string & error )
{
    json body = { { "request_id", request_id }, { "ok", ok } };
    if( ok )
        body["data"] = data.is_null() ? json::object() : data;
    else
        body["error"] = clip( error.empty() ? std::string( "failed" ) : error );
    if( ! connection_.send( IFS_MSG_REPLY, body ) && ! connection_.broken() )
        connection_.send( IFS_MSG_REPLY, { { "request_id", request_id }, { "ok", false },
                                           { "error", "the reply exceeds the protocol size limit" } } );
}

void HelperService::handle( InMessage & msg )
{
    json data;
    std::string error;
    bool ok = false;
    try
    {
        data = dispatch( msg );
        ok = true;
    }
    catch( const std::exception & e )
    {
        error = e.what();
    }
    catch( ... )
    {
        error = "unknown error";
    }
    msg.fd.reset();  // a descriptor no command took is closed now, not when the message is destroyed
    reply( msg.request_id, ok, data, error );
}

json HelperService::dispatch( InMessage & msg )
{
    const json & b = msg.body;
    switch( msg.type )
    {
    case IFS_CMD_QUERY_DEVICE:
        check_fields( b, { "request_id" } );
        return engine().device_info();

    case IFS_CMD_CONFIGURE:
        engine().configure( without_request_id( b ) );
        return json::object();

    case IFS_CMD_START:
        check_fields( b, { "request_id" } );
        engine().start();
        return json::object();

    case IFS_CMD_STOP:
        check_fields( b, { "request_id" } );
        engine().stop();
        return json::object();

    case IFS_CMD_SET_OPTION:
    {
        check_fields( b, { "request_id", "sensor", "option", "value" } );
        const std::string sensor = get_string( b, "sensor" );
        if( sensor != "depth" && sensor != "color" )
            fail( "sensor must be 'depth' or 'color'" );
        const std::string option = get_string( b, "option" );
        if( option.empty() )
            fail( "option name is empty" );
        const double value = get_number( b, "value" );
        engine().set_option( sensor, option, float( value ) );
        return json::object();
    }

    case IFS_CMD_SET_ROI:
    {
        check_fields( b, { "request_id", "x", "y", "width", "height", "distance_m" } );
        const int x = get_int( b, "x" ), y = get_int( b, "y" );
        const int width = get_int( b, "width" ), height = get_int( b, "height" );
        if( width <= 0 || height <= 0 )
            fail( "the ROI must have a positive size" );
        float distance = -1.f;
        auto it = b.find( "distance_m" );
        if( it != b.end() && ! it->is_null() )
            distance = float( get_number( b, "distance_m" ) );
        engine().set_auto_exposure_roi( x, y, width, height, distance );
        return json::object();
    }

    case IFS_CMD_SET_FILTERS:
        engine().set_filters( without_request_id( b ) );
        return json::object();

    case IFS_CMD_RECORD_START:
        return record_start( msg );

    case IFS_CMD_RECORD_STOP:
        check_fields( b, { "request_id" } );
        return record_stop();

    case IFS_CMD_PLAYBACK:
    {
        if( mode_ != HelperMode::playback )
            fail( "playback control is only available for recordings" );
        check_fields( b, { "request_id", "action", "real_time" } );
        const std::string action = get_string( b, "action" );
        if( action != "pause" && action != "resume" )
            fail( "action must be 'pause' or 'resume'" );
        bool real_time = playback_real_time_;
        auto it = b.find( "real_time" );
        if( it != b.end() && ! it->is_null() )
        {
            if( ! it->is_boolean() )
                fail( "field 'real_time' must be a boolean" );
            real_time = it->get< bool >();
        }
        engine().playback_control( action == "pause", real_time );
        if( action == "resume" )
            playback_real_time_ = real_time;
        return json::object();
    }

    case IFS_CMD_HARDWARE_RESET:
        check_fields( b, { "request_id" } );
        if( mode_ == HelperMode::playback )
            fail( "a recording cannot be reset" );
        engine().hardware_reset();
        return json::object();

    case IFS_CMD_QUIT:
        // Never refused: ending the session must always be possible.
        quit_ = true;
        return json::object();
    }
    fail( "unknown command " + std::to_string( msg.type ) );
}

json HelperService::record_start( InMessage & msg )
{
    UniqueFd destination = std::move( msg.fd );  // closed on every error path
    if( mode_ == HelperMode::playback )
        fail( "recording is only available with the live camera" );
    check_fields( msg.body, { "request_id", "format" } );
    if( ! destination.valid() )
        fail( "RECORD_START needs the descriptor of the destination file" );
    std::string format = "db3";
    auto it = msg.body.find( "format" );
    if( it != msg.body.end() && ! it->is_null() )
        format = get_string( msg.body, "format" );
    if( format != "db3" && format != "bag" )
        fail( "unsupported recording format '" + format + "' (db3 or bag)" );
    if( recording_ )
        fail( "a recording is already running" );
    const std::string problem = check_recording_destination( destination.get() );
    if( ! problem.empty() )
        fail( problem );

    CaptureEngine & e = engine();
    std::unique_ptr< RecordingSpool > spool = RecordingSpool::create( format );
    e.start_recording( spool->file_path(), format );
    recording_.reset( new Recording{ std::move( spool ), std::move( destination ) } );
    connection_.send( IFS_MSG_EVENT, event_body( "recording_started", "", json::object() ) );
    return json::object();
}

json HelperService::record_stop()
{
    if( ! recording_ )
        fail( "no recording is running" );
    std::unique_ptr< Recording > rec = std::move( recording_ );
    std::string error;
    try
    {
        engine().stop_recording();
    }
    catch( const std::exception & e )
    {
        error = std::string( "stopping the recording failed: " ) + e.what();
    }
    // Copied even after a failed stop: a partial recording is better than none.
    uint64_t bytes = 0;
    try
    {
        bytes = rec->spool->copy_into( rec->destination.get() );
    }
    catch( const std::exception & e )
    {
        if( error.empty() )
            error = e.what();
    }
    if( rec->destination.reset() != 0 && error.empty() )
        error = std::string( "closing the recording failed: " ) + std::strerror( errno );
    rec->spool->remove();
    if( ! error.empty() )
        fail( error );
    connection_.send( IFS_MSG_EVENT, event_body( "recording_finished", "", { { "bytes", bytes } } ) );
    return { { "bytes", bytes } };
}

void HelperService::shutdown_engine()
{
    if( recording_ )
    {
        try
        {
            json r = record_stop();
            log( "info", "recording delivered before shutdown (" + r["bytes"].dump() + " bytes)" );
        }
        catch( const std::exception & e )
        {
            log( "error", std::string( "recording lost at shutdown: " ) + e.what() );
        }
    }
    if( engine_ )
    {
        try
        {
            engine_->shutdown();
        }
        catch( const std::exception & e )
        {
            log( "warning", std::string( "camera shutdown: " ) + e.what() );
        }
        engine_.reset();
    }
}

}  // namespace ifs
