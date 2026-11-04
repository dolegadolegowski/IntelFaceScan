// License: Apache 2.0. See LICENSE file in root directory.
//
// librealsense implementation of CaptureEngine (see capture_engine.h and docs/CAMERA.md).
//
// Live and playback share one streaming path: the depth, color and motion sensors are opened directly instead of
// through rs2::pipeline, so the process holds exactly one device instance (the patched SDK captures the camera
// from macOS once; every additional instance would open more handles to it) and recording/playback go through the
// same sensors. Depth and color feed an rs2::syncer with a small queue that the processing thread drains; the
// motion sensor's callback only stores the newest IMU sample. SDK callback threads never wait for processing or
// IPC, except in non-real-time playback, where blocking is the intended backpressure.
//
// Locking: every public method holds m_mutex. The processing thread and SDK callbacks never take it; they share
// only ProcessingControl, ImuState, the Session flags and the DeviceWatch, each with its own synchronization.

#include "capture_engine.h"
#include "capture_config.h"
#include "capture_processing.h"
#include "capture_testing.h"
#include "capture_util.h"

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <sys/stat.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

using nlohmann::json;

namespace ifs
{
namespace
{
using steady = std::chrono::steady_clock;

// Enabling advanced mode or a hardware reset makes the D455 re-enumerate, which takes 3-8 s on macOS.
constexpr auto k_reenumeration_timeout = std::chrono::seconds( 20 );
// Synchronized framesets waiting for the processing thread; older ones are dropped (live) or the playback waits.
constexpr unsigned k_sync_queue_size = 2;
constexpr unsigned k_wait_ms = 50;
constexpr auto k_frame_timeout = std::chrono::seconds( 5 );
// Assumed face distance when the app does not know it yet (recommended working distance, docs/CAMERA.md).
constexpr float k_default_roi_distance_m = 0.5f;
constexpr const char * k_d455_product_id = "0B5C";

void emit_event( const CaptureCallbacks & cb, const std::string & event, const json & data = json::object() ) noexcept
{
    if( ! cb.on_event )
        return;
    try
    {
        cb.on_event( event, data );
    }
    catch( ... )
    {
    }
}

void emit_log( const CaptureCallbacks & cb, const char * level, const std::string & message ) noexcept
{
    if( ! cb.on_log )
        return;
    try
    {
        cb.on_log( level, message );
    }
    catch( ... )
    {
    }
}

std::string format_value( float v )
{
    std::ostringstream os;
    os << v;
    return os.str();
}

bool has_extension( const std::string & path, const std::string & ext )
{
    return path.size() > ext.size() && path.compare( path.size() - ext.size(), ext.size(), ext ) == 0;
}

uint64_t file_size( const std::string & path )
{
    struct stat st;
    if( ::stat( path.c_str(), &st ) != 0 )
        return 0;
    return uint64_t( st.st_size );
}

// "848x480@30/60/90" style summary of what a sensor offers for one stream and format.
std::string available_modes( const std::vector< rs2::stream_profile > & profiles, rs2_stream stream, rs2_format format )
{
    std::map< std::pair< int, int >, std::set< int > > modes;
    for( auto & p : profiles )
    {
        auto v = p.as< rs2::video_stream_profile >();
        if( v && p.stream_type() == stream && p.format() == format )
            modes[{ v.width(), v.height() }].insert( p.fps() );
    }
    std::string out;
    for( auto it = modes.rbegin(); it != modes.rend(); ++it )
    {
        if( ! out.empty() )
            out += ", ";
        out += std::to_string( it->first.first ) + "x" + std::to_string( it->first.second ) + "@";
        bool first = true;
        for( int fps : it->second )
        {
            out += ( first ? "" : "/" ) + std::to_string( fps );
            first = false;
        }
    }
    return out.empty() ? "none" : out;
}

rs2::stream_profile find_video_profile( const std::vector< rs2::stream_profile > & profiles, rs2_stream stream,
                                        int index, rs2_format format, const StreamConfig & want )
{
    for( auto & p : profiles )
    {
        auto v = p.as< rs2::video_stream_profile >();
        if( v && p.stream_type() == stream && ( index < 0 || p.stream_index() == index ) && p.format() == format
            && v.width() == want.width && v.height() == want.height && p.fps() == want.fps )
            return p;
    }
    return rs2::stream_profile();
}

std::string describe( const StreamConfig & s )
{
    return std::to_string( s.width ) + "x" + std::to_string( s.height ) + "@" + std::to_string( s.fps );
}

rs2_intrinsics intrinsics_of( const rs2::stream_profile & p )
{
    return p.as< rs2::video_stream_profile >().get_intrinsics();
}

// Tracks the camera in use through the context's devices-changed callback (an SDK thread).
struct DeviceWatch
{
    std::mutex mutex;
    std::condition_variable cv;
    std::string serial;
    rs2::device current;   // to recognize its removal
    rs2::device arrived;   // newest instance with 'serial' that showed up after a removal
    bool lost = false;     // 'current' is gone
    bool expected = false; // we asked the camera to re-enumerate (advanced mode, hardware reset)
    bool closed = false;
};

struct StreamSelection
{
    rs2::sensor depth_sensor;
    rs2::stream_profile depth_profile;
    std::vector< rs2::stream_profile > depth_profiles;  // depth, plus infrared 1 when enabled
    rs2::sensor color_sensor;
    rs2::stream_profile color_profile;
    rs2::sensor motion_sensor;                          // empty without IMU
    std::vector< rs2::stream_profile > motion_profiles;
};

// One streaming run: from start() to stop() (or the end of a playback).
struct Session
{
    StreamSelection streams;
    rs2_extrinsics depth_to_color = identity_extrinsics();
    rs2::syncer syncer{ k_sync_queue_size };
    std::thread worker;
    bool playback = false;
    bool depth_open = false, color_open = false, motion_open = false;
    bool depth_started = false, color_started = false, motion_started = false;
    std::atomic< bool > user_stopping{ false };  // a STOPPED playback status is then ours, not the end of the file
    std::atomic< bool > playing{ false };        // PLAYING was reported for this session (older STOPPEDs are stale)
    std::atomic< bool > stop_requested{ false };
    std::atomic< bool > playback_ended{ false };
    std::atomic< bool > finished{ false };       // the worker reported the end of the playback
};

// State the processing thread shares with the engine for its whole lifetime.
struct Shared
{
    CaptureCallbacks callbacks;
    std::shared_ptr< ProcessingControl > processing = std::make_shared< ProcessingControl >();
    std::shared_ptr< ImuState > imu = std::make_shared< ImuState >();
    std::atomic< uint64_t > next_sequence{ 1 };
};

struct PlaybackStatus
{
    std::mutex mutex;
    std::weak_ptr< Session > session;
};

void run_worker( std::shared_ptr< Session > session, std::shared_ptr< Shared > shared )
{
    const CaptureCallbacks & cb = shared->callbacks;
    FrameProcessor processor( session->depth_to_color );
    FrameCounterMonitor depth_counter, color_counter;
    auto last_frame = steady::now();
    bool warned_no_frames = false;
    uint64_t errors = 0;

    std::vector< std::pair< rs2_stream, int > > expected;
    for( auto & p : session->streams.depth_profiles )
        expected.emplace_back( p.stream_type(), p.stream_index() );
    const rs2::stream_profile & color = session->streams.color_profile;
    expected.emplace_back( color.stream_type(), color.stream_index() );
    FramesetPairer pairer( expected );

    auto handle = [&]( const rs2::frameset & synced ) {
        try
        {
            rs2::frameset frames = pairer.pair( synced );
            if( ! frames )
                return;
            rs2::frame depth = frames.get_depth_frame();
            rs2::frame color = frames.get_color_frame();
            if( depth )
            {
                auto previous = depth_counter.last();
                if( depth_counter.update( depth.get_frame_number() ) )
                {
                    processor.reset_history();
                    emit_event( cb, "stream_restarted",
                                { { "stream", "depth" },
                                  { "previous_frame", *previous },
                                  { "frame", depth.get_frame_number() } } );
                }
            }
            if( color )
            {
                auto previous = color_counter.last();
                if( color_counter.update( color.get_frame_number() ) )
                    emit_event( cb, "stream_restarted",
                                { { "stream", "color" },
                                  { "previous_frame", *previous },
                                  { "frame", color.get_frame_number() } } );
            }
            auto out = processor.process( frames, *shared->processing, shared->imu->snapshot(),
                                          shared->next_sequence.load() );
            if( out )
            {
                shared->next_sequence.fetch_add( 1 );
                if( cb.on_frameset )
                    cb.on_frameset( std::move( *out ) );
            }
        }
        catch( const std::exception & e )
        {
            // A broken frame must not end the stream; repeated failures are logged sparsely.
            if( errors++ % 100 == 0 )
                emit_log( cb, "error", std::string( "Frame processing failed: " ) + e.what() );
        }
    };

    while( ! session->stop_requested.load() )
    {
        rs2::frameset frames;
        bool got = false;
        try
        {
            got = session->syncer.try_wait_for_frames( &frames, k_wait_ms );
        }
        catch( const std::exception & e )
        {
            emit_log( cb, "error", std::string( "Frame synchronization failed: " ) + e.what() );
        }
        if( got )
        {
            last_frame = steady::now();
            warned_no_frames = false;
            handle( frames );
            continue;
        }
        if( session->playback_ended.load() )
        {
            // The playback hands every frame to the syncer before it reports the end of the file.
            while( session->syncer.poll_for_frames( &frames ) )
                handle( frames );
            session->finished = true;
            emit_event( cb, "playback_finished" );
            emit_event( cb, "streaming_stopped" );
            return;
        }
        if( ! session->playback && ! warned_no_frames && steady::now() - last_frame > k_frame_timeout )
        {
            warned_no_frames = true;
            emit_log( cb, "warning", "No synchronized depth+color frames for 5 s (USB bandwidth or cable?)" );
        }
    }
}

class RealSenseEngine final : public CaptureEngine
{
public:
    RealSenseEngine( CaptureCallbacks callbacks, bool playback )
        : m_playback_mode( playback )
        , m_shared( std::make_shared< Shared >() )
        , m_watch( std::make_shared< DeviceWatch >() )
        , m_playback_status( std::make_shared< PlaybackStatus >() )
    {
        m_shared->callbacks = std::move( callbacks );
        m_shared->processing->set( { m_config.filters, m_config.send_depth, m_config.send_ir } );
    }

    ~RealSenseEngine() override
    {
        try
        {
            shutdown();
        }
        catch( ... )
        {
        }
    }

    void open_live();
    void open_device( const rs2::device & device );
    void open_playback( const std::string & file, bool real_time );

    json device_info() override;
    void configure( const json & configuration ) override;
    void set_filters( const json & filters ) override;
    void start() override;
    void stop() override;
    bool streaming() const override;
    void set_option( const std::string & sensor, const std::string & option, float value ) override;
    void set_auto_exposure_roi( int x, int y, int width, int height, float distance_m ) override;
    void start_recording( const std::string & path, const std::string & format ) override;
    void stop_recording() override;
    void playback_control( bool pause, bool real_time ) override;
    void hardware_reset() override;
    void shutdown() override;

private:
    struct OptionPlan
    {
        std::optional< float > preset;
        std::optional< float > depth_units;
        std::optional< int > disparity_shift;
        std::optional< float > emitter;
        std::optional< float > laser_power;
        std::optional< float > depth_auto_exposure;
        std::optional< float > color_auto_exposure;
    };

    const CaptureCallbacks & callbacks() const { return m_shared->callbacks; }
    void log( const char * level, const std::string & message ) const { emit_log( callbacks(), level, message ); }

    void check_open_locked() const;
    void open_device_locked( const rs2::device & device );
    void ensure_device_locked();
    void adopt_device_locked( const rs2::device & device );
    void release_device_locked();
    void expect_reenumeration_locked();
    rs2::device wait_for_reenumeration_locked( const char * what );
    void install_device_callback_locked();

    rs2::sensor find_sensor_locked( const std::string & kind ) const;
    StreamSelection select_streams_locked( const CaptureConfig & config ) const;
    bool session_active_locked() const;
    void start_session_locked( bool emit );
    void teardown_session_locked( bool emit );

    bool needs_advanced_mode_locked( const CaptureConfig & config ) const;
    void enable_advanced_mode_locked();
    OptionPlan plan_options_locked( const CaptureConfig & config ) const;
    void apply_options_locked( const OptionPlan & plan );
    void apply_configuration_locked( const CaptureConfig & config );
    void apply_roi_locked();
    uint64_t finish_recording_locked();

    const bool m_playback_mode;
    std::shared_ptr< Shared > m_shared;
    std::shared_ptr< DeviceWatch > m_watch;
    std::shared_ptr< PlaybackStatus > m_playback_status;

    mutable std::mutex m_mutex;
    bool m_shut_down = false;
    std::unique_ptr< rs2::context > m_ctx;
    rs2::device m_device;
    std::optional< rs2::playback > m_playback;
    bool m_playback_paused = false;
    std::string m_serial;
    CaptureConfig m_config;
    bool m_configured = false;
    std::shared_ptr< Session > m_session;
    std::unique_ptr< rs2::recorder > m_recorder;
    std::string m_recording_path;
    std::optional< PixelRect > m_roi;
    float m_roi_distance_m = 0;
};

void RealSenseEngine::check_open_locked() const
{
    if( m_shut_down )
        throw std::runtime_error( "The camera engine was shut down" );
}

void RealSenseEngine::open_live()
{
    std::lock_guard< std::mutex > lock( m_mutex );
    m_ctx = std::make_unique< rs2::context >();
    rs2::device_list list = m_ctx->query_devices();

    rs2::device chosen;
    int chosen_rank = -1;
    bool recovery = false;
    std::string open_error;
    for( uint32_t i = 0; i < list.size(); ++i )
    {
        rs2::device d;
        try
        {
            d = list[i];
        }
        catch( const rs2::error & e )
        {
            // e.g. not running as root: libusb lists the camera but cannot take it from the macOS drivers
            open_error = e.what();
            log( "warning", "Skipping a RealSense device that could not be opened: " + open_error );
            continue;
        }
        if( d.is< rs2::update_device >() )
        {
            recovery = true;
            continue;
        }
        // The D455 is the camera this app is tuned for; other D400 models work with the same code path.
        int rank = 0;
        if( camera_info( d, RS2_CAMERA_INFO_PRODUCT_ID ) == k_d455_product_id )
            rank = 2;
        else if( camera_info( d, RS2_CAMERA_INFO_PRODUCT_LINE ) == "D400" )
            rank = 1;
        if( rank > chosen_rank )
        {
            chosen = d;
            chosen_rank = rank;
        }
    }
    if( ! chosen )
    {
        if( recovery )
            throw std::runtime_error( "The RealSense camera is in firmware recovery mode. Update its firmware with "
                                      "rs-fw-update (or the RealSense Viewer) and reconnect it." );
        if( ! open_error.empty() )
            throw std::runtime_error( "A RealSense camera is connected but could not be opened: " + open_error );
        throw std::runtime_error( "No Intel RealSense camera found. Connect the D455 directly to a USB 3 port of the "
                                  "Mac with a USB-C cable and try again." );
    }

    open_device_locked( chosen );
}

void RealSenseEngine::open_device( const rs2::device & device )
{
    std::lock_guard< std::mutex > lock( m_mutex );
    m_ctx = std::make_unique< rs2::context >();
    open_device_locked( device );
}

void RealSenseEngine::open_device_locked( const rs2::device & device )
{
    adopt_device_locked( device );
    install_device_callback_locked();

    std::string firmware = camera_info( m_device, RS2_CAMERA_INFO_FIRMWARE_VERSION );
    std::string recommended = camera_info( m_device, RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION );
    std::string usb = camera_info( m_device, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR );
    log( "info", "Using " + camera_info( m_device, RS2_CAMERA_INFO_NAME ) + " (serial " + m_serial + ", firmware "
                     + firmware + ", USB " + usb + ")" );
    if( ! usb.empty() && usb[0] < '3' )
        log( "warning", "The camera is connected over USB " + usb
                            + ": high resolutions are not available. Use a USB 3 port and cable." );
    if( ! recommended.empty() && ! firmware.empty() && firmware != recommended )
        log( "info", "Camera firmware " + firmware + ", recommended " + recommended );
}

void RealSenseEngine::open_playback( const std::string & file, bool real_time )
{
    std::lock_guard< std::mutex > lock( m_mutex );
    struct stat st;
    if( ::stat( file.c_str(), &st ) != 0 || ! S_ISREG( st.st_mode ) )
        throw std::runtime_error( "Recording not found: " + file );
    if( ! has_extension( file, ".db3" ) && ! has_extension( file, ".bag" ) )
        throw std::runtime_error( "Unsupported recording (expected a .db3 or .bag file): " + file );

    m_ctx = std::make_unique< rs2::context >();
    rs2::playback playback = m_ctx->load_device( file );
    playback.set_real_time( real_time );
    m_playback = playback;
    m_device = playback;
    m_serial = camera_info( playback, RS2_CAMERA_INFO_SERIAL_NUMBER );

    auto status = m_playback_status;
    playback.set_status_changed_callback( [status]( rs2_playback_status s ) {
        std::lock_guard< std::mutex > lock( status->mutex );
        auto session = status->session.lock();
        if( ! session )
            return;
        // The STOPPED of a previous session's stop() can arrive after the next session registered; only a stop
        // after this session's own PLAYING is the end of the file.
        if( s == RS2_PLAYBACK_STATUS_PLAYING )
            session->playing = true;
        else if( s == RS2_PLAYBACK_STATUS_STOPPED && session->playing.load() && ! session->user_stopping.load() )
            session->playback_ended = true;
    } );

    // A recording plays with the face-scan defaults until the app configures something else.
    m_configured = true;
    double seconds = std::chrono::duration< double >( playback.get_duration() ).count();
    log( "info", "Playing " + file + " (" + format_value( float( seconds ) ) + " s, "
                     + ( real_time ? "real time" : "every frame" ) + ")" );
}

// Device objects replaced in the watch are destroyed only after its mutex is released: destroying a device runs
// librealsense code that must not wait on the devices-changed thread while that thread waits for the mutex.

void RealSenseEngine::adopt_device_locked( const rs2::device & device )
{
    m_device = device;
    m_serial = camera_info( device, RS2_CAMERA_INFO_SERIAL_NUMBER );
    rs2::device old_current, old_arrived;
    std::lock_guard< std::mutex > lock( m_watch->mutex );
    old_current = m_watch->current;
    old_arrived = m_watch->arrived;
    m_watch->serial = m_serial;
    m_watch->current = device;
    m_watch->arrived = rs2::device();
    m_watch->lost = false;
    m_watch->expected = false;
}

void RealSenseEngine::release_device_locked()
{
    rs2::device old;
    {
        std::lock_guard< std::mutex > lock( m_watch->mutex );
        old = m_watch->current;
        m_watch->current = rs2::device();
    }
    m_device = rs2::device();
}

void RealSenseEngine::install_device_callback_locked()
{
    auto watch = m_watch;
    CaptureCallbacks cb = callbacks();
    m_ctx->set_devices_changed_callback( [watch, cb]( rs2::event_information & info ) {
        // librealsense is called without holding the watch's mutex (the engine's thread takes it while it calls
        // librealsense), the results are applied under it.
        rs2::device current;
        std::string serial;
        {
            std::lock_guard< std::mutex > lock( watch->mutex );
            if( watch->closed )
                return;
            current = watch->current;
            serial = watch->serial;
        }
        bool removed = false;
        rs2::device match;
        try
        {
            removed = current && info.was_removed( current );
            rs2::device_list added = info.get_new_devices();
            for( uint32_t i = 0; i < added.size(); ++i )
            {
                rs2::device d = added[i];
                if( ! d.is< rs2::update_device >() && camera_info( d, RS2_CAMERA_INFO_SERIAL_NUMBER ) == serial )
                    match = d;
            }
        }
        catch( const std::exception & )
        {
            // A device that vanishes again while being inspected; the next event tells the rest.
        }
        bool disconnected = false, connected = false;
        rs2::device replaced;
        {
            std::lock_guard< std::mutex > lock( watch->mutex );
            if( watch->closed )
                return;
            if( removed && ! watch->lost && watch->current.get() == current.get() )
            {
                watch->lost = true;
                disconnected = ! watch->expected;
            }
            if( match )
            {
                replaced = watch->arrived;
                watch->arrived = match;
                connected = watch->lost && ! watch->expected;
            }
            watch->cv.notify_all();
        }
        if( disconnected )
            emit_event( cb, "device_disconnected",
                        { { "serial", serial }, { "message", "The RealSense camera was disconnected" } } );
        if( connected )
            emit_event( cb, "device_connected", { { "serial", serial } } );
    } );
}

void RealSenseEngine::expect_reenumeration_locked()
{
    rs2::device old;
    std::lock_guard< std::mutex > lock( m_watch->mutex );
    old = m_watch->arrived;
    m_watch->expected = true;
    m_watch->lost = false;
    m_watch->arrived = rs2::device();
}

rs2::device RealSenseEngine::wait_for_reenumeration_locked( const char * what )
{
    rs2::device arrived;
    {
        std::unique_lock< std::mutex > lock( m_watch->mutex );
        // expect_reenumeration_locked() cleared 'arrived', so any instance with our serial is the camera coming
        // back (the watcher may report the removal and the arrival in one event, or miss the removal).
        m_watch->cv.wait_for( lock, k_reenumeration_timeout,
                              [&] { return m_watch->closed || bool( m_watch->arrived ); } );
        arrived = m_watch->arrived;
    }
    if( ! arrived )
    {
        // The device watcher polls; a re-enumeration it did not see as remove+add still shows in a fresh query.
        try
        {
            rs2::device_list list = m_ctx->query_devices();
            for( uint32_t i = 0; i < list.size() && ! arrived; ++i )
            {
                rs2::device d = list[i];
                if( camera_info( d, RS2_CAMERA_INFO_SERIAL_NUMBER ) == m_serial && ! d.is< rs2::update_device >() )
                    arrived = d;
            }
        }
        catch( const rs2::error & )
        {
        }
    }
    if( ! arrived )
        throw std::runtime_error( std::string( "The camera did not come back within 20 s after " ) + what
                                  + ". Unplug it, plug it in again and reconnect." );
    return arrived;
}

void RealSenseEngine::ensure_device_locked()
{
    check_open_locked();
    if( m_playback_mode )
        return;
    rs2::device arrived;
    bool lost;
    {
        std::lock_guard< std::mutex > lock( m_watch->mutex );
        lost = m_watch->lost && ! m_watch->expected;
        arrived = m_watch->arrived;
    }
    if( ! lost && m_device )
        return;

    // The camera disappeared: whatever streamed or recorded on the old instance is over.
    if( m_session )
        teardown_session_locked( false );
    if( m_recorder )
    {
        try
        {
            finish_recording_locked();
        }
        catch( const std::exception & e )
        {
            log( "error", std::string( "Recording ended with the disconnected camera: " ) + e.what() );
        }
    }
    if( ! arrived )
        throw std::runtime_error( "The RealSense camera was disconnected. Reconnect it and try again." );
    release_device_locked();
    adopt_device_locked( arrived );
    log( "info", "The camera is back (serial " + m_serial + ")" );
    // Advanced mode survives a power cycle, presets and options do not.
    if( m_configured )
        apply_configuration_locked( m_config );
}

rs2::sensor RealSenseEngine::find_sensor_locked( const std::string & kind ) const
{
    if( ! m_device )
        return rs2::sensor();
    for( auto & s : m_device.query_sensors() )
        if( sensor_kind( s ) == kind )
            return s;
    // Recordings of devices that did not snapshot the sensor extensions: recognize the sensor by its streams.
    rs2_stream stream = kind == "depth" ? RS2_STREAM_DEPTH : kind == "color" ? RS2_STREAM_COLOR : RS2_STREAM_ACCEL;
    for( auto & s : m_device.query_sensors() )
        for( auto & p : s.get_stream_profiles() )
            if( p.stream_type() == stream )
                return s;
    return rs2::sensor();
}

StreamSelection RealSenseEngine::select_streams_locked( const CaptureConfig & config ) const
{
    StreamSelection sel;
    sel.depth_sensor = find_sensor_locked( "depth" );
    sel.color_sensor = find_sensor_locked( "color" );
    if( ! sel.depth_sensor )
        throw std::runtime_error( m_playback_mode ? "The recording has no depth stream"
                                                  : "The camera has no depth sensor" );
    if( ! sel.color_sensor )
        throw std::runtime_error( m_playback_mode ? "The recording has no color stream"
                                                  : "The camera has no RGB sensor" );
    auto depth_profiles = sel.depth_sensor.get_stream_profiles();
    auto color_profiles = sel.color_sensor.get_stream_profiles();

    if( m_playback_mode )
    {
        // A recording plays the streams it contains; the requested ones are preferred when it has several.
        sel.depth_profile = find_video_profile( depth_profiles, RS2_STREAM_DEPTH, -1, RS2_FORMAT_Z16, config.depth );
        for( auto & p : depth_profiles )
            if( ! sel.depth_profile && p.stream_type() == RS2_STREAM_DEPTH && p.format() == RS2_FORMAT_Z16 )
                sel.depth_profile = p;
        if( ! sel.depth_profile )
            throw std::runtime_error( "The recording has no Z16 depth stream" );
        sel.color_profile = find_video_profile( color_profiles, RS2_STREAM_COLOR, -1, RS2_FORMAT_RGB8, config.color );
        for( auto & p : color_profiles )
            if( ! sel.color_profile && p.stream_type() == RS2_STREAM_COLOR && p.format() == RS2_FORMAT_RGB8 )
                sel.color_profile = p;
        if( ! sel.color_profile )
            throw std::runtime_error( "The recording has no RGB8 color stream" );
        sel.depth_profiles.push_back( sel.depth_profile );
        if( config.enable_ir )
            for( auto & p : depth_profiles )
                if( p.stream_type() == RS2_STREAM_INFRARED && p.stream_index() == 1 && p.format() == RS2_FORMAT_Y8 )
                {
                    sel.depth_profiles.push_back( p );
                    break;
                }
    }
    else
    {
        std::string usb = camera_info( m_device, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR, "?" );
        sel.depth_profile = find_video_profile( depth_profiles, RS2_STREAM_DEPTH, -1, RS2_FORMAT_Z16, config.depth );
        if( ! sel.depth_profile )
            throw std::runtime_error( "Depth " + describe( config.depth ) + " Z16 is not available (USB " + usb
                                      + "). Available: "
                                      + available_modes( depth_profiles, RS2_STREAM_DEPTH, RS2_FORMAT_Z16 ) );
        sel.depth_profiles.push_back( sel.depth_profile );
        if( config.enable_ir )
        {
            // The infrared stream must run with the depth stream's resolution and frame rate.
            auto ir = find_video_profile( depth_profiles, RS2_STREAM_INFRARED, 1, RS2_FORMAT_Y8, config.depth );
            if( ! ir )
                throw std::runtime_error( "Infrared " + describe( config.depth ) + " Y8 is not available (USB " + usb
                                          + ")" );
            sel.depth_profiles.push_back( ir );
        }
        sel.color_profile = find_video_profile( color_profiles, RS2_STREAM_COLOR, -1, RS2_FORMAT_RGB8, config.color );
        if( ! sel.color_profile )
            throw std::runtime_error( "Color " + describe( config.color ) + " RGB8 is not available (USB " + usb
                                      + "). Available: "
                                      + available_modes( color_profiles, RS2_STREAM_COLOR, RS2_FORMAT_RGB8 ) );
    }

    if( config.enable_imu )
    {
        if( auto motion = find_sensor_locked( "motion" ) )
        {
            // Only the newest sample is used (camera motion check), so the lowest rate of each stream is enough.
            rs2::stream_profile accel, gyro;
            for( auto & p : motion.get_stream_profiles() )
            {
                if( p.format() != RS2_FORMAT_MOTION_XYZ32F )
                    continue;
                if( p.stream_type() == RS2_STREAM_ACCEL && ( ! accel || p.fps() < accel.fps() ) )
                    accel = p;
                if( p.stream_type() == RS2_STREAM_GYRO && ( ! gyro || p.fps() < gyro.fps() ) )
                    gyro = p;
            }
            if( accel )
                sel.motion_profiles.push_back( accel );
            if( gyro )
                sel.motion_profiles.push_back( gyro );
            if( ! sel.motion_profiles.empty() )
                sel.motion_sensor = motion;
        }
    }
    return sel;
}

bool RealSenseEngine::session_active_locked() const
{
    if( ! m_session || m_session->finished.load() )
        return false;
    if( m_playback_mode )
        return true;
    std::lock_guard< std::mutex > lock( m_watch->mutex );
    return ! ( m_watch->lost && ! m_watch->expected );
}

void RealSenseEngine::start_session_locked( bool emit )
{
    auto session = std::make_shared< Session >();
    session->playback = m_playback_mode;
    session->streams = select_streams_locked( m_config );
    try
    {
        session->depth_to_color = session->streams.depth_profile.get_extrinsics_to( session->streams.color_profile );
    }
    catch( const rs2::error & e )
    {
        log( "warning", std::string( "No depth-to-color extrinsics, assuming identity: " ) + e.what() );
    }
    m_shared->imu->reset();

    if( m_playback_mode )
    {
        std::lock_guard< std::mutex > lock( m_playback_status->mutex );
        m_playback_status->session = session;
    }

    // The worker runs before the first frame can arrive: frames must never pile up in the SDK.
    session->worker = std::thread( run_worker, session, m_shared );
    m_session = session;
    StreamSelection & st = session->streams;
    try
    {
        // A playback starts reading as soon as one sensor starts; pausing until all are started keeps the first
        // frames of the later sensors (resume() rewinds to the last frame actually delivered).
        if( m_playback && ! m_playback_paused )
            m_playback->pause();

        st.color_sensor.open( st.color_profile );
        session->color_open = true;
        st.depth_sensor.open( st.depth_profiles );
        session->depth_open = true;
        if( st.motion_sensor )
        {
            st.motion_sensor.open( st.motion_profiles );
            session->motion_open = true;
        }

        // RGB starts first: the D4xx firmware restarts a running depth stream when RGB is switched on.
        st.color_sensor.start( session->syncer );
        session->color_started = true;
        st.depth_sensor.start( session->syncer );
        session->depth_started = true;
        if( st.motion_sensor )
        {
            auto imu = m_shared->imu;
            st.motion_sensor.start( [imu]( rs2::frame f ) { imu->on_frame( f ); } );
            session->motion_started = true;
        }

        if( m_playback && ! m_playback_paused )
            m_playback->resume();
    }
    catch( const std::exception & e )
    {
        std::string message = std::string( "Could not start streaming: " ) + e.what();
        teardown_session_locked( false );
        throw std::runtime_error( message );
    }

    if( ! m_playback_mode )
    {
        try
        {
            apply_roi_locked();
        }
        catch( const std::exception & e )
        {
            log( "warning", e.what() );
        }
    }

    auto v = st.depth_profile.as< rs2::video_stream_profile >();
    auto c = st.color_profile.as< rs2::video_stream_profile >();
    log( "info", "Streaming depth " + std::to_string( v.width() ) + "x" + std::to_string( v.height() ) + "@"
                     + std::to_string( v.fps() ) + ", color " + std::to_string( c.width() ) + "x"
                     + std::to_string( c.height() ) + "@" + std::to_string( c.fps() )
                     + ( st.depth_profiles.size() > 1 ? ", infrared" : "" )
                     + ( st.motion_sensor ? ", IMU" : "" ) );
    if( emit )
        emit_event( callbacks(), "streaming_started" );
}

void RealSenseEngine::teardown_session_locked( bool emit )
{
    auto session = std::move( m_session );
    if( ! session )
        return;
    bool was_running = ! session->finished.load();
    session->user_stopping = true;
    StreamSelection & st = session->streams;

    // The worker keeps draining while the sensors stop (a non-real-time playback blocks on a full syncer queue).
    // Depth stops before RGB so that the RGB toggle does not restart it.
    auto stop_sensor = [this]( rs2::sensor & s, bool & started ) {
        if( ! started )
            return;
        started = false;
        try
        {
            s.stop();
        }
        catch( const std::exception & e )
        {
            log( "debug", std::string( "Stopping a sensor: " ) + e.what() );
        }
    };
    auto close_sensor = [this]( rs2::sensor & s, bool & open ) {
        if( ! open )
            return;
        open = false;
        try
        {
            s.close();
        }
        catch( const std::exception & e )
        {
            log( "debug", std::string( "Closing a sensor: " ) + e.what() );
        }
    };
    stop_sensor( st.depth_sensor, session->depth_started );
    stop_sensor( st.color_sensor, session->color_started );
    stop_sensor( st.motion_sensor, session->motion_started );
    close_sensor( st.depth_sensor, session->depth_open );
    close_sensor( st.color_sensor, session->color_open );
    close_sensor( st.motion_sensor, session->motion_open );

    session->stop_requested = true;
    if( session->worker.joinable() )
        session->worker.join();
    if( m_playback_mode )
    {
        std::lock_guard< std::mutex > lock( m_playback_status->mutex );
        m_playback_status->session.reset();
    }
    if( emit && was_running )
        emit_event( callbacks(), "streaming_stopped" );
}

bool RealSenseEngine::needs_advanced_mode_locked( const CaptureConfig & config ) const
{
    rs400::advanced_mode adv( m_device );
    if( ! adv || adv.is_enabled() )
        return false;
    // Without advanced mode a D4xx has no visual presets, read-only depth units (1 mm) and no disparity shift.
    rs2::sensor depth = find_sensor_locked( "depth" );
    float units = depth && depth.supports( RS2_OPTION_DEPTH_UNITS ) ? depth.get_option( RS2_OPTION_DEPTH_UNITS )
                                                                     : 0.001f;
    bool units_locked = ! depth || ! depth.supports( RS2_OPTION_DEPTH_UNITS )
                     || depth.is_option_read_only( RS2_OPTION_DEPTH_UNITS );
    bool preset = ! iequals( config.preset, "Custom" ) && ( ! depth || ! depth.supports( RS2_OPTION_VISUAL_PRESET ) );
    return preset || ( units_locked && std::fabs( units - config.depth_units ) > 1e-9f ) || config.disparity_shift != 0;
}

void RealSenseEngine::enable_advanced_mode_locked()
{
    rs400::advanced_mode adv( m_device );
    if( ! adv )
        throw std::runtime_error( "This camera has no advanced mode (needed for visual presets, depth units and "
                                  "disparity shift)" );
    if( adv.is_enabled() )
        return;
    log( "info", "Enabling the camera's advanced mode; the camera restarts (a few seconds, only needed once)" );
    expect_reenumeration_locked();
    adv.toggle_advanced_mode( true );
    release_device_locked();
    adopt_device_locked( wait_for_reenumeration_locked( "enabling advanced mode" ) );
    rs400::advanced_mode again( m_device );
    if( ! again || ! again.is_enabled() )
        throw std::runtime_error( "The camera did not enable advanced mode" );
}

RealSenseEngine::OptionPlan RealSenseEngine::plan_options_locked( const CaptureConfig & config ) const
{
    OptionPlan plan;
    rs2::sensor depth = find_sensor_locked( "depth" );
    rs2::sensor color = find_sensor_locked( "color" );
    if( ! depth )
        throw std::runtime_error( "The camera has no depth sensor" );

    if( depth.supports( RS2_OPTION_VISUAL_PRESET ) )
    {
        auto range = depth.get_option_range( RS2_OPTION_VISUAL_PRESET );
        std::string available;
        for( float v = range.min; v <= range.max; v += std::max( range.step, 1.f ) )
        {
            const char * name = depth.get_option_value_description( RS2_OPTION_VISUAL_PRESET, v );
            if( ! name )
                continue;
            if( iequals( config.preset, name ) )
                plan.preset = v;
            available += ( available.empty() ? "" : ", " ) + std::string( name );
        }
        if( ! plan.preset )
            throw std::runtime_error( "Unknown visual preset '" + config.preset + "'. Available: " + available );
    }
    else if( ! iequals( config.preset, "Custom" ) )
        throw std::runtime_error( "This camera does not support visual presets ('" + config.preset + "')" );

    auto checked = [&]( const rs2::sensor & s, rs2_option option, float value ) {
        auto range = s.get_option_range( option );
        // Tolerance for values that went through float/JSON conversions (0.0001 is not exact in binary).
        float tolerance = std::max( std::fabs( range.step ) * 1e-3f, 1e-9f );
        if( value < range.min - tolerance || value > range.max + tolerance )
            throw std::runtime_error( std::string( rs2_option_to_string( option ) ) + ": value out of range ["
                                      + format_value( range.min ) + ", " + format_value( range.max ) + "]" );
        if( range.step > 0 && option != RS2_OPTION_DEPTH_UNITS )
            value = range.min + std::round( ( value - range.min ) / range.step ) * range.step;
        return std::clamp( value, range.min, range.max );
    };

    if( depth.supports( RS2_OPTION_DEPTH_UNITS ) && ! depth.is_option_read_only( RS2_OPTION_DEPTH_UNITS ) )
        plan.depth_units = checked( depth, RS2_OPTION_DEPTH_UNITS, config.depth_units );
    else
    {
        float units = depth.supports( RS2_OPTION_DEPTH_UNITS ) ? depth.get_option( RS2_OPTION_DEPTH_UNITS ) : 0.001f;
        if( std::fabs( units - config.depth_units ) > 1e-9f )
            throw std::runtime_error( "Depth Units cannot be changed on this camera (fixed at "
                                      + format_value( units ) + " m)" );
    }

    rs400::advanced_mode adv( m_device );
    if( adv && adv.is_enabled() )
        plan.disparity_shift = config.disparity_shift;
    else if( config.disparity_shift != 0 )
        throw std::runtime_error( "Disparity shift needs the camera's advanced mode, which is not available" );

    if( depth.supports( RS2_OPTION_EMITTER_ENABLED ) )
        plan.emitter = config.emitter_enabled ? 1.f : 0.f;
    else if( config.emitter_enabled )
        log( "warning", "This camera has no IR emitter control" );
    if( depth.supports( RS2_OPTION_LASER_POWER ) && ( config.emitter_enabled || config.laser_power ) )
    {
        auto range = depth.get_option_range( RS2_OPTION_LASER_POWER );
        plan.laser_power = checked( depth, RS2_OPTION_LASER_POWER, config.laser_power.value_or( range.max ) );
    }

    if( depth.supports( RS2_OPTION_ENABLE_AUTO_EXPOSURE ) )
        plan.depth_auto_exposure = config.depth_auto_exposure ? 1.f : 0.f;
    if( color && color.supports( RS2_OPTION_ENABLE_AUTO_EXPOSURE ) )
        plan.color_auto_exposure = config.color_auto_exposure ? 1.f : 0.f;
    return plan;
}

void RealSenseEngine::apply_options_locked( const OptionPlan & plan )
{
    rs2::sensor depth = find_sensor_locked( "depth" );
    rs2::sensor color = find_sensor_locked( "color" );
    auto set = []( rs2::sensor & s, rs2_option option, float value ) {
        try
        {
            s.set_option( option, value );
        }
        catch( const rs2::error & e )
        {
            throw std::runtime_error( std::string( "Could not set " ) + rs2_option_to_string( option ) + " to "
                                      + format_value( value ) + ": " + e.what() );
        }
    };

    // Order of docs/PROTOCOL.md: a preset rewrites the depth table, so it comes first; the laser power only
    // applies while the emitter is on.
    if( plan.preset )
        set( depth, RS2_OPTION_VISUAL_PRESET, *plan.preset );
    if( plan.depth_units )
        set( depth, RS2_OPTION_DEPTH_UNITS, *plan.depth_units );  // through the option: keeps the depth scale in sync
    if( plan.disparity_shift )
    {
        rs400::advanced_mode adv( m_device );
        STDepthTableControl table = adv.get_depth_table();
        if( table.disparityShift != *plan.disparity_shift )
        {
            table.disparityShift = *plan.disparity_shift;
            adv.set_depth_table( table );
        }
    }
    if( plan.emitter && *plan.emitter > 0 )
        set( depth, RS2_OPTION_EMITTER_ENABLED, *plan.emitter );
    if( plan.laser_power )
        set( depth, RS2_OPTION_LASER_POWER, *plan.laser_power );
    if( plan.emitter && *plan.emitter == 0 )
        set( depth, RS2_OPTION_EMITTER_ENABLED, 0.f );
    if( plan.depth_auto_exposure )
        set( depth, RS2_OPTION_ENABLE_AUTO_EXPOSURE, *plan.depth_auto_exposure );
    if( plan.color_auto_exposure && color )
        set( color, RS2_OPTION_ENABLE_AUTO_EXPOSURE, *plan.color_auto_exposure );
}

void RealSenseEngine::apply_configuration_locked( const CaptureConfig & config )
{
    if( needs_advanced_mode_locked( config ) )
        enable_advanced_mode_locked();
    apply_options_locked( plan_options_locked( config ) );
}

void RealSenseEngine::apply_roi_locked()
{
    if( ! m_roi || ! session_active_locked() )
        return;
    StreamSelection & st = m_session->streams;
    rs2_intrinsics color = intrinsics_of( st.color_profile );
    rs2_intrinsics depth = intrinsics_of( st.depth_profile );

    auto set_roi = []( const rs2::sensor & s, const PixelRect & r, const char * which ) {
        rs2::roi_sensor roi( s );
        if( ! roi )
            return false;
        rs2::region_of_interest region{ r.x, r.y, r.x + r.width - 1, r.y + r.height - 1 };
        try
        {
            roi.set_region_of_interest( region );
        }
        catch( const rs2::error & e )
        {
            throw std::runtime_error( std::string( "Could not set the " ) + which
                                      + " auto-exposure region: " + e.what() );
        }
        return true;
    };

    if( auto r = clamp_rect( *m_roi, color.width, color.height ) )
        set_roi( st.color_sensor, *r, "RGB" );
    rs2_extrinsics color_to_depth = inverse( m_session->depth_to_color );
    float distance = m_roi_distance_m > 0 ? m_roi_distance_m : k_default_roi_distance_m;
    if( auto r = map_color_rect_to_depth( color, depth, color_to_depth, *m_roi, distance ) )
        set_roi( st.depth_sensor, *r, "depth" );
}

uint64_t RealSenseEngine::finish_recording_locked()
{
    bool restart = session_active_locked();
    if( m_session )
        teardown_session_locked( false );
    // The record device's destructor flushes the write queue and closes the file.
    m_recorder.reset();
    uint64_t bytes = file_size( m_recording_path );
    log( "info", "Recording finished (" + std::to_string( bytes ) + " bytes)" );
    emit_event( callbacks(), "recording_finished", { { "bytes", bytes } } );
    m_recording_path.clear();
    if( restart )
    {
        // The file is complete whatever happens now; a failed restart is reported, not thrown.
        try
        {
            start_session_locked( false );
        }
        catch( const std::exception & e )
        {
            log( "error", e.what() );
            emit_event( callbacks(), "error", { { "message", e.what() } } );
            emit_event( callbacks(), "streaming_stopped" );
        }
    }
    return bytes;
}

json RealSenseEngine::device_info()
{
    std::lock_guard< std::mutex > lock( m_mutex );
    ensure_device_locked();

    json info;
    info["name"] = camera_info( m_device, RS2_CAMERA_INFO_NAME, "Unknown" );
    info["serial"] = m_serial;
    info["firmware"] = camera_info( m_device, RS2_CAMERA_INFO_FIRMWARE_VERSION );
    std::string recommended = camera_info( m_device, RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION );
    info["recommended_firmware"] = recommended.empty() ? json( nullptr ) : json( recommended );
    info["usb_type"] = camera_info( m_device, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR );
    info["product_id"] = camera_info( m_device, RS2_CAMERA_INFO_PRODUCT_ID );
    info["product_line"] = camera_info( m_device, RS2_CAMERA_INFO_PRODUCT_LINE );
    rs400::advanced_mode adv( m_device );
    bool advanced = false;
    if( adv )
    {
        try
        {
            advanced = adv.is_enabled();
        }
        catch( const rs2::error & )
        {
        }
    }
    info["advanced_mode"] = advanced;
    info["source"] = m_playback_mode ? "playback" : "live";

    rs2::sensor depth = find_sensor_locked( "depth" );
    float depth_scale = 0.001f;
    if( auto ds = depth.as< rs2::depth_sensor >() )
        depth_scale = ds.get_depth_scale();
    else if( depth && depth.supports( RS2_OPTION_DEPTH_UNITS ) )
        depth_scale = depth.get_option( RS2_OPTION_DEPTH_UNITS );
    info["depth_scale"] = depth_scale;

    // Baseline from the factory extrinsics between the two imagers; the stereo extension as fallback.
    json baseline = nullptr;
    if( depth )
    {
        rs2::stream_profile left, right;
        for( auto & p : depth.get_stream_profiles() )
        {
            if( p.stream_type() == RS2_STREAM_INFRARED && p.stream_index() == 1 && ! left )
                left = p;
            if( p.stream_type() == RS2_STREAM_INFRARED && p.stream_index() == 2 && ! right )
                right = p;
        }
        try
        {
            if( left && right )
            {
                rs2_extrinsics e = left.get_extrinsics_to( right );
                double b = std::sqrt( double( e.translation[0] ) * e.translation[0]
                                      + double( e.translation[1] ) * e.translation[1]
                                      + double( e.translation[2] ) * e.translation[2] );
                if( b > 0 )
                    baseline = b * 1000.0;
            }
        }
        catch( const rs2::error & )
        {
        }
        if( baseline.is_null() )
        {
            try
            {
                if( auto stereo = depth.as< rs2::depth_stereo_sensor >() )
                    baseline = stereo.get_stereo_baseline();
                else if( depth.supports( RS2_OPTION_STEREO_BASELINE ) )
                    baseline = depth.get_option( RS2_OPTION_STEREO_BASELINE );
            }
            catch( const rs2::error & )
            {
            }
        }
    }
    info["baseline_mm"] = baseline;

    json presets = json::array();
    if( depth && depth.supports( RS2_OPTION_VISUAL_PRESET ) && ! m_playback_mode )
    {
        auto range = depth.get_option_range( RS2_OPTION_VISUAL_PRESET );
        for( float v = range.min; v <= range.max; v += std::max( range.step, 1.f ) )
            if( const char * name = depth.get_option_value_description( RS2_OPTION_VISUAL_PRESET, v ) )
                presets.push_back( name );
    }
    else if( adv && ! m_playback_mode )
    {
        // Presets become available when configure() enables advanced mode.
        for( int p = RS2_RS400_VISUAL_PRESET_CUSTOM; p <= RS2_RS400_VISUAL_PRESET_MEDIUM_DENSITY; ++p )
            presets.push_back( rs2_rs400_visual_preset_to_string( rs2_rs400_visual_preset( p ) ) );
    }
    info["presets"] = presets;

    json sensors = json::array();
    json profiles = json::array();
    bool imu = false;
    for( auto & s : m_device.query_sensors() )
    {
        std::string kind = sensor_kind( s );
        auto sensor_profiles = s.get_stream_profiles();
        if( kind.empty() )
        {
            for( auto & p : sensor_profiles )
            {
                if( p.stream_type() == RS2_STREAM_DEPTH )
                    kind = "depth";
                else if( p.stream_type() == RS2_STREAM_COLOR )
                    kind = "color";
                else if( p.stream_type() == RS2_STREAM_ACCEL || p.stream_type() == RS2_STREAM_GYRO )
                    kind = "motion";
                if( ! kind.empty() )
                    break;
            }
        }
        if( kind.empty() )
            continue;
        if( kind == "motion" )
            imu = true;
        std::string name = s.supports( RS2_CAMERA_INFO_NAME ) ? s.get_info( RS2_CAMERA_INFO_NAME ) : kind;
        sensors.push_back( { { "name", name }, { "kind", kind }, { "options", sensor_options_json( s ) } } );
        for( auto & p : sensor_profiles )
            profiles.push_back( profile_json( p ) );
    }
    info["imu"] = imu;
    info["sensors"] = sensors;
    info["profiles"] = profiles;
    return info;
}

void RealSenseEngine::configure( const json & configuration )
{
    CaptureConfig config = parse_capture_config( configuration );
    std::lock_guard< std::mutex > lock( m_mutex );
    ensure_device_locked();
    bool was_streaming = session_active_locked();

    if( m_playback_mode )
    {
        bool restart = was_streaming && ! config.same_streams( m_config );
        // Validate against the recording before changing anything.
        select_streams_locked( config );
        if( restart )
            teardown_session_locked( false );
        m_config = config;
        m_configured = true;
        m_shared->processing->set( { config.filters, config.send_depth, config.send_ir } );
        if( restart )
            start_session_locked( false );
        return;
    }

    try
    {
        bool restart = was_streaming && ( ! m_configured || ! config.same_streams( m_config ) );
        // A recording keeps the streams it started with (a file with changing stream profiles does not play back).
        if( m_recorder && m_configured && ! config.same_streams( m_config ) )
            throw std::runtime_error( "Stop the recording before changing the streams" );
        select_streams_locked( config );
        if( needs_advanced_mode_locked( config ) )
        {
            if( m_recorder )
                throw std::runtime_error( "Stop the recording first: this configuration needs the camera's advanced "
                                          "mode, and enabling it restarts the camera" );
            teardown_session_locked( false );
            enable_advanced_mode_locked();
            select_streams_locked( config );
        }
        OptionPlan plan = plan_options_locked( config );
        if( restart )
            teardown_session_locked( false );
        apply_options_locked( plan );
        m_config = config;
        m_configured = true;
        m_shared->processing->set( { config.filters, config.send_depth, config.send_ir } );
        if( was_streaming && ! m_session )
            start_session_locked( false );
    }
    catch( ... )
    {
        if( was_streaming && ! session_active_locked() )
            emit_event( callbacks(), "streaming_stopped" );
        throw;
    }
}

void RealSenseEngine::set_filters( const json & filters )
{
    FilterConfig config = parse_filter_config( filters );
    std::lock_guard< std::mutex > lock( m_mutex );
    check_open_locked();
    m_config.filters = config;
    m_shared->processing->set_filters( config );
}

void RealSenseEngine::start()
{
    std::lock_guard< std::mutex > lock( m_mutex );
    ensure_device_locked();
    if( session_active_locked() )
        return;
    if( m_session )
        teardown_session_locked( false );  // a playback that reached its end
    if( ! m_configured )
    {
        apply_configuration_locked( m_config );
        m_configured = true;
    }
    start_session_locked( true );
}

void RealSenseEngine::stop()
{
    std::lock_guard< std::mutex > lock( m_mutex );
    check_open_locked();
    teardown_session_locked( true );
}

bool RealSenseEngine::streaming() const
{
    std::lock_guard< std::mutex > lock( m_mutex );
    return ! m_shut_down && session_active_locked();
}

void RealSenseEngine::set_option( const std::string & sensor, const std::string & option, float value )
{
    std::lock_guard< std::mutex > lock( m_mutex );
    ensure_device_locked();
    if( m_playback_mode )
        throw std::runtime_error( "Options cannot be changed during playback" );
    if( sensor != "depth" && sensor != "color" )
        throw std::runtime_error( "Unknown sensor '" + sensor + "' (expected depth or color)" );
    rs2::sensor s = find_sensor_locked( sensor );
    if( ! s )
        throw std::runtime_error( "The camera has no " + sensor + " sensor" );
    auto id = option_from_name( option );
    if( ! id || ! s.supports( *id ) )
        throw std::runtime_error( "The " + sensor + " sensor has no option '" + option + "'" );
    const char * name = rs2_option_to_string( *id );
    if( s.is_option_read_only( *id ) )
        throw std::runtime_error( std::string( name ) + " is read-only" );
    if( ! std::isfinite( value ) )
        throw std::runtime_error( std::string( name ) + ": not a number" );
    auto range = s.get_option_range( *id );
    if( value < range.min || value > range.max )
        throw std::runtime_error( std::string( name ) + ": value out of range [" + format_value( range.min ) + ", "
                                  + format_value( range.max ) + "]" );
    try
    {
        s.set_option( *id, value );
    }
    catch( const rs2::error & e )
    {
        throw std::runtime_error( std::string( name ) + ": " + e.what() );
    }
}

void RealSenseEngine::set_auto_exposure_roi( int x, int y, int width, int height, float distance_m )
{
    std::lock_guard< std::mutex > lock( m_mutex );
    ensure_device_locked();
    if( width <= 0 || height <= 0 || width > 16384 || height > 16384 || std::abs( x ) > 16384 || std::abs( y ) > 16384 )
        throw std::runtime_error( "Invalid auto-exposure region" );
    if( ! std::isfinite( distance_m ) )
        distance_m = 0;
    m_roi = PixelRect{ x, y, width, height };
    m_roi_distance_m = distance_m;
    if( m_playback_mode )
        return;
    apply_roi_locked();  // without streams it is applied at the next start
}

void RealSenseEngine::start_recording( const std::string & path, const std::string & format )
{
    std::lock_guard< std::mutex > lock( m_mutex );
    ensure_device_locked();
    if( m_playback_mode )
        throw std::runtime_error( "Recording is only available with the live camera" );
    if( m_recorder )
        throw std::runtime_error( "A recording is already running" );
    if( format != "db3" && format != "bag" )
        throw std::runtime_error( "Unknown recording format '" + format + "' (expected db3 or bag)" );
    // librealsense picks the file format from the extension (.db3: ROS2/sqlite, anything else: ROS1 bag).
    if( ! has_extension( path, "." + format ) )
        throw std::runtime_error( "The recording path must end with ." + format );

    bool restart = session_active_locked();
    if( m_session )
        teardown_session_locked( false );
    try
    {
        // Wrapping the device hooks the recorder into the sensors' callbacks when they start.
        m_recorder = std::make_unique< rs2::recorder >( path, m_device );
        m_recording_path = path;
    }
    catch( const rs2::error & e )
    {
        std::string message = std::string( "Could not record to a ." ) + format + " file: " + e.what();
        if( format == "bag" )
            message += " (.bag recording needs patches/0002 in the SDK build)";
        if( restart )
            start_session_locked( false );
        throw std::runtime_error( message );
    }
    if( restart )
    {
        try
        {
            start_session_locked( false );
        }
        catch( ... )
        {
            m_recorder.reset();
            m_recording_path.clear();
            emit_event( callbacks(), "streaming_stopped" );
            throw;
        }
    }
    log( "info", "Recording (" + format + ")" );
    emit_event( callbacks(), "recording_started", { { "format", format } } );
}

void RealSenseEngine::stop_recording()
{
    std::lock_guard< std::mutex > lock( m_mutex );
    check_open_locked();
    if( ! m_recorder )
        throw std::runtime_error( "No recording is running" );
    finish_recording_locked();
}

void RealSenseEngine::playback_control( bool pause, bool real_time )
{
    std::lock_guard< std::mutex > lock( m_mutex );
    check_open_locked();
    if( ! m_playback )
        throw std::runtime_error( "Playback control is only available when playing a recording" );
    m_playback->set_real_time( real_time );
    if( pause && ! m_playback_paused )
        m_playback->pause();
    else if( ! pause && m_playback_paused )
        m_playback->resume();
    m_playback_paused = pause;
}

void RealSenseEngine::hardware_reset()
{
    std::lock_guard< std::mutex > lock( m_mutex );
    ensure_device_locked();
    if( m_playback_mode )
        throw std::runtime_error( "Hardware reset is not available during playback" );
    if( m_recorder )
        throw std::runtime_error( "Stop the recording before resetting the camera" );

    bool was_streaming = session_active_locked();
    teardown_session_locked( false );
    log( "info", "Resetting the camera" );
    expect_reenumeration_locked();
    try
    {
        m_device.hardware_reset();
        release_device_locked();
        adopt_device_locked( wait_for_reenumeration_locked( "the hardware reset" ) );
        if( m_configured )
            apply_configuration_locked( m_config );
        if( was_streaming )
            start_session_locked( false );
    }
    catch( ... )
    {
        {
            std::lock_guard< std::mutex > watch_lock( m_watch->mutex );
            m_watch->expected = false;
        }
        if( was_streaming )
            emit_event( callbacks(), "streaming_stopped" );
        throw;
    }
}

void RealSenseEngine::shutdown()
{
    std::lock_guard< std::mutex > lock( m_mutex );
    if( m_shut_down )
        return;
    m_shut_down = true;
    if( m_recorder )
    {
        try
        {
            if( m_session )
                teardown_session_locked( true );
            finish_recording_locked();
        }
        catch( const std::exception & e )
        {
            log( "error", std::string( "Finishing the recording: " ) + e.what() );
        }
    }
    try
    {
        teardown_session_locked( true );
    }
    catch( const std::exception & e )
    {
        log( "error", std::string( "Stopping the streams: " ) + e.what() );
    }
    m_recorder.reset();
    rs2::device old_current, old_arrived;
    {
        std::lock_guard< std::mutex > watch_lock( m_watch->mutex );
        m_watch->closed = true;
        old_current = m_watch->current;
        old_arrived = m_watch->arrived;
        m_watch->current = rs2::device();
        m_watch->arrived = rs2::device();
        m_watch->cv.notify_all();
    }
    old_current = rs2::device();
    old_arrived = rs2::device();
    if( m_ctx )
    {
        try
        {
            m_ctx->set_devices_changed_callback( []( rs2::event_information & ) {} );
        }
        catch( ... )
        {
        }
    }
    if( m_playback )
    {
        try
        {
            m_playback->set_status_changed_callback( []( rs2_playback_status ) {} );
        }
        catch( ... )
        {
        }
    }
    // Devices hold the context internally: release them first so the context's destruction closes every handle.
    m_playback.reset();
    m_device = rs2::device();
    m_ctx.reset();
}

}  // namespace

std::unique_ptr< CaptureEngine > CaptureEngine::create_live( CaptureCallbacks callbacks )
{
    auto engine = std::make_unique< RealSenseEngine >( std::move( callbacks ), false );
    engine->open_live();
    return engine;
}

std::unique_ptr< CaptureEngine > CaptureEngine::create_playback( const std::string & file, bool real_time,
                                                                 CaptureCallbacks callbacks )
{
    auto engine = std::make_unique< RealSenseEngine >( std::move( callbacks ), true );
    engine->open_playback( file, real_time );
    return engine;
}

std::unique_ptr< CaptureEngine > create_live_engine_for_device( rs2::device device, CaptureCallbacks callbacks )
{
    auto engine = std::make_unique< RealSenseEngine >( std::move( callbacks ), false );
    engine->open_device( device );
    return engine;
}

std::string sdk_version()
{
    return std::to_string( RS2_API_MAJOR_VERSION ) + "." + std::to_string( RS2_API_MINOR_VERSION ) + "."
         + std::to_string( RS2_API_PATCH_VERSION );
}

}  // namespace ifs
