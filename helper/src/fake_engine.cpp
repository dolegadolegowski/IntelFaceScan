// License: Apache 2.0. See LICENSE file in root directory.

#include "fake_engine.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <vector>

using nlohmann::json;

namespace ifs
{

namespace
{
constexpr float kPi = 3.14159265358979f;
// Keeps the largest frameset (color + aligned depth + depth + infrared) well inside IFS_MAX_PAYLOAD_BYTES.
constexpr int kMaxWidth = 2048;
constexpr int kMaxHeight = 2048;

struct Option
{
    std::string name;
    float value, min, max, step, def;
    bool read_only;
    std::map< std::string, std::string > descriptions;

    Option( std::string n, float v, float lo, float hi, float st, float d, bool ro = false,
            std::map< std::string, std::string > desc = {} )
        : name( std::move( n ) )
        , value( v )
        , min( lo )
        , max( hi )
        , step( st )
        , def( d )
        , read_only( ro )
        , descriptions( std::move( desc ) )
    {
    }
};

struct Stream
{
    int width, height, fps;
    bool operator==( const Stream & o ) const { return width == o.width && height == o.height && fps == o.fps; }
    bool operator!=( const Stream & o ) const { return ! ( *this == o ); }
};

struct Settings
{
    Stream color{ 1280, 800, 30 };
    Stream depth{ 1280, 720, 30 };
    float depth_scale = 0.0001f;
    bool imu = true;
    bool send_depth = false;
    bool send_ir = false;
    bool threshold = true;
    float threshold_min = 0.2f;
    float threshold_max = 1.2f;
};

const std::vector< std::string > kPresets
    = { "Custom", "Default", "Hand", "High Accuracy", "High Density", "Medium Density" };

std::string number_text( float v )
{
    char buf[32];
    std::snprintf( buf, sizeof( buf ), "%g", double( v ) );
    return buf;
}

double now_ms()
{
    using namespace std::chrono;
    return duration< double, std::milli >( system_clock::now().time_since_epoch() ).count();
}

Stream parse_stream( const json & c, const char * key, const Stream & current )
{
    auto it = c.find( key );
    if( it == c.end() || it->is_null() )
        return current;
    Stream s{ it->at( "width" ).get< int >(), it->at( "height" ).get< int >(), it->at( "fps" ).get< int >() };
    if( s.width < 64 || s.width > kMaxWidth || s.height < 48 || s.height > kMaxHeight || s.fps < 1 || s.fps > 120 )
        throw std::runtime_error( std::string( "unsupported " ) + key + " profile " + std::to_string( s.width ) + "x"
                                  + std::to_string( s.height ) + "@" + std::to_string( s.fps ) );
    return s;
}

json intrinsics_json( int w, int h, float hfov_deg )
{
    const float fx = 0.5f * float( w ) / std::tan( hfov_deg * kPi / 360.f );
    return { { "width", w }, { "height", h }, { "fx", fx }, { "fy", fx }, { "ppx", 0.5f * float( w - 1 ) + 0.7f },
             { "ppy", 0.5f * float( h - 1 ) - 0.4f }, { "model", "inverse_brown_conrady" },
             { "coeffs", { 0, 0, 0, 0, 0 } } };
}

// Depth of the sphere (center c, radius r) along each pixel ray of a pinhole camera; 0 where the ray misses or
// the threshold filter removes it.
void render_sphere( std::vector< uint16_t > & out, const json & intr, const float c[3], float r, const Settings & s )
{
    const int w = intr["width"], h = intr["height"];
    const float fx = intr["fx"], fy = intr["fy"], ppx = intr["ppx"], ppy = intr["ppy"];
    const float cc = c[0] * c[0] + c[1] * c[1] + c[2] * c[2] - r * r;
    out.assign( size_t( w ) * size_t( h ), 0 );
    for( int y = 0; y < h; ++y )
    {
        const float dy = ( float( y ) - ppy ) / fy;
        for( int x = 0; x < w; ++x )
        {
            const float dx = ( float( x ) - ppx ) / fx;
            const float dd = dx * dx + dy * dy + 1.f;
            const float dc = dx * c[0] + dy * c[1] + c[2];
            const float disc = dc * dc - dd * cc;
            if( disc < 0.f )
                continue;
            const float z = ( dc - std::sqrt( disc ) ) / dd;  // the ray has d.z = 1, so the parameter is the depth
            if( z <= 0.f || ( s.threshold && ( z < s.threshold_min || z > s.threshold_max ) ) )
                continue;
            const float units = z / s.depth_scale + 0.5f;
            if( units < 65535.f )
                out[size_t( y ) * size_t( w ) + size_t( x )] = uint16_t( units );
        }
    }
}

class FakeEngine final : public CaptureEngine
{
public:
    FakeEngine( CaptureCallbacks callbacks, FakeOptions options )
        : callbacks_( std::move( callbacks ) )
        , options_( options )
    {
        std::map< std::string, std::string > preset_names;
        for( size_t i = 0; i < kPresets.size(); ++i )
            preset_names[std::to_string( i )] = kPresets[i];
        depth_options_ = {
            { "Laser Power", 150, 0, 360, 30, 150 },
            { "Emitter Enabled", 1, 0, 2, 1, 1, false, { { "0", "Off" }, { "1", "Laser" }, { "2", "Laser Auto" } } },
            { "Enable Auto Exposure", 1, 0, 1, 1, 1 },
            { "Exposure", 8500, 1, 200000, 1, 8500 },
            { "Gain", 16, 16, 248, 1, 16 },
            { "Visual Preset", 1, 0, 5, 1, 1, false, preset_names },
            { "Depth Units", 0.0001f, 0.000001f, 0.01f, 0.000001f, 0.001f },
            { "Asic Temperature", 36, -40, 125, 0, 0, true },
        };
        color_options_ = {
            { "Enable Auto Exposure", 1, 0, 1, 1, 1 },
            { "Exposure", 156, 1, 10000, 1, 156 },
            { "Gain", 64, 0, 128, 1, 64 },
            { "Brightness", 0, -64, 64, 1, 0 },
            { "Enable Auto White Balance", 1, 0, 1, 1, 1 },
        };
    }

    ~FakeEngine() override { shutdown(); }

    json device_info() override
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        check_alive();
        auto options_json = []( const std::vector< Option > & options ) {
            json list = json::array();
            for( const Option & o : options )
            {
                json descriptions = nullptr;
                if( ! o.descriptions.empty() )
                    descriptions = o.descriptions;
                list.push_back( { { "name", o.name }, { "value", o.value }, { "min", o.min }, { "max", o.max },
                                  { "step", o.step }, { "default", o.def }, { "read_only", o.read_only },
                                  { "value_descriptions", descriptions } } );
            }
            return list;
        };
        json profiles = json::array();
        int index = 0;
        auto profile = [&]( const char * stream, const char * format, int w, int h, int fps ) {
            profiles.push_back( { { "stream", stream }, { "index", index++ }, { "format", format }, { "width", w },
                                  { "height", h }, { "fps", fps } } );
        };
        for( int fps : { 5, 15, 30 } )
        {
            profile( "color", "rgb8", 1280, 800, fps );
            profile( "color", "rgb8", 1280, 720, fps );
            profile( "color", "rgb8", 640, 480, fps );
            profile( "depth", "z16", 1280, 720, fps );
            profile( "depth", "z16", 848, 480, fps );
            profile( "infrared", "y8", 1280, 720, fps );
        }
        profile( "depth", "z16", 848, 480, 60 );
        profile( "depth", "z16", 848, 480, 90 );
        profile( "accel", "motion_xyz32f", 0, 0, 200 );
        profile( "gyro", "motion_xyz32f", 0, 0, 400 );
        return { { "name", "IntelFaceScan Fake Camera" },
                 { "serial", "000000000000" },
                 { "firmware", "5.17.0.10" },
                 { "recommended_firmware", "5.17.0.10" },
                 { "usb_type", "3.2" },
                 { "product_id", "0B5C" },
                 { "product_line", "D400" },
                 { "advanced_mode", true },
                 { "depth_scale", settings_.depth_scale },
                 { "baseline_mm", 95.0 },
                 { "imu", true },
                 { "source", "fake" },
                 { "presets", kPresets },
                 { "sensors",
                   { { { "name", "Stereo Module" }, { "kind", "depth" }, { "options", options_json( depth_options_ ) } },
                     { { "name", "RGB Camera" }, { "kind", "color" }, { "options", options_json( color_options_ ) } },
                     { { "name", "Motion Module" }, { "kind", "motion" }, { "options", json::array() } } } },
                 { "profiles", profiles } };
    }

    void configure( const json & c ) override
    {
        bool restart = false;
        {
            std::lock_guard< std::mutex > lock( mutex_ );
            check_alive();
            if( ! c.is_object() )
                throw std::runtime_error( "the configuration must be a JSON object" );
            Settings s = settings_;
            s.color = parse_stream( c, "color", s.color );
            s.depth = parse_stream( c, "depth", s.depth );
            s.imu = c.value( "enable_imu", s.imu );
            s.send_depth = c.value( "send_depth", s.send_depth );
            s.send_ir = c.value( "send_ir", s.send_ir );
            if( c.contains( "depth_units" ) )
            {
                float units = c.at( "depth_units" ).get< float >();
                if( ! ( units >= 0.000001f && units <= 0.01f ) )
                    throw std::runtime_error( "Depth Units: value out of range [1e-06, 0.01]" );
                s.depth_scale = units;
            }
            // Validate everything before changing anything, like the device applying a configuration.
            std::vector< std::pair< Option *, float > > changes;
            if( c.contains( "preset" ) )
            {
                const std::string preset = c.at( "preset" ).get< std::string >();
                auto it = std::find( kPresets.begin(), kPresets.end(), preset );
                if( it == kPresets.end() )
                    throw std::runtime_error( "unknown visual preset '" + preset + "'" );
                changes.emplace_back( &find( "depth", "Visual Preset" ), float( it - kPresets.begin() ) );
            }
            Option & laser = find( "depth", "Laser Power" );
            if( c.contains( "laser_power" ) )
            {
                const json & lp = c.at( "laser_power" );
                float value = lp.is_null() ? laser.max : lp.get< float >();
                check_range( laser, value );
                changes.emplace_back( &laser, value );
            }
            if( c.contains( "emitter_enabled" ) )
                changes.emplace_back( &find( "depth", "Emitter Enabled" ), c.at( "emitter_enabled" ).get< bool >() ? 1.f : 0.f );
            if( c.contains( "depth_auto_exposure" ) )
                changes.emplace_back( &find( "depth", "Enable Auto Exposure" ), c.at( "depth_auto_exposure" ).get< bool >() ? 1.f : 0.f );
            if( c.contains( "color_auto_exposure" ) )
                changes.emplace_back( &find( "color", "Enable Auto Exposure" ), c.at( "color_auto_exposure" ).get< bool >() ? 1.f : 0.f );
            if( c.contains( "filters" ) )
                apply_filters( c.at( "filters" ), s );
            for( auto & change : changes )
                change.first->value = change.second;
            find( "depth", "Depth Units" ).value = s.depth_scale;
            restart = running_ && ( s.color != settings_.color || s.depth != settings_.depth );
            settings_ = s;
        }
        if( restart )
        {
            stop_producer();
            log( "info", "fake camera: streams restarted with the new profiles" );
            start_producer();
        }
    }

    void set_filters( const json & f ) override
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        check_alive();
        Settings s = settings_;
        apply_filters( f, s );
        settings_ = s;
    }

    void start() override
    {
        {
            std::lock_guard< std::mutex > lock( mutex_ );
            check_alive();
            if( running_ )
                return;
        }
        start_producer();
        Settings s = snapshot();
        log( "info", "fake camera streaming: color " + std::to_string( s.color.width ) + "x"
                         + std::to_string( s.color.height ) + "@" + std::to_string( s.color.fps ) );
        event( "streaming_started", json::object() );
    }

    void stop() override
    {
        {
            std::lock_guard< std::mutex > lock( mutex_ );
            check_alive();
            if( ! running_ )
                return;
        }
        stop_producer();
        event( "streaming_stopped", json::object() );
    }

    bool streaming() const override
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        return running_;
    }

    void set_option( const std::string & sensor, const std::string & name, float value ) override
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        check_alive();
        Option & o = find( sensor, name );
        if( o.read_only )
            throw std::runtime_error( name + " is read-only" );
        check_range( o, value );
        o.value = value;
        if( name == "Depth Units" )
            settings_.depth_scale = value;
    }

    void set_auto_exposure_roi( int x, int y, int width, int height, float distance_m ) override
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        check_alive();
        if( width <= 0 || height <= 0 || x < 0 || y < 0 || x + width > settings_.color.width
            || y + height > settings_.color.height )
            throw std::runtime_error( "auto-exposure ROI outside the color image" );
    }

    void start_recording( const std::string & path, const std::string & format ) override
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        check_alive();
        if( record_fd_ >= 0 )
            throw std::runtime_error( "a recording is already running" );
        int fd = ::open( path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600 );
        if( fd < 0 )
            throw std::runtime_error( "cannot create the recording: " + std::string( std::strerror( errno ) ) );
        const std::string header = "IFSFAKE1 format=" + format + "\n";
        if( ::write( fd, header.data(), header.size() ) != ssize_t( header.size() ) )
        {
            ::close( fd );
            throw std::runtime_error( "cannot write the recording" );
        }
        record_fd_ = fd;
    }

    void stop_recording() override
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        if( record_fd_ < 0 )
            throw std::runtime_error( "no recording is running" );
        int rc = ::close( record_fd_ );
        record_fd_ = -1;
        if( rc != 0 || record_error_ )
        {
            record_error_ = false;
            throw std::runtime_error( "writing the recording failed" );
        }
    }

    void playback_control( bool, bool ) override
    {
        throw std::runtime_error( "playback control is only available for recordings" );
    }

    void hardware_reset() override
    {
        bool was_running;
        {
            std::lock_guard< std::mutex > lock( mutex_ );
            check_alive();
            was_running = running_;
        }
        if( was_running )
            stop_producer();
        event( "device_disconnected", { { "message", "hardware reset" } } );
        std::this_thread::sleep_for( std::chrono::milliseconds( 200 ) );
        event( "device_connected", device_info() );
        if( was_running )
            start_producer();
    }

    void shutdown() override
    {
        {
            std::lock_guard< std::mutex > lock( mutex_ );
            if( shut_down_ )
                return;
        }
        stop_producer();
        std::lock_guard< std::mutex > lock( mutex_ );
        if( record_fd_ >= 0 )
        {
            ::close( record_fd_ );
            record_fd_ = -1;
        }
        shut_down_ = true;
    }

private:
    void check_alive() const
    {
        if( shut_down_ )
            throw std::runtime_error( "the camera was shut down" );
    }

    Option & find( const std::string & sensor, const std::string & name )
    {
        std::vector< Option > * options = sensor == "depth" ? &depth_options_
                                        : sensor == "color" ? &color_options_
                                                            : nullptr;
        if( ! options )
            throw std::runtime_error( "unknown sensor '" + sensor + "'" );
        for( Option & o : *options )
            if( o.name == name )
                return o;
        throw std::runtime_error( "the " + sensor + " sensor has no option '" + name + "'" );
    }

    static void check_range( const Option & o, float value )
    {
        if( ! ( value >= o.min && value <= o.max ) )
            throw std::runtime_error( o.name + ": value out of range [" + number_text( o.min ) + ", "
                                      + number_text( o.max ) + "]" );
    }

    static void apply_filters( const json & f, Settings & s )
    {
        if( ! f.is_object() )
            throw std::runtime_error( "the filter configuration must be a JSON object" );
        if( f.contains( "threshold" ) )
        {
            const json & t = f.at( "threshold" );
            const bool enabled = t.value( "enabled", s.threshold );
            const float lo = t.value( "min_m", s.threshold_min ), hi = t.value( "max_m", s.threshold_max );
            if( ! ( lo >= 0.f && hi > lo && hi <= 65.f ) )
                throw std::runtime_error( "threshold filter: need 0 <= min_m < max_m <= 65" );
            s.threshold = enabled;
            s.threshold_min = lo;
            s.threshold_max = hi;
        }
    }

    Settings snapshot() const
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        return settings_;
    }

    void log( const std::string & level, const std::string & message )
    {
        if( callbacks_.on_log )
            callbacks_.on_log( level, message );
    }

    void event( const std::string & name, const json & data )
    {
        if( callbacks_.on_event )
            callbacks_.on_event( name, data );
    }

    void start_producer()
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        if( running_ )
            return;
        running_ = true;
        producer_ = std::thread( [this] { producer_loop(); } );
    }

    void stop_producer()
    {
        std::thread producer;
        {
            std::lock_guard< std::mutex > lock( mutex_ );
            if( ! running_ )
                return;
            running_ = false;
            producer = std::move( producer_ );
        }
        wake_cv_.notify_all();
        if( producer.joinable() )
            producer.join();
    }

    void producer_loop()
    {
        using clock = std::chrono::steady_clock;
        auto next = clock::now();
        for( ;; )
        {
            Settings s;
            uint64_t index;
            {
                std::lock_guard< std::mutex > lock( mutex_ );
                if( ! running_ )
                    return;
                s = settings_;
                index = frame_index_++;
            }
            Frameset fs = make_frameset( s, index );
            record( fs );
            if( callbacks_.on_frameset )
                callbacks_.on_frameset( std::move( fs ) );

            const auto period = std::chrono::microseconds( 1000000 / s.color.fps );
            next += period;
            const auto now = clock::now();
            if( now > next + 3 * period )
                next = now;  // fell behind (e.g. suspended): do not burst to catch up
            std::unique_lock< std::mutex > lock( mutex_ );
            wake_cv_.wait_until( lock, next, [this] { return ! running_; } );
        }
    }

    Frameset make_frameset( const Settings & s, uint64_t n )
    {
        Frameset fs;
        const double timestamp = now_ms();
        const float t = float( n ) / float( s.color.fps );
        const float center[3] = { 0.05f * std::sin( 2 * kPi * t / 4.f ), 0.02f * std::sin( 2 * kPi * t / 3.f ), 0.45f };
        const float radius = 0.1f;

        const json color_intr = intrinsics_json( s.color.width, s.color.height, 90.f );
        auto aligned = std::make_shared< std::vector< uint16_t > >();
        render_sphere( *aligned, color_intr, center, radius, s );

        const int w = s.color.width, h = s.color.height;
        const size_t stride = size_t( w ) * 3 + size_t( std::max( 0, options_.color_row_padding ) );
        auto color = std::make_shared< std::vector< uint8_t > >( stride * size_t( h ), uint8_t( 0xEE ) );
        const float fx = color_intr["fx"], fy = color_intr["fy"], ppx = color_intr["ppx"], ppy = color_intr["ppy"];
        for( int y = 0; y < h; ++y )
        {
            uint8_t * row = color->data() + size_t( y ) * stride;
            for( int x = 0; x < w; ++x )
            {
                uint8_t * px = row + size_t( x ) * 3;
                const uint16_t d = ( *aligned )[size_t( y ) * size_t( w ) + size_t( x )];
                if( d )
                {
                    // Lambert-shaded skin tone from the sphere normal at the visible point.
                    const float z = float( d ) * s.depth_scale;
                    const float p[3] = { ( float( x ) - ppx ) / fx * z, ( float( y ) - ppy ) / fy * z, z };
                    const float nz = ( p[2] - center[2] ) / radius;
                    const float shade = 0.35f + 0.65f * std::min( 1.f, std::max( 0.f, -nz ) );
                    px[0] = uint8_t( 230.f * shade );
                    px[1] = uint8_t( 180.f * shade );
                    px[2] = uint8_t( 150.f * shade );
                }
                else
                {
                    px[0] = uint8_t( x + 3 * int( n ) );
                    px[1] = uint8_t( y + 2 * int( n ) );
                    px[2] = uint8_t( ( x ^ y ) + int( n ) );
                }
            }
        }

        float laser;
        {
            std::lock_guard< std::mutex > lock( mutex_ );
            laser = find( "depth", "Laser Power" ).value;
        }
        const uint64_t sequence = n + 1;
        fs.metadata = { { "sequence", sequence },
                        { "timestamp_ms", timestamp },
                        { "timestamp_domain", "global_time" },
                        { "depth_scale", s.depth_scale },
                        { "depth_to_color",
                          { { "rotation", { 1, 0, 0, 0, 1, 0, 0, 0, 1 } }, { "translation", { -0.059, 0.0001, 0.0004 } } } },
                        { "imu", nullptr },
                        { "metadata", { { "color_exposure_us", 8000 }, { "depth_exposure_us", 8500 }, { "laser_power", laser } } } };
        if( s.imu )
        {
            const float wobble = 0.05f * std::sin( 2 * kPi * t );
            fs.metadata["imu"] = { { "accel", { wobble, -9.81f, 0.1f } },
                                   { "gyro", { 0.001f * wobble, 0.f, -0.001f } },
                                   { "timestamp_ms", timestamp - 2.0 } };
        }

        auto plane = [&]( const char * kind, const char * format, int pw, int ph, size_t pstride, const json & intr,
                          const uint8_t * data, size_t bytes, std::shared_ptr< void > keep ) {
            Plane p;
            p.desc = { { "kind", kind },       { "format", format },         { "width", pw },
                       { "height", ph },       { "stride", pstride },        { "frame_number", n },
                       { "timestamp_ms", timestamp }, { "intrinsics", intr } };
            p.data = data;
            p.bytes = bytes;
            p.keepalive = std::move( keep );
            fs.planes.push_back( std::move( p ) );
        };
        plane( "color", "rgb8", w, h, stride, color_intr, color->data(), color->size(), color );
        plane( "depth_aligned", "z16", w, h, size_t( w ) * 2, color_intr,
               reinterpret_cast< const uint8_t * >( aligned->data() ), aligned->size() * 2, aligned );

        if( s.send_depth || s.send_ir )
        {
            const json depth_intr = intrinsics_json( s.depth.width, s.depth.height, 87.f );
            const int dw = s.depth.width, dh = s.depth.height;
            if( s.send_depth )
            {
                // The depth camera sits 59 mm to the left of the color camera (depth_to_color translation).
                const float depth_center[3] = { center[0] + 0.059f, center[1], center[2] };
                auto depth = std::make_shared< std::vector< uint16_t > >();
                render_sphere( *depth, depth_intr, depth_center, radius, s );
                plane( "depth", "z16", dw, dh, size_t( dw ) * 2, depth_intr,
                       reinterpret_cast< const uint8_t * >( depth->data() ), depth->size() * 2, depth );
            }
            if( s.send_ir )
            {
                auto ir = std::make_shared< std::vector< uint8_t > >( size_t( dw ) * size_t( dh ) );
                for( int y = 0; y < dh; ++y )
                    for( int x = 0; x < dw; ++x )
                        ( *ir )[size_t( y ) * size_t( dw ) + size_t( x )]
                            = ( unsigned( x * 7919 + y * 104729 ) + unsigned( n ) ) % 97 < 9 ? 255 : 60;
                plane( "ir_left", "y8", dw, dh, size_t( dw ), depth_intr, ir->data(), ir->size(), ir );
            }
        }
        return fs;
    }

    // One line of metadata and a slice of the aligned depth per frame: enough bytes to exercise the copy.
    void record( const Frameset & fs )
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        if( record_fd_ < 0 || fs.planes.size() < 2 )
            return;
        const std::string line = fs.metadata.dump( -1, ' ', false, json::error_handler_t::replace ) + "\n";
        const Plane & depth = fs.planes[1];
        const size_t slice = std::min< size_t >( depth.bytes, 64 * 1024 );
        if( ::write( record_fd_, line.data(), line.size() ) != ssize_t( line.size() )
            || ::write( record_fd_, depth.data, slice ) != ssize_t( slice ) )
            record_error_ = true;
    }

    CaptureCallbacks callbacks_;
    FakeOptions options_;
    mutable std::mutex mutex_;
    std::condition_variable wake_cv_;
    Settings settings_;
    std::vector< Option > depth_options_;
    std::vector< Option > color_options_;
    std::thread producer_;
    bool running_ = false;
    bool shut_down_ = false;
    uint64_t frame_index_ = 0;
    int record_fd_ = -1;
    bool record_error_ = false;
};
}  // namespace

std::unique_ptr< CaptureEngine > create_fake_engine( CaptureCallbacks callbacks, FakeOptions options )
{
    return std::make_unique< FakeEngine >( std::move( callbacks ), options );
}

}  // namespace ifs
