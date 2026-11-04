// License: Apache 2.0. See LICENSE file in root directory.
//
// Tests of the capture module without a camera: configuration validation, helpers, measurement math, and the
// CaptureEngine end to end on synthetic recordings (software device -> .db3/.bag -> playback -> framesets).
// Nothing here may depend on how fast the machine is (CI runs it on a 3-core runner): count framesets only where
// every frame is delivered (non-real-time playback, the live rig's lockstep feed), and otherwise wait for
// conditions with generous bounds. Benchmarks (bench_*) run only when asked for by name: capture-tests bench.

#include "capture_config.h"
#include "capture_engine.h"
#include "capture_measure.h"
#include "capture_testing.h"
#include "capture_util.h"
#include "probe.h"
#include "test_framework.h"
#include "test_recording.h"

#include <librealsense2/rs.hpp>

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <map>
#include <mutex>
#include <set>
#include <thread>

using nlohmann::json;
using namespace ifs;
using namespace ifs::test;
namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace
{

struct TempDir
{
    fs::path path;
    TempDir()
    {
        std::string templ = ( fs::temp_directory_path() / "ifs-capture-tests-XXXXXX" ).string();
        if( ! ::mkdtemp( templ.data() ) )
            throw std::runtime_error( "mkdtemp failed" );
        path = templ;
    }
    ~TempDir()
    {
        std::error_code ec;
        fs::remove_all( path, ec );
    }
    std::string file( const char * name ) const { return ( path / name ).string(); }
};

struct CapturedPlane
{
    json desc;
    std::vector< uint8_t > data;
};

struct CapturedFrameset
{
    json metadata;
    std::vector< CapturedPlane > planes;

    const CapturedPlane * plane( const std::string & kind ) const
    {
        for( auto & p : planes )
            if( p.desc.value( "kind", "" ) == kind )
                return &p;
        return nullptr;
    }
};

// Collects what a CaptureEngine reports. Pixels are copied for the first frameset and then every
// 'pixels_every'-th one, which bounds memory for long recordings.
class Collector
{
public:
    explicit Collector( size_t pixels_every = 5 )
        : m_pixels_every( pixels_every )
    {
    }

    CaptureCallbacks callbacks()
    {
        CaptureCallbacks cb;
        cb.on_frameset = [this]( Frameset && fs ) {
            CapturedFrameset c;
            c.metadata = fs.metadata;
            std::lock_guard< std::mutex > lock( m_mutex );
            bool copy = m_framesets.size() % m_pixels_every == 0;
            for( auto & p : fs.planes )
            {
                CapturedPlane cp;
                cp.desc = p.desc;
                cp.desc["bytes"] = p.bytes;
                if( copy && p.data )
                    cp.data.assign( p.data, p.data + p.bytes );
                c.planes.push_back( std::move( cp ) );
            }
            m_framesets.push_back( std::move( c ) );
            m_cv.notify_all();
        };
        cb.on_event = [this]( const std::string & event, const json & data ) {
            std::lock_guard< std::mutex > lock( m_mutex );
            m_events.emplace_back( event, data );
            m_cv.notify_all();
        };
        cb.on_log = [this]( const std::string & level, const std::string & message ) {
            std::lock_guard< std::mutex > lock( m_mutex );
            m_logs.push_back( level + ": " + message );
        };
        return cb;
    }

    bool wait_for_event( const std::string & name, std::chrono::milliseconds timeout )
    {
        std::unique_lock< std::mutex > lock( m_mutex );
        return m_cv.wait_for( lock, timeout, [&] { return count_locked( name ) > 0; } );
    }

    bool wait_for_framesets( size_t n, std::chrono::milliseconds timeout )
    {
        std::unique_lock< std::mutex > lock( m_mutex );
        return m_cv.wait_for( lock, timeout, [&] { return m_framesets.size() >= n; } );
    }

    std::vector< CapturedFrameset > framesets()
    {
        std::lock_guard< std::mutex > lock( m_mutex );
        return m_framesets;
    }
    size_t frameset_count()
    {
        std::lock_guard< std::mutex > lock( m_mutex );
        return m_framesets.size();
    }
    size_t event_count( const std::string & name )
    {
        std::lock_guard< std::mutex > lock( m_mutex );
        return count_locked( name );
    }
    std::vector< std::pair< std::string, json > > events()
    {
        std::lock_guard< std::mutex > lock( m_mutex );
        return m_events;
    }
    void clear()
    {
        std::lock_guard< std::mutex > lock( m_mutex );
        m_framesets.clear();
        m_events.clear();
    }

private:
    size_t count_locked( const std::string & name ) const
    {
        return size_t( std::count_if( m_events.begin(), m_events.end(),
                                      [&]( const auto & e ) { return e.first == name; } ) );
    }

    size_t m_pixels_every;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::vector< CapturedFrameset > m_framesets;
    std::vector< std::pair< std::string, json > > m_events;
    std::vector< std::string > m_logs;
};

bool near( double a, double b, double tolerance )
{
    return std::fabs( a - b ) <= tolerance;
}

// Waits until no frameset arrived for 'quiet' (at most 'limit') and returns the count then. After a pause, frames
// already in the syncer queue and the processing thread still come out; how long that takes depends on the machine.
size_t wait_until_quiet( Collector & col, std::chrono::milliseconds quiet, std::chrono::milliseconds limit = 20s )
{
    auto end = std::chrono::steady_clock::now() + limit;
    size_t count = col.frameset_count();
    auto changed = std::chrono::steady_clock::now();
    while( std::chrono::steady_clock::now() < end )
    {
        std::this_thread::sleep_for( 10ms );
        size_t now_count = col.frameset_count();
        if( now_count != count )
        {
            count = now_count;
            changed = std::chrono::steady_clock::now();
        }
        else if( std::chrono::steady_clock::now() - changed >= quiet )
            break;
    }
    return count;
}

// A recording at a quarter of the pixels (640x360 depth, 640x400 color): for the lifecycle tests of real-time
// playback, where full-size frames make the processing speed of the machine decide how many framesets arrive.
RecordingOptions small_recording( const std::string & path, int frames )
{
    RecordingOptions o;
    o.path = path;
    o.frames = frames;
    o.depth_width = 640;
    o.depth_height = 360;
    o.color_width = 640;
    o.color_height = 400;
    return o;
}

CaptureConfig config_for( const RecordingOptions & o )
{
    CaptureConfig c;
    c.depth = { o.depth_width, o.depth_height, o.fps };
    c.color = { o.color_width, o.color_height, o.fps };
    return c;
}

uint16_t z16_at( const CapturedPlane & p, int x, int y )
{
    size_t stride = p.desc["stride"].get< size_t >();
    uint16_t v;
    std::memcpy( &v, p.data.data() + size_t( y ) * stride + size_t( x ) * 2, 2 );
    return v;
}

void check_intrinsics( const json & j, const rs2_intrinsics & in )
{
    CHECK( j["width"] == in.width );
    CHECK( j["height"] == in.height );
    CHECK( near( j["fx"].get< double >(), in.fx, 1e-3 ) );
    CHECK( near( j["fy"].get< double >(), in.fy, 1e-3 ) );
    CHECK( near( j["ppx"].get< double >(), in.ppx, 1e-3 ) );
    CHECK( near( j["ppy"].get< double >(), in.ppy, 1e-3 ) );
    CHECK( j["model"] == distortion_name( in.model ) );
    REQUIRE( j["coeffs"].size() == 5 );
    for( int i = 0; i < 5; ++i )
        CHECK( near( j["coeffs"][i].get< double >(), in.coeffs[i], 1e-6 ) );
}

// Runs a playback to its end and returns the collected framesets.
std::vector< CapturedFrameset > play_all( const std::string & file, const CaptureConfig & config, Collector & col,
                                          std::unique_ptr< CaptureEngine > * keep = nullptr )
{
    auto engine = CaptureEngine::create_playback( file, false, col.callbacks() );
    engine->configure( to_json( config ) );
    engine->start();
    REQUIRE( col.wait_for_event( "playback_finished", 60s ) );
    CHECK( ! engine->streaming() );
    auto sets = col.framesets();
    if( keep )
        *keep = std::move( engine );
    else
        engine->shutdown();
    return sets;
}

// Fraction of sampled color pixels where the aligned depth matches the scene seen by the color camera.
struct AlignmentScore
{
    double agree = 0;     // |aligned - truth| <= tolerance among pixels valid in both
    double coverage = 0;  // pixels with aligned depth among pixels where the scene has depth
    size_t samples = 0;
};

AlignmentScore score_alignment( const RecordingOptions & o, const CapturedPlane & aligned, float depth_scale,
                                double tolerance_m )
{
    AlignmentScore s;
    int w = aligned.desc["width"], h = aligned.desc["height"];
    size_t both = 0, ok = 0, truth_valid = 0, measured_valid = 0;
    for( int y = 4; y < h - 4; y += 7 )
        for( int x = 4; x < w - 4; x += 7 )
        {
            float truth = scene_depth_at_color_pixel( o, 0, float( x ), float( y ) );
            float measured = z16_at( aligned, x, y ) * depth_scale;
            if( truth > 0 )
            {
                ++truth_valid;
                if( measured > 0 )
                    ++measured_valid;
            }
            if( truth > 0 && measured > 0 )
            {
                ++both;
                if( std::fabs( truth - measured ) <= tolerance_m )
                    ++ok;
            }
        }
    s.samples = both;
    s.agree = both ? double( ok ) / both : 0;
    s.coverage = truth_valid ? double( measured_valid ) / truth_valid : 0;
    return s;
}

// RMS of (depth - truth) over the head sphere region of a depth plane (depth sensor frame).
double head_depth_rms_mm( const RecordingOptions & o, const CapturedPlane & depth, float depth_scale )
{
    int w = depth.desc["width"], h = depth.desc["height"];
    double sum = 0;
    size_t n = 0;
    for( int y = h / 2 - 60; y < h / 2 + 60; y += 2 )
        for( int x = w / 2 - 60; x < w / 2 + 60; x += 2 )
        {
            float truth = scene_depth_at_depth_pixel( o, 0, float( x ), float( y ) );
            uint16_t raw = z16_at( depth, x, y );
            if( truth <= 0 || truth > o.distance_m || ! raw )
                continue;
            double e = raw * double( depth_scale ) - truth;
            sum += e * e;
            ++n;
        }
    return n ? std::sqrt( sum / double( n ) ) * 1000.0 : -1.0;
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------
// Configuration

TEST_CASE( config_defaults_roundtrip )
{
    CaptureConfig defaults;
    json j = to_json( defaults );
    CHECK( j["depth"]["width"] == 1280 );
    CHECK( j["color"]["height"] == 800 );
    CHECK( j["preset"] == "High Accuracy" );
    CHECK( j["laser_power"].is_null() );
    CaptureConfig parsed = parse_capture_config( j );
    CHECK( parsed.same_streams( defaults ) );
    CHECK( parsed.preset == defaults.preset );
    CHECK( ! parsed.laser_power );
    CHECK( near( parsed.depth_units, 0.0001, 1e-9 ) );
    CHECK( parsed.filters.spatial.magnitude == 2 );
    CHECK( near( parsed.filters.temporal.alpha, 0.4, 1e-6 ) );

    // As Swift encodes it: nil optionals are left out, and the IPC layer may pass request_id along.
    j.erase( "laser_power" );
    j["request_id"] = 3;
    CHECK( ! parse_capture_config( j ).laser_power );
    j["laser_power"] = 240;
    CHECK( parse_capture_config( j ).laser_power.value_or( 0 ) == 240.f );
    j["depth"]["fps"] = 30.0;  // integral floats are integers in JSON
    CHECK( parse_capture_config( j ).depth.fps == 30 );
}

TEST_CASE( config_rejects_invalid_input )
{
    CHECK_THROWS_WITH( parse_capture_config( json::array() ), "expected a JSON object" );

    // A valid configuration with one thing wrong, and the message that must name it.
    using Change = std::function< void( json & ) >;
    const std::vector< std::pair< Change, std::string > > cases = {
        { []( json & j ) { j["bogus"] = 1; }, "CaptureConfiguration.bogus: unknown key" },
        { []( json & j ) { j.erase( "preset" ); }, "CaptureConfiguration.preset: missing" },
        { []( json & j ) { j["depth"]["width"] = "1280"; }, "depth.width: expected an integer" },
        { []( json & j ) { j["depth"]["fps"] = 29.5; }, "depth.fps: expected an integer" },
        { []( json & j ) { j["depth"]["height"] = 0; }, "out of range" },
        { []( json & j ) { j["enable_ir"] = 1; }, "enable_ir: expected true or false" },
        { []( json & j ) { j["preset"] = ""; }, "preset" },
        { []( json & j ) { j["preset"] = "High\nAccuracy"; }, "control characters" },
        { []( json & j ) { j["laser_power"] = -1; }, "laser_power" },
        { []( json & j ) { j["depth_units"] = 0.5; }, "depth_units" },
        { []( json & j ) { j["depth_units"] = nullptr; }, "expected a number" },
        { []( json & j ) { j["disparity_shift"] = 600; }, "disparity_shift" },
        { []( json & j ) { j["align_to_color"] = false; }, "align_to_color" },
        { []( json & j ) { j["send_ir"] = true; }, "send_ir: requires enable_ir" },
        { []( json & j ) { j["filters"]["spatial"]["alpha"] = 2; },
          "CaptureConfiguration.filters.spatial.alpha: 2 is out of range [0.25, 1]" },
        { []( json & j ) { j["filters"]["temporal"]["persistence"] = 9; }, "persistence" },
        { []( json & j ) { j["filters"]["hole_filling"]["mode"] = 3; }, "mode" },
        { []( json & j ) { j["filters"]["threshold"]["min_m"] = 2.0; }, "min_m must be smaller than max_m" },
        { []( json & j ) { j["filters"]["spatial"]["extra"] = 0; }, "unknown key" },
    };
    for( auto & c : cases )
    {
        json j = to_json( CaptureConfig() );
        c.first( j );
        CHECK_THROWS_WITH( parse_capture_config( j ), c.second );
    }
}

TEST_CASE( config_filters_roundtrip )
{
    FilterConfig f;
    f.threshold = { true, 0.3f, 0.9f };
    f.disparity_domain = false;
    f.spatial = { true, 3, 0.6f, 25, 2 };
    f.temporal = { false, 0.2f, 40, 5 };
    f.hole_filling = { true, 2 };
    FilterConfig g = parse_filter_config( to_json( f ) );
    CHECK( g.threshold.enabled && near( g.threshold.min_m, 0.3, 1e-6 ) && near( g.threshold.max_m, 0.9, 1e-6 ) );
    CHECK( ! g.disparity_domain );
    CHECK( g.spatial.magnitude == 3 && g.spatial.delta == 25 && g.spatial.holes_fill == 2 );
    CHECK( ! g.temporal.enabled && g.temporal.delta == 40 && g.temporal.persistence == 5 );
    CHECK( g.hole_filling.enabled && g.hole_filling.mode == 2 );
    FilterConfig none = FilterConfig::none();
    CHECK( ! none.threshold.enabled && ! none.spatial.enabled );
    CHECK( ! none.temporal.enabled && ! none.hole_filling.enabled );
    CHECK_THROWS_WITH( parse_filter_config( json{ { "threshold", 1 } } ), "FilterConfiguration" );
}

// ---------------------------------------------------------------------------------------------------------------
// Helpers

TEST_CASE( util_protocol_names )
{
    CHECK( stream_name( RS2_STREAM_DEPTH ) == "depth" );
    CHECK( stream_name( RS2_STREAM_COLOR ) == "color" );
    CHECK( stream_name( RS2_STREAM_INFRARED ) == "infrared" );
    CHECK( stream_name( RS2_STREAM_ACCEL ) == "accel" );
    CHECK( stream_name( RS2_STREAM_GYRO ) == "gyro" );
    CHECK( format_name( RS2_FORMAT_Z16 ) == "z16" );
    CHECK( format_name( RS2_FORMAT_RGB8 ) == "rgb8" );
    CHECK( format_name( RS2_FORMAT_Y8 ) == "y8" );
    CHECK( format_name( RS2_FORMAT_MOTION_XYZ32F ) == "motion_xyz32f" );
    CHECK( distortion_name( RS2_DISTORTION_NONE ) == "none" );
    CHECK( distortion_name( RS2_DISTORTION_BROWN_CONRADY ) == "brown_conrady" );
    CHECK( distortion_name( RS2_DISTORTION_INVERSE_BROWN_CONRADY ) == "inverse_brown_conrady" );
    CHECK( timestamp_domain_name( RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME ) == "global_time" );
    CHECK( timestamp_domain_name( RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK ) == "hardware_clock" );
    CHECK( timestamp_domain_name( RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME ) == "system_time" );
    CHECK( option_from_name( "Laser Power" ) == RS2_OPTION_LASER_POWER );
    CHECK( option_from_name( "laser power" ) == RS2_OPTION_LASER_POWER );
    CHECK( option_from_name( "Visual Preset" ) == RS2_OPTION_VISUAL_PRESET );
    CHECK( ! option_from_name( "Warp Drive" ) );
    CHECK( iequals( "High Accuracy", "high accuracy" ) && ! iequals( "High", "High Accuracy" ) );
}

TEST_CASE( util_extrinsics_json_and_inverse )
{
    rs2_extrinsics e = synthetic_depth_to_color();
    json j = extrinsics_json( e );
    REQUIRE( j["rotation"].size() == 9 );
    REQUIRE( j["translation"].size() == 3 );
    CHECK( near( j["translation"][0].get< double >(), -0.059, 1e-7 ) );
    rs2_extrinsics inv = inverse( e ), back = inverse( inv );
    for( int i = 0; i < 9; ++i )
        CHECK( near( back.rotation[i], e.rotation[i], 1e-6 ) );
    for( int i = 0; i < 3; ++i )
        CHECK( near( back.translation[i], e.translation[i], 1e-7 ) );
    // A point moved to color and back lands where it started.
    float p[3] = { 0.1f, -0.05f, 0.6f }, q[3], r[3];
    rs2_transform_point_to_point( q, &e, p );
    rs2_transform_point_to_point( r, &inv, q );
    for( int i = 0; i < 3; ++i )
        CHECK( near( r[i], p[i], 1e-6 ) );
}

TEST_CASE( util_roi_mapping )
{
    rs2_intrinsics color = synthetic_color_intrinsics( 1280, 800 );
    rs2_intrinsics depth = synthetic_depth_intrinsics( 1280, 720 );
    rs2_extrinsics c2d = inverse( synthetic_depth_to_color() );

    // A face-sized box in the middle of the color image at 0.5 m.
    PixelRect face{ 540, 250, 200, 280 };
    auto mapped = map_color_rect_to_depth( color, depth, c2d, face, 0.5f );
    REQUIRE( mapped );
    // The RGB camera sits 59 mm to the right of the left imager: at 0.5 m that is ~75 px (632 * 0.059 / 0.5) to
    // the left in the color image, i.e. the depth box is shifted right relative to the color box.
    double color_center_x = face.x + face.width / 2.0;
    double depth_center_x = mapped->x + mapped->width / 2.0;
    CHECK_MSG( near( depth_center_x - ( color_center_x - color.ppx + depth.ppx ), 632 * 0.059 / 0.5, 12 ),
               "shift " << depth_center_x - color_center_x );
    CHECK( near( mapped->width, face.width * depth.fx / color.fx, 8 ) );
    CHECK( mapped->x >= 0 && mapped->y >= 0 && mapped->x + mapped->width <= 1280 && mapped->y + mapped->height <= 720 );

    // Farther away, the parallax shrinks.
    auto far = map_color_rect_to_depth( color, depth, c2d, face, 3.0f );
    REQUIRE( far );
    CHECK( far->x < mapped->x );

    // Clamped at the image border; nothing left -> nullopt.
    auto edge = map_color_rect_to_depth( color, depth, c2d, PixelRect{ 1200, 700, 200, 200 }, 0.5f );
    REQUIRE( edge );
    CHECK( edge->x + edge->width <= 1280 && edge->y + edge->height <= 720 );
    CHECK( ! map_color_rect_to_depth( color, depth, c2d, PixelRect{ 5000, 5000, 10, 10 }, 0.5f ) );
    CHECK( ! map_color_rect_to_depth( color, depth, c2d, face, 0.f ) );
    CHECK( ! clamp_rect( PixelRect{ 0, 0, 0, 10 }, 100, 100 ) );
    auto c = clamp_rect( PixelRect{ -10, -10, 50, 50 }, 100, 100 );
    REQUIRE( c );
    CHECK( c->x == 0 && c->y == 0 && c->width == 40 && c->height == 40 );

    // The color rectangle is deprojected with the color intrinsics and projected with the depth intrinsics (not the
    // other way round): at 0.5 m a 200 px wide face is ~197 px wide in depth (632 / 640.1).
    rs2_intrinsics depth_848 = synthetic_depth_intrinsics( 848, 480 );
    auto small = map_color_rect_to_depth( color, depth_848, c2d, face, 0.5f );
    REQUIRE( small );
    CHECK_MSG( near( small->width, face.width * depth_848.fx / color.fx, 6 ), small->width );
    CHECK( small->x + small->width <= 848 && small->y + small->height <= 480 );
}

TEST_CASE( util_roi_minimum_size )
{
    // A face rectangle that mostly left the image leaves a sliver after clamping; auto exposure needs an area.
    auto sliver = clamp_rect( PixelRect{ 1279, 300, 300, 200 }, 1280, 800 );
    REQUIRE( sliver );
    CHECK( sliver->width == 1 );
    PixelRect r = ensure_min_size( *sliver, k_min_roi_px, k_min_roi_px, 1280, 800 );
    CHECK_MSG( r.width == k_min_roi_px && r.x + r.width == 1280, r.x << " " << r.width );  // moved inside
    CHECK( r.y == 300 && r.height == 200 );                                                // tall enough already
    // Grown about the center when there is room.
    r = ensure_min_size( PixelRect{ 600, 400, 4, 2 }, 32, 32, 1280, 800 );
    CHECK_MSG( r.x == 586 && r.width == 32 && r.y == 385 && r.height == 32, r.x << "," << r.y );
    // Top-left corner, and an image smaller than the minimum.
    r = ensure_min_size( PixelRect{ 0, 0, 1, 1 }, 32, 32, 1280, 800 );
    CHECK( r.x == 0 && r.y == 0 && r.width == 32 && r.height == 32 );
    r = ensure_min_size( PixelRect{ 3, 2, 2, 2 }, 32, 32, 20, 10 );
    CHECK( r.x == 0 && r.y == 0 && r.width == 20 && r.height == 10 );
    // Large rectangles are unchanged.
    r = ensure_min_size( PixelRect{ 500, 220, 300, 380 }, 32, 32, 1280, 800 );
    CHECK( r.x == 500 && r.y == 220 && r.width == 300 && r.height == 380 );
}

TEST_CASE( util_options_to_restore_after_preset )
{
    // The face-scan configuration with the emitter off, before the depth sensor opened ...
    OptionValues before;
    before.preset = 1;
    before.depth_units = 0.0001f;
    before.disparity_shift = 50;
    before.emitter = 0;
    before.laser_power = 360;
    before.depth_auto_exposure = 0;
    before.color_auto_exposure = 1;
    // ... and after librealsense re-applied the "Default" preset (1 mm, shift 0, laser on at 150, auto exposure).
    OptionValues now = before;
    now.depth_units = 0.001f;
    now.disparity_shift = 0;
    now.emitter = 1;
    now.laser_power = 150;
    now.depth_auto_exposure = 1;
    OptionValues r = options_to_restore( before, now );
    CHECK( ! r.preset );
    CHECK( r.depth_units && near( *r.depth_units, 0.0001, 1e-9 ) );
    CHECK( r.disparity_shift == 50 );
    CHECK( r.laser_power == 360.f );
    CHECK( r.emitter == 0.f );  // applied after the laser power, which switches the emitter on
    CHECK( r.depth_auto_exposure == 0.f );
    CHECK( ! r.color_auto_exposure );

    // High Accuracy keeps these values: nothing to do (0.0001 read back as float is not "changed").
    now = before;
    now.depth_units = float( 0.0001 * ( 1 + 1e-8 ) );
    CHECK( options_to_restore( before, now ).empty() );

    // Only the laser power changed while the emitter stays off: the emitter is switched off again after it.
    now = before;
    now.laser_power = 150;
    r = options_to_restore( before, now );
    CHECK( r.laser_power == 360.f && r.emitter == 0.f && ! r.depth_units && ! r.disparity_shift );
    // With the emitter on, only the laser power.
    before.emitter = 1;
    now = before;
    now.laser_power = 150;
    r = options_to_restore( before, now );
    CHECK( r.laser_power == 360.f && ! r.emitter );
    // Options a camera does not have are left alone.
    OptionValues none;
    CHECK( options_to_restore( none, now ).empty() );
}

TEST_CASE( util_frame_counter_monitor )
{
    FrameCounterMonitor m;
    CHECK( ! m.update( 10 ) );
    CHECK( ! m.update( 11 ) );
    CHECK( ! m.update( 11 ) );  // repeated number: not a restart
    CHECK( ! m.update( 15 ) );  // dropped frames: not a restart
    CHECK( m.update( 1 ) );     // counter went back: restart
    CHECK( ! m.update( 2 ) );
    m.reset();
    CHECK( ! m.update( 0 ) );
}

TEST_CASE( util_stereo_range )
{
    // D455 1280x720, nominal 87 deg HFOV: f = 640 / tan(43.5 deg) = 674.4 px, B = 95 mm.
    double f = 640.0 / std::tan( 43.5 * 3.14159265358979 / 180.0 );
    auto r0 = stereo_range( f, 0.095, 0 );
    CHECK( near( r0.min_z_m, 0.5085, 0.002 ) );
    CHECK( r0.max_z_m == 0 );
    auto r50 = stereo_range( f, 0.095, 50 );
    CHECK( near( r50.min_z_m, 0.364, 0.002 ) );
    CHECK( near( r50.max_z_m, 1.281, 0.003 ) );
    CHECK( near( stereo_rms_error_m( 0.6, f, 0.095, 0.08 ) * 1000, 0.45, 0.01 ) );
}

// ---------------------------------------------------------------------------------------------------------------
// Measurement math

TEST_CASE( measure_plane_fit )
{
    // Points on z = 0.5 + 0.1 x tilted plane with +-1 mm alternating offsets along the normal.
    std::vector< double > xyz;
    double n[3] = { 0.1, 0, -1 };
    double len = std::sqrt( 0.01 + 1 );
    for( int i = 0; i < 40; ++i )
        for( int j = 0; j < 40; ++j )
        {
            double x = -0.2 + i * 0.01, y = -0.2 + j * 0.01, z = 0.5 + 0.1 * x;
            double off = ( ( i + j ) % 2 ? 0.001 : -0.001 );
            xyz.insert( xyz.end(), { x + off * n[0] / len, y, z + off * n[2] / len } );
        }
    auto plane = fit_plane( xyz );
    REQUIRE( plane );
    CHECK( plane->normal[2] < 0 );
    CHECK( near( plane->normal[0] / -plane->normal[2], 0.1, 1e-3 ) );
    CHECK( near( plane->rms_m, 0.001, 1e-5 ) );
    CHECK( ! fit_plane( { 0, 0, 1, 0, 0, 2 } ) );
    CHECK( ! fit_plane( { 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0, 4 } ) );  // collinear
}

TEST_CASE( measure_frame_and_temporal_noise )
{
    // A synthetic 200x100 wall at 0.6 m (0.1 mm units) with a hole and a near outlier corner.
    int w = 200, h = 100;
    std::vector< uint16_t > img( size_t( w ) * h, 6000 );
    for( int y = 40; y < 50; ++y )
        for( int x = 90; x < 100; ++x )
            img[size_t( y ) * w + x] = 0;
    img[0] = 3000;
    DepthView view{ img.data(), w, h, size_t( w ) * 2, 0.0001f, 100.f, 100.f, 100.f, 50.f };
    Roi roi = central_roi( w, h, 0.5 );
    CHECK( roi.x == 50 && roi.y == 25 && roi.width == 100 && roi.height == 50 );
    FrameStats s = measure_frame( view, roi, 0.095 );
    CHECK( near( s.fill_rate, 1.0 - 100.0 / 5000.0, 1e-9 ) );
    CHECK( near( s.median_m, 0.6, 1e-6 ) );
    CHECK( s.plane_rms_mm < 1e-3 );
    CHECK( near( s.nearest_m, 0.6, 1e-6 ) );  // one outlier pixel is below the 0.5 percentile
    CHECK( near( s.full_fill_rate, 1.0 - 100.0 / 20000.0, 1e-9 ) );

    // Temporal noise: alternate +-0.2 mm -> per-pixel std of 0.2 mm (n-1 normalization over 2 frames: 0.283).
    TemporalNoise t( roi );
    std::vector< uint16_t > a( img ), b( img );
    for( auto & v : a )
        if( v )
            v += 2;
    for( auto & v : b )
        if( v )
            v -= 2;
    for( int i = 0; i < 10; ++i )
    {
        DepthView frame = view;
        frame.data = ( i % 2 ? a : b ).data();
        t.add( frame );
    }
    auto noise = t.median_std_mm();
    REQUIRE( noise );
    CHECK_MSG( near( *noise, 0.2 * std::sqrt( 10.0 / 9.0 ), 1e-3 ), *noise );
    CHECK( near( quantile( { 3, 1, 2 }, 0.5 ), 2, 1e-12 ) );
    CHECK( near( mean( { 1, 2, 3 } ), 2, 1e-12 ) );
}

// ---------------------------------------------------------------------------------------------------------------
// End to end on synthetic recordings

TEST_CASE( playback_db3_end_to_end )
{
    TempDir dir;
    RecordingOptions o;
    o.path = dir.file( "head.db3" );
    o.frames = 15;
    RecordingInfo info = write_test_recording( o );
    REQUIRE( fs::file_size( o.path ) > 0 );
    // sqlite must leave a single, complete file behind (the helper copies only that file to the app)
    for( auto & entry : fs::directory_iterator( dir.path ) )
        CHECK_MSG( entry.path().filename() == "head.db3", entry.path() );

    CaptureConfig config;
    config.enable_ir = true;
    config.send_ir = true;
    config.send_depth = true;
    config.filters = FilterConfig::none();
    Collector col( 1 );
    auto sets = play_all( o.path, config, col );
    CHECK_MSG( sets.size() == size_t( o.frames ), sets.size() );
    REQUIRE( ! sets.empty() );

    auto events = col.events();
    REQUIRE( events.size() >= 3 );
    CHECK( events.front().first == "streaming_started" );
    CHECK( col.event_count( "playback_finished" ) == 1 );
    CHECK( col.event_count( "stream_restarted" ) == 0 );

    uint64_t previous_sequence = 0;
    for( size_t i = 0; i < sets.size(); ++i )
    {
        const auto & fs = sets[i];
        const json & md = fs.metadata;
        CHECK( md["sequence"].get< uint64_t >() > previous_sequence );
        previous_sequence = md["sequence"].get< uint64_t >();
        CHECK( md["timestamp_domain"] == "global_time" );
        CHECK( near( md["timestamp_ms"].get< double >(), info.first_timestamp_ms + 1000.0 / o.fps * i, 0.01 ) );
        CHECK( near( md["depth_scale"].get< double >(), o.depth_units, 1e-9 ) );
        for( int k = 0; k < 9; ++k )
            CHECK( near( md["depth_to_color"]["rotation"][k].get< double >(), info.depth_to_color.rotation[k], 1e-6 ) );
        for( int k = 0; k < 3; ++k )
            CHECK( near( md["depth_to_color"]["translation"][k].get< double >(), info.depth_to_color.translation[k],
                         1e-6 ) );
        REQUIRE( md["imu"].is_object() );
        CHECK( near( md["imu"]["accel"][1].get< double >(), -9.81, 1e-4 ) );
        CHECK( md["imu"]["gyro"].size() == 3 );
        CHECK_MSG( md["metadata"].value( "depth_exposure_us", 0.0 ) == 8500.0, md["metadata"].dump() );
        CHECK_MSG( md["metadata"].value( "laser_power", 0.0 ) == 360.0, md["metadata"].dump() );
        CHECK_MSG( md["metadata"].value( "color_exposure_us", 0.0 ) == 8000.0, md["metadata"].dump() );

        REQUIRE( fs.planes.size() == 4 );
        CHECK( fs.planes[0].desc["kind"] == "color" );
        CHECK( fs.planes[1].desc["kind"] == "depth_aligned" );
        CHECK( fs.planes[2].desc["kind"] == "depth" );
        CHECK( fs.planes[3].desc["kind"] == "ir_left" );
        const json & color = fs.planes[0].desc;
        CHECK( color["format"] == "rgb8" && color["width"] == 1280 && color["height"] == 800 );
        CHECK( color["stride"] == 3840 && color["bytes"] == 3840 * 800 );
        CHECK( color["frame_number"] == 200 + int( i ) );
        check_intrinsics( color["intrinsics"], info.color_intrinsics );
        const json & aligned = fs.planes[1].desc;
        CHECK( aligned["format"] == "z16" && aligned["width"] == 1280 && aligned["height"] == 800 );
        CHECK( aligned["stride"] == 2560 );
        check_intrinsics( aligned["intrinsics"], info.color_intrinsics );
        const json & depth = fs.planes[2].desc;
        CHECK( depth["format"] == "z16" && depth["width"] == 1280 && depth["height"] == 720 );
        CHECK( depth["frame_number"] == 100 + int( i ) );
        check_intrinsics( depth["intrinsics"], info.depth_intrinsics );
        const json & ir = fs.planes[3].desc;
        CHECK( ir["format"] == "y8" && ir["width"] == 1280 && ir["height"] == 720 && ir["stride"] == 1280 );
        check_intrinsics( ir["intrinsics"], info.depth_intrinsics );
    }

    // Depth without filters is the rendered scene; aligned depth matches what the color camera sees.
    const CapturedFrameset & first = sets.front();
    REQUIRE( ! first.planes[2].data.empty() );
    CHECK_MSG( head_depth_rms_mm( o, first.planes[2], o.depth_units ) < 0.1,
               head_depth_rms_mm( o, first.planes[2], o.depth_units ) );
    AlignmentScore score = score_alignment( o, first.planes[1], o.depth_units, 0.003 );
    CHECK_MSG( score.samples > 10000, score.samples );
    CHECK_MSG( score.agree > 0.97, score.agree );
    // The RGB camera sees ~6 degrees more vertically than the depth imagers: the top and bottom rows have no depth.
    CHECK_MSG( score.coverage > 0.85, score.coverage );
}

TEST_CASE( playback_device_info )
{
    TempDir dir;
    RecordingOptions o;
    o.path = dir.file( "info.db3" );
    o.frames = 3;
    o.pace = false;
    write_test_recording( o );
    Collector col;
    auto engine = CaptureEngine::create_playback( o.path, true, col.callbacks() );
    json info = engine->device_info();
    CHECK( info["source"] == "playback" );
    CHECK( info["serial"] == k_synthetic_serial );
    CHECK( info["name"] == "Intel RealSense D455 (synthetic)" );
    CHECK( info["firmware"] == "5.17.0.10" );
    CHECK( info["recommended_firmware"] == "5.17.0.10" );
    CHECK( info["usb_type"] == "3.2" );
    CHECK( info["product_id"] == "0B5C" );
    CHECK( info["product_line"] == "D400" );
    CHECK( info["advanced_mode"] == false );
    CHECK( near( info["depth_scale"].get< double >(), 0.0001, 1e-9 ) );
    CHECK_MSG( info["baseline_mm"].is_number() && near( info["baseline_mm"].get< double >(), 95.0, 0.01 ),
               info["baseline_mm"].dump() );
    CHECK( info["imu"] == true );
    CHECK( info["presets"].is_array() );
    std::set< std::string > kinds;
    for( auto & s : info["sensors"] )
    {
        kinds.insert( s["kind"].get< std::string >() );
        CHECK( s["options"].is_array() );
        for( auto & opt : s["options"] )
        {
            CHECK( opt["name"].is_string() );
            CHECK( opt["value"].is_number() && opt["min"].is_number() && opt["max"].is_number() );
            CHECK( opt["step"].is_number() && opt["default"].is_number() && opt["read_only"].is_boolean() );
            CHECK( opt.contains( "value_descriptions" ) );
        }
    }
    CHECK( kinds == ( std::set< std::string >{ "depth", "color", "motion" } ) );
    bool has_depth = false, has_color = false, has_accel = false;
    for( auto & p : info["profiles"] )
    {
        if( p["stream"] == "depth" && p["format"] == "z16" && p["width"] == 1280 && p["height"] == 720
            && p["fps"] == 30 )
            has_depth = true;
        if( p["stream"] == "color" && p["format"] == "rgb8" && p["width"] == 1280 && p["height"] == 800 )
            has_color = true;
        if( p["stream"] == "accel" && p["width"] == 0 )
            has_accel = true;
    }
    CHECK( has_depth && has_color && has_accel );
    // The DeviceInfo must survive the IPC size limit (256 KiB of JSON).
    CHECK( info.dump().size() < 256 * 1024 );
}

TEST_CASE( playback_bag )
{
    TempDir dir;
    RecordingOptions o;
    o.path = dir.file( "head.bag" );
    o.frames = 8;
    try
    {
        write_test_recording( o );
    }
    catch( const std::exception & e )
    {
        if( std::string( e.what() ).find( ".db3" ) != std::string::npos )
            SKIP( "this librealsense build cannot write .bag files (needs patches/0002)" );
        throw;
    }
    CaptureConfig config;
    config.filters = FilterConfig::none();
    Collector col( 1 );
    auto sets = play_all( o.path, config, col );
    CHECK_MSG( sets.size() == size_t( o.frames ), sets.size() );
    REQUIRE( ! sets.empty() );
    REQUIRE( sets.front().planes.size() == 2 );
    CHECK( sets.front().planes[1].desc["kind"] == "depth_aligned" );
    AlignmentScore score = score_alignment( o, sets.front().planes[1], o.depth_units, 0.003 );
    CHECK_MSG( score.agree > 0.97, score.agree );
}

TEST_CASE( playback_filters_reduce_noise )
{
    TempDir dir;
    RecordingOptions o;
    o.path = dir.file( "noisy.db3" );
    o.frames = 12;
    o.noise_m = 0.004f;  // 1 mm at 0.5 m
    o.pace = false;
    write_test_recording( o );

    CaptureConfig config;
    config.send_depth = true;
    config.filters = FilterConfig::none();
    Collector raw( 1 );
    auto raw_sets = play_all( o.path, config, raw );
    REQUIRE( raw_sets.size() >= 10 );

    config.filters = FilterConfig();  // face-scan defaults: threshold, spatial, temporal in disparity
    Collector filtered( 1 );
    auto filtered_sets = play_all( o.path, config, filtered );
    REQUIRE( filtered_sets.size() >= 10 );

    // Compare the last frames, after the temporal filter has seen several.
    double raw_rms = head_depth_rms_mm( o, raw_sets[9].planes[2], o.depth_units );
    double filtered_rms = head_depth_rms_mm( o, filtered_sets[9].planes[2], o.depth_units );
    // The front of the head is at 0.41 m: 4 mm * 0.41^2 = 0.67 mm
    CHECK_MSG( raw_rms > 0.5 && raw_rms < 0.9, raw_rms );
    CHECK_MSG( filtered_rms > 0 && filtered_rms < raw_rms * 0.7, filtered_rms << " vs raw " << raw_rms );

    // Threshold: the wall at 1 m disappears with max 0.8 m, the head stays.
    config.filters = FilterConfig::none();
    config.filters.threshold = { true, 0.2f, 0.8f };
    Collector thresholded( 1 );
    auto t_sets = play_all( o.path, config, thresholded );
    REQUIRE( ! t_sets.empty() );
    const CapturedPlane & d = t_sets[0].planes[2];
    CHECK( z16_at( d, 20, 20 ) == 0 );  // wall corner
    CHECK( z16_at( d, 640, 360 ) != 0 );
    const CapturedPlane & r = raw_sets[0].planes[2];
    CHECK( z16_at( r, 20, 20 ) != 0 );
}

TEST_CASE( playback_set_filters_while_streaming )
{
    TempDir dir;
    // 90 frames = 3 s of real time: still playing after the first framesets, however slow the machine.
    RecordingOptions o = small_recording( dir.file( "filters.db3" ), 90 );
    write_test_recording( o );
    Collector col;
    auto engine = CaptureEngine::create_playback( o.path, true, col.callbacks() );
    engine->start();  // defaults without configure (the recording's streams are played)
    REQUIRE( col.wait_for_framesets( 3, 20s ) );
    CHECK( engine->streaming() );
    engine->set_filters( to_json( FilterConfig::none() ) );
    CHECK_THROWS_WITH( engine->set_filters( json{ { "threshold", nullptr } } ), "FilterConfiguration" );
    REQUIRE( col.wait_for_event( "playback_finished", 60s ) );
    engine->shutdown();
    engine->shutdown();  // idempotent
    CHECK_THROWS_WITH( engine->start(), "shut down" );
    CHECK( ! engine->streaming() );
}

TEST_CASE( playback_stream_restart_event )
{
    TempDir dir;
    RecordingOptions o;
    o.path = dir.file( "restart.db3" );
    o.frames = 12;
    o.restart_at = 6;
    o.pace = false;
    write_test_recording( o );
    Collector col( 100 );
    auto sets = play_all( o.path, CaptureConfig(), col );
    CHECK( sets.size() >= 10 );
    CHECK( col.event_count( "stream_restarted" ) == 1 );
    for( auto & e : col.events() )
        if( e.first == "stream_restarted" )
        {
            CHECK( e.second["stream"] == "depth" );
            CHECK( e.second["previous_frame"] == 105 );
            CHECK( e.second["frame"] == 1 );
        }
}

TEST_CASE( playback_restart_stop_and_pause )
{
    // Real-time playback paces by the file and drops framesets that the processing cannot keep up with, so this
    // test counts only what does not depend on the machine's speed: whether framesets arrive, and whether they stop.
    TempDir dir;
    RecordingOptions o = small_recording( dir.file( "paced.db3" ), 90 );
    write_test_recording( o );

    Collector col( 1000 );
    auto engine = CaptureEngine::create_playback( o.path, true, col.callbacks() );
    CaptureConfig config = config_for( o );
    config.enable_imu = false;
    config.filters = FilterConfig::none();  // the lifecycle is tested here, not the filters
    engine->configure( to_json( config ) );

    // Real time: 90 frames at 30 fps take about 3 s (more when the file was written slowly, never less).
    auto t0 = std::chrono::steady_clock::now();
    engine->start();
    REQUIRE( col.wait_for_event( "playback_finished", 60s ) );
    double seconds = std::chrono::duration< double >( std::chrono::steady_clock::now() - t0 ).count();
    CHECK_MSG( seconds > 2.5, seconds );
    CHECK_MSG( col.frameset_count() >= 3, col.frameset_count() );
    CHECK( ! engine->streaming() );

    // After the end, start() plays the file again from the beginning.
    col.clear();
    engine->start();
    REQUIRE( col.wait_for_framesets( 3, 20s ) );

    // Pause: the framesets already on their way come out, then nothing.
    engine->playback_control( true, true );
    size_t paused_at = wait_until_quiet( col, 500ms );
    std::this_thread::sleep_for( 500ms );
    CHECK_MSG( col.frameset_count() == paused_at, col.frameset_count() << " vs " << paused_at );
    CHECK( engine->streaming() );

    // A paused playback stays paused through stop and start (librealsense forgets its pause when the sensors stop;
    // the engine used to start playing while it still believed to be paused, and then ignored the next pause).
    engine->stop();
    CHECK( ! engine->streaming() );
    engine->start();
    CHECK( engine->streaming() );
    std::this_thread::sleep_for( 700ms );
    CHECK_MSG( col.frameset_count() == paused_at, "played while paused: " << col.frameset_count() << " vs "
                                                                           << paused_at );
    engine->playback_control( false, true );
    REQUIRE( col.wait_for_framesets( paused_at + 3, 20s ) );
    engine->playback_control( true, true );
    size_t paused_again = wait_until_quiet( col, 500ms );
    std::this_thread::sleep_for( 500ms );
    CHECK_MSG( col.frameset_count() == paused_again, "pause ignored: " << col.frameset_count() << " vs "
                                                                       << paused_again );
    engine->playback_control( false, true );
    REQUIRE( col.wait_for_framesets( paused_again + 3, 20s ) );

    // A user stop is not the end of the file.
    size_t stopped_before = col.event_count( "streaming_stopped" );
    engine->stop();
    CHECK( ! engine->streaming() );
    std::this_thread::sleep_for( 200ms );
    CHECK( col.event_count( "playback_finished" ) == 0 );
    CHECK( col.event_count( "streaming_stopped" ) == stopped_before + 1 );
    engine->stop();  // idempotent
    CHECK( col.event_count( "streaming_stopped" ) == stopped_before + 1 );

    // Configure while streaming: the next framesets follow it (send_depth does not need a stream restart).
    engine->start();
    REQUIRE( col.wait_for_framesets( col.frameset_count() + 3, 20s ) );
    config.send_depth = true;
    engine->configure( to_json( config ) );
    CHECK( engine->streaming() );
    size_t before = col.frameset_count();
    REQUIRE( col.wait_for_framesets( before + 3, 20s ) );
    CHECK( col.framesets().back().planes.size() == 3 );  // color, depth_aligned, depth
    engine->shutdown();
    CHECK( ! engine->streaming() );
}

TEST_CASE( playback_rejects_live_only_commands )
{
    TempDir dir;
    RecordingOptions o;
    o.path = dir.file( "cmds.db3" );
    o.frames = 2;
    o.pace = false;
    write_test_recording( o );
    Collector col;
    CHECK_THROWS_WITH( CaptureEngine::create_playback( dir.file( "missing.db3" ), true, col.callbacks() ),
                       "not found" );
    std::string txt = dir.file( "notes.txt" );
    { std::FILE * f = std::fopen( txt.c_str(), "w" ); std::fputs( "x", f ); std::fclose( f ); }
    CHECK_THROWS_WITH( CaptureEngine::create_playback( txt, true, col.callbacks() ), "Unsupported recording" );

    auto engine = CaptureEngine::create_playback( o.path, false, col.callbacks() );
    CHECK_THROWS_WITH( engine->set_option( "depth", "Laser Power", 100 ), "playback" );
    CHECK_THROWS_WITH( engine->start_recording( dir.file( "x.db3" ), "db3" ), "live camera" );
    CHECK_THROWS_WITH( engine->stop_recording(), "No recording" );
    CHECK_THROWS_WITH( engine->hardware_reset(), "playback" );
    CHECK_THROWS_WITH( engine->configure( json{ { "depth", 1 } } ), "CaptureConfiguration" );
    engine->set_auto_exposure_roi( 10, 10, 100, 100, 0.5f );  // accepted and ignored
    CHECK_THROWS_WITH( engine->set_auto_exposure_roi( 0, 0, 0, 10, 0.5f ), "Invalid" );
    engine->shutdown();
}

// ---------------------------------------------------------------------------------------------------------------
// The live code path, with a software device standing in for the camera

namespace
{
// A "live" engine around a SyntheticCamera. Frames are fed from the test thread between engine calls (librealsense
// software sensors must not receive frames while another thread starts or stops them).
struct LiveRig
{
    RecordingOptions options;
    SyntheticCamera camera;
    Collector col;
    std::unique_ptr< CaptureEngine > engine;
    int next_frame = 0;

    explicit LiveRig( const RecordingOptions & o, size_t pixels_every = 1 )
        : options( o )
        , camera( o )
        , col( pixels_every )
    {
        engine = create_live_engine_for_device( camera.device(), col.callbacks() );
    }

    // One frame at a time, each after the previous one's frameset came out (bounded wait). A live engine drops the
    // oldest synchronized set when processing falls behind, so feeding at the frame rate made the counts depend on
    // the machine (a 3-core CI runner processes 1280x720 slower than 30 fps); the recorder sees every frame either
    // way. Call warm_up() after every stream (re)start first. Returns the number of framesets that arrived.
    size_t feed( int n )
    {
        size_t start = col.frameset_count();
        for( int i = 0; i < n; ++i )
        {
            size_t before = col.frameset_count();
            camera.emit_frame( next_frame++ );
            col.wait_for_framesets( before + 1, 10s );
        }
        return col.frameset_count() - start;
    }

    // After a stream (re)start the syncer learns the stream set from the frames: the first frame's color, depth and
    // infrared come out as three sets, and when the processing thread is slow to take them the queue of 2 drops one
    // (as with a camera, the first frameset after a start may be lost; nothing else is). Feeds frames until a
    // frameset came out and the stragglers are in; returns the number of frames fed (the recorder sees them all).
    int warm_up()
    {
        size_t start = col.frameset_count();
        for( int i = 1; i <= 10; ++i )
        {
            camera.emit_frame( next_frame++ );
            if( col.wait_for_framesets( start + 1, 2s ) )
            {
                wait_until_quiet( col, 500ms );
                return i;
            }
        }
        throw std::runtime_error( "no frameset after 10 frames" );
    }

    // What a software device can do: no presets, fixed depth units, no advanced mode.
    static CaptureConfig config()
    {
        CaptureConfig c;
        c.preset = "Custom";
        c.filters = FilterConfig::none();
        return c;
    }
};

const json * find_option( const json & info, const std::string & kind, const std::string & name )
{
    for( auto & s : info["sensors"] )
        if( s["kind"] == kind )
            for( auto & o : s["options"] )
                if( o["name"] == name )
                    return &o;
    return nullptr;
}
}  // namespace

TEST_CASE( live_configure_and_options )
{
    RecordingOptions o;
    o.frames = 1;
    LiveRig rig( o );
    CaptureEngine & engine = *rig.engine;
    json info = engine.device_info();
    CHECK( info["source"] == "live" );
    CHECK( info["serial"] == k_synthetic_serial );
    CHECK( info["presets"].empty() );

    CHECK_THROWS_WITH( engine.configure( to_json( CaptureConfig() ) ),
                       "does not support visual presets ('High Accuracy')" );
    CaptureConfig c = LiveRig::config();
    c.depth = { 640, 480, 30 };
    CHECK_THROWS_WITH( engine.configure( to_json( c ) ),
                       "Depth 640x480@30 Z16 is not available (USB 3.2). Available: 1280x720@30" );
    c = LiveRig::config();
    c.color = { 1920, 1080, 30 };
    CHECK_THROWS_WITH( engine.configure( to_json( c ) ), "Color 1920x1080@30 RGB8 is not available" );
    c = LiveRig::config();
    c.depth_units = 0.001f;
    CHECK_THROWS_WITH( engine.configure( to_json( c ) ), "Depth Units cannot be changed" );
    c = LiveRig::config();
    c.disparity_shift = 50;
    CHECK_THROWS_WITH( engine.configure( to_json( c ) ), "Disparity shift needs the camera's advanced mode" );
    c = LiveRig::config();
    c.laser_power = 400.f;
    CHECK_THROWS_WITH( engine.configure( to_json( c ) ), "Laser Power: value out of range [0, 360]" );

    // Valid: laser power null = the maximum, values snap to the option's step, emitter and auto exposure apply.
    c = LiveRig::config();
    engine.configure( to_json( c ) );
    CHECK( ( *find_option( engine.device_info(), "depth", "Laser Power" ) )["value"] == 360 );
    c.laser_power = 100.f;
    c.depth_auto_exposure = false;
    c.color_auto_exposure = false;
    engine.configure( to_json( c ) );
    info = engine.device_info();
    CHECK( ( *find_option( info, "depth", "Laser Power" ) )["value"] == 90 );
    CHECK( ( *find_option( info, "depth", "Enable Auto Exposure" ) )["value"] == 0 );
    CHECK( ( *find_option( info, "color", "Enable Auto Exposure" ) )["value"] == 0 );
    CHECK( ( *find_option( info, "depth", "Emitter Enabled" ) )["value"] == 1 );
    c.emitter_enabled = false;
    engine.configure( to_json( c ) );
    CHECK( ( *find_option( engine.device_info(), "depth", "Emitter Enabled" ) )["value"] == 0 );

    CHECK_THROWS_WITH( engine.set_option( "depth", "Laser Power", 500 ), "Laser Power: value out of range [0, 360]" );
    CHECK_THROWS_WITH( engine.set_option( "depth", "Depth Units", 0.001f ), "Depth Units is read-only" );
    CHECK_THROWS_WITH( engine.set_option( "depth", "Warp Drive", 1 ), "has no option 'Warp Drive'" );
    CHECK_THROWS_WITH( engine.set_option( "left", "Laser Power", 1 ), "Unknown sensor 'left'" );
    CHECK_THROWS_WITH( engine.set_option( "depth", "Laser Power", std::nanf( "" ) ), "not a number" );
    engine.set_option( "color", "exposure", 200 );
    CHECK( ( *find_option( engine.device_info(), "color", "Exposure" ) )["value"] == 200 );
    CHECK_THROWS_WITH( engine.playback_control( true, true ), "only available when playing" );
    engine.shutdown();
}

TEST_CASE( live_streaming_and_recording )
{
    TempDir dir;
    RecordingOptions o;
    o.frames = 1000;
    LiveRig rig( o, 1 );
    CaptureEngine & engine = *rig.engine;
    CaptureConfig c = LiveRig::config();
    c.enable_ir = true;
    c.send_ir = true;
    engine.configure( to_json( c ) );
    engine.set_auto_exposure_roi( 500, 220, 300, 380, 0.5f );  // no ROI support on the software device: ignored
    engine.start();
    CHECK( engine.streaming() );
    rig.warm_up();
    size_t fed = rig.feed( 10 );
    CHECK_MSG( fed == 10, fed );
    auto sets = rig.col.framesets();
    REQUIRE( ! sets.empty() );
    CHECK( sets.back().planes.size() == 3 );  // color, depth_aligned, ir_left
    CHECK( sets.back().metadata["imu"].is_object() );
    AlignmentScore score = score_alignment( o, sets.back().planes[1], o.depth_units, 0.003 );
    CHECK_MSG( score.agree > 0.97, score.agree );

    CHECK_THROWS_WITH( engine.start_recording( dir.file( "a.mp4" ), "mp4" ), "Unknown recording format" );
    CHECK_THROWS_WITH( engine.start_recording( dir.file( "a.bag" ), "db3" ), "must end with .db3" );
    CHECK_THROWS_WITH( engine.stop_recording(), "No recording" );

    // .db3 while streaming: the streams restart with the recorder and again without it.
    std::string db3 = dir.file( "live.db3" );
    engine.start_recording( db3, "db3" );
    CHECK( engine.streaming() );
    int db3_frames = rig.warm_up();
    CHECK( rig.col.event_count( "recording_started" ) == 1 );
    CHECK_THROWS_WITH( engine.start_recording( dir.file( "b.db3" ), "db3" ), "already running" );
    CaptureConfig other = c;
    other.enable_ir = false;
    other.send_ir = false;
    CHECK_THROWS_WITH( engine.configure( to_json( other ) ), "Stop the recording before changing the streams" );
    engine.set_filters( to_json( FilterConfig() ) );  // filters may change while recording
    engine.set_filters( to_json( FilterConfig::none() ) );
    fed = rig.feed( 15 );
    CHECK_MSG( fed == 15, fed );
    db3_frames += 15;
    engine.stop_recording();
    CHECK( engine.streaming() );
    REQUIRE( rig.col.event_count( "recording_finished" ) == 1 );
    uint64_t bytes = 0;
    for( auto & e : rig.col.events() )
        if( e.first == "recording_finished" )
            bytes = e.second["bytes"].get< uint64_t >();
    CHECK( bytes > 0 && bytes == fs::file_size( db3 ) );
    rig.warm_up();
    fed = rig.feed( 5 );
    CHECK_MSG( fed == 5, fed );

    // .bag needs patches/0002 in the SDK build.
    std::string bag = dir.file( "live.bag" );
    bool bag_supported = true;
    int bag_frames = 0;
    try
    {
        engine.start_recording( bag, "bag" );
    }
    catch( const std::exception & e )
    {
        CHECK_MSG( std::string( e.what() ).find( "patches/0002" ) != std::string::npos, e.what() );
        bag_supported = false;
    }
    CHECK( engine.streaming() );
    if( bag_supported )
    {
        bag_frames = rig.warm_up();
        fed = rig.feed( 10 );
        CHECK_MSG( fed == 10, fed );
        bag_frames += 10;
        engine.stop_recording();
    }
    engine.stop();
    CHECK( ! engine.streaming() );
    engine.shutdown();

    // The recordings play back through the same engine code.
    for( const std::string & file : { db3, bag } )
    {
        if( file == bag && ! bag_supported )
            continue;
        Collector col( 1 );
        CaptureConfig pc;
        pc.enable_ir = true;
        pc.send_ir = true;
        pc.filters = FilterConfig::none();
        auto played = play_all( file, pc, col );
        // The recorder takes every frame from the sensors; a non-real-time playback delivers every one of them.
        size_t expected = size_t( file == db3 ? db3_frames : bag_frames );
        CHECK_MSG( played.size() == expected, file << ": " << played.size() << " of " << expected );
        REQUIRE( ! played.empty() );
        CHECK( played.front().planes.size() == 3 );
        check_intrinsics( played.front().planes[0].desc["intrinsics"], rig.camera.info().color_intrinsics );
        CHECK( played.front().metadata["imu"].is_object() );
        AlignmentScore s = score_alignment( o, played.front().planes[1], o.depth_units, 0.003 );
        CHECK_MSG( s.agree > 0.97, s.agree );
    }
}

TEST_CASE( live_depth_units_change_while_streaming )
{
    // DEPTH_UNITS changed while streaming (SET_OPTION, or CONFIGURE with the same streams) keeps the stream profile.
    // librealsense's disparity transforms and NEON align cache the depth scale per profile, so the aligned depth was
    // reprojected with the old scale (10x off) and the disparity-domain filters used thresholds off by 10x.
    RecordingOptions o;
    o.frames = 1000;
    LiveRig rig( o, 1 );
    CaptureEngine & engine = *rig.engine;
    CaptureConfig c = LiveRig::config();
    c.send_depth = true;
    c.filters = FilterConfig();  // face-scan filters: threshold, spatial and temporal in the disparity domain
    engine.configure( to_json( c ) );
    engine.start();
    rig.warm_up();
    CHECK( rig.feed( 6 ) == 6 );

    auto check_last = [&]( float units, const char * what ) {
        auto sets = rig.col.framesets();
        REQUIRE( ! sets.empty() );
        const CapturedFrameset & last = sets.back();
        CHECK_MSG( near( last.metadata["depth_scale"].get< double >(), units, 1e-9 ), what );
        RecordingOptions truth = o;
        truth.depth_units = units;
        const CapturedPlane * depth = last.plane( "depth" );
        const CapturedPlane * aligned = last.plane( "depth_aligned" );
        REQUIRE( depth && aligned );
        double rms = head_depth_rms_mm( truth, *depth, units );
        CHECK_MSG( rms >= 0 && rms < 0.6, what << ": depth RMS " << rms << " mm" );
        // Noise-free scene: 0.996 agree, 0.89 coverage (the RGB camera sees more than the depth imagers); with the
        // stale scale 0.93 and 0.61.
        AlignmentScore score = score_alignment( truth, *aligned, units, 0.003 );
        CHECK_MSG( score.agree > 0.98 && score.coverage > 0.85,
                   what << ": aligned agree " << score.agree << ", coverage " << score.coverage );
    };
    check_last( 0.0001f, "0.1 mm units" );

    rig.camera.set_depth_units( 0.001f );
    CHECK( rig.feed( 6 ) == 6 );
    check_last( 0.001f, "after the change to 1 mm units" );
    engine.shutdown();
}

TEST_CASE( live_device_events_and_shutdown )
{
    // A recording loaded into the context the engine watches stands in for the camera: unloading it is an unplug,
    // loading it again a reconnect (librealsense raises the devices-changed callback for both).
    TempDir dir;
    RecordingOptions o = small_recording( dir.file( "device.db3" ), 3 );
    o.pace = false;
    write_test_recording( o );

    std::mutex mutex;
    std::condition_variable cv;
    int disconnects_entered = 0, disconnects_left = 0;
    Collector col;
    CaptureCallbacks cb = col.callbacks();
    auto record = cb.on_event;
    cb.on_event = [&, record]( const std::string & event, const json & data ) {
        record( event, data );
        if( event != "device_disconnected" )
            return;
        {
            std::lock_guard< std::mutex > lock( mutex );
            ++disconnects_entered;
        }
        cv.notify_all();
        std::this_thread::sleep_for( 300ms );  // e.g. the IPC layer queueing the event
        std::lock_guard< std::mutex > lock( mutex );
        ++disconnects_left;
    };

    rs2::context ctx;
    rs2::device camera = ctx.load_device( o.path );
    auto engine = create_live_engine_for_device( ctx, camera, cb );
    CHECK( engine->device_info()["serial"] == k_synthetic_serial );

    // Unplugged: the event, and commands fail until the camera is back.
    ctx.unload_device( o.path );
    CHECK( col.event_count( "device_disconnected" ) == 1 );
    CHECK_THROWS_WITH( engine->device_info(), "disconnected" );
    CHECK( ! engine->streaming() );

    // Plugged in again: the event, and the next command adopts the new device instance.
    ctx.load_device( o.path );
    CHECK( col.event_count( "device_connected" ) == 1 );
    CHECK( engine->device_info()["serial"] == k_synthetic_serial );

    // A hardware reset re-enumerates the camera (played here by the test): it waits for the new instance and
    // adopts it, and the intentional re-enumeration is not reported as a disconnect.
    std::thread reenumerate( [&] {
        std::this_thread::sleep_for( 200ms );
        ctx.unload_device( o.path );
        ctx.load_device( o.path );
    } );
    engine->hardware_reset();
    reenumerate.join();
    CHECK( col.event_count( "device_disconnected" ) == 1 );
    CHECK( col.event_count( "device_connected" ) == 1 );
    CHECK( engine->device_info()["serial"] == k_synthetic_serial );

    // Unplugged while shutting down: shutdown() waits for the event being emitted and no callback runs after it
    // returned (the callbacks capture objects the caller destroys after shutdown(), e.g. the IPC service).
    std::thread unplug( [&] { ctx.unload_device( o.path ); } );
    {
        std::unique_lock< std::mutex > lock( mutex );
        REQUIRE( cv.wait_for( lock, 10s, [&] { return disconnects_entered == 2; } ) );
    }
    engine->shutdown();
    int left = 0;
    {
        std::lock_guard< std::mutex > lock( mutex );
        left = disconnects_left;
    }
    unplug.join();
    CHECK_MSG( left == 2, "shutdown() returned while a device_disconnected event was still being emitted" );
    size_t events = col.events().size();
    ctx.load_device( o.path );  // nothing may reach the callbacks any more
    std::this_thread::sleep_for( 100ms );
    CHECK( col.events().size() == events );
}

TEST_CASE( live_recording_ended_by_an_unplug_still_stops )
{
    // The camera unplugged mid-recording: the next command finishes the recording on the engine's side. The app's
    // RECORD_STOP that follows must succeed (the IPC layer then delivers the file and reports recording_finished),
    // not fail with "No recording is running".
    TempDir dir;
    RecordingOptions o;
    o.frames = 1000;
    LiveRig rig( o, 1 );
    CaptureEngine & engine = *rig.engine;
    engine.configure( to_json( LiveRig::config() ) );
    engine.start();
    rig.warm_up();
    const std::string file = dir.file( "unplugged.db3" );
    engine.start_recording( file, "db3" );
    rig.warm_up();
    CHECK( rig.feed( 5 ) == 5 );

    mark_device_lost_for_testing( engine );
    CHECK_THROWS_WITH( engine.device_info(), "disconnected" );
    CHECK( rig.col.event_count( "recording_finished" ) == 1 );
    CHECK( ! engine.streaming() );
    engine.stop_recording();
    CHECK( fs::exists( file ) && fs::file_size( file ) > 0 );
    // Only once: a second stop has nothing to stop.
    CHECK_THROWS_WITH( engine.stop_recording(), "No recording" );
    engine.shutdown();
}

// ---------------------------------------------------------------------------------------------------------------
// Throughput (printed for docs/CAMERA.md; the bounds only catch gross regressions)

TEST_CASE( bench_processing_and_recording )
{
    TempDir dir;
    RecordingOptions o;
    o.frames = 90;
    o.pace = false;
    o.imu = false;

    // Recording: uncompressed and compressed .db3 of 1280x720 Z16 + 1280x800 RGB8 (the scene is rendered once).
    for( bool compress : { false, true } )
    {
        o.path = dir.file( compress ? "bench-z.db3" : "bench.db3" );
        o.compress = compress;
        auto t0 = std::chrono::steady_clock::now();
        write_test_recording( o );
        double s = std::chrono::duration< double >( std::chrono::steady_clock::now() - t0 ).count();
        double mb = double( fs::file_size( o.path ) ) / ( 1024.0 * 1024.0 );
        std::printf( "    record .db3 %s: %d frames in %.2f s (%.0f fps), %.2f MB/frame\n",
                     compress ? "compressed  " : "uncompressed", o.frames, s, o.frames / s, mb / o.frames );
    }

    // Processing: playback without real time is bounded by the reader and the processing thread.
    o.path = dir.file( "bench.db3" );
    std::vector< std::pair< const char *, FilterConfig > > variants;
    variants.emplace_back( "no filters", FilterConfig::none() );
    FilterConfig f = FilterConfig::none();
    f.threshold.enabled = true;
    variants.emplace_back( "threshold", f );
    f = FilterConfig::none();
    f.spatial.enabled = true;
    variants.emplace_back( "spatial (disparity)", f );
    f = FilterConfig::none();
    f.temporal.enabled = true;
    variants.emplace_back( "temporal (disparity)", f );
    f = FilterConfig::none();
    f.hole_filling.enabled = true;
    variants.emplace_back( "hole filling", f );
    variants.emplace_back( "face-scan filters", FilterConfig() );
    double none_ms = 0;
    for( auto & v : variants )
    {
        CaptureConfig config;
        config.enable_imu = false;
        config.filters = v.second;
        Collector col( 1000000 );
        auto t0 = std::chrono::steady_clock::now();
        auto sets = play_all( o.path, config, col );
        double s = std::chrono::duration< double >( std::chrono::steady_clock::now() - t0 ).count();
        double ms = s * 1000.0 / std::max< size_t >( sets.size(), 1 );
        if( v.second.threshold.enabled == false && ! v.second.spatial.enabled && ! v.second.temporal.enabled
            && ! v.second.hole_filling.enabled )
            none_ms = ms;
        std::printf( "    playback + %-22s + align: %5.1f ms/frameset (%3.0f fps), filters %+5.1f ms\n", v.first, ms,
                     1000.0 / ms, ms - none_ms );
        CHECK( sets.size() == size_t( o.frames ) );
        CHECK_MSG( 1000.0 / ms > 5, ms );  // only gross regressions: CI machines are slow
    }
}

TEST_CASE( probe_reports_that_it_tried_the_live_camera )
{
    // main.cpp hands the camera back to macOS after a live probe; a probe that failed to open the camera (none is
    // connected here, and tests never run as root) may still have captured part of it, so it counts as well.
    std::vector< std::string > args{ "--enumerate", "--json" };
    std::vector< char * > argv;
    for( auto & a : args )
        argv.push_back( a.data() );
    bool live = false;
    std::fflush( stdout );
    int saved = ::dup( 1 );
    REQUIRE( saved >= 0 );
    std::FILE * null = std::fopen( "/dev/null", "w" );
    REQUIRE( null );
    ::dup2( ::fileno( null ), 1 );
    int rc = run_probe( int( argv.size() ), argv.data(), &live );
    std::fflush( stdout );
    ::dup2( saved, 1 );
    ::close( saved );
    std::fclose( null );
    CHECK( live );
    if( ::geteuid() != 0 )
        CHECK_MSG( rc != 0, "no camera can be opened without root" );
}

TEST_CASE( probe_runs_on_a_recording )
{
    TempDir dir;
    RecordingOptions o;
    o.path = dir.file( "wall.db3" );
    o.scene = "plane";
    o.distance_m = 0.6f;
    o.noise_m = 0.002f;
    o.frames = 20;
    o.pace = false;
    o.imu = false;
    write_test_recording( o );
    std::string out = dir.file( "probe.json" );
    std::vector< std::string > args{ "--playback", o.path, "--json", "--frames", "8", "--warmup", "2",
                                     "--configs", "1280x720:ha:0" };
    std::vector< char * > argv;
    for( auto & a : args )
        argv.push_back( a.data() );
    // The probe prints its report to stdout (it never writes files: it runs as root).
    std::fflush( stdout );
    int saved = ::dup( 1 );
    std::FILE * capture = std::fopen( out.c_str(), "w" );
    REQUIRE( saved >= 0 && capture );
    ::dup2( ::fileno( capture ), 1 );
    bool live = true;
    int rc = run_probe( int( argv.size() ), argv.data(), &live );
    std::fflush( stdout );
    ::dup2( saved, 1 );
    ::close( saved );
    std::fclose( capture );
    REQUIRE( rc == 0 );
    CHECK_MSG( ! live, "a recording needs no USB hand-back" );
    std::FILE * f = std::fopen( out.c_str(), "r" );
    REQUIRE( f );
    std::string text;
    char buf[4096];
    size_t n;
    while( ( n = std::fread( buf, 1, sizeof buf, f ) ) > 0 )
        text.append( buf, n );
    std::fclose( f );
    json report = json::parse( text );
    REQUIRE( report["results"].size() == 1 );
    const json & raw = report["results"][0]["raw"];
    // Noise 2 mm at 1 m -> 0.72 mm at 0.6 m on a flat target; the plane fit sees it, the median sees 0.6 m.
    CHECK_MSG( near( raw["median_m"].get< double >(), 0.6 / std::cos( 10 * 3.14159265 / 180 ), 0.02 ), raw.dump() );
    CHECK_MSG( raw["plane_rms_mm"].get< double >() > 0.5 && raw["plane_rms_mm"].get< double >() < 1.0, raw.dump() );
    double temporal = raw["temporal_noise_mm"].get< double >();
    CHECK_MSG( temporal > 0.5 && temporal < 1.0, raw.dump() );
    CHECK( raw["fill_rate"].get< double >() > 0.99 );
    const json & filtered = report["results"][0]["filtered"];
    CHECK_MSG( filtered["plane_rms_mm"].get< double >() < raw["plane_rms_mm"].get< double >(), filtered.dump() );
}

int main( int argc, char ** argv )
{
    const char * filter = argc > 1 ? argv[1] : nullptr;
    int failed = 0, passed = 0, skipped = 0;
    for( auto & t : registry() )
    {
        if( filter && ! std::strstr( t.name, filter ) )
            continue;
        std::printf( "[ RUN  ] %s\n", t.name );
        std::fflush( stdout );
        int before = failed_checks();
        auto t0 = std::chrono::steady_clock::now();
        std::string status = "PASS";
        try
        {
            // Benchmarks measure (docs/CAMERA.md) rather than test, and take a minute on a slow CI runner: they run
            // only when asked for by name, e.g. "capture-tests bench".
            if( ! filter && std::strncmp( t.name, "bench_", 6 ) == 0 )
                SKIP( "benchmark: run 'capture-tests bench'" );
            t.body();
        }
        catch( const Skipped & e )
        {
            status = std::string( "SKIP (" ) + e.what() + ")";
        }
        catch( const std::exception & e )
        {
            std::fprintf( stderr, "    exception: %s\n", e.what() );
            ++failed_checks();
        }
        double ms = std::chrono::duration< double, std::milli >( std::chrono::steady_clock::now() - t0 ).count();
        if( failed_checks() != before )
        {
            status = "FAIL";
            ++failed;
        }
        else if( status == "PASS" )
            ++passed;
        else
            ++skipped;
        std::printf( "[ %s ] %s (%.0f ms)\n", status.c_str(), t.name, ms );
        std::fflush( stdout );
    }
    std::printf( "%d passed, %d failed, %d skipped\n", passed, failed, skipped );
    return failed ? 1 : 0;
}
