// License: Apache 2.0. See LICENSE file in root directory.
//
// Tests of the capture module without a camera: configuration validation, helpers, measurement math, and the
// CaptureEngine end to end on synthetic recordings (software device -> .db3/.bag -> playback -> framesets).

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
    json good = to_json( CaptureConfig() );
    auto with = [&]( const std::function< void( json & ) > & change ) {
        json j = good;
        change( j );
        return j;
    };
    CHECK_THROWS_WITH( parse_capture_config( json::array() ), "expected a JSON object" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["bogus"] = 1; } ) ), "CaptureConfiguration.bogus: unknown key" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j.erase( "preset" ); } ) ), "CaptureConfiguration.preset: missing" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["depth"]["width"] = "1280"; } ) ), "depth.width: expected an integer" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["depth"]["fps"] = 29.5; } ) ), "depth.fps: expected an integer" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["depth"]["height"] = 0; } ) ), "out of range" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["enable_ir"] = 1; } ) ), "enable_ir: expected true or false" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["preset"] = ""; } ) ), "preset" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["preset"] = "High\nAccuracy"; } ) ), "control characters" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["laser_power"] = -1; } ) ), "laser_power" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["depth_units"] = 0.5; } ) ), "depth_units" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["disparity_shift"] = 600; } ) ), "disparity_shift" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["align_to_color"] = false; } ) ), "align_to_color" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["send_ir"] = true; } ) ), "send_ir: requires enable_ir" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["filters"]["spatial"]["alpha"] = 2; } ) ),
                       "CaptureConfiguration.filters.spatial.alpha: 2 is out of range [0.25, 1]" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["filters"]["temporal"]["persistence"] = 9; } ) ), "persistence" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["filters"]["hole_filling"]["mode"] = 3; } ) ), "mode" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["filters"]["threshold"]["min_m"] = 2.0; } ) ),
                       "min_m must be smaller than max_m" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["filters"]["spatial"]["extra"] = 0; } ) ), "unknown key" );
    CHECK_THROWS_WITH( parse_capture_config( with( []( json & j ) { j["depth_units"] = nullptr; } ) ), "expected a number" );
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
    CHECK( ! none.threshold.enabled && ! none.spatial.enabled && ! none.temporal.enabled && ! none.hole_filling.enabled );
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
            CHECK( near( md["depth_to_color"]["translation"][k].get< double >(), info.depth_to_color.translation[k], 1e-6 ) );
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
        if( p["stream"] == "depth" && p["format"] == "z16" && p["width"] == 1280 && p["height"] == 720 && p["fps"] == 30 )
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
    RecordingOptions o;
    o.path = dir.file( "filters.db3" );
    o.frames = 30;
    o.pace = true;
    write_test_recording( o );
    Collector col;
    auto engine = CaptureEngine::create_playback( o.path, true, col.callbacks() );
    engine->start();  // defaults without configure
    REQUIRE( col.wait_for_framesets( 3, 10s ) );
    CHECK( engine->streaming() );
    engine->set_filters( to_json( FilterConfig::none() ) );
    CHECK_THROWS_WITH( engine->set_filters( json{ { "threshold", nullptr } } ), "FilterConfiguration" );
    REQUIRE( col.wait_for_event( "playback_finished", 20s ) );
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
    TempDir dir;
    RecordingOptions o;
    o.path = dir.file( "paced.db3" );
    o.frames = 45;
    write_test_recording( o );

    Collector col( 1000 );
    auto engine = CaptureEngine::create_playback( o.path, true, col.callbacks() );
    CaptureConfig config;
    config.enable_imu = false;
    engine->configure( to_json( config ) );

    // Real time: 45 frames at 30 fps take about 1.5 s.
    auto t0 = std::chrono::steady_clock::now();
    engine->start();
    REQUIRE( col.wait_for_event( "playback_finished", 20s ) );
    double seconds = std::chrono::duration< double >( std::chrono::steady_clock::now() - t0 ).count();
    CHECK_MSG( seconds > 1.0 && seconds < 4.0, seconds );
    CHECK( col.frameset_count() >= size_t( o.frames - 2 ) );

    // After the end, start() plays the file again from the beginning.
    col.clear();
    engine->start();
    REQUIRE( col.wait_for_framesets( 5, 10s ) );
    engine->playback_control( true, true );
    std::this_thread::sleep_for( 200ms );
    size_t paused_at = col.frameset_count();
    std::this_thread::sleep_for( 400ms );
    CHECK_MSG( col.frameset_count() <= paused_at + 1, col.frameset_count() << " vs " << paused_at );
    engine->playback_control( false, true );
    REQUIRE( col.wait_for_framesets( paused_at + 5, 10s ) );

    // A user stop is not the end of the file.
    engine->stop();
    CHECK( ! engine->streaming() );
    std::this_thread::sleep_for( 200ms );
    CHECK( col.event_count( "playback_finished" ) == 0 );
    CHECK( col.event_count( "streaming_stopped" ) == 1 );
    engine->stop();  // idempotent
    CHECK( col.event_count( "streaming_stopped" ) == 1 );

    // Configure while streaming with other streams restarts them; filters alone do not.
    engine->start();
    REQUIRE( col.wait_for_framesets( col.frameset_count() + 3, 10s ) );
    config.send_depth = true;
    engine->configure( to_json( config ) );
    CHECK( engine->streaming() );
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
    CHECK_THROWS_WITH( CaptureEngine::create_playback( dir.file( "missing.db3" ), true, col.callbacks() ), "not found" );
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

    // At the frame rate: frames that come faster than the processing thread takes them are dropped (live
    // behavior), which would make counts flaky.
    void feed( int n )
    {
        for( int i = 0; i < n; ++i )
        {
            camera.emit_frame( next_frame++ );
            std::this_thread::sleep_for( std::chrono::milliseconds( 1000 / options.fps ) );
        }
        std::this_thread::sleep_for( 100ms );
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

    CHECK_THROWS_WITH( engine.configure( to_json( CaptureConfig() ) ), "does not support visual presets ('High Accuracy')" );
    CaptureConfig c = LiveRig::config();
    c.depth = { 640, 480, 30 };
    CHECK_THROWS_WITH( engine.configure( to_json( c ) ), "Depth 640x480@30 Z16 is not available (USB 3.2). Available: 1280x720@30" );
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
    rig.feed( 10 );
    CHECK_MSG( rig.col.frameset_count() >= 8, rig.col.frameset_count() );
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
    CHECK( rig.col.event_count( "recording_started" ) == 1 );
    CHECK_THROWS_WITH( engine.start_recording( dir.file( "b.db3" ), "db3" ), "already running" );
    CaptureConfig other = c;
    other.enable_ir = false;
    other.send_ir = false;
    CHECK_THROWS_WITH( engine.configure( to_json( other ) ), "Stop the recording before changing the streams" );
    engine.set_filters( to_json( FilterConfig() ) );  // filters may change while recording
    engine.set_filters( to_json( FilterConfig::none() ) );
    rig.feed( 15 );
    engine.stop_recording();
    CHECK( engine.streaming() );
    REQUIRE( rig.col.event_count( "recording_finished" ) == 1 );
    uint64_t bytes = 0;
    for( auto & e : rig.col.events() )
        if( e.first == "recording_finished" )
            bytes = e.second["bytes"].get< uint64_t >();
    CHECK( bytes > 0 && bytes == fs::file_size( db3 ) );
    size_t before = rig.col.frameset_count();
    rig.feed( 5 );
    CHECK( rig.col.frameset_count() >= before + 4 );

    // .bag needs patches/0002 in the SDK build.
    std::string bag = dir.file( "live.bag" );
    bool bag_supported = true;
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
        rig.feed( 10 );
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
        size_t expected = file == db3 ? 15 : 10;
        CHECK_MSG( played.size() + 2 >= expected && played.size() <= expected, file << ": " << played.size() );
        REQUIRE( ! played.empty() );
        CHECK( played.front().planes.size() == 3 );
        check_intrinsics( played.front().planes[0].desc["intrinsics"], rig.camera.info().color_intrinsics );
        CHECK( played.front().metadata["imu"].is_object() );
        AlignmentScore s = score_alignment( o, played.front().planes[1], o.depth_units, 0.003 );
        CHECK_MSG( s.agree > 0.97, s.agree );
    }
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
        CHECK_MSG( 1000.0 / ms > 15, ms );
    }
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
    int rc = run_probe( int( argv.size() ), argv.data() );
    std::fflush( stdout );
    ::dup2( saved, 1 );
    ::close( saved );
    std::fclose( capture );
    REQUIRE( rc == 0 );
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
    CHECK_MSG( raw["temporal_noise_mm"].get< double >() > 0.5 && raw["temporal_noise_mm"].get< double >() < 1.0, raw.dump() );
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
