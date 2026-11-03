// License: Apache 2.0. See LICENSE file in root directory.
//
// Camera measurement tool: `IntelFaceScanHelper --probe ...` (live camera: run with sudo). Enumerates the device
// and measures depth quality on a flat target (a wall) for candidate configurations, through the same
// CaptureEngine and filters the app uses. Results print as markdown tables plus JSON for docs/CAMERA.md.

#include "probe.h"

#include "capture_config.h"
#include "capture_engine.h"
#include "capture_measure.h"
#include "capture_util.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using nlohmann::json;

namespace ifs
{
namespace
{
std::atomic< bool > g_interrupted{ false };

extern "C" void probe_on_signal( int )
{
    g_interrupted = true;
}

struct Interrupted : std::runtime_error
{
    Interrupted()
        : std::runtime_error( "interrupted" )
    {
    }
};

// Restores the previous handlers when the probe returns (main may install its own).
class SignalGuard
{
public:
    SignalGuard()
    {
        g_interrupted = false;
        struct sigaction sa = {};
        sa.sa_handler = probe_on_signal;
        sigemptyset( &sa.sa_mask );
        for( size_t i = 0; i < 3; ++i )
            sigaction( k_signals[i], &sa, &m_previous[i] );
    }
    ~SignalGuard()
    {
        for( size_t i = 0; i < 3; ++i )
            sigaction( k_signals[i], &m_previous[i], nullptr );
    }

private:
    static constexpr int k_signals[3] = { SIGINT, SIGTERM, SIGHUP };
    struct sigaction m_previous[3];
};

struct ProbeConfig
{
    std::string name;
    StreamConfig depth;
    std::string preset;
    int disparity_shift = 0;
};

struct Options
{
    bool json_only = false;
    int frames = 30;
    int warmup = 30;
    double roi = 0.3;
    std::vector< ProbeConfig > configs;
    bool raw = true;
    bool filtered = true;
    bool enumerate_only = false;
    double minz_seconds = 0;
    std::string playback;
};

void usage( std::FILE * out )
{
    std::fprintf( out,
                  "usage: IntelFaceScanHelper --probe [options]      (live camera: sudo; Ctrl-C stops cleanly)\n"
                  "  Point the camera at a flat, textured wall, perpendicular, and keep both still.\n"
                  "  --json             print only the JSON report\n"
                  "  --frames N         frames per measurement (default 30)\n"
                  "  --warmup N         frames skipped after a configuration change (default 30)\n"
                  "  --roi F            central measurement region, fraction of width and height (default 0.3)\n"
                  "  --configs LIST     comma-separated WxH:preset:shift, preset = ha|hd|md|default|<name>\n"
                  "                     (default: 1280x720 and 848x480 x ha,hd x shift 0,50,100)\n"
                  "  --raw-only         measure without filters only\n"
                  "  --filtered-only    measure with the face-scan filters only (threshold off)\n"
                  "  --enumerate        device, options and stream profiles only\n"
                  "  --minz SECONDS     Min-Z test with the first configuration: move the wall slowly toward the\n"
                  "                     camera; reports the nearest distance with >= 90%% valid ROI pixels\n"
                  "  --playback FILE    measure a recording instead of the camera (development)\n" );
}

std::string preset_name( const std::string & s )
{
    static const std::map< std::string, std::string > short_names = {
        { "ha", "High Accuracy" }, { "hd", "High Density" }, { "md", "Medium Density" },
        { "default", "Default" },  { "hand", "Hand" },       { "custom", "Custom" },
    };
    std::string lower;
    for( char c : s )
        lower += char( std::tolower( static_cast< unsigned char >( c ) ) );
    auto it = short_names.find( lower );
    if( it != short_names.end() )
        return it->second;
    std::string name = s;
    std::replace( name.begin(), name.end(), '-', ' ' );
    std::replace( name.begin(), name.end(), '_', ' ' );
    return name;
}

bool parse_config( const std::string & spec, ProbeConfig & out )
{
    int w = 0, h = 0, shift = 0;
    char preset[64] = {};
    if( std::sscanf( spec.c_str(), "%dx%d:%63[^:]:%d", &w, &h, preset, &shift ) != 4 || w <= 0 || h <= 0
        || shift < 0 || shift > 511 )
        return false;
    out.depth = { w, h, 30 };
    out.preset = preset_name( preset );
    out.disparity_shift = shift;
    out.name = std::to_string( w ) + "x" + std::to_string( h ) + " " + out.preset + " shift " + std::to_string( shift );
    return true;
}

std::vector< ProbeConfig > default_configs()
{
    std::vector< ProbeConfig > list;
    for( const char * res : { "1280x720", "848x480" } )
        for( const char * preset : { "ha", "hd" } )
            for( int shift : { 0, 50, 100 } )
            {
                ProbeConfig c;
                parse_config( std::string( res ) + ":" + preset + ":" + std::to_string( shift ), c );
                list.push_back( c );
            }
    return list;
}

// A copy of one depth plane (the probe keeps whole sequences of frames).
struct DepthFrame
{
    std::vector< uint16_t > pixels;
    int width = 0, height = 0;
    float scale = 0.001f;
    float fx = 0, fy = 0, ppx = 0, ppy = 0;

    DepthView view() const
    {
        return DepthView{ pixels.data(), width, height, size_t( width ) * 2, scale, fx, fy, ppx, ppy };
    }
};

class FrameQueue
{
public:
    CaptureCallbacks callbacks( bool quiet )
    {
        CaptureCallbacks cb;
        cb.on_frameset = [this]( Frameset && fs ) {
            const Plane * depth = nullptr;
            for( auto & p : fs.planes )
                if( p.desc.value( "kind", "" ) == "depth" )
                    depth = &p;
            if( ! depth || ! depth->data )
                return;
            DepthFrame f;
            f.width = depth->desc.value( "width", 0 );
            f.height = depth->desc.value( "height", 0 );
            size_t stride = depth->desc.value( "stride", size_t( 0 ) );
            if( f.width <= 0 || f.height <= 0 || stride < size_t( f.width ) * 2
                || depth->bytes < stride * size_t( f.height ) )
                return;
            f.scale = fs.metadata.value( "depth_scale", 0.001f );
            const json & in = depth->desc["intrinsics"];
            f.fx = in.value( "fx", 0.f );
            f.fy = in.value( "fy", 0.f );
            f.ppx = in.value( "ppx", 0.f );
            f.ppy = in.value( "ppy", 0.f );
            f.pixels.resize( size_t( f.width ) * f.height );
            for( int y = 0; y < f.height; ++y )
                std::copy_n( reinterpret_cast< const uint16_t * >( depth->data + size_t( y ) * stride ), f.width,
                             f.pixels.data() + size_t( y ) * f.width );
            std::lock_guard< std::mutex > lock( m_mutex );
            m_frames.push_back( std::move( f ) );
            while( m_frames.size() > 8 )
                m_frames.pop_front();
            m_cv.notify_all();
        };
        cb.on_event = [this]( const std::string & event, const json & ) {
            std::lock_guard< std::mutex > lock( m_mutex );
            if( event == "playback_finished" )
                m_finished = true;
            else if( event == "device_disconnected" )
                m_error = "The camera was disconnected";
            m_cv.notify_all();
        };
        cb.on_log = [quiet]( const std::string & level, const std::string & message ) {
            if( ! quiet || level == "error" )
                std::fprintf( stderr, "[%s] %s\n", level.c_str(), message.c_str() );
        };
        return cb;
    }

    void clear()
    {
        std::lock_guard< std::mutex > lock( m_mutex );
        m_frames.clear();
    }

    // Next frame, or nullopt after 'timeout'; 'finished' tells that a playback reached its end.
    std::optional< DepthFrame > pop( std::chrono::milliseconds timeout, bool & finished )
    {
        std::unique_lock< std::mutex > lock( m_mutex );
        m_cv.wait_for( lock, timeout, [&] { return ! m_frames.empty() || m_finished || ! m_error.empty() || g_interrupted; } );
        if( ! m_error.empty() )
            throw std::runtime_error( m_error );
        finished = m_finished;
        m_finished = false;
        if( m_frames.empty() )
            return std::nullopt;
        DepthFrame f = std::move( m_frames.front() );
        m_frames.pop_front();
        return f;
    }

private:
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::deque< DepthFrame > m_frames;
    bool m_finished = false;
    std::string m_error;
};

std::vector< DepthFrame > collect( CaptureEngine & engine, FrameQueue & queue, int skip, int count )
{
    queue.clear();
    std::vector< DepthFrame > frames;
    int skipped = 0;
    auto last = std::chrono::steady_clock::now();
    while( int( frames.size() ) < count )
    {
        if( g_interrupted )
            throw Interrupted();
        bool finished = false;
        auto f = queue.pop( std::chrono::milliseconds( 200 ), finished );
        if( finished )
            engine.start();  // a recording ran out: play it again
        if( ! f )
        {
            if( std::chrono::steady_clock::now() - last > std::chrono::seconds( 10 ) )
                throw std::runtime_error( "No depth frames for 10 s" );
            continue;
        }
        last = std::chrono::steady_clock::now();
        if( skipped < skip )
            ++skipped;
        else
            frames.push_back( std::move( *f ) );
    }
    return frames;
}

json measure( const std::vector< DepthFrame > & frames, double roi_fraction, double baseline_m )
{
    json out;
    if( frames.empty() )
        return out;
    Roi roi = central_roi( frames[0].width, frames[0].height, roi_fraction );
    TemporalNoise temporal( roi );
    std::vector< double > fill, full_fill, median, rms, subpixel, nearest;
    for( auto & f : frames )
    {
        if( f.width != frames[0].width || f.height != frames[0].height )
            continue;
        FrameStats s = measure_frame( f.view(), roi, baseline_m );
        fill.push_back( s.fill_rate );
        full_fill.push_back( s.full_fill_rate );
        if( s.roi_valid > 0 )
        {
            median.push_back( s.median_m );
            rms.push_back( s.plane_rms_mm );
            subpixel.push_back( s.subpixel_rms );
        }
        if( s.nearest_m > 0 )
            nearest.push_back( s.nearest_m );
        temporal.add( f.view() );
    }
    auto noise = temporal.median_std_mm();
    out = { { "frames", frames.size() },
            { "fill_rate", mean( fill ) },
            { "full_fill_rate", mean( full_fill ) },
            { "median_m", mean( median ) },
            { "plane_rms_mm", mean( rms ) },
            { "subpixel_rms", mean( subpixel ) },
            { "temporal_noise_mm", noise ? json( *noise ) : json( nullptr ) },
            { "nearest_m", quantile( nearest, 0.5 ) },
            { "width", frames[0].width },
            { "height", frames[0].height },
            { "fx", frames[0].fx } };
    return out;
}

json theory( double fx, double baseline_m, int shift, double z )
{
    StereoRange r = stereo_range( fx, baseline_m, shift );
    return { { "focal_px", fx },
             { "min_z_m", r.min_z_m },
             { "max_z_m", r.max_z_m > 0 ? json( r.max_z_m ) : json( nullptr ) },
             { "rms_mm_at_median", z > 0 ? json( stereo_rms_error_m( z, fx, baseline_m ) * 1000.0 ) : json( nullptr ) } };
}

std::string fmt( const json & v, int decimals, double scale = 1.0, const char * suffix = "" )
{
    if( ! v.is_number() )
        return "-";
    char buf[64];
    std::snprintf( buf, sizeof buf, "%.*f%s", decimals, v.get< double >() * scale, suffix );
    return buf;
}

void print_enumeration( const json & info )
{
    std::printf( "## Device\n\n| Field | Value |\n|---|---|\n" );
    for( const char * key : { "name", "serial", "firmware", "recommended_firmware", "usb_type", "product_id",
                              "product_line", "advanced_mode", "depth_scale", "baseline_mm", "imu", "source" } )
        std::printf( "| %s | %s |\n", key,
                     info[key].is_string() ? info[key].get< std::string >().c_str() : info[key].dump().c_str() );
    std::printf( "| librealsense | %s |\n", sdk_version().c_str() );
    std::string presets;
    for( auto & p : info["presets"] )
        presets += ( presets.empty() ? "" : ", " ) + p.get< std::string >();
    std::printf( "| presets | %s |\n\n", presets.c_str() );

    for( auto & s : info["sensors"] )
    {
        std::printf( "## %s (%s) options\n\n| Option | Value | Min | Max | Step | Default | Read-only | Values |\n"
                     "|---|---|---|---|---|---|---|---|\n",
                     s["name"].get< std::string >().c_str(), s["kind"].get< std::string >().c_str() );
        for( auto & o : s["options"] )
        {
            std::string values;
            if( o["value_descriptions"].is_object() )
                for( auto it = o["value_descriptions"].begin(); it != o["value_descriptions"].end(); ++it )
                    values += ( values.empty() ? "" : ", " ) + it.key() + "=" + it.value().get< std::string >();
            std::printf( "| %s | %g | %g | %g | %g | %g | %s | %s |\n", o["name"].get< std::string >().c_str(),
                         o["value"].get< double >(), o["min"].get< double >(), o["max"].get< double >(),
                         o["step"].get< double >(), o["default"].get< double >(),
                         o["read_only"].get< bool >() ? "yes" : "", values.c_str() );
        }
        std::printf( "\n" );
    }

    // stream/format -> resolution -> frame rates
    std::map< std::string, std::map< std::pair< int, int >, std::set< int > > > modes;
    for( auto & p : info["profiles"] )
        modes[p["stream"].get< std::string >() + " " + std::to_string( p["index"].get< int >() ) + " "
              + p["format"].get< std::string >()][{ p["width"].get< int >(), p["height"].get< int >() }]
            .insert( p["fps"].get< int >() );
    std::printf( "## Stream profiles\n\n| Stream (index) | Format | Modes |\n|---|---|---|\n" );
    for( auto & m : modes )
    {
        std::istringstream is( m.first );
        std::string stream, index, format;
        is >> stream >> index >> format;
        std::string list;
        for( auto it = m.second.rbegin(); it != m.second.rend(); ++it )
        {
            std::string rates;
            for( int f : it->second )
                rates += ( rates.empty() ? "" : "/" ) + std::to_string( f );
            list += ( list.empty() ? "" : ", " )
                  + ( it->first.first ? std::to_string( it->first.first ) + "x" + std::to_string( it->first.second ) + "@"
                                      : std::string() )
                  + rates;
        }
        std::printf( "| %s (%s) | %s | %s |\n", stream.c_str(), index.c_str(), format.c_str(), list.c_str() );
    }
    std::printf( "\n" );
}

void print_results( const json & report )
{
    std::printf( "## Depth quality (flat target, central %.0f%% ROI, %d frames)\n\n",
                 report["roi_fraction"].get< double >() * 100, report["frames_per_measurement"].get< int >() );
    std::printf( "| Configuration | Filters | Min-Z / Max-Z (theory) | Median distance | Fill ROI | Fill image | "
                 "Plane RMS | Subpixel RMS | Temporal noise | Nearest valid | RMS theory |\n"
                 "|---|---|---|---|---|---|---|---|---|---|---|\n" );
    for( auto & r : report["results"] )
    {
        std::string name = r["config"]["name"].get< std::string >();
        if( r.contains( "error" ) )
        {
            std::printf( "| %s | - | error: %s |||||||||\n", name.c_str(), r["error"].get< std::string >().c_str() );
            continue;
        }
        for( const char * kind : { "raw", "filtered" } )
        {
            if( ! r.contains( kind ) )
                continue;
            const json & m = r[kind];
            const json & t = r["theory"];
            std::printf( "| %s | %s | %s / %s | %s | %s | %s | %s | %s | %s | %s | %s |\n", name.c_str(), kind,
                         fmt( t["min_z_m"], 3, 1, " m" ).c_str(),
                         t["max_z_m"].is_null() ? "inf" : fmt( t["max_z_m"], 2, 1, " m" ).c_str(),
                         fmt( m["median_m"], 3, 1, " m" ).c_str(), fmt( m["fill_rate"], 1, 100, " %" ).c_str(),
                         fmt( m["full_fill_rate"], 1, 100, " %" ).c_str(), fmt( m["plane_rms_mm"], 2, 1, " mm" ).c_str(),
                         fmt( m["subpixel_rms"], 3, 1, " px" ).c_str(), fmt( m["temporal_noise_mm"], 2, 1, " mm" ).c_str(),
                         fmt( m["nearest_m"], 3, 1, " m" ).c_str(), fmt( t["rms_mm_at_median"], 2, 1, " mm" ).c_str() );
        }
    }
    if( report.contains( "minz" ) )
    {
        const json & z = report["minz"];
        std::printf( "\nMin-Z test (%s): nearest distance with >= 90%% valid ROI pixels: %s (theory %s)\n",
                     z["config"].get< std::string >().c_str(), fmt( z["nearest_full_fill_m"], 3, 1, " m" ).c_str(),
                     fmt( z["theoretical_min_z_m"], 3, 1, " m" ).c_str() );
    }
    std::printf( "\n" );
}

int run( const Options & opt )
{
    FrameQueue queue;
    std::unique_ptr< CaptureEngine > engine = opt.playback.empty()
                                                ? CaptureEngine::create_live( queue.callbacks( opt.json_only ) )
                                                : CaptureEngine::create_playback( opt.playback, false,
                                                                                  queue.callbacks( opt.json_only ) );
    json report;
    report["sdk_version"] = sdk_version();
    report["device"] = engine->device_info();
    report["roi_fraction"] = opt.roi;
    report["frames_per_measurement"] = opt.frames;
    report["warmup_frames"] = opt.warmup;
    report["results"] = json::array();
    double baseline_m = report["device"]["baseline_mm"].is_number()
                          ? report["device"]["baseline_mm"].get< double >() / 1000.0
                          : 0.095;
    if( ! opt.json_only )
        print_enumeration( report["device"] );

    bool interrupted = false;
    try
    {
        if( ! opt.enumerate_only )
        {
            // The filters of the face-scan defaults, without the distance threshold (the wall can be anywhere).
            FilterConfig filters;
            filters.threshold.enabled = false;
            for( auto & c : opt.configs )
            {
                json row = { { "config",
                               { { "name", c.name },
                                 { "depth", { { "width", c.depth.width }, { "height", c.depth.height }, { "fps", c.depth.fps } } },
                                 { "preset", c.preset },
                                 { "disparity_shift", c.disparity_shift } } } };
                try
                {
                    CaptureConfig config;
                    config.depth = c.depth;
                    config.preset = c.preset;
                    config.disparity_shift = c.disparity_shift;
                    config.send_depth = true;
                    config.enable_imu = false;
                    config.filters = FilterConfig::none();
                    if( ! opt.json_only )
                        std::fprintf( stderr, "Measuring %s ...\n", c.name.c_str() );
                    engine->configure( to_json( config ) );
                    if( ! engine->streaming() )
                        engine->start();
                    double z = 0, fx = 0;
                    if( opt.raw )
                    {
                        engine->set_filters( to_json( FilterConfig::none() ) );
                        auto frames = collect( *engine, queue, opt.warmup, opt.frames );
                        row["raw"] = measure( frames, opt.roi, baseline_m );
                        z = row["raw"]["median_m"];
                        fx = frames[0].fx;
                    }
                    if( opt.filtered )
                    {
                        engine->set_filters( to_json( filters ) );
                        // The temporal filter needs a few frames to settle after the switch.
                        auto frames = collect( *engine, queue, opt.raw ? std::max( 5, opt.warmup / 3 ) : opt.warmup,
                                               opt.frames );
                        row["filtered"] = measure( frames, opt.roi, baseline_m );
                        if( z <= 0 )
                            z = row["filtered"]["median_m"];
                        fx = frames[0].fx;
                    }
                    row["theory"] = theory( fx, baseline_m, c.disparity_shift, z );
                }
                catch( const Interrupted & )
                {
                    throw;
                }
                catch( const std::exception & e )
                {
                    row["error"] = e.what();
                }
                report["results"].push_back( row );
            }
        }

        if( opt.minz_seconds > 0 && ! opt.configs.empty() )
        {
            const ProbeConfig & c = opt.configs.front();
            CaptureConfig config;
            config.depth = c.depth;
            config.preset = c.preset;
            config.disparity_shift = c.disparity_shift;
            config.send_depth = true;
            config.enable_imu = false;
            config.filters = FilterConfig::none();
            engine->configure( to_json( config ) );
            if( ! engine->streaming() )
                engine->start();
            if( ! opt.json_only )
                std::fprintf( stderr, "Min-Z test: move the wall slowly toward the camera (%g s)\n", opt.minz_seconds );
            queue.clear();
            auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds( int64_t( opt.minz_seconds * 1000 ) );
            auto next_report = std::chrono::steady_clock::now();
            double nearest_ok = 0, fx = 0;
            size_t samples = 0;
            while( std::chrono::steady_clock::now() < end && ! g_interrupted )
            {
                bool finished = false;
                auto f = queue.pop( std::chrono::milliseconds( 200 ), finished );
                if( finished )
                    engine->start();
                if( ! f )
                    continue;
                fx = f->fx;
                FrameStats s = measure_frame( f->view(), central_roi( f->width, f->height, opt.roi ), baseline_m );
                ++samples;
                if( s.fill_rate >= 0.9 && ( nearest_ok == 0 || s.median_m < nearest_ok ) )
                    nearest_ok = s.median_m;
                if( ! opt.json_only && std::chrono::steady_clock::now() >= next_report )
                {
                    std::fprintf( stderr, "  distance %.3f m, ROI fill %.0f %%, nearest OK %.3f m\n", s.median_m,
                                  s.fill_rate * 100, nearest_ok );
                    next_report += std::chrono::seconds( 1 );
                }
            }
            report["minz"] = { { "config", c.name },
                               { "seconds", opt.minz_seconds },
                               { "samples", samples },
                               { "nearest_full_fill_m", nearest_ok > 0 ? json( nearest_ok ) : json( nullptr ) },
                               { "theoretical_min_z_m", stereo_range( fx, baseline_m, c.disparity_shift ).min_z_m } };
        }
    }
    catch( const Interrupted & )
    {
        interrupted = true;
        report["interrupted"] = true;
    }

    engine->shutdown();
    if( ! opt.json_only )
    {
        if( ! opt.enumerate_only )
            print_results( report );
        std::printf( "```json\n" );
    }
    std::printf( "%s\n", report.dump( 2 ).c_str() );
    if( ! opt.json_only )
        std::printf( "```\n" );
    std::fflush( stdout );
    return interrupted ? 130 : 0;
}
}  // namespace

int run_probe( int argc, char ** argv )
{
    Options opt;
    for( int i = 0; i < argc; ++i )
    {
        std::string a = argv[i];
        const char * v = i + 1 < argc ? argv[i + 1] : nullptr;
        auto number = [&]( double min, double max, double & out ) {
            if( ! v )
                return false;
            char * end = nullptr;
            out = std::strtod( v, &end );
            ++i;
            return end && *end == 0 && out >= min && out <= max;
        };
        double d = 0;
        if( a == "--json" )
            opt.json_only = true;
        else if( a == "--frames" && number( 2, 10000, d ) )
            opt.frames = int( d );
        else if( a == "--warmup" && number( 0, 10000, d ) )
            opt.warmup = int( d );
        else if( a == "--roi" && number( 0.02, 1.0, d ) )
            opt.roi = d;
        else if( a == "--minz" && number( 1, 3600, d ) )
            opt.minz_seconds = d;
        else if( a == "--configs" && v )
        {
            ++i;
            std::stringstream ss( v );
            std::string item;
            while( std::getline( ss, item, ',' ) )
            {
                ProbeConfig c;
                if( ! parse_config( item, c ) )
                {
                    std::fprintf( stderr, "invalid configuration '%s' (expected WxH:preset:shift)\n", item.c_str() );
                    return 2;
                }
                opt.configs.push_back( c );
            }
        }
        else if( a == "--raw-only" )
            opt.filtered = false;
        else if( a == "--filtered-only" )
            opt.raw = false;
        else if( a == "--enumerate" )
            opt.enumerate_only = true;
        else if( a == "--playback" && v )
            opt.playback = argv[++i];
        else if( a == "-h" || a == "--help" )
        {
            usage( stdout );
            return 0;
        }
        else
        {
            std::fprintf( stderr, "invalid argument: %s\n", a.c_str() );
            usage( stderr );
            return 2;
        }
    }
    if( ! opt.raw && ! opt.filtered )
    {
        std::fprintf( stderr, "--raw-only and --filtered-only exclude each other\n" );
        return 2;
    }
    if( opt.configs.empty() )
        opt.configs = default_configs();

    SignalGuard signals;
    try
    {
        return run( opt );
    }
    catch( const std::exception & e )
    {
        std::fprintf( stderr, "probe: %s\n", e.what() );
        return 1;
    }
}

}  // namespace ifs
