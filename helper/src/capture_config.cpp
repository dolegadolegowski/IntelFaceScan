// License: Apache 2.0. See LICENSE file in root directory.
//
// Validation of the configuration JSON sent by the app (see capture_config.h).

#include "capture_config.h"

#include <cmath>
#include <initializer_list>
#include <sstream>
#include <stdexcept>

using nlohmann::json;

namespace
{
std::string format_number( double v )
{
    std::ostringstream os;
    os << v;
    return os.str();
}

[[noreturn]] void fail( const std::string & path, const std::string & message )
{
    throw std::runtime_error( path + ": " + message );
}

void require_object( const json & j, const std::string & path )
{
    if( ! j.is_object() )
        fail( path, "expected a JSON object" );
}

// Unknown keys are rejected so a typo in the app does not silently fall back to a default.
void check_keys( const json & j, const std::string & path, std::initializer_list< const char * > known )
{
    for( auto it = j.begin(); it != j.end(); ++it )
    {
        if( it.key() == "request_id" )
            continue;
        bool found = false;
        for( const char * k : known )
            if( it.key() == k )
                found = true;
        if( ! found )
            fail( path + "." + it.key(), "unknown key" );
    }
}

const json & member( const json & j, const std::string & path, const char * key )
{
    auto it = j.find( key );
    if( it == j.end() )
        fail( path + "." + key, "missing" );
    return *it;
}

bool get_bool( const json & j, const std::string & path, const char * key )
{
    const json & v = member( j, path, key );
    if( ! v.is_boolean() )
        fail( path + "." + key, "expected true or false" );
    return v.get< bool >();
}

double get_number( const json & j, const std::string & path, const char * key, double min, double max )
{
    const json & v = member( j, path, key );
    if( ! v.is_number() )
        fail( path + "." + key, "expected a number" );
    double d = v.get< double >();
    if( ! std::isfinite( d ) || d < min || d > max )
        fail( path + "." + key,
              format_number( d ) + " is out of range [" + format_number( min ) + ", " + format_number( max ) + "]" );
    return d;
}

int get_int( const json & j, const std::string & path, const char * key, int min, int max )
{
    const json & v = member( j, path, key );
    // Integral floats (30.0) are accepted: JSON does not distinguish them and some encoders emit them.
    if( ! v.is_number() || ( v.is_number_float() && std::floor( v.get< double >() ) != v.get< double >() ) )
        fail( path + "." + key, "expected an integer" );
    double d = v.get< double >();
    if( ! std::isfinite( d ) || d < min || d > max )
        fail( path + "." + key, format_number( d ) + " is out of range [" + std::to_string( min ) + ", "
                                    + std::to_string( max ) + "]" );
    return static_cast< int >( d );
}

std::string get_string( const json & j, const std::string & path, const char * key, size_t max_length )
{
    const json & v = member( j, path, key );
    if( ! v.is_string() )
        fail( path + "." + key, "expected a string" );
    auto s = v.get< std::string >();
    if( s.empty() || s.size() > max_length )
        fail( path + "." + key, "expected 1 to " + std::to_string( max_length ) + " characters" );
    for( unsigned char c : s )
        if( c < 0x20 || c == 0x7f )
            fail( path + "." + key, "contains control characters" );
    return s;
}

ifs::StreamConfig parse_stream( const json & parent, const std::string & parent_path, const char * key )
{
    std::string path = parent_path + "." + key;
    const json & j = member( parent, parent_path, key );
    require_object( j, path );
    check_keys( j, path, { "width", "height", "fps" } );
    ifs::StreamConfig s;
    s.width = get_int( j, path, "width", 1, 8192 );
    s.height = get_int( j, path, "height", 1, 8192 );
    s.fps = get_int( j, path, "fps", 1, 1000 );
    return s;
}

ifs::FilterConfig parse_filters_at( const json & j, const std::string & path )
{
    require_object( j, path );
    check_keys( j, path, { "threshold", "disparity_domain", "spatial", "temporal", "hole_filling" } );
    ifs::FilterConfig f;

    // Ranges are the ones librealsense enforces for these processing blocks.
    {
        std::string p = path + ".threshold";
        const json & t = member( j, path, "threshold" );
        require_object( t, p );
        check_keys( t, p, { "enabled", "min_m", "max_m" } );
        f.threshold.enabled = get_bool( t, p, "enabled" );
        f.threshold.min_m = float( get_number( t, p, "min_m", 0.0, 16.0 ) );
        f.threshold.max_m = float( get_number( t, p, "max_m", 0.0, 16.0 ) );
        if( f.threshold.min_m >= f.threshold.max_m )
            fail( p, "min_m must be smaller than max_m" );
    }
    f.disparity_domain = get_bool( j, path, "disparity_domain" );
    {
        std::string p = path + ".spatial";
        const json & s = member( j, path, "spatial" );
        require_object( s, p );
        check_keys( s, p, { "enabled", "magnitude", "alpha", "delta", "holes_fill" } );
        f.spatial.enabled = get_bool( s, p, "enabled" );
        f.spatial.magnitude = get_int( s, p, "magnitude", 1, 5 );
        f.spatial.alpha = float( get_number( s, p, "alpha", 0.25, 1.0 ) );
        f.spatial.delta = get_int( s, p, "delta", 1, 50 );
        f.spatial.holes_fill = get_int( s, p, "holes_fill", 0, 5 );
    }
    {
        std::string p = path + ".temporal";
        const json & t = member( j, path, "temporal" );
        require_object( t, p );
        check_keys( t, p, { "enabled", "alpha", "delta", "persistence" } );
        f.temporal.enabled = get_bool( t, p, "enabled" );
        f.temporal.alpha = float( get_number( t, p, "alpha", 0.0, 1.0 ) );
        f.temporal.delta = get_int( t, p, "delta", 1, 100 );
        f.temporal.persistence = get_int( t, p, "persistence", 0, 8 );
    }
    {
        std::string p = path + ".hole_filling";
        const json & h = member( j, path, "hole_filling" );
        require_object( h, p );
        check_keys( h, p, { "enabled", "mode" } );
        f.hole_filling.enabled = get_bool( h, p, "enabled" );
        f.hole_filling.mode = get_int( h, p, "mode", 0, 2 );
    }
    return f;
}
}  // namespace

namespace ifs
{

FilterConfig FilterConfig::none()
{
    FilterConfig f;
    f.threshold.enabled = false;
    f.spatial.enabled = false;
    f.temporal.enabled = false;
    f.hole_filling.enabled = false;
    return f;
}

FilterConfig parse_filter_config( const json & j )
{
    return parse_filters_at( j, "FilterConfiguration" );
}

CaptureConfig parse_capture_config( const json & j )
{
    const std::string path = "CaptureConfiguration";
    require_object( j, path );
    check_keys( j, path, { "depth", "color", "enable_ir", "enable_imu", "preset", "laser_power", "emitter_enabled",
                           "depth_units", "disparity_shift", "depth_auto_exposure", "color_auto_exposure",
                           "send_depth", "send_ir", "align_to_color", "filters" } );
    CaptureConfig c;
    c.depth = parse_stream( j, path, "depth" );
    c.color = parse_stream( j, path, "color" );
    c.enable_ir = get_bool( j, path, "enable_ir" );
    c.enable_imu = get_bool( j, path, "enable_imu" );
    c.preset = get_string( j, path, "preset", 64 );
    // Swift's JSONEncoder leaves out nil optionals, so a missing laser_power means "maximum" like null does.
    auto lp = j.find( "laser_power" );
    if( lp != j.end() && ! lp->is_null() )
        c.laser_power = float( get_number( j, path, "laser_power", 0.0, 100000.0 ) );
    c.emitter_enabled = get_bool( j, path, "emitter_enabled" );
    // D4xx accept 1 um .. 10 mm per unit; the device range is checked again when the value is applied.
    c.depth_units = float( get_number( j, path, "depth_units", 0.000001, 0.01 ) );
    c.disparity_shift = get_int( j, path, "disparity_shift", 0, 511 );
    c.depth_auto_exposure = get_bool( j, path, "depth_auto_exposure" );
    c.color_auto_exposure = get_bool( j, path, "color_auto_exposure" );
    c.send_depth = get_bool( j, path, "send_depth" );
    c.send_ir = get_bool( j, path, "send_ir" );
    c.align_to_color = get_bool( j, path, "align_to_color" );
    c.filters = parse_filters_at( member( j, path, "filters" ), path + ".filters" );

    if( ! c.align_to_color )
        fail( path + ".align_to_color",
              "false is not supported: framesets always carry depth aligned to color (the reconstruction input)" );
    if( c.send_ir && ! c.enable_ir )
        fail( path + ".send_ir", "requires enable_ir" );
    return c;
}

json to_json( const FilterConfig & f )
{
    return json{
        { "threshold",
          { { "enabled", f.threshold.enabled }, { "min_m", f.threshold.min_m }, { "max_m", f.threshold.max_m } } },
        { "disparity_domain", f.disparity_domain },
        { "spatial",
          { { "enabled", f.spatial.enabled },
            { "magnitude", f.spatial.magnitude },
            { "alpha", f.spatial.alpha },
            { "delta", f.spatial.delta },
            { "holes_fill", f.spatial.holes_fill } } },
        { "temporal",
          { { "enabled", f.temporal.enabled },
            { "alpha", f.temporal.alpha },
            { "delta", f.temporal.delta },
            { "persistence", f.temporal.persistence } } },
        { "hole_filling", { { "enabled", f.hole_filling.enabled }, { "mode", f.hole_filling.mode } } },
    };
}

json to_json( const CaptureConfig & c )
{
    auto stream = []( const StreamConfig & s ) {
        return json{ { "width", s.width }, { "height", s.height }, { "fps", s.fps } };
    };
    json j = {
        { "depth", stream( c.depth ) },
        { "color", stream( c.color ) },
        { "enable_ir", c.enable_ir },
        { "enable_imu", c.enable_imu },
        { "preset", c.preset },
        { "laser_power", nullptr },
        { "emitter_enabled", c.emitter_enabled },
        { "depth_units", c.depth_units },
        { "disparity_shift", c.disparity_shift },
        { "depth_auto_exposure", c.depth_auto_exposure },
        { "color_auto_exposure", c.color_auto_exposure },
        { "send_depth", c.send_depth },
        { "send_ir", c.send_ir },
        { "align_to_color", c.align_to_color },
        { "filters", to_json( c.filters ) },
    };
    if( c.laser_power )
        j["laser_power"] = *c.laser_power;
    return j;
}

}  // namespace ifs
