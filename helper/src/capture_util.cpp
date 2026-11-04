// License: Apache 2.0. See LICENSE file in root directory.
//
// librealsense helpers shared by the capture engine and the probe (see capture_util.h).

#include "capture_util.h"

#include <librealsense2/rsutil.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

using nlohmann::json;

namespace
{
// "Inverse Brown Conrady" -> "inverse_brown_conrady", "MOTION_XYZ32F" -> "motion_xyz32f"
std::string snake_case( const char * s )
{
    std::string out;
    if( ! s )
        return out;
    for( const char * p = s; *p; ++p )
    {
        unsigned char c = static_cast< unsigned char >( *p );
        if( c == ' ' || c == '-' )
            out += '_';
        else
            out += static_cast< char >( std::tolower( c ) );
    }
    return out;
}

bool finite( float v )
{
    return std::isfinite( v );
}
}  // namespace

namespace ifs
{

std::string stream_name( rs2_stream stream )
{
    return snake_case( rs2_stream_to_string( stream ) );
}

std::string format_name( rs2_format format )
{
    return snake_case( rs2_format_to_string( format ) );
}

std::string distortion_name( rs2_distortion model )
{
    return snake_case( rs2_distortion_to_string( model ) );
}

std::string timestamp_domain_name( rs2_timestamp_domain domain )
{
    return snake_case( rs2_timestamp_domain_to_string( domain ) );
}

json intrinsics_json( const rs2_intrinsics & in )
{
    return json{ { "width", in.width },
                 { "height", in.height },
                 { "fx", in.fx },
                 { "fy", in.fy },
                 { "ppx", in.ppx },
                 { "ppy", in.ppy },
                 { "model", distortion_name( in.model ) },
                 { "coeffs", { in.coeffs[0], in.coeffs[1], in.coeffs[2], in.coeffs[3], in.coeffs[4] } } };
}

json extrinsics_json( const rs2_extrinsics & e )
{
    json rotation = json::array();
    for( float r : e.rotation )
        rotation.push_back( r );
    return json{ { "rotation", rotation },
                 { "translation", { e.translation[0], e.translation[1], e.translation[2] } } };
}

json profile_json( const rs2::stream_profile & p )
{
    int width = 0, height = 0;
    if( auto v = p.as< rs2::video_stream_profile >() )
    {
        width = v.width();
        height = v.height();
    }
    return json{ { "stream", stream_name( p.stream_type() ) },
                 { "index", p.stream_index() },
                 { "format", format_name( p.format() ) },
                 { "width", width },
                 { "height", height },
                 { "fps", p.fps() } };
}

rs2_extrinsics identity_extrinsics()
{
    rs2_extrinsics e{};
    e.rotation[0] = e.rotation[4] = e.rotation[8] = 1.f;
    return e;
}

rs2_extrinsics inverse( const rs2_extrinsics & e )
{
    // R is column-major: element (row r, column c) is rotation[c * 3 + r]. The inverse of [R|t] is [R^T | -R^T t].
    rs2_extrinsics out{};
    for( int r = 0; r < 3; ++r )
        for( int c = 0; c < 3; ++c )
            out.rotation[c * 3 + r] = e.rotation[r * 3 + c];
    for( int r = 0; r < 3; ++r )
    {
        float sum = 0;
        for( int c = 0; c < 3; ++c )
            sum += out.rotation[c * 3 + r] * e.translation[c];
        out.translation[r] = -sum;
    }
    return out;
}

std::string sensor_kind( const rs2::sensor & sensor )
{
    if( sensor.is< rs2::depth_sensor >() )
        return "depth";
    if( sensor.is< rs2::color_sensor >() )
        return "color";
    if( sensor.is< rs2::motion_sensor >() )
        return "motion";
    return "";
}

json sensor_options_json( const rs2::sensor & sensor )
{
    json options = json::array();
    for( rs2_option id : const_cast< rs2::sensor & >( sensor ).get_supported_options() )
    {
        try
        {
            rs2::option_range range = sensor.get_option_range( id );
            float value = sensor.get_option( id );
            if( ! finite( value ) || ! finite( range.min ) || ! finite( range.max ) || ! finite( range.step )
                || ! finite( range.def ) )
                continue;
            json o = { { "name", rs2_option_to_string( id ) },
                       { "value", value },
                       { "min", range.min },
                       { "max", range.max },
                       { "step", range.step },
                       { "default", range.def },
                       { "read_only", sensor.is_option_read_only( id ) },
                       { "value_descriptions", nullptr } };
            // Enumerated options (visual preset, emitter mode, ...) describe their values; plain numeric ranges
            // do not, and can be huge, so only short ranges are probed.
            if( range.step > 0 && ( range.max - range.min ) / range.step <= 64 )
            {
                json descriptions = json::object();
                for( float v = range.min; v <= range.max + range.step * 0.5f; v += range.step )
                {
                    const char * d = nullptr;
                    try
                    {
                        d = sensor.get_option_value_description( id, v );
                    }
                    catch( const rs2::error & )
                    {
                        d = nullptr;
                    }
                    if( d && *d )
                        descriptions[std::to_string( static_cast< long long >( std::lround( v ) ) )] = d;
                }
                if( ! descriptions.empty() )
                    o["value_descriptions"] = descriptions;
            }
            options.push_back( std::move( o ) );
        }
        catch( const rs2::error & )
        {
            // e.g. temperatures that can only be read while streaming
        }
    }
    return options;
}

bool iequals( const std::string & a, const std::string & b )
{
    if( a.size() != b.size() )
        return false;
    for( size_t i = 0; i < a.size(); ++i )
        if( std::tolower( static_cast< unsigned char >( a[i] ) )
            != std::tolower( static_cast< unsigned char >( b[i] ) ) )
            return false;
    return true;
}

std::optional< rs2_option > option_from_name( const std::string & name )
{
    for( int i = 0; i < static_cast< int >( RS2_OPTION_COUNT ); ++i )
    {
        auto id = static_cast< rs2_option >( i );
        const char * s = rs2_option_to_string( id );
        if( s && iequals( name, s ) )
            return id;
    }
    return std::nullopt;
}

std::string camera_info( const rs2::device & device, rs2_camera_info info, const std::string & fallback )
{
    try
    {
        if( device.supports( info ) )
            return device.get_info( info );
    }
    catch( const rs2::error & )
    {
    }
    return fallback;
}

std::optional< PixelRect > clamp_rect( const PixelRect & r, int image_width, int image_height )
{
    long x0 = std::max< long >( r.x, 0 );
    long y0 = std::max< long >( r.y, 0 );
    long x1 = std::min< long >( long( r.x ) + r.width, image_width );
    long y1 = std::min< long >( long( r.y ) + r.height, image_height );
    if( r.width <= 0 || r.height <= 0 || x1 <= x0 || y1 <= y0 )
        return std::nullopt;
    return PixelRect{ int( x0 ), int( y0 ), int( x1 - x0 ), int( y1 - y0 ) };
}

PixelRect ensure_min_size( const PixelRect & r, int min_width, int min_height, int image_width, int image_height )
{
    auto grow = []( int pos, int size, int min_size, int limit, int & out_pos, int & out_size ) {
        out_size = std::min( std::max( size, min_size ), std::max( limit, 1 ) );
        // Keep the center (rounded toward the original position), then shift inside [0, limit).
        long start = long( pos ) - ( long( out_size ) - size ) / 2;
        start = std::clamp< long >( start, 0, std::max< long >( long( limit ) - out_size, 0 ) );
        out_pos = int( start );
    };
    PixelRect out;
    grow( r.x, r.width, min_width, image_width, out.x, out.width );
    grow( r.y, r.height, min_height, image_height, out.y, out.height );
    return out;
}

std::optional< PixelRect > map_color_rect_to_depth( const rs2_intrinsics & color,
                                                    const rs2_intrinsics & depth,
                                                    const rs2_extrinsics & color_to_depth,
                                                    const PixelRect & color_rect,
                                                    float distance_m )
{
    if( color_rect.width <= 0 || color_rect.height <= 0 || ! ( distance_m > 0 ) || depth.width <= 0
        || depth.height <= 0 )
        return std::nullopt;

    // The outline is sampled (not only the corners) because lens distortion bends the rectangle's edges.
    constexpr int samples = 8;
    float min_x = std::numeric_limits< float >::max(), min_y = min_x;
    float max_x = std::numeric_limits< float >::lowest(), max_y = max_x;
    bool any = false;
    auto add = [&]( float u, float v ) {
        float pixel[2] = { u, v };
        float point[3];
        rs2_deproject_pixel_to_point( point, &color, pixel, distance_m );
        float in_depth[3];
        rs2_transform_point_to_point( in_depth, &color_to_depth, point );
        if( ! ( in_depth[2] > 0 ) )
            return;
        float projected[2];
        rs2_project_point_to_pixel( projected, &depth, in_depth );
        if( ! std::isfinite( projected[0] ) || ! std::isfinite( projected[1] ) )
            return;
        min_x = std::min( min_x, projected[0] );
        min_y = std::min( min_y, projected[1] );
        max_x = std::max( max_x, projected[0] );
        max_y = std::max( max_y, projected[1] );
        any = true;
    };
    float x0 = float( color_rect.x ), y0 = float( color_rect.y );
    float x1 = float( color_rect.x + color_rect.width ), y1 = float( color_rect.y + color_rect.height );
    for( int i = 0; i <= samples; ++i )
    {
        float t = float( i ) / samples;
        add( x0 + ( x1 - x0 ) * t, y0 );
        add( x0 + ( x1 - x0 ) * t, y1 );
        add( x0, y0 + ( y1 - y0 ) * t );
        add( x1, y0 + ( y1 - y0 ) * t );
    }
    if( ! any )
        return std::nullopt;
    // Keep the projected box within a sane range before converting to int (far outside points stay outside).
    auto to_int = []( float v ) { return int( std::lround( std::clamp( v, -1.0e6f, 1.0e6f ) ) ); };
    PixelRect mapped{ to_int( std::floor( min_x ) ), to_int( std::floor( min_y ) ), 0, 0 };
    mapped.width = to_int( std::ceil( max_x ) ) - mapped.x;
    mapped.height = to_int( std::ceil( max_y ) ) - mapped.y;
    return clamp_rect( mapped, depth.width, depth.height );
}

OptionValues options_to_restore( const OptionValues & before, const OptionValues & now )
{
    auto changed = []( const auto & a, const auto & b ) {
        return a && b && std::fabs( double( *a ) - double( *b ) ) > 1e-9 + 1e-6 * std::fabs( double( *a ) );
    };
    OptionValues restore;
    if( changed( before.depth_units, now.depth_units ) )
        restore.depth_units = before.depth_units;
    if( changed( before.disparity_shift, now.disparity_shift ) )
        restore.disparity_shift = before.disparity_shift;
    bool laser = changed( before.laser_power, now.laser_power );
    if( laser )
        restore.laser_power = before.laser_power;
    if( changed( before.emitter, now.emitter ) || ( laser && before.emitter && *before.emitter == 0 ) )
        restore.emitter = before.emitter;
    if( changed( before.depth_auto_exposure, now.depth_auto_exposure ) )
        restore.depth_auto_exposure = before.depth_auto_exposure;
    if( changed( before.color_auto_exposure, now.color_auto_exposure ) )
        restore.color_auto_exposure = before.color_auto_exposure;
    return restore;
}

bool FrameCounterMonitor::update( unsigned long long frame_number )
{
    bool restarted = m_last && frame_number < *m_last;
    m_last = frame_number;
    return restarted;
}

StereoRange stereo_range( double focal_px, double baseline_m, int disparity_shift, double max_disparity )
{
    StereoRange r;
    double fb = focal_px * baseline_m;
    r.min_z_m = fb / ( max_disparity + disparity_shift );
    r.max_z_m = disparity_shift > 0 ? fb / disparity_shift : 0.0;
    return r;
}

double stereo_rms_error_m( double z_m, double focal_px, double baseline_m, double subpixel )
{
    return z_m * z_m * subpixel / ( focal_px * baseline_m );
}

}  // namespace ifs
