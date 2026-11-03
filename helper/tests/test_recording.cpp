// License: Apache 2.0. See LICENSE file in root directory.
//
// Synthetic D455-like recordings (see test_recording.h).

#include "test_recording.h"

#include <librealsense2/rsutil.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

namespace ifs
{
namespace test
{
namespace
{
constexpr double k_pi = 3.14159265358979323846;
constexpr double k_first_timestamp_ms = 1730000000000.0;
constexpr int k_accel_fps = 100;
constexpr int k_gyro_fps = 200;

struct Vec3
{
    double x = 0, y = 0, z = 0;
};
Vec3 operator+( Vec3 a, Vec3 b ) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
Vec3 operator-( Vec3 a, Vec3 b ) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
Vec3 operator*( Vec3 a, double s ) { return { a.x * s, a.y * s, a.z * s }; }
double dot( Vec3 a, Vec3 b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 normalized( Vec3 a )
{
    double l = std::sqrt( dot( a, a ) );
    return l > 0 ? a * ( 1.0 / l ) : a;
}

struct Rgb
{
    double r = 0, g = 0, b = 0;
};

// splitmix64: deterministic per-pixel noise that does not depend on thread scheduling.
uint64_t mix( uint64_t z )
{
    z += 0x9e3779b97f4a7c15ull;
    z = ( z ^ ( z >> 30 ) ) * 0xbf58476d1ce4e5b9ull;
    z = ( z ^ ( z >> 27 ) ) * 0x94d049bb133111ebull;
    return z ^ ( z >> 31 );
}
double uniform( uint64_t key )
{
    return ( double( mix( key ) >> 11 ) + 0.5 ) * ( 1.0 / 9007199254740992.0 );
}
double gaussian( uint64_t key )
{
    double u1 = uniform( key ), u2 = uniform( key ^ 0x5851f42d4c957f2dull );
    return std::sqrt( -2.0 * std::log( u1 ) ) * std::cos( 2.0 * k_pi * u2 );
}

// The scene lives in the depth sensor's frame (+X right, +Y down, +Z forward, meters).
class Scene
{
public:
    Scene( const RecordingOptions & o, int frame )
        : m_options( o )
    {
        m_yaw = double( o.yaw_deg_per_frame ) * frame * k_pi / 180.0;
        m_head_center = { 0.0, 0.0, double( o.distance_m ) };
        // The nose turns with the head about its vertical axis.
        Vec3 nose_offset{ 0.0, 0.005, -0.095 };
        m_nose_center = m_head_center + Vec3{ nose_offset.x * std::cos( m_yaw ) + nose_offset.z * std::sin( m_yaw ),
                                              nose_offset.y,
                                              -nose_offset.x * std::sin( m_yaw ) + nose_offset.z * std::cos( m_yaw ) };
        double tilt = double( o.plane_tilt_deg ) * k_pi / 180.0;
        m_plane_normal = { std::sin( tilt ), 0.0, -std::cos( tilt ) };
        m_plane_offset = -dot( m_plane_normal, Vec3{ 0, 0, double( o.distance_m ) } );
    }

    // Nearest hit along origin + t*dir (t > 0); returns false when the ray hits nothing.
    bool trace( Vec3 origin, Vec3 dir, Vec3 & hit, Vec3 & normal, Rgb & color ) const
    {
        double best = 1e30;
        int what = 0;
        if( m_options.scene == "plane" )
        {
            double denom = dot( m_plane_normal, dir );
            if( std::fabs( denom ) > 1e-9 )
            {
                double t = -( dot( m_plane_normal, origin ) + m_plane_offset ) / denom;
                if( t > 1e-6 )
                {
                    best = t;
                    what = 4;
                }
            }
        }
        else
        {
            double t;
            if( sphere( origin, dir, m_head_center, k_head_radius, t ) && t < best )
            {
                best = t;
                what = 1;
            }
            if( sphere( origin, dir, m_nose_center, k_nose_radius, t ) && t < best )
            {
                best = t;
                what = 2;
            }
            if( m_options.wall_m > 0 && std::fabs( dir.z ) > 1e-9 )
            {
                t = ( double( m_options.wall_m ) - origin.z ) / dir.z;
                if( t > 1e-6 && t < best )
                {
                    best = t;
                    what = 3;
                }
            }
        }
        if( ! what )
            return false;
        hit = origin + dir * best;
        switch( what )
        {
        case 1:
        {
            normal = normalized( hit - m_head_center );
            // Latitude/longitude checker in head coordinates (turns with the head), skin-like colors.
            double lon = std::atan2( normal.x, -normal.z ) - m_yaw;
            double lat = std::asin( std::clamp( normal.y, -1.0, 1.0 ) );
            int cell = int( std::floor( lon / ( k_pi / 10 ) ) ) + int( std::floor( lat / ( k_pi / 12 ) ) );
            double k = ( cell & 1 ) ? 1.0 : 0.72;
            color = { 225 * k, 172 * k, 140 * k };
            break;
        }
        case 2:
            normal = normalized( hit - m_nose_center );
            color = { 205, 118, 104 };
            break;
        case 3:
        {
            normal = { 0, 0, -1 };
            int cell = int( std::floor( hit.x / 0.05 ) ) + int( std::floor( hit.y / 0.05 ) );
            color = ( cell & 1 ) ? Rgb{ 92, 110, 140 } : Rgb{ 160, 170, 190 };
            break;
        }
        default:
        {
            normal = m_plane_normal;
            int cell = int( std::floor( hit.x / 0.02 ) ) + int( std::floor( hit.y / 0.02 ) );
            color = ( cell & 1 ) ? Rgb{ 70, 140, 90 } : Rgb{ 200, 190, 160 };
            break;
        }
        }
        return true;
    }

private:
    static constexpr double k_head_radius = 0.09;
    static constexpr double k_nose_radius = 0.022;

    static bool sphere( Vec3 origin, Vec3 dir, Vec3 center, double radius, double & t )
    {
        Vec3 oc = origin - center;
        double b = dot( oc, dir ), c = dot( oc, oc ) - radius * radius, a = dot( dir, dir );
        double disc = b * b - a * c;
        if( disc < 0 )
            return false;
        double s = std::sqrt( disc );
        t = ( -b - s ) / a;
        if( t <= 1e-6 )
            t = ( -b + s ) / a;
        return t > 1e-6;
    }

    const RecordingOptions & m_options;
    double m_yaw = 0;
    Vec3 m_head_center, m_nose_center, m_plane_normal;
    double m_plane_offset = 0;
};

Vec3 pixel_ray( const rs2_intrinsics & intrinsics, float u, float v )
{
    float pixel[2] = { u, v }, point[3];
    rs2_deproject_pixel_to_point( point, &intrinsics, pixel, 1.f );
    return { point[0], point[1], point[2] };
}

// Color camera origin and ray direction in the depth frame (extrinsics map depth points into the color frame).
void color_ray( const rs2_extrinsics & d2c, const rs2_intrinsics & color, float u, float v, Vec3 & origin, Vec3 & dir )
{
    Vec3 d = pixel_ray( color, u, v );
    const float * r = d2c.rotation;  // column-major: R(row, col) = r[col * 3 + row]
    auto rt_mul = [r]( Vec3 p ) {    // R^T * p
        return Vec3{ r[0] * p.x + r[1] * p.y + r[2] * p.z, r[3] * p.x + r[4] * p.y + r[5] * p.z,
                     r[6] * p.x + r[7] * p.y + r[8] * p.z };
    };
    origin = rt_mul( Vec3{ d2c.translation[0], d2c.translation[1], d2c.translation[2] } ) * -1.0;
    dir = rt_mul( d );
}

template< typename F >
void parallel_rows( int rows, F && body )
{
    unsigned n = std::clamp( std::thread::hardware_concurrency(), 1u, 16u );
    std::vector< std::thread > threads;
    for( unsigned t = 0; t < n; ++t )
        threads.emplace_back( [&, t] {
            for( int y = int( t ); y < rows; y += int( n ) )
                body( y );
        } );
    for( auto & th : threads )
        th.join();
}

struct FrameImages
{
    std::vector< uint16_t > depth;
    std::vector< uint8_t > color;
    std::vector< uint8_t > infrared;
};

FrameImages render( const RecordingOptions & o, int frame, const rs2_intrinsics & di, const rs2_intrinsics & ci,
                    const rs2_extrinsics & d2c )
{
    Scene scene( o, frame );
    FrameImages img;
    img.depth.assign( size_t( di.width ) * di.height, 0 );
    if( o.infrared )
        img.infrared.assign( size_t( di.width ) * di.height, 0 );
    img.color.assign( size_t( ci.width ) * ci.height * 3, 0 );

    parallel_rows( di.height, [&]( int y ) {
        for( int x = 0; x < di.width; ++x )
        {
            Vec3 dir = pixel_ray( di, float( x ), float( y ) ), hit, normal;
            Rgb c;
            size_t i = size_t( y ) * di.width + x;
            if( ! scene.trace( {}, dir, hit, normal, c ) )
                continue;
            double z = hit.z;
            if( o.noise_m > 0 )
            {
                uint64_t key = ( uint64_t( o.seed ) << 48 ) ^ ( uint64_t( frame ) << 32 ) ^ uint64_t( i );
                z += gaussian( key ) * double( o.noise_m ) * z * z;
            }
            double units = std::round( z / double( o.depth_units ) );
            if( units > 0 && units < 65536 )
                img.depth[i] = uint16_t( units );
            if( o.infrared )
            {
                // Texture brightness plus the projector's dot pattern, as the left imager sees it.
                double lum = ( 0.3 * c.r + 0.59 * c.g + 0.11 * c.b ) * 0.6;
                if( uniform( uint64_t( i ) * 2654435761ull ) < 0.06 )
                    lum += 140;
                img.infrared[i] = uint8_t( std::clamp( lum, 0.0, 255.0 ) );
            }
        }
    } );

    parallel_rows( ci.height, [&]( int y ) {
        for( int x = 0; x < ci.width; ++x )
        {
            Vec3 origin, dir, hit, normal;
            color_ray( d2c, ci, float( x ), float( y ), origin, dir );
            Rgb c{ 28, 28, 32 };
            if( scene.trace( origin, dir, hit, normal, c ) )
            {
                double shade = 0.35 + 0.65 * std::max( 0.0, -dot( normal, normalized( dir ) ) );
                c = { c.r * shade, c.g * shade, c.b * shade };
            }
            uint8_t * p = &img.color[( size_t( y ) * ci.width + x ) * 3];
            p[0] = uint8_t( std::clamp( c.r, 0.0, 255.0 ) );
            p[1] = uint8_t( std::clamp( c.g, 0.0, 255.0 ) );
            p[2] = uint8_t( std::clamp( c.b, 0.0, 255.0 ) );
        }
    } );
    return img;
}

template< typename T >
void * heap_copy( const std::vector< T > & v )
{
    auto * p = new uint8_t[v.size() * sizeof( T )];
    std::memcpy( p, v.data(), v.size() * sizeof( T ) );
    return p;
}

void free_bytes( void * p )
{
    delete[] static_cast< uint8_t * >( p );
}

void set_info( rs2::software_device & dev, rs2_camera_info info, const std::string & value )
{
    if( dev.supports( info ) )
        dev.update_info( info, value );
    else
        dev.register_info( info, value );
}
}  // namespace

rs2_intrinsics synthetic_depth_intrinsics( int width, int height )
{
    // D455 imagers: ~90 x 58 degrees at 1280x720 (fx = fy ~ 632 px); other resolutions scale with the width.
    rs2_intrinsics in{};
    double s = width / 1280.0;
    in.width = width;
    in.height = height;
    in.fx = float( 632.0 * s );
    in.fy = float( 632.0 * s );
    in.ppx = float( ( width - 1 ) * 0.5 + 1.3 * s );
    in.ppy = float( ( height - 1 ) * 0.5 - 2.1 * s );
    in.model = RS2_DISTORTION_BROWN_CONRADY;
    return in;
}

rs2_intrinsics synthetic_color_intrinsics( int width, int height )
{
    // D455 global-shutter RGB (OV9782), 1280x800: ~90 x 65 degrees, small inverse Brown-Conrady distortion.
    rs2_intrinsics in{};
    double s = width / 1280.0;
    in.width = width;
    in.height = height;
    in.fx = float( 640.1 * s );
    in.fy = float( 639.8 * s );
    in.ppx = float( 641.2 * s );
    in.ppy = float( 401.3 * height / 800.0 );
    in.model = RS2_DISTORTION_INVERSE_BROWN_CONRADY;
    const float coeffs[5] = { -0.0557f, 0.0675f, -0.0006f, 0.0006f, -0.0218f };
    std::copy( coeffs, coeffs + 5, in.coeffs );
    return in;
}

rs2_extrinsics synthetic_depth_to_color()
{
    // RGB sensor 59 mm to the right of the left imager, rotated by ~0.2 degrees about Y.
    rs2_extrinsics e{};
    double a = 0.2 * k_pi / 180.0;
    float c = float( std::cos( a ) ), s = float( std::sin( a ) );
    const float rotation[9] = { c, 0, -s, 0, 1, 0, s, 0, c };  // column-major
    std::copy( rotation, rotation + 9, e.rotation );
    e.translation[0] = -0.059f;
    e.translation[1] = 0.0001f;
    e.translation[2] = 0.0004f;
    return e;
}

float scene_depth_at_color_pixel( const RecordingOptions & o, int frame, float u, float v )
{
    Scene scene( o, frame );
    Vec3 origin, dir, hit, normal;
    Rgb c;
    color_ray( synthetic_depth_to_color(), synthetic_color_intrinsics( o.color_width, o.color_height ), u, v, origin, dir );
    return scene.trace( origin, dir, hit, normal, c ) ? float( hit.z ) : 0.f;
}

float scene_depth_at_depth_pixel( const RecordingOptions & o, int frame, float u, float v )
{
    Scene scene( o, frame );
    Vec3 hit, normal;
    Rgb c;
    Vec3 dir = pixel_ray( synthetic_depth_intrinsics( o.depth_width, o.depth_height ), u, v );
    return scene.trace( {}, dir, hit, normal, c ) ? float( hit.z ) : 0.f;
}

struct SyntheticCamera::Impl
{
    RecordingOptions o;
    RecordingInfo info;
    rs2::software_device dev;
    std::optional< rs2::software_sensor > stereo, rgb, motion;
    rs2::stream_profile depth_profile, ir_profile, color_profile, accel_profile, gyro_profile;
    bool per_frame = false;
    FrameImages still;
    int accel_index = 0, gyro_index = 0;
};

SyntheticCamera::SyntheticCamera( const RecordingOptions & o )
    : m( std::make_unique< Impl >() )
{
    if( o.frames < 1 || o.fps < 1 || o.depth_width < 16 || o.depth_height < 16 || o.color_width < 16
        || o.color_height < 16 || ! ( o.depth_units > 0 ) )
        throw std::runtime_error( "Invalid recording options" );
    if( o.scene != "head" && o.scene != "plane" )
        throw std::runtime_error( "Unknown scene '" + o.scene + "' (head or plane)" );
    m->o = o;
    RecordingInfo & info = m->info;
    info.depth_intrinsics = synthetic_depth_intrinsics( o.depth_width, o.depth_height );
    info.color_intrinsics = synthetic_color_intrinsics( o.color_width, o.color_height );
    info.depth_to_color = synthetic_depth_to_color();
    info.first_timestamp_ms = k_first_timestamp_ms;
    info.frames = o.frames;

    rs2::software_device & dev = m->dev;
    set_info( dev, RS2_CAMERA_INFO_NAME, "Intel RealSense D455 (synthetic)" );
    set_info( dev, RS2_CAMERA_INFO_SERIAL_NUMBER, k_synthetic_serial );
    set_info( dev, RS2_CAMERA_INFO_FIRMWARE_VERSION, "5.17.0.10" );
    set_info( dev, RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION, "5.17.0.10" );
    set_info( dev, RS2_CAMERA_INFO_PRODUCT_ID, "0B5C" );
    set_info( dev, RS2_CAMERA_INFO_PRODUCT_LINE, "D400" );
    set_info( dev, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR, "3.2" );

    m->stereo = dev.add_sensor( "Stereo Module" );
    m->rgb = dev.add_sensor( "RGB Camera" );
    rs2::software_sensor & stereo = *m->stereo;
    rs2::software_sensor & rgb = *m->rgb;

    rs2_video_stream depth_stream{ RS2_STREAM_DEPTH, 0, 0, o.depth_width, o.depth_height, o.fps, 2,
                                   RS2_FORMAT_Z16, info.depth_intrinsics };
    m->depth_profile = stereo.add_video_stream( depth_stream, true );
    if( o.infrared )
    {
        rs2_video_stream ir_stream{ RS2_STREAM_INFRARED, 1, 1, o.depth_width, o.depth_height, o.fps, 1,
                                    RS2_FORMAT_Y8, info.depth_intrinsics };
        m->ir_profile = stereo.add_video_stream( ir_stream );
    }
    // Depth units + baseline make the sensor a depth (stereo) sensor for librealsense, also in the recording.
    stereo.add_read_only_option( RS2_OPTION_DEPTH_UNITS, o.depth_units );
    stereo.add_read_only_option( RS2_OPTION_STEREO_BASELINE, k_synthetic_baseline_mm );
    stereo.add_option( RS2_OPTION_LASER_POWER, rs2::option_range{ 0, 360, 150, 30 } );
    stereo.add_option( RS2_OPTION_EMITTER_ENABLED, rs2::option_range{ 0, 2, 1, 1 } );
    stereo.add_option( RS2_OPTION_ENABLE_AUTO_EXPOSURE, rs2::option_range{ 0, 1, 1, 1 } );
    stereo.set_option( RS2_OPTION_LASER_POWER, 360 );
    stereo.set_metadata( RS2_FRAME_METADATA_ACTUAL_EXPOSURE, 8500 );
    stereo.set_metadata( RS2_FRAME_METADATA_FRAME_LASER_POWER, 360 );
    stereo.set_metadata( RS2_FRAME_METADATA_FRAME_EMITTER_MODE, 1 );

    rs2_video_stream color_stream{ RS2_STREAM_COLOR, 0, 2, o.color_width, o.color_height, o.fps, 3,
                                   RS2_FORMAT_RGB8, info.color_intrinsics };
    m->color_profile = rgb.add_video_stream( color_stream, true );
    rgb.add_option( RS2_OPTION_ENABLE_AUTO_EXPOSURE, rs2::option_range{ 0, 1, 1, 1 } );
    rgb.add_option( RS2_OPTION_EXPOSURE, rs2::option_range{ 1, 10000, 156, 1 } );
    rgb.set_metadata( RS2_FRAME_METADATA_ACTUAL_EXPOSURE, 80 );  // UVC: 100 us units

    m->depth_profile.register_extrinsics_to( m->color_profile, info.depth_to_color );
    if( m->ir_profile )
    {
        rs2_extrinsics identity{ { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, { 0, 0, 0 } };
        m->ir_profile.register_extrinsics_to( m->depth_profile, identity );
    }

    if( o.imu )
    {
        m->motion = dev.add_sensor( "Motion Module" );
        rs2_motion_device_intrinsic mi{};
        for( int r = 0; r < 3; ++r )
            mi.data[r][r] = 1.f;
        for( int r = 0; r < 3; ++r )
        {
            mi.noise_variances[r] = 1e-4f;
            mi.bias_variances[r] = 1e-6f;
        }
        m->accel_profile = m->motion->add_motion_stream(
            rs2_motion_stream{ RS2_STREAM_ACCEL, 0, 3, k_accel_fps, RS2_FORMAT_MOTION_XYZ32F, mi } );
        m->gyro_profile = m->motion->add_motion_stream(
            rs2_motion_stream{ RS2_STREAM_GYRO, 0, 4, k_gyro_fps, RS2_FORMAT_MOTION_XYZ32F, mi } );
    }

    // Static scenes are rendered once; noise and motion need a render per frame.
    m->per_frame = o.noise_m > 0 || o.yaw_deg_per_frame != 0;
    if( ! m->per_frame )
        m->still = render( o, 0, info.depth_intrinsics, info.color_intrinsics, info.depth_to_color );
}

SyntheticCamera::~SyntheticCamera() = default;

rs2::software_device & SyntheticCamera::device()
{
    return m->dev;
}

const RecordingInfo & SyntheticCamera::info() const
{
    return m->info;
}

void SyntheticCamera::start_all()
{
    std::vector< rs2::stream_profile > stereo_profiles{ m->depth_profile };
    if( m->ir_profile )
        stereo_profiles.push_back( m->ir_profile );
    m->stereo->open( stereo_profiles );
    m->rgb->open( m->color_profile );
    if( m->motion )
        m->motion->open( { m->accel_profile, m->gyro_profile } );
    // A recorder hooks the sensors' frame callbacks, so each sensor needs one.
    auto ignore = []( rs2::frame ) {};
    m->rgb->start( ignore );
    m->stereo->start( ignore );
    if( m->motion )
        m->motion->start( ignore );
}

void SyntheticCamera::stop_all()
{
    m->stereo->stop();
    m->rgb->stop();
    if( m->motion )
        m->motion->stop();
    m->stereo->close();
    m->rgb->close();
    if( m->motion )
        m->motion->close();
}

void SyntheticCamera::emit_frame( int i )
{
    const RecordingOptions & o = m->o;
    const RecordingInfo & info = m->info;
    const double period_ms = 1000.0 / o.fps;
    double ts = k_first_timestamp_ms + i * period_ms;
    int depth_number = o.restart_at >= 0 && i >= o.restart_at ? 1 + ( i - o.restart_at ) : 100 + i;
    FrameImages rendered;
    const FrameImages & img
        = m->per_frame ? ( rendered = render( o, i, info.depth_intrinsics, info.color_intrinsics, info.depth_to_color ) )
                       : m->still;

    // IMU samples up to this frame's time: gravity along -Y of the camera (held upright), a slight tremor.
    while( m->motion && k_first_timestamp_ms + m->accel_index * 1000.0 / k_accel_fps <= ts )
    {
        float * d = new float[3]{ 0.02f * float( std::sin( m->accel_index * 0.3 ) ), -9.81f, 0.05f };
        m->motion->on_motion_frame( { d, []( void * p ) { delete[] static_cast< float * >( p ); },
                                      k_first_timestamp_ms + m->accel_index * 1000.0 / k_accel_fps,
                                      RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME, m->accel_index, m->accel_profile.get() } );
        ++m->accel_index;
    }
    while( m->motion && k_first_timestamp_ms + m->gyro_index * 1000.0 / k_gyro_fps <= ts )
    {
        float * d = new float[3]{ 0.001f, -0.002f, 0.0005f };
        m->motion->on_motion_frame( { d, []( void * p ) { delete[] static_cast< float * >( p ); },
                                      k_first_timestamp_ms + m->gyro_index * 1000.0 / k_gyro_fps,
                                      RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME, m->gyro_index, m->gyro_profile.get() } );
        ++m->gyro_index;
    }

    m->rgb->on_video_frame( { heap_copy( img.color ), free_bytes, o.color_width * 3, 3, ts,
                              RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME, 200 + i, m->color_profile.get(), 0.f } );
    m->stereo->on_video_frame( { heap_copy( img.depth ), free_bytes, o.depth_width * 2, 2, ts,
                                 RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME, depth_number, m->depth_profile.get(), o.depth_units } );
    if( m->ir_profile )
        m->stereo->on_video_frame( { heap_copy( img.infrared ), free_bytes, o.depth_width, 1, ts,
                                     RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME, depth_number, m->ir_profile.get(), 0.f } );
}

RecordingInfo write_test_recording( const RecordingOptions & o )
{
    if( o.path.empty() )
        throw std::runtime_error( "No output path" );
    SyntheticCamera camera( o );
    {
        rs2::recorder recorder( o.path, camera.device(), o.compress );
        camera.start_all();
        auto started = std::chrono::steady_clock::now();
        for( int i = 0; i < o.frames; ++i )
        {
            if( o.pace )
                std::this_thread::sleep_until( started + std::chrono::microseconds( int64_t( i * 1e6 / o.fps ) ) );
            camera.emit_frame( i );
        }
        camera.stop_all();
    }  // the recorder's destruction flushes and closes the file
    return camera.info();
}

}  // namespace test
}  // namespace ifs
