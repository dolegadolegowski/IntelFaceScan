// License: Apache 2.0. See LICENSE file in root directory.
//
// make-test-recording: writes a synthetic D455-like .db3/.bag recording (see test_recording.h) for end-to-end
// tests without a camera, e.g.
//
//   build/helper-tests/make-test-recording -o /tmp/head.db3 --frames 90 --yaw 0.5 --noise 0.0008
//   IntelFaceScanHelper --socket ... --playback /tmp/head.db3 --no-real-time

#include "test_recording.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

namespace
{
void usage()
{
    std::printf(
        "usage: make-test-recording -o <file.db3|file.bag> [options]\n"
        "  --frames N         frames to record (default 30)\n"
        "  --fps N            frame rate (default 30)\n"
        "  --depth WxH        depth resolution (default 1280x720)\n"
        "  --color WxH        color resolution (default 1280x800)\n"
        "  --scene head|plane head: textured sphere with a nose in front of a wall; plane: tilted flat target\n"
        "  --distance M       head center / plane distance in meters (default 0.5)\n"
        "  --wall M           head scene: wall distance in meters, 0 = none (default 1.0)\n"
        "  --tilt DEG         plane scene: rotation about the vertical axis (default 10)\n"
        "  --noise M          depth noise std at 1 m in meters, scaled by z^2 (default 0)\n"
        "  --yaw DEG          head rotation per frame in degrees (default 0)\n"
        "  --depth-units M    meters per Z16 unit (default 0.0001)\n"
        "  --restart-at N     restart the depth frame counter at frame N (firmware stream restart)\n"
        "  --no-ir            no infrared stream\n"
        "  --no-imu           no motion module\n"
        "  --compress         compress frames in the file\n"
        "  --fast             do not pace frames at the frame rate (file timestamps are then compressed)\n"
        "  --seed N           noise seed (default 1)\n" );
}

bool parse_size( const char * s, int & w, int & h )
{
    return s && std::sscanf( s, "%dx%d", &w, &h ) == 2 && w > 0 && h > 0;
}
}  // namespace

int main( int argc, char ** argv )
{
    ifs::test::RecordingOptions o;
    for( int i = 1; i < argc; ++i )
    {
        std::string a = argv[i];
        const char * v = i + 1 < argc ? argv[i + 1] : nullptr;
        auto need = [&]() {
            if( ! v )
            {
                std::fprintf( stderr, "%s needs a value\n", a.c_str() );
                std::exit( 2 );
            }
            ++i;
            return v;
        };
        if( a == "-o" || a == "--output" )
            o.path = need();
        else if( a == "--frames" )
            o.frames = std::atoi( need() );
        else if( a == "--fps" )
            o.fps = std::atoi( need() );
        else if( a == "--depth" )
        {
            if( ! parse_size( need(), o.depth_width, o.depth_height ) )
                return usage(), 2;
        }
        else if( a == "--color" )
        {
            if( ! parse_size( need(), o.color_width, o.color_height ) )
                return usage(), 2;
        }
        else if( a == "--scene" )
            o.scene = need();
        else if( a == "--distance" )
            o.distance_m = float( std::atof( need() ) );
        else if( a == "--wall" )
            o.wall_m = float( std::atof( need() ) );
        else if( a == "--tilt" )
            o.plane_tilt_deg = float( std::atof( need() ) );
        else if( a == "--noise" )
            o.noise_m = float( std::atof( need() ) );
        else if( a == "--yaw" )
            o.yaw_deg_per_frame = float( std::atof( need() ) );
        else if( a == "--depth-units" )
            o.depth_units = float( std::atof( need() ) );
        else if( a == "--restart-at" )
            o.restart_at = std::atoi( need() );
        else if( a == "--seed" )
            o.seed = uint32_t( std::strtoul( need(), nullptr, 10 ) );
        else if( a == "--no-ir" )
            o.infrared = false;
        else if( a == "--no-imu" )
            o.imu = false;
        else if( a == "--compress" )
            o.compress = true;
        else if( a == "--fast" )
            o.pace = false;
        else if( a == "-h" || a == "--help" )
            return usage(), 0;
        else
        {
            std::fprintf( stderr, "unknown argument: %s\n", a.c_str() );
            usage();
            return 2;
        }
    }
    if( o.path.empty() )
    {
        usage();
        return 2;
    }
    try
    {
        auto info = ifs::test::write_test_recording( o );
        std::printf( "%s: %d frames, depth %dx%d (fx %.1f), color %dx%d (fx %.1f), scene %s\n", o.path.c_str(),
                     info.frames, info.depth_intrinsics.width, info.depth_intrinsics.height, info.depth_intrinsics.fx,
                     info.color_intrinsics.width, info.color_intrinsics.height, info.color_intrinsics.fx,
                     o.scene.c_str() );
        return 0;
    }
    catch( const std::exception & e )
    {
        std::fprintf( stderr, "make-test-recording: %s\n", e.what() );
        return 1;
    }
}
