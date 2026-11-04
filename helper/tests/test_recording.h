// License: Apache 2.0. See LICENSE file in root directory.
//
// Synthetic D455-like recordings for tests without a camera: an rs2::software_device with the D455 sensor layout
// ("Stereo Module": depth Z16 + infrared, "RGB Camera": RGB8, "Motion Module": accel/gyro), factory-like
// intrinsics/extrinsics and metadata, rendering a ray-cast scene (a textured head-sized sphere with a nose in front
// of a wall, or a flat target) into depth, color and infrared, recorded with rs2::recorder to .db3 or .bag.
// Depth and color are rendered from their own viewpoints, so depth aligned to color is checkable against the scene.

#pragma once

#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace ifs
{
namespace test
{

struct RecordingOptions
{
    std::string path;               // .db3 or .bag (the extension selects the format)
    int frames = 30;
    int fps = 30;
    int depth_width = 1280;
    int depth_height = 720;
    int color_width = 1280;
    int color_height = 800;
    float depth_units = 0.0001f;    // meters per Z16 unit
    std::string scene = "head";     // "head" | "plane"
    float distance_m = 0.5f;        // head center, or the plane's distance on the optical axis
    float wall_m = 1.0f;            // head scene: background wall distance (0 = no wall)
    float plane_tilt_deg = 10.f;    // plane scene: rotation about the vertical axis
    float noise_m = 0.f;            // depth noise std at 1 m (grows with z^2 like stereo noise); 0 = exact
    float yaw_deg_per_frame = 0.f;  // head scene: rotation of the head per frame
    bool infrared = true;
    bool imu = true;
    bool compress = false;
    bool pace = true;               // feed frames at the frame rate (realistic file timestamps)
    int restart_at = -1;            // frame index where the depth frame counter starts over (firmware restart)
    uint32_t seed = 1;
};

struct RecordingInfo
{
    rs2_intrinsics depth_intrinsics{};
    rs2_intrinsics color_intrinsics{};
    rs2_extrinsics depth_to_color{};
    int frames = 0;
    double first_timestamp_ms = 0;
};

// The calibration the generator uses for a given resolution (D455-like values).
rs2_intrinsics synthetic_depth_intrinsics( int width, int height );
rs2_intrinsics synthetic_color_intrinsics( int width, int height );
rs2_extrinsics synthetic_depth_to_color();
constexpr float k_synthetic_baseline_mm = 95.f;
constexpr const char * k_synthetic_serial = "000000000455";

// The software device with the D455 sensor layout, which can also stand in for a live camera.
class SyntheticCamera
{
public:
    explicit SyntheticCamera( const RecordingOptions & options );  // 'path' is not used
    ~SyntheticCamera();
    SyntheticCamera( const SyntheticCamera & ) = delete;
    SyntheticCamera & operator=( const SyntheticCamera & ) = delete;

    rs2::software_device & device();
    const RecordingInfo & info() const;
    // Opens and starts every sensor with a callback that ignores the frames (for rs2::recorder).
    void start_all();
    void stop_all();
    // Feeds frame i to the sensors that stream (IMU samples up to its timestamp, color, depth, infrared); frames
    // of stopped sensors are dropped by librealsense.
    void emit_frame( int i );

private:
    struct Impl;
    std::unique_ptr< Impl > m;
};

// Throws std::runtime_error (or rs2::error) on failure.
RecordingInfo write_test_recording( const RecordingOptions & options );

// Ground truth: depth (z in the depth sensor frame, meters; 0 = nothing) seen along the color camera's pixel ray,
// for frame 'frame' of a recording made with 'options'.
float scene_depth_at_color_pixel( const RecordingOptions & options, int frame, float u, float v );
// Ground truth depth at a depth pixel (meters, 0 = nothing).
float scene_depth_at_depth_pixel( const RecordingOptions & options, int frame, float u, float v );

}  // namespace test
}  // namespace ifs
