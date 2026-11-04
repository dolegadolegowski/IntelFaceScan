// License: Apache 2.0. See LICENSE file in root directory.
//
// librealsense helpers shared by the capture engine and the probe: protocol names of streams/formats/models,
// calibration and option descriptions as JSON, the auto-exposure ROI mapping and stream-restart detection.

#pragma once

#include <librealsense2/rs.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace ifs
{

// Protocol spelling (lower case, snake_case) of librealsense enums: "depth", "z16", "inverse_brown_conrady",
// "global_time", ...
std::string stream_name( rs2_stream stream );
std::string format_name( rs2_format format );
std::string distortion_name( rs2_distortion model );
std::string timestamp_domain_name( rs2_timestamp_domain domain );

nlohmann::json intrinsics_json( const rs2_intrinsics & intrinsics );
// rotation column-major as in rs2_extrinsics, translation in meters
nlohmann::json extrinsics_json( const rs2_extrinsics & extrinsics );
nlohmann::json profile_json( const rs2::stream_profile & profile );

rs2_extrinsics identity_extrinsics();
rs2_extrinsics inverse( const rs2_extrinsics & e );

// "depth", "color", "motion" or "" for sensors the protocol does not describe.
std::string sensor_kind( const rs2::sensor & sensor );

// SensorOption list of docs/PROTOCOL.md. Options that cannot be read right now (some only while streaming)
// are left out rather than reported with made-up values.
nlohmann::json sensor_options_json( const rs2::sensor & sensor );

// Case-insensitive ASCII comparison (option and preset names come from people).
bool iequals( const std::string & a, const std::string & b );

// The option called 'name' (rs2_option_to_string spelling, case-insensitive) or nullopt.
std::optional< rs2_option > option_from_name( const std::string & name );

// Value of a camera info field, or 'fallback' when the device does not report it.
std::string camera_info( const rs2::device & device, rs2_camera_info info, const std::string & fallback = "" );

struct PixelRect
{
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
};

// Intersection of 'rect' with the image; nullopt when nothing is left.
std::optional< PixelRect > clamp_rect( const PixelRect & rect, int image_width, int image_height );

// Maps a rectangle of color pixels onto the depth image: the rectangle's outline is deprojected at distance_m,
// moved into the depth sensor frame and projected; the result is the bounding box, clamped to the depth image.
// nullopt when the rectangle does not land on the depth image.
std::optional< PixelRect > map_color_rect_to_depth( const rs2_intrinsics & color,
                                                    const rs2_intrinsics & depth,
                                                    const rs2_extrinsics & color_to_depth,
                                                    const PixelRect & color_rect,
                                                    float distance_m );

// Detects a stream restart from its frame counter: the D4xx firmware restarts depth (counter back near 0) when the
// RGB stream is toggled while depth streams.
class FrameCounterMonitor
{
public:
    // Returns true when the counter went backwards since the previous frame.
    bool update( unsigned long long frame_number );
    void reset() { m_last.reset(); }
    std::optional< unsigned long long > last() const { return m_last; }

private:
    std::optional< unsigned long long > m_last;
};

// Theoretical stereo range and depth noise (docs/CAMERA.md): Min-Z = f*B/(max_disparity + shift),
// Max-Z = f*B/shift (unbounded for shift 0), RMS = z^2 * subpixel / (f*B).
struct StereoRange
{
    double min_z_m = 0;
    double max_z_m = 0;  // 0 = unbounded
};
constexpr double k_d4xx_max_disparity = 126.0;
StereoRange stereo_range( double focal_px, double baseline_m, int disparity_shift,
                          double max_disparity = k_d4xx_max_disparity );
double stereo_rms_error_m( double z_m, double focal_px, double baseline_m, double subpixel = 0.08 );

}  // namespace ifs
