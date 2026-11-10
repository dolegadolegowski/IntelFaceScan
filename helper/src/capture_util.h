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

// True when librealsense's error says this process has no right to the USB device: opening the camera in-process
// (taking it over from the macOS drivers) needs root, patches/0001. A process of the user gets the camera through the
// RealSense camera service instead, and ends up here only when librealsense could not use the service.
bool is_usb_access_error( const std::string & librealsense_error );

// The error to report when a connected camera could not be opened ('librealsense_error' is what librealsense said).
// For a process that is not root and lacks the right to the USB device, it names the cause the user can act on: the
// RealSense camera service is not installed or not answering.
std::string camera_open_error( const std::string & librealsense_error, bool as_root );

struct PixelRect
{
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
};

// Intersection of 'rect' with the image; nullopt when nothing is left.
std::optional< PixelRect > clamp_rect( const PixelRect & rect, int image_width, int image_height );

// 'rect' (inside the image) grown about its center to at least min_width x min_height (at most the image) and moved
// back inside the image where the growth crosses a border. Auto exposure metering a sliver of a few pixels (a face
// rectangle that mostly left the image) would follow those pixels instead of the face.
constexpr int k_min_roi_px = 32;
PixelRect ensure_min_size( const PixelRect & rect, int min_width, int min_height, int image_width, int image_height );

// Maps a rectangle of color pixels onto the depth image: the rectangle's outline is deprojected at distance_m,
// moved into the depth sensor frame and projected; the result is the bounding box, clamped to the depth image.
// nullopt when the rectangle does not land on the depth image.
std::optional< PixelRect > map_color_rect_to_depth( const rs2_intrinsics & color,
                                                    const rs2_intrinsics & depth,
                                                    const rs2_extrinsics & color_to_depth,
                                                    const PixelRect & color_rect,
                                                    float distance_m );

// Values of the options a configuration sets after the visual preset, plus the preset (empty = left alone). The
// engine applies them in this order (docs/PROTOCOL.md): preset, depth units, disparity shift, emitter on / laser
// power / emitter off, depth and color auto exposure.
struct OptionValues
{
    std::optional< float > preset;
    std::optional< float > depth_units;
    std::optional< int > disparity_shift;
    std::optional< float > emitter;
    std::optional< float > laser_power;
    std::optional< float > depth_auto_exposure;
    std::optional< float > color_auto_exposure;

    bool empty() const
    {
        return ! preset && ! depth_units && ! disparity_shift && ! emitter && ! laser_power && ! depth_auto_exposure
            && ! color_auto_exposure;
    }
};

// librealsense re-applies an active visual preset when the depth sensor opens. Given the options read before and
// after the open, the ones to set back: those that changed (never the preset itself), and an emitter that was off
// whenever the laser power is set back (setting the power switches the D4xx emitter on).
OptionValues options_to_restore( const OptionValues & before, const OptionValues & now );

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
