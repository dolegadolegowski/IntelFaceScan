// License: Apache 2.0. See LICENSE file in root directory.
//
// CaptureConfiguration / FilterConfiguration of docs/PROTOCOL.md as C++ values: strict JSON validation (the
// helper may run as root, so everything the app sends is checked before it reaches librealsense) and the
// reverse conversion. No librealsense dependency.

#pragma once

#include <nlohmann/json.hpp>

#include <optional>
#include <string>

namespace ifs
{

struct StreamConfig
{
    int width = 0;
    int height = 0;
    int fps = 0;

    bool operator==( const StreamConfig & o ) const { return width == o.width && height == o.height && fps == o.fps; }
    bool operator!=( const StreamConfig & o ) const { return ! ( *this == o ); }
};

// Defaults are FilterConfiguration.faceScan (Sources/CameraKit/CameraTypes.swift).
struct FilterConfig
{
    struct Threshold
    {
        bool enabled = true;
        float min_m = 0.2f;
        float max_m = 1.2f;
    } threshold;
    bool disparity_domain = true;
    struct Spatial
    {
        bool enabled = true;
        int magnitude = 2;
        float alpha = 0.5f;
        int delta = 20;
        int holes_fill = 0;
    } spatial;
    struct Temporal
    {
        bool enabled = true;
        float alpha = 0.4f;
        int delta = 20;
        int persistence = 3;
    } temporal;
    struct HoleFilling
    {
        bool enabled = false;
        int mode = 1;
    } hole_filling;

    // Every filter off: the raw depth of the camera (still aligned to color).
    static FilterConfig none();
};

// Defaults are CaptureConfiguration.faceScanD455.
struct CaptureConfig
{
    StreamConfig depth{ 1280, 720, 30 };
    StreamConfig color{ 1280, 800, 30 };
    bool enable_ir = false;
    bool enable_imu = true;
    std::string preset = "High Accuracy";
    std::optional< float > laser_power;  // empty = maximum of the device
    bool emitter_enabled = true;
    float depth_units = 0.0001f;
    int disparity_shift = 0;
    bool depth_auto_exposure = true;
    bool color_auto_exposure = true;
    bool send_depth = false;
    bool send_ir = false;
    bool align_to_color = true;
    FilterConfig filters;

    // Same streams (a change of anything else never needs a stream restart).
    bool same_streams( const CaptureConfig & o ) const
    {
        return depth == o.depth && color == o.color && enable_ir == o.enable_ir && enable_imu == o.enable_imu;
    }
};

// Throw std::runtime_error with a message naming the offending field, e.g.
// "CaptureConfiguration.filters.spatial.alpha: 2 is out of range [0.25, 1]".
// Keys must be known; "request_id" is ignored so the IPC layer may pass the whole command object.
FilterConfig parse_filter_config( const nlohmann::json & j );
CaptureConfig parse_capture_config( const nlohmann::json & j );

nlohmann::json to_json( const FilterConfig & config );
nlohmann::json to_json( const CaptureConfig & config );

}  // namespace ifs
