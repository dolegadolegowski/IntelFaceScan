// License: Apache 2.0. See LICENSE file in root directory.
//
// Post-processing of synchronized depth+color framesets into protocol Framesets:
//   threshold -> depth->disparity -> spatial -> temporal -> disparity->depth -> hole filling -> align to color
// plus the FRAMESET metadata (timestamps, depth scale, depth->color extrinsics, IMU, per-frame metadata).
// Runs on the capture engine's processing thread; the settings it reads are shared with the API thread.

#pragma once

#include "capture_config.h"
#include "capture_engine.h"
#include "capture_util.h"

#include <librealsense2/rs.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

namespace ifs
{

// What the processing thread produces; changed by configure()/set_filters() between framesets.
struct ProcessingSettings
{
    FilterConfig filters;
    bool send_depth = false;
    bool send_ir = false;
};

class ProcessingControl
{
public:
    void set( const ProcessingSettings & settings );
    void set_filters( const FilterConfig & filters );
    ProcessingSettings get() const;
    // Increments on every change so the processing thread only reconfigures its blocks when needed.
    uint64_t version() const { return m_version.load(); }

private:
    mutable std::mutex m_mutex;
    ProcessingSettings m_settings;
    std::atomic< uint64_t > m_version{ 1 };
};

// Newest accelerometer / gyroscope samples (written from the motion sensor's callback thread).
class ImuState
{
public:
    void on_frame( const rs2::frame & frame );
    void reset();
    // FRAMESET "imu" value: {"accel","gyro","timestamp_ms"} or null before the first sample.
    nlohmann::json snapshot() const;

private:
    mutable std::mutex m_mutex;
    std::optional< std::array< float, 3 > > m_accel;
    std::optional< std::array< float, 3 > > m_gyro;
    double m_timestamp_ms = 0;
};

// rs2::syncer learns which streams exist from the frames themselves, so the first frames of a stream start come
// out as separate one-stream sets (color, then depth, then infrared). This collects such parts again while their
// timestamps are within half a frame interval and returns them as one set once every expected stream is there;
// complete sets pass through unchanged.
class FramesetPairer
{
public:
    // Streams (type, index) a complete set contains, e.g. depth 0, color 0 and infrared 1.
    explicit FramesetPairer( std::vector< std::pair< rs2_stream, int > > expected );
    FramesetPairer( const FramesetPairer & ) = delete;
    FramesetPairer & operator=( const FramesetPairer & ) = delete;

    // The complete frameset, or an empty one while parts wait for the rest.
    rs2::frameset pair( const rs2::frameset & frames );

private:
    bool complete( const std::vector< rs2::frame > & frames ) const;

    std::vector< std::pair< rs2_stream, int > > m_expected;
    std::vector< rs2::frame > m_pending;
    std::vector< rs2::frame > m_parts;
    rs2::processing_block m_combine;
    rs2::frame_queue m_out{ 1 };
};

class FrameProcessor
{
public:
    // depth_to_color is the extrinsics between the active depth and color profiles (constant for a session).
    explicit FrameProcessor( const rs2_extrinsics & depth_to_color );

    // Returns nullopt when the frameset lacks depth or color (the syncer passes incomplete sets at stream start or
    // after a dropped frame).
    std::optional< Frameset > process( const rs2::frameset & frames,
                                       const ProcessingControl & control,
                                       const nlohmann::json & imu,
                                       uint64_t sequence );

    // Forgets the temporal filter's history (e.g. after the depth stream restarted).
    void reset_history();

private:
    void configure( const ProcessingSettings & settings );

    rs2_extrinsics m_depth_to_color;
    ProcessingSettings m_settings;
    uint64_t m_version = 0;
    float m_depth_units = 0;  // of the previous frameset's depth frame

    rs2::threshold_filter m_threshold;
    rs2::disparity_transform m_to_disparity{ true };
    rs2::spatial_filter m_spatial;
    std::optional< rs2::temporal_filter > m_temporal;
    rs2::disparity_transform m_to_depth{ false };
    rs2::hole_filling_filter m_hole_filling;
    rs2::align m_align{ RS2_STREAM_COLOR };
};

// Builds the plane descriptor of docs/PROTOCOL.md ("offset"/"bytes" are added by the IPC layer) and keeps the
// frame alive with the plane.
Plane make_plane( const char * kind, const rs2::video_frame & frame, const rs2_intrinsics & intrinsics );

// Per-frame metadata that the firmware reports ("color_exposure_us", "depth_exposure_us", "laser_power", ...).
nlohmann::json frame_metadata( const rs2::frame & depth, const rs2::frame & color );

}  // namespace ifs
