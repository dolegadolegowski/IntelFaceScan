// License: Apache 2.0. See LICENSE file in root directory.
//
// CaptureEngine - everything the helper does with librealsense: device discovery, stream configuration,
// presets/options, post-processing filters, depth->color alignment, IMU, recording and playback.
//
// Owner: capture module. The helper's main/IPC code (platform module) only talks to this interface and never
// includes librealsense headers itself. JSON shapes are those of docs/PROTOCOL.md (DeviceInfo,
// CaptureConfiguration, FilterConfiguration, FRAMESET metadata).
//
// Threading: callbacks are invoked from librealsense / processing threads and must not block for long (the IPC
// layer queues framesets and drops old ones). Methods may be called from the IPC thread; implementations
// serialize them internally. Methods throw std::runtime_error with a user-readable message on failure.

#pragma once

#include <nlohmann/json.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ifs
{

// One pixel plane of a frameset. 'data' stays valid as long as 'keepalive' is held (e.g. an rs2::frame).
struct Plane
{
    nlohmann::json desc;               // {"kind","format","width","height","stride","frame_number","timestamp_ms",
                                       //  "intrinsics":{...}} - "offset"/"bytes" are filled by the IPC layer
    const uint8_t * data = nullptr;
    size_t bytes = 0;
    std::shared_ptr< void > keepalive;
};

struct Frameset
{
    nlohmann::json metadata;           // FRAMESET JSON without "planes" (sequence, timestamps, depth_scale,
                                       // depth_to_color, imu, metadata)
    std::vector< Plane > planes;       // color first, then depth_aligned, then optional depth / ir_left
};

struct CaptureCallbacks
{
    std::function< void( Frameset && ) > on_frameset;
    // event names of docs/PROTOCOL.md (EVENT), e.g. "device_disconnected", "stream_restarted", "playback_finished"
    std::function< void( const std::string & event, const nlohmann::json & data ) > on_event;
    // level: "debug" | "info" | "warning" | "error"
    std::function< void( const std::string & level, const std::string & message ) > on_log;
};

class CaptureEngine
{
public:
    // Live camera (requires root on macOS: the first stream start captures the USB device from the system drivers
    // for the lifetime of the process). Throws when no RealSense device is connected.
    static std::unique_ptr< CaptureEngine > create_live( CaptureCallbacks callbacks );

    // Plays a .db3/.bag recording (never used as root). real_time = false delivers every frame (offline processing).
    static std::unique_ptr< CaptureEngine > create_playback( const std::string & file, bool real_time,
                                                             CaptureCallbacks callbacks );

    virtual ~CaptureEngine() = default;

    // DeviceInfo JSON (docs/PROTOCOL.md).
    virtual nlohmann::json device_info() = 0;

    // Applies a CaptureConfiguration: profiles, preset, laser power, emitter, depth units, disparity shift
    // (advanced mode), auto exposure, filters. While streaming, restarts the streams if the profiles changed.
    virtual void configure( const nlohmann::json & configuration ) = 0;
    // Applies a FilterConfiguration to the next framesets (no restart).
    virtual void set_filters( const nlohmann::json & filters ) = 0;

    virtual void start() = 0;
    virtual void stop() = 0;
    virtual bool streaming() const = 0;

    // sensor: "depth" | "color"; option: librealsense option name (rs2_option_to_string).
    virtual void set_option( const std::string & sensor, const std::string & option, float value ) = 0;
    // Auto-exposure ROI given in color pixels; distance_m (<= 0 = unknown) maps it onto the depth sensor.
    virtual void set_auto_exposure_roi( int x, int y, int width, int height, float distance_m ) = 0;

    // Records the raw sensor streams into 'path' (a file inside the helper's root-private temporary directory,
    // chosen by the helper, never by the app). format: "db3" | "bag". May restart the streams.
    virtual void start_recording( const std::string & path, const std::string & format ) = 0;
    // Returns once the recording file is complete and closed.
    virtual void stop_recording() = 0;

    // Playback only.
    virtual void playback_control( bool pause, bool real_time ) = 0;

    virtual void hardware_reset() = 0;

    // Stops everything and destroys the librealsense context (closes all USB handles). Must be called before the
    // helper hands the USB device back to macOS. Idempotent.
    virtual void shutdown() = 0;
};

// librealsense version string, e.g. "2.58.4".
std::string sdk_version();

}  // namespace ifs
