// License: Apache 2.0. See LICENSE file in root directory.
//
// Test seam of the capture engine (not used by the helper): a "live" engine around any librealsense device, e.g. an
// rs2::software_device standing in for the camera, so the live code path (configure, options, recording) runs in
// tests without hardware.

#pragma once

#include "capture_engine.h"

#include <librealsense2/rs.hpp>

#include <memory>

namespace ifs
{
std::unique_ptr< CaptureEngine > create_live_engine_for_device( rs2::device device, CaptureCallbacks callbacks );

// The same with the context the engine watches for device changes (its devices-changed callback): a test can add
// and remove devices in it (rs2::context::load_device / unload_device) to play a disconnect and a reconnect.
std::unique_ptr< CaptureEngine > create_live_engine_for_device( rs2::context context, rs2::device device,
                                                                CaptureCallbacks callbacks );

// Records an unplug of the camera of a live engine, as its devices-changed callback does when librealsense reports
// the removal (for devices a test cannot remove from a context, such as an rs2::software_device).
void mark_device_lost_for_testing( CaptureEngine & engine );
}
