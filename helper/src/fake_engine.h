// License: Apache 2.0. See LICENSE file in root directory.
//
// A CaptureEngine without a camera (`IntelFaceScanHelper --fake`): synthetic framesets streamed through the same IPC
// code as the real camera, for the IPC tests and for UI development. Never touches librealsense or USB, so it runs
// as the user.
//
// Every frame: a moving color pattern (red = x + 3n, green = y + 2n, blue = (x ^ y) + n, all mod 256, n = color
// frame number) with a shaded sphere of radius 0.1 m drifting around (0, 0, 0.45 m) in front of the color camera,
// and the sphere's depth aligned to color (0 = no data around it). Intrinsics, extrinsics, IMU and metadata look
// like a D455 at the configured resolution. Options, presets and ranges behave like a device (errors included).
// Recording writes a small synthetic file ("IFSFAKE1" header) so the descriptor hand-over can be tested. A hardware
// reset plays an unplug and reconnect: device_disconnected + device_connected, and a running recording ends with the
// camera (as with the real engine, the RECORD_STOP that follows still succeeds).

#pragma once

#include "capture_engine.h"

#include <memory>

namespace ifs
{

struct FakeOptions
{
    // Extra bytes at the end of every color row (stride > width * 3), like a camera with aligned rows.
    int color_row_padding = 0;
};

std::unique_ptr< CaptureEngine > create_fake_engine( CaptureCallbacks callbacks, FakeOptions options );

}  // namespace ifs
