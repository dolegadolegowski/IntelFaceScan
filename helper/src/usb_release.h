// License: Apache 2.0. See LICENSE file in root directory.
//
// Hands RealSense cameras back to macOS (port of tools/release/rs-macos-release.c).
//
// librealsense (RSUSB backend) captures the whole USB device from the macOS UVC/HID drivers, which requires root.
// After the capturing process is done the device stays captured: the system drivers are not re-attached and only
// root can open it until it is re-plugged. Re-enumerating the device releases the capture, re-attaches the system
// drivers (the camera works as a webcam again) and also resets a camera whose firmware stopped responding.

#pragma once

#include <functional>
#include <string>

namespace ifs
{

struct UsbReleaseResult
{
    int found = 0;          // RealSense devices on the bus (vendor 0x8086 / 0x38E5, product string "RealSense")
    int reenumerated = 0;   // devices without system drivers that were re-enumerated
    int failed = 0;         // re-enumerations that failed
    bool attached = false;  // after the wait: every device has its macOS drivers again
};

// Re-enumerates every RealSense device that has no macOS driver attached. With wait_ms > 0, waits up to that long
// until the system drivers are attached again. 'log' receives one line per device and the final state.
UsbReleaseResult release_realsense_devices( int wait_ms, const std::function< void( const std::string & ) > & log );

// `IntelFaceScanHelper --release [-w]`: same lines and exit codes as rs-macos-release, on stderr (the app's
// administrator launch discards stdout and shows stderr when the run fails).
int run_release_tool( int argc, char ** argv );

}  // namespace ifs
