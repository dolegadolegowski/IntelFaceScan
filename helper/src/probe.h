// License: Apache 2.0. See LICENSE file in root directory.
//
// Camera measurement tool (owner: capture module): `IntelFaceScanHelper --probe ...` (the live camera as the user
// through the RealSense camera service, or with sudo). Enumerates the device and profiles and measures depth noise /
// fill rate / Min-Z for candidate configurations.

#pragma once

namespace ifs
{
// argv excludes the program name and the "--probe" switch. Returns the process exit code. 'used_live_camera' (when
// given) tells whether the probe tried to open the live camera: a caller running as root then hands it back to macOS
// (USB re-enumeration, usb_release.h, which this module does not link).
int run_probe( int argc, char ** argv, bool * used_live_camera = nullptr );
}
