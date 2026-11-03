// License: Apache 2.0. See LICENSE file in root directory.
//
// Camera measurement tool (owner: capture module): `IntelFaceScanHelper --probe ...` (run with sudo).
// Enumerates the device and profiles and measures depth noise / fill rate / Min-Z for candidate configurations.

#pragma once

namespace ifs
{
// argv excludes the program name and the "--probe" switch. Returns the process exit code.
int run_probe( int argc, char ** argv );
}
