// License: Apache 2.0. See LICENSE file in root directory.
//
// The camera access service of the RealSense tools (the root LaunchDaemon realsense-usbd, installed by RealSense
// Viewer): the patched librealsense (patches/0004) talks to it over XPC, so the helper needs no root and no
// administrator password.
#pragma once

namespace ifs
{
// True when launchd knows the service's Mach name, i.e. the daemon is installed and loaded. It says nothing about
// whether this build is on the daemon's allowlist: the daemon decides that when librealsense connects.
bool camera_access_service_registered();
}  // namespace ifs
