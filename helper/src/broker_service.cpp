// License: Apache 2.0. See LICENSE file in root directory.

#include "broker_service.h"

#include <mach/mach.h>
#include <servers/bootstrap.h>

namespace ifs
{
bool camera_access_service_registered()
{
    mach_port_t port = MACH_PORT_NULL;
    // Must match RSUSB_SERVICE_NAME (third_party/librealsense/src/libusb/broker/rs-usb-broker-protocol.h).
    const kern_return_t kr = bootstrap_look_up( bootstrap_port, "io.github.dolegadolegowski.realsense-usbd", &port );
    if( kr != KERN_SUCCESS )
        return false;
    mach_port_deallocate( mach_task_self(), port );
    return true;
}
}  // namespace ifs
