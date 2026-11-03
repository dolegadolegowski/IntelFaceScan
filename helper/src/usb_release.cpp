// License: Apache 2.0. See LICENSE file in root directory.

#include "usb_release.h"

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/usb/IOUSBLib.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace ifs
{

namespace
{
constexpr SInt32 kRealSenseVendorIds[] = { 0x8086, 0x38E5 };  // Intel, RealSense Inc. (src/usb/usb-types.h)

bool is_realsense( io_service_t dev )
{
    bool result = false;
    auto vid_ref = static_cast< CFNumberRef >(
        IORegistryEntryCreateCFProperty( dev, CFSTR( kUSBVendorID ), kCFAllocatorDefault, 0 ) );
    auto name_ref = static_cast< CFStringRef >(
        IORegistryEntryCreateCFProperty( dev, CFSTR( kUSBProductString ), kCFAllocatorDefault, 0 ) );
    SInt32 vid = 0;
    char name[256] = "";
    if( vid_ref && CFGetTypeID( vid_ref ) == CFNumberGetTypeID() )
        CFNumberGetValue( vid_ref, kCFNumberSInt32Type, &vid );
    if( name_ref && CFGetTypeID( name_ref ) == CFStringGetTypeID() )
        CFStringGetCString( name_ref, name, sizeof( name ), kCFStringEncodingUTF8 );
    for( SInt32 v : kRealSenseVendorIds )
        if( vid == v && std::strstr( name, "RealSense" ) )
            result = true;
    if( vid_ref )
        CFRelease( vid_ref );
    if( name_ref )
        CFRelease( name_ref );
    return result;
}

// An interface without a child entry in the registry has no driver attached (the same test libusb uses).
bool drivers_attached( io_service_t dev )
{
    io_iterator_t it;
    int interfaces = 0, with_driver = 0;
    if( IORegistryEntryGetChildIterator( dev, kIOServicePlane, &it ) != KERN_SUCCESS )
        return false;
    while( io_service_t child = IOIteratorNext( it ) )
    {
        if( IOObjectConformsTo( child, "IOUSBHostInterface" ) )
        {
            io_registry_entry_t grandchild = IO_OBJECT_NULL;
            interfaces++;
            if( IORegistryEntryGetChildEntry( child, kIOServicePlane, &grandchild ) == KERN_SUCCESS && grandchild )
            {
                with_driver++;
                IOObjectRelease( grandchild );
            }
        }
        IOObjectRelease( child );
    }
    IOObjectRelease( it );
    return interfaces > 0 && interfaces == with_driver;
}

IOReturn reenumerate( io_service_t dev )
{
    IOCFPlugInInterface ** plugin = nullptr;
    IOUSBDeviceInterface650 ** usb = nullptr;
    SInt32 score;
    IOReturn kr = IOCreatePlugInInterfaceForService( dev, kIOUSBDeviceUserClientTypeID, kIOCFPlugInInterfaceID,
                                                     &plugin, &score );
    if( kr != kIOReturnSuccess || ! plugin )
        return kr ? kr : kIOReturnError;
    kr = ( *plugin )->QueryInterface( plugin, CFUUIDGetUUIDBytes( kIOUSBDeviceInterfaceID650 ),
                                      reinterpret_cast< LPVOID * >( &usb ) );
    ( *plugin )->Release( plugin );
    if( kr != kIOReturnSuccess || ! usb )
        return kr ? kr : kIOReturnError;
    kr = ( *usb )->USBDeviceOpenSeize( usb );
    if( kr == kIOReturnSuccess )
    {
        kr = ( *usb )->USBDeviceReEnumerate( usb, 0 );
        ( *usb )->USBDeviceClose( usb );
    }
    ( *usb )->Release( usb );
    return kr;
}

// Calls f for every RealSense device; returns false when the registry cannot be enumerated.
template< typename F >
bool for_each_realsense( F && f )
{
    io_iterator_t it;
    if( IOServiceGetMatchingServices( kIOMainPortDefault, IOServiceMatching( "IOUSBHostDevice" ), &it ) != KERN_SUCCESS )
        return false;
    while( io_service_t dev = IOIteratorNext( it ) )
    {
        if( is_realsense( dev ) )
            f( dev );
        IOObjectRelease( dev );
    }
    IOObjectRelease( it );
    return true;
}

// While a device re-enumerates it disappears from the registry for a moment: "all attached" needs at least one.
bool all_attached()
{
    int total = 0, ready = 0;
    if( ! for_each_realsense( [&]( io_service_t dev ) {
            total++;
            ready += drivers_attached( dev );
        } ) )
        return false;
    return total > 0 && ready == total;
}
}  // namespace

UsbReleaseResult release_realsense_devices( int wait_ms, const std::function< void( const std::string & ) > & log )
{
    UsbReleaseResult result;
    auto say = [&]( const std::string & line ) {
        if( log )
            log( line );
    };
    bool ok = for_each_realsense( [&]( io_service_t dev ) {
        result.found++;
        if( drivers_attached( dev ) )
        {
            say( "RealSense device already attached to macOS drivers" );
            return;
        }
        IOReturn kr = reenumerate( dev );
        char line[128];
        std::snprintf( line, sizeof( line ), "RealSense device re-enumerated: %s (0x%08x)",
                       kr == kIOReturnSuccess ? "ok" : "failed", kr );
        say( line );
        result.reenumerated++;
        result.failed += kr != kIOReturnSuccess;
    } );
    if( ! ok )
    {
        say( "failed to enumerate USB devices" );
        result.failed++;
        return result;
    }
    if( ! result.found )
    {
        say( "No RealSense device found" );
        return result;
    }
    if( wait_ms > 0 && ! result.failed )
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds( wait_ms );
        while( ! ( result.attached = all_attached() ) && std::chrono::steady_clock::now() < deadline )
            std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
        say( result.attached ? "macOS drivers attached" : "macOS drivers not attached yet" );
    }
    else
        result.attached = all_attached();
    return result;
}

int run_release_tool( int argc, char ** argv )
{
    bool wait = false;
    for( int i = 0; i < argc; ++i )
    {
        if( std::strcmp( argv[i], "-w" ) == 0 )
            wait = true;
        else
        {
            std::fprintf( stderr, "usage: IntelFaceScanHelper --release [-w]\n" );
            return 64;
        }
    }
    UsbReleaseResult r = release_realsense_devices( wait ? 5000 : 0, []( const std::string & line ) {
        std::printf( "%s\n", line.c_str() );
    } );
    if( r.failed )
        return r.found ? 1 : 2;
    return 0;
}

}  // namespace ifs
