// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: platform module) - IntelFaceScanHelper entry point.
//
//   IntelFaceScanHelper --socket <path> --uid <uid> --gid <gid> --parent-pid <pid>            live camera (root)
//   IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --playback <file> [--no-real-time]
//   IntelFaceScanHelper --probe ...                                                            measurements (root)
//   IntelFaceScanHelper --release [-w]                                                         hand cameras back

#include "capture_engine.h"
#include "probe.h"

#include <cstdio>
#include <cstring>

int main( int argc, char ** argv )
{
    if( argc > 1 && std::strcmp( argv[1], "--probe" ) == 0 )
        return ifs::run_probe( argc - 2, argv + 2 );
    std::fprintf( stderr, "IntelFaceScanHelper (librealsense %s): not implemented yet\n", ifs::sdk_version().c_str() );
    return 64;
}
