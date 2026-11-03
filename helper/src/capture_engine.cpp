// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: capture module) - librealsense implementation of CaptureEngine.

#include "capture_engine.h"

#include <librealsense2/rs.hpp>

#include <stdexcept>

namespace ifs
{

std::unique_ptr< CaptureEngine > CaptureEngine::create_live( CaptureCallbacks )
{
    throw std::runtime_error( "CaptureEngine::create_live is not implemented yet" );
}

std::unique_ptr< CaptureEngine > CaptureEngine::create_playback( const std::string &, bool, CaptureCallbacks )
{
    throw std::runtime_error( "CaptureEngine::create_playback is not implemented yet" );
}

std::string sdk_version()
{
    return std::to_string( RS2_API_MAJOR_VERSION ) + "." + std::to_string( RS2_API_MINOR_VERSION ) + "."
         + std::to_string( RS2_API_PATCH_VERSION );
}

}  // namespace ifs
