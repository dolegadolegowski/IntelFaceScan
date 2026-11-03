# License: Apache 2.0. See LICENSE file in root directory.
#
# IntelFaceScanHelper - the privileged camera helper (links the patched librealsense from scripts/build-sdk.sh).
#   cmake -S helper -B build/helper -DIFS_SDK_DIR=build/sdk/install && cmake --build build/helper
cmake_minimum_required(VERSION 3.20)
project(IntelFaceScanHelper LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OSX_ARCHITECTURES arm64 CACHE STRING "")
if(NOT CMAKE_OSX_DEPLOYMENT_TARGET)
    set(CMAKE_OSX_DEPLOYMENT_TARGET 27.2)
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(IFS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../build/sdk/install" CACHE PATH "librealsense install prefix")
find_package(realsense2 REQUIRED PATHS "${IFS_SDK_DIR}" NO_DEFAULT_PATH)

# All sources in src/ (the platform and capture work streams add files there independently).
file(GLOB IFS_HELPER_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")
add_executable(IntelFaceScanHelper ${IFS_HELPER_SOURCES})
target_include_directories(IntelFaceScanHelper PRIVATE
    src
    "${CMAKE_CURRENT_SOURCE_DIR}/../Sources/IFSProtocol/include"
    "${IFS_SDK_DIR}/include")
target_sources(IntelFaceScanHelper PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../Sources/IFSProtocol/ifs_protocol.c")
target_compile_options(IntelFaceScanHelper PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(IntelFaceScanHelper PRIVATE realsense2::realsense2 "-framework IOKit" "-framework CoreFoundation")
# The bundled helper loads librealsense from the app's Frameworks folder only.
set_target_properties(IntelFaceScanHelper PROPERTIES
    BUILD_RPATH "${IFS_SDK_DIR}/lib"
    INSTALL_RPATH "@executable_path/../Frameworks")
