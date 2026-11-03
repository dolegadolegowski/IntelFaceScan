#!/bin/bash
# Builds the patched librealsense v2.58.4 SDK (shared library + command line tools) natively for Apple Silicon.
#
#   scripts/build-sdk.sh                 Release build, installed into build/sdk/install
#   BUILD_TYPE=Debug scripts/build-sdk.sh
#   IFS_SDK_DIR=/path scripts/build-sdk.sh   use another build/install location
#
# Only the camera helper links librealsense; the SwiftUI app never does (see docs/ARCHITECTURE.md).
# The patches from patches/ come from https://github.com/dolegadolegowski/realsense-viewer-macos and must not be
# reverted: they fix double sensor power release (libc++ std::function moves), infinite USB waits, libusb handle
# thread safety on macOS, the device command queue, and capture the camera once for the lifetime of the process.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/third_party/librealsense"
SDK_DIR="${IFS_SDK_DIR:-$ROOT/build/sdk}"
BUILD="$SDK_DIR/build"
PREFIX="$SDK_DIR/install"
BUILD_TYPE="${BUILD_TYPE:-Release}"
DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-27.2}"
JOBS="$(sysctl -n hw.ncpu)"
# Keep absolute build paths (and with them the local user name) out of the binaries (__FILE__ in log/assert macros)
PREFIX_MAP="-ffile-prefix-map=$ROOT=. -ffile-prefix-map=$BUILD=build"

if [ "$(uname -m)" != "arm64" ]; then
    echo "error: IntelFaceScan targets Apple Silicon (arm64) Macs" >&2
    exit 1
fi
command -v cmake >/dev/null || { echo "error: cmake not found (brew install cmake)" >&2; exit 1; }

if [ ! -f "$SRC/CMakeLists.txt" ]; then
    git -C "$ROOT" submodule update --init --depth 1 third_party/librealsense
fi
"$ROOT/scripts/apply-patches.sh"

if [ -f "$PREFIX/lib/librealsense2.dylib" ] && [ "${IFS_SDK_REBUILD:-0}" != "1" ] \
   && [ "$PREFIX/lib/librealsense2.dylib" -nt "$ROOT/patches" ]; then
    echo "librealsense already built: $PREFIX (IFS_SDK_REBUILD=1 to force)"
    exit 0
fi

cmake -S "$SRC" -B "$BUILD" -G "Unix Makefiles" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DCMAKE_OSX_ARCHITECTURES=arm64 \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET" \
    -DCMAKE_C_FLAGS="$PREFIX_MAP" \
    -DCMAKE_CXX_FLAGS="$PREFIX_MAP" \
    -DBUILD_SHARED_LIBS=ON \
    -DFORCE_RSUSB_BACKEND=ON \
    -DUSE_EXTERNAL_USB=ON \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_GLSL_EXTENSIONS=OFF \
    -DBUILD_TOOLS=ON \
    -DBUILD_UNIT_TESTS=OFF \
    -DBUILD_WITH_OPENMP=OFF \
    -DBUILD_WITH_DDS=OFF \
    -DCHECK_FOR_UPDATES=OFF \
    -DIMPORT_DEPTH_CAM_FW=OFF \
    -DENABLE_CCACHE=OFF

cmake --build "$BUILD" -j "$JOBS"
cmake --install "$BUILD" >/dev/null

# nlohmann/json is fetched by librealsense at configure time; the helper uses the same copy
JSON_HPP="$(find "$BUILD" -path '*include/nlohmann/json.hpp' -print -quit)"
if [ -n "$JSON_HPP" ]; then
    rm -rf "$PREFIX/include/nlohmann"
    mkdir -p "$PREFIX/include/nlohmann"
    cp -R "$(dirname "$JSON_HPP")/." "$PREFIX/include/nlohmann/"
fi

echo "librealsense installed: $PREFIX"
