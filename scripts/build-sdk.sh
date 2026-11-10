#!/bin/bash
# Builds the patched librealsense v2.58.4 SDK (shared library + command line tools) natively for Apple Silicon.
#
#   scripts/build-sdk.sh                 Release build, installed into build/sdk/install
#   BUILD_TYPE=Debug scripts/build-sdk.sh
#   IFS_SDK_DIR=/path scripts/build-sdk.sh   use another build/install location
#   IFS_SDK_REBUILD=1 scripts/build-sdk.sh   rebuild although the installed SDK is current
#
# install/.ifs-sdk-inputs records what the installed SDK was built from (librealsense commit, sha256 of the patches
# and of this script, build type, deployment target); the SDK is rebuilt when any of it differs.
# install/libusb/ keeps the static libusb 1.0.30 the librealsense build fetched and built (lib/libusb-1.0.a and
# include/libusb-1.0/libusb.h): scripts/build.sh links the camera service daemon against it, and the install prefix
# is all that CI caches of this build.
#
# Only the camera helper links librealsense; the SwiftUI app never does (see docs/ARCHITECTURE.md).
# The patches (scripts/apply-patches.sh applies them, in this order):
# * patches/0001 comes from https://github.com/dolegadolegowski/realsense-viewer-macos and must not be reverted: it
#   fixes double sensor power release (libc++ std::function moves), infinite USB waits, libusb handle thread safety
#   on macOS, the device command queue, and captures the camera once for the lifetime of the process. 0002 and 0003
#   are this project's fixes for recordings (docs/CAMERA.md).
# * third_party/realsense-usbd/librealsense/macos-usb-broker.patch is the client of the RealSense camera service
#   (realsense-usbd, a root LaunchDaemon shared by RealSense applications; the kit in third_party/realsense-usbd is a
#   copy of realsense-viewer-macos/macos/broker). With it librealsense asks the service for the camera first, so the
#   helper runs as the user; when the service is not installed or does not admit the user it opens the USB device
#   itself as before, which needs root.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/third_party/librealsense"
SDK_DIR="${IFS_SDK_DIR:-$ROOT/build/sdk}"
BUILD="$SDK_DIR/build"
PREFIX="$SDK_DIR/install"
STAMP="$PREFIX/.ifs-sdk-inputs"
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

# What the installed SDK was built from, kept next to it: a change to any of it (a pull that changes a patch, a
# librealsense bump, another build type or deployment target, an edit of this script) rebuilds the SDK. File times
# are not used: git operations touch patches/ without changing it.
sdk_inputs() {
    local commit="unknown (not a git checkout)"
    if [ "$(git -C "$SRC" rev-parse --show-toplevel 2>/dev/null)" = "$(cd "$SRC" && pwd -P)" ]; then
        commit="$(git -C "$SRC" rev-parse HEAD)"
    fi
    echo "librealsense $commit"
    echo "build_type $BUILD_TYPE"
    echo "deployment_target $DEPLOYMENT_TARGET"
    echo "build-sdk.sh $(shasum -a 256 "$ROOT/scripts/build-sdk.sh" | cut -d ' ' -f 1)"
    # "patch <sha256> <name>" for patches/*.patch and the camera service client, in the order they are applied
    "$ROOT/scripts/apply-patches.sh" --list
}
INPUTS="$(sdk_inputs)"
if [ -f "$PREFIX/lib/librealsense2.dylib" ] && [ "${IFS_SDK_REBUILD:-0}" != "1" ] \
   && [ -f "$STAMP" ] && [ "$(cat "$STAMP")" = "$INPUTS" ]; then
    echo "librealsense already built: $PREFIX (IFS_SDK_REBUILD=1 to force)"
    exit 0
fi
if [ -f "$STAMP" ] && [ -f "$PREFIX/lib/librealsense2.dylib" ]; then
    echo "librealsense in $PREFIX was built from other inputs, rebuilding; now:"
    diff "$STAMP" - <<<"$INPUTS" | sed -n 's/^> /    /p' || true
fi

# A build folder configured from another checkout (an IFS_SDK_DIR shared between worktrees) cannot build this one.
if [ -f "$BUILD/CMakeCache.txt" ]; then
    CONFIGURED_SRC="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$BUILD/CMakeCache.txt")"
    if [ -n "$CONFIGURED_SRC" ] && [ "$(cd "$CONFIGURED_SRC" 2>/dev/null && pwd -P)" != "$(cd "$SRC" && pwd -P)" ]; then
        echo "error: $BUILD was configured for $CONFIGURED_SRC, not $SRC: use another IFS_SDK_DIR for this" \
             "checkout, or remove $BUILD" >&2
        exit 1
    fi
fi
rm -f "$STAMP"

cmake -S "$SRC" -B "$BUILD" -G "Unix Makefiles" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DCMAKE_OSX_ARCHITECTURES=arm64 \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET" \
    -DCMAKE_INSTALL_RPATH="@loader_path/../lib" \
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
# Installing over tools that were not relinked makes install_name_tool fail (harmlessly, but loudly) on build rpaths
# the previous install already removed: install them fresh.
rm -rf "${PREFIX:?}/bin"
cmake --install "$BUILD" >/dev/null

# nlohmann/json is fetched by librealsense at configure time; the helper uses the same copy
JSON_HPP="$(find "$BUILD" -path '*include/nlohmann/json.hpp' -print -quit)"
if [ -n "$JSON_HPP" ]; then
    rm -rf "$PREFIX/include/nlohmann"
    mkdir -p "$PREFIX/include/nlohmann"
    cp -R "$(dirname "$JSON_HPP")/." "$PREFIX/include/nlohmann/"
fi

# The static libusb for the camera service daemon (see the header): only the archive and the header, not libusb's
# CMake package files, which name the build folder.
LIBUSB_BUILT="$BUILD/libusb_install"
if [ ! -f "$LIBUSB_BUILT/lib/libusb-1.0.a" ] || [ ! -f "$LIBUSB_BUILT/include/libusb-1.0/libusb.h" ]; then
    echo "error: the librealsense build left no static libusb in $LIBUSB_BUILT" >&2
    exit 1
fi
rm -rf "$PREFIX/libusb"
mkdir -p "$PREFIX/libusb/lib" "$PREFIX/libusb/include/libusb-1.0"
cp "$LIBUSB_BUILT/lib/libusb-1.0.a" "$PREFIX/libusb/lib/libusb-1.0.a"
cp "$LIBUSB_BUILT/include/libusb-1.0/libusb.h" "$PREFIX/libusb/include/libusb-1.0/libusb.h"

printf '%s\n' "$INPUTS" > "$STAMP"
echo "librealsense installed: $PREFIX"
