#!/bin/bash
# Builds IntelFaceScan from scratch: patched librealsense, the camera helper, the Swift app and the app bundle.
#
#   scripts/build.sh                 -> build/IntelFaceScan.app
#   BUILD_TYPE=Debug scripts/build.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Release}"
DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-27.2}"
export MACOSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET"

if [ "$(uname -m)" != "arm64" ]; then
    echo "error: IntelFaceScan targets Apple Silicon (arm64) Macs" >&2
    exit 1
fi
OS="$(sw_vers -productVersion)"
if [ "$(printf '%s\n27.2\n' "$OS" | sort -V | head -1)" != "27.2" ]; then
    echo "error: macOS 27.2 or newer is required (this is $OS)" >&2
    exit 1
fi
for tool in cmake git clang swift; do
    command -v "$tool" >/dev/null || { echo "error: '$tool' not found (brew install cmake; xcode-select --install)" >&2; exit 1; }
done

cd "$ROOT"
git submodule update --init --depth 1 --recursive

# 1. librealsense v2.58.4 + macOS patches
BUILD_TYPE="$BUILD_TYPE" "$ROOT/scripts/build-sdk.sh"

# 2. camera helper
cmake -S "$ROOT/helper" -B "$ROOT/build/helper" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DIFS_SDK_DIR="$ROOT/build/sdk/install" -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET"
cmake --build "$ROOT/build/helper" -j "$(sysctl -n hw.ncpu)"

# 3. Swift app, reconstruction and CLI
CONFIG="$(echo "$BUILD_TYPE" | tr '[:upper:]' '[:lower:]')"
[ "$CONFIG" = "release" ] || CONFIG=debug
swift build -c "$CONFIG"

# 4. app bundle
SWIFT_CONFIG="$CONFIG" "$ROOT/scripts/bundle-app.sh"
