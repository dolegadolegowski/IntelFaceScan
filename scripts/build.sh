#!/bin/bash
# Builds IntelFaceScan from scratch: patched librealsense, the camera helper, the Swift app and the app bundle.
#
#   scripts/build.sh                 -> build/IntelFaceScan.app
#   BUILD_TYPE=Debug scripts/build.sh
#   IFS_SDK_DIR=/path scripts/build.sh   build/reuse librealsense there (default build/sdk; the install prefix is
#                                        $IFS_SDK_DIR/install)
#   PACKAGE=1 scripts/build.sh       also dist/IntelFaceScan-<version>-macOS-arm64.dmg (scripts/package.sh)
#
# Needs: an Apple Silicon Mac; the Command Line Tools (xcode-select --install) or Xcode 27 with the macOS 27.2 SDK;
# CMake 3.20+ (brew install cmake); network access for the first build (git submodules, and librealsense downloads
# libusb, nlohmann/json, fastcdr, yaml-cpp and sqlite while it configures). No other Homebrew packages: libusb is
# built from source and linked statically. Metal shaders are compiled at runtime, so no Xcode-only tools are used.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Release}"
DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-27.2}"
export MACOSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET"
export IFS_SDK_DIR="${IFS_SDK_DIR:-$ROOT/build/sdk}"
REPO_URL="https://github.com/dolegadolegowski/IntelFaceScan.git"

die() { echo "error: $*" >&2; exit 1; }
warn() { echo "warning: $*" >&2; }
# version_at_least <version> <minimum>
version_at_least() { [ "$(printf '%s\n%s\n' "$1" "$2" | sort -V | head -n 1)" = "$2" ]; }

# --- What a clean Mac needs, checked before anything is built ---------------------------------------------------
[ "$(uname -m)" = "arm64" ] || die "IntelFaceScan targets Apple Silicon (arm64) Macs"

# /usr/bin/git, clang and swift exist even without developer tools (they only open the installer), so ask
# xcode-select and xcrun instead of looking at PATH.
DEVELOPER_DIR_PATH="$(xcode-select -p 2>/dev/null || true)"
[ -n "$DEVELOPER_DIR_PATH" ] && [ -d "$DEVELOPER_DIR_PATH" ] \
    || die "no developer tools found: install the Command Line Tools with 'xcode-select --install' (or Xcode 27)"
if ! SDK_VERSION="$(xcrun --sdk macosx --show-sdk-version 2>&1)"; then
    echo "$SDK_VERSION" >&2
    if [[ "$SDK_VERSION" == *license* ]]; then
        die "the selected Xcode ($DEVELOPER_DIR_PATH) needs its license accepted: 'sudo xcodebuild -license accept'," \
            "or build with the Command Line Tools: DEVELOPER_DIR=/Library/Developer/CommandLineTools scripts/build.sh"
    fi
    die "xcrun cannot find the macOS SDK in $DEVELOPER_DIR_PATH (reinstall the Command Line Tools: xcode-select --install)"
fi
version_at_least "$SDK_VERSION" 27.2 \
    || die "the macOS $SDK_VERSION SDK is too old; IntelFaceScan needs the macOS 27.2 SDK (update the Command Line Tools" \
           "with 'softwareupdate --list' / System Settings > Software Update, or install Xcode 27.2+)"
OS="$(sw_vers -productVersion)"
version_at_least "$OS" 27.2 || warn "this Mac runs macOS $OS; the app needs macOS 27.2 or newer to run (building works)"

for tool in git clang swift install_name_tool otool; do
    xcrun --find "$tool" >/dev/null 2>&1 || die "'$tool' is missing from $DEVELOPER_DIR_PATH (reinstall the Command Line Tools: xcode-select --install)"
done
for tool in iconutil codesign plutil xattr sw_vers sysctl; do
    command -v "$tool" >/dev/null || die "'$tool' not found (part of macOS; is /usr/bin on PATH?)"
done
command -v cmake >/dev/null \
    || die "cmake not found: install it with 'brew install cmake' (https://brew.sh) or from https://cmake.org/download and put it on PATH"
CMAKE_VERSION="$(cmake --version | head -n 1 | awk '{ print $3 }')"
version_at_least "$CMAKE_VERSION" 3.20 || die "cmake $CMAKE_VERSION is too old; 3.20 or newer is required (brew upgrade cmake)"

cd "$ROOT"
JOBS="$(sysctl -n hw.ncpu)"
TIMES=()
STEP_START=$SECONDS
step_done() {
    TIMES+=("$1 $((SECONDS - STEP_START)) s")
    STEP_START=$SECONDS
}

# --- 0. third-party sources ---------------------------------------------------------------------------------------
if git -C "$ROOT" rev-parse --git-dir >/dev/null 2>&1; then
    git submodule update --init --depth 1 --recursive \
        || die "fetching the git submodules failed (network?); retry, or: git submodule update --init --depth 1 --recursive"
else
    for path in $(sed -n 's/^[[:space:]]*path[[:space:]]*=[[:space:]]*//p' "$ROOT/.gitmodules"); do
        [ -n "$(ls -A "$ROOT/$path" 2>/dev/null)" ] \
            || die "$ROOT is not a git checkout and $path is empty (GitHub's source archives do not include submodules): git clone --recursive $REPO_URL"
    done
fi
step_done "submodules"

# 1. librealsense v2.58.4 + macOS patches
BUILD_TYPE="$BUILD_TYPE" "$ROOT/scripts/build-sdk.sh"
step_done "librealsense"

# 2. camera helper
cmake -S "$ROOT/helper" -B "$ROOT/build/helper" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DIFS_SDK_DIR="$IFS_SDK_DIR/install" -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET"
cmake --build "$ROOT/build/helper" -j "$JOBS"
step_done "helper"

# 3. Swift app, reconstruction and CLI
CONFIG="$(echo "$BUILD_TYPE" | tr '[:upper:]' '[:lower:]')"
[ "$CONFIG" = "release" ] || CONFIG=debug
swift build -c "$CONFIG"
step_done "swift"

# 4. app bundle
SWIFT_CONFIG="$CONFIG" "$ROOT/scripts/bundle-app.sh"
step_done "bundle"

# 5. disk image (optional)
if [ "${PACKAGE:-0}" = "1" ]; then
    "$ROOT/scripts/package.sh"
    step_done "dmg"
fi

echo
SUMMARY="$(printf '%s, ' "${TIMES[@]}")"
echo "Built in $SECONDS s (${SUMMARY%, })"
echo "Run:   open \"$ROOT/build/IntelFaceScan.app\""
echo "Tests: scripts/test.sh"
