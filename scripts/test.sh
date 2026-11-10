#!/bin/bash
# Runs the Swift Testing suites (docs/TESTING.md). Extra arguments go to `swift test` (e.g. --filter ICP).
#
# The other tests have their own entry points (none needs a camera, root or the camera service):
#   scripts/test-build-scripts.sh                               the SDK bookkeeping of the build scripts
#   ctest --test-dir build/helper --output-on-failure           the helper (configure it with -DIFS_HELPER_TESTS=ON)
#   third_party/realsense-usbd/tests/test-camera-access.sh build/usbd/realsense-usbd
#                                                               the camera service's installer scripts, as dry runs
#                                                               (scripts/build.sh builds build/usbd/realsense-usbd)
#
# Environment:
#   IFS_TEST_RECORDINGS   folder of .db3/.bag recordings to reconstruct end to end (outside git)
#   IFS_HELPER            IntelFaceScanHelper used for recordings (default: build/helper/IntelFaceScanHelper)
#   IFS_HELPER_PATH       IntelFaceScanHelper for the CameraKit integration tests (same default; skipped without one)
#   DEVELOPER_DIR         e.g. /Library/Developer/CommandLineTools when the selected Xcode's license is not accepted
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
EXTRA=()
# With only the Command Line Tools installed, SwiftPM does not find the Swift Testing macro plugin by itself.
DEV="$(xcode-select -p 2>/dev/null || true)"
PLUGINS="$DEV/usr/lib/swift/host/plugins/testing"
if [[ "$DEV" == *CommandLineTools* ]]; then
    if [ -d "$PLUGINS" ]; then
        EXTRA+=(-Xswiftc -plugin-path -Xswiftc "$PLUGINS")
    else
        echo "warning: $PLUGINS not found; the Swift Testing macros may not build (update the Command Line Tools)" >&2
    fi
fi
exec swift test ${EXTRA[@]+"${EXTRA[@]}"} "$@"
