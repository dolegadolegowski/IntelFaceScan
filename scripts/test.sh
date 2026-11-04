#!/bin/bash
# Runs the Swift Testing suites (docs/TESTING.md). Extra arguments go to `swift test` (e.g. --filter ICP).
#
# Environment:
#   IFS_TEST_RECORDINGS   folder of .db3/.bag recordings to reconstruct end to end (outside git)
#   IFS_HELPER            IntelFaceScanHelper used for recordings (default: build/helper/IntelFaceScanHelper)
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
