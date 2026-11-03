#!/bin/bash
# Runs the Swift Testing suites. Extra arguments go to `swift test` (e.g. --filter ICP).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
EXTRA=()
# With only the Command Line Tools installed, SwiftPM does not find the Swift Testing macro plugin by itself.
DEV="$(xcode-select -p 2>/dev/null || true)"
PLUGINS="$DEV/usr/lib/swift/host/plugins/testing"
if [[ "$DEV" == *CommandLineTools* ]] && [ -d "$PLUGINS" ]; then
    EXTRA+=(-Xswiftc -plugin-path -Xswiftc "$PLUGINS")
fi
exec swift test ${EXTRA[@]+"${EXTRA[@]}"} "$@"
