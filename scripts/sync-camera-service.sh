#!/bin/bash
# Updates the vendored RealSense camera service kit, third_party/realsense-usbd, from its home: the folder macos/broker
# of a realsense-viewer-macos checkout (https://github.com/dolegadolegowski/realsense-viewer-macos). The copy here is
# identical to that folder, file for file, and is never edited in this repository: running this script is how it
# changes.
#
#   scripts/sync-camera-service.sh <path to a realsense-viewer-macos checkout>
#       copies <checkout>/macos/broker/ over third_party/realsense-usbd/ (rsync -rlp --checksum --delete): files
#       the kit no longer has are removed here too, and the executable bits follow the kit. File times are not
#       copied: a file whose content changed gets the time of the sync, a file with the same content is left alone.
#       The kit's build-daemon.sh compares file times, so a file that kept an older time from the other checkout
#       could stay uncompiled.
#   scripts/sync-camera-service.sh --check <path to a realsense-viewer-macos checkout>
#       changes nothing: lists every difference (file contents, files only on one side, executable bits) and exits
#       with status 1 when the copy is not identical
#
# What uses the kit: scripts/apply-patches.sh (librealsense/macos-usb-broker.patch, the client inside librealsense),
# scripts/build.sh (build-daemon.sh), scripts/bundle-app.sh (the daemon, the installer scripts, the manifest),
# Package.swift (client/RealSenseCameraAccess.swift) and the tests. After a sync that changed the patch, the next
# scripts/build.sh applies it and rebuilds librealsense. Commit the synced folder together with whatever had to follow
# it, and name the kit's commit in the message.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/third_party/realsense-usbd"

die() { echo "sync-camera-service.sh: $*" >&2; exit 1; }
usage() {
    echo "usage: scripts/sync-camera-service.sh [--check] <path to a realsense-viewer-macos checkout>" >&2
    exit 2
}

CHECK=0
if [ "${1:-}" = "--check" ]; then
    CHECK=1
    shift
fi
[ $# -eq 1 ] || usage
case "$1" in -*) usage ;; esac
[ -d "$1" ] || die "$1 is not a directory"
SRC="$(cd "$1" && pwd)/macos/broker"
# A folder that is not the kit must never be copied over it (--delete would empty the copy).
for item in rs-usb-broker-protocol.h librealsense/macos-usb-broker.patch daemon install-camera-access.sh; do
    [ -e "$SRC/$item" ] || die "$SRC/$item not found: $1 is not a realsense-viewer-macos checkout with the kit"
done
[ "$SRC" != "$DEST" ] || die "$SRC is the copy itself"

# The files of a kit folder that git would track as executable.
executables() { (cd "$1" && find . -type f ! -name .DS_Store -perm -u+x | LC_ALL=C sort); }

if [ "$CHECK" = 1 ]; then
    [ -d "$DEST" ] || die "$DEST does not exist"
    STATUS=0
    # Finder's .DS_Store files are not part of the kit.
    diff -rq -x .DS_Store "$SRC" "$DEST" || STATUS=1
    if [ "$(executables "$SRC")" != "$(executables "$DEST")" ]; then
        echo "the executable files differ:"
        # diff ends with status 1 here (the lists differ): without "|| true", pipefail and set -e would end the script
        # before the status is set and the closing line is printed.
        diff <(executables "$SRC") <(executables "$DEST") \
            | sed -n 's/^< /    only executable in the kit: /p; s/^> /    only executable in the copy: /p' || true
        STATUS=1
    fi
    if [ "$STATUS" = 0 ]; then
        echo "third_party/realsense-usbd is identical to $SRC"
    else
        echo "third_party/realsense-usbd differs from $SRC (scripts/sync-camera-service.sh $1 updates it)" >&2
    fi
    exit "$STATUS"
fi

mkdir -p "$DEST"
# No -t: a file whose content changed gets the time of the sync, so build-daemon.sh (which compares file times)
# compiles it again; --checksum leaves files with unchanged content, and their times, alone.
rsync -rlp --checksum --delete --exclude .DS_Store "$SRC/" "$DEST/"
COMMIT="$(git -C "$1" rev-parse --short HEAD 2>/dev/null || echo "unknown commit")"
if [ -n "$(git -C "$1" status --porcelain -- macos/broker 2>/dev/null)" ]; then
    COMMIT="$COMMIT with uncommitted changes in macos/broker"
fi
echo "third_party/realsense-usbd now equals macos/broker of $1 ($COMMIT)"
echo "next: scripts/build.sh (applies a changed librealsense patch, rebuilds what changed), then the tests"
