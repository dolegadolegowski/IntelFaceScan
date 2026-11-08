#!/bin/bash
# Lets IntelFaceScan use the camera access service of the RealSense tools (the root LaunchDaemon realsense-usbd),
# so the live camera needs no administrator password per session (docs/HELPER.md, "Camera access").
#
#   sudo scripts/allow-camera-access.sh [--yes] [--helper <IntelFaceScanHelper>] [--viewer <RealSense Viewer.app>]
#
# It runs as root, so read what it is about to allow: it prints the helper and asks before changing anything
# (--yes skips the question). Run it from a copy you trust (the repository), not from a file others can edit.
#
# The daemon only serves programs whose code directory hash (cdhash) is on its allowlist. This script adds the
# helper's cdhash to the allowlist that ships in RealSense Viewer.app (its own entries are kept; hashes of earlier
# IntelFaceScan builds are dropped) and installs the result with the installer of that app. Run it once after installing or rebuilding the app:
#   * every build of the helper has another cdhash (the app is signed ad hoc), so a new build needs it again;
#   * installing camera access from a RealSense Viewer DMG replaces the allowlist, so run this again afterwards.
# Nothing else is changed: the daemon, its configuration and its policies stay as the Viewer installed them.
# Known side effect: the installed list no longer equals the Viewer's own, so the Viewer's launcher may say that
# camera access is newer than the app. Its "Run once as administrator" button still works; installing camera access
# from the Viewer's DMG replaces the list again (then run this script again).
set -euo pipefail
unset IFS CDPATH ENV BASH_ENV
export PATH=/usr/bin:/bin:/usr/sbin:/sbin

ME=allow-camera-access
die() { echo "$ME: error: $*" >&2; exit 1; }
note() { echo "$ME: $*"; }

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HELPER=
ASSUME_YES=0
VIEWER="/Applications/RealSense Viewer.app"
while [ $# -gt 0 ]; do
    case "$1" in
        --yes) ASSUME_YES=1; shift ;;
        --helper) [ $# -ge 2 ] || die "--helper needs a path"; HELPER="$2"; shift 2 ;;
        --viewer) [ $# -ge 2 ] || die "--viewer needs a path"; VIEWER="$2"; shift 2 ;;
        *) die "usage: sudo scripts/allow-camera-access.sh [--yes] [--helper <IntelFaceScanHelper>] [--viewer <RealSense Viewer.app>]" ;;
    esac
done
if [ -z "$HELPER" ]; then
    for candidate in /Applications/IntelFaceScan.app "$ROOT/build/IntelFaceScan.app"; do
        if [ -x "$candidate/Contents/Helpers/IntelFaceScanHelper" ]; then
            HELPER="$candidate/Contents/Helpers/IntelFaceScanHelper"
            break
        fi
    done
fi
[ -n "$HELPER" ] && [ -x "$HELPER" ] || die "IntelFaceScanHelper not found (build the app with scripts/build.sh or pass --helper)"
[ "$(id -u)" = 0 ] || die "run as root: sudo $0"

LABEL=io.github.dolegadolegowski.realsense-usbd
CONF="/Library/Application Support/RealSense Viewer/usbd.plist"
CONF_DIR="/Library/Application Support/RealSense Viewer"
BUNDLED="$VIEWER/Contents/Resources/realsense-usbd/allowlist.plist"
INSTALLER="$VIEWER/Contents/Resources/realsense-usbd/install-camera-access.sh"
DAEMON_DIR="$VIEWER/Contents/MacOS"
[ -f "$CONF" ] || die "camera access is not installed: install it from RealSense Viewer first (Viewer menu or its DMG)"
[ -f "$BUNDLED" ] || die "$BUNDLED not found"
[ -x "$INSTALLER" ] || die "$INSTALLER not found (is RealSense Viewer.app in /Applications?)"
[ -x "$DAEMON_DIR/realsense-usbd" ] || die "$DAEMON_DIR/realsense-usbd not found"

codesign --verify --strict "$HELPER" || die "$HELPER: invalid code signature"
HELPER_ID="$(codesign -dvv "$HELPER" 2>&1 | sed -n 's/^Identifier=//p')"
[ "$HELPER_ID" = io.github.dolegadolegowski.IntelFaceScan.Helper ] || die "$HELPER: signing identifier is '$HELPER_ID', expected io.github.dolegadolegowski.IntelFaceScan.Helper"
HELPER_CDHASH="$(codesign -dvvv --arch arm64 "$HELPER" 2>&1 | sed -n 's/^CDHash=//p')"
[[ "$HELPER_CDHASH" =~ ^[0-9a-f]{40}$ ]] || die "cannot read the cdhash of $HELPER"

pl_get() { plutil -extract "$1" raw -expect "$2" "$3" 2>/dev/null; }
# Base: the allowlist the Viewer app ships (matches its daemon). CONF only supplies the installed BuildNumber.
COUNT="$(pl_get AllowedCDHashes array "$BUNDLED")" || die "$BUNDLED: AllowedCDHashes missing"
HASHES=()
for ((i = 0; i < COUNT; i++)); do
    HASHES+=("$(pl_get "AllowedCDHashes.$i" string "$BUNDLED")")
done
DAEMON_CDHASH="$(pl_get DaemonCDHash string "$BUNDLED")" || die "$BUNDLED: DaemonCDHash missing"
PROTOCOL="$(pl_get ProtocolVersion integer "$BUNDLED")" || die "$BUNDLED: ProtocolVersion missing"
OLD_BUILD="$(pl_get BuildNumber integer "$CONF")" || die "$CONF: BuildNumber missing"
BUNDLED_BUILD="$(pl_get BuildNumber integer "$BUNDLED")" || die "$BUNDLED: BuildNumber missing"
[ "$BUNDLED_BUILD" -le "$OLD_BUILD" ] || OLD_BUILD="$BUNDLED_BUILD"

INSTALLED_HASHES=" "
ICOUNT="$(pl_get AllowedCDHashes array "$CONF" || echo 0)"
for ((i = 0; i < ICOUNT; i++)); do INSTALLED_HASHES="$INSTALLED_HASHES$(pl_get "AllowedCDHashes.$i" string "$CONF") "; done
# Done only when the installed list already holds this helper and every hash of the Viewer app on disk.
MISSING=0
for h in "${HASHES[@]}" "$HELPER_CDHASH"; do
    case "$INSTALLED_HASHES" in *" $h "*) ;; *) MISSING=1 ;; esac
done
if [ "$MISSING" = 0 ]; then
    note "this build of the helper ($HELPER_CDHASH) and the Viewer on disk are already allowed"
    exit 0
fi

echo "$ME: helper  $HELPER"
echo "$ME: cdhash  $HELPER_CDHASH"
echo "$ME: this lets that program use the USB camera service of RealSense Viewer as $(stat -f %Su /dev/console)."
if [ "$ASSUME_YES" != 1 ]; then
    read -r -p "Allow it? [y/N] " answer </dev/tty || answer=n
    case "$answer" in y|Y|yes|YES) ;; *) die "cancelled" ;; esac
fi
HASHES+=("$HELPER_CDHASH")
[ "${#HASHES[@]}" -le 256 ] || die "the allowlist would hold more than 256 entries"

# Same rules as the Viewer's package.sh: sorted unique hashes, BuildID = sha256 of them joined by newlines, and a
# BuildNumber that never goes down (the installer refuses a downgrade); +1 keeps a later Viewer DMG installable.
SORTED="$(printf '%s\n' "${HASHES[@]}" | LC_ALL=C sort -u)"
BUILD_ID="$(printf '%s' "$SORTED" | shasum -a 256 | cut -d' ' -f1)"
BUILD_NUMBER=$((OLD_BUILD + 1))

# The installer only accepts a --source whose every parent is root-owned and not world-writable (/private/var/tmp is).
STAGE="$(mktemp -d "$CONF_DIR/.ifs-allow.XXXXXX")"
trap 'rm -rf "$STAGE"' EXIT
chown root:wheel "$STAGE"
chmod 755 "$STAGE"
ALLOW="$STAGE/allowlist.plist"
plutil -create xml1 "$ALLOW"
plutil -insert Version -integer 1 "$ALLOW"
plutil -insert BuildID -string "$BUILD_ID" "$ALLOW"
plutil -insert BuildNumber -integer "$BUILD_NUMBER" "$ALLOW"
plutil -insert ProtocolVersion -integer "$PROTOCOL" "$ALLOW"
plutil -insert AllowedCDHashes -array "$ALLOW"
while IFS= read -r h; do plutil -insert AllowedCDHashes -string "$h" -append "$ALLOW"; done <<<"$SORTED"
plutil -insert DaemonCDHash -string "$DAEMON_CDHASH" "$ALLOW"
chown root:wheel "$ALLOW"
chmod 644 "$ALLOW"

note "adding the helper ($HELPER_CDHASH) to the allowlist of the camera access service"
"$INSTALLER" --source "$DAEMON_DIR" --source "$STAGE"
note "done: IntelFaceScan uses the live camera without an administrator password"
