#!/bin/bash
# Packages build/IntelFaceScan.app (scripts/build.sh) into a disk image:
#   dist/IntelFaceScan-<version>-macOS-arm64.dmg   (the app, a link to /Applications, a short README)
set -euo pipefail
# The camera service installer inside the app refuses to install from folders that group or others can write to;
# the app may be run straight from the disk image, so its folders there must not depend on the builder's umask.
umask 022

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
APP="$ROOT/build/IntelFaceScan.app"
DIST="$ROOT/dist"

[ -d "$APP" ] || { echo "package.sh: build the app first (scripts/build.sh)" >&2; exit 1; }
codesign --verify --strict --deep "$APP"
VERSION="$(/usr/libexec/PlistBuddy -c 'Print CFBundleShortVersionString' "$APP/Contents/Info.plist")"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
STAGE="$WORK/IntelFaceScan"
mkdir -p "$STAGE"
# ditto keeps the signature's extended layout intact
ditto "$APP" "$STAGE/IntelFaceScan.app"
ln -s /Applications "$STAGE/Applications"
cat > "$STAGE/README.txt" <<TXT
IntelFaceScan $VERSION for Apple Silicon Macs (macOS 27.2 or newer)

Drag IntelFaceScan to Applications. The app is signed ad hoc: the first time, open it with right click > Open.

The live camera (Intel RealSense D455 over USB-C)
macOS only lets root take the camera over from its USB video driver. IntelFaceScan therefore uses the RealSense
camera service (realsense-usbd), a small system service that does this for RealSense applications; RealSense Viewer
for macOS uses the same one. The first time the camera is started, IntelFaceScan installs the service, and macOS
asks for an administrator's name and password for that, once. From then on the camera starts without a password
for whoever is logged in at the Mac's screen, and the service hands the camera back to macOS a few seconds after
the last application let go of it. When another application already installed a current version of the service,
nothing is asked at all.

The service is installed from inside the app, and its installer only accepts files that no other user could have
changed: copy IntelFaceScan to /Applications from an administrator's account first.

Without the service
When the service cannot be installed or used, IntelFaceScan offers to use the camera once with administrator
rights instead: it asks for the password for that camera session, runs only its small camera helper as root, and
hands the camera back to macOS when the session ends.

From Terminal
Check the service, and install it if it is missing (run it as yourself, not with sudo):
    /Applications/IntelFaceScan.app/Contents/Resources/realsense-usbd/ensure-camera-access.sh
Remove the service (other RealSense applications on this Mac lose it as well):
    sudo /Applications/IntelFaceScan.app/Contents/Resources/realsense-usbd/uninstall-camera-access.sh

If the app was ever killed while it used the camera with administrator rights and the camera no longer works as a
webcam, re-plug it, or run
    sudo /Applications/IntelFaceScan.app/Contents/Helpers/rs-macos-release
TXT
xattr -cr "$STAGE" 2>/dev/null || true

mkdir -p "$DIST"
DMG="$DIST/IntelFaceScan-$VERSION-macOS-arm64.dmg"
rm -f "$DMG"
# diskutil image replaces "hdiutil create" (deprecated in macOS 27); older systems only have hdiutil.
if diskutil image create from --help >/dev/null 2>&1; then
    out="$(diskutil image create from --format UDZO --volumeName "IntelFaceScan $VERSION" "$STAGE" "$DMG" 2>&1)" \
        || { echo "$out" >&2; exit 1; }
else
    hdiutil create -volname "IntelFaceScan $VERSION" -srcfolder "$STAGE" -ov -format UDZO "$DMG" >/dev/null
fi
echo "created $DMG ($(du -h "$DMG" | cut -f1 | tr -d ' '))"
