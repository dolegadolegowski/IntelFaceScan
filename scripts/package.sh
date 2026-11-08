#!/bin/bash
# Packages build/IntelFaceScan.app (scripts/build.sh) into a disk image:
#   dist/IntelFaceScan-<version>-macOS-arm64.dmg   (the app, a link to /Applications, a short README)
set -euo pipefail

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

The live camera (Intel RealSense D455 over USB-C) needs administrator rights: macOS only lets root take the camera
over from its USB video driver. IntelFaceScan asks for the password once per camera session and runs only its small
camera helper with these rights; when the session ends the camera is handed back to macOS.

If the app was ever killed while the camera was in use and the camera no longer works as a webcam, re-plug it, or run
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
