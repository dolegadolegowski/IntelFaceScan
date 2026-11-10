#!/bin/bash
# Assembles build/IntelFaceScan.app from the Swift, helper, librealsense and camera service builds (the last step of
# scripts/build.sh).
#
#   SWIFT_CONFIG=release scripts/bundle-app.sh     (debug: SWIFT_CONFIG=debug)
#   IFS_SDK_DIR=/path scripts/bundle-app.sh        librealsense build location, as for scripts/build-sdk.sh
#
# IntelFaceScan.app/Contents/
#   MacOS/IntelFaceScan                 the app (SwiftPM product)
#   MacOS/ifscan                        command line tool
#   Helpers/IntelFaceScanHelper         camera helper (the user with the camera service, root on the administrator
#                                       path), rpath @executable_path/../Frameworks
#   Helpers/rs-macos-release            recovery: hands a camera left captured by a crashed root helper back to macOS
#   Helpers/realsense-usbd              the RealSense camera service daemon (build/usbd, from the kit in
#                                       third_party/realsense-usbd): installed once as a root LaunchDaemon, and run
#                                       from here as the user to ask the installed service (--probe)
#   Frameworks/librealsense2.X.Y.dylib  install name @rpath/librealsense2.X.Y.dylib
#   Resources/realsense-usbd/           the kit's install-camera-access.sh, uninstall-camera-access.sh and
#                                       ensure-camera-access.sh, and service.plist, the manifest of the daemon above
#   Resources/AppIcon.icns              rendered by tools/release/make-app-icon.swift
#   Resources/NOTICE.txt, Licenses/     our license and NOTICE, and the licenses of the bundled third-party code
# Every Mach-O is signed ad hoc, inside out (Apple Silicon runs nothing unsigned). The order for the camera service is
# the kit's: copy the files, sign the daemon (hardened runtime, the service's own identifier), write its manifest
# with the kit's make-service-manifest.sh, then sign the bundle, whose signature seals the manifest. Release bundles
# ship the two Swift executables without the linker's debug map (strip -S); build/dSYMs/ keeps their debug
# information (not shipped).
set -euo pipefail
# The camera service installer only installs from folders that group and others cannot write to (the kit's trust
# rule), so the bundle must not depend on the umask of whoever builds it.
umask 022

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="${SWIFT_CONFIG:-release}"
SDK_PREFIX="${IFS_SDK_DIR:-$ROOT/build/sdk}/install"
HELPER_BUILD="$ROOT/build/helper"
KIT="$ROOT/third_party/realsense-usbd"
USBD="$ROOT/build/usbd/realsense-usbd"
USBD_ID="io.github.dolegadolegowski.realsense-usbd"
USBD_SCRIPTS=(install-camera-access.sh uninstall-camera-access.sh ensure-camera-access.sh)
TOOLS="$ROOT/build/tools"
APP="$ROOT/build/IntelFaceScan.app"
DSYMS="$ROOT/build/dSYMs"
DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-27.2}"
BUNDLE_ID="io.github.dolegadolegowski.IntelFaceScan"
VERSION="0.1.0"
PREFIX_MAP="-ffile-prefix-map=$ROOT=."

die() { echo "bundle-app.sh: $*" >&2; exit 1; }

cd "$ROOT"
BIN="$(swift build -c "$CONFIG" --show-bin-path)"
HELPER="$HELPER_BUILD/IntelFaceScanHelper"
for f in "$BIN/IntelFaceScan" "$BIN/ifscan" "$HELPER" "$USBD"; do
    [ -x "$f" ] || die "missing $f (run scripts/build.sh)"
done
for f in make-service-manifest.sh "${USBD_SCRIPTS[@]}"; do
    [ -f "$KIT/$f" ] || die "missing $KIT/$f (the camera service kit; scripts/sync-camera-service.sh updates it)"
done
# The helper names the exact librealsense it was linked against.
LIBNAME="$(otool -L "$HELPER" | awk '$1 ~ /^@rpath\/librealsense2/ { sub("@rpath/", "", $1); print $1; exit }')"
[ -n "$LIBNAME" ] || die "$HELPER does not link @rpath/librealsense2"
[ -f "$SDK_PREFIX/lib/$LIBNAME" ] || die "missing $SDK_PREFIX/lib/$LIBNAME (run scripts/build-sdk.sh)"

mkdir -p "$TOOLS"
# Small tools, rebuilt when their source changed.
if [ ! -x "$TOOLS/rs-macos-release" ] || [ "$ROOT/tools/release/rs-macos-release.c" -nt "$TOOLS/rs-macos-release" ]; then
    clang -O2 -Wall -arch arm64 -mmacosx-version-min="$DEPLOYMENT_TARGET" "$PREFIX_MAP" \
        "$ROOT/tools/release/rs-macos-release.c" -framework IOKit -framework CoreFoundation \
        -o "$TOOLS/rs-macos-release"
fi
if [ ! -x "$TOOLS/make-app-icon" ] || [ "$ROOT/tools/release/make-app-icon.swift" -nt "$TOOLS/make-app-icon" ]; then
    swiftc -O -target "arm64-apple-macos$DEPLOYMENT_TARGET" "$ROOT/tools/release/make-app-icon.swift" \
        -o "$TOOLS/make-app-icon"
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
"$TOOLS/make-app-icon" "$WORK/AppIcon.iconset"
iconutil -c icns "$WORK/AppIcon.iconset" -o "$WORK/AppIcon.icns"

# Removes every LC_RPATH and adds the one that points into the bundle.
relink() {
    local file="$1" rpath="$2"
    otool -l "$file" | awk '/cmd LC_RPATH/ { getline; getline; print $2 }' | while read -r old; do
        install_name_tool -delete_rpath "$old" "$file" 2>/dev/null || true
    done
    install_name_tool -add_rpath "$rpath" "$file"
}

rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Helpers" "$APP/Contents/Frameworks" "$APP/Contents/Resources"

cp "$BIN/IntelFaceScan" "$APP/Contents/MacOS/IntelFaceScan"
cp "$BIN/ifscan" "$APP/Contents/MacOS/ifscan"
if [ "$CONFIG" = "release" ]; then
    # The linker's debug map lists the absolute paths of every object file (and with them the builder's user name):
    # strip it from the shipped copies; the symbols stay, so crash reports still name functions. dSYMs made from the
    # unstripped builds (same UUID) resolve release crash logs to source lines, e.g.
    #   atos -o build/dSYMs/IntelFaceScan.dSYM/Contents/Resources/DWARF/IntelFaceScan -l <load address> <address>
    rm -rf "$DSYMS"
    mkdir -p "$DSYMS"
    for exe in IntelFaceScan ifscan; do
        strip -S "$APP/Contents/MacOS/$exe"
        dsymutil "$BIN/$exe" -o "$DSYMS/$exe.dSYM" || echo "bundle-app.sh: warning: no dSYM for $exe" >&2
    done
fi
# SwiftPM resource bundles, should a target ever declare resources.
for bundle in "$BIN"/*.bundle; do
    [ -d "$bundle" ] && cp -R "$bundle" "$APP/Contents/Resources/"
done

cp "$HELPER" "$APP/Contents/Helpers/IntelFaceScanHelper"
relink "$APP/Contents/Helpers/IntelFaceScanHelper" "@executable_path/../Frameworks"
cp "$TOOLS/rs-macos-release" "$APP/Contents/Helpers/rs-macos-release"

# RealSense camera service, in the layout the kit's client and scripts look for: the daemon in Contents/Helpers, the
# scripts in Contents/Resources/realsense-usbd. The manifest joins them once the daemon is signed (below).
USBD_BUNDLED="$APP/Contents/Helpers/realsense-usbd"
USBD_RESOURCES="$APP/Contents/Resources/realsense-usbd"
mkdir -p "$USBD_RESOURCES"
cp "$USBD" "$USBD_BUNDLED"
chmod 755 "$USBD_BUNDLED"
for script in "${USBD_SCRIPTS[@]}"; do
    cp "$KIT/$script" "$USBD_RESOURCES/$script"
    chmod 755 "$USBD_RESOURCES/$script"
done

cp -L "$SDK_PREFIX/lib/$LIBNAME" "$APP/Contents/Frameworks/$LIBNAME"
chmod 644 "$APP/Contents/Frameworks/$LIBNAME"
install_name_tool -id "@rpath/$LIBNAME" "$APP/Contents/Frameworks/$LIBNAME"
otool -l "$APP/Contents/Frameworks/$LIBNAME" | awk '/cmd LC_RPATH/ { getline; getline; print $2 }' | while read -r old; do
    install_name_tool -delete_rpath "$old" "$APP/Contents/Frameworks/$LIBNAME" 2>/dev/null || true
done

cp "$WORK/AppIcon.icns" "$APP/Contents/Resources/AppIcon.icns"

# Licenses of everything in the bundle. librealsense's NOTICE.md covers the code its build fetches and links into the
# dylib (libusb under LGPL-2.1, nlohmann/json, Fast-CDR, rosbag2, sqlite, LZ4, yaml-cpp).
LICENSES="$APP/Contents/Resources/Licenses"
mkdir -p "$LICENSES"
cp "$ROOT/NOTICE" "$APP/Contents/Resources/NOTICE.txt"
license() {
    [ -f "$ROOT/$2" ] || { echo "error: $2 is missing (run git submodule update --init)" >&2; exit 1; }
    cp "$ROOT/$2" "$LICENSES/$1"
}
license IntelFaceScan-LICENSE.txt   LICENSE
license librealsense-LICENSE.txt    third_party/librealsense/LICENSE
license librealsense-NOTICE.md      third_party/librealsense/NOTICE.md
license Eigen-COPYING.MPL2.txt      third_party/eigen/COPYING.MPL2
license nanoflann-COPYING.txt       third_party/nanoflann/COPYING
license meshoptimizer-LICENSE.md    third_party/meshoptimizer/LICENSE.md
license xatlas-LICENSE.txt          third_party/xatlas/LICENSE
license PoissonRecon-LICENSE.txt    third_party/PoissonRecon/LICENSE
BUILD_NUMBER="$(git -C "$ROOT" rev-list --count HEAD 2>/dev/null || echo 1)"
cat > "$APP/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleName</key>                   <string>IntelFaceScan</string>
    <key>CFBundleDisplayName</key>            <string>IntelFaceScan</string>
    <key>CFBundleIdentifier</key>             <string>$BUNDLE_ID</string>
    <key>CFBundleExecutable</key>             <string>IntelFaceScan</string>
    <key>CFBundleIconFile</key>               <string>AppIcon</string>
    <key>CFBundlePackageType</key>            <string>APPL</string>
    <key>CFBundleShortVersionString</key>     <string>$VERSION</string>
    <key>CFBundleVersion</key>                <string>$BUILD_NUMBER</string>
    <key>CFBundleInfoDictionaryVersion</key>  <string>6.0</string>
    <key>CFBundleDevelopmentRegion</key>      <string>en</string>
    <key>CFBundleLocalizations</key>          <array><string>en</string><string>pl</string></array>
    <key>LSMinimumSystemVersion</key>         <string>$DEPLOYMENT_TARGET</string>
    <key>LSArchitecturePriority</key>         <array><string>arm64</string></array>
    <key>LSApplicationCategoryType</key>      <string>public.app-category.graphics-design</string>
    <key>NSHighResolutionCapable</key>        <true/>
    <key>NSSupportsAutomaticGraphicsSwitching</key><true/>
    <key>NSHumanReadableCopyright</key>       <string>Apache License 2.0. librealsense: Copyright RealSense, Inc. (Apache 2.0).</string>
</dict>
</plist>
PLIST
printf 'APPL????' > "$APP/Contents/PkgInfo"
plutil -lint "$APP/Contents/Info.plist" >/dev/null

# Extended attributes (quarantine, Finder info) break signatures.
xattr -cr "$APP" 2>/dev/null || true
sign() {
    local out
    out="$(codesign --force --sign - --timestamp=none "$@" 2>&1)" || { echo "$out" >&2; return 1; }
}
sign "$APP/Contents/Frameworks/$LIBNAME"
sign --identifier "$BUNDLE_ID.Helper" "$APP/Contents/Helpers/IntelFaceScanHelper"
sign --identifier "$BUNDLE_ID.rs-macos-release" "$APP/Contents/Helpers/rs-macos-release"
sign --identifier "$BUNDLE_ID.ifscan" "$APP/Contents/MacOS/ifscan"
for bundle in "$APP/Contents/Resources"/*.bundle; do
    [ -d "$bundle" ] && sign "$bundle"
done
# The camera service daemon: hardened runtime and the service's own identifier, the same in every application that
# carries it. Its manifest records the cdhash of this signature (the installer installs nothing else), so the daemon
# is not signed again after this line; the bundle signature below seals the manifest with the other resources.
sign --options runtime --identifier "$USBD_ID" "$USBD_BUNDLED"
"$KIT/make-service-manifest.sh" "$USBD_BUNDLED" "$USBD_RESOURCES/service.plist"
chmod 644 "$USBD_RESOURCES/service.plist"
sign "$APP"
codesign --verify --strict --deep "$APP"

# The bundled helper must start with the bundled library only.
"$APP/Contents/Helpers/IntelFaceScanHelper" --version >/dev/null \
    || die "the bundled helper does not start (librealsense not found in Contents/Frameworks?)"

# The camera service as the app and the installer will find it: every file is there, the manifest names the bundled
# daemon as it is signed now, the daemon runs and is the version the manifest says, and nothing in the bundle is
# writable by group or others (the installer would refuse to install from it).
for f in "$USBD_BUNDLED" "${USBD_SCRIPTS[@]/#/$USBD_RESOURCES/}"; do
    [ -f "$f" ] && [ -x "$f" ] || die "the bundle lacks $f, or it is not executable"
done
[ -f "$USBD_RESOURCES/service.plist" ] || die "the bundle lacks $USBD_RESOURCES/service.plist"
USBD_SIGNATURE="$(codesign -dvvv --arch arm64 "$USBD_BUNDLED" 2>&1)" || die "cannot read the signature of $USBD_BUNDLED"
USBD_CDHASH="$(sed -n 's/^CDHash=//p' <<<"$USBD_SIGNATURE")"
[[ "$USBD_CDHASH" =~ ^[0-9a-f]{40}$ ]] || die "no arm64 cdhash in the signature of $USBD_BUNDLED"
grep -q "^Identifier=$USBD_ID\$" <<<"$USBD_SIGNATURE" || die "$USBD_BUNDLED is not signed with the identifier $USBD_ID"
grep -q '^CodeDirectory .*flags=0x[0-9a-f]*(.*runtime.*)' <<<"$USBD_SIGNATURE" \
    || die "$USBD_BUNDLED is not signed with the hardened runtime"
MANIFEST_CDHASH="$(plutil -extract DaemonCDHash raw "$USBD_RESOURCES/service.plist")"
[ "$USBD_CDHASH" = "$MANIFEST_CDHASH" ] \
    || die "service.plist names the daemon $MANIFEST_CDHASH, the bundled realsense-usbd is $USBD_CDHASH"
USBD_VERSION="$("$USBD_BUNDLED" --version)" || die "the bundled realsense-usbd does not run (--version)"
[[ "$USBD_VERSION" =~ ^realsense-usbd\ ([0-9]+)\ protocol\ ([0-9]+)-([0-9]+)$ ]] \
    || die "the bundled realsense-usbd --version printed: $USBD_VERSION"
[ "${BASH_REMATCH[1]}" = "$(plutil -extract ServiceVersion raw "$USBD_RESOURCES/service.plist")" ] \
    || die "service.plist and realsense-usbd --version ($USBD_VERSION) name different service versions"
WRITABLE="$(find "$APP" ! -type l \( -perm -020 -o -perm -002 \))"
[ -z "$WRITABLE" ] || die "writable by group or others in the bundle: $WRITABLE"

echo "created $APP ($(du -sh "$APP" | cut -f1 | tr -d ' '), $CONFIG, camera service: $USBD_VERSION)"
