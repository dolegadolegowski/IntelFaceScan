#!/bin/bash
# Tests the SDK bookkeeping of scripts/apply-patches.sh and scripts/build-sdk.sh against a tiny stand-in for
# librealsense (a git repository with a one-file shared library), in a temporary folder: no network, a few seconds.
#
#   scripts/test-build-scripts.sh
#
# Covers moving between versions of the patches (changed, removed, created files, checkouts patched before the
# record existed, hand edits) and the SDK input stamp (no rebuild for touched files; rebuilds for another librealsense
# commit, patch, build type or deployment target; a build folder of another checkout is refused).
set -euo pipefail

SCRIPTS="$(cd "$(dirname "$0")" && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
export GIT_CONFIG_NOSYSTEM=1 GIT_CONFIG_GLOBAL=/dev/null
export GIT_AUTHOR_NAME=test GIT_AUTHOR_EMAIL=test@example.invalid
export GIT_COMMITTER_NAME=test GIT_COMMITTER_EMAIL=test@example.invalid
unset IFS_SDK_DIR IFS_SDK_REBUILD BUILD_TYPE MACOSX_DEPLOYMENT_TARGET

PASSED=0
fail() { echo "FAIL: $*" >&2; [ -f "$WORK/out" ] && sed 's/^/    | /' "$WORK/out" >&2; exit 1; }
ok() { PASSED=$((PASSED + 1)); echo "ok: $*"; }
# run <expected status> <command...>: output in $WORK/out
run() {
    local expected="$1" status=0
    shift
    "$@" > "$WORK/out" 2>&1 || status=$?
    [ "$status" = "$expected" ] || fail "'$*' exited with $status, expected $expected"
}
expect_output() { grep -qF -- "$1" "$WORK/out" || fail "expected output: $1"; }
reject_output() { ! grep -qF -- "$1" "$WORK/out" || fail "unexpected output: $1"; }
# config_has <text...>: every text is in librealsense's config.h
config_has() {
    local text
    for text in "$@"; do grep -qF -- "$text" "$SRC/config.h" || fail "config.h lacks '$text': $(cat "$SRC/config.h")"; done
}
# config.h of the stand-in: the settings are far enough apart that patches of different lines do not share context.
config_h() {
    printf '#define VERSION %s\n\n\n\n\n\n\n\n#define USB_TIMEOUT_MS 0\n\n\n\n\n\n\n\n#define LOCK_HANDLES 0\n' "$1"
}

# --- A checkout: scripts/, patches/ and third_party/librealsense (the stand-in, one commit) --------------------------
make_checkout() {
    local root="$1" src="$1/third_party/librealsense"
    mkdir -p "$root/scripts" "$root/patches" "$src"
    cp "$SCRIPTS/apply-patches.sh" "$SCRIPTS/build-sdk.sh" "$root/scripts/"
    cat > "$src/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.20)
project(fake_librealsense C)
add_library(realsense2 SHARED rs.c)
install(TARGETS realsense2 LIBRARY DESTINATION lib)
CMAKE
    printf '%s\n' '#include "config.h"' 'int rs_api_version(void) { return VERSION; }' > "$src/rs.c"
    config_h 1 > "$src/config.h"
    git -C "$src" init -q
    git -C "$src" add -A
    git -C "$src" commit -q -m "librealsense"
}

# write_patch <checkout> <name> <file> <sed expression>: a patch of one file, made from the pristine commit
write_patch() {
    local root="$1" name="$2" file="$3" expr="$4" tmp="$WORK/patch-src"
    rm -rf "$tmp"
    git clone -q "$root/third_party/librealsense" "$tmp"
    sed -i '' "$expr" "$tmp/$file"
    git -C "$tmp" diff > "$root/patches/$name"
}
# new_file_patch <checkout> <name> <file> <content>: a patch that creates a file
new_file_patch() {
    local root="$1" name="$2" file="$3" content="$4" tmp="$WORK/patch-src"
    rm -rf "$tmp"
    git clone -q "$root/third_party/librealsense" "$tmp"
    printf '%s\n' "$content" > "$tmp/$file"
    git -C "$tmp" add -N "$file"
    git -C "$tmp" diff > "$root/patches/$name"
}

R="$WORK/repo"
SRC="$R/third_party/librealsense"
make_checkout "$R"
write_patch "$R" 0001-usb-timeouts.patch config.h 's/USB_TIMEOUT_MS 0/USB_TIMEOUT_MS 5000/'
new_file_patch "$R" 0002-recording.patch recording.c 'int rs_recording(void) { return 1; }'

# --- apply-patches.sh ---------------------------------------------------------------------------------------------
run 0 "$R/scripts/apply-patches.sh"
expect_output "patch applied: 0001-usb-timeouts.patch"
config_has 'USB_TIMEOUT_MS 5000'
[ -f "$SRC/recording.c" ] || fail "recording.c not created"
grep -q '^created recording.c$' "$SRC/.ifs-patches" || fail "the record does not list the created file"
ok "applies the patches and records them"

run 0 "$R/scripts/apply-patches.sh"
expect_output "patch already applied: 0001-usb-timeouts.patch"
expect_output "patch already applied: 0002-recording.patch"
ok "is idempotent"

# A pull changes a patch: the new version neither applies nor reverts on the old one.
write_patch "$R" 0001-usb-timeouts.patch config.h 's/USB_TIMEOUT_MS 0/USB_TIMEOUT_MS 2500/'
run 0 "$R/scripts/apply-patches.sh"
expect_output "patches/ changed"
config_has 'USB_TIMEOUT_MS 2500'
[ -f "$SRC/recording.c" ] || fail "the unchanged patch's file is missing after the restore"
ok "moves to a changed version of a patch"

# A patch is added (it also changes a file another patch changed) and one is removed by the repository.
write_patch "$R" 0003-lock-handles.patch config.h 's/LOCK_HANDLES 0/LOCK_HANDLES 1/'
rm "$R/patches/0002-recording.patch"
run 0 "$R/scripts/apply-patches.sh"
[ ! -e "$SRC/recording.c" ] || fail "the file created by the removed patch is still there"
config_has 'LOCK_HANDLES 1' 'USB_TIMEOUT_MS 2500'
ok "follows added and removed patches (files of removed patches are deleted)"

# A checkout patched by the old apply-patches.sh (no record) with an older version of a patch.
new_file_patch "$R" 0002-recording.patch recording.c 'int rs_recording(void) { return 1; }'
run 0 "$R/scripts/apply-patches.sh" --revert
[ -z "$(git -C "$SRC" status --porcelain)" ] || fail "--revert leaves changes: $(git -C "$SRC" status --porcelain)"
ok "--revert restores the pristine commit"
cp "$R/patches/0001-usb-timeouts.patch" "$WORK/0001-new.patch"
write_patch "$R" 0001-usb-timeouts.patch config.h 's/USB_TIMEOUT_MS 0/USB_TIMEOUT_MS 5000/'
for p in "$R"/patches/*.patch; do git -C "$SRC" apply "$p"; done
cp "$WORK/0001-new.patch" "$R/patches/0001-usb-timeouts.patch"
run 0 "$R/scripts/apply-patches.sh"
expect_output "patched by an older version"
config_has 'USB_TIMEOUT_MS 2500' 'LOCK_HANDLES 1'
[ -f "$SRC/recording.c" ] || fail "recording.c not created"
ok "moves a checkout patched before the record existed"

# Without a record, a current checkout is adopted as it is.
rm "$SRC/.ifs-patches"
run 0 "$R/scripts/apply-patches.sh"
reject_output "restored"
[ -f "$SRC/.ifs-patches" ] || fail "no record written"
ok "adopts a current checkout without a record"

# A hand edit that conflicts with a patch is not silently discarded.
sed -i '' 's/USB_TIMEOUT_MS 2500/USB_TIMEOUT_MS 7/' "$SRC/config.h"
run 1 "$R/scripts/apply-patches.sh"
expect_output "local changes that conflict"
config_has 'USB_TIMEOUT_MS 7'
run 0 "$R/scripts/apply-patches.sh" --reset
config_has 'USB_TIMEOUT_MS 2500'
ok "refuses to overwrite hand edits; --reset does"

# --- build-sdk.sh: the input stamp --------------------------------------------------------------------------------
run 0 "$R/scripts/build-sdk.sh"
expect_output "librealsense installed"
[ -f "$R/build/sdk/install/lib/librealsense2.dylib" ] && [ -f "$R/build/sdk/install/.ifs-sdk-inputs" ] \
    || fail "no library or stamp installed"
ok "builds and stamps the SDK"

run 0 "$R/scripts/build-sdk.sh"
expect_output "librealsense already built"
touch "$R/patches" "$R"/patches/*.patch
sleep 1
touch "$R/patches"
run 0 "$R/scripts/build-sdk.sh"
expect_output "librealsense already built"
ok "no rebuild for unchanged inputs with newer file times"

run 0 env BUILD_TYPE=Debug "$R/scripts/build-sdk.sh"
expect_output "build_type Debug"
expect_output "librealsense installed"
run 0 env BUILD_TYPE=Debug "$R/scripts/build-sdk.sh"
expect_output "librealsense already built"
ok "rebuilds for another build type"

run 0 env BUILD_TYPE=Debug MACOSX_DEPLOYMENT_TARGET=27.0 "$R/scripts/build-sdk.sh"
expect_output "deployment_target 27.0"
expect_output "librealsense installed"
ok "rebuilds for another deployment target"

run 0 "$R/scripts/apply-patches.sh" --revert
config_h 2 > "$SRC/config.h"
git -C "$SRC" commit -q -am "librealsense bump"
run 0 "$R/scripts/build-sdk.sh"
expect_output "librealsense $(git -C "$SRC" rev-parse HEAD)"
expect_output "librealsense installed"
config_has 'VERSION 2' 'USB_TIMEOUT_MS 2500'
ok "rebuilds for another librealsense commit"

write_patch "$R" 0003-lock-handles.patch config.h 's/LOCK_HANDLES 0/LOCK_HANDLES 2/'
run 0 "$R/scripts/build-sdk.sh"
expect_output "patches/ changed"
expect_output "librealsense installed"
config_has 'LOCK_HANDLES 2'
ok "rebuilds for a changed patch"

# Another checkout (a worktree) sharing this SDK folder: reused while the inputs match, refused otherwise.
R2="$WORK/worktree"
mkdir -p "$R2/scripts" "$R2/patches" "$R2/third_party"
cp "$R"/scripts/*.sh "$R2/scripts/"
cp "$R"/patches/*.patch "$R2/patches/"
git clone -q "$SRC" "$R2/third_party/librealsense"
run 0 env IFS_SDK_DIR="$R/build/sdk" "$R2/scripts/build-sdk.sh"
expect_output "librealsense already built"
write_patch "$R2" 0003-lock-handles.patch config.h 's/LOCK_HANDLES 0/LOCK_HANDLES 3/'
run 1 env IFS_SDK_DIR="$R/build/sdk" "$R2/scripts/build-sdk.sh"
expect_output "was configured for"
reject_output "does not match the source"
ok "a shared SDK folder is reused with the same inputs and refused with others"

echo "all $PASSED build script tests passed"
