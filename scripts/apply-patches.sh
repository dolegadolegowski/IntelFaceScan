#!/bin/bash
# Applies the librealsense patches to the submodule (idempotent) and moves between versions of them. The patches, in
# the order they are applied:
#   patches/*.patch                                                  macOS support and fixes (0001 comes from
#                                                                    realsense-viewer-macos, see scripts/build-sdk.sh)
#   third_party/realsense-usbd/librealsense/macos-usb-broker.patch   the client of the RealSense camera service: part
#                                                                    of the vendored kit, never edited here (it is
#                                                                    updated by scripts/sync-camera-service.sh)
#
#   scripts/apply-patches.sh            apply every patch that is not applied yet
#   scripts/apply-patches.sh --reset    restore librealsense's files, then apply every patch again
#   scripts/apply-patches.sh --revert   only restore librealsense's files (scripts/build.sh, before it checks out
#                                       another librealsense commit)
#   scripts/apply-patches.sh --list     print "patch <sha256> <name>" for every patch, in order, and change nothing
#                                       (scripts/build-sdk.sh: the patches are inputs of the SDK)
#
# third_party/librealsense/.ifs-patches records the applied patches (sha256, name) and the files they created. When
# the patches no longer match it (a pull or a sync of the kit changed, added or removed one), the submodule's tracked
# files are restored from its commit, the files the old patches created are removed, and all patches are applied
# again. A checkout patched before the record existed is adopted when every patch is applied or applies, and restored
# the same way otherwise. Edits made by hand inside third_party/librealsense do not survive a restore: changes to
# librealsense belong in patches/ (or, for the camera service client, in the kit's own repository).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/third_party/librealsense"
RECORD="$SRC/.ifs-patches"

die() { echo "apply-patches.sh: $*" >&2; exit 1; }

MODE="${1:-apply}"
case "$MODE" in
    apply|--reset|--revert|--list) ;;
    *) die "usage: scripts/apply-patches.sh [--reset | --revert | --list]" ;;
esac

# The camera service client goes last: it is written against librealsense with patches/ applied.
KIT_PATCH="$ROOT/third_party/realsense-usbd/librealsense/macos-usb-broker.patch"
PATCHES=("$ROOT"/patches/*.patch)
[ -f "${PATCHES[0]}" ] || die "no patches in $ROOT/patches"
[ -f "$KIT_PATCH" ] || die "$KIT_PATCH is missing (the camera service kit in third_party/realsense-usbd is part of" \
                           "the repository; scripts/sync-camera-service.sh copies it from realsense-viewer-macos)"
PATCHES+=("$KIT_PATCH")

# The submodule must be its own git checkout: a restore resets it, which must never reach the superproject.
is_own_checkout() {
    [ -d "$SRC" ] && [ "$(git -C "$SRC" rev-parse --show-toplevel 2>/dev/null)" = "$(cd "$SRC" && pwd -P)" ]
}

# "patch <sha256> <name>" for every patch, in the order they are applied.
patch_lines() {
    local patch
    for patch in "${PATCHES[@]}"; do
        printf 'patch %s %s\n' "$(shasum -a 256 "$patch" | cut -d ' ' -f 1)" "$(basename "$patch")"
    done
}

# The paths a patch touches that librealsense's commit does not have: the files it creates. (-z keeps paths exact;
# a rename lists "added<TAB>deleted<TAB>", then the source and the destination as separate entries.)
created_files() {
    local file
    is_own_checkout || return 0
    git -C "$SRC" apply --numstat -z "$1" | tr '\0' '\n' \
        | awk -F '\t' 'NF >= 3 { if ($3 != "") print $3; next } NF > 0 { print }' \
        | while IFS= read -r file; do
              git -C "$SRC" cat-file -e "HEAD:$file" 2>/dev/null || printf '%s\n' "$file"
          done
}

# Restores the submodule's tracked files from its commit and removes the files patches created: the recorded ones
# (older versions of the patches) and those of the current patches (applying them again recreates them).
restore() {
    is_own_checkout || die "$SRC is not a git checkout of its own; cannot restore it (re-clone with --recursive)"
    local file patch
    {
        [ -f "$RECORD" ] && sed -n 's/^created //p' "$RECORD"
        for patch in "${PATCHES[@]}"; do created_files "$patch"; done
    } | sort -u | while IFS= read -r file; do
        case "$file" in
            "" | /* | .. | ../* | */.. | */../*) continue ;;   # only paths inside the submodule
        esac
        rm -f "$SRC/$file"
    done
    git -C "$SRC" reset --hard --quiet
    rm -f "$RECORD"
    echo "librealsense restored to its commit $(git -C "$SRC" rev-parse --short HEAD)"
}

# Applies every patch that is not applied yet, in order; fails on the first that applies neither way.
apply_all() {
    local patch name
    for patch in "${PATCHES[@]}"; do
        name="$(basename "$patch")"
        if git -C "$SRC" apply --reverse --check "$patch" 2>/dev/null; then
            echo "patch already applied: $name"
        elif git -C "$SRC" apply --whitespace=nowarn "$patch"; then
            echo "patch applied: $name"
        else
            echo "patch does not apply: $name" >&2
            return 1
        fi
    done
}

write_record() {
    local patch
    {
        echo "# Written by scripts/apply-patches.sh: the patches applied here (sha256, name) and the files they created."
        patch_lines
        for patch in "${PATCHES[@]}"; do created_files "$patch"; done | sort -u | sed 's/^/created /'
    } > "$RECORD.tmp"
    mv "$RECORD.tmp" "$RECORD"
}

case "$MODE" in
    --list) patch_lines; exit 0 ;;
    --revert) restore; exit 0 ;;
    --reset) restore ;;
esac

had_record=0
if [ -f "$RECORD" ]; then
    had_record=1
    if [ "$(grep '^patch ' "$RECORD" || true)" != "$(patch_lines)" ]; then
        echo "the patches changed since they were applied: restoring librealsense and applying every patch again"
        restore
    fi
fi
pristine=0
if is_own_checkout && [ -z "$(git -C "$SRC" status --porcelain --untracked-files=no)" ]; then
    pristine=1
fi
if ! apply_all; then
    NOT_APPLICABLE="the patches do not apply to librealsense"
    NOT_APPLICABLE="$NOT_APPLICABLE $(git -C "$SRC" rev-parse --short HEAD 2>/dev/null || true)"
    [ "$pristine" = 0 ] || die "$NOT_APPLICABLE"
    # Only a checkout patched before the record existed (by an older version of the patches) is restored here; a
    # recorded, current checkout that conflicts has been edited by hand.
    if [ "$had_record" = 1 ]; then
        die "third_party/librealsense has local changes that conflict with patches/ (changes to librealsense belong" \
            "in patches/); scripts/apply-patches.sh --reset restores it and applies every patch again"
    fi
    echo "librealsense was patched by an older version of the patches: restoring it and applying every patch again"
    restore
    apply_all || die "$NOT_APPLICABLE"
fi
write_record
