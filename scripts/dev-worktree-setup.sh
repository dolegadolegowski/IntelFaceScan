#!/bin/bash
# Prepares a linked git worktree for development: initializes the third_party submodules by cloning them from the
# main checkout (no network; falls back to the network when the main checkout lacks the pinned commit), and prints
# how to build the helper against the main checkout's librealsense build.
#
#   scripts/dev-worktree-setup.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MAIN="$(cd "$(git -C "$ROOT" rev-parse --git-common-dir)/.." && pwd)"
cd "$ROOT"

for path in $(git config -f .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}'); do
    if [ -f "$ROOT/$path/.git" ] || [ -d "$ROOT/$path/.git" ]; then
        continue
    fi
    # submodule.<name>.url: the names in .gitmodules equal the paths. "shallow = true" makes git warn that --depth
    # is ignored for local clones; that warning is noise here.
    if [ "$MAIN" != "$ROOT" ] && [ -e "$MAIN/$path/.git" ] \
       && git -c protocol.file.allow=always -c "submodule.$path.url=$MAIN/$path" submodule update --init -- "$path" \
              >/dev/null 2>&1; then
        :
    else
        git submodule update --init --depth 1 -- "$path" >/dev/null
    fi
    echo "submodule ready: $path"
done

if [ "$MAIN" != "$ROOT" ] && [ -f "$MAIN/build/sdk/install/lib/librealsense2.dylib" ]; then
    SDK="$MAIN/build/sdk/install"
    echo "librealsense SDK: $SDK"
    echo "  helper: cmake -S helper -B build/helper -DIFS_SDK_DIR=$SDK -DIFS_HELPER_TESTS=ON && cmake --build build/helper -j"
    echo "  (scripts/build.sh builds its own SDK in this worktree's build/sdk; IFS_SDK_DIR=$MAIN/build/sdk reuses the main"
    echo "  checkout's while it was built from the same librealsense commit, patches and build-sdk.sh, and stops otherwise)"
fi
