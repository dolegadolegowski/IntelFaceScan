#!/bin/bash
# Prepares a linked git worktree for development: initializes the third_party submodules by cloning them from the
# main checkout (no network), and points the helper build at the main checkout's librealsense build.
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
    if [ "$MAIN" != "$ROOT" ] && [ -e "$MAIN/$path/.git" ]; then
        git -c protocol.file.allow=always -c "submodule.$path.url=$MAIN/$path" submodule update --init -- "$path" >/dev/null
    else
        git submodule update --init --depth 1 -- "$path" >/dev/null
    fi
    echo "submodule ready: $path"
done

if [ "$MAIN" != "$ROOT" ] && [ -f "$MAIN/build/sdk/install/lib/librealsense2.dylib" ]; then
    echo "librealsense SDK: $MAIN/build/sdk/install (cmake -DIFS_SDK_DIR=$MAIN/build/sdk/install)"
fi
