#!/usr/bin/env bash
# SceneState scene-heap migration — native functional check.
#
# Compiles check_scene_state.cpp against the firmware's scene_state.h with
# the ProtoGC DESKTOP backend (native g++, no Pico SDK, no hardware) and runs
# it. This is the desktop gate for the bump-pool → HeapAllocator migration.
#
# Usage (from the repo root or anywhere else):
#   ./tests/syntax_check/run_scene_check.sh
#
# Exit code: 0 = build + run green, 1 = build or test failure.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORKSPACE="$(cd "$REPO_ROOT/.." && pwd)"
PROTOGL_SRC="$WORKSPACE/ProtoGL/src"
PROTOGC_SRC="$WORKSPACE/ProtoGC/src"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/scene-check}"
CXX="${CXX:-g++}"

for d in "$PROTOGL_SRC" "$PROTOGC_SRC"; do
    if [ ! -d "$d" ]; then
        echo "ERROR: expected ProtoGL/ProtoGC sources at $d"
        exit 1
    fi
done

mkdir -p "$BUILD_DIR"

echo "SceneState native check"
echo "  compiler: $CXX"

set -e
"$CXX" -std=gnu++17 -Wall -Wextra -O1 -g \
    -DPGC_BACKEND_DESKTOP -DPROTOGC_OVERRIDE_NEW=0 \
    -I "$REPO_ROOT/src" -I "$PROTOGL_SRC" -I "$PROTOGC_SRC" \
    "$PROTOGC_SRC/pgc_desktop.cpp" \
    "$SCRIPT_DIR/check_scene_state.cpp" \
    -o "$BUILD_DIR/check_scene_state"
set +e

echo "build ok, running..."
echo
"$BUILD_DIR/check_scene_state"
rc=$?
exit $rc
