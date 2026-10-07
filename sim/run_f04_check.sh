#!/usr/bin/env bash
# Historical F04 gate now proves full redraw and consumer-visible frame changes.
# Uses real parser/FrameRenderer and the native two-worker scheduler; no stubs.
# Usage: ./sim/run_f04_check.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/sim}"
CXX="${CXX:-g++}"
mkdir -p "$BUILD_DIR"
BIN="$BUILD_DIR/test_f04_skip"
echo "Full-redraw geometry/material/resource-only check ($CXX)"
"$CXX" -std=gnu++17 -Wall -Wextra -O1 -g -pthread \
    -Wno-class-memaccess -Wno-format -Wno-unused-but-set-variable -Wno-unused-parameter \
    -DPGC_BACKEND_DESKTOP -DPROTOGC_OVERRIDE_NEW=0 -DPROTOGPU_NATIVE_SCHEDULER \
    -I "$REPO_ROOT/src" -I "$REPO_ROOT/ProtoGL/src" -I "$REPO_ROOT/ProtoGC/src" \
    "$REPO_ROOT/src/command_parser.cpp" \
    "$REPO_ROOT/src/phase_scratch.cpp" \
    "$REPO_ROOT/src/math/pgl_math.cpp" \
    "$REPO_ROOT/src/render/frame_renderer.cpp" \
    "$REPO_ROOT/src/render/rasterizer.cpp" \
    "$REPO_ROOT/src/render/triangle2d.cpp" \
    "$REPO_ROOT/src/render/rasterizer_2d.cpp" \
    "$REPO_ROOT/src/render/screenspace_effects.cpp" \
    "$REPO_ROOT/src/render/pgl_shader_vm.cpp" \
    "$REPO_ROOT/src/scheduler/pgl_tile_scheduler.cpp" \
    "$REPO_ROOT/src/scheduler/pgl_scheduler_native.cpp" \
    "$REPO_ROOT/src/memory/mem_assets.cpp" "$REPO_ROOT/src/memory/scene_assets.cpp" \
    "$REPO_ROOT/ProtoGC/src/pgc_desktop.cpp" \
    "$SCRIPT_DIR/test_f04_skip.cpp" -o "$BIN"
exec "$BIN"
