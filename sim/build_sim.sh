#!/usr/bin/env bash
# Native protocol9 simulator: live parser/frame engine and two-worker scheduler.
# No display manager, physical-driver stubs, legacy memory tiers or quadtree.
# Usage: ./sim/build_sim.sh [output.ppm] | ./sim/build_sim.sh --build-only
# Historical golden images are diagnostic references; never regenerated here.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/sim}"
CXX="${CXX:-g++}"
mkdir -p "$BUILD_DIR"
echo "ProtoGPU native protocol9 sim build ($CXX, two-worker scheduler)"
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
    "$SCRIPT_DIR/sim_main.cpp" -o "$BUILD_DIR/protogpu_sim"
if [[ "${1:-}" == --build-only ]]; then
    exit 0
fi
cd "$REPO_ROOT"
exec "$BUILD_DIR/protogpu_sim" "${1:-sim/out/frame.ppm}"
