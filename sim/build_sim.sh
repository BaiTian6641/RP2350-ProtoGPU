#!/usr/bin/env bash
# Desktop soft-GPU simulator (milestone A5-1) — native build + run.
#
# Compiles the REAL RP2350-ProtoGPU render pipeline (command parser, scene
# state, rasterizer, tile loop, screenspace shaders, memory tier) with native
# g++ against the ProtoGC DESKTOP backend — no Pico SDK, no hardware.  Only
# hardware-bound drivers (QSPI VRAM, flash persistence) are stubbed, in sim/.
#
# Usage (from the repo root or anywhere else):
#   ./sim/build_sim.sh [ppm-output-path]
#
# Default PPM output: sim/out/frame.ppm (relative to the repo root).
# Exit code: 0 = build + run + content self-check green, 1 = failure.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$(cd "$REPO_ROOT/.." && pwd)"
PROTOGL_SRC="$WORKSPACE/ProtoGL/src"
PROTOGC_SRC="$WORKSPACE/ProtoGC/src"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/sim}"
CXX="${CXX:-g++}"
PPM_PATH="${1:-sim/out/frame.ppm}"

for d in "$PROTOGL_SRC" "$PROTOGC_SRC"; do
    if [ ! -d "$d" ]; then
        echo "ERROR: expected ProtoGL/ProtoGC sources at $d"
        exit 1
    fi
done

mkdir -p "$BUILD_DIR"

echo "ProtoGPU desktop sim build"
echo "  compiler: $CXX"

# Pre-existing firmware warning classes (memset on non-trivial types, %u
# format specifiers, set-but-unused vars, unused parameters) are suppressed
# LOCALLY here — the firmware sources themselves are intentionally not patched
# for them.
set -e
"$CXX" -std=gnu++17 -Wall -Wextra -O2 -g \
    -Wno-class-memaccess -Wno-format -Wno-unused-but-set-variable \
    -Wno-unused-parameter \
    -DPGC_BACKEND_DESKTOP -DPROTOGC_OVERRIDE_NEW=0 \
    -I "$REPO_ROOT/src" -I "$PROTOGL_SRC" -I "$PROTOGC_SRC" \
    "$REPO_ROOT/src/command_parser.cpp" \
    "$REPO_ROOT/src/math/pgl_math.cpp" \
    "$REPO_ROOT/src/render/rasterizer.cpp" \
    "$REPO_ROOT/src/render/triangle2d.cpp" \
    "$REPO_ROOT/src/render/quadtree.cpp" \
    "$REPO_ROOT/src/render/rasterizer_2d.cpp" \
    "$REPO_ROOT/src/render/screenspace_effects.cpp" \
    "$REPO_ROOT/src/render/pgl_shader_vm.cpp" \
    "$REPO_ROOT/src/memory/mem_tier.cpp" \
    "$REPO_ROOT/src/display/display_manager.cpp" \
    "$PROTOGC_SRC/pgc_desktop.cpp" \
    "$SCRIPT_DIR/sim_qspi_vram_stub.cpp" \
    "$SCRIPT_DIR/sim_main.cpp" \
    -o "$BUILD_DIR/protogpu_sim"
set +e

echo "build ok, running..."
echo
cd "$REPO_ROOT"
"$BUILD_DIR/protogpu_sim" "$PPM_PATH"
rc=$?
exit $rc
