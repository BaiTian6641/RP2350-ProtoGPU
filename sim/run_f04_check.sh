#!/usr/bin/env bash
# F-04 functional gate — frame-signature version-counter semantics.
#
# Compiles and RUNS sim/test_f04_signature.cpp against the real firmware
# pipeline (same TU set as the desktop sim): drives multi-frame sequences
# through PglEncoder → CommandParser::Parse → Rasterizer::PrepareFrame and
# asserts skip / no-skip behavior plus exact meshVersion[] bump counts.
#
# The golden frames (run_golden.sh) prove pixel-identity; THIS gate proves
# the skipped-frame optimization keeps its exact invalidation semantics.
#
# Usage (from the repo root or anywhere else):
#   ./sim/run_f04_check.sh
#
# Exit code: 0 = all checks passed, 1 otherwise.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$(cd "$REPO_ROOT/.." && pwd)"
PROTOGL_SRC="$WORKSPACE/ProtoGL/src"
PROTOGC_SRC="$WORKSPACE/ProtoGC/src"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/sim}"
CXX="${CXX:-g++}"
BIN="$BUILD_DIR/test_f04_signature"

for d in "$PROTOGL_SRC" "$PROTOGC_SRC"; do
    if [ ! -d "$d" ]; then
        echo "ERROR: expected ProtoGL/ProtoGC sources at $d"
        exit 1
    fi
done

mkdir -p "$BUILD_DIR"

echo "F-04 version-counter semantics check"
echo "  compiler: $CXX"

# Same suppression set as build_sim.sh (pre-existing firmware warning classes).
set -e
"$CXX" -std=gnu++17 -Wall -Wextra -O1 -g \
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
    "$SCRIPT_DIR/test_f04_signature.cpp" \
    -o "$BIN"
set +e

echo "build ok, running..."
echo
"$BIN"
rc=$?
exit $rc
