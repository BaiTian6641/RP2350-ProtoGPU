#!/usr/bin/env bash
# Shader VM + screenspace post-FX native checks (P05-07/P05-08).
#
# Compiles the firmware's pgl_shader_vm.cpp and screenspace_effects.cpp
# against the real scene_state.h with the ProtoGC DESKTOP backend (native
# g++, no Pico SDK, no hardware) and runs both functional suites:
#   - test_shader_decode_vm.cpp     PSB1 decode/verification + VM semantics
#   - test_screenspace_effects.cpp  ApplyShaderSlots engine behaviour
#
# Usage (from the repo root or anywhere else):
#   ./tests/shader_vm/run_shader_checks.sh
#
# Exit code: 0 = build + all tests green, 1 = build or test failure.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PROTOGL_SRC="$REPO_ROOT/ProtoGL/src"
PROTOGC_SRC="$REPO_ROOT/ProtoGC/src"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/shader-checks}"
CXX="${CXX:-g++}"

for d in "$PROTOGL_SRC" "$PROTOGC_SRC"; do
    if [ ! -d "$d" ]; then
        echo "ERROR: expected ProtoGL/ProtoGC sources at $d"
        exit 1
    fi
done

mkdir -p "$BUILD_DIR"

CXXFLAGS="-std=gnu++17 -Wall -Wextra -O1 -g \
    -DPGC_BACKEND_DESKTOP -DPROTOGC_OVERRIDE_NEW=0 \
    -I $REPO_ROOT/src -I $PROTOGL_SRC -I $PROTOGC_SRC"

rc=0

echo "Shader decode/VM native check"
echo "  compiler: $CXX"
# shellcheck disable=SC2086
"$CXX" $CXXFLAGS \
    "$PROTOGC_SRC/pgc_desktop.cpp" \
    "$REPO_ROOT/src/render/pgl_shader_vm.cpp" \
    "$SCRIPT_DIR/test_shader_decode_vm.cpp" \
    -o "$BUILD_DIR/test_shader_decode_vm" || exit 1
"$BUILD_DIR/test_shader_decode_vm" || rc=1

echo
echo "Screenspace effects native check"
# shellcheck disable=SC2086
"$CXX" $CXXFLAGS \
    "$PROTOGC_SRC/pgc_desktop.cpp" \
    "$REPO_ROOT/src/render/pgl_shader_vm.cpp" \
    "$REPO_ROOT/src/render/screenspace_effects.cpp" \
    "$REPO_ROOT/src/scheduler/pgl_tile_scheduler.cpp" \
    "$REPO_ROOT/src/scheduler/pgl_scheduler_native.cpp" \
    "$SCRIPT_DIR/test_screenspace_effects.cpp" \
    -ffunction-sections -fdata-sections -Wl,--gc-sections -pthread -o "$BUILD_DIR/test_screenspace_effects" || exit 1
"$BUILD_DIR/test_screenspace_effects" || rc=1

exit $rc
