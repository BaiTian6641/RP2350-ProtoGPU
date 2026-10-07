#!/usr/bin/env bash
# Native rasterizer-kernel regression tests (protocol 9).
#
# Builds the REAL SceneState + Rasterizer + Triangle2D + PglMath with the
# ProtoGC desktop backend under ASan/UBSan and runs deterministic
# reference-based kernel cases (dense overlap, pool overflow, near clip /
# morph / AABB admission, perspective UV + depth, shared edges, alpha/mask
# semantics, blend saturation, camera targets, service callbacks).
#
# Usage (from the repo root or anywhere else):
#   ./tests/native/run_kernel_tests.sh
# Exit code: 0 = build + all cases green, 1 = failure.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PROTOGL_SRC="$REPO_ROOT/ProtoGL/src"
PROTOGC_SRC="$REPO_ROOT/ProtoGC/src"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/tests}"
CXX="${CXX:-g++}"

for d in "$PROTOGL_SRC" "$PROTOGC_SRC"; do
    if [ ! -d "$d" ]; then
        echo "ERROR: expected ProtoGL/ProtoGC sources at $d"
        exit 1
    fi
done

mkdir -p "$BUILD_DIR"

echo "rasterizer kernel test build"
echo "  compiler: $CXX"

# -O1 pins float codegen the same way as the golden-frame harness.
set -e
"$CXX" -std=gnu++17 -Wall -Wextra -O1 -g \
    -fsanitize=address,undefined -fno-sanitize-recover=all \
    -Wno-class-memaccess -Wno-format -Wno-unused-but-set-variable \
    -Wno-unused-parameter \
    -DPGC_BACKEND_DESKTOP -DPROTOGC_OVERRIDE_NEW=0 \
    -I "$REPO_ROOT/src" -I "$PROTOGL_SRC" -I "$PROTOGC_SRC" \
    "$REPO_ROOT/tests/native/test_rasterizer_kernel.cpp" \
    "$REPO_ROOT/src/phase_scratch.cpp" \
    "$REPO_ROOT/src/render/rasterizer.cpp" \
    "$REPO_ROOT/src/render/triangle2d.cpp" \
    "$REPO_ROOT/src/math/pgl_math.cpp" \
    "$REPO_ROOT/src/memory/mem_assets.cpp" "$REPO_ROOT/src/memory/scene_assets.cpp" \
    "$PROTOGC_SRC/pgc_desktop.cpp" \
    -o "$BUILD_DIR/test_rasterizer_kernel"
set +e

echo "build ok, running..."
echo
"$BUILD_DIR/test_rasterizer_kernel"
rc=$?
exit $rc
