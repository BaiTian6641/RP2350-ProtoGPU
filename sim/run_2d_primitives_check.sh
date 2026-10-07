#!/usr/bin/env bash
# 2D primitive engine regression gate — exact-pixel oracle checks.
#
# Compiles and RUNS sim/test_2d_primitives.cpp against the real firmware
# rasterizer (src/render/rasterizer_2d.cpp) natively. Asserts hand-computed
# pixels for clipping, extreme coordinates, sprite flips/formats/tails,
# blend endpoints/saturation, gradient endpoints, glyph bounds/newline,
# clip/viewport snapshot precedence, stride and service pacing.
#
# Usage (from the repo root or anywhere else):
#   ./sim/run_2d_primitives_check.sh
#
# Exit code: 0 = all checks passed, 1 otherwise.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROTOGL_SRC="$REPO_ROOT/ProtoGL/src"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/sim}"
CXX="${CXX:-g++}"
BIN="$BUILD_DIR/test_2d_primitives"

if [ ! -d "$PROTOGL_SRC" ]; then
    echo "ERROR: expected ProtoGL sources at $PROTOGL_SRC"
    exit 1
fi

mkdir -p "$BUILD_DIR"

echo "2D primitive engine check"
echo "  compiler: $CXX"

set -e
"$CXX" -std=gnu++17 -Wall -Wextra -O1 -g \
    -I "$REPO_ROOT/src" -I "$PROTOGL_SRC" \
    "$REPO_ROOT/src/render/rasterizer_2d.cpp" \
    "$SCRIPT_DIR/test_2d_primitives.cpp" \
    -o "$BIN"
set +e

echo "build ok, running..."
echo
"$BIN"
rc=$?
exit $rc
