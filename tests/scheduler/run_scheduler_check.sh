#!/usr/bin/env bash
# Bounded dual-core scheduler — native behavior check (P04).
#
# Compiles the REAL scheduler core (src/scheduler/pgl_tile_scheduler.cpp)
# plus the REAL native dual-worker backend (src/scheduler/pgl_scheduler_native.cpp,
# a second std::thread) and runs the behavior suite in check_scheduler.cpp:
# exactly-once tile claims under skew, stale-epoch safety, nonreentrancy,
# async queue/cancel bounds, deterministic partial completion publication,
# maintenance park, shutdown/restart.
#
# -ffunction-sections + --gc-sections: the test never calls the Rasterizer*
# DispatchTilePass overload, so the adapter (the only reference to
# Rasterizer::RasterizeTile, a firmware symbol) is garbage-collected and no
# renderer objects need linking.
#
# Usage (from the repo root or anywhere else):
#   ./tests/scheduler/run_scheduler_check.sh
#
# Exit code: 0 = build + run green, 1 = build or test failure.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/scheduler-check}"
CXX="${CXX:-g++}"

mkdir -p "$BUILD_DIR"

echo "Scheduler native behavior check"
echo "  compiler: $CXX"

set -e
"$CXX" -std=gnu++17 -Wall -Wextra -O1 -g \
    -ffunction-sections -fdata-sections \
    -DPGL_SCHEDULER_TEST_HOOKS \
    -I "$REPO_ROOT/src" -I "$REPO_ROOT/ProtoGL/src" -I "$REPO_ROOT/ProtoGC/src" \
    "$REPO_ROOT/src/scheduler/pgl_tile_scheduler.cpp" \
    "$REPO_ROOT/src/scheduler/pgl_scheduler_native.cpp" \
    "$SCRIPT_DIR/check_scheduler.cpp" \
    -Wl,--gc-sections -pthread \
    -o "$BUILD_DIR/check_scheduler"
set +e

echo "build ok, running..."
echo
"$BUILD_DIR/check_scheduler"
rc=$?
exit $rc
