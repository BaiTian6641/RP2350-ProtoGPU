#!/usr/bin/env bash
# Clock-slice native tests (P09 deferred profile state machine).
#
# Builds src/gpu_clock.cpp + the assert-based suite with the NATIVE host
# g++ (no Pico SDK, no cross compiler) and runs it. The Platform seam
# records the physical transition calls the shipping logic makes; PLL/QMI
# register execution and electrical qualification remain target/HIL gates.
#
# Usage (from the repo root or anywhere else):
#   ./tests/clock/run_tests.sh
#
# Exit code: 0 if the build and all tests pass, 1 otherwise.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/clock-tests}"
CXX="${CXX:-g++}"

CXXFLAGS="-std=gnu++17 -Wall -Wextra -O1 -g"

echo "Clock profile native tests"
echo "  compiler: $CXX"
echo "  build:    $BUILD_DIR"
echo

mkdir -p "$BUILD_DIR"

if ! "$CXX" $CXXFLAGS -I "$REPO_ROOT/src" -I "$REPO_ROOT/ProtoGL/src" \
        "$REPO_ROOT/src/gpu_clock.cpp" \
        "$SCRIPT_DIR/test_gpu_clock.cpp" \
        -o "$BUILD_DIR/gpu_clock_tests"; then
    echo "RESULT: FAIL (build error)"
    exit 1
fi

echo "build ok, running..."
echo

"$BUILD_DIR/gpu_clock_tests"
status=$?

echo
if [ "$status" -eq 0 ]; then
    exit 0
fi
if [ "$status" -ne 1 ]; then
    echo "RESULT: FAIL (test binary exited with status $status)"
fi
exit 1
