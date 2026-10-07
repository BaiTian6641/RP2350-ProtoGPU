#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${BUILD_DIR:-$ROOT/build/pipeline-tests}"
mkdir -p "$OUT"
"${CXX:-g++}" -std=gnu++17 -O1 -g -Wall -Wextra -pthread \
    -fsanitize=address,undefined -fno-sanitize-recover=all \
    -DPROTOGPU_NATIVE_SCHEDULER -DPGC_BACKEND_DESKTOP -DPROTOGC_OVERRIDE_NEW=0 \
    -I "$ROOT/src" -I "$ROOT/ProtoGL/src" -I "$ROOT/ProtoGC/src" \
    "$ROOT/tests/native/test_command_pipeline.cpp" \
    "$ROOT/src/command_parser.cpp" "$ROOT/src/render/frame_renderer.cpp" \
    "$ROOT/src/phase_scratch.cpp" \
    "$ROOT/src/render/rasterizer.cpp" "$ROOT/src/render/triangle2d.cpp" \
    "$ROOT/src/render/rasterizer_2d.cpp" "$ROOT/src/render/screenspace_effects.cpp" \
    "$ROOT/src/render/pgl_shader_vm.cpp" "$ROOT/src/math/pgl_math.cpp" \
    "$ROOT/src/scheduler/pgl_tile_scheduler.cpp" "$ROOT/src/scheduler/pgl_scheduler_native.cpp" \
    "$ROOT/src/memory/mem_assets.cpp" "$ROOT/src/memory/scene_assets.cpp" "$ROOT/ProtoGC/src/pgc_desktop.cpp" \
    -o "$OUT/test_command_pipeline"
"$OUT/test_command_pipeline"
