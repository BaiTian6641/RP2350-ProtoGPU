#pragma once
#include <cstdint>
#include <PglRuntimeProtocol.h>
#include "../scene_state.h"
#include "rasterizer.h"
#include "../scheduler/pgl_tile_scheduler.h"

struct FrameTimings {
    uint32_t prepareUs = 0, rasterUs = 0, effectsUs = 0, layersUs = 0;
    uint32_t triangles = 0;
};

class FrameRenderer {
public:
    FrameRenderer(Rasterizer& rasterizer, PglTileScheduler& scheduler)
        : rasterizer_(rasterizer), scheduler_(scheduler) {}
    PglRuntime::Result Render(SceneState& scene, uint16_t* color, PhaseScratch::DepthWorkspace& depth,
                              uint16_t width, uint16_t height, FrameTimings& timings,
                              uint64_t (*nowUs)(), void (*service)());
private:
    PglRuntime::Result DrawLayers(SceneState&, uint16_t*, uint16_t, uint16_t, bool clears, void(*)());
    void Composite(SceneState&, uint16_t*, uint16_t, uint16_t, void(*)());
    Rasterizer& rasterizer_;
    PglTileScheduler& scheduler_;
};
