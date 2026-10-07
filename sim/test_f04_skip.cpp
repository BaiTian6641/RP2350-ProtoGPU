// Full-redraw cutover regression: historical F04 skipping is intentionally gone.
// Exercise actual pixels across buffer reuse, geometry/material updates and
// resource-only admission through the live parser, frame engine and two workers.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <PglEncoder.h>
#include "gpu_config.h"
#include "scene_state.h"
#include "command_parser.h"
#include "render/frame_renderer.h"

namespace {
constexpr uint16_t W = GpuConfig::PANEL_WIDTH, H = GpuConfig::PANEL_HEIGHT;
constexpr size_t Pixels = size_t(W) * H;
const PglQuat Identity{1, 0, 0, 0};
const PglVec3 Vertices[] = {
    {-0.5f,-0.5f,0.5f}, {0.5f,-0.5f,0.5f}, {0.5f,0.5f,0.5f}, {-0.5f,0.5f,0.5f},
    {-0.5f,-0.5f,-0.5f}, {0.5f,-0.5f,-0.5f}, {0.5f,0.5f,-0.5f}, {-0.5f,0.5f,-0.5f}
};
const PglIndex3 Indices[] = {
    {0,1,2},{0,2,3},{5,4,7},{5,7,6},{1,5,6},{1,6,2},
    {4,0,3},{4,3,7},{3,2,6},{3,6,7},{4,5,1},{4,1,0}
};
SceneState scene;
Rasterizer rasterizer;
PglTileScheduler scheduler;
FrameRenderer renderer(rasterizer, scheduler);
std::array<uint16_t, Pixels> colorA{}, colorB{};
PhaseScratch::DepthWorkspace depth;
uint16_t* front = colorA.data();
uint16_t* back = colorB.data();
uint8_t bytes[32 * 1024];
unsigned failures = 0;
uint32_t frameNumber = 0;

void Check(bool ok, const char* message) {
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", message);
    failures += !ok;
}
uint64_t NowUs() {
    return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
void Begin(PglEncoder& encoder) { encoder.BeginFrame(++frameNumber, 16666); }
void Camera(PglEncoder& encoder) {
    encoder.SetCamera(0, 0, {0,0,-5}, Identity, {1,1,1}, Identity, Identity, false);
}
void Draw(PglEncoder& encoder, float x = 0) {
    encoder.DrawObject(0, 0, {x,0,0}, Identity, {2.5f,2.5f,2.5f},
                       Identity, Identity, {0,0,0}, {0,0,0}, true);
}
bool Submit(PglEncoder& encoder, bool resourceOnly = false) {
    if (resourceOnly) encoder.EndResourceBatch(); else encoder.EndFrame();
    if (encoder.HasOverflow() || encoder.HasInvalidCommand()) {
        Check(false, "valid host encoding");
        return false;
    }
    CommandParser::BatchInfo info;
    const auto parsed = CommandParser::Parse(encoder.GetBuffer(), encoder.GetLength(),
                                             &scene, info, resourceOnly);
    if (parsed != PglRuntime::Result::Ok) {
        std::fprintf(stderr, "parser result=%u mask=0x%08X\n", unsigned(parsed),
                     CommandParser::GetParserErrorMask());
        Check(false, "accepted live parser batch");
        return false;
    }
    if (resourceOnly) return true;
    // A skipped/no-op render cannot accidentally pass by preserving the old
    // target: poison every reused target before asking the engine to redraw it.
    std::fill_n(back, Pixels, uint16_t(0xA55A));
    FrameTimings timing;
    const auto result = renderer.Render(scene, back, depth, W, H, timing, NowUs, nullptr);
    if (result != PglRuntime::Result::Ok) {
        std::fprintf(stderr, "renderer result=%u\n", unsigned(result));
        Check(false, "completed live frame");
        return false;
    }
    std::swap(front, back);
    scene.RetireFrameData();
    return true;
}
size_t Coverage() { return std::count_if(front, front + Pixels, [](uint16_t p) { return p != 0; }); }
bool Same(const std::array<uint16_t, Pixels>& image) { return std::equal(image.begin(), image.end(), front); }
} // namespace

int main() {
    scene.renderWidth = W;
    scene.renderHeight = H;
    if (!scene.InitSceneHeap()) { std::fprintf(stderr, "scene heap initialization failed\n"); return 1; }
    scene.Reset();
    scheduler.Initialize();
    if (!scheduler.StartWorker()) { std::fprintf(stderr, "native worker startup failed\n"); return 1; }
    struct WorkerLifetime { ~WorkerLifetime() { scheduler.Shutdown(); } } workerLifetime;
    const PglParamSimple red{255,0,0}, green{0,255,0}, blue{0,0,255};
    {
        PglEncoder encoder(bytes, sizeof(bytes)); Begin(encoder);
        encoder.CreateMesh(0, Vertices, 8, Indices, 12);
        encoder.CreateMaterial(0, PGL_MAT_SIMPLE, PGL_BLEND_BASE, &red, sizeof(red));
        Camera(encoder); Draw(encoder);
        if (!Submit(encoder)) return 1;
    }
    std::array<uint16_t, Pixels> original;
    std::copy_n(front, Pixels, original.data());
    const size_t originalCoverage = Coverage();
    Check(front[size_t(H/2)*W + W/2] == 0xF800 && originalCoverage > 100,
          "initial opaque red geometry occupies the center");
    for (unsigned repeat = 0; repeat < 3; ++repeat) {
        PglEncoder encoder(bytes, sizeof(bytes)); Begin(encoder); Draw(encoder);
        if (!Submit(encoder)) return 1;
        Check(Same(original), "identical frame fully redraws either poisoned back buffer");
    }
    {
        PglEncoder encoder(bytes, sizeof(bytes)); Begin(encoder); Draw(encoder, 1.5f);
        if (!Submit(encoder)) return 1;
        Check(!Same(original), "transform change changes presented geometry");
    }
    {
        PglVec3 smaller[8];
        for (unsigned i = 0; i < 8; ++i) smaller[i] = {Vertices[i].x*0.5f,Vertices[i].y*0.5f,Vertices[i].z*0.5f};
        PglEncoder encoder(bytes, sizeof(bytes)); Begin(encoder);
        encoder.UpdateVertices(0, smaller, 8); Draw(encoder);
        if (!Submit(encoder)) return 1;
        Check(Coverage() > 0 && Coverage() < originalCoverage,
              "vertex update shrinks visible geometry");
    }
    {
        PglEncoder encoder(bytes, sizeof(bytes)); Begin(encoder);
        encoder.UpdateVertices(0, Vertices, 8); encoder.UpdateMaterial(0, &green, sizeof(green)); Draw(encoder);
        if (!Submit(encoder)) return 1;
        Check(front[size_t(H/2)*W+W/2] == 0x07E0, "material update reaches actual presented pixels");
    }
    std::array<uint16_t, Pixels> visible;
    std::copy_n(front, Pixels, visible.data());
    const uint64_t elapsed = scene.elapsedTimeUs;
    const uint32_t visibleFrame = scene.frameNumber;
    uint16_t* const visibleBuffer = front;
    {
        PglEncoder encoder(bytes, sizeof(bytes)); encoder.BeginResourceBatch();
        encoder.UpdateMaterial(0, &blue, sizeof(blue));
        if (!Submit(encoder, true)) return 1;
        Check(front == visibleBuffer && Same(visible), "resource-only material commit does not present");
        Check(scene.elapsedTimeUs == elapsed && scene.frameNumber == visibleFrame,
              "resource-only upload does not advance displayed animation state");
    }
    {
        PglEncoder encoder(bytes, sizeof(bytes)); Begin(encoder); Draw(encoder);
        if (!Submit(encoder)) return 1;
        Check(front[size_t(H/2)*W+W/2] == 0x001F, "next normal frame consumes resource-only material update");
    }
    {
        PglEncoder encoder(bytes, sizeof(bytes)); Begin(encoder);
        encoder.SetCameraTarget(0, 0, 0, 0, W/2, H, PGL_CAMERA_TARGET_SCISSOR); Draw(encoder);
        if (!Submit(encoder)) return 1;
        bool rightBlack = true;
        for (uint16_t y = 0; y < H; ++y)
            for (uint16_t x = W/2; x < W; ++x) rightBlack &= front[size_t(y)*W+x] == 0;
        Check(Coverage() > 0 && rightBlack, "scissored camera clears old pixels outside its pass");
    }
    {
        PglEncoder encoder(bytes, sizeof(bytes)); Begin(encoder);
        if (!Submit(encoder)) return 1;
        Check(Coverage() == 0, "empty submitted frame presents a fully cleared image");
    }
    std::printf("FULL REDRAW RESULT: %s\n", failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}
