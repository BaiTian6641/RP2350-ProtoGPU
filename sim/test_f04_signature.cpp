/**
 * @file test_f04_signature.cpp
 * @brief F-04 functional gate — frame-signature version-counter semantics.
 *
 * The golden frames prove pixel-identity but render ONE frame per process,
 * so they never exercise the skipped-frame path.  This test drives the REAL
 * pipeline (PglEncoder → CommandParser::Parse → SceneState →
 * Rasterizer::PrepareFrame) across multiple frames and asserts the exact
 * invalidation semantics the version counters must preserve:
 *
 *   - identical re-sent frame (no mesh writes)      → frame SKIPPED
 *   - transform change in the draw list             → NOT skipped
 *   - UPDATE_VERTICES write                         → NOT skipped
 *   - UPDATE_VERTICES write of IDENTICAL bytes      → NOT skipped
 *     (conservative: the old byte-hash would have skipped; version counters
 *     cannot detect same-content writes — re-render is always safe)
 *   - UPDATE_VERTICES_DELTA write                   → NOT skipped
 *   - DESTROY+CREATE on the same slot (re-gen)      → NOT skipped
 *   - DESTROY_MESH                                  → NOT skipped
 *   - meshVersion[] bumps exactly once per write
 *   - SceneState::Reset() does NOT clear meshVersion[] (monotonic — a scene
 *     rebuilt after reset must never reproduce a pre-reset signature)
 *
 * Exit code: 0 = all checks passed, 1 = at least one check failed.
 */

#include <cstdint>
#include <cstdio>
#include <cstring>

// ─── ProtoGL host side (real wire-format encoder) ───────────────────────────
#include <PglTypes.h>
#include <PglEncoder.h>

// ─── Firmware side (real, unmodified) ───────────────────────────────────────
#include "gpu_config.h"
#include "scene_state.h"
#include "command_parser.h"
#include "render/rasterizer.h"
#include "memory/mem_qspi_vram.h"
#include "memory/mem_tier.h"
#include "memory/mem_pool.h"
#include "display/display_manager.h"

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    if (ok) {
        std::printf("PASS %s\n", what);
    } else {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

// ─── Shared pipeline state (mirrors sim_main.cpp) ───────────────────────────

static uint16_t  framebufferA[GpuConfig::FRAMEBUF_PIXELS];
static uint16_t  framebufferB[GpuConfig::FRAMEBUF_PIXELS];
static uint16_t* frontBuffer = framebufferA;
static uint16_t* backBuffer  = framebufferB;
static uint16_t  zBuffer[GpuConfig::FRAMEBUF_PIXELS];

static SceneState      sceneState;
static Rasterizer      rasterizer;
static MemTierManager  memTierManager;
static MemPoolManager  memPoolManager;
static DisplayManager  displayManager;

static uint8_t cmdBuffer[64 * 1024];

// ─── Minimal cube mesh (from src/selftest/headless_selftest.cpp) ────────────

static const PglVec3 kCubeVerts[] = {
    { -0.5f, -0.5f,  0.5f }, {  0.5f, -0.5f,  0.5f },
    {  0.5f,  0.5f,  0.5f }, { -0.5f,  0.5f,  0.5f },
    { -0.5f, -0.5f, -0.5f }, {  0.5f, -0.5f, -0.5f },
    {  0.5f,  0.5f, -0.5f }, { -0.5f,  0.5f, -0.5f },
};
static const PglIndex3 kCubeIndices[] = {
    { 0, 1, 2 }, { 0, 2, 3 }, { 5, 4, 7 }, { 5, 7, 6 },
    { 1, 5, 6 }, { 1, 6, 2 }, { 4, 0, 3 }, { 4, 3, 7 },
    { 3, 2, 6 }, { 3, 6, 7 }, { 4, 5, 1 }, { 4, 1, 0 },
};
static constexpr uint16_t kCubeVertCount = 8;
static constexpr uint16_t kCubeFaceCount = 12;

static const PglQuat kIdentityQuat = { 1.0f, 0.0f, 0.0f, 0.0f };

static uint32_t g_frameNo = 0;

// ─── Frame helpers ──────────────────────────────────────────────────────────

void BeginCmds(PglEncoder& enc) {
    enc.BeginFrame(++g_frameNo, 16666);
}

void AddCameraAndDraw(PglEncoder& enc, float posX) {
    enc.SetCamera(0, 0, { 0.0f, 0.0f, -5.0f }, kIdentityQuat,
                  { 1.0f, 1.0f, 1.0f }, kIdentityQuat, kIdentityQuat, false);
    enc.DrawObject(0, 0, { posX, 0.0f, 0.0f }, kIdentityQuat,
                   { 2.5f, 2.5f, 2.5f }, kIdentityQuat, kIdentityQuat,
                   { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, true);
}

/// Encode + parse one frame through the real parser, then PrepareFrame.
/// `expectedNewErrors` = parser errors this frame is EXPECTED to latch
/// (fail-closed rejections are counted by design — e.g. a draw call that
/// references a just-destroyed mesh).  Returns rasterizer.IsFrameSkipped().
bool RunFrame(const PglEncoder& enc, uint16_t expectedNewErrors = 0) {
    const uint16_t errBefore = CommandParser::GetParserErrorCount();
    CommandParser::ParseResult result = CommandParser::Parse(
        enc.GetBuffer(), static_cast<uint32_t>(enc.GetLength()), &sceneState);
    if (result != CommandParser::ParseResult::Ok) {
        std::fprintf(stderr, "[f04] FATAL: frame parse failed (%d)\n",
                     static_cast<int>(result));
        ++g_failures;
    }
    const uint16_t errDelta = CommandParser::GetParserErrorCount() - errBefore;
    if (errDelta != expectedNewErrors) {
        std::fprintf(stderr, "[f04] FATAL: frame latched %u parser error(s), expected %u\n",
                     errDelta, expectedNewErrors);
        ++g_failures;
    }
    memTierManager.BeginFrame();
    rasterizer.PrepareFrame(&sceneState);
    return rasterizer.IsFrameSkipped();
}

}  // namespace

int main() {
    // ── Boot-time init (mirrors GpuCore::Initialize / sim RunScene) ──────
    sceneState.InitSceneHeap();
    sceneState.Reset();

    MemTierConfig tierCfg = {};
    tierCfg.sramCacheBudget     = 0;  // SRAM-only (bench config)
    tierCfg.cacheLineSize       = GpuConfig::MEM_TIER_CACHE_LINE_SIZE;
    tierCfg.alphaWeight         = GpuConfig::MEM_TIER_ALPHA_WEIGHT;
    tierCfg.betaScore           = GpuConfig::MEM_TIER_BETA_SCORE;
    tierCfg.demotionThreshold   = GpuConfig::MEM_TIER_DEMOTION_THRESHOLD;
    tierCfg.promotionHysteresis = GpuConfig::MEM_TIER_PROMOTION_HYSTERESIS;
    memTierManager.Initialize(tierCfg, nullptr);

    CommandParser::InitMemory(nullptr, nullptr, &memTierManager,
                              frontBuffer, backBuffer,
                              GpuConfig::FRAMEBUF_PIXELS);
    displayManager.Init();
    CommandParser::InitDisplayAndPools(&displayManager, &memPoolManager);
    rasterizer.Initialize(&sceneState, zBuffer, GpuConfig::PANEL_WIDTH,
                          GpuConfig::PANEL_HEIGHT);

    // ── F1: upload cube mesh + material + camera + draw ──────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);
        PglParamLight light{};
        enc.CreateMaterial(0, PGL_MAT_LIGHT, PGL_BLEND_BASE, &light, sizeof(light));
        AddCameraAndDraw(enc, 0.0f);
        enc.EndFrame();
        check(!RunFrame(enc) , "F1 first frame renders (never skipped)");
        check(sceneState.meshVersion[0] == 1, "F1 CREATE_MESH bumped version to 1");
    }

    // ── F2: identical frame, no mesh writes → MUST skip ───────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.0f);
        enc.EndFrame();
        check(RunFrame(enc), "F2 identical frame SKIPPED (skip optimization intact)");
        check(sceneState.meshVersion[0] == 1, "F2 no mesh write — version unchanged");
    }

    // ── F3: transform change → NOT skipped ────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);  // moved draw position
        enc.EndFrame();
        check(!RunFrame(enc), "F3 transform change NOT skipped");
    }

    // ── F4: identical to F3 → skipped again ───────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(RunFrame(enc), "F4 re-sent frame SKIPPED after transform settled");
    }

    // ── F5: UPDATE_VERTICES (changed bytes) → NOT skipped ─────────────────
    {
        PglVec3 moved[kCubeVertCount];
        std::memcpy(moved, kCubeVerts, sizeof(moved));
        moved[0].x += 0.01f;
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.UpdateVertices(0, moved, kCubeVertCount);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F5 UPDATE_VERTICES write NOT skipped");
        check(sceneState.meshVersion[0] == 2, "F5 UPDATE_VERTICES bumped version to 2");
    }

    // ── F5b: UPDATE_VERTICES of IDENTICAL bytes → still NOT skipped ───────
    // Conservative invalidation: version counters cannot detect same-content
    // writes (the old byte hash would have skipped here).  A spurious
    // re-render is always pixel-correct; a missed invalidation is not.
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.UpdateVertices(0, kCubeVerts, kCubeVertCount);  // original bytes
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F5b same-bytes UPDATE_VERTICES NOT skipped (conservative)");
        check(sceneState.meshVersion[0] == 3, "F5b write bumped version to 3");
    }

    // ── F6: quiet frame → skipped ─────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(RunFrame(enc), "F6 quiet frame SKIPPED after updates settled");
    }

    // ── F7: UPDATE_VERTICES_DELTA → NOT skipped ───────────────────────────
    {
        PglVertexDelta d = { 3, 0.02f, 0.0f, 0.0f };
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.UpdateVerticesDelta(0, &d, 1);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F7 UPDATE_VERTICES_DELTA NOT skipped");
        check(sceneState.meshVersion[0] == 4, "F7 delta write bumped version to 4");
    }

    // ── F8: DESTROY + CREATE same slot, same bytes → NOT skipped ──────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.DestroyMesh(0);
        enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F8 DESTROY+CREATE re-gen NOT skipped");
        check(sceneState.meshVersion[0] == 6, "F8 destroy+create bumped version to 6");
    }

    // ── F9: quiet frame → skipped ─────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(RunFrame(enc), "F9 quiet frame SKIPPED after re-gen settled");
    }

    // ── F10: DESTROY_MESH → NOT skipped (draw now invalid → empty list) ───
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.DestroyMesh(0);
        AddCameraAndDraw(enc, 0.25f);   // rejected: mesh handle invalid
        enc.EndFrame();
        check(!RunFrame(enc, 1), "F10 DESTROY_MESH NOT skipped (draw fail-closed, 1 parser error expected)");
        check(sceneState.meshVersion[0] == 7, "F10 destroy bumped version to 7");
    }

    // ── Reset survival: versions must be monotonic across SceneState::Reset ─
    {
        sceneState.Reset();
        check(sceneState.meshVersion[0] == 7,
              "Reset() does NOT clear meshVersion (monotonic across reset)");

        // Rebuild the identical scene — the signature must still differ from
        // the pre-reset one (versions kept counting), so no stale skip.
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);
        PglParamLight light{};
        enc.CreateMaterial(0, PGL_MAT_LIGHT, PGL_BLEND_BASE, &light, sizeof(light));
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "post-reset rebuilt scene NOT skipped");
        check(sceneState.meshVersion[0] == 8, "post-reset CREATE bumped version to 8");
    }

    // ── Summary ───────────────────────────────────────────────────────────
    std::printf("\n");
    if (g_failures == 0) {
        std::printf("RESULT: PASS (F-04 version-counter semantics)\n");
        return 0;
    }
    std::printf("RESULT: FAIL (%d failing check(s))\n", g_failures);
    return 1;
}
