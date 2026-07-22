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
 * V9 extension — materialVersion[] / textureVersion[] (same rules):
 *   - UPDATE_MATERIAL (changed AND identical params)  → NOT skipped
 *   - CREATE_TEXTURE / texture re-upload              → NOT skipped
 *   - DESTROY_TEXTURE / DESTROY+CREATE material       → NOT skipped
 *   - quiet frames re-skip after each invalidating write
 *   - exact bump counts; versions survive Reset()
 *
 * Exit code: 0 = all checks passed, 1 = at least one check failed.
 */

#include <cstdint>
#include <cstdio>
#include <cstring>

// ─── ProtoGL host side (real wire-format encoder) ───────────────────────────
#include <PglTypes.h>
#include <PglEncoder.h>
#include <PglShaderCompiler.h>   // runtime PGLSL → PSB for the shader-state checks

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

/// Same, but the draw call uses an explicit material (Combine/sub-material
/// and shader-state sequences need to keep a non-zero material constant).
void AddCameraAndDrawMat(PglEncoder& enc, float posX, uint16_t materialId) {
    enc.SetCamera(0, 0, { 0.0f, 0.0f, -5.0f }, kIdentityQuat,
                  { 1.0f, 1.0f, 1.0f }, kIdentityQuat, kIdentityQuat, false);
    enc.DrawObject(0, materialId, { posX, 0.0f, 0.0f }, kIdentityQuat,
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
        check(sceneState.materialVersion[0] == 2,
              "post-reset CREATE_MATERIAL bumped materialVersion to 2 (F1 created = 1)");
    }

    // ── F-04 extension (V9): materialVersion / textureVersion ──────────────
    // Mirror of the mesh rules for the two remaining content classes that
    // the ROP reads but the draw-list hash cannot see.

    // ── F11: UPDATE_MATERIAL (changed params) → NOT skipped ───────────────
    {
        PglParamLight light{};
        light.ambientR = 40;   // changed vs the all-zero params used so far
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.UpdateMaterial(0, &light, sizeof(light));
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F11 UPDATE_MATERIAL write NOT skipped");
        check(sceneState.materialVersion[0] == 3,
              "F11 UPDATE_MATERIAL bumped materialVersion to 3");
    }

    // ── F11b: UPDATE_MATERIAL of IDENTICAL params → still NOT skipped ──────
    // Same conservative rule as F5b: a spurious re-render is always safe.
    {
        PglParamLight light{};
        light.ambientR = 40;   // identical to F11
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.UpdateMaterial(0, &light, sizeof(light));
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F11b same-params UPDATE_MATERIAL NOT skipped (conservative)");
        check(sceneState.materialVersion[0] == 4,
              "F11b write bumped materialVersion to 4");
    }

    // ── F12: quiet frame → skipped ────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(RunFrame(enc), "F12 quiet frame SKIPPED after material updates settled");
    }

    // ── F13: CREATE_TEXTURE (new active slot) → NOT skipped ───────────────
    static uint16_t texPixels[16 * 16];
    {
        for (int i = 0; i < 16 * 16; ++i) texPixels[i] = (uint16_t)(0xF800 + i);
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.CreateTexture(0, 16, 16, PGL_TEX_RGB565, texPixels);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F13 CREATE_TEXTURE NOT skipped");
        check(sceneState.textureVersion[0] == 1,
              "F13 CREATE_TEXTURE bumped textureVersion to 1");
    }

    // ── F14: CREATE_TEXTURE same slot (re-upload) → NOT skipped ───────────
    {
        for (int i = 0; i < 16 * 16; ++i) texPixels[i] = (uint16_t)(0x001F + i);
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.CreateTexture(0, 16, 16, PGL_TEX_RGB565, texPixels);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F14 texture re-upload NOT skipped");
        check(sceneState.textureVersion[0] == 2,
              "F14 re-upload bumped textureVersion to 2");
    }

    // ── F15: quiet frame → skipped ────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(RunFrame(enc), "F15 quiet frame SKIPPED after texture uploads settled");
    }

    // ── F16: DESTROY_TEXTURE → NOT skipped (leaves the version hash) ──────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.DestroyTexture(0);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F16 DESTROY_TEXTURE NOT skipped");
        check(sceneState.textureVersion[0] == 3,
              "F16 destroy bumped textureVersion to 3");
    }

    // ── F17: quiet frame → skipped ────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(RunFrame(enc), "F17 quiet frame SKIPPED after texture destroy settled");
    }

    // ── F18: DESTROY_MATERIAL + CREATE_MATERIAL (re-gen) → NOT skipped ────
    {
        PglParamLight light{};
        light.ambientR = 40;   // same params as F11/F11b — re-gen must still bump
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.DestroyMaterial(0);
        enc.CreateMaterial(0, PGL_MAT_LIGHT, PGL_BLEND_BASE, &light, sizeof(light));
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F18 DESTROY+CREATE material re-gen NOT skipped");
        check(sceneState.materialVersion[0] == 6,
              "F18 destroy+create bumped materialVersion to 6");
    }

    // ── F19: quiet frame → skipped ────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(RunFrame(enc), "F19 quiet frame SKIPPED after material re-gen settled");
    }

    // ── Reset survival (material/texture versions — same monotonic rule) ──
    {
        sceneState.Reset();
        check(sceneState.materialVersion[0] == 6 && sceneState.textureVersion[0] == 3,
              "Reset() does NOT clear material/textureVersion (monotonic across reset)");

        // Rebuild the identical scene — versions kept counting, so the
        // signature must still differ from the pre-reset one (no stale skip).
        PglParamLight light{};
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);
        enc.CreateMaterial(0, PGL_MAT_LIGHT, PGL_BLEND_BASE, &light, sizeof(light));
        enc.CreateTexture(0, 16, 16, PGL_TEX_RGB565, texPixels);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "post-reset rebuilt scene NOT skipped (mat/tex versions)");
        check(sceneState.materialVersion[0] == 7 &&
              sceneState.textureVersion[0] == 4,
              "post-reset CREATEs bumped materialVersion to 7, textureVersion to 4");
    }

    // ── F-04 extension (V9): all-active materialVersion + shaderStateVersion ─

    // ── F20: build a Combine material chain (mat 2 references sub-mat 1) ──
    // The draw call references material 2 ONLY — material 1 is reached
    // indirectly through the Combine params (no draw-call reference chain).
    {
        PglParamSimple red{};
        red.r = 255; red.g = 0; red.b = 0;
        PglParamCombine comb{};
        comb.materialIdA = 1;
        comb.materialIdB = 1;
        comb.blendMode   = PGL_BLEND_BASE;
        comb.opacity     = 1.0f;
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.CreateMaterial(1, PGL_MAT_SIMPLE, PGL_BLEND_BASE, &red, sizeof(red));
        enc.CreateMaterial(2, PGL_MAT_COMBINE, PGL_BLEND_BASE, &comb, sizeof(comb));
        AddCameraAndDrawMat(enc, 0.25f, 2);
        enc.EndFrame();
        check(!RunFrame(enc), "F20 Combine chain setup renders (not skipped)");
        check(sceneState.materialVersion[1] == 1 && sceneState.materialVersion[2] == 1,
              "F20 creates bumped materialVersion[1..2] to 1");
    }

    // ── F21: quiet frame → skipped ────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDrawMat(enc, 0.25f, 2);
        enc.EndFrame();
        check(RunFrame(enc), "F21 quiet frame SKIPPED (Combine chain settled)");
    }

    // ── F22: UPDATE the SUB-material (not referenced by any draw call) ────
    // THE headline all-active check: with referenced-only hashing this frame
    // would skip stale — material 1 is only reachable via material 2's
    // Combine params.  All-active folding must invalidate.
    {
        PglParamSimple blue{};
        blue.r = 0; blue.g = 0; blue.b = 255;   // changed colour
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.UpdateMaterial(1, &blue, sizeof(blue));
        AddCameraAndDrawMat(enc, 0.25f, 2);
        enc.EndFrame();
        check(!RunFrame(enc), "F22 Combine SUB-material update NOT skipped (all-active)");
        check(sceneState.materialVersion[1] == 2,
              "F22 sub-material write bumped materialVersion[1] to 2");
    }

    // ── F23: quiet frame → skipped ────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDrawMat(enc, 0.25f, 2);
        enc.EndFrame();
        check(RunFrame(enc), "F23 quiet frame SKIPPED after sub-material update settled");
    }

    // ── F24: shader-state setup — compile + create + bind + uniform ───────
    // Minimal PGLSL program with one time-style user uniform (the realistic
    // trigger: host re-uses the bound program and animates the uniform only).
    static const char kPglslTime[] = R"pglsl(
uniform float u_time;
void main() {
    vec2 uv = gl_FragCoord.xy / u_resolution;
    vec4 color = texture2D(u_framebuffer, uv);
    gl_FragColor = vec4(color.rgb * (0.5 + 0.5 * u_time), 1.0);
}
)pglsl";
    {
        auto res = PglShaderCompiler::Compile(kPglslTime,
                                              std::strlen(kPglslTime));
        check(res.success, "F24 shader test program compiles");
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        if (res.success) {
            enc.CreateShaderProgram(0, res.bytecode, res.bytecodeSize);
        }
        AddCameraAndDraw(enc, 0.25f);            // SetCamera BEFORE bind
        enc.BindShaderProgram(0, 0, 0, 1.0f);    // camera 0, slot 0, program 0
        enc.SetShaderUniform(0, PSB_USER_UNIFORM_START, 0.0f);
        enc.EndFrame();
        check(!RunFrame(enc), "F24 shader program create+bind+uniform NOT skipped");
        check(sceneState.shaderStateVersion == 3,
              "F24 create+bind+uniform bumped shaderStateVersion to 3");
    }

    // ── F25: quiet frame → skipped ────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(RunFrame(enc), "F25 quiet frame SKIPPED (shader state settled)");
    }

    // ── F26: uniform-ONLY update on a static scene → NOT skipped ──────────
    // THE headline shader-state check: nothing else changed — without the
    // shaderStateVersion fold this frame is skipped and the post-FX freezes.
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.SetShaderUniform(0, PSB_USER_UNIFORM_START, 0.5f);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F26 uniform-only update NOT skipped (no frozen animation)");
        check(sceneState.shaderStateVersion == 4,
              "F26 uniform write bumped shaderStateVersion to 4");
    }

    // ── F27: quiet frame → skipped ────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(RunFrame(enc), "F27 quiet frame SKIPPED after uniform settled");
    }

    // ── F28: uniform write of the SAME value → still NOT skipped ──────────
    // Same conservative rule as F5b/F11b: counters cannot detect same-value
    // writes; a spurious re-render is always safe.
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.SetShaderUniform(0, PSB_USER_UNIFORM_START, 0.5f);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F28 same-value uniform write NOT skipped (conservative)");
        check(sceneState.shaderStateVersion == 5,
              "F28 write bumped shaderStateVersion to 5");
    }

    // ── F29: quiet frame → skipped ────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(RunFrame(enc), "F29 quiet frame SKIPPED after same-value write settled");
    }

    // ── F30: re-BIND with changed intensity → NOT skipped ─────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.BindShaderProgram(0, 0, 0, 0.5f);    // same program, new intensity
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "F30 bind-program change NOT skipped");
        check(sceneState.shaderStateVersion == 6,
              "F30 bind bumped shaderStateVersion to 6");
    }

    // ── F31: quiet frame → skipped ────────────────────────────────────────
    {
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(RunFrame(enc), "F31 quiet frame SKIPPED after re-bind settled");
    }

    // ── Reset survival (shaderStateVersion — same monotonic rule) ─────────
    {
        sceneState.Reset();
        check(sceneState.shaderStateVersion == 6,
              "Reset() does NOT clear shaderStateVersion (monotonic across reset)");

        // Rebuild WITHOUT any shader commands — the per-slot versions keep
        // counting, so the rebuilt scene must still render (no stale skip).
        PglParamLight light{};
        PglEncoder enc(cmdBuffer, sizeof(cmdBuffer));
        BeginCmds(enc);
        enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);
        enc.CreateMaterial(0, PGL_MAT_LIGHT, PGL_BLEND_BASE, &light, sizeof(light));
        AddCameraAndDraw(enc, 0.25f);
        enc.EndFrame();
        check(!RunFrame(enc), "post-reset rebuilt scene NOT skipped (shader version)");
        check(sceneState.shaderStateVersion == 6,
              "shader-state-free rebuild leaves shaderStateVersion at 6");
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
