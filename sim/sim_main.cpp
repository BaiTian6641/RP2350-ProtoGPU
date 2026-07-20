/**
 * @file sim_main.cpp
 * @brief Desktop soft-GPU simulator — milestone A5-1 (skeleton).
 *
 * Runs the REAL RP2350-ProtoGPU render pipeline on a desktop host:
 *
 *   PglEncoder (ProtoGL, host side)
 *     → CommandParser::Parse        (real firmware parser, sync + CRC-16)
 *     → SceneState                  (real, ProtoGC desktop heap backend)
 *     → Rasterizer::PrepareFrame    (real: transform → project → QuadTree)
 *     → serial tile rasterization   (real Rasterizer::RasterizeTile, driven
 *                                    in the firmware's Morton tile order —
 *                                    see note below)
 *     → ScreenspaceShaders::ApplyShaders (real; no-op without active shaders)
 *     → PPM dump + content self-check → exit 0/1
 *
 * The serial tile loop replicates PglTileScheduler::ProcessTiles()
 * (src/scheduler/pgl_tile_scheduler.cpp) 1:1.  That .cpp is the only render-path
 * file that cannot compile on desktop (RP2350 multicore FIFO), so the sim
 * re-uses its portable header (TileConfig::MORTON_ORDER / COLS / ROWS) and
 * re-implements the 6-line dispatch loop with the atomic counter replaced by a
 * plain for-loop.  On the real hardware both cores write disjoint tiles, so the
 * serial result is bit-identical to the dual-core result.
 *
 * What is shimmed (see sim_qspi_vram_stub.cpp):
 *   - QspiVramDriver  — all methods no-op/false (QSPI_VRAM_MODE = NONE semantics,
 *                       matching the current bench config in gpu_config.h).
 *   - FlashPersistManager — in-memory no-op (flash is hardware; not exercised
 *                       by the test scene, but command_parser.cpp holds a static
 *                       instance so the symbols must resolve).
 *
 * Everything else in the render path is the unmodified firmware code.
 *
 * Scene: unit cube (mesh data copied from the headless self-test,
 * src/selftest/headless_selftest.cpp) scaled 2.5× at the origin, gently rotated,
 * warm directional-light material, camera at (0,0,-5) looking toward +Z.
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <filesystem>

// ─── ProtoGL host side (real wire-format encoder) ───────────────────────────
#include <PglTypes.h>
#include <PglEncoder.h>

// ─── Firmware side (real, unmodified) ───────────────────────────────────────
#include "gpu_config.h"
#include "scene_state.h"
#include "command_parser.h"
#include "math/pgl_math.h"
#include "render/rasterizer.h"
#include "render/screenspace_effects.h"
#include "scheduler/pgl_tile_scheduler.h"   // TileConfig only — see header note
#include "memory/mem_qspi_vram.h"
#include "memory/mem_tier.h"
#include "memory/mem_pool.h"
#include "display/display_manager.h"

// ─── Test Scene — Cube (from src/selftest/headless_selftest.cpp) ────────────

static const PglVec3 kCubeVerts[] = {
    { -0.5f, -0.5f,  0.5f },  // 0  front bottom-left
    {  0.5f, -0.5f,  0.5f },  // 1  front bottom-right
    {  0.5f,  0.5f,  0.5f },  // 2  front top-right
    { -0.5f,  0.5f,  0.5f },  // 3  front top-left
    { -0.5f, -0.5f, -0.5f },  // 4  back  bottom-left
    {  0.5f, -0.5f, -0.5f },  // 5  back  bottom-right
    {  0.5f,  0.5f, -0.5f },  // 6  back  top-right
    { -0.5f,  0.5f, -0.5f },  // 7  back  top-left
};
static const PglIndex3 kCubeIndices[] = {
    { 0, 1, 2 }, { 0, 2, 3 },   // Front  (+Z)
    { 5, 4, 7 }, { 5, 7, 6 },   // Back   (-Z)
    { 1, 5, 6 }, { 1, 6, 2 },   // Right  (+X)
    { 4, 0, 3 }, { 4, 3, 7 },   // Left   (-X)
    { 3, 2, 6 }, { 3, 6, 7 },   // Top    (+Y)
    { 4, 5, 1 }, { 4, 1, 0 },   // Bottom (-Y)
};
static constexpr uint16_t kCubeVertCount = 8;
static constexpr uint16_t kCubeFaceCount = 12;

// ─── Sim State (mirrors gpu_core.cpp statics) ───────────────────────────────

static constexpr uint16_t W = GpuConfig::PANEL_WIDTH;    // 128
static constexpr uint16_t H = GpuConfig::PANEL_HEIGHT;   // 64

static uint16_t  framebufferA[GpuConfig::FRAMEBUF_PIXELS];
static uint16_t  framebufferB[GpuConfig::FRAMEBUF_PIXELS];
static uint16_t* frontBuffer = framebufferA;
static uint16_t* backBuffer  = framebufferB;
static uint16_t  zBuffer[GpuConfig::FRAMEBUF_PIXELS];

static SceneState      sceneState;
static Rasterizer      rasterizer;
static QspiVramDriver  vramDriver;      // sim stub — never initialized (NONE mode)
static MemTierManager  memTierManager;
static MemPoolManager  memPoolManager;
static DisplayManager  displayManager;

static uint8_t         cmdBuffer[16 * 1024];

// ─── PPM Dump ───────────────────────────────────────────────────────────────

static bool WritePPM(const char* path, const uint16_t* fb, uint16_t w, uint16_t h) {
    std::error_code ec;
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path(), ec);

    FILE* f = std::fopen(path, "wb");
    if (!f) {
        std::fprintf(stderr, "[sim] ERROR: cannot open %s for writing\n", path);
        return false;
    }
    std::fprintf(f, "P6\n%u %u\n255\n", w, h);
    for (uint32_t i = 0; i < static_cast<uint32_t>(w) * h; ++i) {
        uint16_t c = fb[i];
        // Same unpack as the firmware rasterizer (no bit replication).
        uint8_t rgb[3] = {
            static_cast<uint8_t>(((c >> 11) & 0x1F) << 3),
            static_cast<uint8_t>(((c >> 5)  & 0x3F) << 2),
            static_cast<uint8_t>((c & 0x1F) << 3),
        };
        std::fwrite(rgb, 1, 3, f);
    }
    std::fclose(f);
    return true;
}

// ─── ASCII Preview (2×2 downsample → 64×32) ─────────────────────────────────

static void PrintAsciiPreview(const uint16_t* fb, uint16_t w, uint16_t h) {
    static const char kRamp[] = " .:-=+*#%@";   // 10 levels
    std::printf("\n[sim] Framebuffer preview (%ux%u → %ux%u chars):\n+", w, h, w / 2, h / 2);
    for (uint16_t x = 0; x < w / 2; ++x) std::putchar('-');
    std::printf("+\n");
    for (uint16_t y = 0; y < h; y += 2) {
        std::putchar('|');
        for (uint16_t x = 0; x < w; x += 2) {
            uint32_t lum = 0;
            for (uint16_t dy = 0; dy < 2; ++dy) {
                for (uint16_t dx = 0; dx < 2; ++dx) {
                    uint16_t c = fb[(y + dy) * w + (x + dx)];
                    uint32_t r = (c >> 11) & 0x1F;
                    uint32_t g = (c >> 5)  & 0x3F;
                    uint32_t b =  c        & 0x1F;
                    lum += (r * 2 + g * 3 + b) / 6;   // rough luminance, 0..~31
                }
            }
            lum /= 4;
            uint32_t level = lum * 9 / 31;
            std::putchar(kRamp[level > 9 ? 9 : level]);
        }
        std::printf("|\n");
    }
    std::putchar('+');
    for (uint16_t x = 0; x < w / 2; ++x) std::putchar('-');
    std::printf("+\n\n");
}

// ─── Content Self-Check ─────────────────────────────────────────────────────

static bool CheckFramebuffer(const uint16_t* fb, uint16_t w, uint16_t h) {
    uint32_t nonBlack = 0;
    uint16_t minX = w, minY = h, maxX = 0, maxY = 0;
    for (uint16_t y = 0; y < h; ++y) {
        for (uint16_t x = 0; x < w; ++x) {
            if (fb[y * w + x] != 0x0000) {
                ++nonBlack;
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
        }
    }

    const uint32_t total = static_cast<uint32_t>(w) * h;
    std::printf("[sim] Non-background pixels: %u / %u (%.1f%%)\n",
                nonBlack, total, 100.0 * nonBlack / total);
    if (nonBlack > 0) {
        std::printf("[sim] Content bbox: x[%u..%u] y[%u..%u] (%ux%u)\n",
                    minX, maxX, minY, maxY, maxX - minX + 1, maxY - minY + 1);
    }

    bool ok = true;
    auto check = [&](bool cond, const char* what) {
        std::printf("  [%s] %s\n", cond ? " OK " : "FAIL", what);
        if (!cond) ok = false;
    };

    // Expect the cube to cover a substantial central region but not the frame edge.
    check(nonBlack > 800 && nonBlack < total * 3 / 4,
          "coverage in sane range (800 < n < 75% of frame)");
    check(minX > 4 && minY > 2 && maxX < w - 5 && maxY < h - 3,
          "content does not touch the frame border");
    if (nonBlack > 0) {
        int cx = (static_cast<int>(minX) + maxX) / 2;
        int cy = (static_cast<int>(minY) + maxY) / 2;
        int bw = maxX - minX + 1;
        int bh = maxY - minY + 1;
        check(cx >= w / 2 - 12 && cx <= w / 2 + 12 &&
              cy >= h / 2 - 12 && cy <= h / 2 + 12,
              "content bbox centered (±12 px of panel center)");
        check(bw >= 16 && bh >= 12, "content bbox has substance (≥16×12)");
    }
    check(fb[0] == 0x0000 && fb[w - 1] == 0x0000 &&
          fb[(h - 1) * w] == 0x0000 && fb[h * w - 1] == 0x0000,
          "all four corners remain background (black)");
    return ok;
}

// ─── Scene Encoding (host side, real PglEncoder) ────────────────────────────

static size_t EncodeTestScene(uint8_t* buf, size_t capacity) {
    PglEncoder enc(buf, capacity);

    enc.BeginFrame(1, 16666);

    // Mesh 0: cube (no UV).
    enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);

    // Material 0: warm directional light (front-lit variant of the self-test's).
    PglParamLight light{};
    light.lightDirX = 0.5f;  light.lightDirY = 0.8f;  light.lightDirZ = 0.6f;
    light.ambientR  = 40;    light.ambientG  = 15;    light.ambientB  = 10;
    light.diffuseR  = 255;   light.diffuseG  = 190;   light.diffuseB  = 80;
    enc.CreateMaterial(0, PGL_MAT_LIGHT, PGL_BLEND_BASE, &light, sizeof(light));

    // Camera 0: perspective at z=-5, identity rotation (looks toward +Z).
    const PglQuat identity = { 1.0f, 0.0f, 0.0f, 0.0f };
    enc.SetCamera(0, 0,
                  { 0.0f, 0.0f, -5.0f }, identity,
                  { 1.0f, 1.0f, 1.0f }, identity, identity, false);

    // Draw call: cube at origin, scale 2.5, fixed tilt (0.6 rad Y + 0.35 rad X)
    // so three faces are visible with distinct light shading.
    const float halfY = 0.30f, halfX = 0.175f;
    PglQuat qy  = { cosf(halfY), 0.0f, sinf(halfY), 0.0f };
    PglQuat qx  = { cosf(halfX), sinf(halfX), 0.0f, 0.0f };
    PglQuat rot = PglMath::QuatMul(qy, qx);
    enc.DrawObject(0, 0,
                   { 0.0f, 0.0f, 0.0f }, rot,
                   { 2.5f, 2.5f, 2.5f },
                   identity, identity,
                   { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                   true);

    enc.EndFrame();

    if (enc.HasOverflow()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return 0;
    }
    return enc.GetLength();
}

// ─── FNV-1a (frame checksum for future golden-reference comparisons) ────────

static uint32_t Fnv1a(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t hsh = 0x811c9dc5u;
    for (size_t i = 0; i < len; ++i) { hsh ^= p[i]; hsh *= 0x01000193u; }
    return hsh;
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const char* ppmPath = (argc > 1) ? argv[1] : "sim/out/frame.ppm";

    std::printf("=== RP2350-ProtoGPU desktop soft-GPU sim (A5-1 skeleton) ===\n");
    std::printf("[sim] Panel: %ux%u RGB565, tiles %ux%u (%ux%u px, Morton order)\n",
                W, H, TileConfig::COLS, TileConfig::ROWS,
                TileConfig::TILE_W, TileConfig::TILE_H);

    // ── 1. Boot-time init (mirrors GpuCore::Initialize, SRAM-only path) ──
    sceneState.InitSceneHeap();
    sceneState.Reset();

    MemTierConfig tierCfg = {};
    tierCfg.sramCacheBudget     = 0;  // VRAMless mode (gpu_core SRAM_ONLY path)
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

    rasterizer.Initialize(&sceneState, zBuffer, W, H);

    // ── 2. Encode the test scene with the real ProtoGL encoder ──────────
    size_t frameLen = EncodeTestScene(cmdBuffer, sizeof(cmdBuffer));
    if (frameLen == 0) return 1;
    std::printf("[sim] Encoded frame: %zu bytes\n", frameLen);

    // ── 3. Parse through the REAL firmware CommandParser (sync + CRC) ───
    CommandParser::ParseResult result =
        CommandParser::Parse(cmdBuffer, static_cast<uint32_t>(frameLen), &sceneState);
    std::printf("[sim] Parse result: %d, parser errors: %u (mask 0x%04X), "
                "draw calls: %u\n",
                static_cast<int>(result), CommandParser::GetParserErrorCount(),
                CommandParser::GetParserErrorMask(), sceneState.drawCallCount);

    bool ok = true;
    if (result != CommandParser::ParseResult::Ok) {
        std::fprintf(stderr, "[sim] FAIL: frame did not parse cleanly\n");
        ok = false;
    }
    if (CommandParser::GetParserErrorCount() != 0) {
        std::fprintf(stderr, "[sim] FAIL: parser latched %u error(s)\n",
                     CommandParser::GetParserErrorCount());
        ok = false;
    }
    if (sceneState.drawCallCount != 1) {
        std::fprintf(stderr, "[sim] FAIL: expected 1 draw call, got %u\n",
                     sceneState.drawCallCount);
        ok = false;
    }
    if (!ok) return 1;

    // ── 4. Phase 1: PrepareFrame (transform → project → QuadTree) ───────
    memTierManager.BeginFrame();
    rasterizer.PrepareFrame(&sceneState);
    if (rasterizer.IsFrameSkipped()) {
        std::fprintf(stderr, "[sim] FAIL: first frame unexpectedly skipped\n");
        return 1;
    }
    rasterizer.SetElapsedTime(0.0f);
    std::printf("[sim] PrepareFrame: %u triangles projected\n",
                rasterizer.GetTriangleCount());
    if (rasterizer.GetTriangleCount() != kCubeFaceCount) {
        std::fprintf(stderr, "[sim] FAIL: expected %u triangles, got %u\n",
                     kCubeFaceCount, rasterizer.GetTriangleCount());
        return 1;
    }

    // ── 5. Phase 2: serial tile rasterization ───────────────────────────
    // 1:1 serial replica of PglTileScheduler::ProcessTiles(): same Morton
    // order, same per-tile RasterizeTile() call.  The firmware scheduler
    // itself is not compilable here (RP2350 multicore FIFO); on hardware the
    // two cores write disjoint tiles, so the serial pass is equivalent.
    for (uint32_t idx = 0; idx < TileConfig::TILE_COUNT; ++idx) {
        uint8_t  linearId = TileConfig::MORTON_ORDER[idx];
        uint16_t tileCol  = linearId % TileConfig::COLS;
        uint16_t tileRow  = linearId / TileConfig::COLS;
        rasterizer.RasterizeTile(backBuffer, zBuffer,
                                 tileCol, tileRow,
                                 TileConfig::TILE_W, TileConfig::TILE_H);
    }

    // ── 6. Screenspace post-processing (no active shaders → pass-through)
    ScreenspaceShaders::ApplyShaders(backBuffer, zBuffer, W, H,
                                     &sceneState, 0.0f);

    // (2D compositing skipped: drawCmd2DCount == 0 && activeLayerCount == 0,
    //  same guard as gpu_core.cpp.)

    // ── 7. Buffer swap (present) ────────────────────────────────────────
    uint16_t* tmp = frontBuffer;
    frontBuffer = backBuffer;
    backBuffer  = tmp;
    CommandParser::UpdateFramebufferPtrs(frontBuffer, backBuffer);

    // ── 8. Dump + self-check ────────────────────────────────────────────
    uint32_t checksum = Fnv1a(frontBuffer, GpuConfig::FRAMEBUF_SIZE);
    std::printf("[sim] Frame FNV-1a checksum: 0x%08X\n", checksum);

    if (!WritePPM(ppmPath, frontBuffer, W, H)) return 1;
    std::printf("[sim] Wrote %s\n", ppmPath);

    PrintAsciiPreview(frontBuffer, W, H);

    if (!CheckFramebuffer(frontBuffer, W, H)) {
        std::printf("=== SIM RESULT: FAIL ===\n");
        return 1;
    }
    std::printf("=== SIM RESULT: PASS ===\n");
    return 0;
}
