/**
 * @file sim_main.cpp
 * @brief Desktop soft-GPU simulator — milestone A5-2 (benchmark scene suite).
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
 *     → ScreenspaceShaders::ApplyShaders (real; PSB VM for programmable shaders)
 *     → 2D draw queue + layer compositing (1:1 replica of the gpu_core.cpp
 *                                    frame path — see note below)
 *     → PPM dump + content self-check + FNV-1a checksum → exit 0/1
 *
 * A5-2 additions (see docs/OPTIMIZATION_PLAN.md §2):
 *   - Scene registry: one encoded frame set + one rendered PPM per benchmark
 *     scene.  Scenes B1–B6 are the frozen pixel-regression gate for all
 *     future firmware optimizations (F-01…, S-01…, V9 features).
 *   - Per-stage wall-clock timing (std::chrono): parse / PrepareFrame /
 *     tile raster / screenspace / 2D+composite / total — the sim-side analog
 *     of the firmware's DWT PerfCounters.
 *   - Golden-frame harness: sim/run_golden.sh compares rendered frames
 *     against sim/golden/<scene>.ppm (pixel-identical by default).
 *
 * The serial tile loop replicates PglTileScheduler::ProcessTiles()
 * (src/scheduler/pgl_tile_scheduler.cpp) 1:1.  That .cpp is the only render-path
 * file that cannot compile on desktop (RP2350 multicore FIFO), so the sim
 * re-uses its portable header (TileConfig::MORTON_ORDER / COLS / ROWS) and
 * re-implements the 6-line dispatch loop with the atomic counter replaced by a
 * plain for-loop.  On the real hardware both cores write disjoint tiles, so the
 * serial result is bit-identical to the dual-core result.
 *
 * The 2D frame path (Process2DDrawQueue + CompositeLayers) is a verbatim
 * replica of the static functions in src/gpu_core.cpp (lines ~140–299) —
 * gpu_core.cpp itself is not desktop-compilable (hardware drivers), and the
 * functions are file-static, so they are mirrored here.  Keep in sync if the
 * firmware compositing logic changes.
 *
 * What is shimmed (see sim_qspi_vram_stub.cpp):
 *   - QspiVramDriver  — all methods no-op/false (QSPI_VRAM_MODE = NONE semantics,
 *                       matching the current bench config in gpu_config.h).
 *   - FlashPersistManager — in-memory no-op (flash is hardware; not exercised
 *                       by the test scenes, but command_parser.cpp holds a static
 *                       instance so the symbols must resolve).
 *
 * Everything else in the render path is the unmodified firmware code.
 *
 * CLI:
 *   protogpu_sim                      → default scene ("cube") → sim/out/frame.ppm
 *   protogpu_sim OUT.ppm              → default scene → OUT.ppm
 *   protogpu_sim --scene NAME [OUT]   → named scene (default OUT sim/out/NAME.ppm)
 *   protogpu_sim --list               → list registered scenes
 *
 * Scenes are rendered ONE PER PROCESS: parser error counters are
 * process-lifetime latches and a fresh SceneState per process keeps scenes
 * hermetic (run_golden.sh loops over scenes).
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <chrono>
#include <filesystem>

// ─── ProtoGL host side (real wire-format encoder + PGLSL compiler) ──────────
#include <PglTypes.h>
#include <PglEncoder.h>
#include <PglShaderCompiler.h>   // B4: runtime PGLSL → PSB bytecode

// ─── Firmware side (real, unmodified) ───────────────────────────────────────
#include "gpu_config.h"
#include "scene_state.h"
#include "command_parser.h"
#include "math/pgl_math.h"
#include "render/rasterizer.h"
#include "render/rasterizer_2d.h"
#include "render/screenspace_effects.h"
#include "scheduler/pgl_tile_scheduler.h"   // TileConfig only — see header note
#include "memory/mem_qspi_vram.h"
#include "memory/mem_tier.h"
#include "memory/mem_pool.h"
#include "display/display_manager.h"

// ─── Real benchmark mesh data (firmware tree, read-only) ────────────────────
#include "selftest/teapot_mesh.h"

// B1 requirement check: the frozen benchmark spec (OPTIMIZATION_PLAN §2) pins
// the teapot at 587 verts / 1166 tris — fail the build if the header drifts.
static_assert(kTeapotVertCount == 587, "B1 expects 587 teapot vertices");
static_assert(kTeapotFaceCount == 1166, "B1 expects 1166 teapot triangles");

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

// ─── B3: textured-quad mesh + 16 procedural textures ────────────────────────
//
// One quad mesh (2 triangles, with UVs) drawn 16 times in a 4×4 grid, each
// cell 32×16 px, with a different 32×32 RGB565 texture per cell.  UVs cover
// the [0..0.5]² quadrant of the texture (16×16 texels), so on-screen each
// texel spans 2×1 px — MAGNIFIED nearest-neighbour sampling with clearly
// visible texel blocks.  The future V9 bilinear filtering (G6) will diff
// visibly against this golden.

static constexpr uint16_t kB3TexSize = 32;   // 32×32 RGB565 = 2 KB per texture
static constexpr uint8_t  kB3TexCount = 16;  // 16 × 2 KB = 32 KB total

static const PglVec3 kQuadVerts[] = {
    { -0.5f, -0.5f, 0.0f },   // 0 top-left
    {  0.5f, -0.5f, 0.0f },   // 1 top-right
    {  0.5f,  0.5f, 0.0f },   // 2 bottom-right
    { -0.5f,  0.5f, 0.0f },   // 3 bottom-left
};
static const PglIndex3 kQuadIndices[] = { { 0, 1, 2 }, { 0, 2, 3 } };
static const PglVec2 kQuadUVs[] = {
    { 0.0f, 0.0f }, { 0.5f, 0.0f }, { 0.5f, 0.5f }, { 0.0f, 0.5f },
};
static const PglIndex3 kQuadUVIndices[] = { { 0, 1, 2 }, { 0, 2, 3 } };

// Bold, fully deterministic texture patterns (no rand(), no time).  Every
// texel is non-black so the 4×4 grid gives ~100% non-background coverage.
static const uint16_t kB3Colors[8] = {
    0xF800,  // red
    0x07E0,  // green
    0x001F,  // blue
    0xFFE0,  // yellow
    0xF81F,  // magenta
    0x07FF,  // cyan
    0xFC00,  // orange
    0xFFFF,  // white
};

static uint16_t B3Texel(int i, int x, int y) {
    const uint16_t cA = kB3Colors[i % 8];
    const uint16_t cB = kB3Colors[(i + 3) % 8];
    switch (i % 8) {
        case 0: return ((x ^ y) & 4) ? cA : cB;                 // 4px checker
        case 1: return (y & 4) ? cA : cB;                       // h-stripes
        case 2: return (x & 4) ? cA : cB;                       // v-stripes
        case 3: return (((x + y) >> 2) & 1) ? cA : cB;          // diagonal
        case 4: { int dx = x - 16, dy = y - 16;
                  return ((dx * dx + dy * dy) & 64) ? cA : cB; } // rings
        case 5: return ((x * y) & 8) ? cA : cB;                 // scatter
        case 6: return (((x >> 1) + (y >> 1)) & 2) ? cA : cB;   // 2px checker
        default: return (x == y || x + y == 31) ? cA : cB;      // X
    }
}

static uint16_t b3Textures[kB3TexCount][kB3TexSize * kB3TexSize];
static bool b3TexturesReady = false;

static void B3InitTextures() {
    if (b3TexturesReady) return;
    for (int i = 0; i < kB3TexCount; ++i) {
        for (int y = 0; y < kB3TexSize; ++y) {
            for (int x = 0; x < kB3TexSize; ++x) {
                b3Textures[i][y * kB3TexSize + x] = B3Texel(i, x, y);
            }
        }
    }
    b3TexturesReady = true;
}

// ─── B4: PGLSL post-FX sources (compiled at runtime, see EncodeB4) ──────────
// These are VERBATIM copies of the stock ProtoGL samples
//   ../ProtoGL/shaders/invert.pglsl
//   ../ProtoGL/shaders/gamma.pglsl
//   ../ProtoGL/shaders/vignette.pglsl
// embedded as string literals (the sim's cwd varies between build_sim.sh and
// run_golden.sh, so runtime file loading would be fragile; the stock files
// themselves are compile-gated by ProtoGL/tests/syntax_check/
// run_shader_compile.sh). They compile now that PglShaderCompiler's register
// allocator frees expression temporaries (LIFO temp stack, 2026-07-20) —
// before that fix all 8 stock samples failed with "register allocation
// overflow". Chain: invert (SUB) → gamma (POW) → vignette (LEN2/MIX); all
// three sample u_framebuffer (TEX2D), exercising the PSB VM's scratch-copy
// path on every pixel.

// stock ProtoGL/shaders/invert.pglsl (no user uniform; intensity mixes)
static const char kPglslInvert[] = R"pglsl(
void main() {
    vec2 uv = gl_FragCoord.xy / u_resolution;
    vec4 color = texture2D(u_framebuffer, uv);
    gl_FragColor = vec4(vec3(1.0) - color.rgb, 1.0);
}
)pglsl";

// stock ProtoGL/shaders/gamma.pglsl (uniform float u_gamma)
static const char kPglslGamma[] = R"pglsl(
uniform float u_gamma;

void main() {
    vec2 uv = gl_FragCoord.xy / u_resolution;
    vec4 color = texture2D(u_framebuffer, uv);
    vec3 corrected = vec3(
        pow(color.r, u_gamma),
        pow(color.g, u_gamma),
        pow(color.b, u_gamma)
    );
    gl_FragColor = vec4(corrected, 1.0);
}
)pglsl";

// stock ProtoGL/shaders/vignette.pglsl (uniform float u_strength)
static const char kPglslVignette[] = R"pglsl(
uniform float u_strength;

void main() {
    vec2 uv = gl_FragCoord.xy / u_resolution;
    vec2 center = vec2(0.5, 0.5);
    float dist = length(uv - center) * 1.414;
    float vignette = mix(1.0, 1.0 - dist * dist, u_strength);

    vec4 color = texture2D(u_framebuffer, uv);
    gl_FragColor = vec4(color.rgb * vignette, 1.0);
}
)pglsl";

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

// 32 KB — matches the firmware's SPI_RING_BUFFER_SIZE; the B1 teapot
// CreateMesh command alone is ~14 KB.
static uint8_t         cmdBuffer[32 * 1024];

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

// ─── FNV-1a (frame checksum for golden-reference comparisons) ───────────────

static uint32_t Fnv1a(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t hsh = 0x811c9dc5u;
    for (size_t i = 0; i < len; ++i) { hsh ^= p[i]; hsh *= 0x01000193u; }
    return hsh;
}

// ─── Per-stage timing (the optimization baseline) ───────────────────────────
// Wall-clock analog of the firmware's DWT PerfCounters
// (docs/OPTIMIZATION_PLAN.md §2).  Not part of the golden gate — reported only.

struct StageTiming {
    double parseMs       = 0.0;
    double prepareMs     = 0.0;
    double rasterMs      = 0.0;
    double screenspaceMs = 0.0;
    double draw2dMs      = 0.0;   // 2D draw queue + layer compositing
    double totalMs       = 0.0;   // parse → present
};

using Clock = std::chrono::steady_clock;

static double MsSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// ═══════════════════════════════════════════════════════════════════════════
// 2D frame path — verbatim replica of the static helpers in gpu_core.cpp
// (M12: execute queued 2D draw commands, then composite layers over the 3D
// back buffer).  Mirrored because gpu_core.cpp is not desktop-compilable and
// the functions are file-static.  KEEP IN SYNC with src/gpu_core.cpp.
// ═══════════════════════════════════════════════════════════════════════════

/// Process the queued 2D draw commands, dispatching each to Rasterizer2D.
static void Process2DDrawQueue(SceneState* scene) {
    if (scene->drawCmd2DCount == 0) return;

    for (uint16_t i = 0; i < scene->drawCmd2DCount; ++i) {
        const DrawCmd2D& cmd = scene->drawCmds2D[i];

        // Use the explicit layerId field set during enqueue
        uint8_t layerId = cmd.layerId;

        if (layerId >= PGL_MAX_LAYERS || !scene->layers[layerId].active ||
            !scene->layers[layerId].pixels) {
            continue;
        }

        const LayerSlot& layer = scene->layers[layerId];
        Rasterizer2D::Target target;
        target.pixels = layer.pixels;
        target.width  = layer.width;
        target.height = layer.height;

        switch (cmd.type) {
            case DRAW_CMD_2D_RECT:
                Rasterizer2D::DrawRect(target, cmd.rect.x, cmd.rect.y,
                                       cmd.rect.w, cmd.rect.h,
                                       cmd.rect.color, cmd.rect.filled != 0);
                break;
            case DRAW_CMD_2D_LINE:
                Rasterizer2D::DrawLine(target, cmd.line.x0, cmd.line.y0,
                                       cmd.line.x1, cmd.line.y1,
                                       cmd.line.color);
                break;
            case DRAW_CMD_2D_CIRCLE:
                Rasterizer2D::DrawCircle(target, cmd.circle.cx, cmd.circle.cy,
                                         cmd.circle.radius, cmd.circle.color,
                                         cmd.circle.filled != 0);
                break;
            case DRAW_CMD_2D_SPRITE: {
                // Look up texture in scene state for source pixels
                uint16_t texId = cmd.sprite.textureId;
                if (texId < GpuConfig::MAX_TEXTURES &&
                    scene->textures[texId].active &&
                    scene->textures[texId].pixels) {
                    Rasterizer2D::DrawSprite(
                        target, cmd.sprite.x, cmd.sprite.y,
                        reinterpret_cast<const uint16_t*>(scene->textures[texId].pixels),
                        scene->textures[texId].width,
                        scene->textures[texId].height,
                        (cmd.sprite.flags & PGL_SPRITE_FLIP_H) != 0,
                        (cmd.sprite.flags & PGL_SPRITE_FLIP_V) != 0);
                }
                break;
            }
            case DRAW_CMD_2D_CLEAR:
                Rasterizer2D::Clear(target, cmd.clear.color);
                break;
            case DRAW_CMD_2D_ROUNDED_RECT:
                Rasterizer2D::DrawRoundedRect(target,
                    cmd.roundedRect.x, cmd.roundedRect.y,
                    cmd.roundedRect.w, cmd.roundedRect.h,
                    cmd.roundedRect.radius, cmd.roundedRect.color,
                    cmd.roundedRect.filled != 0);
                break;
            case DRAW_CMD_2D_ARC:
                Rasterizer2D::DrawArc(target, cmd.arc.cx, cmd.arc.cy,
                                      cmd.arc.radius,
                                      cmd.arc.startAngleDeg, cmd.arc.endAngleDeg,
                                      cmd.arc.color);
                break;
            case DRAW_CMD_2D_TRIANGLE:
                Rasterizer2D::DrawTriangle(target,
                    cmd.triangle.x0, cmd.triangle.y0,
                    cmd.triangle.x1, cmd.triangle.y1,
                    cmd.triangle.x2, cmd.triangle.y2,
                    cmd.triangle.color);
                break;
        }
    }
}

/// Composite 2D layers over the 3D back buffer in ascending layer order.
/// Uses alpha blending based on each layer's blend mode and opacity.
static void CompositeLayers(uint16_t* backBuf, uint16_t panelW, uint16_t panelH,
                            SceneState* scene) {
    if (scene->activeLayerCount == 0) return;

    for (uint8_t layerId = 1; layerId < PGL_MAX_LAYERS; ++layerId) {
        const LayerSlot& layer = scene->layers[layerId];
        if (!layer.active || !layer.visible || !layer.pixels || layer.opacity == 0) {
            continue;
        }

        // Compute overlap region between layer (with offset) and back buffer
        int32_t srcStartX = 0, srcStartY = 0;
        int32_t dstStartX = layer.offsetX, dstStartY = layer.offsetY;
        int32_t copyW = layer.width, copyH = layer.height;

        // Left/top clipping
        if (dstStartX < 0) { srcStartX = -dstStartX; copyW += dstStartX; dstStartX = 0; }
        if (dstStartY < 0) { srcStartY = -dstStartY; copyH += dstStartY; dstStartY = 0; }
        // Right/bottom clipping
        if (dstStartX + copyW > panelW) copyW = panelW - dstStartX;
        if (dstStartY + copyH > panelH) copyH = panelH - dstStartY;

        if (copyW <= 0 || copyH <= 0) continue;

        // Fully opaque fast path — memcpy rows
        if (layer.opacity == 255 && layer.blendMode == PGL_LAYER_BLEND_ALPHA) {
            for (int32_t row = 0; row < copyH; ++row) {
                uint16_t* dst = &backBuf[(dstStartY + row) * panelW + dstStartX];
                const uint16_t* src = &layer.pixels[(srcStartY + row) * layer.width + srcStartX];
                std::memcpy(dst, src, copyW * sizeof(uint16_t));
            }
            continue;
        }

        // Per-pixel blending path
        const uint8_t alpha = layer.opacity;
        for (int32_t row = 0; row < copyH; ++row) {
            uint16_t* dst = &backBuf[(dstStartY + row) * panelW + dstStartX];
            const uint16_t* src = &layer.pixels[(srcStartY + row) * layer.width + srcStartX];

            for (int32_t col = 0; col < copyW; ++col) {
                uint16_t srcPx = src[col];
                uint16_t dstPx = dst[col];

                switch (layer.blendMode) {
                    case PGL_LAYER_BLEND_ALPHA:
                    default:
                        dst[col] = Rasterizer2D::BlendRGB565(srcPx, dstPx, alpha);
                        break;

                    case PGL_LAYER_BLEND_ADDITIVE: {
                        // Additive: clamp(src + dst)
                        uint16_t sr = (srcPx >> 11) & 0x1F, sg = (srcPx >> 5) & 0x3F, sb = srcPx & 0x1F;
                        uint16_t dr = (dstPx >> 11) & 0x1F, dg = (dstPx >> 5) & 0x3F, db = dstPx & 0x1F;
                        sr = (sr * alpha) >> 8; sg = (sg * alpha) >> 8; sb = (sb * alpha) >> 8;
                        uint16_t r = (sr + dr > 31) ? 31 : sr + dr;
                        uint16_t g = (sg + dg > 63) ? 63 : sg + dg;
                        uint16_t b = (sb + db > 31) ? 31 : sb + db;
                        dst[col] = (r << 11) | (g << 5) | b;
                        break;
                    }
                    case PGL_LAYER_BLEND_MULTIPLY: {
                        // Multiply: (src * dst) / max_channel
                        uint16_t sr = (srcPx >> 11) & 0x1F, sg = (srcPx >> 5) & 0x3F, sb = srcPx & 0x1F;
                        uint16_t dr = (dstPx >> 11) & 0x1F, dg = (dstPx >> 5) & 0x3F, db = dstPx & 0x1F;
                        uint16_t r = (sr * dr) / 31;
                        uint16_t g = (sg * dg) / 63;
                        uint16_t b = (sb * db) / 31;
                        // Apply opacity as lerp between result and original dst
                        r = dr + (((int16_t)r - dr) * alpha >> 8);
                        g = dg + (((int16_t)g - dg) * alpha >> 8);
                        b = db + (((int16_t)b - db) * alpha >> 8);
                        dst[col] = (r << 11) | (g << 5) | b;
                        break;
                    }
                }
            }
        }
    }
}

// ─── Scene encoding helpers (host side, real PglEncoder) ────────────────────

/// encode() error sentinel (0 is a valid "no more frames" return).
static constexpr size_t kEncodeError = ~static_cast<size_t>(0);

static const PglQuat kIdentityQuat = { 1.0f, 0.0f, 0.0f, 0.0f };

/// Warm directional-light material (shared by cube / B1 / B2 / B4 scenes).
static void EncodeWarmLightMaterial(PglEncoder& enc, uint16_t materialId) {
    PglParamLight light{};
    light.lightDirX = 0.5f;  light.lightDirY = 0.8f;  light.lightDirZ = 0.6f;
    light.ambientR  = 40;    light.ambientG  = 15;    light.ambientB  = 10;
    light.diffuseR  = 255;   light.diffuseG  = 190;   light.diffuseB  = 80;
    enc.CreateMaterial(materialId, PGL_MAT_LIGHT, PGL_BLEND_BASE,
                       &light, sizeof(light));
}

/// Perspective camera at (0,0,z), identity rotation (looks toward +Z).
static void EncodePerspectiveCamera(PglEncoder& enc, uint8_t cameraId, float z) {
    enc.SetCamera(cameraId, 0,
                  { 0.0f, 0.0f, z }, kIdentityQuat,
                  { 1.0f, 1.0f, 1.0f }, kIdentityQuat, kIdentityQuat, false);
}

/// Gentle fixed tilt (same angles as the A5-1 cube scene).
static PglQuat GentleTilt() {
    const float halfY = 0.30f, halfX = 0.175f;
    PglQuat qy  = { cosf(halfY), 0.0f, sinf(halfY), 0.0f };
    PglQuat qx  = { cosf(halfX), sinf(halfX), 0.0f, 0.0f };
    return PglMath::QuatMul(qy, qx);
}

/// Cube mesh 0 + light material 0 + one draw call (shared by cube/B2/B4).
static void EncodeCubeDraw(PglEncoder& enc, float scale) {
    enc.DrawObject(0, 0,
                   { 0.0f, 0.0f, 0.0f }, GentleTilt(),
                   { scale, scale, scale },
                   kIdentityQuat, kIdentityQuat,
                   { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                   true);
}

// ─── Scene: cube (A5-1 legacy default) ──────────────────────────────────────

static size_t EncodeCubeScene(uint8_t* buf, size_t capacity, uint8_t frameIndex) {
    if (frameIndex > 0) return 0;   // single-frame scene
    PglEncoder enc(buf, capacity);

    enc.BeginFrame(1, 16666);

    // Mesh 0: cube (no UV).
    enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);

    // Material 0: warm directional light (front-lit variant of the self-test's).
    EncodeWarmLightMaterial(enc, 0);

    // Camera 0: perspective at z=-5, identity rotation (looks toward +Z).
    EncodePerspectiveCamera(enc, 0, -5.0f);

    // Draw call: cube at origin, scale 2.5, fixed tilt (0.6 rad Y + 0.35 rad X)
    // so three faces are visible with distinct light shading.
    EncodeCubeDraw(enc, 2.5f);

    enc.EndFrame();

    if (enc.HasOverflow()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Scene B1: teapot (3D-heavy) ────────────────────────────────────────────
//
// Real Utah teapot data (src/selftest/teapot_mesh.h, 587 verts / 1166 tris),
// LIGHT material, drawn once.  The camera is CLOSE (z=-1.1, teapot scale 0.6,
// gentle tilt): the front of the pot straddles the view-space near plane —
// 22 of 1166 triangles have at least one vertex behind the camera (measured
// with the exact scene transform; see the A5-2 report).
//
// TODAY'S BEHAVIOR (pinned by this golden): near-plane crossing is NOT
// clipped and NOT culled.  PglMath::PerspectiveProject clamps z to 0.001
// before writing outZ, so the rasterizer's per-triangle "near-plane cull"
// (za <= 0 at rasterizer.cpp:950) is DEAD CODE — it can never fire.  Instead
// every behind-camera vertex projects to view.x·64000+64 (±~64k px); the
// screen-AABB frustum cull drops fully-off-screen triangles, but crossing
// triangles survive with their AABB clamped to the panel and rasterize as
// GIANT FLAT SLIVERS whose interpolated z (~0.001) beats the real teapot in
// the Z test — they paint over it as the large flat regions visible in this
// golden.  The future V9 near-plane CLIP (G5) will rasterize the in-front
// slivers of those same triangles instead, producing an obvious golden diff
// across exactly these wash regions.

static size_t EncodeB1Teapot(uint8_t* buf, size_t capacity, uint8_t frameIndex) {
    if (frameIndex > 0) return 0;   // single-frame scene
    PglEncoder enc(buf, capacity);

    enc.BeginFrame(1, 16666);

    // Mesh 0: Utah teapot (no UV).
    enc.CreateMesh(0, kTeapotVerts, kTeapotVertCount,
                   kTeapotIndices, kTeapotFaceCount);

    // Material 0: warm directional light (same as the cube scene).
    EncodeWarmLightMaterial(enc, 0);

    // Camera 0: perspective, CLOSE to the origin so the teapot's near side
    // crosses the view-space near plane (see comment block above).
    EncodePerspectiveCamera(enc, 0, -1.1f);

    // Draw call: teapot at origin, scale 0.6, gentle tilt.
    enc.DrawObject(0, 0,
                   { 0.0f, 0.0f, 0.0f }, GentleTilt(),
                   { 0.6f, 0.6f, 0.6f },
                   kIdentityQuat, kIdentityQuat,
                   { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                   true);

    enc.EndFrame();

    if (enc.HasOverflow()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Scene B2: 2D layer storm ───────────────────────────────────────────────
//
// 3D cube background + two compositing layers exercising EVERY 2D primitive
// opcode implemented by the parser (0xA0–0xAC range):
//   LAYER_CREATE (×2), LAYER_SET_PROPS, LAYER_CLEAR (×2),
//   DRAW_RECT_2D (filled + outline), DRAW_LINE_2D (×2), DRAW_CIRCLE_2D
//   (filled + outline), DRAW_ROUNDED_RECT (filled + outline), DRAW_ARC,
//   DRAW_TRIANGLE_2D, DRAW_SPRITE (with FLIP_H; texture 0 = 16×16 checker).
// (DRAW_TEXT 0xA7 and DRAW_SPRITE_BATCH 0xA8 have no parser handlers — skipped.)
// Layer 1: opaque ALPHA HUD panel (56×64 at offset x=72) — fast memcpy path.
// Layer 2: full-frame ADDITIVE glow at opacity 90 — per-pixel blend path.

// 16×16 checker texture for the sprite (generated, deterministic).
static uint16_t b2SpriteTex[16 * 16];
static bool     b2SpriteTexReady = false;

static void B2InitSpriteTexture() {
    if (b2SpriteTexReady) return;
    for (int y = 0; y < 16; ++y) {
        for (int x = 0; x < 16; ++x) {
            b2SpriteTex[y * 16 + x] =
                (((x ^ y) >> 2) & 1) ? 0x07FF /*cyan*/ : 0xFFFF /*white*/;
        }
    }
    b2SpriteTexReady = true;
}

static size_t EncodeB2LayerStorm(uint8_t* buf, size_t capacity, uint8_t frameIndex) {
    if (frameIndex > 0) return 0;   // single-frame scene
    B2InitSpriteTexture();
    PglEncoder enc(buf, capacity);

    enc.BeginFrame(1, 16666);

    // ── 3D background: cube at scale 2.0 (sits left-of-centre) ──
    enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);
    EncodeWarmLightMaterial(enc, 0);
    EncodePerspectiveCamera(enc, 0, -5.0f);
    EncodeCubeDraw(enc, 2.0f);

    // ── Texture 0: 16×16 checker (sprite source) ──
    enc.CreateTexture(0, 16, 16, PGL_TEX_RGB565, b2SpriteTex);

    // ── Layer 1: 56×64 opaque ALPHA HUD panel, offset to x=72 ──
    enc.LayerCreate(1, 56, 64, 0, PGL_LAYER_BLEND_ALPHA, 255);
    enc.LayerSetProps(1, 255, PGL_LAYER_BLEND_ALPHA, 72, 0);
    enc.LayerClear(1, 0x0841);   // dark navy-grey panel background

    // Every implemented 2D primitive on layer 1 (layer-local coords):
    enc.DrawRect2D(1, 2, 2, 20, 12, 0xF800, true);          // filled red rect
    enc.DrawRect2D(1, 26, 2, 26, 12, 0xFFE0, false);        // yellow outline rect
    enc.DrawLine2D(1, 2, 18, 52, 26, 0x07E0);               // green diagonal
    enc.DrawLine2D(1, 52, 18, 2, 26, 0x07FF);               // cyan anti-diagonal
    enc.DrawCircle2D(1, 14, 38, 9, 0xF81F, true);           // filled magenta circle
    enc.DrawCircle2D(1, 40, 38, 9, 0xFFFF, false);          // white outline circle
    enc.DrawRoundedRect(1, 2, 50, 24, 12, 4, 0xFC00, true); // filled orange rounded
    enc.DrawRoundedRect(1, 30, 50, 24, 12, 4, 0x07E0, false); // green outline rounded
    enc.DrawArc(1, 28, 32, 14, 200, 340, 0x001F);           // blue arc segment
    enc.DrawTriangle2D(1, 44, 60, 52, 44, 36, 52, 0xFFE0);  // yellow triangle
    enc.DrawSprite(1, 44, 2, 0, PGL_SPRITE_FLIP_H);         // checker sprite, H-flip

    // ── Layer 2: full-frame ADDITIVE glow at opacity 90 ──
    enc.LayerCreate(2, 128, 64, 0, PGL_LAYER_BLEND_ADDITIVE, 90);
    enc.LayerClear(2, 0x0000);   // black = additive no-op background
    enc.DrawLine2D(2, 0, 0, 127, 63, 0x4000);               // dim red diagonal
    enc.DrawArc(2, 64, 32, 28, 0, 180, 0x0400);             // dim green arc
    enc.DrawRect2D(2, 1, 1, 126, 62, 0x0010, false);        // dim blue frame border

    enc.EndFrame();

    if (enc.HasOverflow()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Scene B3: texture-heavy ────────────────────────────────────────────────
//
// 16 textures (32×32 RGB565, procedural — 32 KB total, fits the 112 KB scene
// heap cap), one quad mesh with UVs drawn 16 times in a 4×4 grid (each cell
// 32×16 px), each with its own PGL_MAT_IMAGE material.  UVs span the
// [0..0.5]² texel quadrant → MAGNIFIED sampling (2×1 px texel blocks) so the
// future V9 bilinear filtering (G6) diffs visibly against this golden.
//
// MULTI-FRAME: 16 texture uploads (~33 KB) do not fit one 32 KB wire frame
// (the firmware's SPI ring is 32 KB), so — like a real host — the scene
// streams resources across frames: frame 1 uploads textures 0–7, frame 2
// textures 8–15, frame 3 creates mesh/materials/camera and issues the 16
// draw calls.  Resources persist across frames (SceneState::BeginFrame only
// clears the draw list); the rendered benchmark image is the LAST frame.

static size_t EncodeB3Textures(uint8_t* buf, size_t capacity, uint8_t frameIndex) {
    B3InitTextures();
    PglEncoder enc(buf, capacity);

    enc.BeginFrame(static_cast<uint32_t>(frameIndex) + 1, 16666);

    if (frameIndex == 0 || frameIndex == 1) {
        // Texture upload frames: 8 × (2 KB + header) ≈ 16.5 KB each.
        const uint16_t base = frameIndex * 8;
        for (uint16_t i = base; i < base + 8; ++i) {
            enc.CreateTexture(i, kB3TexSize, kB3TexSize, PGL_TEX_RGB565,
                              b3Textures[i]);
        }
    } else if (frameIndex == 2) {
        // Quad mesh 0 with UVs.
        enc.CreateMesh(0, kQuadVerts, 4, kQuadIndices, 2,
                       true, kQuadUVs, 4, kQuadUVIndices);

        // 16 image materials, ids 0..15 (textureId i, scale 1, offset 0).
        for (uint16_t i = 0; i < kB3TexCount; ++i) {
            PglParamImage img{};
            img.textureId = i;
            img.offsetX = 0.0f; img.offsetY = 0.0f;
            img.scaleX  = 1.0f; img.scaleY  = 1.0f;
            enc.CreateMaterial(i, PGL_MAT_IMAGE, PGL_BLEND_BASE, &img, sizeof(img));
        }

        EncodePerspectiveCamera(enc, 0, -5.0f);

        // 4×4 grid of quads.  Camera z=-5, fovFactor = W/2 = 64 → a world unit
        // at z=0 spans 64/5 = 12.8 px.  Cell 32×16 px ↔ 2.5×1.25 world units.
        for (uint8_t r = 0; r < 4; ++r) {
            for (uint8_t c = 0; c < 4; ++c) {
                const uint8_t i = r * 4 + c;
                const float wx = (c * 32.0f - 48.0f) / 12.8f;   // cell centre x
                const float wy = (r * 16.0f - 24.0f) / 12.8f;   // cell centre y
                enc.DrawObject(0, i,
                               { wx, wy, 0.0f }, kIdentityQuat,
                               { 2.5f, 1.25f, 1.0f },
                               kIdentityQuat, kIdentityQuat,
                               { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                               true);
            }
        }
    } else {
        return 0;   // no more frames
    }

    enc.EndFrame();

    if (enc.HasOverflow()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Scene B4: PSB post-FX chain (3 programs) ───────────────────────────────
//
// 3 PGLSL shaders compiled AT RUNTIME with ProtoGL's header-only compiler
// (PglShaderCompiler.h), uploaded via CREATE_SHADER_PROGRAM, bound to the
// three shader slots of camera 0 with distinct uniforms (SET_SHADER_UNIFORM).
// Chain order (ApplyShaders iterates slots in order):
//   invert (SUB) → gamma (POW) → vignette (LEN2/MIX).
// The sources are the STOCK ProtoGL shaders (verbatim copies, see the note
// above kPglslInvert) — before the compiler's register-allocator fix they
// failed with "register allocation overflow". All three sample u_framebuffer
// (TEX2D), exercising the PSB VM's scratch-copy path on every pixel.
// Background: the standard lit cube.

struct B4Shader {
    const char* name;
    const char* source;
    float       uniformValue;   // user uniform (slot PSB_USER_UNIFORM_START)
    float       intensity;      // slot mix factor (1.0 = full effect)
};

static const B4Shader kB4Shaders[3] = {
    { "invert",   kPglslInvert,   0.0f, 1.0f },   // no user uniform (unused)
    { "gamma",    kPglslGamma,    2.2f, 1.0f },   // u_gamma
    { "vignette", kPglslVignette, 0.7f, 1.0f },   // u_strength
};

static size_t EncodeB4PsbPostFx(uint8_t* buf, size_t capacity, uint8_t frameIndex) {
    if (frameIndex > 0) return 0;   // single-frame scene
    PglEncoder enc(buf, capacity);

    enc.BeginFrame(1, 16666);

    // Compile + upload the 3 shader programs (runtime PGLSL compile).
    for (uint8_t i = 0; i < 3; ++i) {
        auto res = PglShaderCompiler::Compile(kB4Shaders[i].source,
                                              std::strlen(kB4Shaders[i].source));
        if (!res.success) {
            std::fprintf(stderr, "[sim] ERROR: PGLSL compile failed for %s: %s\n",
                         kB4Shaders[i].name, res.errorMsg);
            return kEncodeError;
        }
        std::printf("[sim] PGLSL %-9s compiled: %u bytes PSB\n",
                    kB4Shaders[i].name, res.bytecodeSize);
        enc.CreateShaderProgram(i, res.bytecode, res.bytecodeSize);
    }

    // Background scene: lit cube.
    enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);
    EncodeWarmLightMaterial(enc, 0);
    EncodePerspectiveCamera(enc, 0, -5.0f);
    EncodeCubeDraw(enc, 2.5f);

    // Bind the chain to camera 0's shader slots (in order) + set uniforms.
    for (uint8_t i = 0; i < 3; ++i) {
        enc.BindShaderProgram(0, i, i, kB4Shaders[i].intensity);
        enc.SetShaderUniform(i, PSB_USER_UNIFORM_START,
                             kB4Shaders[i].uniformValue);
    }

    enc.EndFrame();

    if (enc.HasOverflow()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Scene B6: empty frame (baseline overhead reference) ────────────────────

static size_t EncodeB6Empty(uint8_t* buf, size_t capacity, uint8_t frameIndex) {
    if (frameIndex > 0) return 0;   // single-frame scene
    PglEncoder enc(buf, capacity);
    enc.BeginFrame(1, 16666);
    enc.EndFrame();
    if (enc.HasOverflow()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Content self-checks ────────────────────────────────────────────────────

struct CheckResult { bool ok; uint32_t nonBlack; };

static uint32_t CountNonBlack(const uint16_t* fb, uint16_t w, uint16_t h) {
    uint32_t n = 0;
    for (uint32_t i = 0; i < static_cast<uint32_t>(w) * h; ++i) {
        if (fb[i] != 0x0000) ++n;
    }
    return n;
}

/// Bespoke A5-1 cube check: sane coverage, centered bbox, black corners.
static bool CheckCubeScene(const uint16_t* fb, uint16_t w, uint16_t h) {
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

/// Generic coverage check for benchmark scenes: non-background pixel count
/// within [minNonBlack, maxNonBlack] (maxNonBlack=0 → no upper bound).
static bool CheckCoverage(const uint16_t* fb, uint16_t w, uint16_t h,
                          uint32_t minNonBlack, uint32_t maxNonBlack) {
    uint32_t nonBlack = CountNonBlack(fb, w, h);
    const uint32_t total = static_cast<uint32_t>(w) * h;
    std::printf("[sim] Non-background pixels: %u / %u (%.1f%%)\n",
                nonBlack, total, 100.0 * nonBlack / total);
    bool ok = nonBlack >= minNonBlack &&
              (maxNonBlack == 0 || nonBlack <= maxNonBlack);
    std::printf("  [%s] coverage in [%u, %s]\n", ok ? " OK " : "FAIL",
                minNonBlack,
                maxNonBlack ? std::to_string(maxNonBlack).c_str() : "∞");
    return ok;
}

/// B6: the framebuffer must remain entirely background.
static bool CheckEmptyScene(const uint16_t* fb, uint16_t w, uint16_t h) {
    uint32_t nonBlack = CountNonBlack(fb, w, h);
    std::printf("[sim] Non-background pixels: %u / %u\n",
                nonBlack, static_cast<uint32_t>(w) * h);
    bool ok = (nonBlack == 0);
    std::printf("  [%s] framebuffer fully background (empty frame)\n",
                ok ? " OK " : "FAIL");
    return ok;
}

/// B2 extra check: the opaque HUD panel must show the filled red rect at
/// layer-local (8,8) → screen (72+8, 8) = (80,8) — exactly 0xF800 (layer 2's
/// additive pixels are black there, so the panel pixel is unmodified).
static bool CheckB2Scene(const uint16_t* fb, uint16_t w, uint16_t h) {
    if (!CheckCoverage(fb, w, h, 1500, 0)) return false;
    bool ok = fb[8 * w + 80] == 0xF800;
    std::printf("  [%s] HUD panel red-rect pixel (80,8) == 0xF800 (got 0x%04X)\n",
                ok ? " OK " : "FAIL", fb[8 * w + 80]);
    return ok;
}

// ─── Scene registry ─────────────────────────────────────────────────────────
// Each scene = one or more encoded frames + one rendered PPM.  B1–B6 are the
// frozen benchmark suite of docs/OPTIMIZATION_PLAN.md §2; "cube" is the A5-1
// legacy default kept as the build_sim.sh smoke scene.
//
// Multi-frame scenes: encode() is called with frameIndex 0,1,2,… and must
// return the encoded frame length, 0 when the scene has no more frames, or
// kEncodeError on failure.  Frames are parsed in order through the real
// parser (resources persist across frames, like a real host streaming
// uploads); the pipeline renders once after the LAST frame.

struct SceneDef {
    const char* name;             // registry key / golden filename stem
    const char* blurb;            // one-line description
    size_t (*encode)(uint8_t* buf, size_t capacity, uint8_t frameIndex);
    uint16_t  expectedDrawCalls;  // parser-level draw call count (last frame)
    int32_t   triMin;             // projected-triangle bounds (−1/−1 = skip)
    int32_t   triMax;
    bool (*check)(const uint16_t* fb, uint16_t w, uint16_t h);
};

static const SceneDef kScenes[] = {
    { "cube",         "A5-1 legacy cube (default smoke scene)",
      EncodeCubeScene, 1, kCubeFaceCount, kCubeFaceCount, CheckCubeScene },

    { "B1_teapot",    "B1 teapot 587v/1166t, LIGHT, near-plane crossing",
      EncodeB1Teapot, 1, 100, kTeapotFaceCount,
      [](const uint16_t* fb, uint16_t w, uint16_t h) {
          return CheckCoverage(fb, w, h, 1500, 0);
      } },

    { "B2_2d_layers", "B2 2D layer storm: all primitives + compositing over 3D",
      EncodeB2LayerStorm, 1, kCubeFaceCount, kCubeFaceCount, CheckB2Scene },

    { "B3_textures",  "B3 texture-heavy: 16×32×32 RGB565, magnified sampling",
      EncodeB3Textures, 16, 32, 32,
      [](const uint16_t* fb, uint16_t w, uint16_t h) {
          return CheckCoverage(fb, w, h, 7000, 0);
      } },

    { "B4_psb_postfx", "B4 PSB post-FX chain: invert→gamma→vignette (stock)",
      EncodeB4PsbPostFx, 1, kCubeFaceCount, kCubeFaceCount,
      [](const uint16_t* fb, uint16_t w, uint16_t h) {
          return CheckCoverage(fb, w, h, 400, 0);
      } },

    // B5 LVGL widgets: PLACEHOLDER — the LVGL track hasn't reached L3 yet
    // (docs/OPTIMIZATION_PLAN.md §2).  Slot kept so adding it is trivial.
    { "B5_lvgl",      "B5 LVGL widgets (PLACEHOLDER — LVGL track not at L3)",
      nullptr, 0, -1, -1, nullptr },

    { "B6_empty",     "B6 empty frame (BeginFrame/EndFrame only, baseline)",
      EncodeB6Empty, 0, 0, 0, CheckEmptyScene },
};

static constexpr size_t kSceneCount = sizeof(kScenes) / sizeof(kScenes[0]);

static const SceneDef* FindScene(const char* name) {
    for (size_t i = 0; i < kSceneCount; ++i) {
        if (std::strcmp(kScenes[i].name, name) == 0) return &kScenes[i];
    }
    return nullptr;
}

// ─── Frame runner ───────────────────────────────────────────────────────────

static int RunScene(const SceneDef& scene, const char* ppmPath) {
    std::printf("=== RP2350-ProtoGPU desktop soft-GPU sim (A5-2) ===\n");
    std::printf("[sim] Scene: %s — %s\n", scene.name, scene.blurb);
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

    StageTiming timing;
    const Clock::time_point frameStart = Clock::now();

    // ── 2+3. Encode + parse ALL frames of the scene through the real ────
    // PglEncoder / firmware CommandParser (sync + CRC).  Multi-frame scenes
    // stream resource uploads across frames like a real host; resources
    // persist in SceneState between frames (only the draw list / 2D queue
    // reset per frame).  Rendering happens once, after the last frame.
    uint8_t frameIdx = 0;
    for (;; ++frameIdx) {
        size_t frameLen = scene.encode(cmdBuffer, sizeof(cmdBuffer), frameIdx);
        if (frameLen == kEncodeError) return 1;
        if (frameLen == 0) break;
        std::printf("[sim] Encoded frame %u: %zu bytes\n",
                    static_cast<unsigned>(frameIdx + 1), frameLen);

        const Clock::time_point tParse = Clock::now();
        CommandParser::ParseResult result =
            CommandParser::Parse(cmdBuffer, static_cast<uint32_t>(frameLen), &sceneState);
        timing.parseMs += MsSince(tParse);
        std::printf("[sim] Parse result: %d, parser errors: %u (mask 0x%04X), "
                    "draw calls: %u\n",
                    static_cast<int>(result), CommandParser::GetParserErrorCount(),
                    CommandParser::GetParserErrorMask(), sceneState.drawCallCount);

        if (result != CommandParser::ParseResult::Ok) {
            std::fprintf(stderr, "[sim] FAIL: frame did not parse cleanly\n");
            return 1;
        }
        if (CommandParser::GetParserErrorCount() != 0) {
            std::fprintf(stderr, "[sim] FAIL: parser latched %u error(s)\n",
                         CommandParser::GetParserErrorCount());
            return 1;
        }
        if (frameIdx >= 7) {
            std::fprintf(stderr, "[sim] FAIL: scene exceeded 8 frames (registry bug)\n");
            return 1;
        }
    }
    if (frameIdx == 0) {
        std::fprintf(stderr, "[sim] FAIL: scene produced no frames\n");
        return 1;
    }
    if (sceneState.drawCallCount != scene.expectedDrawCalls) {
        std::fprintf(stderr, "[sim] FAIL: expected %u draw calls, got %u\n",
                     scene.expectedDrawCalls, sceneState.drawCallCount);
        return 1;
    }

    // ── 4. Phase 1: PrepareFrame (transform → project → QuadTree) ───────
    memTierManager.BeginFrame();
    const Clock::time_point tPrepare = Clock::now();
    rasterizer.PrepareFrame(&sceneState);
    timing.prepareMs = MsSince(tPrepare);
    if (rasterizer.IsFrameSkipped()) {
        std::fprintf(stderr, "[sim] FAIL: first frame unexpectedly skipped\n");
        return 1;
    }
    rasterizer.SetElapsedTime(0.0f);   // deterministic animated-material time
    std::printf("[sim] PrepareFrame: %u triangles projected\n",
                rasterizer.GetTriangleCount());
    if (scene.triMin >= 0 &&
        (rasterizer.GetTriangleCount() < static_cast<uint32_t>(scene.triMin) ||
         rasterizer.GetTriangleCount() > static_cast<uint32_t>(scene.triMax))) {
        std::fprintf(stderr, "[sim] FAIL: projected triangles %u outside [%d, %d]\n",
                     rasterizer.GetTriangleCount(), scene.triMin, scene.triMax);
        return 1;
    }

    // ── 5. Phase 2: serial tile rasterization ───────────────────────────
    // 1:1 serial replica of PglTileScheduler::ProcessTiles(): same Morton
    // order, same per-tile RasterizeTile() call.  The firmware scheduler
    // itself is not compilable here (RP2350 multicore FIFO); on hardware the
    // two cores write disjoint tiles, so the serial pass is equivalent.
    const Clock::time_point tRaster = Clock::now();
    for (uint32_t idx = 0; idx < TileConfig::TILE_COUNT; ++idx) {
        uint8_t  linearId = TileConfig::MORTON_ORDER[idx];
        uint16_t tileCol  = linearId % TileConfig::COLS;
        uint16_t tileRow  = linearId / TileConfig::COLS;
        rasterizer.RasterizeTile(backBuffer, zBuffer,
                                 tileCol, tileRow,
                                 TileConfig::TILE_W, TileConfig::TILE_H);
    }
    timing.rasterMs = MsSince(tRaster);

    // ── 6. Screenspace post-processing (PSB VM when programs are bound) ─
    const Clock::time_point tShaders = Clock::now();
    ScreenspaceShaders::ApplyShaders(backBuffer, zBuffer, W, H,
                                     &sceneState, 0.0f);
    timing.screenspaceMs = MsSince(tShaders);

    // ── 7. 2D layer rendering + compositing (M12; mirrors gpu_core.cpp) ─
    const Clock::time_point t2d = Clock::now();
    if (sceneState.drawCmd2DCount > 0 || sceneState.activeLayerCount > 0) {
        Process2DDrawQueue(&sceneState);
        CompositeLayers(backBuffer, W, H, &sceneState);
    }
    timing.draw2dMs = MsSince(t2d);

    // ── 8. Buffer swap (present) ────────────────────────────────────────
    uint16_t* tmp = frontBuffer;
    frontBuffer = backBuffer;
    backBuffer  = tmp;
    CommandParser::UpdateFramebufferPtrs(frontBuffer, backBuffer);

    timing.totalMs = MsSince(frameStart);

    // ── 9. Dump + self-check ────────────────────────────────────────────
    uint32_t checksum = Fnv1a(frontBuffer, GpuConfig::FRAMEBUF_SIZE);
    std::printf("[sim] Frame FNV-1a checksum: 0x%08X\n", checksum);

    if (!WritePPM(ppmPath, frontBuffer, W, H)) return 1;
    std::printf("[sim] Wrote %s\n", ppmPath);

    PrintAsciiPreview(frontBuffer, W, H);

    // Per-stage timing table (the optimization baseline; informational only).
    std::printf("[sim] ── Per-stage timing (wall clock) ─────────────\n");
    std::printf("[sim]   parse         : %8.3f ms\n", timing.parseMs);
    std::printf("[sim]   PrepareFrame  : %8.3f ms\n", timing.prepareMs);
    std::printf("[sim]   tile raster   : %8.3f ms\n", timing.rasterMs);
    std::printf("[sim]   screenspace   : %8.3f ms\n", timing.screenspaceMs);
    std::printf("[sim]   2D+composite  : %8.3f ms\n", timing.draw2dMs);
    std::printf("[sim]   total (encode→present): %8.3f ms\n", timing.totalMs);

    if (!scene.check(frontBuffer, W, H)) {
        std::printf("=== SIM RESULT: FAIL ===\n");
        return 1;
    }
    std::printf("=== SIM RESULT: PASS ===\n");
    return 0;
}

// ─── Main ───────────────────────────────────────────────────────────────────

static void PrintUsage(const char* argv0) {
    std::printf("Usage:\n"
                "  %s                        default scene (cube) → sim/out/frame.ppm\n"
                "  %s OUT.ppm                default scene → OUT.ppm\n"
                "  %s --scene NAME [OUT.ppm] render scene NAME (default sim/out/NAME.ppm)\n"
                "  %s --list                 list registered scenes\n",
                argv0, argv0, argv0, argv0);
}

int main(int argc, char** argv) {
    const char* sceneName = "cube";
    const char* ppmArg    = nullptr;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--list") == 0) {
            std::printf("Registered scenes (%zu):\n", kSceneCount);
            for (size_t s = 0; s < kSceneCount; ++s) {
                std::printf("  %-14s %s%s\n", kScenes[s].name, kScenes[s].blurb,
                            kScenes[s].encode ? "" : "  [SKIPPED — placeholder]");
            }
            return 0;
        }
        if (std::strcmp(argv[i], "--scene") == 0) {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "[sim] ERROR: --scene needs a name\n");
                PrintUsage(argv[0]);
                return 2;
            }
            sceneName = argv[++i];
            continue;
        }
        if (argv[i][0] == '-') {
            std::fprintf(stderr, "[sim] ERROR: unknown flag %s\n", argv[i]);
            PrintUsage(argv[0]);
            return 2;
        }
        ppmArg = argv[i];
    }

    const SceneDef* scene = FindScene(sceneName);
    if (!scene) {
        std::fprintf(stderr, "[sim] ERROR: unknown scene '%s'\n", sceneName);
        PrintUsage(argv[0]);
        return 2;
    }

    if (!scene->encode) {
        std::printf("[sim] Scene %s is a placeholder — SKIPPED (%s)\n",
                    scene->name, scene->blurb);
        return 0;
    }

    char ppmPathBuf[512];
    const char* ppmPath = ppmArg;
    if (!ppmPath) {
        if (std::strcmp(sceneName, "cube") == 0) {
            std::snprintf(ppmPathBuf, sizeof(ppmPathBuf), "sim/out/frame.ppm");
        } else {
            std::snprintf(ppmPathBuf, sizeof(ppmPathBuf), "sim/out/%s.ppm", sceneName);
        }
        ppmPath = ppmPathBuf;
    }

    return RunScene(*scene, ppmPath);
}
