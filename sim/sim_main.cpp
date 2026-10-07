/**
 * Native protocol9 renderer corpus: real PglEncoder -> CommandParser ->
 * FrameRenderer -> PglTileScheduler (control thread + native worker).
 *
 * Every normal batch renders before the next batch is admitted. Resource-only
 * batches mutate resources without rendering or replacing the visible image.
 * The CLI writes actual RGB565 pixels as PPM, reports live frame-engine timings,
 * and applies geometric/material content oracles. No hardware is simulated.
 * Historical goldens are diagnostic references, not permission to repin
 * intentional depth, edge-coverage, transparency, or target-space changes.
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
#include "render/frame_renderer.h"
#include "render/pgl_shader_vm.h"

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
//   ProtoGL/shaders/invert.pglsl
//   ProtoGL/shaders/gamma.pglsl
//   ProtoGL/shaders/vignette.pglsl
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
static PhaseScratch::DepthWorkspace zBuffer;

static SceneState sceneState;
static Rasterizer rasterizer;
static PglTileScheduler scheduler;
static FrameRenderer renderer(rasterizer, scheduler);

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

    if (enc.HasOverflow() || enc.HasInvalidCommand()) {
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
// True near-plane clipping, perspective depth, and exact edge coverage now run
// through the live engine; historical pixel equality is not assumed.

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

    if (enc.HasOverflow() || enc.HasInvalidCommand()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Scene B2: 2D layer storm ───────────────────────────────────────────────
//
// 3D cube background + two compositing layers exercising rectangle, line,
// circle, rounded rectangle, arc, triangle and flipped sprite commands.
// Clears execute before 3D; other 2D commands execute after camera effects.
// Layer 1 is an opaque HUD; layer 2 supplies additive glow.

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

    if (enc.HasOverflow() || enc.HasInvalidCommand()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Scene B3: texture-heavy ────────────────────────────────────────────────
//
// 16 textures (32x32 RGB565, procedural: 32 KiB total), one UV quad mesh
// drawn sixteen times in a 4x4 grid. UVs span the [0..0.5] texel quadrant
// for magnified nearest-neighbour sampling.
//
// Three bounded resource batches upload all sixteen textures; the final
// ordinary frame creates mesh/materials/camera and draws the full grid.

static size_t EncodeB3Textures(uint8_t* buf, size_t capacity, uint8_t frameIndex) {
    B3InitTextures();
    PglEncoder enc(buf, capacity);

    if(frameIndex<3)enc.BeginResourceBatch();else enc.BeginFrame(1,16666);

    if (frameIndex < 3) {
        const uint16_t base = frameIndex * 7;
        const uint16_t end = base+7 < kB3TexCount ? base+7 : kB3TexCount;
        for (uint16_t i = base; i < end; ++i) {
            enc.CreateTexture(i, kB3TexSize, kB3TexSize, PGL_TEX_RGB565,
                              b3Textures[i]);
        }
    } else if (frameIndex == 3) {
        // Quad mesh 0 with UVs.
        enc.CreateMesh(0, kQuadVerts, 4, kQuadIndices, 2,
                       true, kQuadUVs, 4, kQuadUVIndices);

        // 16 image materials, ids 0..15 (textureId i, scale 1, offset 0).
        // Explicit current image-material payload; default filter is nearest.
        for (uint16_t i = 0; i < kB3TexCount; ++i) {
            PglParamImage img{};
            img.textureId = i;
            img.offsetX = 0.0f; img.offsetY = 0.0f;
            img.scaleX  = 1.0f; img.scaleY  = 1.0f;
            enc.CreateMaterial(i, PGL_MAT_IMAGE, PGL_BLEND_BASE,
                               &img, sizeof(img));
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

    if(frameIndex<3)enc.EndResourceBatch();else enc.EndFrame();

    if (enc.HasOverflow() || enc.HasInvalidCommand()) {
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
// The full 8192-pixel chain costs 1,335,296 verified weighted operations.
// The finite firmware budget is 2 Mi operations; the image is not narrowed
// to make a resource gate pass.
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
    uint32_t weightedCost = 0;
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
        ShaderProgram program{};
        if (DecodeShaderProgram(res.bytecode, res.bytecodeSize, i, program) !=
            PglRuntime::Result::Ok) {
            std::fprintf(stderr, "[sim] ERROR: compiled shader verification failed\n");
            return kEncodeError;
        }
        weightedCost += program.weightedCost;
        enc.CreateShaderProgram(i, res.bytecode, res.bytecodeSize);
    }
    std::printf("[sim] Post-FX weighted work: %u ops/pixel, %u pixels\n",weightedCost,GpuConfig::FRAMEBUF_PIXELS);
    if (uint64_t(weightedCost) * GpuConfig::FRAMEBUF_PIXELS > GpuConfig::POSTFX_WORK_BUDGET) {
        std::fprintf(stderr, "[sim] ERROR: post-FX corpus exceeds weighted work budget\n");
        return kEncodeError;
    }

    // Background scene: lit cube.
    enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);
    EncodeWarmLightMaterial(enc, 0);
    EncodePerspectiveCamera(enc, 0, -5.0f);
    EncodeCubeDraw(enc, 2.5f);

    // Bind the chain to camera 0's shader slots (in order) + set uniforms.
    for (uint8_t i = 0; i < 3; ++i) {
        enc.BindShaderProgram(0, i, i, kB4Shaders[i].intensity);
        if (i != 0) enc.SetShaderUniform(i, PSB_USER_UNIFORM_START,
                                       kB4Shaders[i].uniformValue);
    }

    enc.EndFrame();

    if (enc.HasOverflow() || enc.HasInvalidCommand()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Scene B7: 3D alpha blending (V9/G4) ────────────────────────────────────
//
// One cube mesh drawn three times over the plain (black) background:
//   A — OPAQUE orange cube at mid depth (PGL_BLEND_BASE),
//   B — translucent CYAN cube (alpha 0.45) CLOSER to the camera, overlapping
//       A on screen: where it covers A the pixel is src·0.45 + A·0.55 (blend
//       correctness), where it extends past A it darkens over the background,
//   C — translucent MAGENTA cube (alpha 0.45) whose ENTIRE depth range sits
//       behind A's (near z 1.7 > A's far z 1.5): fully occluded where it
//       overlaps A's silhouette (Z interplay — the deferred translucent pass
//       Z-tests against the finished opaque depth buffer), visible as a clean
//       magenta-over-background strip where it peeks out past A's right edge.
// Both translucent materials are created with CreateMaterialAlpha() — the
// PGL_BLEND_ALPHA + appended-float-alpha wire convention (SIMPLE params are
// 3 B, so the param block is 7 B).  The golden pins the exact blend values:
// check() verifies B-over-A, B-over-background, C-over-background and the
// opaque occlusion pixel to the byte.
//
// (Note: this pipeline's winding convention keeps each cube's FAR faces —
// flat-coloured SIMPLE cubes still read as solid squares, and all depth
// relationships above are stated in world-z so they hold regardless.)

static size_t EncodeB7Alpha3D(uint8_t* buf, size_t capacity, uint8_t frameIndex) {
    if (frameIndex > 0) return 0;   // single-frame scene
    PglEncoder enc(buf, capacity);

    enc.BeginFrame(1, 16666);

    // Mesh 0: cube (no UV — SIMPLE materials).
    enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);

    // Material 0: opaque orange (blend BASE — untouched by the alpha path).
    PglParamSimple orange{};
    orange.r = 255; orange.g = 128; orange.b = 0;
    enc.CreateMaterial(0, PGL_MAT_SIMPLE, PGL_BLEND_BASE,
                       &orange, sizeof(orange));

    // Material 1: translucent cyan, alpha 0.45 (appended float on the wire).
    PglParamSimple cyan{};
    cyan.r = 0; cyan.g = 200; cyan.b = 255;
    enc.CreateMaterialAlpha(1, PGL_MAT_SIMPLE, &cyan, sizeof(cyan), 0.45f);

    // Material 2: translucent magenta, alpha 0.45.
    PglParamSimple magenta{};
    magenta.r = 255; magenta.g = 0; magenta.b = 200;
    enc.CreateMaterialAlpha(2, PGL_MAT_SIMPLE, &magenta, sizeof(magenta), 0.45f);

    EncodePerspectiveCamera(enc, 0, -5.0f);

    // Draw order is deliberately NOT back-to-front for the opaque cube —
    // the GPU's Z-buffer + deferred translucent pass resolves it (the
    // painter's-algorithm contract only concerns stacked translucents).
    enc.DrawObject(0, 0,                                // A: opaque, mid depth
                   { 0.3f, -0.1f, 0.3f }, kIdentityQuat,
                   { 2.4f, 2.4f, 2.4f },
                   kIdentityQuat, kIdentityQuat,
                   { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                   true);
    enc.DrawObject(0, 2,                                // C: translucent, BEHIND A
                   { 2.9f, -0.35f, 2.5f }, kIdentityQuat,  // (near z 1.7 > A far 1.5)
                   { 1.6f, 1.6f, 1.6f },
                   kIdentityQuat, kIdentityQuat,
                   { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                   true);
    enc.DrawObject(0, 1,                                // B: translucent, CLOSER
                   { -0.75f, 0.25f, -1.4f }, kIdentityQuat,
                   { 1.5f, 1.5f, 1.5f },
                   kIdentityQuat, kIdentityQuat,
                   { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                   true);

    enc.EndFrame();

    if (enc.HasOverflow() || enc.HasInvalidCommand()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Scene B8: bilinear texture filtering (V9/G6) ───────────────────────────
//
// The SAME 8×8 texture drawn twice side by side, heavily magnified (~7 px
// per texel): left half sampled NEAREST (material 0 — sent as the frozen
// 18-byte v8 PglParamImage), right half sampled BILINEAR (material 1 —
// grown 20-byte form, filterFlags bit0).  The texture is a 2×2-texel-cell
// red/blue checkerboard, so the golden itself shows crisp texel blocks on
// the left and smooth lerp ramps between cells on the right.  check()
// verifies the left half contains ONLY pure texel colours while the right
// half contains mixed (lerped) pixels.

// Full-range UVs for the B8 quads (B3's kQuadUVs span only [0..0.5]²).
static const PglVec2 kB8QuadUVs[] = {
    { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 1.0f }, { 0.0f, 1.0f },
};

static size_t EncodeB8Bilinear(uint8_t* buf, size_t capacity, uint8_t frameIndex) {
    if (frameIndex > 0) return 0;   // single-frame scene
    PglEncoder enc(buf, capacity);

    enc.BeginFrame(1, 16666);

    // Texture 0: 8×8 RGB565, 2×2-texel checker cells alternating red/blue.
    static uint16_t b8Tex[8 * 8];
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
            const int cx = x / 2, cy = y / 2;
            b8Tex[y * 8 + x] = ((cx ^ cy) & 1) ? 0x001F /*blue*/ : 0xF800 /*red*/;
        }
    }
    enc.CreateTexture(0, 8, 8, PGL_TEX_RGB565, b8Tex);

    // Quad mesh 0 with full-range UVs.
    enc.CreateMesh(0, kQuadVerts, 4, kQuadIndices, 2,
                   true, kB8QuadUVs, 4, kQuadUVIndices);

    // Material 0: current image-material payload with nearest filtering.
    PglParamImage img{};
    img.textureId = 0;
    img.offsetX = 0.0f; img.offsetY = 0.0f;
    img.scaleX  = 1.0f; img.scaleY  = 1.0f;
    enc.CreateMaterial(0, PGL_MAT_IMAGE, PGL_BLEND_BASE,
                       &img, sizeof(img));

    // Material 1: BILINEAR via the grown 20-byte form (filterFlags bit0).
    enc.CreateImageMaterial(1, PGL_BLEND_BASE, img, PGL_IMAGE_FILTER_BILINEAR);

    EncodePerspectiveCamera(enc, 0, -5.0f);

    // Two quads side by side, ~54 px each: camera z=-5 → 12.8 px/world-unit.
    enc.DrawObject(0, 0,                                // left: nearest
                   { -2.5f, 0.0f, 0.0f }, kIdentityQuat,
                   { 4.2f, 4.2f, 1.0f },
                   kIdentityQuat, kIdentityQuat,
                   { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                   true);
    enc.DrawObject(0, 1,                                // right: bilinear
                   { 2.5f, 0.0f, 0.0f }, kIdentityQuat,
                   { 4.2f, 4.2f, 1.0f },
                   kIdentityQuat, kIdentityQuat,
                   { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                   true);

    enc.EndFrame();

    if (enc.HasOverflow() || enc.HasInvalidCommand()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Scene B9: multi-camera + render-to-layer (V9 G3/G7) ────────────────────
//
// THREE active cameras over ONE shared draw list (cube + teapot):
//   cam0: z=-5,          → layer 0, viewport (0,0,64,64)   — LEFT half: cube
//   cam1: z=-3 (closer), → layer 0, viewport (64,0,64,64)  — RIGHT half: teapot
//   cam2: (-3.75,0,-5), → LAYER 3 (64×32), viewport (4,4,56,24) — "3D-in-UI"
// Each camera projects in its actual target's pixel space, with its declared
// scissor. Layer 3 composites at (32,16), and its title bar/outline draw after
// the 3D pass. Layer clearing happens before 3D, never erasing a later pass.
//
// check() pins: left/right content, in-window 3D content, the navy bar and
// white frame on top of the 3D layer, the black 3D background inside the
// viewport, and no bleed outside the window / viewport edges.

static size_t EncodeB9Multicam(uint8_t* buf, size_t capacity, uint8_t frameIndex) {
    if (frameIndex > 0) return 0;   // single-frame scene
    PglEncoder enc(buf, capacity);

    enc.BeginFrame(1, 16666);

    // Mesh 0: cube;  mesh 1: Utah teapot (real data, as B1).
    enc.CreateMesh(0, kCubeVerts, kCubeVertCount, kCubeIndices, kCubeFaceCount);
    enc.CreateMesh(1, kTeapotVerts, kTeapotVertCount,
                   kTeapotIndices, kTeapotFaceCount);

    // Material 0: warm directional light (shared by both draws).
    EncodeWarmLightMaterial(enc, 0);

    // Layer 3: 64×32 opaque window at offset (32,16) — the 3D-in-UI target.
    // MUST be created before cam2 targets it (the parser validates targets
    // against existing layers, fail-closed).
    enc.LayerCreate(3, 64, 32, 0, PGL_LAYER_BLEND_ALPHA, 255);
    enc.LayerSetProps(3, 255, PGL_LAYER_BLEND_ALPHA, 32, 16);

    // cam0: left half of the back buffer.
    EncodePerspectiveCamera(enc, 0, -5.0f);
    enc.SetCameraTarget(0, 0, 0, 0, 64, 64, PGL_CAMERA_TARGET_SCISSOR);

    // cam1: right half of the back buffer, closer view.
    EncodePerspectiveCamera(enc, 1, -3.0f);
    enc.SetCameraTarget(1, 0, 64, 0, 64, 64, PGL_CAMERA_TARGET_SCISSOR);

    // cam2: into layer 3, scissored to (4,4,56,24) inside the 64×32 FB.
    // Projection remains panel-space: cube (-3.75,0,0) maps to layer
    // (16,16.64), then the layer offset places its center near screen (48,33).
    enc.SetCamera(2, 0,
                  { 0.0f, 1.2f, -5.0f }, kIdentityQuat,
                  { 1.0f, 1.0f, 1.0f }, kIdentityQuat, kIdentityQuat, false);
    enc.SetCameraTarget(2, 3, 4, 4, 56, 24, PGL_CAMERA_TARGET_SCISSOR);

    // 2D "UI" over the 3D layer (executed after the 3D pass — see header):
    // navy title bar + white frame outline.
    enc.DrawRect2D(3, 0, 0, 64, 6, 0x0841, true);    // filled navy title bar
    enc.DrawRect2D(3, 0, 6, 64, 26, 0xFFFF, false);  // white frame outline

    // Draw 1: cube, world x=-3.75 → left half at z=5 (cam0) / into the layer
    // window (cam2); lands at screen x<0 in cam1's closer view (culled).
    enc.DrawObject(0, 0,
                   { -3.75f, 0.0f, 0.0f }, GentleTilt(),
                   { 1.2f, 1.2f, 1.2f },
                   kIdentityQuat, kIdentityQuat,
                   { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                   true);

    // Draw 2: teapot, world x=+2.4 → right half at z=3 (cam1); clipped out of
    // cam0's left-half scissor and outside cam2's layer region (x≈95 > 64).
    enc.DrawObject(1, 0,
                   { 2.4f, 0.0f, 0.0f }, GentleTilt(),
                   { 0.35f, 0.35f, 0.35f },
                   kIdentityQuat, kIdentityQuat,
                   { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                   true);

    enc.EndFrame();

    if (enc.HasOverflow() || enc.HasInvalidCommand()) {
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
    if (enc.HasOverflow() || enc.HasInvalidCommand()) {
        std::fprintf(stderr, "[sim] ERROR: encoder overflow\n");
        return kEncodeError;
    }
    return enc.GetLength();
}

// ─── Content self-checks ────────────────────────────────────────────────────


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

/// B7 (G4): pin the exact ROP blend results at four probe pixels:
///   (55,35) — translucent cyan (α=0.45) over opaque orange  → 0x8D0D
///   (37,38) — translucent cyan over the black background    → 0x02CD
///   (95,28) — translucent magenta over the black background → 0x680B
///   (86,28) — opaque orange where A occludes the behind cube → 0xFC00
/// Expected values derived from the ROP formula dst = src·a + dst·(1−a)
/// (per-channel float, truncated, a = 0.45f) on the unpacked RGB565 channels.
static bool CheckB7Scene(const uint16_t* fb, uint16_t w, uint16_t h) {
    if (!CheckCoverage(fb, w, h, 1500, 0)) return false;
    bool ok = true;
    auto probe = [&](uint16_t x, uint16_t y, uint16_t expect, const char* what) {
        const uint16_t got = fb[y * w + x];
        const bool good = (got == expect);
        std::printf("  [%s] %s (%u,%u) == 0x%04X (got 0x%04X)\n",
                    good ? " OK " : "FAIL", what, x, y, expect, got);
        if (!good) ok = false;
    };
    probe(55, 35, 0x8D0D, "cyan a=0.45 over opaque orange");
    probe(37, 38, 0x02CD, "cyan a=0.45 over background   ");
    probe(95, 28, 0x680B, "magenta a=0.45 over background ");
    probe(86, 28, 0xFC00, "opaque occludes behind-cube   ");
    return ok;
}

/// B8 (G6): left quad (x 5..59) is nearest-sampled — every pixel must be a
/// PURE texel colour (0xF800 red or 0x001F blue).  Right quad (x 69..123) is
/// bilinear — must contain mixed (lerped) pixels, and the two halves must
/// differ.  Quad rects are generous (full interior incl. ramps).
static bool CheckB8Scene(const uint16_t* fb, uint16_t w, uint16_t h) {
    if (!CheckCoverage(fb, w, h, 4000, 0)) return false;

    bool ok = true;
    uint32_t leftImpure = 0, rightMixed = 0, halvesDiffer = 0;
    for (uint16_t y = 5; y < 59; ++y) {
        for (uint16_t x = 5; x < 59; ++x) {
            const uint16_t c = fb[y * w + x];
            if (c != 0xF800 && c != 0x001F) ++leftImpure;
        }
        for (uint16_t x = 69; x < 123; ++x) {
            const uint16_t c = fb[y * w + x];
            const uint8_t r = (c >> 11) & 0x1F, b = c & 0x1F;
            if (r > 0 && b > 0) ++rightMixed;
        }
        // Same relative position inside each quad (54 px pitch, offset 64).
        for (uint16_t x = 5; x < 59; ++x) {
            if (fb[y * w + x] != fb[y * w + x + 64]) ++halvesDiffer;
        }
    }

    auto check = [&](bool cond, const char* what, uint32_t got) {
        std::printf("  [%s] %s (got %u)\n", cond ? " OK " : "FAIL", what, got);
        if (!cond) ok = false;
    };
    check(leftImpure == 0,  "nearest half: pure texel colours only", leftImpure);
    check(rightMixed > 100, "bilinear half: mixed lerp pixels present", rightMixed);
    check(halvesDiffer > 100, "halves differ (filter actually applied)", halvesDiffer);
    return ok;
}

/// B9 (G3/G7): pin the multi-camera structure — left-half cube (cam0,
/// scissor (0,0,64,64)), right-half teapot (cam1, scissor (64,0,64,64)), and
/// the layer-3 3D window composited at (32,16) with 2D UI drawn over it.
/// All screen coordinates: window = x[32,96) × y[16,48).
static bool CheckB9Scene(const uint16_t* fb, uint16_t w, uint16_t h) {
    bool ok = true;
    // Independent camera regions must each contain geometry; a title bar or
    // another camera cannot satisfy an aggregate full-frame coverage bound.
    auto region = [&](uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                      uint32_t minimum, const char* what) {
        uint32_t count = 0;
        for (uint16_t y = y0; y < y1; ++y)
            for (uint16_t x = x0; x < x1; ++x) count += fb[size_t(y)*w+x] != 0;
        const bool good = count >= minimum;
        std::printf("  [%s] %s: %u nonblack pixels (minimum %u)\n",
                    good ? " OK " : "FAIL", what, count, minimum);
        if (!good) ok = false;
    };
    region(0, 16, 32, 48, 64, "cam0 geometry outside composited window");
    region(96, 0, w, h, 64, "cam1 geometry outside composited window");
    region(44,29,52,37,16,"cam2 panel-projected cube interior");
    auto probe = [&](uint16_t x, uint16_t y, uint16_t expect, const char* what) {
        const uint16_t got = fb[y * w + x];
        const bool good = (got == expect);
        std::printf("  [%s] %s (%u,%u) == 0x%04X (got 0x%04X)\n",
                    good ? " OK " : "FAIL", what, x, y, expect, got);
        if (!good) ok = false;
    };
    auto probeNonBlack = [&](uint16_t x, uint16_t y, const char* what) {
        const uint16_t got = fb[y * w + x];
        const bool good = (got != 0x0000);
        std::printf("  [%s] %s (%u,%u) != black (got 0x%04X)\n",
                    good ? " OK " : "FAIL", what, x, y, got);
        if (!good) ok = false;
    };
    probeNonBlack(16, 32,  "cam0 left-half cube content        ");
    probeNonBlack(115, 32, "cam1 right-half teapot content     ");
    probeNonBlack(48,33,"cam2 panel-projected cube inside the target layer");
    probe(33, 17, 0x0841,  "navy 2D title bar over the 3D layer");
    probe(32, 22, 0xFFFF,  "white 2D frame over the 3D layer   ");
    probe(90,32,0x0000,"viewport background inside composited window");
    probe(31, 32, 0x0000,  "one px left of window (offset exact)");
    probe(16, 10, 0x0000,  "left half above window (no bleed)   ");
    probe(63, 60, 0x0000,  "cam0 half, below window (no bleed)  ");
    probe(64, 60, 0x0000,  "cam1 half, below teapot (no bleed)  ");
    return ok;
}

// ─── Scene registry ─────────────────────────────────────────────────────────
// Each scene encodes one or more batches and produces one final PPM.
// All normal frames render immediately; resource-only uploads do not present.
// Content oracles constrain visible geometry/materials, not internal counts.

struct SceneDef {
    const char* name;             // registry key / golden filename stem
    const char* blurb;            // one-line description
    size_t (*encode)(uint8_t* buf, size_t capacity, uint8_t frameIndex);
    uint8_t resourceBatches;      // leading batches admitted without rendering
    bool (*check)(const uint16_t* fb, uint16_t w, uint16_t h);
};

static const SceneDef kScenes[] = {
    { "cube", "Centered lit cube", EncodeCubeScene, 0, CheckCubeScene },
    { "B1_teapot", "Teapot 587v/1166t, near-plane crossing", EncodeB1Teapot, 0,
      [](const uint16_t* fb, uint16_t w, uint16_t h) {
          return CheckCoverage(fb, w, h, 1500, 0);
      } },
    { "B2_2d_layers", "2D primitives and layer compositing", EncodeB2LayerStorm, 0, CheckB2Scene },
    { "B3_textures", "Sixteen magnified procedural textures", EncodeB3Textures, 3,
      [](const uint16_t* fb, uint16_t w, uint16_t h) {
          return CheckCoverage(fb, w, h, 7000, 0);
      } },
    { "B4_psb_postfx", "Full-frame invert/gamma/vignette chain", EncodeB4PsbPostFx, 0,
      [](const uint16_t* fb, uint16_t w, uint16_t h) {
          return CheckCoverage(fb,w,h,7000,0);
      } },
    { "B6_empty", "Empty frame clears the target", EncodeB6Empty, 0, CheckEmptyScene },
    { "B7_alpha3d", "Opaque depth and ordered straight alpha", EncodeB7Alpha3D, 0, CheckB7Scene },
    { "B8_bilinear", "Nearest versus bilinear texture filtering", EncodeB8Bilinear, 0, CheckB8Scene },
    { "B9_multicam", "Split screen and panel-space 3D-in-UI", EncodeB9Multicam, 0, CheckB9Scene },
};

static constexpr size_t kSceneCount = sizeof(kScenes) / sizeof(kScenes[0]);

static const SceneDef* FindScene(const char* name) {
    for (size_t i = 0; i < kSceneCount; ++i) {
        if (std::strcmp(kScenes[i].name, name) == 0) return &kScenes[i];
    }
    return nullptr;
}

// ─── Frame runner ───────────────────────────────────────────────────────────

static uint64_t NowUs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now().time_since_epoch()).count());
}

static int RunScene(const SceneDef& scene, const char* ppmPath) {
    std::printf("=== RP2350-ProtoGPU native protocol9 renderer ===\n");
    std::printf("[sim] Scene: %s — %s\n", scene.name, scene.blurb);
    std::printf("[sim] Target: %ux%u RGB565; %ux%u tiles; two native workers\n",
                W, H, TileConfig::ColsFor(W), TileConfig::RowsFor(H));
    sceneState.renderWidth = W;
    sceneState.renderHeight = H;
    if (!sceneState.InitSceneHeap()) {
        std::fprintf(stderr, "[sim] FAIL: real scene heap initialization failed\n");
        return 1;
    }
    sceneState.Reset();
    scheduler.Initialize();
    if (!scheduler.StartWorker()) {
        std::fprintf(stderr, "[sim] FAIL: native worker startup failed\n");
        return 1;
    }
    struct WorkerLifetime {
        ~WorkerLifetime() { scheduler.Shutdown(); }
    } workerLifetime;
    StageTiming timing;
    const auto start = Clock::now();
    unsigned batches = 0, rendered = 0;
    for (uint8_t index = 0; index < 8; ++index) {
        const size_t length = scene.encode(cmdBuffer, sizeof(cmdBuffer), index);
        if (length == kEncodeError) return 1;
        if (!length) break;
        const bool resourceOnly = index < scene.resourceBatches;
        CommandParser::BatchInfo batch;
        const auto parseStart = Clock::now();
        const auto parsed = CommandParser::Parse(cmdBuffer, length, &sceneState, batch, resourceOnly);
        timing.parseMs += MsSince(parseStart);
        ++batches;
        std::printf("[sim] Batch %u: %zu bytes, frame %u, %s, result %u\n",
                    batches, length, batch.frameNumber,
                    resourceOnly ? "resource-only" : "normal frame", unsigned(parsed));
        if (parsed != PglRuntime::Result::Ok) {
            std::fprintf(stderr, "[sim] FAIL: parser result %u, errors %u, mask 0x%08X\n",
                         unsigned(parsed), CommandParser::GetParserErrorCount(),
                         CommandParser::GetParserErrorMask());
            return 1;
        }
        if (resourceOnly) continue;
        FrameTimings frame;
        const auto result = renderer.Render(sceneState, backBuffer, zBuffer,
                                           W, H, frame, NowUs, nullptr);
        std::printf("[sim] Frame %u: render result %u, triangles %u; "
                    "prepare %u us, raster %u us, effects %u us, layers %u us\n",
                    batch.frameNumber, unsigned(result), frame.triangles,
                    frame.prepareUs, frame.rasterUs, frame.effectsUs, frame.layersUs);
        if (result != PglRuntime::Result::Ok) {
            std::fprintf(stderr, "[sim] FAIL: failed target is not presented\n");
            return 1;
        }
        timing.prepareMs += frame.prepareUs / 1000.0;
        timing.rasterMs += frame.rasterUs / 1000.0;
        timing.screenspaceMs += frame.effectsUs / 1000.0;
        timing.draw2dMs += frame.layersUs / 1000.0;
        auto* previous = frontBuffer;
        frontBuffer = backBuffer;
        backBuffer = previous;
        ++rendered;
        sceneState.RetireFrameData();
    }
    if (!rendered || batches == 8) {
        std::fprintf(stderr, "[sim] FAIL: missing normal frame or unterminated corpus\n");
        return 1;
    }
    timing.totalMs = MsSince(start);
    std::printf("[sim] Rendered %u normal frame(s), admitted %u batch(es)\n", rendered, batches);
    std::printf("[sim] Frame FNV-1a checksum: 0x%08X\n", Fnv1a(frontBuffer, GpuConfig::FRAMEBUF_SIZE));
    if (!WritePPM(ppmPath, frontBuffer, W, H)) return 1;
    std::printf("[sim] Wrote %s\n", ppmPath);
    PrintAsciiPreview(frontBuffer, W, H);
    std::printf("[sim] Aggregate actual wall-clock stages (ms): parse %.3f, prepare %.3f, "
                "raster %.3f, effects %.3f, layers %.3f; encode-to-present %.3f\n",
                timing.parseMs, timing.prepareMs, timing.rasterMs, timing.screenspaceMs,
                timing.draw2dMs, timing.totalMs);
    const bool ok = scene.check(frontBuffer, W, H);
    std::printf("=== SIM RESULT: %s ===\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
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
                std::printf("  %-14s %s\n", kScenes[s].name, kScenes[s].blurb);
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
