/**
 * @file rasterizer.cpp
 * @brief GPU-side rasterizer — transform/clip/project + two-pass tile ROP.
 *
 * Pipeline (protocol 9 — see rasterizer.h for the declared conventions):
 *   PrepareFrame / PrepareNextCameraPass (core 0, single-threaded):
 *     per-camera pass — bind target stride + viewport scissor, then per draw
 *     call: morph-aware conservative
 *     AABB cull (near-crossing draws are NEVER culled from base bounds) →
 *     vertex transform → view-space (rotation ∘ baseRotation ∘ lookOffset,
 *     component camera scale) → near-plane clip (perspective) → project →
 *     compact 80 B Triangle2D pool + per-triangle tile-coverage rectangles
 *     (lossless); retire world scratch, start/clear the target-strided depth.
 *   RasterizeTile (both cores, disjoint tiles):
 *     clear the scissored tile band, then opaque pass (strict-depth write)
 *     and translucent pass (PGL_BLEND_ALPHA source-over) — BOTH in stable
 *     submission (pool) order; the translucent pass depth-tests against the
 *     opaque depth buffer and performs NO depth writes.  alpha == 0 and
 *     mask-discarded pixels write neither colour nor depth (never occlude).
 *
 * Capacity contract: projected-pool overflow drops ONLY the extra triangle
 * and latches the frame error (GetFrameError() == RenderOverflow) — no
 * silent geometry loss: the integrator must not present the failed target.
 */

#include "rasterizer.h"
#include "../scene_state.h"
#include "../gpu_config.h"
#include "../math/pgl_math.h"
#include "triangle2d.h"
#include "../phase_scratch.h"

#include <cstring>
#include <cstdio>
#include <cmath>

// ─── Debug Options ───────────────────────────────────────────────────────────
#ifndef GPU_CONFIG_DEBUG_PREPARE_PRINT
#define GPU_CONFIG_DEBUG_PREPARE_PRINT 0
#endif

// ─── Tile Coverage Geometry ─────────────────────────────────────────────────
// Tiles are 16×16 target pixels. The supported grid has at most 64 cells,
// so inclusive tile bounds fit uint8_t exactly on either axis.

static constexpr uint16_t kTileW = 16;
static constexpr uint16_t kTileH = 16;
static constexpr uint32_t kMaxTileGridCells = 64;

// ─── Static Projected-Triangle Pool ─────────────────────────────────────────
// Pre-allocated pool of accepted projected triangles for the current pass.
//   trianglePool:       1280 × 80 B = 100 KB
//   triangleTileBounds: 1280 ×  4 B =   5 KB
// The former 64-bit masks were filled with every bit in an AABB rectangle.
// Four inclusive bounds represent exactly the same candidates, without any
// capped list or per-triangle bitset fill.

struct TileBounds { uint8_t x0, y0, x1, y1; };
static_assert(sizeof(TileBounds) == 4, "compact lossless tile coverage");
static Triangle2D trianglePool[GpuConfig::MAX_TRIANGLES];
static TileBounds triangleTileBounds[GpuConfig::MAX_TRIANGLES];
static uint16_t trianglePoolUsed = 0;
static bool s_poolOverflow = false; // latched by emit, read by pass prep

// A preparation scope ends world-vertex lifetime and restarts/clears depth on
// every exit. Both tile workers start only after this scope has retired.
struct DepthPreparation {
    PhaseScratch::DepthWorkspace& workspace;
    uint32_t pixels;
    PglVec3* vertices;
    DepthPreparation(PhaseScratch::DepthWorkspace& depth, uint32_t count)
        : workspace(depth), pixels(count), vertices(depth.BeginWorld()) {}
    ~DepthPreparation() {
        std::memset(workspace.BeginDepth(), 0xFF, pixels * sizeof(uint16_t));
    }
};

// ─── uint16_t Z-buffer Conversion ───────────────────────────────────────
// The Z-buffer uses uint16_t to save SRAM (vs float).  Positive IEEE float
// bit patterns are monotonic as unsigned integers; their upper 16 bits give
// monotonic but quantized depth (distinct nearby depths can tie).

/// Convert a positive float depth to a sortable uint16_t.
static inline uint16_t FloatZToU16(float z) {
    uint32_t bits;
    __builtin_memcpy(&bits, &z, 4);
    return static_cast<uint16_t>(bits >> 16);
}

// ─── RGB565 Utility Functions ───────────────────────────────────────────────

static inline uint16_t PackRGB565(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static inline void UnpackRGB565(uint16_t c, uint8_t& r, uint8_t& g, uint8_t& b) {
    r = static_cast<uint8_t>(((c >> 11) & 0x1F) << 3);
    g = static_cast<uint8_t>(((c >> 5)  & 0x3F) << 2);
    b = static_cast<uint8_t>((c & 0x1F) << 3);
}

// ─── Source-Over Alpha Blend for the 3D ROP ─────────────────────────────────
// dst = src·a + dst·(1−a) per RGB channel in float, truncated back to RGB565
// (same quantisation convention as the material evaluators).
//
// Endpoint contract (exact):
//   a ≤ 0 or NaN → dst unchanged (the caller also skips whole triangles with
//                  non-positive alpha — the blend is a bit-exact identity:
//                  src·0 == +0, dst·1 == dst, Pack(Unpack(dst)) == dst).
//   a ≥ 1        → src exactly (src·1 == src, dst·0 == +0 for non-negative
//                  channels, sum exact — bit-identical to the opaque write).
static inline uint16_t BlendAlphaRGB565(uint16_t src, uint16_t dst, float alpha) {
    if (!(alpha > 0.0f)) return dst;
    if (alpha >= 1.0f)   return src;
    uint8_t sr, sg, sb, dr, dg, db;
    UnpackRGB565(src, sr, sg, sb);
    UnpackRGB565(dst, dr, dg, db);
    const float inv = 1.0f - alpha;   // alpha ∈ (0,1) here → inv ∈ (0,1)
    const uint8_t r = static_cast<uint8_t>(
        static_cast<float>(sr) * alpha + static_cast<float>(dr) * inv);
    const uint8_t g = static_cast<uint8_t>(
        static_cast<float>(sg) * alpha + static_cast<float>(dg) * inv);
    const uint8_t b = static_cast<uint8_t>(
        static_cast<float>(sb) * alpha + static_cast<float>(db) * inv);
    return PackRGB565(r, g, b);
}

// ─── Cortex-M33 DSP/SIMD Optimized RGB565 Blend ────────────────────────────
// Compile-time switch: PGL_USE_DSP_BLEND (default: enabled on ARM Cortex-M33).
//
// The DSP path uses packed 16-bit SIMD intrinsics (__uqadd16, __uqsub16,
// __usat16) to operate on the R/G channels simultaneously.  Channel packing:
// {R8, G8} as packed halfwords in a uint32_t, B8 separate; green keeps full
// 8-bit precision (repacked to 6-bit at final output).
//
// Correctness contract (matches the scalar float path):
//   - ADD saturates each channel at 255 via __usat16 (a plain __uqadd16 only
//     saturates the 16-bit halfword at 65535 — that was the old overflow bug,
//     e.g. 200+200 wrapped to 144 on re-pack).
//   - SUBTRACT saturates each channel at 0 via __uqsub16.
//   - opacity ≤ 0 / NaN → colorA exactly; opacity ≥ 1 → the raw blend result
//     (both bit-identical to the scalar path endpoints).
//   - Mid-range uses a fixed-point lerp with alpha256 = trunc(opacity·255),
//     which is bounded to ≤1 LSB of the scalar float lerp — the declared
//     target-DSP convention (native goldens remain the scalar reference).

#if defined(__ARM_FEATURE_DSP) || defined(__ARM_ARCH_8M_MAIN__)
#define PGL_USE_DSP_BLEND 1
#else
#define PGL_USE_DSP_BLEND 0
#endif

#if PGL_USE_DSP_BLEND
#include <arm_acle.h>   // __uqadd16, __uqsub16, __usat16

// Unpack RGB565 to {R8|G8} packed halfword + B8
static inline void UnpackRGB565_DSP(uint16_t c, uint32_t& rg, uint8_t& b) {
    uint8_t r8 = static_cast<uint8_t>(((c >> 11) & 0x1F) << 3);
    uint8_t g8 = static_cast<uint8_t>(((c >> 5)  & 0x3F) << 2);
    b = static_cast<uint8_t>((c & 0x1F) << 3);
    rg = (static_cast<uint32_t>(r8) << 16) | static_cast<uint32_t>(g8);
}

// Repack {R8|G8} + B8 to RGB565
static inline uint16_t PackRGB565_DSP(uint32_t rg, uint8_t b) {
    uint8_t r8 = static_cast<uint8_t>(rg >> 16);
    uint8_t g8 = static_cast<uint8_t>(rg & 0xFF);
    return static_cast<uint16_t>(((r8 >> 3) << 11) | ((g8 >> 2) << 5) | (b >> 3));
}

// Fixed-point lerp for a single byte: a + ((b-a) * alpha256) >> 8.
// Inputs/outputs are 0..255; the >> on the (possibly negative) product is an
// arithmetic shift on all supported toolchains (declared convention).
static inline uint8_t LerpByte(uint8_t a, uint8_t b, uint8_t alpha256) {
    return static_cast<uint8_t>(a + ((static_cast<int>(b) - a) * alpha256 >> 8));
}

// Fixed-point lerp for packed {R|G} halfwords (each 0..255).
static inline uint32_t LerpRG(uint32_t rgA, uint32_t rgB, uint8_t alpha256) {
    int rA = static_cast<int>(rgA >> 16);
    int gA = static_cast<int>(rgA & 0xFFFF);
    int rB = static_cast<int>(rgB >> 16);
    int gB = static_cast<int>(rgB & 0xFFFF);
    uint8_t rR = static_cast<uint8_t>(rA + ((rB - rA) * alpha256 >> 8));
    uint8_t gR = static_cast<uint8_t>(gA + ((gB - gA) * alpha256 >> 8));
    return (static_cast<uint32_t>(rR) << 16) | static_cast<uint32_t>(gR);
}

/// DSP-accelerated RGB565 blend for common modes.
/// Returns true if the DSP path handled the blend; false = fall through to scalar.
static bool BlendRGB565_DSP(uint16_t colorA, uint16_t colorB,
                            PglBlendMode mode, float opacity,
                            uint16_t& result) {
    // Endpoint contract identical to the scalar path (also makes NaN safe).
    if (!(opacity > 0.0f)) { result = colorA; return true; }

    uint32_t rgA, rgB;
    uint8_t bA, bB;
    UnpackRGB565_DSP(colorA, rgA, bA);
    UnpackRGB565_DSP(colorB, rgB, bB);

    uint32_t rgBlend;
    uint8_t  bBlend;

    switch (mode) {
    case PGL_BLEND_ADD: {
        // Per-channel saturating add: __uqadd16 saturates the 16-bit lanes
        // (no wrap), then __usat16 clamps each lane to 0..255 — this is the
        // declared ADD saturation, matching clamp(fa + fb, 0, 255).
        rgBlend = __usat16(__uqadd16(rgA, rgB), 8);
        const uint32_t bSum = static_cast<uint32_t>(bA) + bB;
        bBlend = static_cast<uint8_t>(bSum > 255 ? 255 : bSum);
        break;
    }
    case PGL_BLEND_SUBTRACT: {
        // Per-channel clamp at 0 — matches clamp(fa - fb, 0, 255).
        rgBlend = __uqsub16(rgA, rgB);
        bBlend  = static_cast<uint8_t>(bA > bB ? bA - bB : 0);
        break;
    }
    case PGL_BLEND_DARKEN: {
        uint8_t rA8 = static_cast<uint8_t>(rgA >> 16), rB8 = static_cast<uint8_t>(rgB >> 16);
        uint8_t gA8 = static_cast<uint8_t>(rgA), gB8 = static_cast<uint8_t>(rgB);
        rgBlend = (static_cast<uint32_t>(rA8 < rB8 ? rA8 : rB8) << 16)
                | static_cast<uint32_t>(gA8 < gB8 ? gA8 : gB8);
        bBlend  = bA < bB ? bA : bB;
        break;
    }
    case PGL_BLEND_LIGHTEN: {
        uint8_t rA8 = static_cast<uint8_t>(rgA >> 16), rB8 = static_cast<uint8_t>(rgB >> 16);
        uint8_t gA8 = static_cast<uint8_t>(rgA), gB8 = static_cast<uint8_t>(rgB);
        rgBlend = (static_cast<uint32_t>(rA8 > rB8 ? rA8 : rB8) << 16)
                | static_cast<uint32_t>(gA8 > gB8 ? gA8 : gB8);
        bBlend  = bA > bB ? bA : bB;
        break;
    }
    case PGL_BLEND_BASE:
        result = colorA;
        return true;
    case PGL_BLEND_REPLACE:
        rgBlend = rgB;
        bBlend  = bB;
        break;
    default:
        // Complex modes (Multiply, Divide, Screen, Overlay, SoftLight,
        // EfficientMask) fall through to the scalar float path.
        return false;
    }

    // opacity ≥ 1 → raw blend result, bit-identical to the scalar endpoint.
    if (opacity >= 1.0f) {
        result = PackRGB565_DSP(rgBlend, bBlend);
        return true;
    }

    // Mid-range fixed-point lerp (declared ≤1 LSB convention, see header).
    const uint8_t alpha256 = static_cast<uint8_t>(opacity * 255.0f);
    result = PackRGB565_DSP(LerpRG(rgA, rgBlend, alpha256),
                            LerpByte(bA, bBlend, alpha256));
    return true;
}
#endif  // PGL_USE_DSP_BLEND

// ─── 3D Simplex Noise ──────────────────────────────────────────────────────
// Minimal implementation for GPU-side noise materials.
// Uses a 256-entry permutation table and 12 gradient vectors.

static const uint8_t s_perm[256] = {
    151,160,137, 91, 90, 15,131, 13,201, 95, 96, 53,194,233,  7,225,
    140, 36,103, 30, 69,142,  8, 99, 37,240, 21, 10, 23,190,  6,148,
    247,120,234, 75,  0, 26,197, 62, 94,252,219,203,117, 35, 11, 32,
     57,177, 33, 88,237,149, 56, 87,174, 20,125,136,171,168, 68,175,
     74,165, 71,134,139, 48, 27,166, 77,146,158,231, 83,111,229,122,
     60,211,133,230,220,105, 92, 41, 55, 46,245, 40,244,102,143, 54,
     65, 25, 63,161,  1,216, 80, 73,209, 76,132,187,208, 89, 18,169,
    200,196,135,130,116,188,159, 86,164,100,109,198,173,186,  3, 64,
     52,217,226,250,124,123,  5,202, 38,147,118,126,255, 82, 85,212,
    207,206, 59,227, 47, 16, 58, 17,182,189, 28, 42,223,183,170,213,
    119,248,152,  2, 44,154,163, 70,221,153,101,155,167, 43,172,  9,
    129, 22, 39,253, 19, 98,108,110, 79,113,224,232,178,185,112,104,
    218,246, 97,228,251, 34,242,193,238,210,144, 12,191,179,162,241,
     81, 51,145,235,249, 14,239,107, 49,192,214, 31,181,199,106,157,
    184, 84,204,176,115,121, 50, 45,127,  4,150,254,138,236,205, 93,
    222,114, 67, 29, 24, 72,243,141,128,195, 78, 66,215, 61,156,180
};

static inline uint8_t permAt(int i) { return s_perm[i & 0xFF]; }

static const float s_grad3[12][3] = {
    { 1, 1, 0}, {-1, 1, 0}, { 1,-1, 0}, {-1,-1, 0},
    { 1, 0, 1}, {-1, 0, 1}, { 1, 0,-1}, {-1, 0,-1},
    { 0, 1, 1}, { 0,-1, 1}, { 0, 1,-1}, { 0,-1,-1}
};

static inline float grad3Dot(int hash, float x, float y, float z) {
    const float* g = s_grad3[hash % 12];
    return g[0] * x + g[1] * y + g[2] * z;
}

/// Standard 3D simplex noise.  Returns value in [-1, 1].
static float SimplexNoise3D(float xin, float yin, float zin) {
    static constexpr float F3 = 1.0f / 3.0f;
    static constexpr float G3 = 1.0f / 6.0f;

    float s = (xin + yin + zin) * F3;
    int i = static_cast<int>(floorf(xin + s));
    int j = static_cast<int>(floorf(yin + s));
    int k = static_cast<int>(floorf(zin + s));

    float t = static_cast<float>(i + j + k) * G3;
    float X0 = static_cast<float>(i) - t;
    float Y0 = static_cast<float>(j) - t;
    float Z0 = static_cast<float>(k) - t;
    float x0 = xin - X0;
    float y0 = yin - Y0;
    float z0 = zin - Z0;

    int i1, j1, k1;
    int i2, j2, k2;
    if (x0 >= y0) {
        if (y0 >= z0)      { i1=1; j1=0; k1=0; i2=1; j2=1; k2=0; }
        else if (x0 >= z0) { i1=1; j1=0; k1=0; i2=1; j2=0; k2=1; }
        else               { i1=0; j1=0; k1=1; i2=1; j2=0; k2=1; }
    } else {
        if (y0 < z0)       { i1=0; j1=0; k1=1; i2=0; j2=1; k2=1; }
        else if (x0 < z0)  { i1=0; j1=1; k1=0; i2=0; j2=1; k2=1; }
        else               { i1=0; j1=1; k1=0; i2=1; j2=1; k2=0; }
    }

    float x1 = x0 - static_cast<float>(i1) + G3;
    float y1 = y0 - static_cast<float>(i1) + G3;
    float z1 = z0 - static_cast<float>(i1) + G3;
    float x2 = x0 - static_cast<float>(i2) + 2.0f * G3;
    float y2 = y0 - static_cast<float>(i2) + 2.0f * G3;
    float z2 = z0 - static_cast<float>(i2) + 2.0f * G3;
    float x3 = x0 - 1.0f + 3.0f * G3;
    float y3 = y0 - 1.0f + 3.0f * G3;
    float z3 = z0 - 1.0f + 3.0f * G3;

    int ii = i & 0xFF;
    int jj = j & 0xFF;
    int kk = k & 0xFF;
    int gi0 = permAt(ii + permAt(jj + permAt(kk)));
    int gi1 = permAt(ii + i1 + permAt(jj + j1 + permAt(kk + k1)));
    int gi2 = permAt(ii + i2 + permAt(jj + j2 + permAt(kk + k2)));
    int gi3 = permAt(ii + 1  + permAt(jj + 1  + permAt(kk + 1 )));

    float n = 0.0f;

    float t0 = 0.6f - x0*x0 - y0*y0 - z0*z0;
    if (t0 > 0.0f) { t0 *= t0; n += t0 * t0 * grad3Dot(gi0, x0, y0, z0); }

    float t1 = 0.6f - x1*x1 - y1*y1 - z1*z1;
    if (t1 > 0.0f) { t1 *= t1; n += t1 * t1 * grad3Dot(gi1, x1, y1, z1); }

    float t2 = 0.6f - x2*x2 - y2*y2 - z2*z2;
    if (t2 > 0.0f) { t2 *= t2; n += t2 * t2 * grad3Dot(gi2, x2, y2, z2); }

    float t3 = 0.6f - x3*x3 - y3*y3 - z3*z3;
    if (t3 > 0.0f) { t3 *= t3; n += t3 * t3 * grad3Dot(gi3, x3, y3, z3); }

    return 32.0f * n;  // scale to [-1, 1]
}

// ─── HSV → RGB565 ──────────────────────────────────────────────────────────
// h: [0, 360), s: [0, 1], v: [0, 1]

static uint16_t HSVtoRGB565(float h, float s, float v) {
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float r1, g1, b1;
    if      (h < 60.0f)  { r1=c; g1=x; b1=0; }
    else if (h < 120.0f) { r1=x; g1=c; b1=0; }
    else if (h < 180.0f) { r1=0; g1=c; b1=x; }
    else if (h < 240.0f) { r1=0; g1=x; b1=c; }
    else if (h < 300.0f) { r1=x; g1=0; b1=c; }
    else                 { r1=c; g1=0; b1=x; }
    uint8_t r = static_cast<uint8_t>((r1 + m) * 255.0f);
    uint8_t g = static_cast<uint8_t>((g1 + m) * 255.0f);
    uint8_t b = static_cast<uint8_t>((b1 + m) * 255.0f);
    return PackRGB565(r, g, b);
}

// ─── Blend Mode Evaluation ─────────────────────────────────────────────────
// Operates on two RGB565 colours.  Unpacks to 888, blends per-channel, repacks.
// Opacity endpoints are exact in both this scalar path and the DSP path:
// opacity ≤ 0 → A, opacity ≥ 1 → the (clamped) blend result.

static inline uint8_t BlendChannel(uint8_t a, uint8_t b, PglBlendMode mode, float opacity) {
    float fa = static_cast<float>(a);
    float fb = static_cast<float>(b);
    float result;

    switch (mode) {
        case PGL_BLEND_BASE:           result = fa; break;
        case PGL_BLEND_ADD:            result = fa + fb; break;
        case PGL_BLEND_SUBTRACT:       result = fa - fb; break;
        case PGL_BLEND_MULTIPLY:       result = fa * fb / 255.0f; break;
        case PGL_BLEND_DIVIDE:         result = (fb > 0.5f) ? (fa * 255.0f / fb) : 255.0f; break;
        case PGL_BLEND_DARKEN:         result = fminf(fa, fb); break;
        case PGL_BLEND_LIGHTEN:        result = fmaxf(fa, fb); break;
        case PGL_BLEND_SCREEN:         result = 255.0f - (255.0f - fa) * (255.0f - fb) / 255.0f; break;
        case PGL_BLEND_OVERLAY:
            result = (fa < 128.0f)
                ? (2.0f * fa * fb / 255.0f)
                : (255.0f - 2.0f * (255.0f - fa) * (255.0f - fb) / 255.0f);
            break;
        case PGL_BLEND_SOFTLIGHT:
            result = ((1.0f - 2.0f * fb / 255.0f) * fa * fa / 255.0f)
                   + (2.0f * fb * fa / 255.0f);
            break;
        case PGL_BLEND_REPLACE:        result = fb; break;
        case PGL_BLEND_EFFICIENT_MASK: result = (fb > 128.0f) ? fa : 0.0f; break;
        default:                       result = fa; break;
    }

    // Apply opacity: lerp between original A and blended result
    result = fa + (result - fa) * opacity;

    // Clamp to [0, 255]
    if (result < 0.0f) result = 0.0f;
    if (result > 255.0f) result = 255.0f;
    return static_cast<uint8_t>(result);
}

static uint16_t BlendRGB565(uint16_t colorA, uint16_t colorB,
                            PglBlendMode mode, float opacity) {
#if PGL_USE_DSP_BLEND
    // Try DSP-accelerated path for common blend modes (identical endpoints
    // and saturation; ≤1 LSB mid-range convention — see the DSP block).
    uint16_t dspResult;
    if (BlendRGB565_DSP(colorA, colorB, mode, opacity, dspResult)) {
        return dspResult;
    }
    // Fall through to scalar path for complex modes (Multiply, Screen, etc.)
#endif

    uint8_t rA, gA, bA, rB, gB, bB;
    UnpackRGB565(colorA, rA, gA, bA);
    UnpackRGB565(colorB, rB, gB, bB);
    uint8_t r = BlendChannel(rA, rB, mode, opacity);
    uint8_t g = BlendChannel(gA, gB, mode, opacity);
    uint8_t b = BlendChannel(bA, bB, mode, opacity);
    return PackRGB565(r, g, b);
}

// ─── Texture Sampling ──────────────────────────────────────────────────────
// Nearest-neighbour sampling from a TextureSlot (default everywhere).
// Opt-in bilinear 4-tap is selected per material via PglParamImage::
// filterFlags bit0.  UV clamped to [0, 1].

/// Fetch one texel as RGB565 (bounds-checked against the uploaded byte count).
static inline uint16_t FetchTexelRGB565(const TextureSlot& tex,
                                        uint16_t px, uint16_t py) {
    uint32_t idx = static_cast<uint32_t>(py) * tex.width + px;

    if (tex.format == PGL_TEX_RGB565) {
        uint32_t byteIdx = idx * 2;
        if (byteIdx + 1 >= tex.pixelDataSize) return 0x0000;
        return static_cast<uint16_t>(tex.pixels[byteIdx])
             | (static_cast<uint16_t>(tex.pixels[byteIdx + 1]) << 8);
    } else {  // PGL_TEX_RGB888
        uint32_t byteIdx = idx * 3;
        if (byteIdx + 2 >= tex.pixelDataSize) return 0x0000;
        return PackRGB565(tex.pixels[byteIdx], tex.pixels[byteIdx + 1],
                          tex.pixels[byteIdx + 2]);
    }
}

static uint16_t SampleTexture(const TextureSlot& tex, float u, float v,
                              bool bilinear) {
    if (!tex.active || !tex.pixels || tex.width == 0 || tex.height == 0) {
        return 0xF81F;  // magenta = missing texture
    }

    // Clamp UV to [0, 1)
    u = PglMath::Clamp(u, 0.0f, 0.9999f);
    v = PglMath::Clamp(v, 0.0f, 0.9999f);

    if (!bilinear) {
        // Nearest-neighbour — unchanged v8 semantics.
        uint16_t px = static_cast<uint16_t>(u * tex.width);
        uint16_t py = static_cast<uint16_t>(v * tex.height);
        return FetchTexelRGB565(tex, px, py);
    }

    // ── Bilinear 4-tap, texel-centre mapping ──────────────────────────────
    // Sample position in texel space is u·width − 0.5 so that integer
    // coordinates sit exactly on texel centres; edge taps clamp (no
    // wraparound, no mip).  Each tap is unpacked from RGB565, lerped per
    // channel in float, and truncated back — the same conventions as the
    // material evaluators.
    const float fx = u * static_cast<float>(tex.width)  - 0.5f;
    const float fy = v * static_cast<float>(tex.height) - 0.5f;
    const int x0 = static_cast<int>(floorf(fx));
    const int y0 = static_cast<int>(floorf(fy));
    const float tx = fx - static_cast<float>(x0);
    const float ty = fy - static_cast<float>(y0);

    const int tw = static_cast<int>(tex.width);
    const int th = static_cast<int>(tex.height);
    const int xa = PglMath::Min(PglMath::Max(x0,     0), tw - 1);
    const int xb = PglMath::Min(PglMath::Max(x0 + 1, 0), tw - 1);
    const int ya = PglMath::Min(PglMath::Max(y0,     0), th - 1);
    const int yb = PglMath::Min(PglMath::Max(y0 + 1, 0), th - 1);

    uint8_t r00, g00, b00, r10, g10, b10, r01, g01, b01, r11, g11, b11;
    UnpackRGB565(FetchTexelRGB565(tex, static_cast<uint16_t>(xa),
                                      static_cast<uint16_t>(ya)),
                 r00, g00, b00);
    UnpackRGB565(FetchTexelRGB565(tex, static_cast<uint16_t>(xb),
                                      static_cast<uint16_t>(ya)),
                 r10, g10, b10);
    UnpackRGB565(FetchTexelRGB565(tex, static_cast<uint16_t>(xa),
                                      static_cast<uint16_t>(yb)),
                 r01, g01, b01);
    UnpackRGB565(FetchTexelRGB565(tex, static_cast<uint16_t>(xb),
                                      static_cast<uint16_t>(yb)),
                 r11, g11, b11);

    // Lerp horizontal pairs, then the two rows vertically (per channel).
    const float rTop = PglMath::Lerp(r00, r10, tx);
    const float rBot = PglMath::Lerp(r01, r11, tx);
    const float gTop = PglMath::Lerp(g00, g10, tx);
    const float gBot = PglMath::Lerp(g01, g11, tx);
    const float bTop = PglMath::Lerp(b00, b10, tx);
    const float bBot = PglMath::Lerp(b01, b11, tx);

    return PackRGB565(
        static_cast<uint8_t>(PglMath::Lerp(rTop, rBot, ty)),
        static_cast<uint8_t>(PglMath::Lerp(gTop, gBot, ty)),
        static_cast<uint8_t>(PglMath::Lerp(bTop, bBot, ty)));
}

// ─── Material Evaluation — Full M6 (12 Types) ──────────────────────────────
//
// Returns an RGB565 colour for a given material + intersection context.
// Supports recursive evaluation for Combine, Mask, and Animator materials
// (depth-limited to prevent stack overflow on Cortex-M33).
//
// `outDiscard` (top-level ROP only): set true when a MASK material rejects
// the pixel — the ROP then writes NEITHER colour NOR depth, so masked-out
// surfaces never occlude geometry behind them.  Recursive evaluations pass
// nullptr: a nested mask failure contributes black to its parent blend
// (unchanged historical semantics) without discarding the whole pixel.
//
// Nested material/texture references on the stored wire params retain their
// [generation:8 | index:8] handle encoding, so every nested access extracts
// the slot with PglHandleIndex() before bounds-checking — the full handle is
// never used as an array index.

static constexpr uint8_t MAX_MATERIAL_RECURSION = 3;

static uint16_t EvaluateMaterial(const MaterialSlot& mat,
                                 const PglVec3& point,
                                 const PglVec3& normal,
                                 const PglVec2& uv,
                                 const SceneState* scene,
                                 float elapsedTimeS,
                                 uint8_t depth,
                                 bool* outDiscard) {
    if (depth > MAX_MATERIAL_RECURSION) return 0xF81F;  // magenta = recursion limit

    switch (mat.type) {

    // ── PGL_MAT_SIMPLE (0x00): Solid colour ────────────────────────────
    case PGL_MAT_SIMPLE: {
        const auto* p = reinterpret_cast<const PglParamSimple*>(mat.params);
        return PackRGB565(p->r, p->g, p->b);
    }

    // ── PGL_MAT_NORMAL (0x01): Map face normal to colour ───────────────
    case PGL_MAT_NORMAL: {
        // Normal components are in [-1, 1]; map to [0, 255]
        uint8_t r = static_cast<uint8_t>(PglMath::Clamp((normal.x + 1.0f) * 0.5f, 0.0f, 1.0f) * 255.0f);
        uint8_t g = static_cast<uint8_t>(PglMath::Clamp((normal.y + 1.0f) * 0.5f, 0.0f, 1.0f) * 255.0f);
        uint8_t b = static_cast<uint8_t>(PglMath::Clamp((normal.z + 1.0f) * 0.5f, 0.0f, 1.0f) * 255.0f);
        return PackRGB565(r, g, b);
    }

    // ── PGL_MAT_DEPTH (0x02): Depth-based gradient ─────────────────────
    case PGL_MAT_DEPTH: {
        const auto* p = reinterpret_cast<const PglParamDepth*>(mat.params);
        float range = p->farZ - p->nearZ;
        float t = (range > 1e-6f)
                ? PglMath::Clamp((point.z - p->nearZ) / range, 0.0f, 1.0f)
                : 0.0f;
        uint8_t r = static_cast<uint8_t>(PglMath::Lerp(static_cast<float>(p->nearR),
                                                        static_cast<float>(p->farR), t));
        uint8_t g = static_cast<uint8_t>(PglMath::Lerp(static_cast<float>(p->nearG),
                                                        static_cast<float>(p->farG), t));
        uint8_t b = static_cast<uint8_t>(PglMath::Lerp(static_cast<float>(p->nearB),
                                                        static_cast<float>(p->farB), t));
        return PackRGB565(r, g, b);
    }

    // ── PGL_MAT_GRADIENT (0x10): Multi-stop gradient ───────────────────
    case PGL_MAT_GRADIENT: {
        const auto* hdr = reinterpret_cast<const PglParamGradientHeader*>(mat.params);
        uint8_t stopCount = hdr->stopCount;
        if (stopCount == 0 || stopCount > 7) return 0x0000;

        // Parse stops from params buffer
        const uint8_t* stopData = mat.params + sizeof(PglParamGradientHeader);
        const auto* stops = reinterpret_cast<const PglGradientStop*>(stopData);

        // Parse axis info after the stops
        const uint8_t* afterStops = stopData + stopCount * sizeof(PglGradientStop);
        uint8_t axis = afterStops[0];

        float rangeMin, rangeMax;
        std::memcpy(&rangeMin, afterStops + 1, sizeof(float));
        std::memcpy(&rangeMax, afterStops + 5, sizeof(float));

        // Pick the position value based on axis
        float pos;
        switch (axis) {
            case 0:  pos = point.x; break;  // X axis
            case 1:  pos = point.y; break;  // Y axis
            default: pos = point.z; break;  // Z axis
        }

        // Normalize position to [0, 1] within range
        float range = rangeMax - rangeMin;
        float t = (range > 1e-6f)
                ? PglMath::Clamp((pos - rangeMin) / range, 0.0f, 1.0f)
                : 0.0f;

        // Find the two surrounding stops and interpolate
        // Stops are assumed sorted by position in [0, 1]
        uint8_t lo = 0, hi = 0;
        for (uint8_t s = 0; s < stopCount - 1; ++s) {
            if (t >= stops[s].position && t <= stops[s + 1].position) {
                lo = s;
                hi = s + 1;
                break;
            }
            hi = s + 1;
        }

        float stopRange = stops[hi].position - stops[lo].position;
        float lerp = (stopRange > 1e-6f)
                   ? (t - stops[lo].position) / stopRange
                   : 0.0f;
        lerp = PglMath::Clamp(lerp, 0.0f, 1.0f);

        uint8_t r = static_cast<uint8_t>(PglMath::Lerp(
            static_cast<float>(stops[lo].r), static_cast<float>(stops[hi].r), lerp));
        uint8_t g = static_cast<uint8_t>(PglMath::Lerp(
            static_cast<float>(stops[lo].g), static_cast<float>(stops[hi].g), lerp));
        uint8_t b = static_cast<uint8_t>(PglMath::Lerp(
            static_cast<float>(stops[lo].b), static_cast<float>(stops[hi].b), lerp));
        return PackRGB565(r, g, b);
    }

    // ── PGL_MAT_LIGHT (0x20): Directional Lambert + ambient ────────────
    case PGL_MAT_LIGHT: {
        const auto* p = reinterpret_cast<const PglParamLight*>(mat.params);
        PglVec3 lightDir = PglMath::Normalize({p->lightDirX, p->lightDirY, p->lightDirZ});

        // N·L diffuse (clamped to [0,1])
        float ndotl = PglMath::Clamp(PglMath::Dot(normal, lightDir), 0.0f, 1.0f);

        uint8_t r = static_cast<uint8_t>(PglMath::Clamp(
            static_cast<float>(p->ambientR) + static_cast<float>(p->diffuseR) * ndotl,
            0.0f, 255.0f));
        uint8_t g = static_cast<uint8_t>(PglMath::Clamp(
            static_cast<float>(p->ambientG) + static_cast<float>(p->diffuseG) * ndotl,
            0.0f, 255.0f));
        uint8_t b = static_cast<uint8_t>(PglMath::Clamp(
            static_cast<float>(p->ambientB) + static_cast<float>(p->diffuseB) * ndotl,
            0.0f, 255.0f));
        return PackRGB565(r, g, b);
    }

    // ── PGL_MAT_SIMPLEX_NOISE (0x30): Simplex noise two-colour blend ───
    case PGL_MAT_SIMPLEX_NOISE: {
        const auto* p = reinterpret_cast<const PglParamSimplexNoise*>(mat.params);
        float nx = point.x * p->scaleX;
        float ny = point.y * p->scaleY;
        float nz = point.z * p->scaleZ + elapsedTimeS * p->speed;
        float noise = SimplexNoise3D(nx, ny, nz);

        // Map [-1, 1] → [0, 1]
        float t = PglMath::Clamp((noise + 1.0f) * 0.5f, 0.0f, 1.0f);

        uint8_t r = static_cast<uint8_t>(PglMath::Lerp(
            static_cast<float>(p->colorAR), static_cast<float>(p->colorBR), t));
        uint8_t g = static_cast<uint8_t>(PglMath::Lerp(
            static_cast<float>(p->colorAG), static_cast<float>(p->colorBG), t));
        uint8_t b = static_cast<uint8_t>(PglMath::Lerp(
            static_cast<float>(p->colorAB), static_cast<float>(p->colorBB), t));
        return PackRGB565(r, g, b);
    }

    // ── PGL_MAT_RAINBOW_NOISE (0x31): Noise → hue (rainbow) ────────────
    case PGL_MAT_RAINBOW_NOISE: {
        const auto* p = reinterpret_cast<const PglParamRainbowNoise*>(mat.params);
        float nx = point.x * p->scale;
        float ny = point.y * p->scale;
        float nz = point.z * p->scale + elapsedTimeS * p->speed;
        float noise = SimplexNoise3D(nx, ny, nz);

        // Map [-1, 1] → [0, 360) hue
        float hue = (noise + 1.0f) * 180.0f;
        if (hue < 0.0f)   hue = 0.0f;
        if (hue >= 360.0f) hue = 359.9f;
        return HSVtoRGB565(hue, 1.0f, 1.0f);
    }

    // ── PGL_MAT_IMAGE (0x40): Texture sampling with UV offset/scale ────
    case PGL_MAT_IMAGE: {
        const auto* p = reinterpret_cast<const PglParamImage*>(mat.params);
        // Nested texture reference: handle → slot index (generation byte
        // stays on the wire param; it was validated once at admission).
        const uint8_t texIdx = PglHandleIndex(p->textureId);
        if (texIdx >= GpuConfig::MAX_TEXTURES) return 0xF81F;

        float su = uv.x * p->scaleX + p->offsetX;
        float sv = uv.y * p->scaleY + p->offsetY;
        // filterFlags bit0 selects bilinear when UV-mapped.  It reads 0
        // (nearest) for hosts that sent the frozen 18-byte v8 form — the
        // parser zeroes the params tail.  With no UVs the constant
        // uv={0,0} resolves both filters to texel (0,0) identically.
        return SampleTexture(scene->textures[texIdx], su, sv,
                             (p->filterFlags & PGL_IMAGE_FILTER_BILINEAR) != 0);
    }

    // ── PGL_MAT_COMBINE (0x50): Blend two materials ────────────────────
    case PGL_MAT_COMBINE: {
        const auto* p = reinterpret_cast<const PglParamCombine*>(mat.params);
        const uint8_t idxA = PglHandleIndex(p->materialIdA);
        const uint8_t idxB = PglHandleIndex(p->materialIdB);
        if (idxA >= GpuConfig::MAX_MATERIALS ||
            idxB >= GpuConfig::MAX_MATERIALS) return 0xF81F;

        const MaterialSlot& matA = scene->materials[idxA];
        const MaterialSlot& matB = scene->materials[idxB];

        uint16_t colorA = matA.active
            ? EvaluateMaterial(matA, point, normal, uv, scene, elapsedTimeS, depth + 1, nullptr)
            : 0x0000;
        uint16_t colorB = matB.active
            ? EvaluateMaterial(matB, point, normal, uv, scene, elapsedTimeS, depth + 1, nullptr)
            : 0x0000;

        return BlendRGB565(colorA, colorB,
                           static_cast<PglBlendMode>(p->blendMode), p->opacity);
    }

    // ── PGL_MAT_MASK (0x51): Threshold-based masking ───────────────────
    case PGL_MAT_MASK: {
        const auto* p = reinterpret_cast<const PglParamMask*>(mat.params);
        const uint8_t idxBase = PglHandleIndex(p->baseMaterialId);
        const uint8_t idxMask = PglHandleIndex(p->maskMaterialId);
        if (idxBase >= GpuConfig::MAX_MATERIALS ||
            idxMask >= GpuConfig::MAX_MATERIALS) return 0xF81F;

        // Evaluate mask material as grayscale luminance
        const MaterialSlot& maskMat = scene->materials[idxMask];
        uint16_t maskColor = maskMat.active
            ? EvaluateMaterial(maskMat, point, normal, uv, scene, elapsedTimeS, depth + 1, nullptr)
            : 0x0000;

        uint8_t mr, mg, mb;
        UnpackRGB565(maskColor, mr, mg, mb);
        // Approximate luminance: (R + G + B) / 3
        float lum = (static_cast<float>(mr) + static_cast<float>(mg)
                    + static_cast<float>(mb)) / (3.0f * 255.0f);

        if (lum >= p->threshold) {
            const MaterialSlot& baseMat = scene->materials[idxBase];
            return baseMat.active
                ? EvaluateMaterial(baseMat, point, normal, uv, scene, elapsedTimeS, depth + 1, nullptr)
                : 0x0000;
        }
        // Masked out: DISCARD at the top level (no colour, no depth — a
        // failed mask never occludes).  Nested mask failures still
        // contribute black to their parent (outDiscard == nullptr there).
        if (outDiscard) *outDiscard = true;
        return 0x0000;
    }

    // ── PGL_MAT_ANIMATOR (0x52): Lerp between two materials ────────────
    case PGL_MAT_ANIMATOR: {
        const auto* p = reinterpret_cast<const PglParamAnimator*>(mat.params);
        const uint8_t idxA = PglHandleIndex(p->materialIdA);
        const uint8_t idxB = PglHandleIndex(p->materialIdB);
        if (idxA >= GpuConfig::MAX_MATERIALS ||
            idxB >= GpuConfig::MAX_MATERIALS) return 0xF81F;

        const MaterialSlot& matA = scene->materials[idxA];
        const MaterialSlot& matB = scene->materials[idxB];

        uint16_t colorA = matA.active
            ? EvaluateMaterial(matA, point, normal, uv, scene, elapsedTimeS, depth + 1, nullptr)
            : 0x0000;
        uint16_t colorB = matB.active
            ? EvaluateMaterial(matB, point, normal, uv, scene, elapsedTimeS, depth + 1, nullptr)
            : 0x0000;

        float ratio = PglMath::Clamp(p->ratio, 0.0f, 1.0f);

        // Lerp per-channel
        uint8_t rA, gA, bA, rB, gB, bB;
        UnpackRGB565(colorA, rA, gA, bA);
        UnpackRGB565(colorB, rB, gB, bB);

        uint8_t r = static_cast<uint8_t>(PglMath::Lerp(
            static_cast<float>(rA), static_cast<float>(rB), ratio));
        uint8_t g = static_cast<uint8_t>(PglMath::Lerp(
            static_cast<float>(gA), static_cast<float>(gB), ratio));
        uint8_t b = static_cast<uint8_t>(PglMath::Lerp(
            static_cast<float>(bA), static_cast<float>(bB), ratio));
        return PackRGB565(r, g, b);
    }

    // ── PGL_MAT_PRERENDERED (0xF0): Direct texture lookup ──────────────
    case PGL_MAT_PRERENDERED: {
        const auto* p = reinterpret_cast<const PglParamPreRendered*>(mat.params);
        const uint8_t texIdx = PglHandleIndex(p->textureId);
        if (texIdx >= GpuConfig::MAX_TEXTURES) return 0x8410;  // mid-grey fallback
        return SampleTexture(scene->textures[texIdx], uv.x, uv.y,
                             false);  // nearest — bilinear filtering is IMAGE-only
    }

    // ── Unknown material type ──────────────────────────────────────────
    default:
        return 0xF81F;  // magenta: easy to spot
    }
}

// ─── Initialize ─────────────────────────────────────────────────────────────

void Rasterizer::Initialize(SceneState* scene, PhaseScratch::DepthWorkspace& depth,
                            uint16_t width, uint16_t height) {
    this->scene = scene;
    this->depthWorkspace = &depth;
    depth.BeginDepth();
    this->width   = width;
    this->height  = height;
    this->projectedTriCount = 0;

    gridCols = (static_cast<uint32_t>(width)  + kTileW - 1) / kTileW;
    gridRows = (static_cast<uint32_t>(height) + kTileH - 1) / kTileH;
    gridValid = gridCols > 0 && gridRows > 0 &&
                gridCols * gridRows <= kMaxTileGridCells;
}

// ─── View-Space Projection Helpers ──────────────────────────────────────────
// The view transform mirrors ProtoTracer: view rotation =
// rotation ∘ baseRotation ∘ lookOffset (lookOffset applies first, in the
// camera frame), then component-wise camera scale.  For an identity
// lookOffset and identity scale every operation reduces bit-exactly to the
// historical rotation ∘ baseRotation pipeline (QuatMul by identity and
// multiply-by-1.0f are exact).

/// View-space point of a world-space vertex (camera rotation conjugate +
/// component camera scale — ProtoTracer UnrotateVector(p − pos) · scale).
static inline PglVec3 ViewSpacePoint(const PglVec3& world,
                                     const PglVec3& camPos,
                                     const PglQuat& viewRotConj,
                                     const PglVec3& camScale) {
    PglVec3 view = PglMath::QuatRotate(viewRotConj, PglMath::Sub(world, camPos));
    view.x *= camScale.x;
    view.y *= camScale.y;
    view.z *= camScale.z;
    return view;
}

/// Project an already view-space point to screen space.  Same expression
/// tree as the projection stage of PglMath::PerspectiveProject
/// (invZ = fovFactor / zDiv; x·invZ + centre); the divisor guard only
/// engages for the AABB probe corners at/behind the near plane (rasterized
/// vertices are clipped to z >= kNearPlaneZ, so the guard is inert for them).
static inline PglVec2 ProjectViewZ(const PglVec3& view, float fovFactor,
                                   float screenW, float screenH) {
    const float zDiv = (view.z < PglMath::kNearPlaneZ) ? PglMath::kNearPlaneZ
                                                       : view.z;
    const float invZ = fovFactor / zDiv;
    return {
        view.x * invZ + screenW * 0.5f,
        view.y * invZ + screenH * 0.5f
    };
}

/// Declared orthographic mapping: pixel-space (world − camPos)·camScale +
/// screen centre.  Identity camera scale is bit-identical to the historical
/// OrthoProject (multiply by 1.0f is exact).  Depth is the transformed world
/// z (handled by the caller); camera rotation/lookOffset do not apply to the
/// declared orthographic convention.
static inline PglVec2 OrthoProjectScaled(const PglVec3& world,
                                         const PglVec3& camPos,
                                         const PglVec3& camScale,
                                         float screenW, float screenH) {
    return {
        (world.x - camPos.x) * camScale.x + screenW * 0.5f,
        (world.y - camPos.y) * camScale.y + screenH * 0.5f
    };
}

// ─── Near-Plane Clipping (perspective) ──────────────────────────────────────
// Perspective triangles crossing the view-space near plane
// (z = PglMath::kNearPlaneZ) are Sutherland–Hodgman clipped BEFORE
// projection, emitting 1–2 triangles with per-vertex attributes (position
// and UV) linearly interpolated at the intersection points.  The face
// normal is a per-face attribute and is NOT interpolated.  After clipping,
// every emitted vertex has z ≥ kNearPlaneZ > 0, so the depths fed to the
// Z-buffer stay strictly positive.

/// One polygon corner for the near-plane clip: view-space position plus the
/// per-corner UV carried through the clip (only meaningful when the source
/// mesh provides UVs for the triangle).
struct NearClipVert {
    PglVec3 view;
    PglVec2 uv;
};

/// Sutherland–Hodgman clip of a view-space triangle against the near plane
/// z = PglMath::kNearPlaneZ, keeping the z > near half-space.  `in` is the
/// source triangle (3 corners); `out` receives up to 4 corners in the same
/// winding order.  Returns the output corner count.  Along a crossing edge
/// a→b the intersection parameter is t = (a.z − near)/(a.z − b.z) and x/y/uv
/// are interpolated with that same t; the intersection's z is set to
/// kNearPlaneZ exactly.
static uint8_t ClipTriangleNearPlane(const NearClipVert in[3], NearClipVert out[4]) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < 3; ++i) {
        const NearClipVert& a = in[i];
        const NearClipVert& b = in[(i + 1) % 3];
        const bool aIn = a.view.z > PglMath::kNearPlaneZ;
        const bool bIn = b.view.z > PglMath::kNearPlaneZ;
        if (aIn) {
            out[n++] = a;
        }
        if (aIn != bIn) {
            const float t = (a.view.z - PglMath::kNearPlaneZ) /
                            (a.view.z - b.view.z);
            NearClipVert isect;
            isect.view.x = a.view.x + t * (b.view.x - a.view.x);
            isect.view.y = a.view.y + t * (b.view.y - a.view.y);
            isect.view.z = PglMath::kNearPlaneZ;
            isect.uv.x   = a.uv.x + t * (b.uv.x - a.uv.x);
            isect.uv.y   = a.uv.y + t * (b.uv.y - a.uv.y);
            out[n++] = isect;
        }
    }
    return n;
}

/// Resolve a triangle's three corner UVs.  Returns false when the mesh has
/// no usable UVs for this triangle (defensive: indices and counts are
/// re-validated here even though admission checked them).
static bool ResolveCornerUVs(const MeshSlot& mesh, uint16_t triIndex, PglVec2 out[3]) {
    if (!mesh.uvVertices || !mesh.uvIndices || triIndex >= mesh.triangleCount)
        return false;
    const PglIndex3& uvIdx = mesh.uvIndices[triIndex];
    if (uvIdx.a >= mesh.uvVertexCount ||
        uvIdx.b >= mesh.uvVertexCount ||
        uvIdx.c >= mesh.uvVertexCount)
        return false;
    out[0] = mesh.uvVertices[uvIdx.a];
    out[1] = mesh.uvVertices[uvIdx.b];
    out[2] = mesh.uvVertices[uvIdx.c];
    return true;
}

// ─── Emit: cull → pool → coverage rectangle ─────────────────────────────────
// Shared emit tail for the preparation passes: back-face cull → finite/
// screen-AABB cull → pool allocate → Setup → attributes → clamped tile
// coverage rectangle. Silently drops the triangle ONLY when it is back-facing,
// non-finite, degenerate, or entirely off-screen (none of which is visible
// geometry).  A full pool is NOT silent: it drops the extra triangle and
// latches s_poolOverflow → GetFrameError() == RenderOverflow.

static bool EmitProjectedTriangle(const PglVec2& sa, const PglVec2& sb, const PglVec2& sc,
                                  float za, float zb, float zc,
                                  const PglVec3& nva, const PglVec3& nvb, const PglVec3& nvc,
                                  const PglVec2* uvs,
                                  uint16_t drawCallIndex, uint16_t meshTriIndex,
                                  uint8_t triFlags,
                                  uint16_t screenW, uint16_t screenH,
                                  uint32_t gridCols, uint32_t gridRows) {
    // Back-face culling: skip if the triangle has zero or negative area.
    // Written as !(area > 0) so NaN (non-finite projection) is culled too.
    const float area2d = PglMath::TriangleArea2D(sa, sb, sc);
    if (!std::isfinite(area2d) || !(area2d >= 0.5e-6f)) return false;

    PglMath::AABB2D bounds = PglMath::TriangleBounds2D(sa, sb, sc);
    if (!std::isfinite(bounds.minX) || !std::isfinite(bounds.minY) ||
        !std::isfinite(bounds.maxX) || !std::isfinite(bounds.maxY)) {
        return false;  // non-finite projection — never visible
    }

    // Frustum cull: skip if entirely off-screen
    if (bounds.maxX < 0.0f || bounds.minX >= static_cast<float>(screenW) ||
        bounds.maxY < 0.0f || bounds.minY >= static_cast<float>(screenH)) {
        return false;
    }

    // ── Allocate from the pool (overflow is EXPLICIT, never silent) ──
    if (trianglePoolUsed >= GpuConfig::MAX_TRIANGLES) {
        s_poolOverflow = true;
        return false;
    }
    Triangle2D& tri = trianglePool[trianglePoolUsed];
    if (!tri.Setup(sa, sb, sc, za, zb, zc)) {
        return false;  // degenerate (slot is reused by the next candidate)
    }

    tri.flags         = triFlags | (uvs ? uint8_t(Triangle2D::HAS_UV) : uint8_t(0));
    tri.drawCallIndex = drawCallIndex;
    tri.meshTriIndex  = meshTriIndex;

    // Face normal from the unclipped transformed 3D vertices (flat).
    tri.faceNormal = PglMath::Normalize(PglMath::TriangleNormal(nva, nvb, nvc));

    if (uvs) {
        tri.uv0 = uvs[0];
        tri.uv1 = uvs[1];
        tri.uv2 = uvs[2];
    }

    // Clamp AABB to screen bounds, then retain its inclusive tile rectangle.
    bounds.minX = (bounds.minX > 0.0f) ? bounds.minX : 0.0f;
    bounds.minY = (bounds.minY > 0.0f) ? bounds.minY : 0.0f;
    bounds.maxX = (bounds.maxX < static_cast<float>(screenW))  ? bounds.maxX : static_cast<float>(screenW);
    bounds.maxY = (bounds.maxY < static_cast<float>(screenH)) ? bounds.maxY : static_cast<float>(screenH);

    TileBounds coverage{0, 0, UINT8_MAX, UINT8_MAX}; // invalid grid: all candidates
    if (gridCols > 0 && gridRows > 0 && gridCols * gridRows <= kMaxTileGridCells) {
        uint32_t tx0 = static_cast<uint32_t>(bounds.minX) / kTileW;
        uint32_t ty0 = static_cast<uint32_t>(bounds.minY) / kTileH;
        uint32_t tx1 = static_cast<uint32_t>(bounds.maxX) / kTileW;
        uint32_t ty1 = static_cast<uint32_t>(bounds.maxY) / kTileH;
        if (tx1 >= gridCols) tx1 = gridCols - 1;
        if (ty1 >= gridRows) ty1 = gridRows - 1;
        if (tx0 >= gridCols) tx0 = gridCols - 1;
        if (ty0 >= gridRows) ty0 = gridRows - 1;
        coverage = {static_cast<uint8_t>(tx0), static_cast<uint8_t>(ty0),
                    static_cast<uint8_t>(tx1), static_cast<uint8_t>(ty1)};
    }
    triangleTileBounds[trianglePoolUsed] = coverage;
    ++trianglePoolUsed;
    return true;
}

// ─── PrepareFrame (core 0, single-threaded) ─────────────────────────────────
//
// Frame-level entry: reset of the per-frame pipeline state, then the legacy
// binding — the FIRST active camera whose target resolves to the back buffer
// gets its pass prepared.  Every frame is prepared in full: there is no
// frame-signature skip (the unsafe reuse optimization was removed outright —
// the integrator re-renders and re-presents every frame).

void Rasterizer::PrepareFrame(SceneState* scene) {
    this->scene = scene;
    projectedTriCount = 0;
    frameError        = PglRuntime::Result::Ok;

    // Default pass state: full-frame scissor, panel FB stride, empty pool.
    // With no valid camera pass the caller's tile pass clears the frame to
    // black (no stale two-frame-old content — the integrator also clears
    // the primary back target at frame start).
    fbStride       = width;
    targetWidth = width; targetHeight = height;
    scX0 = 0; scY0 = 0; scX1 = width; scY1 = height;
    preparedCamIdx = -1;
    nextCamCursor  = 0;
    passHasTranslucent = false;
    trianglePoolUsed   = 0;
    s_poolOverflow     = false;

    if (!scene || !depthWorkspace || !width || !height ||
        static_cast<uint32_t>(width) * height > GpuConfig::FRAMEBUF_PIXELS) {
        RecordFrameError(PglRuntime::Result::InvalidValue);
        return; // No world-scratch lifetime has started; depth remains active.
    }

    gridCols = (static_cast<uint32_t>(width)  + kTileW - 1) / kTileW;
    gridRows = (static_cast<uint32_t>(height) + kTileH - 1) / kTileH;
    gridValid = gridCols > 0 && gridRows > 0 &&
                gridCols * gridRows <= kMaxTileGridCells;
    if (!gridValid) {
        // Unsupported panel/tile geometry — coverage falls back to the
        // lossless analytic path, but the frame is flagged failed.
        RecordFrameError(PglRuntime::Result::InvalidValue);
    }

    // First ACTIVE camera with a valid BACK-BUFFER target.  Cameras
    // targeting layers are prepared by PrepareNextCameraPass().
    for (uint8_t c = 0; c < PGL_MAX_CAMERAS; ++c) {
        if (!scene->cameras[c].active) continue;
        if (scene->cameras[c].targetLayer != PGL_LAYER_3D) {
            continue;                        // layer-bound → camera loop
        }
        CameraTargetInfo ti =
            scene->ResolveCameraTarget(c, nullptr, width, height);
        if (!ti.valid) continue;             // destroyed target → fail-closed
        PrepareCameraPass(scene, c, ti);
        preparedCamIdx = c;
        break;
    }
}

// ─── PrepareNextCameraPass ──────────────────────────────────────────────────
//
// Iterates the remaining ACTIVE cameras in slot order, skipping the one
// PrepareFrame already bound and cameras whose target no longer resolves
// (fail-closed).  Each accepted camera gets a freshly prepared pass: its own
// Z clear + coverage + projection, its own viewport scissor + FB stride.

bool Rasterizer::PrepareNextCameraPass(SceneState* scene, uint8_t* outCamIdx) {
    for (uint8_t c = nextCamCursor; c < PGL_MAX_CAMERAS; ++c) {
        nextCamCursor = static_cast<uint8_t>(c + 1);
        if (c == static_cast<uint8_t>(preparedCamIdx)) continue;  // already bound
        if (!scene->cameras[c].active) continue;
        CameraTargetInfo ti =
            scene->ResolveCameraTarget(c, nullptr, width, height);
        if (!ti.valid) continue;             // destroyed target → fail-closed
        PrepareCameraPass(scene, c, ti);
        if (outCamIdx) *outCamIdx = c;
        return true;
    }
    return false;
}

// ─── PrepareCameraPass (per-camera: transform → clip → project → coverage) ──
//
// Binds the pass's target geometry (FB stride + viewport scissor), borrows
// retired depth storage for world vertices, and runs the full projection
// pipeline. Before returning it retires world vertices and clears the fresh
// depth plane, ready for both tile workers and target-strided addressing.
// The tile pass between cameras drains both cores, so their projected pool
// and the two preparation workspaces are safely reused per pass.

void Rasterizer::PrepareCameraPass(SceneState* scene, uint8_t camIdx,
                                   const CameraTargetInfo& target) {
    this->scene = scene;

    // ── Bind pass state: target FB stride + viewport scissor ────────────
    // The scissor is a clip, not a projection change.  Projection remains
    // full-frame panel space; coverage and addressing use actual target
    // extents, and RasterizeTile intersects with this scissor.
    fbStride = targetWidth = target.width;
    targetHeight = target.height;
    scX0 = target.scX0; scY0 = target.scY0;
    scX1 = target.scX1; scY1 = target.scY1;
    passHasTranslucent = false;
    gridCols = (static_cast<uint32_t>(targetWidth) + kTileW - 1) / kTileW;
    gridRows = (static_cast<uint32_t>(targetHeight) + kTileH - 1) / kTileH;
    gridValid = gridCols > 0 && gridRows > 0 &&
                gridCols * gridRows <= kMaxTileGridCells;
    trianglePoolUsed = 0;
    const uint32_t pixelCount = static_cast<uint32_t>(targetWidth) * targetHeight;
    if (!depthWorkspace || pixelCount > GpuConfig::FRAMEBUF_PIXELS) {
        RecordFrameError(PglRuntime::Result::InvalidValue);
        return; // Pixel-capacity rejection precedes every fixed-workspace touch.
    }
    if (!gridValid) RecordFrameError(PglRuntime::Result::InvalidValue);
    PhaseScratch::Lease<PhaseScratch::ViewVertices> viewScratch;
    if (!viewScratch) {
        RecordFrameError(PglRuntime::Result::Busy);
        return;
    }
    DepthPreparation worldScratch(*depthWorkspace, pixelCount);
    PglVec3* const transformedVerts = worldScratch.vertices;
    PglVec3* const viewVerts = viewScratch->values;

    const CameraSlot* activeCam = &scene->cameras[camIdx];

    // Camera parameters — full explicit composition (ProtoTracer order):
    //   view rotation = rotation ∘ baseRotation ∘ lookOffset
    //   view point    = (rotation⁻¹ · (world − position)) · scale (component-wise)
    const PglVec3& camPos   = activeCam->position;
    const PglVec3& camScale = activeCam->scale;
    const PglQuat  viewRot  = PglMath::QuatMul(
        PglMath::QuatMul(activeCam->rotation, activeCam->baseRotation),
        activeCam->lookOffset);
    const bool     is2D     = activeCam->is2D;

    // Conjugate of the view rotation (exact — sign flips only).
    const PglQuat  viewRotConj = PglMath::QuatConjugate(viewRot);

    // FOV factor for perspective projection (panel-width-derived, declared).
    const float fovFactor = static_cast<float>(width) * 0.5f;

    // Near-plane threshold for the conservative AABB classification:
    // perspective culls/clips at view z = kNearPlaneZ; the declared
    // orthographic path culls triangles at world z ≤ 0 per triangle.
    const float nearZ = is2D ? 0.0f : PglMath::kNearPlaneZ;

    // ── Process each draw call ──────────────────────────────────────────
    for (uint16_t d = 0; d < scene->drawCallCount; ++d) {
        const DrawCall& dc = scene->drawList[d];
        if (!dc.enabled) continue;
        if (dc.meshId >= GpuConfig::MAX_MESHES) continue;

        const MeshSlot& mesh = scene->meshes[dc.meshId];
        if (!mesh.active) continue;
        if (mesh.vertexCount == 0 || mesh.triangleCount == 0) continue;

        // Source vertices: override (morph) vertices if present, else the
        // mesh slot's base vertices.
        const PglVec3* srcVerts = mesh.vertices;
        uint16_t vertCount = mesh.vertexCount;
        if (dc.hasVertexOverride && dc.overrideVertices) {
            srcVerts = dc.overrideVertices;
            vertCount = dc.overrideVertexCount;
        }

        // Clamp to buffer capacity (admission bounds this; defensive).
        if (vertCount > GpuConfig::MAX_VERTICES) {
            vertCount = GpuConfig::MAX_VERTICES;
        }

        // Compose this draw call's rotation ONCE — invariant across all
        // AABB corners and vertices below.
        const PglTransform& xform = dc.transform;
        const PglQuat drawFullRot = PglMath::TransformFullRotation(xform);

        // Translucent classification (pixel-invariant per draw call).
        bool drawTranslucent = false;
        if (dc.materialId < GpuConfig::MAX_MATERIALS) {
            const MaterialSlot& m = scene->materials[dc.materialId];
            drawTranslucent = m.active && m.blendMode == PGL_BLEND_ALPHA;
        }

        // ── Mesh-level conservative AABB cull ───────────────────────────
        // Morph overrides can move geometry ANYWHERE, so their bounds come
        // from the override vertices — never from the base mesh AABB (the
        // old base-bounds test could cull visible morphed geometry).
        // Near-plane-crossing draws are kept: the screen-AABB cull applies
        // only when ALL 8 corners are in front of the near threshold (the
        // projected image of a fully-in-front convex box is the convex hull
        // of its projected corners — a valid superset of every triangle's
        // image).  Mixed draws (some corners at/behind the near threshold)
        // proceed to the per-triangle near clip.
        PglVec3 aabbMn, aabbMx;
        if (dc.hasVertexOverride && dc.overrideVertices && vertCount > 0) {
            aabbMn = aabbMx = srcVerts[0];
            for (uint16_t v = 1; v < vertCount; ++v) {
                const PglVec3& p = srcVerts[v];
                if (p.x < aabbMn.x) aabbMn.x = p.x;
                if (p.y < aabbMn.y) aabbMn.y = p.y;
                if (p.z < aabbMn.z) aabbMn.z = p.z;
                if (p.x > aabbMx.x) aabbMx.x = p.x;
                if (p.y > aabbMx.y) aabbMx.y = p.y;
                if (p.z > aabbMx.z) aabbMx.z = p.z;
            }
        } else {
            aabbMn = mesh.aabbMin;
            aabbMx = mesh.aabbMax;
        }
        {
            const PglVec3& mn = aabbMn;
            const PglVec3& mx = aabbMx;
            const PglVec3 corners[8] = {
                {mn.x, mn.y, mn.z}, {mx.x, mn.y, mn.z},
                {mn.x, mx.y, mn.z}, {mx.x, mx.y, mn.z},
                {mn.x, mn.y, mx.z}, {mx.x, mn.y, mx.z},
                {mn.x, mx.y, mx.z}, {mx.x, mx.y, mx.z},
            };
            float sMinX = 1e30f, sMinY = 1e30f;
            float sMaxX = -1e30f, sMaxY = -1e30f;
            bool screenCullValid = true;
            uint8_t frontCount = 0;
            for (int c = 0; c < 8; ++c) {
                const PglVec3 tv = PglMath::TransformVertex(xform, drawFullRot, corners[c]);
                float cz;
                PglVec2 sp;
                if (is2D) {
                    sp = OrthoProjectScaled(tv, camPos, camScale,
                                            static_cast<float>(width),
                                            static_cast<float>(height));
                    cz = tv.z;
                } else {
                    const PglVec3 vv = ViewSpacePoint(tv, camPos, viewRotConj, camScale);
                    cz = vv.z;
                    sp = ProjectViewZ(vv, fovFactor,
                                      static_cast<float>(width),
                                      static_cast<float>(height));
                }
                if (cz > nearZ) {
                    ++frontCount;
                    if (std::isfinite(sp.x) && std::isfinite(sp.y)) {
                        if (sp.x < sMinX) sMinX = sp.x;
                        if (sp.y < sMinY) sMinY = sp.y;
                        if (sp.x > sMaxX) sMaxX = sp.x;
                        if (sp.y > sMaxY) sMaxY = sp.y;
                    } else {
                        screenCullValid = false;  // keep the draw — never
                                                  // cull from non-finite bounds
                    }
                }
            }
            // Fully at/behind the near threshold → nothing visible.
            if (frontCount == 0) continue;
            // Fully in front AND projected AABB off-screen → cull.  Mixed
            // (near-crossing) or non-finite bounds → keep, conservatively.
            if (frontCount == 8 && screenCullValid &&
                (sMaxX < 0.0f || sMinX >= static_cast<float>(targetWidth) ||
                 sMaxY < 0.0f || sMinY >= static_cast<float>(targetHeight))) {
                continue;
            }
        }

        // ── Step 1: transform vertices by the DrawCall's transform ──────
        for (uint16_t v = 0; v < vertCount; ++v) {
            transformedVerts[v] = PglMath::TransformVertex(xform, drawFullRot, srcVerts[v]);
        }

        // View-space vertices (perspective only): rotation conjugate +
        // component camera scale.  Same op sequence as the view transform
        // inside PglMath::PerspectiveProject for identity scale, so these
        // depths/projections are bit-identical to the historical pipeline.
        if (!is2D) {
            for (uint16_t v = 0; v < vertCount; ++v) {
                viewVerts[v] = ViewSpacePoint(transformedVerts[v], camPos,
                                              viewRotConj, camScale);
            }
        }

        // ── Step 2: project each triangle to 2D and pool it ─────────────
        const PglIndex3* indices = mesh.indices;
        const uint8_t baseFlags = drawTranslucent ? uint8_t(Triangle2D::TRANSLUCENT) : uint8_t(0);
        bool emittedTranslucent = false;

        for (uint16_t t = 0; t < mesh.triangleCount; ++t) {
            const PglIndex3& idx = indices[t];
            // Bounds check indices (defensive — admission validates these)
            if (idx.a >= vertCount || idx.b >= vertCount || idx.c >= vertCount)
                continue;

            const PglVec3& va = transformedVerts[idx.a];
            const PglVec3& vb = transformedVerts[idx.b];
            const PglVec3& vc = transformedVerts[idx.c];

            PglVec2 cornerUVs[3] = {};
            const bool hasUV = ResolveCornerUVs(mesh, t, cornerUVs);

            if (is2D) {
                // Declared orthographic path: pixel-space XY, world-z depth,
                // no clip (no projection singularity) — triangles with any
                // vertex z ≤ 0 are dropped whole.
                const float za = va.z, zb = vb.z, zc = vc.z;
                if (za <= 0.0f || zb <= 0.0f || zc <= 0.0f) continue;

                const PglVec2 sa = OrthoProjectScaled(va, camPos, camScale,
                                                      static_cast<float>(width),
                                                      static_cast<float>(height));
                const PglVec2 sb = OrthoProjectScaled(vb, camPos, camScale,
                                                      static_cast<float>(width),
                                                      static_cast<float>(height));
                const PglVec2 sc = OrthoProjectScaled(vc, camPos, camScale,
                                                      static_cast<float>(width),
                                                      static_cast<float>(height));
                if (EmitProjectedTriangle(sa, sb, sc, za, zb, zc, va, vb, vc,
                                          hasUV ? cornerUVs : nullptr,
                                          d, t, baseFlags, targetWidth, targetHeight,
                                          gridValid ? gridCols : 0,
                                          gridValid ? gridRows : 0)) {
                    emittedTranslucent = emittedTranslucent || drawTranslucent;
                }
                continue;
            }

            // ── Perspective: classify against the near plane in view space ──
            const PglVec3& wva = viewVerts[idx.a];
            const PglVec3& wvb = viewVerts[idx.b];
            const PglVec3& wvc = viewVerts[idx.c];
            const int frontCount =
                (wva.z > PglMath::kNearPlaneZ ? 1 : 0) +
                (wvb.z > PglMath::kNearPlaneZ ? 1 : 0) +
                (wvc.z > PglMath::kNearPlaneZ ? 1 : 0);

            if (frontCount == 0) {
                continue;  // fully at/behind the near plane — drop
            }

            if (frontCount < 3) {
                // Crossing: Sutherland–Hodgman clip of the view-space
                // polygon against z = kNearPlaneZ, emitting 1–2
                // sub-triangles with interpolated UVs.  Pool capacity is
                // checked per emit — a full pool drops the EXTRA triangle
                // only and latches the frame error.
                NearClipVert inV[3];
                inV[0].view = wva;  inV[0].uv = cornerUVs[0];
                inV[1].view = wvb;  inV[1].uv = cornerUVs[1];
                inV[2].view = wvc;  inV[2].uv = cornerUVs[2];

                NearClipVert outV[4];
                const uint8_t outN = ClipTriangleNearPlane(inV, outV);

                // Fan-triangulate the clipped polygon (3 or 4 corners),
                // preserving the original winding.
                const float w = static_cast<float>(width);
                const float h = static_cast<float>(height);
                for (uint8_t k = 0; k + 2 < outN; ++k) {
                    const NearClipVert& p0 = outV[0];
                    const NearClipVert& p1 = outV[k + 1];
                    const NearClipVert& p2 = outV[k + 2];
                    const PglVec2 cuv[3] = { p0.uv, p1.uv, p2.uv };
                    if (EmitProjectedTriangle(
                            ProjectViewZ(p0.view, fovFactor, w, h),
                            ProjectViewZ(p1.view, fovFactor, w, h),
                            ProjectViewZ(p2.view, fovFactor, w, h),
                            p0.view.z, p1.view.z, p2.view.z,
                            va, vb, vc,
                            hasUV ? cuv : nullptr,
                            d, t,
                            baseFlags | Triangle2D::PERSPECTIVE,
                            targetWidth, targetHeight,
                            gridValid ? gridCols : 0,
                            gridValid ? gridRows : 0)) {
                        emittedTranslucent = emittedTranslucent || drawTranslucent;
                    }
                }
                continue;
            }

            // Fully in front — unchanged projection path (view z >
            // kNearPlaneZ for all three by the classification above).
            const PglVec2 sa = ProjectViewZ(wva, fovFactor,
                                            static_cast<float>(width),
                                            static_cast<float>(height));
            const PglVec2 sb = ProjectViewZ(wvb, fovFactor,
                                            static_cast<float>(width),
                                            static_cast<float>(height));
            const PglVec2 sc = ProjectViewZ(wvc, fovFactor,
                                            static_cast<float>(width),
                                            static_cast<float>(height));
            if (EmitProjectedTriangle(sa, sb, sc, wva.z, wvb.z, wvc.z,
                                      va, vb, vc,
                                      hasUV ? cornerUVs : nullptr,
                                      d, t,
                                      baseFlags | Triangle2D::PERSPECTIVE,
                                      targetWidth, targetHeight,
                                      gridValid ? gridCols : 0,
                                      gridValid ? gridRows : 0)) {
                emittedTranslucent = emittedTranslucent || drawTranslucent;
            }
        }

        if (emittedTranslucent) passHasTranslucent = true;
        projectedTriCount += mesh.triangleCount;

        // Bounded preparation slice — core 0's host-service callback.
        if (prepService) prepService();
    }

    if (s_poolOverflow) {
        RecordFrameError(PglRuntime::Result::RenderOverflow);
    }

#if GPU_CONFIG_DEBUG_PREPARE_PRINT
    printf("[Rasterizer] pass cam=%d: %u draw calls, %u src tris, %u pooled%s\n",
           (int)camIdx, scene->drawCallCount, (unsigned)projectedTriCount,
           (unsigned)trianglePoolUsed,
           s_poolOverflow ? " OVERFLOW" : "");
#endif
}

// ─── RasterizeTile (both cores, tile-parallel) ─────────────────────────────
//
// Two passes per tile, BOTH in stable submission (pool) order:
//   1. Opaque: strict-depth test (uint16 z, LESS wins; exact ties go to the
//      first-submitted triangle) and depth write.  No hidden front-to-back
//      reordering — submission order is the declared tie-break.
//   2. Translucent (PGL_BLEND_ALPHA only, and only when the pass emitted
//      any): depth test against the finished opaque depth buffer, source-
//      over blend in submission order, NO depth writes.  alpha ≤ 0
//      triangles and mask-discarded pixels write nothing — they never
//      occlude.  Hosts draw translucent geometry back-to-front (painter's
//      algorithm; alpha == 1 is bit-identical to the opaque write).
//
// Candidate selection is lossless: every pooled triangle carries inclusive
// tile bounds; a tile visits exactly the former mask's candidates in emission
// order (no capped lists, no QuadTree truncation). When the coverage grid is
// unavailable (non-16×16 tile call or unsupported panel geometry) the
// same selection falls back to an analytic AABB overlap test — still
// lossless.

struct TilePassCtx {
    uint16_t* fb;
    uint16_t* zBuf;
    const SceneState* scene;
    uint16_t px0, py0, px1, py1;   // scissored tile rect (exclusive end)
    uint16_t fbStride;             // target FB row stride (pixels)
    uint16_t zStride;              // actual target width (shared Z workspace)
    uint16_t tileX, tileY;         // coverage-grid coordinates (useBounds only)
    bool useBounds;
    float    elapsedTimeS;
};

static void RasterizeTilePass(const TilePassCtx& ctx, bool alphaPass,
                              void (*service)()) {
    const uint16_t triCount = trianglePoolUsed;

    for (uint16_t h = 0; h < triCount; ++h) {
        const Triangle2D& tri = trianglePool[h];
        const bool translucent = (tri.flags & Triangle2D::TRANSLUCENT) != 0;
        if (translucent != alphaPass) continue;

        // Lossless tile-coverage test
        if (ctx.useBounds) {
            const TileBounds& bounds = triangleTileBounds[h];
            if (ctx.tileX < bounds.x0 || ctx.tileX > bounds.x1 ||
                ctx.tileY < bounds.y0 || ctx.tileY > bounds.y1) continue;
        } else {
            const float minX = fminf(tri.v0.x, fminf(tri.v1.x, tri.v2.x));
            const float maxX = fmaxf(tri.v0.x, fmaxf(tri.v1.x, tri.v2.x));
            const float minY = fminf(tri.v0.y, fminf(tri.v1.y, tri.v2.y));
            const float maxY = fmaxf(tri.v0.y, fmaxf(tri.v1.y, tri.v2.y));
            if (maxX < static_cast<float>(ctx.px0) ||
                minX >= static_cast<float>(ctx.px1) ||
                maxY < static_cast<float>(ctx.py0) ||
                minY >= static_cast<float>(ctx.py1)) continue;
        }

        // Material resolve — defensive index validation (the pool is
        // prepared by core 0 and only read here; state cannot mutate
        // mid-pass, but a corrupt index must never reach memory).
        if (tri.drawCallIndex >= ctx.scene->drawCallCount) continue;
        const DrawCall& dc = ctx.scene->drawList[tri.drawCallIndex];
        if (dc.materialId >= GpuConfig::MAX_MATERIALS) continue;
        const MaterialSlot& mat = ctx.scene->materials[dc.materialId];

        float alpha = 1.0f;
        if (alphaPass) {
            // Emit classified this triangle active+ALPHA; revalidate the
            // slot defensively and skip non-positive/NaN alpha outright
            // (the blend is a bit-exact identity there — no colour change,
            // and with no depth write the surface never occludes).
            if (!mat.active || mat.blendMode != PGL_BLEND_ALPHA) continue;
            alpha = mat.alpha;
            if (!(alpha > 0.0f)) continue;
        }

        // ── Per-(triangle, tile) derivation: edge coefficients, top-left
        // inclusion bits, reciprocal depths and perspective UV weights —
        // once per triangle per tile, outside the pixel loops.
        Triangle2D::Deriv d;
        tri.Derive(d);

        // Hi-Z bound: this triangle's closest vertex depth in z-buffer
        // units (valid lower bound for both affine and 1/z-interpolated
        // depth: z(p) ≥ min(z0,z1,z2) for barycentric p inside).
        float triMinZ = tri.z0;
        if (tri.z1 < triMinZ) triMinZ = tri.z1;
        if (tri.z2 < triMinZ) triMinZ = tri.z2;
        const uint16_t triMinZU16 = FloatZToU16(triMinZ);

        const bool persp = (tri.flags & Triangle2D::PERSPECTIVE) != 0;
        const bool hasUV = (tri.flags & Triangle2D::HAS_UV)      != 0;
        const float v2x = tri.v2.x;
        const float v2y = tri.v2.y;

        for (uint16_t y = ctx.py0; y < ctx.py1; ++y) {
            // Both FB and Z rows use the actual pass target width.
            const uint32_t fbRow = static_cast<uint32_t>(y) * ctx.fbStride;
            const uint32_t zRow  = static_cast<uint32_t>(y) * ctx.zStride;
            const float py = static_cast<float>(y) + 0.5f;  // pixel centre

            // Row-invariant edge terms.
            const float dy   = py - v2y;
            const float rowU = d.e21x * dy;
            const float rowV = d.e02x * dy;
            const float rowW = d.e10x * (py - tri.v0.y);

            for (uint16_t x = ctx.px0; x < ctx.px1; ++x) {
                // Hi-Z early-out against the current z-buffer value.
                const uint16_t curZU16 = ctx.zBuf[zRow + x];
                if (triMinZU16 >= curZU16) continue;

                const float px = static_cast<float>(x) + 0.5f;  // pixel centre
                const float dx = px - v2x;

                // Barycentric coordinates with the top-left shared-edge
                // coverage rule: a pixel exactly on an edge is covered iff
                // that edge is a top or left edge (d.inc bits).  NaN-safe:
                // every comparison is false for NaN, so non-finite
                // barycentrics are never covered.
                const float edgeU = fmaf(d.e10y, dx, rowU);
                const float edgeV = fmaf(d.e20y, dx, rowV);
                const float edgeW = fmaf(d.e01y, px - tri.v0.x, rowW);
                const bool covered =
                    ((edgeU > 0.0f) || (edgeU == 0.0f && (d.inc & 0x01))) &&
                    ((edgeV > 0.0f) || (edgeV == 0.0f && (d.inc & 0x02))) &&
                    ((edgeW > 0.0f) || (edgeW == 0.0f && (d.inc & 0x04)));
                if (!covered) continue;
                const float u = edgeU * d.invDenom;
                const float v = edgeV * d.invDenom;
                const float w = 1.0f - u - v;

                // Depth — perspective-correct (1/z linear) or declared
                // orthographic affine.
                float z;
                if (persp) {
                    const float rz = fmaf(u, d.rz0, fmaf(v, d.rz1, w * d.rz2));
                    z = 1.0f / rz;
                } else {
                    z = fmaf(u, tri.z0, fmaf(v, tri.z1, w * tri.z2));
                }
                if (!(z > 0.0f) || !std::isfinite(z)) continue;
                const uint16_t zU16 = FloatZToU16(z);
                if (zU16 >= curZU16) continue;  // strict LESS — first submission wins ties

                // Colour
                uint16_t color;
                if (!mat.active) {
                    color = 0xFFFF;  // no material → solid white
                } else {
                    // UV — perspective-correct (uv/z linear, reconstructed
                    // with the true z) or declared orthographic affine
                    // (same expression tree as the historical path).
                    PglVec2 uv = {0.0f, 0.0f};
                    if (hasUV) {
                        if (persp) {
                            uv.x = fmaf(u, d.wu0, fmaf(v, d.wu1, w * d.wu2)) * z;
                            uv.y = fmaf(u, d.wv0, fmaf(v, d.wv1, w * d.wv2)) * z;
                        } else {
                            uv.x = fmaf(u, tri.uv0.x, fmaf(v, tri.uv1.x, w * tri.uv2.x));
                            uv.y = fmaf(u, tri.uv0.y, fmaf(v, tri.uv1.y, w * tri.uv2.y));
                        }
                    }

                    // Noise/gradient use the screen-space position, Light/
                    // Normal use the stored face normal.
                    const PglVec3 intersectionPoint = {px, py, z};
                    bool discard = false;
                    color = EvaluateMaterial(mat, intersectionPoint,
                                             tri.faceNormal, uv,
                                             ctx.scene, ctx.elapsedTimeS, 0,
                                             &discard);
                    if (discard) continue;  // mask-fail: no colour, no
                                            // depth — never occludes
                }

                if (alphaPass) {
                    // Source-over against the completed opaque scene (or
                    // background); NO depth write for translucent surfaces.
                    ctx.fb[fbRow + x] =
                        BlendAlphaRGB565(color, ctx.fb[fbRow + x], alpha);
                } else {
                    ctx.fb[fbRow + x] = color;
                    ctx.zBuf[zRow + x] = zU16;
                }
            }
        }

        // Bounded per-triangle slice — core 0's host-service callback.
        if (service) service();
    }
}

void Rasterizer::RasterizeTile(uint16_t* framebuffer, uint16_t* zBuf,
                               uint16_t tileX, uint16_t tileY,
                               uint16_t tileW, uint16_t tileH,
                               void (*service)()) {
    // Tile rect in pixels, intersected with the pass viewport scissor AND
    // actual target extents (both framebuffer and Z are target-strided).
    // The scissor is a clip, not a projection change.  u32 tile products
    // handle out-of-range callers without wrapping (fail-closed clamps).
    const uint32_t tx0 = static_cast<uint32_t>(tileX) * tileW;
    const uint32_t ty0 = static_cast<uint32_t>(tileY) * tileH;
    uint32_t tx1 = tx0 + tileW;
    uint32_t ty1 = ty0 + tileH;
    if (tx1 > targetWidth)  tx1 = targetWidth;
    if (ty1 > targetHeight) ty1 = targetHeight;

    const uint32_t px0u = (tx0 > scX0) ? tx0 : scX0;
    const uint32_t py0u = (ty0 > scY0) ? ty0 : scY0;
    const uint32_t px1u = (tx1 < scX1) ? tx1 : scX1;
    const uint32_t py1u = (ty1 < scY1) ? ty1 : scY1;
    if (px0u >= px1u || py0u >= py1u) return;

    const uint16_t px0 = static_cast<uint16_t>(px0u);
    const uint16_t py0 = static_cast<uint16_t>(py0u);
    const uint16_t px1 = static_cast<uint16_t>(px1u);
    const uint16_t py1 = static_cast<uint16_t>(py1u);

    // Clear the scissored tile band to black — covered pixels are
    // overwritten below.  FB rows use the pass target stride.
    for (uint16_t y = py0; y < py1; ++y) {
        const uint32_t rowStart = static_cast<uint32_t>(y) * fbStride;
        std::memset(&framebuffer[rowStart + px0], 0,
                    (px1 - px0) * sizeof(uint16_t));
    }
    if (service) service();  // bounded clear-band slice

    if (trianglePoolUsed == 0) return;

    // For standard tiles, use the exact same inclusive AABB rectangle as the
    // former mask. Nonstandard tile sizes retain the analytic lossless path.
    const bool useBounds = gridValid && tileW == kTileW && tileH == kTileH &&
                           tileX < gridCols && tileY < gridRows;

    TilePassCtx ctx{
        framebuffer, zBuf, scene,
        px0, py0, px1, py1,
        fbStride, targetWidth,
        tileX, tileY, useBounds, elapsedTimeS
    };

    // Pass 1: opaque (strict-depth write), submission order.
    RasterizeTilePass(ctx, false, service);

    // Pass 2: translucent source-over, submission order, no depth writes.
    if (passHasTranslucent) {
        RasterizeTilePass(ctx, true, service);
    }
}
