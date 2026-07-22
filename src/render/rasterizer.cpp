/**
 * @file rasterizer.cpp
 * @brief GPU-side rasterizer implementation — full M4 pipeline + M6 materials.
 *
 * Milestone progress:
 *   M0-M1: Skeleton compiles, PrepareFrame/RasterizeRange were stubs.
 *   M3:    Scene state and math library complete.
 *   M4:    Full vertex transform, perspective project, QuadTree build,
 *          per-pixel rasterization with barycentric Z-test and material eval.
 *   M5:    Frame caching, DWT profiling, HUB75 polling.
 *   M6:    Full 12-type material system: Simple, Normal, Depth, Gradient,
 *          Light, SimplexNoise, RainbowNoise, Image, Combine (12 blend modes),
 *          Mask, Animator, PreRendered.
 *   V9-G4: PGL_BLEND_ALPHA materials render in a deferred translucent pass
 *          (source-over blend over the finished opaque scene, painter's
 *          algorithm — no OIT; alpha == 1.0f is bit-identical to opaque).
 *   V9-G6: bilinear texture filtering for PGL_MAT_IMAGE (opt-in per
 *          material via PglParamImage::filterFlags; nearest stays default).
 *   V9-G3/G7: multi-camera + render-to-layer.  PrepareFrame keeps the v8
 *          legacy binding (first valid back-buffer camera) so an unchanged
 *          caller renders bit-identically; PrepareNextCameraPass iterates
 *          the remaining active cameras, each into its own target (layer FB
 *          or back buffer) with a viewport scissor on the tile loop.
 */

#include "rasterizer.h"
#include "../scene_state.h"
#include "../gpu_config.h"
#include "../math/pgl_math.h"
#include "triangle2d.h"
#include "quadtree.h"

#include <cstring>
#include <cstdio>
#include <cmath>

// ─── Debug Options ───────────────────────────────────────────────────────────
// GPU_CONFIG_DEBUG_PREPARE_PRINT: when 1, PrepareFrame prints a per-frame
// summary line over UART.  At 115200 baud one line costs ~1 ms+ on-device —
// hot path, keep 0 unless actively debugging the transform stage.
#ifndef GPU_CONFIG_DEBUG_PREPARE_PRINT
#define GPU_CONFIG_DEBUG_PREPARE_PRINT 0
#endif

// ─── FNV-1a hash helper ────────────────────────────────────────────────────
// Used for frame signature caching — identical draw list + mesh versions +
// cameras = skip raster.

static uint32_t fnv1a_hash(const void* data, size_t len, uint32_t seed = 0x811c9dc5u) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t h = seed;
    for (size_t i = 0; i < len; ++i) {
        h ^= p[i];
        h *= 0x01000193u;
    }
    return h;
}

// ─── Static Triangle2D Pool ────────────────────────────────────────────────
// Pre-allocated pool for projected triangles each frame.
// Max triangles across all draw calls = MAX_TRIANGLES.
// Size: 1024 × ~96 bytes ≈ 96 KB (accounts for faceNormal field addition).

static Triangle2D trianglePool[GpuConfig::MAX_TRIANGLES];
static uint16_t   trianglePoolUsed = 0;

// Scratch buffer for per-draw-call transformed vertices.
// We transform into this buffer before projecting, then discard.
static PglVec3 transformedVerts[GpuConfig::MAX_VERTICES];

// G5: view-space depth (z) per transformed vertex, filled per draw call for
// perspective cameras.  Used to classify triangles against the near plane
// before projection (see PrepareFrame).  Same op sequence as the view
// transform inside PglMath::PerspectiveProject, so these depths are
// bit-identical to the za/zb/zc the projection itself produces.
static float viewZScratch[GpuConfig::MAX_VERTICES];

// QuadTree instance — rebuilt every frame by PrepareFrame().
static QuadTree quadTree;

// ─── uint16_t Z-buffer Conversion ───────────────────────────────────────
// The Z-buffer uses uint16_t to save 16 KB SRAM (vs float).
// We exploit the fact that IEEE 754 positive floats are order-preserving
// when their bit-pattern is interpreted as unsigned integers.  Taking the
// upper 16 bits of the float gives us a compact representation that
// preserves the depth ordering perfectly for all positive z values.
//
// Precision: sign(1) + exponent(8) + top-7-mantissa = 16 bits.
// This gives 128 discrete depth levels per power-of-two range, which is
// more than sufficient for a 128×64 panel.

/// Convert a positive float depth to a sortable uint16_t.
/// Preserves ordering: closer (smaller z) → smaller uint16_t value.
static inline uint16_t FloatZToU16(float z) {
    uint32_t bits;
    __builtin_memcpy(&bits, &z, 4);
    return static_cast<uint16_t>(bits >> 16);
}

/// Far-plane sentinel: FloatZToU16(1e30f).
static constexpr uint16_t Z_FAR_U16 = 0x720D;  // upper 16 bits of 1e30f

// ─── RGB565 Utility Functions ───────────────────────────────────────────────

static inline uint16_t PackRGB565(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static inline void UnpackRGB565(uint16_t c, uint8_t& r, uint8_t& g, uint8_t& b) {
    r = static_cast<uint8_t>(((c >> 11) & 0x1F) << 3);
    g = static_cast<uint8_t>(((c >> 5)  & 0x3F) << 2);
    b = static_cast<uint8_t>((c & 0x1F) << 3);
}

// ─── V9 (G4): Source-Over Alpha Blend for the 3D ROP ───────────────────────
// dst = src·a + dst·(1−a) per RGB channel in float, truncated back to RGB565
// (same quantisation convention as the material evaluators).
//
// alpha == 1.0f is BIT-IDENTICAL to the opaque write path: src·1.0f == src
// exactly in IEEE-754, dst·0.0f == +0.0f (channels are non-negative), the
// sum is exact, and PackRGB565(UnpackRGB565(c)) == c for every RGB565 value
// (unpack shifts up / pack shifts down, no rounding).  The blend branch is
// only reachable from the translucent pass in RasterizeTile, so existing
// materials never execute it.
static inline uint16_t BlendAlphaRGB565(uint16_t src, uint16_t dst, float alpha) {
    uint8_t sr, sg, sb, dr, dg, db;
    UnpackRGB565(src, sr, sg, sb);
    UnpackRGB565(dst, dr, dg, db);
    const float inv = 1.0f - alpha;   // parser clamps alpha to [0,1] → inv ∈ [0,1]
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
// The DSP path uses packed 16-bit SIMD instructions (__UADD16, __USUB16,
// __UHADD16, __UQADD16, __UQSUB16) to operate on two channels simultaneously.
// For the common ADD/SUBTRACT/BASE blend modes, this avoids per-channel
// float→int→float conversion, reducing cycles/pixel significantly.
//
// Channel packing: {R8, G8} as packed halfwords in a uint32_t, B8 separate.
// This keeps green at full 8-bit precision (repacked to 6-bit at final output).

#if defined(__ARM_FEATURE_DSP) || defined(__ARM_ARCH_8M_MAIN__)
#define PGL_USE_DSP_BLEND 1
#else
#define PGL_USE_DSP_BLEND 0
#endif

#if PGL_USE_DSP_BLEND
#include <arm_acle.h>   // __uqadd16, __uqsub16, __uhadd16, etc.
// arm_acle.h uses lowercase; define uppercase aliases for portability
#ifndef __UQADD16
#define __UQADD16(a, b)  __uqadd16((a), (b))
#define __UQSUB16(a, b)  __uqsub16((a), (b))
#define __UHADD16(a, b)  __uhadd16((a), (b))
#endif

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

// Fixed-point lerp for a single byte: a + ((b-a) * alpha256) >> 8
static inline uint8_t LerpByte(uint8_t a, uint8_t b, uint8_t alpha256) {
    return static_cast<uint8_t>(a + ((static_cast<int16_t>(b) - a) * alpha256 >> 8));
}

// Fixed-point lerp for packed {R|G} halfwords
static inline uint32_t LerpRG(uint32_t rgA, uint32_t rgB, uint8_t alpha256) {
    // Unpack, lerp each, repack
    int16_t rA = static_cast<int16_t>(rgA >> 16);
    int16_t gA = static_cast<int16_t>(rgA & 0xFFFF);
    int16_t rB = static_cast<int16_t>(rgB >> 16);
    int16_t gB = static_cast<int16_t>(rgB & 0xFFFF);
    uint8_t rR = static_cast<uint8_t>(rA + ((rB - rA) * alpha256 >> 8));
    uint8_t gR = static_cast<uint8_t>(gA + ((gB - gA) * alpha256 >> 8));
    return (static_cast<uint32_t>(rR) << 16) | static_cast<uint32_t>(gR);
}

/// DSP-accelerated RGB565 blend for common modes.
/// Returns true if the DSP path handled the blend; false = fall through to scalar.
static bool BlendRGB565_DSP(uint16_t colorA, uint16_t colorB,
                            PglBlendMode mode, float opacity,
                            uint16_t& result) {
    uint32_t rgA, rgB;
    uint8_t bA, bB;
    UnpackRGB565_DSP(colorA, rgA, bA);
    UnpackRGB565_DSP(colorB, rgB, bB);

    uint8_t alpha256 = static_cast<uint8_t>(opacity * 255.0f);
    uint32_t rgResult;
    uint8_t  bResult;

    switch (mode) {
    case PGL_BLEND_ADD: {
        // Saturating add via __UQADD16 (dual 16-bit)
        uint32_t rgSum = __UQADD16(rgA, rgB);
        uint8_t  bSum  = static_cast<uint8_t>((bA + bB) > 255 ? 255 : (bA + bB));
        // Lerp with opacity: result = A + (blended - A) * opacity
        rgResult = LerpRG(rgA, rgSum, alpha256);
        bResult  = LerpByte(bA, bSum, alpha256);
        break;
    }
    case PGL_BLEND_SUBTRACT: {
        // Saturating subtract via __UQSUB16 (dual 16-bit, clamps to 0)
        uint32_t rgDiff = __UQSUB16(rgA, rgB);
        uint8_t  bDiff  = static_cast<uint8_t>(bA > bB ? bA - bB : 0);
        rgResult = LerpRG(rgA, rgDiff, alpha256);
        bResult  = LerpByte(bA, bDiff, alpha256);
        break;
    }
    case PGL_BLEND_DARKEN: {
        // Per-channel min — use conditional (no direct SIMD min for u16 pairs)
        uint8_t rA8 = static_cast<uint8_t>(rgA >> 16), rB8 = static_cast<uint8_t>(rgB >> 16);
        uint8_t gA8 = static_cast<uint8_t>(rgA), gB8 = static_cast<uint8_t>(rgB);
        uint32_t rgMin = (static_cast<uint32_t>(rA8 < rB8 ? rA8 : rB8) << 16)
                       | static_cast<uint32_t>(gA8 < gB8 ? gA8 : gB8);
        uint8_t  bMin  = bA < bB ? bA : bB;
        rgResult = LerpRG(rgA, rgMin, alpha256);
        bResult  = LerpByte(bA, bMin, alpha256);
        break;
    }
    case PGL_BLEND_LIGHTEN: {
        uint8_t rA8 = static_cast<uint8_t>(rgA >> 16), rB8 = static_cast<uint8_t>(rgB >> 16);
        uint8_t gA8 = static_cast<uint8_t>(rgA), gB8 = static_cast<uint8_t>(rgB);
        uint32_t rgMax = (static_cast<uint32_t>(rA8 > rB8 ? rA8 : rB8) << 16)
                       | static_cast<uint32_t>(gA8 > gB8 ? gA8 : gB8);
        uint8_t  bMax  = bA > bB ? bA : bB;
        rgResult = LerpRG(rgA, rgMax, alpha256);
        bResult  = LerpByte(bA, bMax, alpha256);
        break;
    }
    case PGL_BLEND_BASE: {
        result = colorA;
        return true;
    }
    case PGL_BLEND_REPLACE: {
        rgResult = LerpRG(rgA, rgB, alpha256);
        bResult  = LerpByte(bA, bB, alpha256);
        break;
    }
    default:
        // Complex modes (Multiply, Divide, Screen, Overlay, SoftLight,
        // EfficientMask) fall through to the scalar float path.
        return false;
    }

    result = PackRGB565_DSP(rgResult, bResult);
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
    float y1 = y0 - static_cast<float>(j1) + G3;
    float z1 = z0 - static_cast<float>(k1) + G3;
    float x2 = x0 - static_cast<float>(i2) + 2.0f * G3;
    float y2 = y0 - static_cast<float>(j2) + 2.0f * G3;
    float z2 = z0 - static_cast<float>(k2) + 2.0f * G3;
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
    // Try DSP-accelerated path for common blend modes
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
// Nearest-neighbour sampling from a TextureSlot (default everywhere; the only
// v8 path).  V9 (G6) adds an opt-in bilinear 4-tap mode, selected per
// material via PglParamImage::filterFlags bit0.  UV clamped to [0, 1].

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

    // ── V9 (G6): bilinear 4-tap, texel-centre mapping ─────────────────────
    // Sample position in texel space is u·width − 0.5 so that integer
    // coordinates sit exactly on texel centres; edge taps clamp (no
    // wraparound, no mip — mip is explicitly out of scope for G6).  Each tap
    // is unpacked from RGB565, lerped per channel in float, and truncated
    // back — the same conventions as the material evaluators.
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

static constexpr uint8_t MAX_MATERIAL_RECURSION = 3;

static uint16_t EvaluateMaterial(const MaterialSlot& mat,
                                 const PglVec3& point,
                                 const PglVec3& normal,
                                 const PglVec2& uv,
                                 const SceneState* scene,
                                 float elapsedTimeS,
                                 uint8_t depth) {
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
        if (p->textureId >= GpuConfig::MAX_TEXTURES) return 0xF81F;

        float su = uv.x * p->scaleX + p->offsetX;
        float sv = uv.y * p->scaleY + p->offsetY;
        // V9 (G6): filterFlags bit0 selects bilinear when UV-mapped.  It
        // reads 0 (nearest) for hosts that sent the frozen 18-byte v8 form —
        // the parser zeroes the params tail.  With no UVs the constant
        // uv={0,0} resolves both filters to texel (0,0) identically.
        return SampleTexture(scene->textures[p->textureId], su, sv,
                             (p->filterFlags & PGL_IMAGE_FILTER_BILINEAR) != 0);
    }

    // ── PGL_MAT_COMBINE (0x50): Blend two materials ────────────────────
    case PGL_MAT_COMBINE: {
        const auto* p = reinterpret_cast<const PglParamCombine*>(mat.params);
        if (p->materialIdA >= GpuConfig::MAX_MATERIALS ||
            p->materialIdB >= GpuConfig::MAX_MATERIALS) return 0xF81F;

        const MaterialSlot& matA = scene->materials[p->materialIdA];
        const MaterialSlot& matB = scene->materials[p->materialIdB];

        uint16_t colorA = matA.active
            ? EvaluateMaterial(matA, point, normal, uv, scene, elapsedTimeS, depth + 1)
            : 0x0000;
        uint16_t colorB = matB.active
            ? EvaluateMaterial(matB, point, normal, uv, scene, elapsedTimeS, depth + 1)
            : 0x0000;

        return BlendRGB565(colorA, colorB,
                           static_cast<PglBlendMode>(p->blendMode), p->opacity);
    }

    // ── PGL_MAT_MASK (0x51): Threshold-based masking ───────────────────
    case PGL_MAT_MASK: {
        const auto* p = reinterpret_cast<const PglParamMask*>(mat.params);
        if (p->baseMaterialId >= GpuConfig::MAX_MATERIALS ||
            p->maskMaterialId >= GpuConfig::MAX_MATERIALS) return 0xF81F;

        // Evaluate mask material as grayscale luminance
        const MaterialSlot& maskMat = scene->materials[p->maskMaterialId];
        uint16_t maskColor = maskMat.active
            ? EvaluateMaterial(maskMat, point, normal, uv, scene, elapsedTimeS, depth + 1)
            : 0x0000;

        uint8_t mr, mg, mb;
        UnpackRGB565(maskColor, mr, mg, mb);
        // Approximate luminance: (R + G + B) / 3
        float lum = (static_cast<float>(mr) + static_cast<float>(mg)
                    + static_cast<float>(mb)) / (3.0f * 255.0f);

        if (lum >= p->threshold) {
            const MaterialSlot& baseMat = scene->materials[p->baseMaterialId];
            return baseMat.active
                ? EvaluateMaterial(baseMat, point, normal, uv, scene, elapsedTimeS, depth + 1)
                : 0x0000;
        }
        return 0x0000;  // masked out → black (transparent)
    }

    // ── PGL_MAT_ANIMATOR (0x52): Lerp between two materials ────────────
    case PGL_MAT_ANIMATOR: {
        const auto* p = reinterpret_cast<const PglParamAnimator*>(mat.params);
        if (p->materialIdA >= GpuConfig::MAX_MATERIALS ||
            p->materialIdB >= GpuConfig::MAX_MATERIALS) return 0xF81F;

        const MaterialSlot& matA = scene->materials[p->materialIdA];
        const MaterialSlot& matB = scene->materials[p->materialIdB];

        uint16_t colorA = matA.active
            ? EvaluateMaterial(matA, point, normal, uv, scene, elapsedTimeS, depth + 1)
            : 0x0000;
        uint16_t colorB = matB.active
            ? EvaluateMaterial(matB, point, normal, uv, scene, elapsedTimeS, depth + 1)
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
        if (p->textureId >= GpuConfig::MAX_TEXTURES) return 0x8410;  // mid-grey fallback
        return SampleTexture(scene->textures[p->textureId], uv.x, uv.y,
                             false);  // nearest — G6 filtering is IMAGE-only
    }

    // ── Unknown material type ──────────────────────────────────────────
    default:
        return 0xF81F;  // magenta: easy to spot
    }
}

// ─── Initialize ─────────────────────────────────────────────────────────────

void Rasterizer::Initialize(SceneState* scene, uint16_t* zBuffer,
                            uint16_t width, uint16_t height) {
    this->scene   = scene;
    this->zBuffer = zBuffer;
    this->width   = width;
    this->height  = height;
    this->projectedTriCount = 0;

    quadTree.Initialize(width, height);
}

// ─── Frame Signature ────────────────────────────────────────────────────────
//
// FNV-1a hash of the draw list (transforms, mesh/material IDs, enabled flags),
// per-mesh CONTENT VERSIONS, and all active camera state.  If the signature
// matches the previous frame, PrepareFrame can skip the full
// transform→project→QuadTree pipeline and the rasterizer reuses the previous
// frame's pixel data.
//
// F-04: mesh vertex bytes are no longer hashed (up to ~24 KB/frame).  The
// command parser bumps SceneState::meshVersion[] on every mesh data write
// (CREATE_MESH, UPDATE_VERTICES, UPDATE_VERTICES_DELTA, DESTROY_MESH), so
// comparing versions detects the same content changes in O(draws) instead of
// O(verts).  Morph-override vertices are per-frame data (frame pool, re-read
// from the wire each frame) — a version cannot detect "same bytes re-sent",
// so those bytes are still content-hashed.

uint32_t Rasterizer::ComputeFrameSignature(const SceneState* s) const {
    uint32_t h = 0x811c9dc5u;

    // Hash draw call count + each draw call's key fields
    h = fnv1a_hash(&s->drawCallCount, sizeof(s->drawCallCount), h);
    for (uint16_t d = 0; d < s->drawCallCount; ++d) {
        const DrawCall& dc = s->drawList[d];
        h = fnv1a_hash(&dc.enabled,    sizeof(dc.enabled),    h);
        h = fnv1a_hash(&dc.meshId,     sizeof(dc.meshId),     h);
        h = fnv1a_hash(&dc.materialId, sizeof(dc.materialId), h);
        h = fnv1a_hash(&dc.transform,  sizeof(dc.transform),  h);
        h = fnv1a_hash(&dc.hasVertexOverride, sizeof(dc.hasVertexOverride), h);
        // If vertices are overridden (morphed), hash the vertex data itself
        if (dc.hasVertexOverride && dc.overrideVertices && dc.overrideVertexCount > 0) {
            h = fnv1a_hash(dc.overrideVertices,
                           dc.overrideVertexCount * sizeof(PglVec3), h);
        }
    }

    // Hash active cameras' transform state
    for (uint8_t c = 0; c < PGL_MAX_CAMERAS; ++c) {
        const CameraSlot& cam = s->cameras[c];
        h = fnv1a_hash(&cam.active, sizeof(cam.active), h);
        if (cam.active) {
            h = fnv1a_hash(&cam.position,     sizeof(cam.position),     h);
            h = fnv1a_hash(&cam.rotation,     sizeof(cam.rotation),     h);
            h = fnv1a_hash(&cam.baseRotation, sizeof(cam.baseRotation), h);
            h = fnv1a_hash(&cam.is2D,         sizeof(cam.is2D),         h);
            // V9 (G3/G7): render target + viewport are camera state too —
            // folded exactly like the transform fields, so a
            // SET_CAMERA_TARGET write invalidates the skip-cache with no
            // version counter (mirrors how SET_CAMERA has always worked).
            h = fnv1a_hash(&cam.targetLayer,  sizeof(cam.targetLayer),  h);
            h = fnv1a_hash(&cam.vpX,          sizeof(cam.vpX),          h);
            h = fnv1a_hash(&cam.vpY,          sizeof(cam.vpY),          h);
            h = fnv1a_hash(&cam.vpW,          sizeof(cam.vpW),          h);
            h = fnv1a_hash(&cam.vpH,          sizeof(cam.vpH),          h);
            h = fnv1a_hash(&cam.vpFlags,      sizeof(cam.vpFlags),      h);
        }
    }

    // Hash mesh content versions for active meshes (F-04 — replaces hashing
    // the vertex bytes themselves; the parser bumps a version on any write,
    // which covers every change the byte hash would have caught).
    for (uint16_t d = 0; d < s->drawCallCount; ++d) {
        const DrawCall& dc = s->drawList[d];
        if (!dc.enabled || dc.meshId >= GpuConfig::MAX_MESHES) continue;
        if (dc.hasVertexOverride) continue;  // already hashed above
        const MeshSlot& mesh = s->meshes[dc.meshId];
        if (mesh.active && mesh.vertices && mesh.vertexCount > 0) {
            h = fnv1a_hash(&s->meshVersion[dc.meshId],
                           sizeof(s->meshVersion[dc.meshId]), h);
        }
    }

    // Hash material content versions for all ACTIVE material slots (F-04
    // extension — the ROP reads material params that the draw-list hash
    // above cannot see; a param update on an otherwise static scene used to
    // be frame-skipped stale).  All-active, not referenced-only: materials
    // reached indirectly through Combine/Mask/Animator have no draw-call
    // reference chain, so — like textures below — every active slot is
    // folded in (editing any material forces one safe re-render).  The
    // active guard means a destroy/re-create transition changes the hash
    // stream exactly like the mesh case.
    for (uint16_t m = 0; m < GpuConfig::MAX_MATERIALS; ++m) {
        if (s->materials[m].active) {
            h = fnv1a_hash(&s->materialVersion[m],
                           sizeof(s->materialVersion[m]), h);
        }
    }

    // Hash texture content versions for all ACTIVE texture slots (F-04
    // extension).  Unlike meshes/materials there is no draw-call reference
    // chain to follow (textures are reached indirectly through IMAGE/
    // PRERENDERED materials, possibly via Combine/Mask/Animator), so all
    // active versions are folded in — conservative: an upload to an
    // unreferenced texture forces one safe re-render.  The active guard
    // again makes destroy/create transitions change the signature.
    for (uint16_t t = 0; t < GpuConfig::MAX_TEXTURES; ++t) {
        if (s->textures[t].active) {
            h = fnv1a_hash(&s->textureVersion[t],
                           sizeof(s->textureVersion[t]), h);
        }
    }

    // Global shader-state version (F-04 extension): screen-space post-FX
    // state (bound programs, builtin slot configs, PSB uniforms) is skipped
    // together with the raster on a signature hit, so any shader-state
    // write must invalidate — otherwise uniform-only animation on a static
    // scene freezes.  Folded unconditionally (it is a single global clock).
    h = fnv1a_hash(&s->shaderStateVersion, sizeof(s->shaderStateVersion), h);

    return h;
}

// ─── G5 (V9): Near-plane clipping ───────────────────────────────────────────
//
// Perspective triangles crossing the view-space near plane
// (z = PglMath::kNearPlaneZ) used to be MIS-PROJECTED, not culled:
// PglMath::PerspectiveProject clamped z to 0.001 before dividing, so a
// crossing triangle's behind-camera vertices projected to ±~64k px and the
// triangle rasterized as a giant flat sliver whose clamped depth (~0.001)
// won the Z test against real geometry.  G5 replaces that with a true
// Sutherland–Hodgman clip of the 3-vertex view-space polygon against
// z = kNearPlaneZ, emitting 1–2 triangles, with per-vertex attributes
// (position and UV) linearly interpolated at the intersection points using
// the same float math the rasterizer uses elsewhere.  The face normal is a
// per-face attribute and is NOT interpolated.
//
// Per-triangle classification in PrepareFrame (after the world transform,
// before projection):
//   all 3 vertices in front (z > near)  → unchanged projection path
//   all 3 behind (z ≤ near)             → drop the triangle
//   crossing                            → clip + emit the 1–2 sub-triangles
//
// After clipping, every emitted vertex has z ≥ kNearPlaneZ > 0, so the
// depths fed to the Z-buffer stay strictly positive (FloatZToU16 is
// order-preserving only for positive floats).

/// One polygon corner for the near-plane clip: view-space position plus the
/// per-corner UV carried through the clip (only meaningful when the source
/// mesh provides UVs for the triangle).
struct NearClipVert {
    PglVec3 view;
    PglVec2 uv;
};

/// Project an already view-space point to screen space.  Same expression
/// tree as the projection stage of PglMath::PerspectiveProject
/// (invZ = fovFactor / z; x·invZ + centre) — used for clipped vertices,
/// which have no world-space counterpart to feed through PerspectiveProject.
/// Caller guarantees view.z ≥ kNearPlaneZ (true for clip output), so no
/// division guard is needed here.
static PglVec2 ProjectViewPoint(const PglVec3& view, float fovFactor,
                                float screenW, float screenH) {
    float invZ = fovFactor / view.z;
    return {
        view.x * invZ + screenW * 0.5f,
        view.y * invZ + screenH * 0.5f
    };
}

/// Sutherland–Hodgman clip of a view-space triangle against the near plane
/// z = PglMath::kNearPlaneZ, keeping the z > near half-space.  `in` is the
/// source triangle (3 corners); `out` receives up to 4 corners in the same
/// winding order (output is a triangle or a quad).  Returns the output
/// corner count.  Along a crossing edge a→b the intersection parameter is
///   t = (a.z − near) / (a.z − b.z)     (0 at a, 1 at b)
/// and x/y/uv are interpolated with that same t; the intersection's z is set
/// to kNearPlaneZ exactly (the point lies ON the plane by construction).
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

/// Resolve a triangle's three corner UVs (same lookup rules the emit tail
/// has always used).  Returns false when the mesh has no usable UVs.
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

/// Shared emit tail for PrepareFrame: back-face cull → screen-AABB cull →
/// pool allocate → Setup → face normal / UV → clamp AABB → QuadTree insert.
/// Used by both the unchanged (fully-in-front) path and the G5 clip output.
/// Silently drops the triangle when it is back-facing, entirely off-screen,
/// degenerate, or the triangle pool is full (fail-safe: a full pool drops
/// ONLY the extra triangle — no out-of-bounds write, no corruption).
/// nva/nvb/nvc are the UNCLIPPED transformed vertices (face normal is a
/// per-face attribute); uvs is the 3 corner UVs or nullptr.
static void EmitProjectedTriangle(const PglVec2& sa, const PglVec2& sb, const PglVec2& sc,
                                  float za, float zb, float zc,
                                  const PglVec3& nva, const PglVec3& nvb, const PglVec3& nvc,
                                  const PglVec2* uvs,
                                  uint16_t drawCallIndex, uint16_t meshTriIndex,
                                  uint16_t screenW, uint16_t screenH) {
    // Back-face culling: skip if triangle has zero or negative area
    float area2d = PglMath::TriangleArea2D(sa, sb, sc);
    if (area2d <= 0.0f) return;

    // Frustum cull: skip if entirely off-screen
    PglMath::AABB2D bounds = PglMath::TriangleBounds2D(sa, sb, sc);
    if (bounds.maxX < 0.0f || bounds.minX >= static_cast<float>(screenW) ||
        bounds.maxY < 0.0f || bounds.minY >= static_cast<float>(screenH)) {
        return;
    }

    // ── Allocate Triangle2D from pool ──
    if (trianglePoolUsed >= GpuConfig::MAX_TRIANGLES) return;
    Triangle2D& tri = trianglePool[trianglePoolUsed];
    if (!tri.Setup(sa, sb, sc, za, zb, zc)) {
        return;  // degenerate triangle (slot is reused by the next candidate)
    }

    tri.drawCallIndex = drawCallIndex;
    tri.meshTriIndex  = meshTriIndex;

    // Compute and store face normal from the unclipped transformed 3D
    // vertices.  Used by NormalMaterial and LightMaterial for shading.
    PglVec3 faceNorm = PglMath::TriangleNormal(nva, nvb, nvc);
    tri.faceNormal = PglMath::Normalize(faceNorm);

    // Set UV data if available (clip output receives interpolated UVs)
    if (uvs) {
        tri.uv0 = uvs[0];
        tri.uv1 = uvs[1];
        tri.uv2 = uvs[2];
        tri.hasUV = true;
    }

    // Clamp AABB to screen bounds before QuadTree insertion.
    // Prevents oversized nodes from large triangles extending off-screen.
    bounds.minX = (bounds.minX > 0.0f) ? bounds.minX : 0.0f;
    bounds.minY = (bounds.minY > 0.0f) ? bounds.minY : 0.0f;
    bounds.maxX = (bounds.maxX < static_cast<float>(screenW))  ? bounds.maxX : static_cast<float>(screenW);
    bounds.maxY = (bounds.maxY < static_cast<float>(screenH)) ? bounds.maxY : static_cast<float>(screenH);

    // Insert into QuadTree
    quadTree.Insert(trianglePoolUsed, bounds);
    trianglePoolUsed++;
}

// ─── PrepareFrame (Core 0, single-threaded) ─────────────────────────────────
//
// Frame-level entry: frame-signature check (identical scene → skip), reset of
// the per-frame pipeline state, then the v8-compatible legacy binding — the
// FIRST active camera whose target resolves to the back buffer gets its pass
// prepared (transform → clip → project → QuadTree).  An unchanged caller that
// runs one tile pass into the back buffer therefore renders exactly the v8
// image.  Multi-camera callers (G3/G7) loop PrepareNextCameraPass() for the
// remaining cameras.
//
// Per-camera pass work lives in PrepareCameraPass():
//   1. Look up MeshSlot → get vertex/index data
//   2. Look up CameraSlot → get camera transform + projection params
//   3. Transform vertices by DrawCall.transform (via PglMath::TransformVertex)
//   4. Project to 2D (PglMath::PerspectiveProject or OrthoProject)
//   5. Build Triangle2D from projected verts, insert into QuadTree

void Rasterizer::PrepareFrame(SceneState* scene) {
    this->scene = scene;
    projectedTriCount = 0;
    trianglePoolUsed  = 0;
    frameSkipped      = false;

    // ── Frame signature caching ─────────────────────────────────────────
    // Hash the scene state. If identical to previous frame, skip rasterization
    // entirely — the back buffer from the prior frame can be reused as-is.
    uint32_t sig = ComputeFrameSignature(scene);
    if (sig == prevFrameSignature && prevFrameSignature != 0) {
        frameSkipped = true;
        return;
    }
    prevFrameSignature = sig;

    // Default pass state: full-frame scissor, panel FB stride, empty pool.
    // With no valid camera pass the caller's tile pass reproduces the v8
    // no-camera behaviour exactly (frame clears to black).
    fbStride       = width;
    scX0 = 0; scY0 = 0; scX1 = width; scY1 = height;
    preparedCamIdx = -1;
    nextCamCursor  = 0;

    // Clear and re-initialize QuadTree for this frame
    quadTree.Clear();
    quadTree.Initialize(width, height);

    // ── Legacy binding: first ACTIVE camera with a valid BACK-BUFFER ────
    // target.  Cameras targeting layers are deliberately NOT prepared here —
    // an unchanged caller renders the prepared pass into the back buffer,
    // and a layer-bound pass would corrupt it with the wrong stride
    // (fail-closed).  PrepareNextCameraPass() covers every valid camera.
    for (uint8_t c = 0; c < PGL_MAX_CAMERAS; ++c) {
        if (!scene->cameras[c].active) continue;
        if (scene->cameras[c].targetLayer != PGL_LAYER_3D) {
            continue;                        // layer-bound → deferred to G3 loop
        }
        CameraTargetInfo ti =
            scene->ResolveCameraTarget(c, nullptr, width, height);
        if (!ti.valid) continue;             // destroyed target → fail-closed
        PrepareCameraPass(scene, c, ti);
        preparedCamIdx = c;
        break;
    }
}

// ─── PrepareNextCameraPass (V9 G3/G7) ───────────────────────────────────────
//
// Iterates the remaining ACTIVE cameras in slot order, skipping the one
// PrepareFrame already bound and cameras whose target no longer resolves
// (fail-closed).  Each accepted camera gets a freshly prepared pass: its own
// Z clear + QuadTree + projection, its own viewport scissor + FB stride.
// The caller runs a tile pass into the camera's resolved target FB.

bool Rasterizer::PrepareNextCameraPass(SceneState* scene, uint8_t* outCamIdx) {
    if (frameSkipped) return false;

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

// ─── PrepareCameraPass (per-camera: transform → clip → project → QuadTree) ──
//
// Binds the pass's target geometry (FB stride + viewport scissor), clears the
// shared panel-addressed Z buffer (sequential passes make sharing safe — each
// pass starts from far-plane), rebuilds the QuadTree, and runs the full
// projection pipeline for this camera.  Cameras render strictly one-after-
// another (the tile pass between passes drains both cores), so the static
// triangle pool + QuadTree are safely reused per pass.

void Rasterizer::PrepareCameraPass(SceneState* scene, uint8_t camIdx,
                                   const CameraTargetInfo& target) {
    this->scene = scene;

    // ── Bind pass state: target FB stride + viewport scissor ────────────
    // The scissor is a clip, not a projection change: all projection and
    // QuadTree math below stays full-frame panel space; RasterizeTile
    // intersects each tile rect with this scissor.
    fbStride = target.width;
    scX0 = target.scX0; scY0 = target.scY0;
    scX1 = target.scX1; scY1 = target.scY1;

    // Clear Z-buffer to far plane (uint16_t representation) — panel-sized,
    // per pass.  Z addressing in the tile loop is panel-strided for every
    // target, so the clear region matches exactly what a pass can touch.
    const uint32_t pixelCount = static_cast<uint32_t>(width) * height;
    std::memset(zBuffer, 0xFF, pixelCount * sizeof(uint16_t));  // 0xFFFF > Z_FAR_U16

    // Clear and re-initialize QuadTree for this pass
    quadTree.Clear();
    quadTree.Initialize(width, height);

    const CameraSlot* activeCam = &scene->cameras[camIdx];

    // Camera parameters
    const PglVec3& camPos = activeCam->position;
    const PglQuat  camRot = PglMath::QuatMul(activeCam->rotation,
                                              activeCam->baseRotation);
    const bool     is2D   = activeCam->is2D;

    // G5: conjugate of the camera rotation, hoisted for the view-space depth
    // scratch and the near-plane clip.  QuatConjugate is exact (sign flips
    // only), so this is bit-identical to the per-call conjugate inside
    // PglMath::PerspectiveProject.
    const PglQuat  camRotConj = PglMath::QuatConjugate(camRot);

    // FOV factor for perspective projection.
    // ProtoTracer uses a fovFactor that relates pixel units to world units.
    // For a 128×64 panel, a factor of ~60-80 gives a reasonable field of view.
    const float fovFactor = static_cast<float>(width) * 0.5f;

    // ── Process each draw call ──────────────────────────────────────────
    for (uint16_t d = 0; d < scene->drawCallCount; ++d) {
        const DrawCall& dc = scene->drawList[d];
        if (!dc.enabled) continue;
        if (dc.meshId >= GpuConfig::MAX_MESHES) continue;

        const MeshSlot& mesh = scene->meshes[dc.meshId];
        if (!mesh.active) continue;
        if (mesh.vertexCount == 0 || mesh.triangleCount == 0) continue;

        // Source vertices: use override (morph) vertices if available,
        // otherwise use the mesh slot's base vertices.
        const PglVec3* srcVerts = mesh.vertices;
        uint16_t vertCount = mesh.vertexCount;
        if (dc.hasVertexOverride && dc.overrideVertices) {
            srcVerts = dc.overrideVertices;
            vertCount = dc.overrideVertexCount;
        }

        // Clamp to buffer capacity
        if (vertCount > GpuConfig::MAX_VERTICES) {
            vertCount = GpuConfig::MAX_VERTICES;
        }

        // Get transform early — needed for both AABB cull and vertex transform
        const PglTransform& xform = dc.transform;

        // F-3: compose this draw call's rotation ONCE — it is invariant
        // across all AABB corners and vertices below.  The per-vertex
        // operation sequence is unchanged (bit-identical float results).
        const PglQuat drawFullRot = PglMath::TransformFullRotation(xform);

        // ── Mesh-level AABB frustum cull ────────────────────────────
        // Transform the 8 corners of the mesh AABB, project to screen,
        // and skip the entire draw call if fully off-screen.
        // For morph overrides we use the base mesh AABB (conservative).
        {
            const PglVec3& mn = mesh.aabbMin;
            const PglVec3& mx = mesh.aabbMax;
            const PglVec3 corners[8] = {
                {mn.x, mn.y, mn.z}, {mx.x, mn.y, mn.z},
                {mn.x, mx.y, mn.z}, {mx.x, mx.y, mn.z},
                {mn.x, mn.y, mx.z}, {mx.x, mn.y, mx.z},
                {mn.x, mx.y, mx.z}, {mx.x, mx.y, mx.z},
            };
            float sMinX = 1e30f, sMinY = 1e30f;
            float sMaxX = -1e30f, sMaxY = -1e30f;
            bool anyInFront = false;
            for (int c = 0; c < 8; ++c) {
                PglVec3 tv = PglMath::TransformVertex(xform, drawFullRot, corners[c]);
                float cz;
                PglVec2 sp;
                if (is2D) {
                    sp = PglMath::OrthoProject(tv, camPos,
                             static_cast<float>(width), static_cast<float>(height));
                    cz = tv.z;
                } else {
                    sp = PglMath::PerspectiveProject(tv, camPos, camRot, fovFactor,
                             static_cast<float>(width), static_cast<float>(height), &cz);
                }
                // G5: cz is now the TRUE view-space z (no projection clamp),
                // so this in-front test finally works as intended — corners
                // at/behind the near plane are excluded from the AABB.
                if (cz > PglMath::kNearPlaneZ) {
                    anyInFront = true;
                    if (sp.x < sMinX) sMinX = sp.x;
                    if (sp.y < sMinY) sMinY = sp.y;
                    if (sp.x > sMaxX) sMaxX = sp.x;
                    if (sp.y > sMaxY) sMaxY = sp.y;
                }
            }
            // Skip entire mesh if all corners behind camera or projected AABB off-screen
            if (!anyInFront ||
                sMaxX < 0.0f || sMinX >= static_cast<float>(width) ||
                sMaxY < 0.0f || sMinY >= static_cast<float>(height)) {
                continue;  // skip this draw call entirely
            }
        }

        // ── Step 1: Transform vertices by the DrawCall's transform ──
        // Applies scale (around scaleOffset), rotation (around rotationOffset),
        // then translation.  This matches ProtoTracer's Transform pipeline.
        for (uint16_t v = 0; v < vertCount; ++v) {
            transformedVerts[v] = PglMath::TransformVertex(xform, drawFullRot, srcVerts[v]);
        }

        // G5: view-space depth per vertex for near-plane classification
        // (perspective only).  Sub → QuatRotate(conjugate) is exactly the
        // view transform PerspectiveProject performs internally, so these
        // depths are bit-identical to the za/zb/zc it writes below.
        if (!is2D) {
            for (uint16_t v = 0; v < vertCount; ++v) {
                viewZScratch[v] = PglMath::QuatRotate(
                    camRotConj, PglMath::Sub(transformedVerts[v], camPos)).z;
            }
        }

        // ── Step 2: Project each triangle to 2D and insert into QuadTree ──
        const PglIndex3* indices = mesh.indices;
        for (uint16_t t = 0; t < mesh.triangleCount; ++t) {
            if (trianglePoolUsed >= GpuConfig::MAX_TRIANGLES) break;

            const PglIndex3& idx = indices[t];
            // Bounds check indices
            if (idx.a >= vertCount || idx.b >= vertCount || idx.c >= vertCount)
                continue;

            const PglVec3& va = transformedVerts[idx.a];
            const PglVec3& vb = transformedVerts[idx.b];
            const PglVec3& vc = transformedVerts[idx.c];

            // Project to screen space
            float za, zb, zc;
            PglVec2 sa, sb, sc;

            if (is2D) {
                sa = PglMath::OrthoProject(va, camPos,
                                           static_cast<float>(width),
                                           static_cast<float>(height));
                sb = PglMath::OrthoProject(vb, camPos,
                                           static_cast<float>(width),
                                           static_cast<float>(height));
                sc = PglMath::OrthoProject(vc, camPos,
                                           static_cast<float>(width),
                                           static_cast<float>(height));
                za = va.z; zb = vb.z; zc = vc.z;

                // Behind-camera cull for the 2D path — unchanged by G5
                // (always live here: OrthoProject never clamped z).  The
                // perspective path classifies against the near plane instead.
                if (za <= 0.0f || zb <= 0.0f || zc <= 0.0f) continue;
            } else {
                // ── G5: classify against the near plane in view space ──
                const int frontCount =
                    (viewZScratch[idx.a] > PglMath::kNearPlaneZ ? 1 : 0) +
                    (viewZScratch[idx.b] > PglMath::kNearPlaneZ ? 1 : 0) +
                    (viewZScratch[idx.c] > PglMath::kNearPlaneZ ? 1 : 0);

                if (frontCount == 0) {
                    continue;  // fully behind the near plane — drop
                }

                if (frontCount < 3) {
                    // Crossing: Sutherland–Hodgman clip of the view-space
                    // polygon against z = kNearPlaneZ, emitting 1–2
                    // sub-triangles.  Full view coordinates are recomputed
                    // only for these (rare) triangles — same op sequence as
                    // the PerspectiveProject internals, so untouched front
                    // corners still project identically.
                    NearClipVert inV[3];
                    inV[0].view = PglMath::QuatRotate(camRotConj, PglMath::Sub(va, camPos));
                    inV[1].view = PglMath::QuatRotate(camRotConj, PglMath::Sub(vb, camPos));
                    inV[2].view = PglMath::QuatRotate(camRotConj, PglMath::Sub(vc, camPos));

                    PglVec2 cornerUV[3] = {};
                    const bool hasUV = ResolveCornerUVs(mesh, t, cornerUV);
                    inV[0].uv = cornerUV[0];
                    inV[1].uv = cornerUV[1];
                    inV[2].uv = cornerUV[2];

                    NearClipVert outV[4];
                    const uint8_t outN = ClipTriangleNearPlane(inV, outV);

                    // Fan-triangulate the clipped polygon (3 or 4 corners),
                    // preserving the original winding.  Pool capacity is
                    // checked per emit — a full pool drops the EXTRA
                    // triangle only, never corrupts.
                    const float w = static_cast<float>(width);
                    const float h = static_cast<float>(height);
                    for (uint8_t k = 0; k + 2 < outN; ++k) {
                        const NearClipVert& p0 = outV[0];
                        const NearClipVert& p1 = outV[k + 1];
                        const NearClipVert& p2 = outV[k + 2];
                        const PglVec2 cuv[3] = { p0.uv, p1.uv, p2.uv };
                        EmitProjectedTriangle(
                            ProjectViewPoint(p0.view, fovFactor, w, h),
                            ProjectViewPoint(p1.view, fovFactor, w, h),
                            ProjectViewPoint(p2.view, fovFactor, w, h),
                            p0.view.z, p1.view.z, p2.view.z,
                            va, vb, vc,
                            hasUV ? cuv : nullptr,
                            d, t, width, height);
                    }
                    continue;
                }

                // Fully in front — unchanged projection path.  outZ now
                // carries the true view z (all three > kNearPlaneZ by the
                // classification above), so the old dead `za <= 0` near-cull
                // is subsumed and removed.
                sa = PglMath::PerspectiveProject(va, camPos, camRot, fovFactor,
                         static_cast<float>(width),
                         static_cast<float>(height), &za);
                sb = PglMath::PerspectiveProject(vb, camPos, camRot, fovFactor,
                         static_cast<float>(width),
                         static_cast<float>(height), &zb);
                sc = PglMath::PerspectiveProject(vc, camPos, camRot, fovFactor,
                         static_cast<float>(width),
                         static_cast<float>(height), &zc);
            }

            // Shared emit tail: back-face cull → screen-AABB cull → pool →
            // Setup → face normal/UV → clamp → QuadTree insert.
            PglVec2 cornerUVs[3];
            EmitProjectedTriangle(sa, sb, sc, za, zb, zc, va, vb, vc,
                                  ResolveCornerUVs(mesh, t, cornerUVs) ? cornerUVs : nullptr,
                                  d, t, width, height);
        }

        projectedTriCount += mesh.triangleCount;
    }

#if GPU_CONFIG_DEBUG_PREPARE_PRINT
    if (projectedTriCount > 0) {
        printf("[Rasterizer] PrepareFrame: %u draw calls, %u triangles projected, "
               "%u in pool, %u QuadTree nodes\n",
               scene->drawCallCount, projectedTriCount,
               trianglePoolUsed, quadTree.GetNodeCount());
    }
#endif
}

// ─── RasterizeRange (both cores, parallel) ──────────────────────────────────
//
// For each pixel (x, y) in [0, width) × [yStart, yEnd):
//   1. Query QuadTree for triangles overlapping this pixel's column
//   2. For each candidate triangle:
//      a. Compute barycentric coordinates
//      b. If inside triangle, interpolate Z
//      c. Z-buffer test
//      d. If closer, evaluate material → write RGB565
//
// Each core writes a distinct Y band, so no synchronization is needed.
//
// NOTE: this legacy band path has no callers (kept for backward
// compatibility).  V9 (G4) alpha blending and the two-pass translucent ROP
// are implemented in RasterizeTile() only — the production 3D path; alpha
// materials render opaque here.

void Rasterizer::RasterizeRange(uint16_t* framebuffer,
                                uint16_t yStart, uint16_t yEnd) {
    if (trianglePoolUsed == 0) {
        // No triangles — clear the band to black
        for (uint16_t y = yStart; y < yEnd && y < height; ++y) {
            uint32_t rowStart = static_cast<uint32_t>(y) * width;
            std::memset(&framebuffer[rowStart], 0, width * sizeof(uint16_t));
        }
        return;
    }

    // Temporary buffer for QuadTree query results
    static constexpr uint16_t MAX_QUERY_RESULTS = 128;

    // Tile size for QuadTree queries.  Querying per-pixel is too expensive;
    // instead we query in horizontal tiles and iterate candidates.
    // For 128-wide panels, querying per column (tile width = 1) is viable
    // because the QuadTree is shallow and the panel is small.
    // A future optimization (M5) could use 4×4 or 8×8 tiles.

    for (uint16_t y = yStart; y < yEnd && y < height; ++y) {
        uint32_t rowStart = static_cast<uint32_t>(y) * width;
        float fy = static_cast<float>(y) + 0.5f;  // pixel center

        for (uint16_t x = 0; x < width; ++x) {
            float fx = static_cast<float>(x) + 0.5f;  // pixel center

            // Query QuadTree for triangles whose AABB covers this pixel
            TriHandle candidates[MAX_QUERY_RESULTS];
            uint16_t hitCount = quadTree.Query(
                fx - 0.5f, fy - 0.5f,
                fx + 0.5f, fy + 0.5f,
                candidates, MAX_QUERY_RESULTS);

            // Find the nearest triangle at this pixel
            uint32_t pixelIdx = rowStart + x;
            uint16_t bestZU16 = zBuffer[pixelIdx];
            float    bestZ    = 1e30f;  // float z kept for material evaluation
            uint16_t bestColor = 0x0000;  // black (background)
            bool     hit = false;

            for (uint16_t h = 0; h < hitCount; ++h) {
                TriHandle handle = candidates[h];
                if (handle >= trianglePoolUsed) continue;

                const Triangle2D& tri = trianglePool[handle];

                // Barycentric test
                float u, v, w;
                if (!tri.Barycentric(fx, fy, u, v, w)) continue;

                // Interpolate depth
                float z = tri.InterpolateZ(u, v, w);
                uint16_t zU16 = FloatZToU16(z);
                if (zU16 >= bestZU16) continue;  // behind existing pixel

                // This triangle is closer — look up material
                const DrawCall& dc = scene->drawList[tri.drawCallIndex];
                uint16_t matId = dc.materialId;
                if (matId >= GpuConfig::MAX_MATERIALS) continue;

                const MaterialSlot& mat = scene->materials[matId];
                if (!mat.active) {
                    // No material assigned — render solid white
                    bestZ = z;
                    bestZU16 = zU16;
                    bestColor = 0xFFFF;
                    hit = true;
                    continue;
                }

                // Interpolate UV (if available)
                PglVec2 uv = {0.0f, 0.0f};
                if (tri.hasUV) {
                    uv = tri.InterpolateUV(u, v, w);
                }

                // Evaluate material
                // Pass face normal and screen-space intersection point.
                // Noise/gradient use screen-space position for pattern generation,
                // while Light and Normal use the stored face normal from PrepareFrame.
                PglVec3 intersectionPoint = {fx, fy, z};
                PglVec3 normal = tri.faceNormal;

                bestColor = EvaluateMaterial(mat, intersectionPoint, normal, uv,
                                             scene, elapsedTimeS, 0);
                bestZ = z;
                bestZU16 = zU16;
                hit = true;
            }

            // Write pixel
            if (hit) {
                framebuffer[pixelIdx] = bestColor;
                zBuffer[pixelIdx] = bestZU16;
            } else {
                framebuffer[pixelIdx] = 0x0000;  // background: black
            }
        }
    }
}


// ─── RasterizeTile (both cores, tile-parallel) ─────────────────────────────
//
// Tile-based rasterisation: query the QuadTree ONCE for the entire 16×16 tile,
// cache the candidate list, then iterate all 256 pixels testing only those
// cached candidates.  This amortises the QuadTree traversal cost across 256
// pixels instead of doing it per-pixel (as in RasterizeRange).
//
// Memory footprint per tile:
//   FB:   16×16×2 = 512 bytes   (fits in L1)
//   ZBuf: 16×16×4 = 1024 bytes  (fits in L1)
//   Candidates: up to 128 handles × 2 = 256 bytes
//   Total: ~1.8 KB per tile — well within Cortex-M33 L1/TCM.
//
// The framebuffer and zBuffer pointers are full-panel-sized; we only write
// to the tile's sub-region [px0..px0+tileW) × [py0..py0+tileH).

void Rasterizer::RasterizeTile(uint16_t* framebuffer, uint16_t* zBuf,
                                uint16_t tileX, uint16_t tileY,
                                uint16_t tileW, uint16_t tileH) {
    // Convert tile coords to pixel coords
    const uint16_t tx0 = tileX * tileW;
    const uint16_t ty0 = tileY * tileH;
    const uint16_t tx1 = (tx0 + tileW < width)  ? (tx0 + tileW) : width;
    const uint16_t ty1 = (ty0 + tileH < height) ? (ty0 + tileH) : height;

    // V9 (G3/G7): intersect with the pass viewport scissor.  Tiles fully
    // outside are skipped WITHOUT clearing — target pixels outside the
    // viewport belong to other cameras / the host UI (the scissor is a
    // clip, not a projection change).  With the default full-frame scissor
    // this is exactly the v8 tile rect.
    const uint16_t px0 = (tx0 > scX0) ? tx0 : scX0;
    const uint16_t py0 = (ty0 > scY0) ? ty0 : scY0;
    const uint16_t px1 = (tx1 < scX1) ? tx1 : scX1;
    const uint16_t py1 = (ty1 < scY1) ? ty1 : scY1;
    if (px0 >= px1 || py0 >= py1) return;

    // Early out: no triangles this frame — clear the scissored tile rect
    // to black.  FB rows use the pass target stride (fbStride); the v8
    // default (stride = panel width) is unchanged.
    if (trianglePoolUsed == 0) {
        for (uint16_t y = py0; y < py1; ++y) {
            uint32_t rowStart = static_cast<uint32_t>(y) * fbStride;
            std::memset(&framebuffer[rowStart + px0], 0,
                        (px1 - px0) * sizeof(uint16_t));
        }
        return;
    }

    // ── Step 1: Query QuadTree for the entire tile AABB ──────────────
    // This is the key optimisation — one traversal for 256 pixels.
    static constexpr uint16_t MAX_QUERY_RESULTS = 128;
    TriHandle candidates[MAX_QUERY_RESULTS];

    uint16_t hitCount = quadTree.Query(
        static_cast<float>(px0),
        static_cast<float>(py0),
        static_cast<float>(px1),
        static_cast<float>(py1),
        candidates, MAX_QUERY_RESULTS);

    // ── Step 2: If no candidates, clear the scissored rect to black ────
    if (hitCount == 0) {
        for (uint16_t y = py0; y < py1; ++y) {
            uint32_t rowStart = static_cast<uint32_t>(y) * fbStride;
            std::memset(&framebuffer[rowStart + px0], 0,
                        (px1 - px0) * sizeof(uint16_t));
        }
        return;
    }

    // ── Step 2b: Sort candidates front-to-back by minimum Z ──────────
    // This ensures closer triangles are tested first, maximising Z-buffer
    // rejection for subsequent (farther) candidates.  Insertion sort is
    // efficient for the small candidate lists (typically 5-30 items).
    for (uint16_t i = 1; i < hitCount; ++i) {
        TriHandle key = candidates[i];
        const Triangle2D& triKey = trianglePool[key];
        float keyMinZ = triKey.z0;
        if (triKey.z1 < keyMinZ) keyMinZ = triKey.z1;
        if (triKey.z2 < keyMinZ) keyMinZ = triKey.z2;

        int j = static_cast<int>(i) - 1;
        while (j >= 0) {
            const Triangle2D& triJ = trianglePool[candidates[j]];
            float jMinZ = triJ.z0;
            if (triJ.z1 < jMinZ) jMinZ = triJ.z1;
            if (triJ.z2 < jMinZ) jMinZ = triJ.z2;
            if (jMinZ <= keyMinZ) break;
            candidates[j + 1] = candidates[j];
            --j;
        }
        candidates[j + 1] = key;
    }

    // ── Step 3: Rasterize pixels, testing only the cached candidates ─
    //
    // S-01: triangle-outer loop with row-invariant edge evaluation.  The
    // per-pixel barycentric test uses the SAME float expression tree as
    // Triangle2D::Barycentric(), so every computed value is bit-identical
    // to the previous pixel-outer loop:
    //   * dy = py - v2.y, rowU = e21x*dy, rowV = e02x*dy are invariant
    //     along a scanline row — computing them once per (triangle, row)
    //     instead of per pixel does not change any bit pattern.
    //   * Per-pixel state evolution is unchanged: candidates are processed
    //     in the same front-to-back sorted order and a winning triangle
    //     writes zBuf[pixelIdx] immediately.  The old loop's per-pixel
    //     bestZU16 mirrored exactly this state machine (same initial value,
    //     same transition conditions, same winning values), so final
    //     framebuffer and z-buffer contents are identical.  The old hi-Z
    //     `break` becomes a per-triangle per-pixel skip: candidates are
    //     sorted by minZ and FloatZToU16() is monotone for the (strictly
    //     positive) depths in the pool, so a triangle skipped by hi-Z could
    //     never have won the pixel anyway.
    //   * The tile's framebuffer is cleared to black up front — identical
    //     to the old per-pixel !hit write; winners overwrite their pixels.
    //
    // Triangle-outer also hoists every pixel-invariant per-triangle
    // quantity (min-Z hi-Z bound + FloatZToU16 conversion, material
    // lookup) out of the pixel loop, and keeps the row edge terms in
    // registers — no scratch arrays, no extra stack (core 1 has 2 KB).

    // Clear the scissored rect to black (background) — covered pixels are
    // overwritten below.
    for (uint16_t y = py0; y < py1; ++y) {
        uint32_t rowStart = static_cast<uint32_t>(y) * fbStride;
        std::memset(&framebuffer[rowStart + px0], 0,
                    (px1 - px0) * sizeof(uint16_t));
    }

    // V9 (G4): set when any candidate carries a PGL_BLEND_ALPHA material.
    // Those triangles are deferred to the translucent pass below; when no
    // alpha materials are present the flag stays false and this tile runs
    // EXACTLY the old single pass (existing materials never reach the blend
    // branch — alpha == 1.0f would be bit-identical to it anyway).
    bool sawAlpha = false;

    for (uint16_t h = 0; h < hitCount; ++h) {
        TriHandle handle = candidates[h];
        if (handle >= trianglePoolUsed) continue;

        const Triangle2D& tri = trianglePool[handle];

        // Hi-Z bound: this triangle's closest vertex depth in z-buffer
        // units.  Hoisted out of the pixel loop (was recomputed per pixel).
        float triMinZ = tri.z0;
        if (tri.z1 < triMinZ) triMinZ = tri.z1;
        if (tri.z2 < triMinZ) triMinZ = tri.z2;
        const uint16_t triMinZU16 = FloatZToU16(triMinZ);

        // Material lookup is pixel-invariant — hoist it too.  A triangle
        // with an invalid material id never wins a pixel (skipped after
        // the z-test in the old loop, without touching the z-buffer).
        const DrawCall& dc = scene->drawList[tri.drawCallIndex];
        const uint16_t matId = dc.materialId;
        if (matId >= GpuConfig::MAX_MATERIALS) continue;
        const MaterialSlot& mat = scene->materials[matId];

        // V9 (G4): translucent materials (PGL_BLEND_ALPHA) defer to the
        // second pass so they blend OVER the finished opaque scene instead
        // of hard-overwriting it (and before farther opaque geometry gets a
        // chance to Z-fail against them).
        if (mat.active && mat.blendMode == PGL_BLEND_ALPHA) {
            sawAlpha = true;
            continue;
        }

        for (uint16_t y = py0; y < py1; ++y) {
            // FB rows use the pass target stride; Z rows are always
            // panel-strided (identical in the v8 default: fbStride == width).
            const uint32_t fbRow = static_cast<uint32_t>(y) * fbStride;
            const uint32_t zRow  = static_cast<uint32_t>(y) * width;
            float py = static_cast<float>(y) + 0.5f;  // pixel centre

            // Row-invariant edge terms — identical expressions (and hence
            // identical results) to Triangle2D::Barycentric().
            float dy   = py - tri.v2.y;
            float rowU = tri.e21x * dy;
            float rowV = tri.e02x * dy;

            for (uint16_t x = px0; x < px1; ++x) {
                // Hi-Z early-out against the current z-buffer value.
                const uint16_t curZU16 = zBuf[zRow + x];
                if (triMinZU16 >= curZU16) continue;

                float px = static_cast<float>(x) + 0.5f;  // pixel centre
                float dx = px - tri.v2.x;

                // Barycentric test — same expression tree as Barycentric().
                float u = fmaf(tri.e10y, dx, rowU) * tri.invDenom;
                float v = fmaf(tri.e20y, dx, rowV) * tri.invDenom;
                float w = 1.0f - u - v;
                if (u < 0.0f || v < 0.0f || w < 0.0f) continue;

                // Interpolate depth
                float z = tri.InterpolateZ(u, v, w);
                uint16_t zU16 = FloatZToU16(z);
                if (zU16 >= curZU16) continue;  // behind existing pixel

                uint16_t color;
                if (!mat.active) {
                    color = 0xFFFF;  // no material → solid white
                } else {
                    // Interpolate UV (if available)
                    PglVec2 uv = {0.0f, 0.0f};
                    if (tri.hasUV) {
                        uv = tri.InterpolateUV(u, v, w);
                    }

                    // Evaluate material — noise/gradient use the screen-space
                    // position, Light/Normal use the stored face normal.
                    PglVec3 intersectionPoint = {px, py, z};
                    PglVec3 normal = tri.faceNormal;

                    color = EvaluateMaterial(mat, intersectionPoint, normal, uv,
                                             scene, elapsedTimeS, 0);
                }

                framebuffer[fbRow + x] = color;
                zBuf[zRow + x] = zU16;
            }
        }
    }

    // ── Pass 2 (V9/G4): translucent triangles — PGL_BLEND_ALPHA ─────────
    //
    // Runs only when pass 1 deferred at least one alpha-material candidate.
    // The candidate order (front-to-back by min-Z) and all per-pixel math
    // are identical to pass 1, but the Z-test now runs against the FINISHED
    // opaque depth buffer, so translucent surfaces behind opaque geometry
    // are occluded exactly like opaque ones, and the blend destination is
    // the completed opaque scene (or background).  A winning pixel blends
    //
    //     dst = src·alpha + dst·(1−alpha)     per RGB channel in float,
    //
    // quantised back to RGB565, and writes its depth — so among stacked
    // translucent surfaces the NEAREST blends exactly once.  There is NO
    // order-independent transparency: hosts must draw translucent geometry
    // back-to-front (painter's algorithm, see PglTypes.h).  alpha == 1.0f
    // is bit-identical to the opaque write (src·1 + dst·0 == src exactly in
    // IEEE-754); non-alpha materials never enter this pass.
    if (sawAlpha) {
        for (uint16_t h = 0; h < hitCount; ++h) {
            TriHandle handle = candidates[h];
            if (handle >= trianglePoolUsed) continue;

            const Triangle2D& tri = trianglePool[handle];

            // Hi-Z bound — same hoisted computation as pass 1.
            float triMinZ = tri.z0;
            if (tri.z1 < triMinZ) triMinZ = tri.z1;
            if (tri.z2 < triMinZ) triMinZ = tri.z2;
            const uint16_t triMinZU16 = FloatZToU16(triMinZ);

            const DrawCall& dc = scene->drawList[tri.drawCallIndex];
            const uint16_t matId = dc.materialId;
            if (matId >= GpuConfig::MAX_MATERIALS) continue;
            const MaterialSlot& mat = scene->materials[matId];

            // Pass 2 is the exact complement of the pass-1 deferral filter.
            if (!(mat.active && mat.blendMode == PGL_BLEND_ALPHA)) continue;
            const float alpha = mat.alpha;   // pixel-invariant — hoisted

            for (uint16_t y = py0; y < py1; ++y) {
                // Same dual-stride addressing as pass 1 (FB stride / panel Z).
                const uint32_t fbRow = static_cast<uint32_t>(y) * fbStride;
                const uint32_t zRow  = static_cast<uint32_t>(y) * width;
                float py = static_cast<float>(y) + 0.5f;  // pixel centre

                // Row-invariant edge terms — identical to pass 1.
                float dy   = py - tri.v2.y;
                float rowU = tri.e21x * dy;
                float rowV = tri.e02x * dy;

                for (uint16_t x = px0; x < px1; ++x) {
                    // Hi-Z early-out against the opaque pass's depth.
                    const uint16_t curZU16 = zBuf[zRow + x];
                    if (triMinZU16 >= curZU16) continue;

                    float px = static_cast<float>(x) + 0.5f;  // pixel centre
                    float dx = px - tri.v2.x;

                    // Barycentric test — same expression tree as pass 1.
                    float u = fmaf(tri.e10y, dx, rowU) * tri.invDenom;
                    float v = fmaf(tri.e20y, dx, rowV) * tri.invDenom;
                    float w = 1.0f - u - v;
                    if (u < 0.0f || v < 0.0f || w < 0.0f) continue;

                    // Z-test against the finished opaque depth buffer.
                    float z = tri.InterpolateZ(u, v, w);
                    uint16_t zU16 = FloatZToU16(z);
                    if (zU16 >= curZU16) continue;  // occluded by nearer surface

                    // mat is guaranteed active by the pass-2 filter above.
                    PglVec2 uv = {0.0f, 0.0f};
                    if (tri.hasUV) {
                        uv = tri.InterpolateUV(u, v, w);
                    }
                    PglVec3 intersectionPoint = {px, py, z};
                    PglVec3 normal = tri.faceNormal;

                    const uint16_t src = EvaluateMaterial(
                        mat, intersectionPoint, normal, uv,
                        scene, elapsedTimeS, 0);

                    // Source-over blend against the current destination
                    // (opaque scene or background), then take the depth.
                    framebuffer[fbRow + x] =
                        BlendAlphaRGB565(src, framebuffer[fbRow + x], alpha);
                    zBuf[zRow + x] = zU16;
                }
            }
        }
    }
}
