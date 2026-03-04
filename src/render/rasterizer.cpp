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

// ─── FNV-1a hash helper ────────────────────────────────────────────────────
// Used for frame signature caching — identical draw list + cameras = skip raster.

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

// QuadTree instance — rebuilt every frame by PrepareFrame().
static QuadTree quadTree;

// ─── RGB565 Utility Functions ───────────────────────────────────────────────

static inline uint16_t PackRGB565(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static inline void UnpackRGB565(uint16_t c, uint8_t& r, uint8_t& g, uint8_t& b) {
    r = static_cast<uint8_t>(((c >> 11) & 0x1F) << 3);
    g = static_cast<uint8_t>(((c >> 5)  & 0x3F) << 2);
    b = static_cast<uint8_t>((c & 0x1F) << 3);
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
#include <arm_compat.h>  // __UQADD16, __UQSUB16, __UHADD16, etc.

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
// Nearest-neighbour sampling from a TextureSlot.  UV clamped to [0, 1].

static uint16_t SampleTexture(const TextureSlot& tex, float u, float v) {
    if (!tex.active || !tex.pixels || tex.width == 0 || tex.height == 0) {
        return 0xF81F;  // magenta = missing texture
    }

    // Clamp UV to [0, 1)
    u = PglMath::Clamp(u, 0.0f, 0.9999f);
    v = PglMath::Clamp(v, 0.0f, 0.9999f);

    uint16_t px = static_cast<uint16_t>(u * tex.width);
    uint16_t py = static_cast<uint16_t>(v * tex.height);
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
        return SampleTexture(scene->textures[p->textureId], su, sv);
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
        return SampleTexture(scene->textures[p->textureId], uv.x, uv.y);
    }

    // ── Unknown material type ──────────────────────────────────────────
    default:
        return 0xF81F;  // magenta: easy to spot
    }
}

// ─── Initialize ─────────────────────────────────────────────────────────────

void Rasterizer::Initialize(SceneState* scene, float* zBuffer,
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
// FNV-1a hash of the draw list (transforms, mesh/material IDs, enabled flags)
// and all active camera state.  If the signature matches the previous frame,
// PrepareFrame can skip the full transform→project→QuadTree pipeline and
// the rasterizer reuses the previous frame's pixel data.

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
        }
    }

    // Hash mesh vertex data for active meshes (catches base vertex changes)
    for (uint16_t d = 0; d < s->drawCallCount; ++d) {
        const DrawCall& dc = s->drawList[d];
        if (!dc.enabled || dc.meshId >= GpuConfig::MAX_MESHES) continue;
        if (dc.hasVertexOverride) continue;  // already hashed above
        const MeshSlot& mesh = s->meshes[dc.meshId];
        if (mesh.active && mesh.vertices && mesh.vertexCount > 0) {
            h = fnv1a_hash(mesh.vertices,
                           mesh.vertexCount * sizeof(PglVec3), h);
        }
    }

    return h;
}

// ─── PrepareFrame (Core 0, single-threaded) ─────────────────────────────────
//
// For each enabled DrawCall:
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

    // Clear Z-buffer to far plane
    const uint32_t pixelCount = static_cast<uint32_t>(width) * height;
    for (uint32_t i = 0; i < pixelCount; ++i) {
        zBuffer[i] = 1e30f;
    }

    // Clear and re-initialize QuadTree for this frame
    quadTree.Clear();
    quadTree.Initialize(width, height);

    // Find the active camera (use camera 0 by default)
    const CameraSlot* activeCam = nullptr;
    for (uint8_t c = 0; c < PGL_MAX_CAMERAS; ++c) {
        if (scene->cameras[c].active) {
            activeCam = &scene->cameras[c];
            break;
        }
    }
    if (!activeCam) {
        // No camera set — nothing to render
        return;
    }

    // Camera parameters
    const PglVec3& camPos = activeCam->position;
    const PglQuat  camRot = PglMath::QuatMul(activeCam->rotation,
                                              activeCam->baseRotation);
    const bool     is2D   = activeCam->is2D;

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

        // ── Step 1: Transform vertices by the DrawCall's transform ──
        // Applies scale (around scaleOffset), rotation (around rotationOffset),
        // then translation.  This matches ProtoTracer's Transform pipeline.
        const PglTransform& xform = dc.transform;
        for (uint16_t v = 0; v < vertCount; ++v) {
            transformedVerts[v] = PglMath::TransformVertex(xform, srcVerts[v]);
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
            } else {
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

            // Back-face culling: skip if triangle has zero or negative area
            float area2d = PglMath::TriangleArea2D(sa, sb, sc);
            if (area2d <= 0.0f) continue;

            // Frustum cull: skip if entirely off-screen
            PglMath::AABB2D bounds = PglMath::TriangleBounds2D(sa, sb, sc);
            if (bounds.maxX < 0.0f || bounds.minX >= static_cast<float>(width) ||
                bounds.maxY < 0.0f || bounds.minY >= static_cast<float>(height)) {
                continue;
            }

            // Near-plane cull: skip if any vertex is behind camera
            if (za <= 0.0f || zb <= 0.0f || zc <= 0.0f) continue;

            // ── Allocate Triangle2D from pool ──
            Triangle2D& tri = trianglePool[trianglePoolUsed];
            if (!tri.Setup(sa, sb, sc, za, zb, zc)) {
                continue;  // degenerate triangle
            }

            tri.drawCallIndex = d;
            tri.meshTriIndex  = t;

            // Compute and store face normal from transformed 3D vertices.
            // Used by NormalMaterial and LightMaterial for shading.
            PglVec3 faceNorm = PglMath::TriangleNormal(va, vb, vc);
            tri.faceNormal = PglMath::Normalize(faceNorm);

            // Set UV data if available
            if (mesh.uvVertices && mesh.uvIndices && t < mesh.triangleCount) {
                const PglIndex3& uvIdx = mesh.uvIndices[t];
                if (uvIdx.a < mesh.uvVertexCount &&
                    uvIdx.b < mesh.uvVertexCount &&
                    uvIdx.c < mesh.uvVertexCount) {
                    tri.uv0 = mesh.uvVertices[uvIdx.a];
                    tri.uv1 = mesh.uvVertices[uvIdx.b];
                    tri.uv2 = mesh.uvVertices[uvIdx.c];
                    tri.hasUV = true;
                }
            }

            // Insert into QuadTree
            quadTree.Insert(trianglePoolUsed, bounds);
            trianglePoolUsed++;
        }

        projectedTriCount += mesh.triangleCount;
    }

    if (projectedTriCount > 0) {
        printf("[Rasterizer] PrepareFrame: %u draw calls, %u triangles projected, "
               "%u in pool, %u QuadTree nodes\n",
               scene->drawCallCount, projectedTriCount,
               trianglePoolUsed, quadTree.GetNodeCount());
    }
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
            float    bestZ = zBuffer[pixelIdx];
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
                if (z >= bestZ) continue;  // behind existing pixel

                // This triangle is closer — look up material
                const DrawCall& dc = scene->drawList[tri.drawCallIndex];
                uint16_t matId = dc.materialId;
                if (matId >= GpuConfig::MAX_MATERIALS) continue;

                const MaterialSlot& mat = scene->materials[matId];
                if (!mat.active) {
                    // No material assigned — render solid white
                    bestZ = z;
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
                hit = true;
            }

            // Write pixel
            if (hit) {
                framebuffer[pixelIdx] = bestColor;
                zBuffer[pixelIdx] = bestZ;
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

void Rasterizer::RasterizeTile(uint16_t* framebuffer, float* zBuf,
                                uint16_t tileX, uint16_t tileY,
                                uint16_t tileW, uint16_t tileH) {
    // Convert tile coords to pixel coords
    const uint16_t px0 = tileX * tileW;
    const uint16_t py0 = tileY * tileH;
    const uint16_t px1 = (px0 + tileW < width)  ? (px0 + tileW) : width;
    const uint16_t py1 = (py0 + tileH < height) ? (py0 + tileH) : height;

    // Early out: no triangles this frame — clear tile to black
    if (trianglePoolUsed == 0) {
        for (uint16_t y = py0; y < py1; ++y) {
            uint32_t rowStart = static_cast<uint32_t>(y) * width;
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

    // ── Step 2: If no candidates, clear tile to black ────────────────
    if (hitCount == 0) {
        for (uint16_t y = py0; y < py1; ++y) {
            uint32_t rowStart = static_cast<uint32_t>(y) * width;
            std::memset(&framebuffer[rowStart + px0], 0,
                        (px1 - px0) * sizeof(uint16_t));
        }
        return;
    }

    // ── Step 3: Rasterize pixels, testing only the cached candidates ─
    for (uint16_t y = py0; y < py1; ++y) {
        uint32_t rowStart = static_cast<uint32_t>(y) * width;
        float fy = static_cast<float>(y) + 0.5f;  // pixel centre

        for (uint16_t x = px0; x < px1; ++x) {
            float fx = static_cast<float>(x) + 0.5f;  // pixel centre

            uint32_t pixelIdx = rowStart + x;
            float    bestZ = zBuf[pixelIdx];
            uint16_t bestColor = 0x0000;
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
                if (z >= bestZ) continue;  // behind existing pixel

                // Material lookup
                const DrawCall& dc = scene->drawList[tri.drawCallIndex];
                uint16_t matId = dc.materialId;
                if (matId >= GpuConfig::MAX_MATERIALS) continue;

                const MaterialSlot& mat = scene->materials[matId];
                if (!mat.active) {
                    bestZ = z;
                    bestColor = 0xFFFF;  // no material → solid white
                    hit = true;
                    continue;
                }

                // Interpolate UV
                PglVec2 uv = {0.0f, 0.0f};
                if (tri.hasUV) {
                    uv = tri.InterpolateUV(u, v, w);
                }

                // Evaluate material
                PglVec3 intersectionPoint = {fx, fy, z};
                PglVec3 normal = tri.faceNormal;

                bestColor = EvaluateMaterial(mat, intersectionPoint, normal, uv,
                                             scene, elapsedTimeS, 0);
                bestZ = z;
                hit = true;
            }

            // Write pixel
            if (hit) {
                framebuffer[pixelIdx] = bestColor;
                zBuf[pixelIdx] = bestZ;
            } else {
                framebuffer[pixelIdx] = 0x0000;
            }
        }
    }
}
