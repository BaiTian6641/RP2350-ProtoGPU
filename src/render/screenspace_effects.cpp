/**
 * @file screenspace_effects.cpp
 * @brief GPU-side screen-space shader engine — RP2350 implementation.
 *
 * Three built-in shader classes plus the verified PSB1 VM stage, all executed
 * through the unified target-scoped engine declared in screenspace_effects.h:
 *   CONVOLUTION   — configurable 1D/2D blur kernel (direction, shape, radius,
 *                   auto-rotation).  Subsumes horizontal/vertical/radial blur
 *                   and anti-aliasing.
 *   DISPLACEMENT  — coordinate warp with optional per-channel chromatic split.
 *                   Subsumes PhaseOffsetX/Y/R and adds new waveform types.
 *   COLOR_ADJUST  — per-pixel colour transform.  Subsumes edge feather and
 *                   adds brightness, contrast, gamma, threshold, invert, and
 *                   Sobel edge detection.
 *   PROGRAM       — verified PSB1 bytecode (see pgl_shader_vm.cpp).
 *
 * Multiworker determinism (S-02/P05-08): every slot runs as two horizontal
 * row bands via PairDispatch::Run.  Bands write disjoint rows of the target
 * and read only the immutable dense snapshot (neighbour/texture reads) or
 * their own row's pixel (in-place classes), so serial execution (desktop
 * sim) is byte-identical to dual-core execution (firmware).  The service
 * callback fires from the core-0 band only, at bounded row slices.
 *
 * Weighted-op accounting: each accepted slot is charged
 * EstimateSlotWeightedOps() × scissor pixels against SceneState::shaderFrameOps
 * (reset by the frame loop); exceeding GpuConfig::POSTFX_WORK_BUDGET stops
 * further passes with Result::Capacity — budget exhaustion is contained, the
 * already-applied slots stay applied.
 */

#include "screenspace_effects.h"

#include "../scene_state.h"
#include "../gpu_config.h"
#include "pgl_shader_vm.h"
#include "../scheduler/pgl_tile_scheduler.h"

#include <PglTypes.h>
#include <PglShaderBytecode.h>
#include <PglShaderBackend.h>

#include <cmath>
#include <cstring>

// ─── Backend alias ──────────────────────────────────────────────────────────
namespace BE = PglShaderBackend;

// ─── RGB565 Helpers (thin delegates to backend) ─────────────────────────────

static inline uint8_t  R5(uint16_t c) { return BE::R5(c); }
static inline uint8_t  G6(uint16_t c) { return BE::G6(c); }
static inline uint8_t  B5(uint16_t c) { return BE::B5(c); }

static inline uint16_t PackRGB565(uint8_t r5, uint8_t g6, uint8_t b5) {
    return BE::PackRGB565i(r5, g6, b5);
}

static inline uint8_t  Clamp5(int v) { return BE::Clamp5(v); }
static inline uint8_t  Clamp6(int v) { return BE::Clamp6(v); }
static inline int      ClampI(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float    ClampF(float v, float lo, float hi) { return BE::Clamp(v, lo, hi); }

static inline float MapF(float value, float inMin, float inMax, float outMin, float outMax) {
    return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
}

/// Blend an effect colour (5/6/5-bit channels) with the source pixel by t.
/// t == 1 is the exact-effect fast path; intensity 0 never reaches here
/// (bypassed slots are skipped before dispatch).
static inline uint16_t Blend565(uint16_t orig, int r5, int g6, int b5, float t) {
    if (t >= 1.0f) return PackRGB565(Clamp5(r5), Clamp6(g6), Clamp5(b5));
    const int or5 = R5(orig), og6 = G6(orig), ob5 = B5(orig);
    return PackRGB565(
        Clamp5(or5 + static_cast<int>((r5 - or5) * t)),
        Clamp6(og6 + static_cast<int>((g6 - og6) * t)),
        Clamp5(ob5 + static_cast<int>((b5 - ob5) * t)));
}

// ─── Oscillator Functions (stateless, driven by elapsed time) ───────────────

static constexpr float MPI = 3.14159265f;

static inline float OscSawtooth(float time, float period) {
    if (period <= 0.0001f) return 0.0f;
    return BE::Mod(time, period) / period;
}

/// General oscillator — returns value in [0,1] based on waveform type.
static inline float Oscillate(float time, float period, uint8_t waveform) {
    if (period <= 0.0001f) return 0.0f;
    float t = BE::Mod(time, period) / period;  // [0,1)
    switch (waveform) {
        default:
        case PGL_WAVE_SAWTOOTH: return t;
        case PGL_WAVE_SINE:     return 0.5f + 0.5f * BE::Sin(2.0f * MPI * t);
        case PGL_WAVE_TRIANGLE: return t < 0.5f ? (2.0f * t) : (2.0f - 2.0f * t);
        case PGL_WAVE_SQUARE:   return t < 0.5f ? 0.0f : 1.0f;
    }
}

/// Oscillator mapped to an arbitrary range.
static inline float OscRange(float time, float period, uint8_t waveform,
                              float minVal, float maxVal) {
    return minVal + Oscillate(time, period, waveform) * (maxVal - minVal);
}

// ─── Kernel weight function ─────────────────────────────────────────────────

/// Returns unnormalised weight for a sample at distance `d` from centre,
/// given the kernel shape and sigma value.
static inline float KernelWeight(int d, uint8_t shape, float sigma) {
    switch (shape) {
        default:
        case PGL_KERNEL_BOX:
            return 1.0f;
        case PGL_KERNEL_GAUSSIAN: {
            float s = (sigma > 0.01f) ? sigma : 1.0f;
            float fd = static_cast<float>(d);
            return BE::Exp(-(fd * fd) / (2.0f * s * s));
        }
        case PGL_KERNEL_TRIANGLE: {
            // Linearly decreasing — but we pass radius externally, so just use
            // the absolute distance.  Normalisation happens in the caller.
            return 1.0f - static_cast<float>(d < 0 ? -d : d) * 0.1f;
        }
    }
}

// ─── Band region ────────────────────────────────────────────────────────────

namespace {

/// Rows of target between service() invocations (bounded work slice).
constexpr uint16_t kServiceSliceRows = 4;

/// Everything a single-core band needs.  Two bands partition the scissor rows
/// [y0,y1) into disjoint [yStart,yEnd) ranges; writes never overlap.
struct FxRegion {
    uint16_t*       fb;        ///< Strided output target: pixel (x,y) at fb[y*stride + x]
    const uint16_t* src;       ///< Dense width×height immutable snapshot (nullptr for in-place classes)
    uint16_t        width, height, stride;
    uint16_t        x0, x1;    ///< Scissor column range
    uint16_t        yStart, yEnd;  ///< Row range owned by THIS band
    float           intensity; ///< Clamped to (0,1]
    float           elapsed;   ///< Seconds (finite)
    void          (*service)();///< Non-null on the core-0 band ONLY
};

/// Fire the service callback every kServiceSliceRows completed rows.
inline void ServiceSlice(const FxRegion& r, uint32_t rowsDone) {
    if (r.service && (rowsDone % kServiceSliceRows) == 0) r.service();
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// ── CONVOLUTION SHADER (reads immutable snapshot) ───────────────────────
// ═══════════════════════════════════════════════════════════════════════════

static void ConvolutionRows(const FxRegion& r, const ShaderSlot& slot) {
    PglShaderParamsConvolution cp;
    std::memcpy(&cp, slot.params, sizeof(cp));

    // Estimator already rejected radius > GpuConfig::MAX_CONVOLUTION_RADIUS;
    // clamp defensively so this function stays bounded in isolation.
    int radius = cp.radius;
    if (radius < 1) radius = 1;
    if (radius > GpuConfig::MAX_CONVOLUTION_RADIUS)
        radius = GpuConfig::MAX_CONVOLUTION_RADIUS;

    const int w = r.width, h = r.height;
    const uint16_t* src = r.src;  // dense snapshot — guaranteed for this class
    const float t = r.intensity;
    uint32_t rowsDone = 0;

    // ── Separable mode (2D, 4-neighbour weighted average) ───────────────
    if (cp.separable) {
        float smoothing = (cp.sigma > 0.001f) ? ClampF(cp.sigma, 0.0f, 1.0f) : 0.25f;
        float invSmooth = 1.0f - smoothing;

        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;
                int cR = R5(src[idx]), cG = G6(src[idx]), cB = B5(src[idx]);
                int nR = 0, nG = 0, nB = 0, nCount = 0;

                if (y > 0)     { uint32_t ni = idx - w; nR += R5(src[ni]); nG += G6(src[ni]); nB += B5(src[ni]); nCount++; }
                if (y < h - 1) { uint32_t ni = idx + w; nR += R5(src[ni]); nG += G6(src[ni]); nB += B5(src[ni]); nCount++; }
                if (x > 0)     { uint32_t ni = idx - 1; nR += R5(src[ni]); nG += G6(src[ni]); nB += B5(src[ni]); nCount++; }
                if (x < w - 1) { uint32_t ni = idx + 1; nR += R5(src[ni]); nG += G6(src[ni]); nB += B5(src[ni]); nCount++; }

                int er = cR, eg = cG, eb = cB;
                if (nCount > 0) {
                    nR /= nCount; nG /= nCount; nB /= nCount;
                    er = static_cast<int>(cR * invSmooth + nR * smoothing);
                    eg = static_cast<int>(cG * invSmooth + nG * smoothing);
                    eb = static_cast<int>(cB * invSmooth + nB * smoothing);
                }
                r.fb[static_cast<uint32_t>(y) * r.stride + x] =
                    Blend565(src[idx], er, eg, eb, t);
            }
            ServiceSlice(r, ++rowsDone);
        }
        return;
    }

    // ── Directional 1D kernel (angle + optional auto-rotation) ──────────

    float angleDeg = cp.angle;
    if (cp.anglePeriod > 0.001f) {
        angleDeg += OscRange(r.elapsed, cp.anglePeriod, PGL_WAVE_SAWTOOTH,
                             0.0f, 360.0f);
    }
    float angleRad = angleDeg * (MPI / 180.0f);
    float dirX = BE::Cos(angleRad);
    float dirY = BE::Sin(angleRad);

    // Simple axis-aligned fast path (no trig per-pixel)
    const bool isHorizontal = (BE::Abs(dirY) < 0.001f);
    const bool isVertical   = (BE::Abs(dirX) < 0.001f);

    if (isHorizontal) {
        // ── Horizontal blur fast path ───────────────────────────────────
        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;
                float wSum = KernelWeight(0, cp.kernelShape, cp.sigma);
                float sumR = R5(src[idx]) * wSum;
                float sumG = G6(src[idx]) * wSum;
                float sumB = B5(src[idx]) * wSum;

                for (int j = -radius; j <= radius; ++j) {
                    if (!j) continue;
                    const int sx = static_cast<int>(x) + j;
                    if (sx < 0 || sx >= w) continue;
                    const float kw = KernelWeight(j, cp.kernelShape, cp.sigma);
                    const uint16_t sample = src[row + sx];
                    sumR += R5(sample) * kw;
                    sumG += G6(sample) * kw;
                    sumB += B5(sample) * kw;
                    wSum += kw;
                }

                r.fb[static_cast<uint32_t>(y) * r.stride + x] = Blend565(src[idx],
                    static_cast<int>(sumR / wSum),
                    static_cast<int>(sumG / wSum),
                    static_cast<int>(sumB / wSum), t);
            }
            ServiceSlice(r, ++rowsDone);
        }
    } else if (isVertical) {
        // ── Vertical blur fast path ─────────────────────────────────────
        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;
                float wSum = KernelWeight(0, cp.kernelShape, cp.sigma);
                float sumR = R5(src[idx]) * wSum;
                float sumG = G6(src[idx]) * wSum;
                float sumB = B5(src[idx]) * wSum;

                for (int j = -radius; j <= radius; ++j) {
                    if (!j) continue;
                    const int sy = static_cast<int>(y) + j;
                    if (sy < 0 || sy >= h) continue;
                    const float kw = KernelWeight(j, cp.kernelShape, cp.sigma);
                    const uint16_t sample = src[static_cast<uint32_t>(sy) * w + x];
                    sumR += R5(sample) * kw;
                    sumG += G6(sample) * kw;
                    sumB += B5(sample) * kw;
                    wSum += kw;
                }

                r.fb[static_cast<uint32_t>(y) * r.stride + x] = Blend565(src[idx],
                    static_cast<int>(sumR / wSum),
                    static_cast<int>(sumG / wSum),
                    static_cast<int>(sumB / wSum), t);
            }
            ServiceSlice(r, ++rowsDone);
        }
    } else {
        // ── General angled blur (radial / diagonal / arbitrary) ─────────
        float cx = static_cast<float>(w) * 0.5f;
        float cy = static_cast<float>(h) * 0.5f;

        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;

                // For auto-rotating mode, use direction from centre rotated
                float rx = dirX, ry = dirY;
                if (cp.anglePeriod > 0.001f) {
                    float dx = static_cast<float>(x) - cx;
                    float dy = static_cast<float>(y) - cy;
                    float len = BE::Len2(dx, dy);
                    if (len > 0.001f) {
                        float cosA = BE::Cos(angleRad), sinA = BE::Sin(angleRad);
                        rx = (dx * cosA - dy * sinA) / len;
                        ry = (dx * sinA + dy * cosA) / len;
                    }
                }

                float wSum = KernelWeight(0, cp.kernelShape, cp.sigma);
                float sumR = R5(src[idx]) * wSum;
                float sumG = G6(src[idx]) * wSum;
                float sumB = B5(src[idx]) * wSum;

                for (int j = -radius; j <= radius; ++j) {
                    if (!j) continue;
                    const int sx = static_cast<int>(x + rx * j);
                    const int sy = static_cast<int>(y + ry * j);
                    if (sx < 0 || sx >= w || sy < 0 || sy >= h) continue;
                    const float kw = KernelWeight(j, cp.kernelShape, cp.sigma);
                    const uint16_t sample = src[static_cast<uint32_t>(sy) * w + sx];
                    sumR += R5(sample) * kw;
                    sumG += G6(sample) * kw;
                    sumB += B5(sample) * kw;
                    wSum += kw;
                }

                r.fb[static_cast<uint32_t>(y) * r.stride + x] = Blend565(src[idx],
                    static_cast<int>(sumR / wSum),
                    static_cast<int>(sumG / wSum),
                    static_cast<int>(sumB / wSum), t);
            }
            ServiceSlice(r, ++rowsDone);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ── DISPLACEMENT SHADER (reads immutable snapshot) ──────────────────────
// ═══════════════════════════════════════════════════════════════════════════

static void DisplacementRows(const FxRegion& r, const ShaderSlot& slot) {
    PglShaderParamsDisplacement dp;
    std::memcpy(&dp, slot.params, sizeof(dp));

    const int w = r.width, h = r.height;
    const uint16_t* src = r.src;  // dense snapshot — guaranteed for this class
    const float t = r.intensity;

    float amplitude = static_cast<float>(dp.amplitude);
    if (amplitude < 1.0f) amplitude = 1.0f;
    float freq  = (dp.frequency > 0.001f) ? dp.frequency : 1.0f;

    // Primary oscillator phase (time-based animation)
    float oscPhase = (dp.period > 0.001f)
                   ? 2.0f * MPI * Oscillate(r.elapsed, dp.period, dp.waveform)
                   : 0.0f;

    static constexpr float PI2_OVER3 = 2.0f * MPI * 0.333f;
    static constexpr float PI4_OVER3 = 2.0f * MPI * 0.666f;

    const bool chromatic = (dp.perChannel != 0);
    const int iRange = static_cast<int>(amplitude);
    uint32_t rowsDone = 0;

    // ── Axis X: horizontal displacement ─────────────────────────────────
    if (dp.axis == PGL_AXIS_X) {
        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * w;
            float coordY = static_cast<float>(y) / (10.0f / freq);

            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;

                if (chromatic) {
                    float sR = BE::Sin(coordY + oscPhase * 8.0f);
                    float sG = BE::Sin(coordY + oscPhase * 8.0f + PI2_OVER3);
                    float sB = BE::Sin(coordY + oscPhase * 8.0f + PI4_OVER3);

                    int oR = ClampI(static_cast<int>(MapF(sR, -1.f, 1.f, 1.f, amplitude)), 1, iRange);
                    int oG = ClampI(static_cast<int>(MapF(sG, -1.f, 1.f, 1.f, amplitude)), 1, iRange);
                    int oB = ClampI(static_cast<int>(MapF(sB, -1.f, 1.f, 1.f, amplitude)), 1, iRange);

                    int xR = static_cast<int>(x) + oR;
                    int xG = static_cast<int>(x) + oG;
                    int xB = static_cast<int>(x) + oB;

                    uint8_t er = (xR >= 0 && xR < w) ? R5(src[row + xR]) : 0;
                    uint8_t eg = (xG >= 0 && xG < w) ? G6(src[row + xG]) : 0;
                    uint8_t eb = (xB >= 0 && xB < w) ? B5(src[row + xB]) : 0;
                    r.fb[static_cast<uint32_t>(y) * r.stride + x] =
                        Blend565(src[idx], er, eg, eb, t);
                } else {
                    float s = BE::Sin(coordY + oscPhase * 8.0f);
                    int off = ClampI(static_cast<int>(MapF(s, -1.f, 1.f, 1.f, amplitude)), 1, iRange);
                    int sx = static_cast<int>(x) + off;
                    uint16_t px = (sx >= 0 && sx < w) ? src[row + sx] : 0;
                    r.fb[static_cast<uint32_t>(y) * r.stride + x] =
                        Blend565(src[idx], R5(px), G6(px), B5(px), t);
                }
            }
            ServiceSlice(r, ++rowsDone);
        }
        return;
    }

    // ── Axis Y: vertical displacement ───────────────────────────────────
    if (dp.axis == PGL_AXIS_Y) {
        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;
                float coordX = static_cast<float>(x) / (10.0f / freq);

                if (chromatic) {
                    float sR = BE::Sin(coordX + oscPhase * 8.0f);
                    float sG = BE::Sin(coordX + oscPhase * 8.0f + PI2_OVER3);
                    float sB = BE::Sin(coordX + oscPhase * 8.0f + PI4_OVER3);

                    int oR = ClampI(static_cast<int>(MapF(sR, -1.f, 1.f, 1.f, amplitude)), 1, iRange);
                    int oG = ClampI(static_cast<int>(MapF(sG, -1.f, 1.f, 1.f, amplitude)), 1, iRange);
                    int oB = ClampI(static_cast<int>(MapF(sB, -1.f, 1.f, 1.f, amplitude)), 1, iRange);

                    int yR = static_cast<int>(y) - oR;
                    int yG = static_cast<int>(y) - oG;
                    int yB = static_cast<int>(y) - oB;

                    uint8_t er = (yR >= 0 && yR < h) ? R5(src[static_cast<uint32_t>(yR) * w + x]) : 0;
                    uint8_t eg = (yG >= 0 && yG < h) ? G6(src[static_cast<uint32_t>(yG) * w + x]) : 0;
                    uint8_t eb = (yB >= 0 && yB < h) ? B5(src[static_cast<uint32_t>(yB) * w + x]) : 0;
                    r.fb[static_cast<uint32_t>(y) * r.stride + x] =
                        Blend565(src[idx], er, eg, eb, t);
                } else {
                    float s = BE::Sin(coordX + oscPhase * 8.0f);
                    int off = ClampI(static_cast<int>(MapF(s, -1.f, 1.f, 1.f, amplitude)), 1, iRange);
                    int sy = static_cast<int>(y) - off;
                    uint16_t px = (sy >= 0 && sy < h)
                                ? src[static_cast<uint32_t>(sy) * w + x] : 0;
                    r.fb[static_cast<uint32_t>(y) * r.stride + x] =
                        Blend565(src[idx], R5(px), G6(px), B5(px), t);
                }
            }
            ServiceSlice(r, ++rowsDone);
        }
        return;
    }

    // ── Axis RADIAL: radial chromatic aberration ────────────────────────
    {
        float rotPeriod = (dp.period > 0.001f) ? dp.period : 3.7f;
        float p1Period  = (dp.phase1Period > 0.001f) ? dp.phase1Period : 4.5f;
        float p2Period  = (dp.phase2Period > 0.001f) ? dp.phase2Period : 3.2f;

        float rotation = OscRange(r.elapsed, rotPeriod, dp.waveform, 0.0f, 360.0f);
        float offset1  = OscSawtooth(r.elapsed, p1Period);
        float offset2  = OscSawtooth(r.elapsed, p2Period);

        float phase120 = 2.0f * MPI * 0.333f;
        float phase240 = 2.0f * MPI * 0.666f;
        float mpiR = 2.0f * MPI * 8.0f;

        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;

                float coordX = static_cast<float>(x) / (10.0f / freq);
                float coordY = static_cast<float>(y) / (5.0f / freq);

                float sineR = BE::Sin(coordX + mpiR * offset1) + BE::Cos(coordY + mpiR * offset2);
                float sineG = BE::Sin(coordX + (mpiR + phase120) * offset1) + BE::Cos(coordY + (mpiR + phase120) * offset2);
                float sineB = BE::Sin(coordX + (mpiR + phase240) * offset1) + BE::Cos(coordY + (mpiR + phase240) * offset2);

                int blurR = ClampI(static_cast<int>(MapF(sineR, -2.f, 2.f, 1.f, amplitude)), 1, iRange);
                int blurG = ClampI(static_cast<int>(MapF(sineG, -2.f, 2.f, 1.f, amplitude)), 1, iRange);
                int blurB = ClampI(static_cast<int>(MapF(sineB, -2.f, 2.f, 1.f, amplitude)), 1, iRange);

                auto SampleRadial = [&](float angleDeg, int dist) -> uint32_t {
                    float rad = angleDeg * (MPI / 180.0f);
                    int sx = static_cast<int>(x) + static_cast<int>(dist * BE::Cos(rad));
                    int sy = static_cast<int>(y) + static_cast<int>(dist * BE::Sin(rad));
                    if (sx >= 0 && sx < w && sy >= 0 && sy < h)
                        return static_cast<uint32_t>(sy) * w + sx;
                    return UINT32_MAX;
                };

                uint32_t idxR = SampleRadial(rotation,          blurR);
                uint32_t idxG = SampleRadial(rotation + 120.0f, blurG);
                uint32_t idxB = SampleRadial(rotation + 240.0f, blurB);

                uint8_t er = (idxR != UINT32_MAX) ? R5(src[idxR]) : 0;
                uint8_t eg = (idxG != UINT32_MAX) ? G6(src[idxG]) : 0;
                uint8_t eb = (idxB != UINT32_MAX) ? B5(src[idxB]) : 0;

                r.fb[static_cast<uint32_t>(y) * r.stride + x] =
                    Blend565(src[idx], er, eg, eb, t);
            }
            ServiceSlice(r, ++rowsDone);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ── COLOR ADJUST SHADER ─────────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════════════

static void ColorAdjustRows(const FxRegion& r, const ShaderSlot& slot) {
    PglShaderParamsColorAdjust cp;
    std::memcpy(&cp, slot.params, sizeof(cp));

    const int w = r.width, h = r.height;
    const uint32_t stride = r.stride;
    const float t = r.intensity;
    uint32_t rowsDone = 0;

    switch (cp.operation) {

    // ── Edge Feather (dim pixels adjacent to black; snapshot neighbours) ─
    case PGL_COLOR_EDGE_FEATHER: {
        const uint16_t* src = r.src;  // guaranteed for this operation
        float strength = cp.strength;
        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;
                const uint16_t orig = src[idx];
                if (orig == 0x0000) continue;

                bool isEdge = false;
                if (y == 0 || src[idx - w] == 0x0000) isEdge = true;
                if (!isEdge && (y == h - 1 || src[idx + w] == 0x0000)) isEdge = true;
                if (!isEdge && (x == 0 || src[idx - 1] == 0x0000)) isEdge = true;
                if (!isEdge && (x == w - 1 || src[idx + 1] == 0x0000)) isEdge = true;

                if (isEdge) {
                    int er = static_cast<int>(R5(orig) * strength);
                    int eg = static_cast<int>(G6(orig) * strength);
                    int eb = static_cast<int>(B5(orig) * strength);
                    r.fb[static_cast<uint32_t>(y) * stride + x] =
                        Blend565(orig, er, eg, eb, t);
                }
            }
            ServiceSlice(r, ++rowsDone);
        }
        break;
    }

    // ── Brightness (in-place: reads only the pixel it owns) ─────────────
    case PGL_COLOR_BRIGHTNESS: {
        // strength: -1.0 to +1.0 mapped to 5/6-bit delta
        float delta5 = cp.strength * 31.0f;
        float delta6 = cp.strength * 63.0f;
        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * stride;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;
                const uint16_t orig = r.fb[idx];
                r.fb[idx] = Blend565(orig,
                    static_cast<int>(R5(orig) + delta5),
                    static_cast<int>(G6(orig) + delta6),
                    static_cast<int>(B5(orig) + delta5), t);
            }
            ServiceSlice(r, ++rowsDone);
        }
        break;
    }

    // ── Contrast ────────────────────────────────────────────────────────
    case PGL_COLOR_CONTRAST: {
        // strength: 0.0 = flat grey, 1.0 = unchanged, 2.0 = double contrast
        float s = cp.strength;
        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * stride;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;
                const uint16_t orig = r.fb[idx];
                float fr = (R5(orig) / 31.0f - 0.5f) * s + 0.5f;
                float fg = (G6(orig) / 63.0f - 0.5f) * s + 0.5f;
                float fb2 = (B5(orig) / 31.0f - 0.5f) * s + 0.5f;
                r.fb[idx] = Blend565(orig,
                    static_cast<int>(fr * 31.0f),
                    static_cast<int>(fg * 63.0f),
                    static_cast<int>(fb2 * 31.0f), t);
            }
            ServiceSlice(r, ++rowsDone);
        }
        break;
    }

    // ── Gamma ───────────────────────────────────────────────────────────
    case PGL_COLOR_GAMMA: {
        float gamma = (cp.param2 > 0.01f) ? cp.param2 : 2.2f;
        float invGamma = 1.0f / gamma;
        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * stride;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;
                const uint16_t orig = r.fb[idx];
                float fr = BE::Pow(R5(orig) / 31.0f, invGamma);
                float fg = BE::Pow(G6(orig) / 63.0f, invGamma);
                float fb2 = BE::Pow(B5(orig) / 31.0f, invGamma);
                r.fb[idx] = Blend565(orig,
                    static_cast<int>(fr * 31.0f),
                    static_cast<int>(fg * 63.0f),
                    static_cast<int>(fb2 * 31.0f), t);
            }
            ServiceSlice(r, ++rowsDone);
        }
        break;
    }

    // ── Threshold ───────────────────────────────────────────────────────
    case PGL_COLOR_THRESHOLD: {
        float thresh = cp.strength;  // 0.0–1.0
        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * stride;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;
                const uint16_t orig = r.fb[idx];
                float lum = (R5(orig) / 31.0f * 0.299f +
                             G6(orig) / 63.0f * 0.587f +
                             B5(orig) / 31.0f * 0.114f);
                r.fb[idx] = (lum >= thresh) ? Blend565(orig, 31, 63, 31, t)
                                            : Blend565(orig, 0, 0, 0, t);
            }
            ServiceSlice(r, ++rowsDone);
        }
        break;
    }

    // ── Invert ──────────────────────────────────────────────────────────
    case PGL_COLOR_INVERT: {
        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * stride;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;
                const uint16_t orig = r.fb[idx];
                r.fb[idx] = Blend565(orig, 31 - R5(orig), 63 - G6(orig),
                                     31 - B5(orig), t);
            }
            ServiceSlice(r, ++rowsDone);
        }
        break;
    }

    // ── Edge Detect (Sobel; snapshot neighbours) ────────────────────────
    case PGL_COLOR_EDGE_DETECT: {
        const uint16_t* src = r.src;  // guaranteed for this operation
        float scale = (cp.strength > 0.01f) ? cp.strength : 1.0f;
        for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
            const uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = r.x0; x < r.x1; ++x) {
                const uint32_t idx = row + x;
                int er = 0, eg = 0, eb = 0;
                if (y > 0 && y < h - 1 && x > 0 && x < w - 1) {
                    // Luminance for 3×3 neighbourhood (green channel, 6-bit)
                    auto L = [&](int dx, int dy) -> float {
                        return G6(src[static_cast<uint32_t>(y + dy) * w + (x + dx)]) / 63.0f;
                    };
                    float gx = -L(-1,-1) + L(1,-1) - 2*L(-1,0) + 2*L(1,0) - L(-1,1) + L(1,1);
                    float gy = -L(-1,-1) - 2*L(0,-1) - L(1,-1) + L(-1,1) + 2*L(0,1) + L(1,1);
                    float mag = BE::Clamp(BE::Sqrt(gx*gx + gy*gy) * scale, 0.0f, 1.0f);
                    er = static_cast<int>(mag * 31.0f);
                    eg = static_cast<int>(mag * 63.0f);
                    eb = er;
                }
                // Image-border pixels → black (defined, as before)
                r.fb[static_cast<uint32_t>(y) * stride + x] =
                    Blend565(src[idx], er, eg, eb, t);
            }
            ServiceSlice(r, ++rowsDone);
        }
        break;
    }

    default:
        break;  // unknown operation — rejected by the estimator before dispatch
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ── PROGRAMMABLE (PSB1 VM) BAND ─────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════════════

// The VM executes a VERIFIED program (DecodeShaderProgram).  When the program
// reads the framebuffer (derived TEX2D presence, not the host flag), both the
// per-pixel input and TEX2D samples come from the immutable dense snapshot;
// otherwise the input is the band's own pixel and the sample pointer is never
// dereferenced (the verified program contains no TEX2D).

static void ProgramRows(const FxRegion& r, const ShaderProgram& prog,
                        const float* uniforms, const uint16_t* sampleFb) {
    PglShaderVM vm;  // per-band instance — register file is per-instance state
    const uint32_t stride = r.stride;
    const uint16_t w = r.width;
    const float t = r.intensity;
    uint32_t rowsDone = 0;

    for (uint16_t y = r.yStart; y < r.yEnd; ++y) {
        for (uint16_t x = r.x0; x < r.x1; ++x) {
            const uint16_t pixel = r.src ? r.src[static_cast<uint32_t>(y) * w + x]
                                         : r.fb[static_cast<uint32_t>(y) * stride + x];
            float inR = static_cast<float>(R5(pixel)) / 31.0f;
            float inG = static_cast<float>(G6(pixel)) / 63.0f;
            float inB = static_cast<float>(B5(pixel)) / 31.0f;

            float outR, outG, outB;  // finite on return (VM output containment)
            vm.Execute(prog, uniforms, static_cast<float>(x), static_cast<float>(y),
                       inR, inG, inB,
                       sampleFb, w, r.height,
                       outR, outG, outB);

            // Intensity blending: mix(original, shader output, intensity)
            if (t < 1.0f) {
                outR = inR + (outR - inR) * t;
                outG = inG + (outG - inG) * t;
                outB = inB + (outB - inB) * t;
            }

            r.fb[static_cast<uint32_t>(y) * stride + x] = PackRGB565(
                Clamp5(static_cast<int>(outR * 31.0f + 0.5f)),
                Clamp6(static_cast<int>(outG * 63.0f + 0.5f)),
                Clamp5(static_cast<int>(outB * 31.0f + 0.5f)));
        }
        ServiceSlice(r, ++rowsDone);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ── DISPATCHER + PUBLIC API ─────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════════════

namespace {

struct FxBandContext {
    FxRegion            region;
    const ShaderSlot*   slot;
    const ShaderProgram* prog;     // PROGRAM class only (immutable resident code)
    const float*        uniforms; // PROGRAM class only (immutable pass-local bank)
    const uint16_t*     sampleFb;  // PROGRAM class only (dense snapshot or nullptr)
};

void RunFxBand(void* ctxPtr) {
    const FxBandContext& c = *static_cast<const FxBandContext*>(ctxPtr);
    switch (c.slot->shaderClass) {
        case PGL_SHADER_CONVOLUTION:  ConvolutionRows(c.region, *c.slot); break;
        case PGL_SHADER_DISPLACEMENT: DisplacementRows(c.region, *c.slot); break;
        case PGL_SHADER_COLOR_ADJUST: ColorAdjustRows(c.region, *c.slot); break;
        case PGL_SHADER_PROGRAM:      ProgramRows(c.region, *c.prog, c.uniforms, c.sampleFb); break;
        default: break;  // unreachable — estimator rejects unknown classes
    }
}

/// Whether the slot's class/operation reads neighbours or texture data and
/// therefore needs the immutable snapshot taken before it runs.
bool SlotNeedsSnapshot(const SceneState* scene, const ShaderSlot& slot) {
    switch (slot.shaderClass) {
        case PGL_SHADER_CONVOLUTION:
        case PGL_SHADER_DISPLACEMENT:
            return true;
        case PGL_SHADER_COLOR_ADJUST: {
            PglShaderParamsColorAdjust cp;
            std::memcpy(&cp, slot.params, sizeof(cp));
            return cp.operation == PGL_COLOR_EDGE_FEATHER ||
                   cp.operation == PGL_COLOR_EDGE_DETECT;
        }
        case PGL_SHADER_PROGRAM: {
            if (!scene || slot.programId >= GpuConfig::MAX_SHADER_PROGRAMS)
                return false;  // rejected by the estimator anyway
            // DERIVED requirement — the host PSB_FLAG is never consulted here.
            return scene->shaderPrograms[slot.programId].readsFramebuffer;
        }
        default:
            return false;
    }
}

}  // namespace

uint32_t ScreenspaceShaders::EstimateSlotWeightedOps(const SceneState* scene,
                                                      const ShaderSlot& slot,
                                                      uint32_t pixelCount) {
    if (!slot.active || slot.shaderClass == PGL_SHADER_NONE) return 0;
    if (!std::isfinite(slot.intensity)) return UINT32_MAX;
    if (slot.intensity <= 0.0f) return 0;  // exact bypass — free

    uint32_t perPixel;
    switch (slot.shaderClass) {
        case PGL_SHADER_CONVOLUTION: {
            PglShaderParamsConvolution cp;
            std::memcpy(&cp, slot.params, sizeof(cp));
            if(cp.kernelShape>PGL_KERNEL_TRIANGLE || cp.separable>1 || cp._pad ||
               !std::isfinite(cp.angle) || !std::isfinite(cp.anglePeriod) || cp.anglePeriod<0 ||
               !std::isfinite(cp.sigma) || (cp.kernelShape==PGL_KERNEL_GAUSSIAN && cp.sigma<=0) ||
               (cp.separable && (cp.sigma<0 || cp.sigma>1))) return UINT32_MAX;
            int radius = cp.radius;
            if (radius < 1) radius = 1;
            if (radius > GpuConfig::MAX_CONVOLUTION_RADIUS) return UINT32_MAX;
            perPixel = cp.separable ? 6u
                                    : static_cast<uint32_t>(2 * radius + 1) * 2u;
            break;
        }
        case PGL_SHADER_DISPLACEMENT: {
            PglShaderParamsDisplacement dp;
            std::memcpy(&dp, slot.params, sizeof(dp));
            if(dp.axis>PGL_AXIS_RADIAL || dp.perChannel>1 || dp.waveform>PGL_WAVE_SQUARE ||
               dp.amplitude>32 || !std::isfinite(dp.period) || dp.period<0 ||
               !std::isfinite(dp.frequency) || dp.frequency<0 ||
               !std::isfinite(dp.phase1Period) || dp.phase1Period<0 ||
               !std::isfinite(dp.phase2Period) || dp.phase2Period<0) return UINT32_MAX;
            perPixel = (dp.axis == PGL_AXIS_RADIAL) ? 16u
                     : (dp.perChannel ? 12u : 6u);
            break;
        }
        case PGL_SHADER_COLOR_ADJUST: {
            PglShaderParamsColorAdjust cp;
            std::memcpy(&cp, slot.params, sizeof(cp));
            if(!std::isfinite(cp.strength) || !std::isfinite(cp.param2) ||
               cp._pad[0] || cp._pad[1] || cp._pad[2]) return UINT32_MAX;
            switch (cp.operation) {
                case PGL_COLOR_EDGE_FEATHER: perPixel = 5u;  break;
                case PGL_COLOR_EDGE_DETECT:  perPixel = 12u; break;
                case PGL_COLOR_GAMMA:        perPixel = 12u; break;
                case PGL_COLOR_THRESHOLD:
                case PGL_COLOR_INVERT:
                case PGL_COLOR_BRIGHTNESS:
                case PGL_COLOR_CONTRAST:     perPixel = 2u;  break;
                default: return UINT32_MAX;  // unknown operation
            }
            break;
        }
        case PGL_SHADER_PROGRAM: {
            if (!scene || slot.programId >= GpuConfig::MAX_SHADER_PROGRAMS)
                return UINT32_MAX;
            const ShaderProgram& prog = scene->shaderPrograms[slot.programId];
            if (!prog.active || !prog.verified) return UINT32_MAX;
            perPixel = prog.weightedCost;
            break;
        }
        default:
            return UINT32_MAX;  // unknown shader class
    }
    return perPixel * pixelCount;
}

PglRuntime::Result ScreenspaceShaders::ApplyShaderSlots(
        SceneState* scene, const ShaderSlot* slots, size_t count,
        uint16_t* framebuffer, uint16_t width, uint16_t height, uint16_t stride,
        uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
        uint16_t* scratch, size_t scratchPixels,
        float elapsedSeconds, void (*service)()) {
    using Result = PglRuntime::Result;

    if (!scene || !framebuffer)                 return Result::InvalidValue;
    if (count > 0 && !slots)                    return Result::InvalidValue;
    if (width == 0 || height == 0 || stride < width) return Result::InvalidValue;
    if (x0 > x1 || y0 > y1 || x1 > width || y1 > height) return Result::InvalidValue;
    if (!std::isfinite(elapsedSeconds))         return Result::InvalidValue;
    if (count == 0 || x0 == x1 || y0 == y1)     return Result::Ok;

    Result firstError = Result::Ok;
    const uint32_t scissorPixels = static_cast<uint32_t>(x1 - x0)
                                 * static_cast<uint32_t>(y1 - y0);

    // Only auto/user uniforms vary per pass; never clone verified bytecode or
    // constants onto either core's small stack. No initialization for builtins.
    float uniforms[PSB_MAX_UNIFORMS];

    for (size_t i = 0; i < count; ++i) {
        const ShaderSlot& slot = slots[i];
        if (!slot.active || slot.shaderClass == PGL_SHADER_NONE) continue;

        if (!std::isfinite(slot.intensity)) {
            if (firstError == Result::Ok) firstError = Result::InvalidValue;
            continue;
        }
        if (slot.intensity <= 0.0f) continue;  // exact bypass: zero pixels, zero cost
        const float intensity = (slot.intensity >= 1.0f) ? 1.0f : slot.intensity;

        // ── Structural validation + weighted cost (shared with admission) ─
        const uint32_t slotCost = EstimateSlotWeightedOps(scene, slot, scissorPixels);
        if (slotCost == UINT32_MAX) {
            if (firstError == Result::Ok) firstError = Result::InvalidValue;
            continue;  // consumer-visible rejection: invalid slot never runs
        }

        // ── Frame budget: stop deterministically when exhausted ─────────
        if (static_cast<uint64_t>(scene->shaderFrameOps) + slotCost >
            GpuConfig::POSTFX_WORK_BUDGET) {
            return (firstError != Result::Ok) ? firstError : Result::Capacity;
        }
        scene->shaderFrameOps += slotCost;

        // ── Immutable source snapshot for neighbour/texture reads ───────
        const bool needsSnapshot = SlotNeedsSnapshot(scene, slot);
        if (needsSnapshot) {
            if (!scratch || scratchPixels < static_cast<size_t>(width) * height) {
                if (firstError == Result::Ok) firstError = Result::Capacity;
                continue;  // scratch must cover the actual target
            }
            // Dense copy of the FULL logical image (not just the scissor) so
            // TEX2D UV and kernel reads at scissor edges stay defined.
            for (uint16_t y = 0; y < height; ++y) {
                std::memcpy(scratch + static_cast<size_t>(y) * width,
                            framebuffer + static_cast<size_t>(y) * stride,
                            static_cast<size_t>(width) * sizeof(uint16_t));
            }
        }

        // ── PROGRAM class: bind auto uniforms into a read-only bank ──────
        const ShaderProgram* progPtr = nullptr;
        if (slot.shaderClass == PGL_SHADER_PROGRAM) {
            progPtr = &scene->shaderPrograms[slot.programId];
            std::memcpy(uniforms, progPtr->uniforms, sizeof(uniforms));
            uniforms[PSB_AUTO_UNIFORM_RESOLUTION_X] = static_cast<float>(width);
            uniforms[PSB_AUTO_UNIFORM_RESOLUTION_Y] = static_cast<float>(height);
            uniforms[PSB_AUTO_UNIFORM_TIME]         = elapsedSeconds;
        }

        // ── Dual-band dispatch: disjoint rows, immutable sources ─────────
        // Band A (core 0) owns the service callback; band B (core 1) gets
        // nullptr.  `service` is ALSO the PairDispatch idle function, polled
        // by core 0 while core 1 finishes.
        const uint16_t yMid = static_cast<uint16_t>(y0 + (y1 - y0) / 2);

        FxBandContext bandA = {
            { framebuffer, needsSnapshot ? scratch : nullptr,
              width, height, stride, x0, x1, y0, yMid,
              intensity, elapsedSeconds, service },
            &slot, progPtr, uniforms, needsSnapshot ? scratch : nullptr
        };
        FxBandContext bandB = {
            { framebuffer, needsSnapshot ? scratch : nullptr,
              width, height, stride, x0, x1, yMid, y1,
              intensity, elapsedSeconds, nullptr },
            &slot, progPtr, uniforms, needsSnapshot ? scratch : nullptr
        };

        const auto dispatched = PairDispatch::Run(y0 < yMid ? RunFxBand : nullptr, &bandA,
                                                 yMid < y1 ? RunFxBand : nullptr, &bandB,
                                                 service);
        // Bank/band contexts remain live until both workers complete.
        if (dispatched != PglSchedResult::Ok) return Result::FrameFailed;
    }

    return firstError;
}
