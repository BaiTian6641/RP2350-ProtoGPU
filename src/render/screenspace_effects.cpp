/**
 * @file screenspace_effects.cpp
 * @brief GPU-side general screen-space shader system — RP2350 implementation.
 *
 * Three shader classes handle all post-processing:
 *   CONVOLUTION   — configurable 1D/2D blur kernel (direction, shape, radius,
 *                   auto-rotation).  Subsumes horizontal/vertical/radial blur
 *                   and anti-aliasing.
 *   DISPLACEMENT  — coordinate warp with optional per-channel chromatic split.
 *                   Subsumes PhaseOffsetX/Y/R and adds new waveform types.
 *   COLOR_ADJUST  — per-pixel colour transform.  Subsumes edge feather and
 *                   adds brightness, contrast, gamma, threshold, invert, and
 *                   Sobel edge detection.
 *
 * Performance notes (128×64 panel, Cortex-M33 @ 150 MHz):
 *   - Convolution:   O(pixels × radius), ~0.05-0.3 ms
 *   - Displacement:  O(pixels), ~0.1-0.3 ms (sinf via hardware FPU)
 *   - ColorAdjust:   O(pixels), ~0.05-0.15 ms
 *   - Worst case (4 shaders): < 1.0 ms, well within 16.6 ms budget
 */

#include "screenspace_effects.h"
#include "../scene_state.h"
#include "../gpu_config.h"

#include <PglTypes.h>

#include <cmath>
#include <cstring>
#include <cstdio>

// ─── RGB565 Helpers ─────────────────────────────────────────────────────────

static inline uint8_t R5(uint16_t c) { return (c >> 11) & 0x1F; }
static inline uint8_t G6(uint16_t c) { return (c >>  5) & 0x3F; }
static inline uint8_t B5(uint16_t c) { return  c        & 0x1F; }

static inline uint16_t PackRGB565(uint8_t r5, uint8_t g6, uint8_t b5) {
    return (static_cast<uint16_t>(r5) << 11)
         | (static_cast<uint16_t>(g6) << 5)
         | static_cast<uint16_t>(b5);
}

static inline uint8_t Clamp5(int v) { return static_cast<uint8_t>(v < 0 ? 0 : (v > 31 ? 31 : v)); }
static inline uint8_t Clamp6(int v) { return static_cast<uint8_t>(v < 0 ? 0 : (v > 63 ? 63 : v)); }
static inline int     ClampI(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float   ClampF(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static inline float MapF(float value, float inMin, float inMax, float outMin, float outMax) {
    return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
}

// ─── Oscillator Functions (stateless, driven by elapsed time) ───────────────

static constexpr float MPI = 3.14159265f;

static inline float OscSawtooth(float time, float period) {
    if (period <= 0.0001f) return 0.0f;
    return fmodf(time, period) / period;
}

/// General oscillator — returns value in [0,1] based on waveform type.
static inline float Oscillate(float time, float period, uint8_t waveform) {
    if (period <= 0.0001f) return 0.0f;
    float t = fmodf(time, period) / period;  // [0,1)
    switch (waveform) {
        default:
        case PGL_WAVE_SAWTOOTH: return t;
        case PGL_WAVE_SINE:     return 0.5f + 0.5f * sinf(2.0f * MPI * t);
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
            return expf(-(fd * fd) / (2.0f * s * s));
        }
        case PGL_KERNEL_TRIANGLE: {
            // Linearly decreasing — but we pass radius externally, so just use
            // the absolute distance.  Normalisation happens in the caller.
            return 1.0f - static_cast<float>(d < 0 ? -d : d) * 0.1f;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ── CONVOLUTION SHADER ──────────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════════════

static void ApplyConvolution(uint16_t* fb, uint16_t* scratch,
                              uint16_t w, uint16_t h,
                              float intensity, const uint8_t* params,
                              float elapsedTimeS) {
    PglShaderParamsConvolution cp;
    std::memcpy(&cp, params, sizeof(cp));

    int radius = cp.radius;
    if (radius < 1) radius = 1;
    int blurRange = ClampI(static_cast<int>(intensity * radius), 1, radius);

    // ── Separable mode (2D, 4-neighbour weighted average) ───────────────
    if (cp.separable) {
        float smoothing = (cp.sigma > 0.001f) ? ClampF(cp.sigma, 0.0f, 1.0f) : 0.25f;
        float invSmooth = 1.0f - smoothing;

        for (uint16_t y = 0; y < h; ++y) {
            uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = 0; x < w; ++x) {
                uint32_t idx = row + x;
                int cR = R5(fb[idx]), cG = G6(fb[idx]), cB = B5(fb[idx]);
                int nR = 0, nG = 0, nB = 0, nCount = 0;

                if (y > 0)     { uint32_t ni = idx - w; nR += R5(fb[ni]); nG += G6(fb[ni]); nB += B5(fb[ni]); nCount++; }
                if (y < h - 1) { uint32_t ni = idx + w; nR += R5(fb[ni]); nG += G6(fb[ni]); nB += B5(fb[ni]); nCount++; }
                if (x > 0)     { uint32_t ni = idx - 1; nR += R5(fb[ni]); nG += G6(fb[ni]); nB += B5(fb[ni]); nCount++; }
                if (x < w - 1) { uint32_t ni = idx + 1; nR += R5(fb[ni]); nG += G6(fb[ni]); nB += B5(fb[ni]); nCount++; }

                if (nCount > 0) {
                    nR /= nCount; nG /= nCount; nB /= nCount;
                    scratch[idx] = PackRGB565(
                        Clamp5(static_cast<int>(cR * invSmooth + nR * smoothing)),
                        Clamp6(static_cast<int>(cG * invSmooth + nG * smoothing)),
                        Clamp5(static_cast<int>(cB * invSmooth + nB * smoothing)));
                } else {
                    scratch[idx] = fb[idx];
                }
            }
        }
        std::memcpy(fb, scratch, static_cast<size_t>(w) * h * 2);
        return;
    }

    // ── Directional 1D kernel (angle + optional auto-rotation) ──────────

    float angleDeg = cp.angle;
    if (cp.anglePeriod > 0.001f) {
        angleDeg += OscRange(elapsedTimeS, cp.anglePeriod, PGL_WAVE_SAWTOOTH,
                             0.0f, 360.0f);
    }
    float angleRad = angleDeg * (MPI / 180.0f);
    float dirX = cosf(angleRad);
    float dirY = sinf(angleRad);

    // Simple axis-aligned fast path (no trig per-pixel)
    bool isHorizontal = (fabsf(dirY) < 0.001f);
    bool isVertical   = (fabsf(dirX) < 0.001f);

    if (isHorizontal) {
        // ── Horizontal blur fast path ───────────────────────────────────
        for (uint16_t y = 0; y < h; ++y) {
            uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = 0; x < w; ++x) {
                uint32_t idx = row + x;
                float wSum = KernelWeight(0, cp.kernelShape, cp.sigma);
                float sumR = R5(fb[idx]) * wSum;
                float sumG = G6(fb[idx]) * wSum;
                float sumB = B5(fb[idx]) * wSum;

                for (int j = 1; j <= blurRange; ++j) {
                    int xl = static_cast<int>(x) - j;
                    int xr = static_cast<int>(x) + j;
                    if (xl >= 0 && xr < w) {
                        float kw = KernelWeight(j, cp.kernelShape, cp.sigma);
                        uint32_t il = row + xl, ir = row + xr;
                        sumR += (R5(fb[il]) + R5(fb[ir])) * kw;
                        sumG += (G6(fb[il]) + G6(fb[ir])) * kw;
                        sumB += (B5(fb[il]) + B5(fb[ir])) * kw;
                        wSum += 2.0f * kw;
                    }
                }

                scratch[idx] = PackRGB565(Clamp5(static_cast<int>(sumR / wSum)),
                                           Clamp6(static_cast<int>(sumG / wSum)),
                                           Clamp5(static_cast<int>(sumB / wSum)));
            }
        }
    } else if (isVertical) {
        // ── Vertical blur fast path ─────────────────────────────────────
        for (uint16_t y = 0; y < h; ++y) {
            uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = 0; x < w; ++x) {
                uint32_t idx = row + x;
                float wSum = KernelWeight(0, cp.kernelShape, cp.sigma);
                float sumR = R5(fb[idx]) * wSum;
                float sumG = G6(fb[idx]) * wSum;
                float sumB = B5(fb[idx]) * wSum;

                for (int j = 1; j <= blurRange; ++j) {
                    int yu = static_cast<int>(y) - j;
                    int yd = static_cast<int>(y) + j;
                    if (yu >= 0 && yd < h) {
                        float kw = KernelWeight(j, cp.kernelShape, cp.sigma);
                        uint32_t iu = static_cast<uint32_t>(yu) * w + x;
                        uint32_t id = static_cast<uint32_t>(yd) * w + x;
                        sumR += (R5(fb[iu]) + R5(fb[id])) * kw;
                        sumG += (G6(fb[iu]) + G6(fb[id])) * kw;
                        sumB += (B5(fb[iu]) + B5(fb[id])) * kw;
                        wSum += 2.0f * kw;
                    }
                }

                scratch[idx] = PackRGB565(Clamp5(static_cast<int>(sumR / wSum)),
                                           Clamp6(static_cast<int>(sumG / wSum)),
                                           Clamp5(static_cast<int>(sumB / wSum)));
            }
        }
    } else {
        // ── General angled blur (radial / diagonal / arbitrary) ─────────
        float cx = static_cast<float>(w) * 0.5f;
        float cy = static_cast<float>(h) * 0.5f;

        for (uint16_t y = 0; y < h; ++y) {
            uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = 0; x < w; ++x) {
                uint32_t idx = row + x;

                // For auto-rotating mode, use direction from centre rotated
                float rx = dirX, ry = dirY;
                if (cp.anglePeriod > 0.001f) {
                    float dx = static_cast<float>(x) - cx;
                    float dy = static_cast<float>(y) - cy;
                    float len = sqrtf(dx * dx + dy * dy);
                    if (len > 0.001f) {
                        float cosA = cosf(angleRad), sinA = sinf(angleRad);
                        rx = (dx * cosA - dy * sinA) / len;
                        ry = (dx * sinA + dy * cosA) / len;
                    }
                }

                float wSum = KernelWeight(0, cp.kernelShape, cp.sigma);
                float sumR = R5(fb[idx]) * wSum;
                float sumG = G6(fb[idx]) * wSum;
                float sumB = B5(fb[idx]) * wSum;

                for (int j = 1; j <= blurRange; ++j) {
                    int sx1 = static_cast<int>(x + rx * j);
                    int sy1 = static_cast<int>(y + ry * j);
                    int sx2 = static_cast<int>(x - rx * j);
                    int sy2 = static_cast<int>(y - ry * j);

                    if (sx1 >= 0 && sx1 < w && sy1 >= 0 && sy1 < h &&
                        sx2 >= 0 && sx2 < w && sy2 >= 0 && sy2 < h) {
                        float kw = KernelWeight(j, cp.kernelShape, cp.sigma);
                        uint32_t i1 = static_cast<uint32_t>(sy1) * w + sx1;
                        uint32_t i2 = static_cast<uint32_t>(sy2) * w + sx2;
                        sumR += (R5(fb[i1]) + R5(fb[i2])) * kw;
                        sumG += (G6(fb[i1]) + G6(fb[i2])) * kw;
                        sumB += (B5(fb[i1]) + B5(fb[i2])) * kw;
                        wSum += 2.0f * kw;
                    }
                }

                scratch[idx] = PackRGB565(Clamp5(static_cast<int>(sumR / wSum)),
                                           Clamp6(static_cast<int>(sumG / wSum)),
                                           Clamp5(static_cast<int>(sumB / wSum)));
            }
        }
    }

    std::memcpy(fb, scratch, static_cast<size_t>(w) * h * 2);
}

// ═══════════════════════════════════════════════════════════════════════════
// ── DISPLACEMENT SHADER ─────────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════════════

static void ApplyDisplacement(uint16_t* fb, uint16_t* scratch,
                               uint16_t w, uint16_t h,
                               float intensity, const uint8_t* params,
                               float elapsedTimeS) {
    PglShaderParamsDisplacement dp;
    std::memcpy(&dp, params, sizeof(dp));

    float amplitude = static_cast<float>(dp.amplitude);
    float range = (amplitude - 1.0f) * intensity + 1.0f;
    float freq  = (dp.frequency > 0.001f) ? dp.frequency : 1.0f;

    // Primary oscillator phase (time-based animation)
    float oscPhase = (dp.period > 0.001f)
                   ? 2.0f * MPI * Oscillate(elapsedTimeS, dp.period, dp.waveform)
                   : 0.0f;

    static constexpr float PI2_OVER3 = 2.0f * MPI * 0.333f;
    static constexpr float PI4_OVER3 = 2.0f * MPI * 0.666f;

    bool chromatic = (dp.perChannel != 0);

    // ── Axis X: horizontal displacement ─────────────────────────────────
    if (dp.axis == PGL_AXIS_X) {
        for (uint16_t y = 0; y < h; ++y) {
            uint32_t row = static_cast<uint32_t>(y) * w;
            float coordY = static_cast<float>(y) / (10.0f / freq);

            for (uint16_t x = 0; x < w; ++x) {
                uint32_t idx = row + x;

                if (chromatic) {
                    float sR = sinf(coordY + oscPhase * 8.0f);
                    float sG = sinf(coordY + oscPhase * 8.0f + PI2_OVER3);
                    float sB = sinf(coordY + oscPhase * 8.0f + PI4_OVER3);

                    int oR = ClampI(static_cast<int>(MapF(sR, -1.f, 1.f, 1.f, range)), 1, static_cast<int>(range));
                    int oG = ClampI(static_cast<int>(MapF(sG, -1.f, 1.f, 1.f, range)), 1, static_cast<int>(range));
                    int oB = ClampI(static_cast<int>(MapF(sB, -1.f, 1.f, 1.f, range)), 1, static_cast<int>(range));

                    int xR = static_cast<int>(x) + oR;
                    int xG = static_cast<int>(x) + oG;
                    int xB = static_cast<int>(x) + oB;

                    uint8_t r = (xR >= 0 && xR < w) ? R5(fb[row + xR]) : 0;
                    uint8_t g = (xG >= 0 && xG < w) ? G6(fb[row + xG]) : 0;
                    uint8_t b = (xB >= 0 && xB < w) ? B5(fb[row + xB]) : 0;
                    scratch[idx] = PackRGB565(r, g, b);
                } else {
                    float s = sinf(coordY + oscPhase * 8.0f);
                    int off = ClampI(static_cast<int>(MapF(s, -1.f, 1.f, 1.f, range)), 1, static_cast<int>(range));
                    int sx = static_cast<int>(x) + off;
                    scratch[idx] = (sx >= 0 && sx < w) ? fb[row + sx] : 0;
                }
            }
        }
        std::memcpy(fb, scratch, static_cast<size_t>(w) * h * 2);
        return;
    }

    // ── Axis Y: vertical displacement ───────────────────────────────────
    if (dp.axis == PGL_AXIS_Y) {
        for (uint16_t y = 0; y < h; ++y) {
            uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = 0; x < w; ++x) {
                uint32_t idx = row + x;
                float coordX = static_cast<float>(x) / (10.0f / freq);

                if (chromatic) {
                    float sR = sinf(coordX + oscPhase * 8.0f);
                    float sG = sinf(coordX + oscPhase * 8.0f + PI2_OVER3);
                    float sB = sinf(coordX + oscPhase * 8.0f + PI4_OVER3);

                    int oR = ClampI(static_cast<int>(MapF(sR, -1.f, 1.f, 1.f, range)), 1, static_cast<int>(range));
                    int oG = ClampI(static_cast<int>(MapF(sG, -1.f, 1.f, 1.f, range)), 1, static_cast<int>(range));
                    int oB = ClampI(static_cast<int>(MapF(sB, -1.f, 1.f, 1.f, range)), 1, static_cast<int>(range));

                    int yR = static_cast<int>(y) - oR;
                    int yG = static_cast<int>(y) - oG;
                    int yB = static_cast<int>(y) - oB;

                    uint8_t r = (yR >= 0 && yR < h) ? R5(fb[static_cast<uint32_t>(yR) * w + x]) : 0;
                    uint8_t g = (yG >= 0 && yG < h) ? G6(fb[static_cast<uint32_t>(yG) * w + x]) : 0;
                    uint8_t b = (yB >= 0 && yB < h) ? B5(fb[static_cast<uint32_t>(yB) * w + x]) : 0;
                    scratch[idx] = PackRGB565(r, g, b);
                } else {
                    float s = sinf(coordX + oscPhase * 8.0f);
                    int off = ClampI(static_cast<int>(MapF(s, -1.f, 1.f, 1.f, range)), 1, static_cast<int>(range));
                    int sy = static_cast<int>(y) - off;
                    scratch[idx] = (sy >= 0 && sy < h) ? fb[static_cast<uint32_t>(sy) * w + x] : 0;
                }
            }
        }
        std::memcpy(fb, scratch, static_cast<size_t>(w) * h * 2);
        return;
    }

    // ── Axis RADIAL: radial chromatic aberration ────────────────────────
    {
        float rotPeriod = (dp.period > 0.001f) ? dp.period : 3.7f;
        float p1Period  = (dp.phase1Period > 0.001f) ? dp.phase1Period : 4.5f;
        float p2Period  = (dp.phase2Period > 0.001f) ? dp.phase2Period : 3.2f;

        float rotation = OscRange(elapsedTimeS, rotPeriod, dp.waveform, 0.0f, 360.0f);
        float offset1  = OscSawtooth(elapsedTimeS, p1Period);
        float offset2  = OscSawtooth(elapsedTimeS, p2Period);

        float phase120 = 2.0f * MPI * 0.333f;
        float phase240 = 2.0f * MPI * 0.666f;
        float mpiR = 2.0f * MPI * 8.0f;

        for (uint16_t y = 0; y < h; ++y) {
            uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = 0; x < w; ++x) {
                uint32_t idx = row + x;

                float coordX = static_cast<float>(x) / (10.0f / freq);
                float coordY = static_cast<float>(y) / (5.0f / freq);

                float sineR = sinf(coordX + mpiR * offset1) + cosf(coordY + mpiR * offset2);
                float sineG = sinf(coordX + (mpiR + phase120) * offset1) + cosf(coordY + (mpiR + phase120) * offset2);
                float sineB = sinf(coordX + (mpiR + phase240) * offset1) + cosf(coordY + (mpiR + phase240) * offset2);

                int blurR = ClampI(static_cast<int>(MapF(sineR, -2.f, 2.f, 1.f, range)), 1, static_cast<int>(range));
                int blurG = ClampI(static_cast<int>(MapF(sineG, -2.f, 2.f, 1.f, range)), 1, static_cast<int>(range));
                int blurB = ClampI(static_cast<int>(MapF(sineB, -2.f, 2.f, 1.f, range)), 1, static_cast<int>(range));

                auto SampleRadial = [&](float angleDeg, int dist) -> uint32_t {
                    float rad = angleDeg * (MPI / 180.0f);
                    int sx = static_cast<int>(x) + static_cast<int>(dist * cosf(rad));
                    int sy = static_cast<int>(y) + static_cast<int>(dist * sinf(rad));
                    if (sx >= 0 && sx < w && sy >= 0 && sy < h)
                        return static_cast<uint32_t>(sy) * w + sx;
                    return UINT32_MAX;
                };

                uint32_t idxR = SampleRadial(rotation,          blurR);
                uint32_t idxG = SampleRadial(rotation + 120.0f, blurG);
                uint32_t idxB = SampleRadial(rotation + 240.0f, blurB);

                uint8_t r = (idxR != UINT32_MAX) ? R5(fb[idxR]) : 0;
                uint8_t g = (idxG != UINT32_MAX) ? G6(fb[idxG]) : 0;
                uint8_t b = (idxB != UINT32_MAX) ? B5(fb[idxB]) : 0;

                scratch[idx] = PackRGB565(r, g, b);
            }
        }
        std::memcpy(fb, scratch, static_cast<size_t>(w) * h * 2);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ── COLOR ADJUST SHADER ─────────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════════════

static void ApplyColorAdjust(uint16_t* fb, uint16_t* scratch,
                              uint16_t w, uint16_t h,
                              float /*intensity*/, const uint8_t* params) {
    PglShaderParamsColorAdjust cp;
    std::memcpy(&cp, params, sizeof(cp));

    const uint32_t totalPixels = static_cast<uint32_t>(w) * h;

    switch (cp.operation) {

    // ── Edge Feather (dim pixels adjacent to black) ─────────────────────
    case PGL_COLOR_EDGE_FEATHER: {
        float strength = cp.strength;
        for (uint16_t y = 0; y < h; ++y) {
            uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = 0; x < w; ++x) {
                uint32_t idx = row + x;
                if (fb[idx] == 0x0000) continue;

                bool isEdge = false;
                if (y == 0 || fb[idx - w] == 0x0000) isEdge = true;
                if (!isEdge && (y == h - 1 || fb[idx + w] == 0x0000)) isEdge = true;
                if (!isEdge && (x == 0 || fb[idx - 1] == 0x0000)) isEdge = true;
                if (!isEdge && (x == w - 1 || fb[idx + 1] == 0x0000)) isEdge = true;

                if (isEdge) {
                    uint8_t r = static_cast<uint8_t>(R5(fb[idx]) * strength);
                    uint8_t g = static_cast<uint8_t>(G6(fb[idx]) * strength);
                    uint8_t b = static_cast<uint8_t>(B5(fb[idx]) * strength);
                    fb[idx] = PackRGB565(r, g, b);
                }
            }
        }
        break;
    }

    // ── Brightness ──────────────────────────────────────────────────────
    case PGL_COLOR_BRIGHTNESS: {
        // strength: -1.0 to +1.0 mapped to 5/6-bit delta
        float delta5 = cp.strength * 31.0f;
        float delta6 = cp.strength * 63.0f;
        for (uint32_t i = 0; i < totalPixels; ++i) {
            fb[i] = PackRGB565(Clamp5(static_cast<int>(R5(fb[i]) + delta5)),
                               Clamp6(static_cast<int>(G6(fb[i]) + delta6)),
                               Clamp5(static_cast<int>(B5(fb[i]) + delta5)));
        }
        break;
    }

    // ── Contrast ────────────────────────────────────────────────────────
    case PGL_COLOR_CONTRAST: {
        // strength: 0.0 = flat grey, 1.0 = unchanged, 2.0 = double contrast
        float s = cp.strength;
        for (uint32_t i = 0; i < totalPixels; ++i) {
            float r = (R5(fb[i]) / 31.0f - 0.5f) * s + 0.5f;
            float g = (G6(fb[i]) / 63.0f - 0.5f) * s + 0.5f;
            float b = (B5(fb[i]) / 31.0f - 0.5f) * s + 0.5f;
            fb[i] = PackRGB565(Clamp5(static_cast<int>(r * 31.0f)),
                               Clamp6(static_cast<int>(g * 63.0f)),
                               Clamp5(static_cast<int>(b * 31.0f)));
        }
        break;
    }

    // ── Gamma ───────────────────────────────────────────────────────────
    case PGL_COLOR_GAMMA: {
        float gamma = (cp.param2 > 0.01f) ? cp.param2 : 2.2f;
        float invGamma = 1.0f / gamma;
        for (uint32_t i = 0; i < totalPixels; ++i) {
            float r = powf(R5(fb[i]) / 31.0f, invGamma);
            float g = powf(G6(fb[i]) / 63.0f, invGamma);
            float b = powf(B5(fb[i]) / 31.0f, invGamma);
            fb[i] = PackRGB565(Clamp5(static_cast<int>(r * 31.0f)),
                               Clamp6(static_cast<int>(g * 63.0f)),
                               Clamp5(static_cast<int>(b * 31.0f)));
        }
        break;
    }

    // ── Threshold ───────────────────────────────────────────────────────
    case PGL_COLOR_THRESHOLD: {
        float thresh = cp.strength;  // 0.0–1.0
        for (uint32_t i = 0; i < totalPixels; ++i) {
            float lum = (R5(fb[i]) / 31.0f * 0.299f +
                         G6(fb[i]) / 63.0f * 0.587f +
                         B5(fb[i]) / 31.0f * 0.114f);
            fb[i] = (lum >= thresh) ? 0xFFFF : 0x0000;
        }
        break;
    }

    // ── Invert ──────────────────────────────────────────────────────────
    case PGL_COLOR_INVERT: {
        for (uint32_t i = 0; i < totalPixels; ++i) {
            fb[i] = PackRGB565(31 - R5(fb[i]), 63 - G6(fb[i]), 31 - B5(fb[i]));
        }
        break;
    }

    // ── Edge Detect (Sobel) ─────────────────────────────────────────────
    case PGL_COLOR_EDGE_DETECT: {
        float scale = (cp.strength > 0.01f) ? cp.strength : 1.0f;
        for (uint16_t y = 1; y < h - 1; ++y) {
            uint32_t row = static_cast<uint32_t>(y) * w;
            for (uint16_t x = 1; x < w - 1; ++x) {
                uint32_t idx = row + x;
                // Luminance for 3×3 neighbourhood (using green channel, 6-bit)
                auto L = [&](int dx, int dy) -> float {
                    return G6(fb[static_cast<uint32_t>(y + dy) * w + (x + dx)]) / 63.0f;
                };
                float gx = -L(-1,-1) + L(1,-1) - 2*L(-1,0) + 2*L(1,0) - L(-1,1) + L(1,1);
                float gy = -L(-1,-1) - 2*L(0,-1) - L(1,-1) + L(-1,1) + 2*L(0,1) + L(1,1);
                float mag = ClampF(sqrtf(gx*gx + gy*gy) * scale, 0.0f, 1.0f);
                uint8_t r5 = static_cast<uint8_t>(mag * 31.0f);
                uint8_t g6 = static_cast<uint8_t>(mag * 63.0f);
                scratch[idx] = PackRGB565(r5, g6, r5);
            }
        }
        // Border pixels → black
        for (uint16_t x = 0; x < w; ++x) {
            scratch[x] = 0;
            scratch[(h-1) * w + x] = 0;
        }
        for (uint16_t y = 0; y < h; ++y) {
            scratch[y * w] = 0;
            scratch[y * w + w - 1] = 0;
        }
        std::memcpy(fb, scratch, static_cast<size_t>(w) * h * 2);
        break;
    }

    default:
        break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ── DISPATCHER + PUBLIC API ─────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════════════

static void ApplySingleShader(uint16_t* fb, uint16_t* scratch,
                               uint16_t w, uint16_t h,
                               const ShaderSlot& slot,
                               float elapsedTimeS) {
    switch (slot.shaderClass) {
        case PGL_SHADER_CONVOLUTION:
            ApplyConvolution(fb, scratch, w, h, slot.intensity, slot.params, elapsedTimeS);
            break;
        case PGL_SHADER_DISPLACEMENT:
            ApplyDisplacement(fb, scratch, w, h, slot.intensity, slot.params, elapsedTimeS);
            break;
        case PGL_SHADER_COLOR_ADJUST:
            ApplyColorAdjust(fb, scratch, w, h, slot.intensity, slot.params);
            break;
        default:
            break;
    }
}

void ScreenspaceShaders::ApplyShaders(uint16_t* framebuffer,
                                       uint16_t* scratchBuffer,
                                       uint16_t width, uint16_t height,
                                       const SceneState* scene,
                                       float elapsedTimeS) {
    for (uint8_t c = 0; c < PGL_MAX_CAMERAS; ++c) {
        const CameraSlot& cam = scene->cameras[c];
        if (!cam.active) continue;

        for (uint8_t s = 0; s < PGL_MAX_SHADERS_PER_CAMERA; ++s) {
            const ShaderSlot& slot = cam.shaders[s];
            if (!slot.active) continue;

            ApplySingleShader(framebuffer, scratchBuffer, width, height,
                              slot, elapsedTimeS);
        }
    }
}
