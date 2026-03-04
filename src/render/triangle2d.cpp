/**
 * @file triangle2d.cpp
 * @brief Projected 2D triangle — barycentric, Z-interp, UV-interp.
 *
 * Cortex-M33 DSP optimization notes:
 *   - Barycentric Setup() precomputes edge coefficients to reduce per-pixel work.
 *   - Barycentric() uses fmaf() (FMA in single cycle on CM33 FPU) to eliminate
 *     separate multiply + add instructions.
 *   - InterpolateZ() uses fmaf() for fused multiply-accumulate.
 *   - These changes are always-on (no compile flag) because fmaf() has zero
 *     overhead on CM33 vs separate mul+add and reduces rounding error.
 */

#include "triangle2d.h"
#include <cmath>

// ─── Setup ──────────────────────────────────────────────────────────────────

bool Triangle2D::Setup(const PglVec2& a, const PglVec2& b, const PglVec2& c,
                       float za, float zb, float zc) {
    v0 = a;  v1 = b;  v2 = c;
    z0 = za; z1 = zb; z2 = zc;
    hasUV = false;
    drawCallIndex = 0;
    meshTriIndex  = 0;

    // Compute denominator for barycentric coordinates:
    //   denom = (v1.y - v2.y)*(v0.x - v2.x) + (v2.x - v1.x)*(v0.y - v2.y)
    float denom = fmaf(v1.y - v2.y, v0.x - v2.x,
                       (v2.x - v1.x) * (v0.y - v2.y));

    if (fabsf(denom) < 1e-6f) {
        invDenom = 0.0f;
        return false;  // degenerate
    }

    invDenom = 1.0f / denom;

    // Precompute edge coefficients for Barycentric() — reduces per-pixel
    // work from 4 subtracts + 4 multiplies to 2 subtracts + 2 FMAs + 1 subtract.
    e10y = v1.y - v2.y;  // edge1 delta Y
    e21x = v2.x - v1.x;  // edge2 delta X
    e20y = v2.y - v0.y;  // edge2 delta Y
    e02x = v0.x - v2.x;  // edge0 delta X

    return true;
}

// ─── Barycentric ────────────────────────────────────────────────────────────
// Uses precomputed edge coefficients and fmaf() for fused multiply-accumulate.
// On Cortex-M33 FPU, fmaf() is single-cycle and reduces rounding error.

bool Triangle2D::Barycentric(float px, float py,
                             float& u, float& v, float& w) const {
    float dx = px - v2.x;
    float dy = py - v2.y;

    u = fmaf(e10y, dx, e21x * dy) * invDenom;
    v = fmaf(e20y, dx, e02x * dy) * invDenom;
    w = 1.0f - u - v;

    return (u >= 0.0f) && (v >= 0.0f) && (w >= 0.0f);
}

// ─── Interpolation ──────────────────────────────────────────────────────────
// Uses fmaf() chains for Z and UV interpolation.

float Triangle2D::InterpolateZ(float u, float v, float w) const {
    return fmaf(u, z0, fmaf(v, z1, w * z2));
}

PglVec2 Triangle2D::InterpolateUV(float u, float v, float w) const {
    return {
        fmaf(u, uv0.x, fmaf(v, uv1.x, w * uv2.x)),
        fmaf(u, uv0.y, fmaf(v, uv1.y, w * uv2.y))
    };
}

// ─── Bounds ─────────────────────────────────────────────────────────────────

PglMath::AABB2D Triangle2D::GetBounds() const {
    return PglMath::TriangleBounds2D(v0, v1, v2);
}
