/**
 * @file triangle2d.cpp
 * @brief Projected 2D triangle — barycentric, Z-interp, UV-interp.
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
    float denom = (v1.y - v2.y) * (v0.x - v2.x) +
                  (v2.x - v1.x) * (v0.y - v2.y);

    if (fabsf(denom) < 1e-6f) {
        invDenom = 0.0f;
        return false;  // degenerate
    }

    invDenom = 1.0f / denom;
    return true;
}

// ─── Barycentric ────────────────────────────────────────────────────────────

bool Triangle2D::Barycentric(float px, float py,
                             float& u, float& v, float& w) const {
    u = ((v1.y - v2.y) * (px - v2.x) + (v2.x - v1.x) * (py - v2.y)) * invDenom;
    v = ((v2.y - v0.y) * (px - v2.x) + (v0.x - v2.x) * (py - v2.y)) * invDenom;
    w = 1.0f - u - v;

    return (u >= 0.0f) && (v >= 0.0f) && (w >= 0.0f);
}

// ─── Interpolation ──────────────────────────────────────────────────────────

float Triangle2D::InterpolateZ(float u, float v, float w) const {
    return u * z0 + v * z1 + w * z2;
}

PglVec2 Triangle2D::InterpolateUV(float u, float v, float w) const {
    return {
        u * uv0.x + v * uv1.x + w * uv2.x,
        u * uv0.y + v * uv1.y + w * uv2.y
    };
}

// ─── Bounds ─────────────────────────────────────────────────────────────────

PglMath::AABB2D Triangle2D::GetBounds() const {
    return PglMath::TriangleBounds2D(v0, v1, v2);
}
