/**
 * @file triangle2d.cpp
 * @brief Projected 2D triangle — setup + per-tile coefficient derivation.
 *
 * Derive() is the single place where barycentric edge coefficients,
 * reciprocal depths, perspective UV weights, and the top-left shared-edge
 * inclusion bits are computed.  It runs once per triangle per tile, so the
 * inner pixel loops execute only FMAs/compares — no per-pixel subtractions
 * beyond the (px,py) pixel-centre offset and no per-pixel divides except the
 * single perspective depth reconstruction (1/rz) for covered candidates.
 *
 * Top-left shared-edge rule (declared coverage contract): a pixel centre
 * lying exactly on an edge is covered iff that edge is a "top" edge
 * (horizontal, triangle interior below it) or a "left" edge (non-horizontal,
 * interior to its right).  With the interior on the mathematical left of
 * each directed edge (guaranteed by the positive-area winding kept by the
 * back-face cull) and a y-down screen, the rule reduces to:
 *
 *     edge (A -> B) is inclusive  <=>  (B.y - A.y < 0) ||
 *                                      (B.y - A.y == 0 && B.x - A.x > 0)
 *
 * Adjacent triangles then own every shared-edge pixel exactly once — no
 * cracks and no double-blended edges.
 */

#include "triangle2d.h"
#include <cmath>

// ─── Setup ──────────────────────────────────────────────────────────────────

bool Triangle2D::Setup(const PglVec2& a, const PglVec2& b, const PglVec2& c,
                       float za, float zb, float zc) {
    v0 = a;  v1 = b;  v2 = c;
    z0 = za; z1 = zb; z2 = zc;
    flags         = 0;
    drawCallIndex = 0;
    meshTriIndex  = 0;

    // Reject non-finite coordinates/depth before any reciprocal or edge
    // evaluation; a finite AABB alone can hide a NaN vertex via fmin/fmax.
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(b.x) || !std::isfinite(b.y) ||
        !std::isfinite(c.x) || !std::isfinite(c.y) ||
        !std::isfinite(za) || !std::isfinite(zb) || !std::isfinite(zc) ||
        !(za > 0.0f) || !(zb > 0.0f) || !(zc > 0.0f)) return false;
    const float denom = fmaf(v1.y - v2.y, v0.x - v2.x,
                             (v2.x - v1.x) * (v0.y - v2.y));
    return std::isfinite(denom) && fabsf(denom) >= 1e-6f;
}

// ─── Per-tile derivation ────────────────────────────────────────────────────

void Triangle2D::Derive(Deriv& d) const {
    // Barycentric edge coefficients relative to v2 — identical expressions
    // (and therefore identical bits) to the historical Triangle2D::Setup()
    // precompute, just produced per (triangle, tile) instead of stored.
    d.e10y = v1.y - v2.y;   // edge1 Δy
    d.e21x = v2.x - v1.x;   // edge2 Δx
    d.e20y = v2.y - v0.y;   // edge2 Δy
    d.e02x = v0.x - v2.x;   // edge0 Δx
    d.e01y = v0.y - v1.y;   // third edge, evaluated relative to v0
    d.e10x = v1.x - v0.x;

    const float denom = fmaf(v1.y - v2.y, v0.x - v2.x,
                             (v2.x - v1.x) * (v0.y - v2.y));
    d.invDenom = 1.0f / denom;   // denom ≥ 1e-6 guaranteed by Setup()

    // Top-left inclusion per barycentric edge:
    //   u == 0 on the edge v1 -> v2,  v == 0 on v2 -> v0,  w == 0 on v0 -> v1.
    uint8_t inc = 0;
    {
        const float dx = v2.x - v1.x, dy = v2.y - v1.y;
        if (dy < 0.0f || (dy == 0.0f && dx > 0.0f)) inc |= 0x01;
    }
    {
        const float dx = v0.x - v2.x, dy = v0.y - v2.y;
        if (dy < 0.0f || (dy == 0.0f && dx > 0.0f)) inc |= 0x02;
    }
    {
        const float dx = v1.x - v0.x, dy = v1.y - v0.y;
        if (dy < 0.0f || (dy == 0.0f && dx > 0.0f)) inc |= 0x04;
    }
    d.inc = inc;

    // Perspective-correct interpolation terms.  1/z is linear in screen
    // space under a pinhole projection, so per-vertex reciprocals and
    // z-scaled UVs interpolate linearly and the true attribute/depth is
    // reconstructed with one division at covered pixels.
    if (flags & PERSPECTIVE) {
        // All pooled perspective depths are > kNearPlaneZ > 0 (near-plane
        // clip), so the reciprocals are finite and strictly positive.
        d.rz0 = 1.0f / z0;
        d.rz1 = 1.0f / z1;
        d.rz2 = 1.0f / z2;
        if (flags & HAS_UV) {
            d.wu0 = uv0.x * d.rz0;  d.wv0 = uv0.y * d.rz0;
            d.wu1 = uv1.x * d.rz1;  d.wv1 = uv1.y * d.rz1;
            d.wu2 = uv2.x * d.rz2;  d.wv2 = uv2.y * d.rz2;
        }
    }
}
