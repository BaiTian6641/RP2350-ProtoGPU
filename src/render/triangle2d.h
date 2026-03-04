/**
 * @file triangle2d.h
 * @brief Projected 2D triangle representation used during rasterization.
 *
 * After the rasterizer transforms + projects each mesh triangle to screen
 * space, it creates a Triangle2D and inserts it into the QuadTree.
 *
 * During per-pixel rasterization, Triangle2D provides barycentric coordinate
 * interpolation, Z-interpolation, and UV-interpolation.
 */

#pragma once

#include <cstdint>
#include <PglTypes.h>
#include "../math/pgl_math.h"

struct Triangle2D {
    // Screen-space projected vertices
    PglVec2 v0, v1, v2;

    // Depth at each vertex (for Z-buffer interpolation)
    float z0, z1, z2;

    // UV coordinates at each vertex (optional — only if mesh has UVs)
    PglVec2 uv0, uv1, uv2;
    bool    hasUV;

    // Face normal in camera/world space (computed during PrepareFrame).
    // Used by NormalMaterial and LightMaterial for shading.
    PglVec3 faceNormal;

    // Back-reference to the originating draw call and triangle index
    uint16_t drawCallIndex;
    uint16_t meshTriIndex;

    // Pre-computed for barycentric interpolation
    float invDenom;   // 1 / (edgeA cross edgeB)

    // Precomputed edge coefficients — eliminates 4 subtractions per pixel
    float e10y;   // v1.y - v2.y  (edge1 Δy)
    float e21x;   // v2.x - v1.x  (edge2 Δx)
    float e20y;   // v2.y - v0.y  (edge2 Δy)
    float e02x;   // v0.x - v2.x  (edge0 Δx)

    // ── Methods ─────────────────────────────────────────────────────────

    /// Set up the triangle from projected vertices.  Computes invDenom.
    /// Returns false if the triangle is degenerate (zero area).
    bool Setup(const PglVec2& a, const PglVec2& b, const PglVec2& c,
               float za, float zb, float zc);

    /// Compute barycentric coordinates (u, v, w) for a point (px, py).
    /// u + v + w ≈ 1.  Returns false if point is outside the triangle.
    bool Barycentric(float px, float py,
                     float& u, float& v, float& w) const;

    /// Interpolate depth at barycentric (u, v, w).
    float InterpolateZ(float u, float v, float w) const;

    /// Interpolate UV at barycentric (u, v, w).
    PglVec2 InterpolateUV(float u, float v, float w) const;

    /// Axis-aligned bounding box (for QuadTree insertion).
    PglMath::AABB2D GetBounds() const;
};
