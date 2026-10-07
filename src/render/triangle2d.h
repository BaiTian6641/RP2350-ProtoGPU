/**
 * @file triangle2d.h
 * @brief Projected 2D triangle record used during rasterization.
 *
 * After the rasterizer transforms + projects each mesh triangle to screen
 * space it stores one compact Triangle2D per accepted (projected) triangle
 * in the frame pool.  The record is deliberately attribute-only: every
 * quantity derivable from the vertices (barycentric edge coefficients,
 * reciprocal depths, perspective UV weights, top-left edge inclusion) is
 * derived ONCE per triangle per tile by Derive() — outside the pixel loop —
 * instead of being stored redundantly in every pool slot.  This keeps the
 * record at 80 bytes while retaining full float XY/UV/normal quality.
 *
 * Per-triangle conservative tile coverage (the lossless replacement for the
 * old QuadTree/capped-candidate lists) lives in parallel 4-byte inclusive
 * tile bounds owned by the rasterizer — see rasterizer.cpp.
 */

#pragma once

#include <cstdint>
#include <PglTypes.h>

struct Triangle2D {
    // Screen-space projected vertices (panel pixel space, y grows downward,
    // pixel centres at n + 0.5).
    PglVec2 v0, v1, v2;             // 24 B

    // Positive view-space depth at each vertex.  For perspective triangles
    // this is the true (post-clip, post camera-scale) view z; for the
    // declared orthographic convention it is the transformed world z.
    float z0, z1, z2;               // 12 B

    // UV coordinates at each vertex (meaningful only when flags & HAS_UV).
    PglVec2 uv0, uv1, uv2;          // 24 B

    // Face normal from the unclipped transformed 3D vertices (flat per face;
    // used by NormalMaterial and LightMaterial).
    PglVec3 faceNormal;             // 12 B

    // Back-references (packed: index + flags share the final word).
    uint16_t drawCallIndex;         //  2 B — index into SceneState::drawList
    uint16_t meshTriIndex;          //  2 B — source triangle index (diagnostics)
    uint8_t  flags;                 //  1 B — Flags bitmask
    // 80 B total (3 B tail padding)

    enum Flags : uint8_t {
        HAS_UV      = 0x01,  ///< uv0..uv2 are valid
        PERSPECTIVE = 0x02,  ///< perspective-correct interp; clear = affine ortho
        TRANSLUCENT = 0x04,  ///< PGL_BLEND_ALPHA material — deferred second pass
    };

    /// Per-(triangle, tile) derived coefficients.  Computed once per
    /// triangle per tile outside the pixel loops; never stored per triangle.
    struct Deriv {
        float invDenom;         // 1 / signed double-area (positive — cull kept CCW)
        float e10y, e21x;       // u-numerator edge coefficients (vs v2)
        float e20y, e02x;       // v-numerator edge coefficients (vs v2)
        float e01y, e10x;       // third edge coefficients (relative to v0)
        float rz0, rz1, rz2;    // 1/z per vertex (PERSPECTIVE only)
        float wu0, wu1, wu2;    // uv.x * rz per vertex (PERSPECTIVE + HAS_UV)
        float wv0, wv1, wv2;    // uv.y * rz per vertex (PERSPECTIVE + HAS_UV)
        uint8_t inc;            // bit0/1/2: u/v/w == 0 edge is top-left (inclusive)
    };

    /// Fill the triangle from projected vertices.  Returns false when the
    /// triangle is degenerate (zero/non-finite double-area); the pool slot is
    /// then reused by the next candidate.
    bool Setup(const PglVec2& a, const PglVec2& b, const PglVec2& c,
               float za, float zb, float zc);

    /// Derive all three edge equations once per tile.  Coverage tests the
    /// unnormalised edges independently (never a rounded 1-u-v residual).
    /// Perspective terms are computed only when their matching flags are set.
    void Derive(Deriv& d) const;
};

static_assert(sizeof(Triangle2D) == 80,
              "Triangle2D must stay compact (80 B) for the projected pool");
