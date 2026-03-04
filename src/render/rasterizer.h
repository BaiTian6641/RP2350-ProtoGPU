/**
 * @file rasterizer.h
 * @brief GPU-side rasterizer — transforms, projects, and draws triangles.
 *
 * Mirrors ProtoTracer's Camera::Rasterize() but operates on the GPU's
 * SceneState and writes directly to an RGB565 framebuffer with Z-buffer.
 *
 * Core 0 calls PrepareFrame() (single-threaded: transform + project + QuadTree),
 * then both cores call RasterizeRange() on non-overlapping Y bands.
 */

#pragma once

#include <cstdint>

struct SceneState;  // forward

class Rasterizer {
public:
    /// Bind the scene, Z-buffer, and panel dimensions.
    void Initialize(SceneState* scene, float* zBuffer,
                    uint16_t width, uint16_t height);

    /// Phase 1 (single-threaded, Core 0):
    /// For every enabled DrawCall, transform vertices → project to 2D →
    /// insert projected triangles into the QuadTree.
    void PrepareFrame(SceneState* scene);

    /// Phase 2 (parallel, both cores):
    /// Rasterize all triangles that intersect the Y range [yStart, yEnd).
    /// Each core writes its own non-overlapping band of `framebuffer`.
    void RasterizeRange(uint16_t* framebuffer,
                        uint16_t yStart, uint16_t yEnd);

    /// Total projected triangles this frame (diagnostic).
    uint32_t GetTriangleCount() const { return projectedTriCount; }

    /// Returns true if the last PrepareFrame() determined the scene is
    /// identical to the previous frame and rasterization can be skipped.
    bool IsFrameSkipped() const { return frameSkipped; }

    /// Set the accumulated wall time (seconds) for animated materials.
    /// Call BEFORE RasterizeRange on each frame.
    void SetElapsedTime(float t) { elapsedTimeS = t; }

private:
    SceneState* scene    = nullptr;
    float*      zBuffer  = nullptr;
    uint16_t    width    = 0;
    uint16_t    height   = 0;

    uint32_t    projectedTriCount = 0;
    bool        frameSkipped      = false;
    float       elapsedTimeS      = 0.0f;  ///< animated material time

    /// Frame signature: FNV-1a hash of draw list + camera state.
    /// If identical to the previous frame, rasterization is skipped.
    uint32_t    prevFrameSignature = 0;

    /// Compute a hash over the current scene state relevant to rendering.
    uint32_t ComputeFrameSignature(const SceneState* scene) const;
};
