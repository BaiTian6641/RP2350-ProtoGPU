/**
 * @file rasterizer.h
 * @brief GPU-side rasterizer — transforms, projects, and draws triangles.
 *
 * Mirrors ProtoTracer's Camera::Rasterize() but operates on the GPU's
 * SceneState and writes directly to an RGB565 framebuffer with Z-buffer.
 *
 * Core 0 calls PrepareFrame() (single-threaded: transform + project + QuadTree),
 * then both cores call RasterizeTile() on non-overlapping 16×16 tiles pulled
 * from a shared atomic work queue (PglTileScheduler).
 *
 * RasterizeRange() is kept for backward compatibility and decomposes internally
 * into full-width scanline iteration (no tile scheduling).
 */

#pragma once

#include <cstdint>

struct SceneState;  // forward

class Rasterizer {
public:
    /// Bind the scene, Z-buffer, and panel dimensions.
    void Initialize(SceneState* scene, uint16_t* zBuffer,
                    uint16_t width, uint16_t height);

    /// Phase 1 (single-threaded, Core 0):
    /// For every enabled DrawCall, transform vertices → project to 2D →
    /// insert projected triangles into the QuadTree.
    void PrepareFrame(SceneState* scene);

    /// Phase 2a — legacy (parallel, both cores):
    /// Rasterize all triangles that intersect the Y range [yStart, yEnd).
    /// Each core writes its own non-overlapping band of `framebuffer`.
    void RasterizeRange(uint16_t* framebuffer,
                        uint16_t yStart, uint16_t yEnd);

    /// Phase 2b — tile-based (parallel, both cores):
    /// Rasterize a single 16×16 tile.  The tile is identified by (tileX, tileY)
    /// in tile-grid coordinates (0-based).  The framebuffer and Z-buffer are
    /// full-panel-sized; the method only writes the tile's sub-region.
    ///
    /// Advantages over RasterizeRange:
    ///   - QuadTree query per tile (16×16 AABB) instead of per pixel column
    ///   - FB+Z data for tile fits in L1/TCM (512 + 1024 = 1.5 KB)
    ///   - Amortises traversal cost across 256 pixels
    ///
    /// @param framebuffer  Full-panel framebuffer (128×64 RGB565)
    /// @param zBuf         Full-panel Z-buffer (128×64 uint16_t)
    /// @param tileX        Tile column index (0..PANEL_WIDTH/TILE_W - 1)
    /// @param tileY        Tile row index (0..PANEL_HEIGHT/TILE_H - 1)
    /// @param tileW        Tile width in pixels (typically 16)
    /// @param tileH        Tile height in pixels (typically 16)
    void RasterizeTile(uint16_t* framebuffer, uint16_t* zBuf,
                       uint16_t tileX, uint16_t tileY,
                       uint16_t tileW, uint16_t tileH);

    /// Total projected triangles this frame (diagnostic).
    uint32_t GetTriangleCount() const { return projectedTriCount; }

    /// Returns true if the last PrepareFrame() determined the scene is
    /// identical to the previous frame and rasterization can be skipped.
    bool IsFrameSkipped() const { return frameSkipped; }

    /// Set the accumulated wall time (seconds) for animated materials.
    /// Call BEFORE RasterizeRange/RasterizeTile on each frame.
    void SetElapsedTime(float t) { elapsedTimeS = t; }

private:
    SceneState* scene    = nullptr;
    uint16_t*   zBuffer  = nullptr;
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
