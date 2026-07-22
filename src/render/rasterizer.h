/**
 * @file rasterizer.h
 * @brief GPU-side rasterizer — transforms, projects, and draws triangles.
 *
 * Mirrors ProtoTracer's Camera::Rasterize() but operates on the GPU's
 * SceneState and writes directly to an RGB565 framebuffer with Z-buffer.
 *
 * Core 0 calls PrepareFrame() (single-threaded: frame signature check +
 * first camera pass: transform + project + QuadTree), then both cores call
 * RasterizeTile() on non-overlapping 16×16 tiles pulled from a shared atomic
 * work queue (PglTileScheduler).
 *
 * V9 (G3 multi-camera + G7 render-to-layer): the frame is a SEQUENCE of
 * per-camera passes.  PrepareFrame() prepares the first valid camera whose
 * target is the back buffer (the v8-compatible legacy binding — an unchanged
 * caller running one tile pass into the back buffer gets exactly the v8
 * image).  Multi-camera callers then loop PrepareNextCameraPass() and run a
 * tile pass per returned camera into its resolved target (back buffer or a
 * layer FB, scissored to the camera's viewport).  Cameras sharing one target
 * render in slot order; sequential passes share the panel-addressed Z buffer
 * safely (each pass clears it).
 *
 * RasterizeRange() is kept for backward compatibility and decomposes internally
 * into full-width scanline iteration (no tile scheduling).
 */

#pragma once

#include <cstdint>

struct SceneState;        // forward
struct CameraTargetInfo;  // forward (defined in scene_state.h)

class Rasterizer {
public:
    /// Bind the scene, Z-buffer, and panel dimensions.
    void Initialize(SceneState* scene, uint16_t* zBuffer,
                    uint16_t width, uint16_t height);

    /// Phase 1 (single-threaded, Core 0):
    /// Frame-signature check (identical scene → IsFrameSkipped()), then
    /// prepare the FIRST valid camera pass bound to the back buffer:
    /// Z clear → per-draw-call transform → near-plane clip (G5, perspective
    /// only) → project to 2D → insert projected triangles into the QuadTree.
    /// With no valid back-buffer camera the pass state stays "empty
    /// full-frame" (the legacy no-camera behaviour: a tile pass clears the
    /// frame to black).
    void PrepareFrame(SceneState* scene);

    /// V9 (G3/G7): advance to the next ACTIVE camera with a valid render
    /// target, skipping the one already prepared by PrepareFrame().  On
    /// success the pass state (Z cleared, QuadTree rebuilt, viewport
    /// scissor + FB stride bound) is ready for a tile pass into the
    /// camera's resolved target (SceneState::ResolveCameraTarget) and
    /// *outCamIdx holds the camera slot.  Returns false when no further
    /// camera exists.  Cameras sharing a target render in slot order.
    bool PrepareNextCameraPass(SceneState* scene, uint8_t* outCamIdx);

    /// Camera slot prepared by PrepareFrame() (the legacy back-buffer
    /// binding), or -1 when no valid back-buffer camera exists.
    int8_t GetPreparedCameraIndex() const { return preparedCamIdx; }

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
    /// V9 (G3/G7): the tile rect is intersected with the current pass's
    /// viewport scissor — tiles fully outside are skipped WITHOUT touching
    /// the target (the scissor is a clip, not a projection change; the
    /// QuadTree/cull math stays full-frame).  Framebuffer rows use the
    /// pass's target stride; the Z buffer is always panel-addressed.
    ///
    /// V9 (G4): runs as two passes — opaque materials first (Z-buffered
    /// overwrite), then PGL_BLEND_ALPHA materials (Z-tested source-over
    /// blend).  Tiles without alpha materials execute the opaque pass only.
    ///
    /// Advantages over RasterizeRange:
    ///   - QuadTree query per tile (16×16 AABB) instead of per pixel column
    ///   - FB+Z data for tile fits in L1/TCM (512 + 1024 = 1.5 KB)
    ///   - Amortises traversal cost across 256 pixels
    ///
    /// @param framebuffer  Target framebuffer (RGB565, pass stride × pass dims)
    /// @param zBuf         Full-panel Z-buffer (PANEL_WIDTH × PANEL_HEIGHT u16)
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

    // ── Per-camera pass state (V9 G3/G7) ──────────────────────────────────
    // Bound by PrepareFrame()/PrepareNextCameraPass() and consumed by
    // RasterizeTile().  Defaults (full-frame scissor, stride = panel width)
    // reproduce the v8 single-pass behaviour bit-identically.
    uint16_t    fbStride        = 0;   ///< Target FB row stride in pixels
    uint16_t    scX0 = 0, scY0 = 0;    ///< Pass viewport scissor (pixels)
    uint16_t    scX1 = 0, scY1 = 0;    ///< (exclusive; intersected per tile)
    int8_t      preparedCamIdx  = -1;  ///< Camera pass bound by PrepareFrame
    uint8_t     nextCamCursor   = 0;   ///< PrepareNextCameraPass iteration

    /// Frame signature: FNV-1a hash of draw list + camera state + mesh
    /// content versions (F-04 — parser-bumped counters, not vertex bytes).
    /// If identical to the previous frame, rasterization is skipped.
    uint32_t    prevFrameSignature = 0;

    /// Compute a hash over the current scene state relevant to rendering.
    uint32_t ComputeFrameSignature(const SceneState* scene) const;

    /// Prepare one camera's render pass: bind scissor + stride from the
    /// resolved target, clear the Z buffer, rebuild the QuadTree, and run
    /// the transform/clip/project pipeline for every enabled DrawCall.
    void PrepareCameraPass(SceneState* scene, uint8_t camIdx,
                           const CameraTargetInfo& target);
};
