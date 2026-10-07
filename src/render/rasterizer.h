/**
 * @file rasterizer.h
 * @brief GPU-side rasterizer — transforms, clips, projects, and draws triangles.
 *
 * Core 0 calls PrepareFrame() (single-threaded: first back-buffer camera
 * pass — transform + near-clip + project + per-tile coverage), then both
 * cores call RasterizeTile() on non-overlapping 16×16 tiles pulled from a
 * shared atomic work queue (PglTileScheduler).  Multi-camera callers loop
 * PrepareNextCameraPass() and run a tile pass per returned camera into its
 * resolved target (back buffer or a layer FB, scissored to the camera's
 * viewport).  Cameras execute in slot order; sequential passes share the
 * fixed-capacity Z workspace safely (each pass clears its target extent).
 *
 * ── Declared conventions (protocol 9) ──────────────────────────────────────
 * Screen space:  panel pixels, origin top-left, y grows downward, pixel
 *                centres at n + 0.5. Projection remains full panel-space;
 *                a camera viewport is a scissor CLIP, not a projection change.
 * Perspective:   camera faces +Z in view space; fov factor = panelW/2;
 *                view-space near clip at z = 0.001 (Sutherland–Hodgman,
 *                1–2 output triangles, UVs interpolated at crossings); no
 *                far-plane clip.  Depth and UV interpolate PERSPECTIVE-
 *                CORRECT (1/z and uv/z linear in screen space, one
 *                reconstructing division per covered pixel).
 * Orthographic:  declared pixel-space mapping screen = (world − camPos)·camScale
 *                + screenCentre; depth = transformed world z; triangles with
 *                any vertex z ≤ 0 are dropped (no clip — no singularity).
 * Camera state:  view rotation = rotation ∘ baseRotation ∘ lookOffset
 *                (ProtoTracer order; lookOffset applies first, in the camera
 *                frame).  View-space coordinates are then scaled component-
 *                wise by the camera scale (ProtoTracer camera-scale
 *                semantics; identity scale is bit-identical to before).
 * Coverage:      top-left shared-edge rule — a pixel centre exactly on an
 *                edge belongs to the triangle for which that edge is a top
 *                or left edge; shared edges are owned exactly once (no
 *                cracks, no double-blended alpha edges).
 * Depth buffer:  uint16 upper bits of a positive IEEE float (monotonic,
 *                quantized), cleared to 0xFFFF, strict LESS test — on equal
 *                quantized depths the first-submitted triangle wins.
 * Blending:      two passes per tile in stable submission (pool) order —
 *                opaque first (strict-depth write), then PGL_BLEND_ALPHA
 *                materials source-over in submission order, depth-tested
 *                against the opaque depth buffer with NO depth writes.
 *                alpha == 0 and mask-discarded pixels write neither colour
 *                nor depth, so they never occlude.  alpha == 1 is bit-
 *                identical to an opaque write.
 * Capacity:      the projected-triangle pool is bounded (GpuConfig::
 *                MAX_TRIANGLES, includes near-clip expansion).  Overflow is
 *                NOT silent: the overflowing triangle is dropped and
 *                GetFrameError() returns PglRuntime::Result::RenderOverflow
 *                (the integrator must not present the failed target).
 *                Per-tile candidate selection is lossless: every accepted
 *                triangle carries compact conservative tile bounds and is
 *                visited in emission order by every overlapping tile; there
 *                are no capped candidate lists.
 * Service:       RasterizeTile()'s optional final service callback runs at
 *                bounded slices (once per completed triangle pass and once
 *                per tile clear band); the scheduler supplies it on core 0
 *                and nullptr on core 1.  SetServiceCallback() installs the
 *                same for the transform/clip preparation slices (core 0).
 */

#pragma once

#include <cstdint>
#include <PglRuntimeProtocol.h>
#include "../phase_scratch.h"

struct SceneState;        // forward
struct CameraTargetInfo;  // forward (defined in scene_state.h)

class Rasterizer {
public:
    /// Bind the scene and typed shared world/depth workspace.
    void Initialize(SceneState* scene, PhaseScratch::DepthWorkspace& depth,
                    uint16_t width, uint16_t height);

    /// Phase 1 (single-threaded, core 0):
    /// Reset the per-frame pipeline state (pool, frame error, pass state)
    /// and prepare the FIRST valid camera pass bound to the back buffer:
    /// Per-draw transform → near-plane clip (perspective only) → projection
    /// → conservative tile bounds → depth clear. With no valid
    /// back-buffer camera the pass state stays "empty full-frame" (a tile
    /// pass clears the frame to black).  Every frame is prepared in full —
    /// there is no frame-signature skip.
    void PrepareFrame(SceneState* scene);

    /// Advance to the next ACTIVE camera with a valid render target,
    /// skipping the one already prepared by PrepareFrame().  On success the
    /// pass state (Z cleared, coverage rebuilt, viewport scissor + FB
    /// stride bound) is ready for a tile pass into the camera's resolved
    /// target (SceneState::ResolveCameraTarget) and *outCamIdx holds the
    /// camera slot.  Returns false when no further camera exists.  Cameras
    /// execute in slot order.
    bool PrepareNextCameraPass(SceneState* scene, uint8_t* outCamIdx);

    /// Camera slot prepared by PrepareFrame() (the legacy back-buffer
    /// binding), or -1 when no valid back-buffer camera exists.
    int8_t GetPreparedCameraIndex() const { return preparedCamIdx; }

    /// Phase 2 (parallel, both cores):
    /// Rasterize a single tile.  The tile is identified by (tileX, tileY)
    /// in tile-grid coordinates (0-based, tileW × tileH pixels, normally
    /// 16×16).  The tile rect is intersected with the current pass's
    /// viewport scissor AND actual target extents; tiles fully outside are
    /// skipped WITHOUT touching the target.  Framebuffer and Z rows both
    /// use the actual target width; the shared Z workspace has the same
    /// maximum pixel capacity as a framebuffer.
    ///
    /// Runs as two passes in stable submission order — opaque (strict-depth
    /// write) then translucent (PGL_BLEND_ALPHA source-over, depth test
    /// against the opaque depth, no depth writes).  Tiles whose pass has no
    /// translucent materials execute the opaque pass only.
    ///
    /// @param service  Optional host-service callback (core 0 only);
    ///                 invoked at bounded slices: once per completed
    ///                 triangle per pass and once per tile clear band.
    void RasterizeTile(uint16_t* framebuffer, uint16_t* zBuf,
                       uint16_t tileX, uint16_t tileY,
                       uint16_t tileW, uint16_t tileH,
                       void (*service)() = nullptr);

    /// Total source triangles projected this frame (diagnostic; includes
    /// triangles later culled/dropped, excludes clip-expansion copies).
    uint32_t GetTriangleCount() const { return projectedTriCount; }

    /// Result of the current frame's preparation passes.  Ok on success;
    /// RenderOverflow when the projected-triangle pool overflowed (the
    /// extra triangles were dropped — the integrator must not present the
    /// failed target); InvalidValue for invalid internal state.  Sticky
    /// for the whole frame (first error wins), reset by PrepareFrame().
    PglRuntime::Result GetFrameError() const { return frameError; }

    /// Set the accumulated wall time (seconds) for animated materials.
    /// Call BEFORE RasterizeTile() on each frame.
    void SetElapsedTime(float t) { elapsedTimeS = t; }

    /// Install the host-service callback for the single-threaded
    /// preparation slices (transform/clip/project).  Called once per
    /// processed draw call.  Core 0 installs it; nullptr disables.
    void SetServiceCallback(void (*service)()) { prepService = service; }

private:
    SceneState* scene    = nullptr;
    PhaseScratch::DepthWorkspace* depthWorkspace = nullptr;
    uint16_t    width    = 0;
    uint16_t    height   = 0;

    uint32_t    projectedTriCount = 0;
    float       elapsedTimeS      = 0.0f;  ///< animated material time
    PglRuntime::Result frameError = PglRuntime::Result::Ok;
    void      (*prepService)()    = nullptr;

    // ── Per-camera pass state ────────────────────────────────────────────
    // Bound by PrepareFrame()/PrepareNextCameraPass() and consumed by
    // RasterizeTile().  Defaults (full-frame scissor, stride = panel width)
    // reproduce the single-pass behaviour.
    uint16_t    fbStride        = 0;   ///< Target FB row stride in pixels
    uint16_t    targetWidth = 0, targetHeight = 0; ///< Actual pass extent
    uint16_t    scX0 = 0, scY0 = 0;    ///< Pass viewport scissor (pixels)
    uint16_t    scX1 = 0, scY1 = 0;    ///< (exclusive; intersected per tile)
    int8_t      preparedCamIdx  = -1;  ///< Camera pass bound by PrepareFrame
    uint8_t     nextCamCursor   = 0;   ///< PrepareNextCameraPass iteration
    bool        passHasTranslucent = false;  ///< Any deferred-alpha triangle

    // ── Tile coverage grid (target space) ────────────────────────────────
    // cols×rows cells of kTileW×kTileH pixels over the target; ≤ 64 cells.
    uint32_t    gridCols        = 0;
    uint32_t    gridRows        = 0;
    bool        gridValid       = false;

    /// Record a preparation failure (first error wins for the frame).
    void RecordFrameError(PglRuntime::Result r) {
        if (frameError == PglRuntime::Result::Ok) frameError = r;
    }

    /// Prepare one camera's render pass: bind scissor + stride from the
    /// resolved target, clear the Z buffer, and run the
    /// transform/clip/project/coverage pipeline for every enabled DrawCall.
    void PrepareCameraPass(SceneState* scene, uint8_t camIdx,
                           const CameraTargetInfo& target);
};
