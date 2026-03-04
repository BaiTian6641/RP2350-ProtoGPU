/**
 * @file screenspace_effects.h
 * @brief [SHADER:FUTURE] GPU-side screen-space post-processing shader system.
 *
 * General-purpose shader pipeline that operates on the RGB565 framebuffer
 * AFTER rasterization completes and BEFORE the framebuffer swap.  A scratch
 * buffer (typically the Z-buffer, reclaimed after raster) is used for shaders
 * that need a secondary pixel buffer.
 *
 * Shaders are applied per-camera in slot order (0..PGL_MAX_SHADERS_PER_CAMERA-1).
 *
 * Shader classes (PglShaderClass):
 *   0x01 CONVOLUTION   — Configurable 1D/2D blur kernel (direction, shape,
 *                         radius, auto-rotation).  Expresses: horizontal blur,
 *                         vertical blur, diagonal blur, radial blur,
 *                         anti-aliasing, gaussian blur, etc.
 *   0x02 DISPLACEMENT  — Coordinate-space warp with per-channel chromatic
 *                         split.  Expresses: phase-offset X/Y/radial,
 *                         sine/triangle/sawtooth wave distortion, etc.
 *   0x03 COLOR_ADJUST  — Per-pixel colour transform.  Expresses: edge
 *                         feathering, brightness, contrast, gamma, threshold,
 *                         invert, Sobel edge detection, etc.
 *
 * [SHADER:FUTURE] Refinement areas:
 *   - Host-side Effect → shader mapping (requires Effect RTTI or registration)
 *   - Multi-pass shader chaining (convolution + displacement in sequence)
 *   - Shader parameter animation (time-varying intensity, kernel modulation)
 *   - Shader slot allocation strategy across multiple cameras
 */

#pragma once

#include <cstdint>

struct SceneState;  // forward

namespace ScreenspaceShaders {

/**
 * @brief Apply all active screen-space shaders for every camera.
 *
 * Must be called AFTER both cores finish rasterization, BEFORE framebuffer swap.
 * Uses `scratchBuffer` (same size as framebuffer) for shaders that need
 * read-from-src / write-to-dst semantics.  The caller typically passes the
 * Z-buffer (reinterpreted as uint16_t*) since Z data is stale post-raster.
 *
 * @param framebuffer   Back-buffer being composed (RGB565, width × height).
 * @param scratchBuffer Temporary buffer, at least width × height × 2 bytes.
 * @param width         Panel width in pixels.
 * @param height        Panel height in pixels.
 * @param scene         Scene state containing camera shader slots.
 * @param elapsedTimeS  Accumulated wall time in seconds (for animated shaders).
 */
void ApplyShaders(uint16_t* framebuffer,
                  uint16_t* scratchBuffer,
                  uint16_t width, uint16_t height,
                  const SceneState* scene,
                  float elapsedTimeS);

}  // namespace ScreenspaceShaders
