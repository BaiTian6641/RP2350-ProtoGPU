/**
 * @file headless_selftest.h
 * @brief Headless self-test scene provider — meshes, materials, draw calls.
 *
 * Provides scene content for the headless self-test mode.  The actual
 * rendering uses the SAME ProtoGL pipeline as the normal GPU path:
 *   SceneState → Rasterizer.PrepareFrame() → PglTileScheduler (dual-core)
 *
 * This module supplies:
 *   - BuildScene()     : one-time setup (cube + teapot meshes, 2 light
 *                        materials, perspective camera)
 *   - UpdateDrawList() : per-frame draw-call update — rotating cube for the
 *                        first 10 s, then switches to Utah teapot
 *   - DrawHUD()        : FPS/CPU% text overlay on the SSD1331 output buffer
 *
 * Only compiled when RP2350GPU_HEADLESS_SELFTEST is defined.
 */

#pragma once

#include <cstdint>

struct SceneState;

namespace HeadlessSelfTest {

/**
 * @brief One-time scene setup: meshes, materials, camera.
 *
 * Creates:
 *   - Mesh 0: unit cube (8 verts, 12 faces)
 *   - Mesh 1: Utah teapot (587 verts, 1166 faces)
 *   - Material 0: warm directional light (for the cube)
 *   - Material 1: cool directional light (for the teapot)
 *   - Camera 0: perspective at z=-5, looking toward +Z
 *
 * @param scene  Scene state (should be freshly Reset()).
 */
void BuildScene(SceneState* scene);

/**
 * @brief Rebuild the draw list for this frame.
 *
 * For the first 10 seconds: single centered rotating cube.
 * After 10 seconds: switches to the Utah teapot (rotating).
 * Both use directional Lambert shading.
 *
 * @param scene       Scene state to update (meshes/materials preserved).
 * @param cubeAngleY  Y-axis rotation angle (radians).
 * @param cubeAngleX  X-axis rotation angle (radians).
 * @param elapsedS    Seconds since startup (for cube→teapot switch).
 */
void UpdateDrawList(SceneState* scene, float cubeAngleY, float cubeAngleX,
                    float elapsedS);

/**
 * @brief Draw FPS and CPU usage text overlay onto a framebuffer.
 *
 * Uses a built-in 5×7 bitmap font.  FPS in the top-left corner (yellow),
 * CPU% in the top-right corner (cyan).
 *
 * @param fb      Framebuffer to draw into (RGB565).
 * @param w       Framebuffer width in pixels.
 * @param h       Framebuffer height in pixels.
 * @param fps     Frames per second to display.
 * @param cpuPct  CPU usage percentage to display.
 */
void DrawHUD(uint16_t* fb, uint16_t w, uint16_t h,
             uint16_t fps, uint16_t cpuPct);

}  // namespace HeadlessSelfTest
