/**
 * @file rasterizer.cpp
 * @brief GPU-side rasterizer implementation (M1 skeleton — builds, does not render yet).
 *
 * Milestone progress:
 *   M0-M1: Skeleton compiles, PrepareFrame/RasterizeRange are stubs.
 *   M3:    Fill in vertex transform, perspective project, QuadTree build.
 *   M4:    Fill in per-pixel rasterization (barycentric, Z-test, shading).
 *   M5:    Material evaluation, blending, screen-space shaders.
 */

#include "rasterizer.h"
#include "../scene_state.h"
#include "../gpu_config.h"
#include "../math/pgl_math.h"
#include "triangle2d.h"
#include "quadtree.h"

#include <cstring>
#include <cstdio>

// ─── Initialize ─────────────────────────────────────────────────────────────

void Rasterizer::Initialize(SceneState* scene, float* zBuffer,
                            uint16_t width, uint16_t height) {
    this->scene   = scene;
    this->zBuffer = zBuffer;
    this->width   = width;
    this->height  = height;
    this->projectedTriCount = 0;
}

// ─── PrepareFrame (Core 0, single-threaded) ─────────────────────────────────
//
// For each enabled DrawCall:
//   1. Look up MeshSlot + CameraSlot
//   2. Transform vertices by DrawCall.transform
//   3. Project to 2D (perspective or ortho)
//   4. Build Triangle2D list and insert into QuadTree
//
// TODO(M4): Implement the full pipeline.  For now, just count triangles.

void Rasterizer::PrepareFrame(SceneState* scene) {
    this->scene = scene;
    projectedTriCount = 0;

    // Clear Z-buffer
    const uint32_t pixelCount = static_cast<uint32_t>(width) * height;
    for (uint32_t i = 0; i < pixelCount; ++i) {
        zBuffer[i] = 1e30f;  // far plane
    }

    // TODO(M4): Iterate draw calls, transform, project, build QuadTree
    for (uint16_t d = 0; d < scene->drawCallCount; ++d) {
        const DrawCall& dc = scene->drawList[d];
        if (!dc.enabled) continue;
        if (dc.meshId >= GpuConfig::MAX_MESHES) continue;

        const MeshSlot& mesh = scene->meshes[dc.meshId];
        if (!mesh.active) continue;

        // Count triangles even though we don't rasterize yet
        projectedTriCount += mesh.triangleCount;

        // TODO(M4): For each vertex, TransformVertex() then Project()
        // TODO(M4): For each triangle, create Triangle2D, insert into QuadTree
    }

    if (projectedTriCount > 0) {
        printf("[Rasterizer] PrepareFrame: %u draw calls, %u triangles\n",
               scene->drawCallCount, projectedTriCount);
    }
}

// ─── RasterizeRange (both cores, parallel) ──────────────────────────────────
//
// For each triangle that intersects [yStart, yEnd):
//   1. Compute barycentric coordinates per pixel
//   2. Z-test against zBuffer
//   3. Evaluate material → get RGB
//   4. Write RGB565 to framebuffer
//
// TODO(M4): Implement the full per-pixel loop.
// For now (M1), fill the band with a test pattern so HUB75 has something to show.

void Rasterizer::RasterizeRange(uint16_t* framebuffer,
                                uint16_t yStart, uint16_t yEnd) {
    // M1 test pattern: vertical gradient (green channel increases with Y)
    for (uint16_t y = yStart; y < yEnd && y < height; ++y) {
        uint8_t green = static_cast<uint8_t>((y * 63) / (height - 1));  // 6-bit green
        uint16_t color = static_cast<uint16_t>(green << 5);  // RGB565: 00000 GGGGGG 00000

        uint32_t rowStart = static_cast<uint32_t>(y) * width;
        for (uint16_t x = 0; x < width; ++x) {
            framebuffer[rowStart + x] = color;
        }
    }
}
