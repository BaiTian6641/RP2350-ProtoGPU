/**
 * @file rasterizer_2d.h
 * @brief GPU-side 2D rasterizer — drawing primitives for compositing layers.
 *
 * All functions write directly to a layer's RGB565 framebuffer.
 * Pixel coordinates are clipped to the layer bounds (no out-of-bounds writes).
 *
 * Algorithms:
 *   - DrawRect: memset / 4-edge loop
 *   - DrawLine: Bresenham's line algorithm
 *   - DrawCircle: Midpoint circle algorithm
 *   - DrawRoundedRect: Rect body + quarter-circle corners
 *   - DrawTriangle2D: Scanline fill with edge sorting
 *   - DrawArc: Incremental angle stepping with trig lookup
 *   - DrawSprite: Block copy from texture with optional flip
 *
 * M12 — ProtoGL v0.7.2
 */

#pragma once

#include <cstdint>

namespace Rasterizer2D {

/// Target framebuffer descriptor passed to all draw calls.
struct Target {
    uint16_t* pixels;   ///< Pointer to RGB565 pixel data
    uint16_t  width;    ///< Framebuffer width
    uint16_t  height;   ///< Framebuffer height
};

// ─── Primitives ─────────────────────────────────────────────────────────────

/// Draw a filled or outlined rectangle.
void DrawRect(const Target& t, int16_t x, int16_t y,
              uint16_t w, uint16_t h, uint16_t color, bool filled);

/// Draw a line using Bresenham's algorithm.
void DrawLine(const Target& t, int16_t x0, int16_t y0,
              int16_t x1, int16_t y1, uint16_t color);

/// Draw a filled or outlined circle using the midpoint algorithm.
void DrawCircle(const Target& t, int16_t cx, int16_t cy,
                uint16_t radius, uint16_t color, bool filled);

/// Draw a filled or outlined rounded rectangle.
void DrawRoundedRect(const Target& t, int16_t x, int16_t y,
                     uint16_t w, uint16_t h, uint16_t radius,
                     uint16_t color, bool filled);

/// Draw a filled 2D triangle using scanline rasterization.
void DrawTriangle(const Target& t, int16_t x0, int16_t y0,
                  int16_t x1, int16_t y1,
                  int16_t x2, int16_t y2, uint16_t color);

/// Draw an arc (outline only).
void DrawArc(const Target& t, int16_t cx, int16_t cy,
             uint16_t radius, int16_t startDeg, int16_t endDeg,
             uint16_t color);

/// Blit an RGB565 texture onto the target (with optional H/V flip).
/// srcPixels must be srcW × srcH RGB565.
void DrawSprite(const Target& t, int16_t dstX, int16_t dstY,
                const uint16_t* srcPixels, uint16_t srcW, uint16_t srcH,
                bool flipH, bool flipV);

/// Fill the entire target with a solid color.
void Clear(const Target& t, uint16_t color);

/// Alpha-blend two RGB565 colors. alpha = 0..255 (0=fully dst, 255=fully src).
uint16_t BlendRGB565(uint16_t src, uint16_t dst, uint8_t alpha);

}  // namespace Rasterizer2D
