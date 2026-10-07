/**
 * @file rasterizer_2d.h
 * @brief GPU-side 2D rasterizer — drawing primitives for compositing layers.
 *
 * All functions write directly to a layer's RGB565 framebuffer. Every draw is
 * clipped to the operation's clip rectangle (intersected with the target
 * extents); no out-of-bounds writes, no heap allocation, traversal is bounded
 * by the visible output rather than by off-screen radii or line lengths.
 *
 * Coordinate pipeline (per queued operation — the caller snapshots clip and
 * viewport state into the Target when the command is queued, so later
 * SetClipRect/SetViewport records never rewrite already-queued behavior):
 *
 *   device = viewOffset + ((logical * viewScaleQ8) >> 8)   (arithmetic shift,
 *                                                           floor for negatives)
 *
 * The viewport transform applies to all logical coordinates and extents.
 * Non-uniform scale turns circles/rounded corners into axis-aligned ellipses.
 * The clip rectangle is in DEVICE pixels and applies AFTER the viewport
 * transform. Non-positive Q8.8 scales are rejected by admission; if one still
 * reaches the rasterizer the draw is skipped (fail-closed, never wraps).
 *
 * Algorithms:
 *   - DrawRect:        row fill / 4-edge loop (int32-widened)
 *   - DrawLine:        Cohen–Sutherland clip + Bresenham (int32)
 *   - DrawCircle:      midpoint circle (uniform scale) / scanline ellipse
 *   - DrawRoundedRect: rect body + quarter-circle (or quarter-ellipse) corners
 *   - DrawTriangle:    scanline fill with edge sorting (int64 interpolation)
 *   - DrawArc:         1-degree trig-table stepping (≤361 iterations)
 *   - DrawSprite:      clipped blit, optional H/V flip, RGB565/RGB888 source,
 *                      nearest-neighbor stretch under viewport scale
 *   - DrawText:        RGB565 nonzero-mask glyph atlas, tinted
 *   - DrawGradientRect: per-row/column channel interpolation, endpoints exact
 *
 * Service callback: long row/span loops accept an optional core-0 service
 * function (default nullptr). The rasterizer only ever invokes the supplied
 * pointer, at bounded intervals (every kServiceInterval rows/items); core 1
 * callers pass nullptr.
 *
 * M12 — ProtoGL v0.7.2; v9 — clip/viewport snapshots, text/sprite-batch/
 * gradient records, RGB888 sources, exact blend helpers.
 */

#pragma once

#include <cstdint>
#include <PglRenderCommands.h>   // PglSpritePosition (canonical wire record)

namespace Rasterizer2D {

/// Optional core-0 service hook invoked at bounded slices of long loops.
using ServiceFn = void (*)();

/// Rows/items between service-callback invocations in long loops.
constexpr uint32_t kServiceInterval = 16;

/// Sprite source pixel formats (values mirror PglTextureFormat).
constexpr uint8_t SRC_FORMAT_RGB565 = 0;
constexpr uint8_t SRC_FORMAT_RGB888 = 1;

/// DrawText result codes (non-negative return = characters consumed).
constexpr int32_t kTextErrInvalid = -1;  ///< Null/bad args (atlas, text, dims)
constexpr int32_t kTextErrGlyph   = -2;  ///< Character has no atlas cell

/// Target framebuffer descriptor passed to all draw calls. Constructed per
/// queued operation; clip/viewport fields are the operation's snapshot.
/// Defaults reproduce legacy behavior: packed rows, full-target clip,
/// identity viewport.
struct Target {
    uint16_t* pixels     = nullptr;  ///< RGB565 pixel data
    uint16_t  width      = 0;        ///< Target width in pixels
    uint16_t  height     = 0;        ///< Target height in pixels
    uint16_t  stride     = 0;        ///< Pixels per row; 0 = packed (== width)

    // Device-space clip rectangle, applied after the viewport transform and
    // intersected with the target bounds. clipW/clipH == 0 suppresses all
    // output. The defaults select the full target.
    int16_t   clipX      = 0;
    int16_t   clipY      = 0;
    uint16_t  clipW      = 0xFFFF;
    uint16_t  clipH      = 0xFFFF;

    // Logical→device viewport: dev = viewOffset + ((logical * scaleQ8) >> 8).
    // Positive Q8.8; 256 = 1.0. Non-positive scale draws nothing.
    int16_t   viewOffsetX = 0;
    int16_t   viewOffsetY = 0;
    int16_t   viewScaleXQ8 = 256;
    int16_t   viewScaleYQ8 = 256;
};

/// Sprite blit source descriptor (real stride/extents/format/byte bounds).
struct SpriteSource {
    const uint8_t* pixels = nullptr;  ///< Texel bytes (RGB565 LE or RGB888)
    uint32_t byteLength = 0;          ///< Valid byte count; reads past it
                                      ///< return black (truncated-upload tail)
    uint16_t width     = 0;           ///< Source width in texels
    uint16_t height    = 0;           ///< Source height in texels
    uint16_t stride    = 0;           ///< Texels per row; 0 = packed (== width)
    uint8_t  format    = SRC_FORMAT_RGB565;  ///< SRC_FORMAT_*
};

/// Glyph atlas descriptor. RGB565 texels; any NONZERO texel is a mask pixel
/// drawn with the text tint. Cells are glyphW×glyphH, `columns` per row,
/// cell 0 corresponds to character code `firstChar`.
struct GlyphAtlas {
    const uint16_t* pixels = nullptr;  ///< RGB565 atlas, width × height
    uint16_t width    = 0;             ///< Atlas width in texels (row stride)
    uint16_t height   = 0;             ///< Atlas height in texels
    uint8_t  glyphW   = 0;             ///< Cell width in texels
    uint8_t  glyphH   = 0;             ///< Cell height in texels
    uint8_t  columns  = 0;             ///< Cells per atlas row
    uint8_t  firstChar = 0;            ///< Character code of cell 0
};

// ─── Primitives ─────────────────────────────────────────────────────────────

/// Fill the clip region with a solid color.
void Clear(const Target& t, uint16_t color, ServiceFn service = nullptr);

/// Draw a filled or outlined rectangle.
void DrawRect(const Target& t, int16_t x, int16_t y,
              uint16_t w, uint16_t h, uint16_t color, bool filled,
              ServiceFn service = nullptr);

/// Draw a line (Cohen–Sutherland clipped Bresenham).
void DrawLine(const Target& t, int16_t x0, int16_t y0,
              int16_t x1, int16_t y1, uint16_t color);

/// Draw a filled or outlined circle (ellipse under non-uniform viewport).
/// Radius 0 plots the center pixel, matching legacy behavior.
/// The uniform-scale contour uses the legacy integer midpoint convention
/// (initial decision 1-radius; a zero decision keeps the outer coordinate).
void DrawCircle(const Target& t, int16_t cx, int16_t cy,
                uint16_t radius, uint16_t color, bool filled,
                ServiceFn service = nullptr);

/// Draw a filled or outlined rounded rectangle.
/// Device-space corner radii are clamped to half of each device extent.
/// Uniform corners use the same integer midpoint convention as DrawCircle;
/// clamping a large radius does not preserve the silhouette of a smaller one.
void DrawRoundedRect(const Target& t, int16_t x, int16_t y,
                     uint16_t w, uint16_t h, uint16_t radius,
                     uint16_t color, bool filled,
                     ServiceFn service = nullptr);

/// Draw a filled 2D triangle using scanline rasterization.
void DrawTriangle(const Target& t, int16_t x0, int16_t y0,
                  int16_t x1, int16_t y1,
                  int16_t x2, int16_t y2, uint16_t color,
                  ServiceFn service = nullptr);

/// Draw an arc (outline only, 1° steps; radius 0 draws nothing).
void DrawArc(const Target& t, int16_t cx, int16_t cy,
             uint16_t radius, int16_t startDeg, int16_t endDeg,
             uint16_t color);

/// Blit a sprite onto the target with optional H/V flip. The device extent
/// is the source extent scaled by the viewport; scaling uses nearest-
/// neighbor source sampling. Truncated source tails read as black.
void DrawSprite(const Target& t, int16_t dstX, int16_t dstY,
                const SpriteSource& src, bool flipH, bool flipV,
                ServiceFn service = nullptr);

/// Blit the same sprite at `count` positions (PglCmdDrawSpriteBatch payload).
void DrawSpriteBatch(const Target& t, const SpriteSource& src,
                     const PglSpritePosition* positions, uint16_t count,
                     bool flipH, bool flipV,
                     ServiceFn service = nullptr);

/// Draw text from a glyph atlas. '\n' (0x0A) moves to the next line
/// (pen back to `x`, down one glyphH). The WHOLE string is validated before
/// any pixel is written: a character without an atlas cell aborts the draw
/// and returns kTextErrGlyph; bad arguments return kTextErrInvalid.
/// On success returns the number of characters consumed (== length).
int32_t DrawText(const Target& t, int16_t x, int16_t y,
                 const GlyphAtlas& atlas, const char* text, uint16_t length,
                 uint16_t color, ServiceFn service = nullptr);

/// Draw a rectangle with a two-color gradient. direction 0 = horizontal
/// (color0 at the left column, color1 at the right), 1 = vertical (color0
/// top, color1 bottom); endpoints included. A single-pixel span draws
/// color0. Channel rule: c = c0 + (c1 - c0) * i / (n - 1), truncating.
void DrawGradientRect(const Target& t, int16_t x, int16_t y,
                      uint16_t w, uint16_t h,
                      uint16_t color0, uint16_t color1, uint8_t direction,
                      ServiceFn service = nullptr);

// ─── Blending ───────────────────────────────────────────────────────────────

/// Alpha-blend two RGB565 colors. alpha = 0..255 (0 = dst, 255 = src).
/// Endpoint-exact: (src*alpha + dst*(255-alpha)) / 255 per channel.
uint16_t BlendRGB565(uint16_t src, uint16_t dst, uint8_t alpha);

/// Saturating additive blend: min(dst + src*alpha/255, channel max).
/// Endpoint-exact (alpha 0 → dst, 255 → saturating src+dst).
uint16_t BlendAddRGB565(uint16_t src, uint16_t dst, uint8_t alpha);

/// Multiply blend with opacity: lerp(dst, src*dst/max, alpha).
/// Endpoint-exact (alpha 0 → dst, 255 → src*dst/max per channel, floor).
uint16_t BlendMultiplyRGB565(uint16_t src, uint16_t dst, uint8_t alpha);

/// Layer compositing dispatcher for PglLayerBlendMode values
/// (0 = ALPHA/source-over, 1 = ADDITIVE, 2 = MULTIPLY; unknown → ALPHA).
uint16_t CompositeLayerPixel(uint16_t src, uint16_t dst,
                             uint8_t blendMode, uint8_t opacity);

}  // namespace Rasterizer2D
