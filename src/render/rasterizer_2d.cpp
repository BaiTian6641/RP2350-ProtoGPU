/**
 * @file rasterizer_2d.cpp
 * @brief GPU-side 2D rasterizer implementation.
 *
 * All primitives clip to the operation's clip rectangle (intersected with the
 * target extents). No heap allocation. Arithmetic is widened to int32/int64
 * before clipping and viewport transforms; traversal is capped to the visible
 * output (Cohen–Sutherland line clipping, circle bounding-box reject plus a
 * Chebyshev break, clip-row-bounded ellipse/triangle loops) so huge
 * off-screen radii or line lengths cannot spin the CPU.
 *
 * Optimized for RP2350 Cortex-M33 @ 150 MHz with small RGB565 targets
 * (≤ 8192 logical pixels).
 *
 * M12 — ProtoGL v0.7.2; v9 — clip/viewport snapshots, glyph text, sprite
 * batch, gradient, RGB888 sources, exact blend helpers.
 */

#include "rasterizer_2d.h"

#include <cstring>
#include <cstdlib>    // abs
#include <algorithm>  // std::swap, std::min, std::max

// ─── Helpers ────────────────────────────────────────────────────────────────

namespace {

using Rasterizer2D::Target;
using Rasterizer2D::SpriteSource;
using Rasterizer2D::GlyphAtlas;
using Rasterizer2D::ServiceFn;

/// Half-open device-space clip box, intersected with the target bounds.
struct ClipBox {
    int32_t x0, y0, x1, y1;  // [x0,x1) × [y0,y1)
};

inline uint32_t TargetStride(const Target& t) {
    return t.stride ? t.stride : t.width;
}

/// Compute the effective clip box. Returns false when nothing can be drawn
/// (null/empty target or empty clip).
bool ComputeClip(const Target& t, ClipBox& c) {
    if (!t.pixels || t.width == 0 || t.height == 0) return false;
    const int64_t cx1 = static_cast<int64_t>(t.clipX) + t.clipW;
    const int64_t cy1 = static_cast<int64_t>(t.clipY) + t.clipH;
    c.x0 = (t.clipX > 0) ? t.clipX : 0;
    c.y0 = (t.clipY > 0) ? t.clipY : 0;
    c.x1 = (cx1 < t.width)  ? static_cast<int32_t>(cx1) : t.width;
    c.y1 = (cy1 < t.height) ? static_cast<int32_t>(cy1) : t.height;
    return c.x0 < c.x1 && c.y0 < c.y1;
}

/// Admission rejects non-positive viewport scales; fail closed if one slips.
inline bool ViewportOk(const Target& t) {
    return t.viewScaleXQ8 > 0 && t.viewScaleYQ8 > 0;
}

/// Logical → device transform, widened before the multiply.
/// dev = offset + ((v * scaleQ8) >> 8)  (arithmetic shift: floor).
inline int32_t TxX(const Target& t, int32_t v) {
    return t.viewOffsetX +
           static_cast<int32_t>((static_cast<int64_t>(v) * t.viewScaleXQ8) >> 8);
}
inline int32_t TxY(const Target& t, int32_t v) {
    return t.viewOffsetY +
           static_cast<int32_t>((static_cast<int64_t>(v) * t.viewScaleYQ8) >> 8);
}

/// Logical extent → device extent (floor).
inline int32_t TxLen(int32_t len, int16_t scaleQ8) {
    return static_cast<int32_t>((static_cast<int64_t>(len) * scaleQ8) >> 8);
}

/// Safely write one pixel with clip checking.
inline void PutPixel(const Target& t, const ClipBox& c,
                     int32_t x, int32_t y, uint16_t color) {
    if (x >= c.x0 && x < c.x1 && y >= c.y0 && y < c.y1) {
        t.pixels[y * TargetStride(t) + x] = color;
    }
}

/// Fill an already-clipped row span [x0, x1] inclusive (word-fast middle).
inline void FillSpanRaw(const Target& t, int32_t x0, int32_t x1,
                        int32_t y, uint16_t color) {
    uint16_t* row = t.pixels + y * TargetStride(t);
    int32_t n = x1 - x0 + 1;
    uint16_t* p = row + x0;
    // Scalar head to 4-byte alignment
    while (n > 0 && (reinterpret_cast<uintptr_t>(p) & 0x3)) {
        *p++ = color;
        --n;
    }
    const uint32_t c32 = (static_cast<uint32_t>(color) << 16) | color;
    uint32_t* p32 = reinterpret_cast<uint32_t*>(p);
    int32_t words = n >> 1;
    for (int32_t i = 0; i < words; ++i) p32[i] = c32;
    if (n & 1) p[n - 1] = color;
}

/// Draw a horizontal span (clipped, inclusive endpoints).
inline void HLine(const Target& t, const ClipBox& c,
                  int32_t x0, int32_t x1, int32_t y, uint16_t color) {
    if (y < c.y0 || y >= c.y1) return;
    if (x0 > x1) std::swap(x0, x1);
    if (x1 < c.x0 || x0 >= c.x1) return;
    if (x0 < c.x0) x0 = c.x0;
    if (x1 >= c.x1) x1 = c.x1 - 1;
    FillSpanRaw(t, x0, x1, y, color);
}

/// Draw a vertical span (clipped, inclusive endpoints).
inline void VLine(const Target& t, const ClipBox& c,
                  int32_t x, int32_t y0, int32_t y1, uint16_t color) {
    if (x < c.x0 || x >= c.x1) return;
    if (y0 > y1) std::swap(y0, y1);
    if (y1 < c.y0 || y0 >= c.y1) return;
    if (y0 < c.y0) y0 = c.y0;
    if (y1 >= c.y1) y1 = c.y1 - 1;
    const uint32_t stride = TargetStride(t);
    uint16_t* p = t.pixels + y0 * stride + x;
    for (int32_t y = y0; y <= y1; ++y) {
        *p = color;
        p += stride;
    }
}

/// Fill a device-space half-open rectangle, clipped.
inline void FillRectDev(const Target& t, const ClipBox& c,
                        int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                        uint16_t color) {
    if (x0 < c.x0) x0 = c.x0;
    if (y0 < c.y0) y0 = c.y0;
    if (x1 > c.x1) x1 = c.x1;
    if (y1 > c.y1) y1 = c.y1;
    // An intersection can be empty or reversed (a texel wholly left/right
    // of the clip, or one collapsed by fractional viewport scaling).
    // FillSpanRaw requires a nonempty span: a negative count can otherwise
    // reach its odd-tail write and overwrite a pixel outside the clip.
    if (x0 >= x1 || y0 >= y1) return;
    for (int32_t y = y0; y < y1; ++y) {
        FillSpanRaw(t, x0, x1 - 1, y, color);
    }
}

/// Plot 8 symmetric circle points (midpoint circle, outline).
inline void CirclePoints(const Target& t, const ClipBox& c,
                         int32_t cx, int32_t cy, int32_t dx, int32_t dy,
                         uint16_t color) {
    PutPixel(t, c, cx + dx, cy + dy, color);
    PutPixel(t, c, cx - dx, cy + dy, color);
    PutPixel(t, c, cx + dx, cy - dy, color);
    PutPixel(t, c, cx - dx, cy - dy, color);
    PutPixel(t, c, cx + dy, cy + dx, color);
    PutPixel(t, c, cx - dy, cy + dx, color);
    PutPixel(t, c, cx + dy, cy - dx, color);
    PutPixel(t, c, cx - dy, cy - dx, color);
}

/// Fill 4 symmetric horizontal spans (midpoint circle, filled).
inline void CircleHLines(const Target& t, const ClipBox& c,
                         int32_t cx, int32_t cy, int32_t dx, int32_t dy,
                         uint16_t color) {
    HLine(t, c, cx - dx, cx + dx, cy + dy, color);
    HLine(t, c, cx - dx, cx + dx, cy - dy, color);
    HLine(t, c, cx - dy, cx + dy, cy + dx, color);
    HLine(t, c, cx - dy, cx + dy, cy - dx, color);
}

/// 1D distance from v to the inclusive interval [lo, hi] (0 when inside).
inline int32_t DistToInterval(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo - v;
    if (v > hi) return v - hi;
    return 0;
}

/// Integer floor square root of a non-negative int64 (bit-by-bit, exact).
uint32_t ISqrt64(uint64_t v) {
    uint64_t res = 0;
    uint64_t bit = uint64_t{1} << 62;  // highest even power of four ≤ 2^63
    while (bit > v) bit >>= 2;
    while (bit != 0) {
        if (v >= res + bit) {
            v -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return static_cast<uint32_t>(res);
}

/// Fetch one sprite texel as RGB565. Reads past byteLength return black,
/// matching the 3D pipeline's truncated-upload convention.
inline uint16_t FetchSrcTexel(const SpriteSource& s, uint32_t col, uint32_t row) {
    const uint32_t stride = s.stride ? s.stride : s.width;
    const uint64_t idx = static_cast<uint64_t>(row) * stride + col;
    if (s.format == Rasterizer2D::SRC_FORMAT_RGB888) {
        const uint64_t b = idx * 3;
        if (s.byteLength && b + 2 >= s.byteLength) return 0x0000;
        // Same channel reduction as the 3D sampler: truncate to 5/6/5.
        return static_cast<uint16_t>(((s.pixels[b] >> 3) << 11) |
                                     ((s.pixels[b + 1] >> 2) << 5) |
                                     (s.pixels[b + 2] >> 3));
    }
    const uint64_t b = idx * 2;
    if (s.byteLength && b + 1 >= s.byteLength) return 0x0000;
    return static_cast<uint16_t>(s.pixels[b]) |
           (static_cast<uint16_t>(s.pixels[b + 1]) << 8);
}

/// Core sprite blit: device rect [x0d,x1d) × [y0d,y1d), nearest sampling.
void BlitSprite(const Target& t, const ClipBox& c, const SpriteSource& src,
                int32_t x0d, int32_t y0d, int32_t x1d, int32_t y1d,
                bool flipH, bool flipV, ServiceFn service) {
    const int32_t dw = x1d - x0d;
    const int32_t dh = y1d - y0d;
    if (dw <= 0 || dh <= 0) return;

    int32_t px0 = (x0d > c.x0) ? x0d : c.x0;
    int32_t py0 = (y0d > c.y0) ? y0d : c.y0;
    int32_t px1 = (x1d < c.x1) ? x1d : c.x1;
    int32_t py1 = (y1d < c.y1) ? y1d : c.y1;
    if (px0 >= px1 || py0 >= py1) return;

    const bool unity = (dw == src.width) && (dh == src.height);
    const bool rgb565 = (src.format == Rasterizer2D::SRC_FORMAT_RGB565);
    const uint32_t sStride = src.stride ? src.stride : src.width;
    const uint32_t tStride = TargetStride(t);

    for (int32_t dy = py0; dy < py1; ++dy) {
        int32_t srow = static_cast<int32_t>(
            (static_cast<int64_t>(dy - y0d) * src.height) / dh);
        if (flipV) srow = src.height - 1 - srow;
        uint16_t* drow = t.pixels + dy * tStride;

        if (unity && rgb565 && !flipH) {
            // Row fast path: direct copy of the visible column window.
            const int32_t scol0 = px0 - x0d;
            const int32_t len = px1 - px0;
            const uint64_t firstByte =
                (static_cast<uint64_t>(srow) * sStride + scol0) * 2;
            const uint64_t lastByte = firstByte + static_cast<uint64_t>(len) * 2;
            if (!src.byteLength || lastByte <= src.byteLength) {
                std::memcpy(drow + px0, src.pixels + firstByte,
                            static_cast<size_t>(len) * sizeof(uint16_t));
            } else {
                for (int32_t dx = px0; dx < px1; ++dx) {
                    drow[dx] = FetchSrcTexel(src, dx - x0d, srow);
                }
            }
        } else {
            for (int32_t dx = px0; dx < px1; ++dx) {
                int32_t scol = static_cast<int32_t>(
                    (static_cast<int64_t>(dx - x0d) * src.width) / dw);
                if (flipH) scol = src.width - 1 - scol;
                drow[dx] = FetchSrcTexel(src, scol, srow);
            }
        }

        if (service && (((dy - py0) + 1) % Rasterizer2D::kServiceInterval) == 0) {
            service();
        }
    }
}

/// Scanline ellipse fill (non-uniform viewport circles / rounded corners).
/// Row-iteration is bounded by the clip box. Half-width per row:
/// xhalf = floor(rx * isqrt(ry² - dy²) / ry).
void EllipseFillRows(const Target& t, const ClipBox& c,
                     int32_t cx, int32_t cy, int32_t rx, int32_t ry,
                     uint16_t color, ServiceFn service) {
    int32_t y0 = cy - ry; if (y0 < c.y0) y0 = c.y0;
    int32_t y1 = cy + ry; if (y1 >= c.y1) y1 = c.y1 - 1;
    const uint64_t ry2 = static_cast<uint64_t>(ry) * ry;
    for (int32_t y = y0; y <= y1; ++y) {
        const int64_t dy = y - cy;
        const uint64_t rem = ry2 - static_cast<uint64_t>(dy * dy);
        const int32_t xhalf = static_cast<int32_t>(
            (static_cast<uint64_t>(rx) * ISqrt64(rem)) / static_cast<uint64_t>(ry));
        HLine(t, c, cx - xhalf, cx + xhalf, y, color);
        if (service && (((y - y0) + 1) % Rasterizer2D::kServiceInterval) == 0) {
            service();
        }
    }
}

/// Scanline ellipse outline: dual row/column sweep so steep and shallow arcs
/// both stay closed. Bounded by the clip box extents.
void EllipseOutline(const Target& t, const ClipBox& c,
                    int32_t cx, int32_t cy, int32_t rx, int32_t ry,
                    uint16_t color) {
    const uint64_t ry2 = static_cast<uint64_t>(ry) * ry;
    const uint64_t rx2 = static_cast<uint64_t>(rx) * rx;
    int32_t y0 = cy - ry; if (y0 < c.y0) y0 = c.y0;
    int32_t y1 = cy + ry; if (y1 >= c.y1) y1 = c.y1 - 1;
    for (int32_t y = y0; y <= y1; ++y) {
        const int64_t dy = y - cy;
        const uint64_t rem = ry2 - static_cast<uint64_t>(dy * dy);
        const int32_t xhalf = static_cast<int32_t>(
            (static_cast<uint64_t>(rx) * ISqrt64(rem)) / static_cast<uint64_t>(ry));
        PutPixel(t, c, cx - xhalf, y, color);
        PutPixel(t, c, cx + xhalf, y, color);
    }
    int32_t x0 = cx - rx; if (x0 < c.x0) x0 = c.x0;
    int32_t x1 = cx + rx; if (x1 >= c.x1) x1 = c.x1 - 1;
    for (int32_t x = x0; x <= x1; ++x) {
        const int64_t dx = x - cx;
        const uint64_t rem = rx2 - static_cast<uint64_t>(dx * dx);
        const int32_t yhalf = static_cast<int32_t>(
            (static_cast<uint64_t>(ry) * ISqrt64(rem)) / static_cast<uint64_t>(rx));
        PutPixel(t, c, x, cy - yhalf, color);
        PutPixel(t, c, x, cy + yhalf, color);
    }
}

/// One corner of an outlined ellipse-derived rounded rect (non-uniform
/// viewport). Plots only the quadrant selected by (negX, negY) relative to
/// the corner center. Loops are bounded by the clip box.
void EllipseCornerOutline(const Target& t, const ClipBox& c,
                          int32_t ccx, int32_t ccy, int32_t rx, int32_t ry,
                          bool negX, bool negY, uint16_t color) {
    const uint64_t ry2 = static_cast<uint64_t>(ry) * ry;
    const uint64_t rx2 = static_cast<uint64_t>(rx) * rx;
    // Row sweep: dy from 0..ry in the selected vertical direction.
    int32_t dy0 = negY ? ccy - (c.y1 - 1) : (c.y0 - ccy);
    int32_t dy1 = negY ? (ccy - c.y0) : ((c.y1 - 1) - ccy);
    if (dy0 < 0) dy0 = 0;
    if (dy1 > ry) dy1 = ry;
    for (int32_t dy = dy0; dy <= dy1; ++dy) {
        const uint64_t rem = ry2 - static_cast<uint64_t>(dy) * dy;
        const int32_t xh = static_cast<int32_t>(
            (static_cast<uint64_t>(rx) * ISqrt64(rem)) / static_cast<uint64_t>(ry));
        const int32_t x = negX ? ccx - xh : ccx + xh;
        const int32_t y = negY ? ccy - dy : ccy + dy;
        PutPixel(t, c, x, y, color);
    }
    // Column sweep.
    int32_t dx0 = negX ? ccx - (c.x1 - 1) : (c.x0 - ccx);
    int32_t dx1 = negX ? (ccx - c.x0) : ((c.x1 - 1) - ccx);
    if (dx0 < 0) dx0 = 0;
    if (dx1 > rx) dx1 = rx;
    for (int32_t dx = dx0; dx <= dx1; ++dx) {
        const uint64_t rem = rx2 - static_cast<uint64_t>(dx) * dx;
        const int32_t yh = static_cast<int32_t>(
            (static_cast<uint64_t>(ry) * ISqrt64(rem)) / static_cast<uint64_t>(rx));
        const int32_t x = negX ? ccx - dx : ccx + dx;
        const int32_t y = negY ? ccy - yh : ccy + yh;
        PutPixel(t, c, x, y, color);
    }
}

/// Filled rounded-rect corner bands under non-uniform viewport scale.
/// For rows in the top band [y0d, y0d+ry) the corner cut is measured from a
/// quarter ellipse centered on (x0d+rx, y0d+ry) / (x1d-1-rx, y0d+ry);
/// the bottom band mirrors it. Bounded by the clip box.
void RoundedCornerBands(const Target& t, const ClipBox& c,
                        int32_t x0d, int32_t y0d, int32_t x1d, int32_t y1d,
                        int32_t rx, int32_t ry, uint16_t color,
                        ServiceFn service) {
    const uint64_t ry2 = static_cast<uint64_t>(ry) * ry;
    int32_t rows = 0;
    for (int32_t band = 0; band < 2; ++band) {
        // band 0: top, center row y0d+ry; band 1: bottom, center row y1d-1-ry
        const int32_t cRow = (band == 0) ? (y0d + ry) : (y1d - 1 - ry);
        int32_t yStart = (band == 0) ? y0d : (y1d - ry);
        int32_t yEnd   = (band == 0) ? (y0d + ry - 1) : (y1d - 1);
        if (yStart < c.y0) yStart = c.y0;
        if (yEnd >= c.y1) yEnd = c.y1 - 1;
        for (int32_t y = yStart; y <= yEnd; ++y) {
            const int64_t dyC = (band == 0) ? (cRow - y) : (y - cRow);
            const uint64_t rem = ry2 - static_cast<uint64_t>(dyC * dyC);
            const int32_t xh = static_cast<int32_t>(
                (static_cast<uint64_t>(rx) * ISqrt64(rem)) /
                static_cast<uint64_t>(ry));
            const int32_t cut = rx - xh;
            HLine(t, c, x0d + cut, x1d - 1 - cut, y, color);
            if (service && ((++rows) % Rasterizer2D::kServiceInterval) == 0) {
                service();
            }
        }
    }
}

}  // anonymous namespace

// ─── Implementations ────────────────────────────────────────────────────────

void Rasterizer2D::Clear(const Target& t, uint16_t color, ServiceFn service) {
    ClipBox c;
    if (!ComputeClip(t, c)) return;
    for (int32_t y = c.y0; y < c.y1; ++y) {
        FillSpanRaw(t, c.x0, c.x1 - 1, y, color);
        if (service && (((y - c.y0) + 1) % kServiceInterval) == 0) {
            service();
        }
    }
}

void Rasterizer2D::DrawRect(const Target& t, int16_t x, int16_t y,
                            uint16_t w, uint16_t h,
                            uint16_t color, bool filled, ServiceFn service) {
    if (w == 0 || h == 0) return;
    ClipBox c;
    if (!ComputeClip(t, c) || !ViewportOk(t)) return;

    // Half-open device rectangle from transformed corners (int32-widened).
    const int32_t x0d = TxX(t, x);
    const int32_t y0d = TxY(t, y);
    const int32_t x1d = TxX(t, static_cast<int32_t>(x) + w);
    const int32_t y1d = TxY(t, static_cast<int32_t>(y) + h);
    if (x1d <= x0d || y1d <= y0d) return;

    if (filled) {
        int32_t fy0 = (y0d > c.y0) ? y0d : c.y0;
        int32_t fy1 = (y1d < c.y1) ? y1d : c.y1;
        int32_t fx0 = (x0d > c.x0) ? x0d : c.x0;
        int32_t fx1 = (x1d < c.x1) ? x1d : c.x1;
        for (int32_t row = fy0; row < fy1; ++row) {
            FillSpanRaw(t, fx0, fx1 - 1, row, color);
            if (service && (((row - fy0) + 1) % kServiceInterval) == 0) {
                service();
            }
        }
    } else {
        HLine(t, c, x0d, x1d - 1, y0d, color);      // top
        HLine(t, c, x0d, x1d - 1, y1d - 1, color);  // bottom
        VLine(t, c, x0d, y0d, y1d - 1, color);      // left
        VLine(t, c, x1d - 1, y0d, y1d - 1, color);  // right
    }
}

void Rasterizer2D::DrawLine(const Target& t, int16_t x0, int16_t y0,
                            int16_t x1, int16_t y1, uint16_t color) {
    ClipBox c;
    if (!ComputeClip(t, c) || !ViewportOk(t)) return;

    int32_t ax = TxX(t, x0), ay = TxY(t, y0);
    int32_t bx = TxX(t, x1), by = TxY(t, y1);

    // Cohen–Sutherland clip against the inclusive clip box: caps traversal to
    // the visible span and keeps Bresenham state in int32 range.
    const int32_t cx1 = c.x1 - 1, cy1 = c.y1 - 1;
    auto OutCode = [&](int32_t x, int32_t y) -> int {
        int code = 0;
        if (x < c.x0) code |= 1;
        if (x > cx1)  code |= 2;
        if (y < c.y0) code |= 4;
        if (y > cy1)  code |= 8;
        return code;
    };
    int codeA = OutCode(ax, ay), codeB = OutCode(bx, by);
    for (;;) {
        if ((codeA | codeB) == 0) break;          // trivially inside
        if ((codeA & codeB) != 0) return;         // trivially outside
        int out = codeA ? codeA : codeB;
        int32_t nx, ny;
        if (out & 8) {          // below
            nx = ax + static_cast<int32_t>(
                (static_cast<int64_t>(bx - ax) * (cy1 - ay)) / (by - ay));
            ny = cy1;
        } else if (out & 4) {   // above
            nx = ax + static_cast<int32_t>(
                (static_cast<int64_t>(bx - ax) * (c.y0 - ay)) / (by - ay));
            ny = c.y0;
        } else if (out & 2) {   // right
            ny = ay + static_cast<int32_t>(
                (static_cast<int64_t>(by - ay) * (cx1 - ax)) / (bx - ax));
            nx = cx1;
        } else {                // left
            ny = ay + static_cast<int32_t>(
                (static_cast<int64_t>(by - ay) * (c.x0 - ax)) / (bx - ax));
            nx = c.x0;
        }
        if (out == codeA) { ax = nx; ay = ny; codeA = OutCode(ax, ay); }
        else              { bx = nx; by = ny; codeB = OutCode(bx, by); }
    }

    // Bresenham on the clipped segment (int32 throughout).
    const int32_t dx = std::abs(bx - ax);
    const int32_t dy = -std::abs(by - ay);
    const int32_t sx = (ax < bx) ? 1 : -1;
    const int32_t sy = (ay < by) ? 1 : -1;
    int32_t err = dx + dy;
    for (;;) {
        PutPixel(t, c, ax, ay, color);
        if (ax == bx && ay == by) break;
        const int32_t e2 = 2 * err;
        if (e2 >= dy) { err += dy; ax += sx; }
        if (e2 <= dx) { err += dx; ay += sy; }
    }
}

void Rasterizer2D::DrawCircle(const Target& t, int16_t cx, int16_t cy,
                              uint16_t radius, uint16_t color, bool filled,
                              ServiceFn service) {
    ClipBox c;
    if (!ComputeClip(t, c) || !ViewportOk(t)) return;

    const int32_t dcx = TxX(t, cx);
    const int32_t dcy = TxY(t, cy);
    const int32_t rx = TxLen(radius, t.viewScaleXQ8);
    const int32_t ry = TxLen(radius, t.viewScaleYQ8);

    if (rx == 0 || ry == 0) {
        // Degenerate: point / horizontal / vertical line segment.
        if (rx == 0 && ry == 0) PutPixel(t, c, dcx, dcy, color);
        else if (rx == 0) VLine(t, c, dcx, dcy - ry, dcy + ry, color);
        else              HLine(t, c, dcx - rx, dcx + rx, dcy, color);
        return;
    }

    // Bounding-box reject: nothing visible, no radius-scaled looping.
    if (dcx + rx < c.x0 || dcx - rx >= c.x1 ||
        dcy + ry < c.y0 || dcy - ry >= c.y1) {
        return;
    }

    if (rx != ry) {
        // Non-uniform viewport: scanline ellipse, bounded by the clip box.
        if (filled) EllipseFillRows(t, c, dcx, dcy, rx, ry, color, service);
        else        EllipseOutline(t, c, dcx, dcy, rx, ry, color);
        return;
    }

    const int32_t r = rx;

    if (filled) {
        // Coverage shortcut: if the farthest clip corner is inside the
        // circle, the visible fill is exactly the clip box.
        int64_t maxD2 = 0;
        const int32_t xs[2] = {c.x0, c.x1 - 1};
        const int32_t ys[2] = {c.y0, c.y1 - 1};
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                const int64_t dx = xs[i] - dcx, dy = ys[j] - dcy;
                const int64_t d2 = dx * dx + dy * dy;
                if (d2 > maxD2) maxD2 = d2;
            }
        }
        if (maxD2 <= static_cast<int64_t>(r) * r) {
            for (int32_t y = c.y0; y < c.y1; ++y) {
                FillSpanRaw(t, c.x0, c.x1 - 1, y, color);
                if (service && (((y - c.y0) + 1) % kServiceInterval) == 0) {
                    service();
                }
            }
            return;
        }
    }

    // Chebyshev distance from the center to the clip box: every plotted point
    // of the midpoint walk has Chebyshev distance x from the center, so once
    // x drops below this no remaining point can be visible.
    const int32_t minUseful = std::max(
        DistToInterval(dcx, c.x0, c.x1 - 1),
        DistToInterval(dcy, c.y0, c.y1 - 1));

    // Midpoint circle algorithm (pixel-identical to the legacy walk).
    int32_t x = r;
    int32_t y = 0;
    int32_t d = 1 - x;
    uint32_t iter = 0;
    while (x >= y) {
        if (x < minUseful) break;
        if (filled) CircleHLines(t, c, dcx, dcy, x, y, color);
        else        CirclePoints(t, c, dcx, dcy, x, y, color);
        ++y;
        if (d <= 0) {
            d += 2 * y + 1;
        } else {
            --x;
            d += 2 * (y - x) + 1;
        }
        if (service && ((++iter) % kServiceInterval) == 0) {
            service();
        }
    }
}

void Rasterizer2D::DrawRoundedRect(const Target& t, int16_t x, int16_t y,
                                   uint16_t w, uint16_t h, uint16_t r,
                                   uint16_t color, bool filled,
                                   ServiceFn service) {
    if (w == 0 || h == 0) return;
    ClipBox c;
    if (!ComputeClip(t, c) || !ViewportOk(t)) return;

    const int32_t x0d = TxX(t, x);
    const int32_t y0d = TxY(t, y);
    const int32_t x1d = TxX(t, static_cast<int32_t>(x) + w);
    const int32_t y1d = TxY(t, static_cast<int32_t>(y) + h);
    const int32_t dw = x1d - x0d;
    const int32_t dh = y1d - y0d;
    if (dw <= 0 || dh <= 0) return;

    if (r == 0) {
        // Same shape as a plain rect; reuse the device-space path inline.
        if (filled) {
            FillRectDev(t, c, x0d, y0d, x1d, y1d, color);
        } else {
            HLine(t, c, x0d, x1d - 1, y0d, color);
            HLine(t, c, x0d, x1d - 1, y1d - 1, color);
            VLine(t, c, x0d, y0d, y1d - 1, color);
            VLine(t, c, x1d - 1, y0d, y1d - 1, color);
        }
        return;
    }

    // Clamp radius to half the smallest device dimension (per axis).
    int32_t rx = TxLen(r, t.viewScaleXQ8);
    int32_t ry = TxLen(r, t.viewScaleYQ8);
    const int32_t maxRx = dw / 2, maxRy = dh / 2;
    if (rx > maxRx) rx = maxRx;
    if (ry > maxRy) ry = maxRy;
    if (rx <= 0 || ry <= 0) {
        // Scaled corner radius vanished: plain rect.
        if (filled) {
            FillRectDev(t, c, x0d, y0d, x1d, y1d, color);
        } else {
            HLine(t, c, x0d, x1d - 1, y0d, color);
            HLine(t, c, x0d, x1d - 1, y1d - 1, color);
            VLine(t, c, x0d, y0d, y1d - 1, color);
            VLine(t, c, x1d - 1, y0d, y1d - 1, color);
        }
        return;
    }

    if (rx != ry) {
        // Non-uniform viewport: scanline quarter-ellipse corners.
        if (filled) {
            // Middle band (full width), then the two corner bands.
            int32_t my0 = y0d + ry, my1 = y1d - ry;  // half-open
            FillRectDev(t, c, x0d, my0, x1d, my1, color);
            RoundedCornerBands(t, c, x0d, y0d, x1d, y1d, rx, ry, color, service);
        } else {
            HLine(t, c, x0d + rx, x1d - 1 - rx, y0d, color);
            HLine(t, c, x0d + rx, x1d - 1 - rx, y1d - 1, color);
            VLine(t, c, x0d, y0d + ry, y1d - 1 - ry, color);
            VLine(t, c, x1d - 1, y0d + ry, y1d - 1 - ry, color);
            EllipseCornerOutline(t, c, x0d + rx, y0d + ry, rx, ry, true, true, color);
            EllipseCornerOutline(t, c, x1d - 1 - rx, y0d + ry, rx, ry, false, true, color);
            EllipseCornerOutline(t, c, x0d + rx, y1d - 1 - ry, rx, ry, true, false, color);
            EllipseCornerOutline(t, c, x1d - 1 - rx, y1d - 1 - ry, rx, ry, false, false, color);
        }
        return;
    }

    const int32_t ri = rx;

    if (filled) {
        // Fill center rectangle (excluding corners)
        int32_t row0 = y0d + ri, row1 = y1d - 1 - ri;
        for (int32_t row = row0; row <= row1; ++row) {
            HLine(t, c, x0d, x1d - 1, row, color);
            if (service && (((row - row0) + 1) % kServiceInterval) == 0) {
                service();
            }
        }

        // Fill top and bottom strips using midpoint quarter-circle fills
        const int32_t cx_left  = x0d + ri;
        const int32_t cx_right = x1d - 1 - ri;
        const int32_t cy_top   = y0d + ri;
        const int32_t cy_bot   = y1d - 1 - ri;

        int32_t px = ri, py = 0;
        int32_t d = 1 - px;
        while (px >= py) {
            // Top band
            HLine(t, c, cx_left - px, cx_right + px, cy_top - py, color);
            HLine(t, c, cx_left - py, cx_right + py, cy_top - px, color);
            // Bottom band
            HLine(t, c, cx_left - px, cx_right + px, cy_bot + py, color);
            HLine(t, c, cx_left - py, cx_right + py, cy_bot + px, color);

            ++py;
            if (d <= 0) {
                d += 2 * py + 1;
            } else {
                --px;
                d += 2 * (py - px) + 1;
            }
        }
    } else {
        // Outline: straight edges + corner arcs
        HLine(t, c, x0d + ri, x1d - 1 - ri, y0d, color);
        HLine(t, c, x0d + ri, x1d - 1 - ri, y1d - 1, color);
        VLine(t, c, x0d, y0d + ri, y1d - 1 - ri, color);
        VLine(t, c, x1d - 1, y0d + ri, y1d - 1 - ri, color);

        // Quarter-circle corners (midpoint algorithm, one octant mirrored)
        const int32_t cx_tl = x0d + ri, cy_tl = y0d + ri;
        const int32_t cx_tr = x1d - 1 - ri, cy_tr = y0d + ri;
        const int32_t cx_bl = x0d + ri, cy_bl = y1d - 1 - ri;
        const int32_t cx_br = x1d - 1 - ri, cy_br = y1d - 1 - ri;

        int32_t px = ri, py = 0;
        int32_t d = 1 - px;
        while (px >= py) {
            PutPixel(t, c, cx_tl - px, cy_tl - py, color);
            PutPixel(t, c, cx_tl - py, cy_tl - px, color);
            PutPixel(t, c, cx_tr + px, cy_tr - py, color);
            PutPixel(t, c, cx_tr + py, cy_tr - px, color);
            PutPixel(t, c, cx_bl - px, cy_bl + py, color);
            PutPixel(t, c, cx_bl - py, cy_bl + px, color);
            PutPixel(t, c, cx_br + px, cy_br + py, color);
            PutPixel(t, c, cx_br + py, cy_br + px, color);

            ++py;
            if (d <= 0) {
                d += 2 * py + 1;
            } else {
                --px;
                d += 2 * (py - px) + 1;
            }
        }
    }
}

void Rasterizer2D::DrawTriangle(const Target& t,
                                int16_t x0, int16_t y0,
                                int16_t x1, int16_t y1,
                                int16_t x2, int16_t y2,
                                uint16_t color, ServiceFn service) {
    ClipBox c;
    if (!ComputeClip(t, c) || !ViewportOk(t)) return;

    int32_t ax = TxX(t, x0), ay = TxY(t, y0);
    int32_t bx = TxX(t, x1), by = TxY(t, y1);
    int32_t cx = TxX(t, x2), cy = TxY(t, y2);

    // Sort vertices by Y coordinate (ay <= by <= cy)
    if (ay > by) { std::swap(ay, by); std::swap(ax, bx); }
    if (by > cy) { std::swap(by, cy); std::swap(bx, cx); }
    if (ay > by) { std::swap(ay, by); std::swap(ax, bx); }

    if (ay == cy) {
        // Degenerate: all on one scanline
        const int32_t lo = std::min({ax, bx, cx});
        const int32_t hi = std::max({ax, bx, cx});
        HLine(t, c, lo, hi, ay, color);
        return;
    }

    // Scanline fill with int64 edge interpolation; rows clamped to the clip
    // box so off-screen spans never iterate.
    auto FillSpan = [&](int32_t yStart, int32_t yEnd,
                        int32_t xa, int32_t ya, int32_t xb, int32_t yb,
                        int32_t xc, int32_t yc, int32_t xd, int32_t yd,
                        uint32_t& rowCount) {
        if (yStart == yEnd) return;
        int32_t r0 = (yStart > c.y0) ? yStart : c.y0;
        int32_t r1 = (yEnd < c.y1) ? yEnd : c.y1;  // half-open
        for (int32_t row = r0; row < r1; ++row) {
            const int64_t t1 = row - ya;
            const int64_t t2 = row - yc;
            const int32_t xLeft = static_cast<int32_t>(
                xa + (static_cast<int64_t>(xb - xa) * t1) / (yb - ya));
            const int32_t xRight = static_cast<int32_t>(
                xc + (static_cast<int64_t>(xd - xc) * t2) / (yd - yc));
            HLine(t, c, xLeft, xRight, row, color);
            if (service && ((++rowCount) % kServiceInterval) == 0) {
                service();
            }
        }
    };

    uint32_t rowCount = 0;
    // Upper half: ay → by (edges a→b and a→c)
    if (ay != by) {
        FillSpan(ay, by, ax, ay, bx, by, ax, ay, cx, cy, rowCount);
    }
    // Lower half: by → cy (edges b→c and a→c)
    if (by != cy) {
        FillSpan(by, cy, bx, by, cx, cy, ax, ay, cx, cy, rowCount);
    }
}

void Rasterizer2D::DrawArc(const Target& t, int16_t cx, int16_t cy,
                           uint16_t radius, int16_t startDeg, int16_t endDeg,
                           uint16_t color) {
    if (radius == 0) return;
    ClipBox c;
    if (!ComputeClip(t, c) || !ViewportOk(t)) return;

    const int32_t dcx = TxX(t, cx);
    const int32_t dcy = TxY(t, cy);
    const int32_t rx = TxLen(radius, t.viewScaleXQ8);
    const int32_t ry = TxLen(radius, t.viewScaleYQ8);
    if (rx == 0 && ry == 0) return;

    // Normalize angles to the 0–359 range
    int32_t startA = startDeg, endA = endDeg;
    startA %= 360; if (startA < 0) startA += 360;
    endA   %= 360; if (endA   < 0) endA   += 360;

    // Precomputed sine table (0–90 degrees, Q15 fixed-point, 1-degree steps)
    static const int16_t sinTable[91] = {
            0,   572,  1144,  1715,  2286,  2856,  3425,  3993,  4560,  5126,
         5690,  6252,  6813,  7371,  7927,  8481,  9032,  9580, 10126, 10668,
        11207, 11743, 12275, 12803, 13328, 13848, 14365, 14876, 15384, 15886,
        16384, 16877, 17364, 17847, 18324, 18795, 19261, 19720, 20174, 20622,
        21063, 21498, 21926, 22348, 22763, 23170, 23571, 23965, 24351, 24730,
        25102, 25466, 25822, 26170, 26510, 26842, 27166, 27482, 27789, 28088,
        28378, 28660, 28932, 29197, 29452, 29698, 29935, 30163, 30382, 30592,
        30792, 30983, 31164, 31336, 31499, 31651, 31795, 31928, 32052, 32166,
        32270, 32365, 32449, 32524, 32588, 32643, 32688, 32723, 32748, 32763,
        32767   // sin(90°) clamped to Q15 max (32768 not representable in int16_t)
    };

    auto SinQ15 = [&](int32_t deg) -> int32_t {
        deg = ((deg % 360) + 360) % 360;
        if (deg <= 90) return sinTable[deg];
        if (deg <= 180) return sinTable[180 - deg];
        if (deg <= 270) return -sinTable[deg - 180];
        return -sinTable[360 - deg];
    };
    auto CosQ15 = [&](int32_t deg) -> int32_t {
        return SinQ15(deg + 90);
    };

    // Step through each degree in the arc and plot pixels (≤361 iterations).
    // Same rounding convention as the legacy path: (v*trig + 16384) / 32768
    // with C truncating division, widened to int64 for scaled radii.
    auto PlotArcDegree = [&](int32_t deg) {
        const int32_t px = dcx + static_cast<int32_t>(
            (static_cast<int64_t>(rx) * CosQ15(deg) + 16384) / 32768);
        const int32_t py = dcy - static_cast<int32_t>(
            (static_cast<int64_t>(ry) * SinQ15(deg) + 16384) / 32768);
        PutPixel(t, c, px, py, color);
    };

    if (startA <= endA) {
        for (int32_t d = startA; d <= endA; ++d) PlotArcDegree(d);
    } else {
        // Arc wraps around 0°
        for (int32_t d = startA; d < 360; ++d) PlotArcDegree(d);
        for (int32_t d = 0; d <= endA; ++d) PlotArcDegree(d);
    }
}

void Rasterizer2D::DrawSprite(const Target& t, int16_t dstX, int16_t dstY,
                              const SpriteSource& src,
                              bool flipH, bool flipV, ServiceFn service) {
    if (!src.pixels || src.width == 0 || src.height == 0) return;
    if (src.format != SRC_FORMAT_RGB565 && src.format != SRC_FORMAT_RGB888) {
        return;
    }
    ClipBox c;
    if (!ComputeClip(t, c) || !ViewportOk(t)) return;

    const int32_t x0d = TxX(t, dstX);
    const int32_t y0d = TxY(t, dstY);
    const int32_t x1d = TxX(t, static_cast<int32_t>(dstX) + src.width);
    const int32_t y1d = TxY(t, static_cast<int32_t>(dstY) + src.height);
    BlitSprite(t, c, src, x0d, y0d, x1d, y1d, flipH, flipV, service);
}

void Rasterizer2D::DrawSpriteBatch(const Target& t, const SpriteSource& src,
                                   const PglSpritePosition* positions,
                                   uint16_t count,
                                   bool flipH, bool flipV,
                                   ServiceFn service) {
    if (!positions || count == 0) return;
    if (!src.pixels || src.width == 0 || src.height == 0) return;
    if (src.format != SRC_FORMAT_RGB565 && src.format != SRC_FORMAT_RGB888) {
        return;
    }
    ClipBox c;
    if (!ComputeClip(t, c) || !ViewportOk(t)) return;

    for (uint16_t i = 0; i < count; ++i) {
        // Same corner-transform rule as the single-sprite path.
        const int32_t x0d = TxX(t, positions[i].x);
        const int32_t y0d = TxY(t, positions[i].y);
        const int32_t x1d =
            TxX(t, static_cast<int32_t>(positions[i].x) + src.width);
        const int32_t y1d =
            TxY(t, static_cast<int32_t>(positions[i].y) + src.height);
        BlitSprite(t, c, src, x0d, y0d, x1d, y1d, flipH, flipV, nullptr);
        if (service && (((i) + 1) % kServiceInterval) == 0) {
            service();
        }
    }
}

int32_t Rasterizer2D::DrawText(const Target& t, int16_t x, int16_t y,
                               const GlyphAtlas& atlas,
                               const char* text, uint16_t length,
                               uint16_t color, ServiceFn service) {
    if (!t.pixels || !text || !atlas.pixels ||
        atlas.glyphW == 0 || atlas.glyphH == 0 || atlas.columns == 0 ||
        atlas.width == 0 || atlas.height == 0) {
        return kTextErrInvalid;
    }

    // Validate the whole string BEFORE any pixel is written: a malformed or
    // out-of-atlas character rejects the entire draw (no partial output).
    for (uint16_t i = 0; i < length; ++i) {
        const uint8_t ch = static_cast<uint8_t>(text[i]);
        if (ch == '\n') continue;
        if (ch < atlas.firstChar) return kTextErrGlyph;
        const uint32_t cell = ch - atlas.firstChar;
        const uint32_t col = cell % atlas.columns;
        const uint32_t row = cell / atlas.columns;
        if ((col + 1) * atlas.glyphW > static_cast<uint32_t>(atlas.width) ||
            (row + 1) * atlas.glyphH > static_cast<uint32_t>(atlas.height)) {
            return kTextErrGlyph;
        }
    }

    ClipBox c;
    if (!ComputeClip(t, c) || !ViewportOk(t)) return length;  // valid, invisible

    int32_t penX = x, penY = y;
    for (uint16_t i = 0; i < length; ++i) {
        const uint8_t ch = static_cast<uint8_t>(text[i]);
        if (ch == '\n') {
            penX = x;
            penY += atlas.glyphH;
            continue;
        }
        const uint32_t cell = ch - atlas.firstChar;
        const uint32_t col = cell % atlas.columns;
        const uint32_t row = cell / atlas.columns;
        const uint32_t ax = col * atlas.glyphW;
        const uint32_t ay = row * atlas.glyphH;

        for (uint32_t my = 0; my < atlas.glyphH; ++my) {
            const uint16_t* maskRow =
                atlas.pixels + (ay + my) * atlas.width + ax;
            for (uint32_t mx = 0; mx < atlas.glyphW; ++mx) {
                if (maskRow[mx] != 0) {
                    // Viewport-scaled device rectangle for this mask texel.
                    FillRectDev(t, c,
                                TxX(t, penX + static_cast<int32_t>(mx)),
                                TxY(t, penY + static_cast<int32_t>(my)),
                                TxX(t, penX + static_cast<int32_t>(mx) + 1),
                                TxY(t, penY + static_cast<int32_t>(my) + 1),
                                color);
                }
            }
        }
        penX += atlas.glyphW;

        if (service && (((i) + 1) % kServiceInterval) == 0) {
            service();
        }
    }
    return length;
}

void Rasterizer2D::DrawGradientRect(const Target& t, int16_t x, int16_t y,
                                    uint16_t w, uint16_t h,
                                    uint16_t color0, uint16_t color1,
                                    uint8_t direction, ServiceFn service) {
    if (w == 0 || h == 0) return;
    ClipBox c;
    if (!ComputeClip(t, c) || !ViewportOk(t)) return;

    const int32_t x0d = TxX(t, x);
    const int32_t y0d = TxY(t, y);
    const int32_t x1d = TxX(t, static_cast<int32_t>(x) + w);
    const int32_t y1d = TxY(t, static_cast<int32_t>(y) + h);
    if (x1d <= x0d || y1d <= y0d) return;

    const int32_t r0 = (color0 >> 11) & 0x1F, g0 = (color0 >> 5) & 0x3F,
                  b0 =  color0        & 0x1F;
    const int32_t r1 = (color1 >> 11) & 0x1F, g1 = (color1 >> 5) & 0x3F,
                  b1 =  color1        & 0x1F;

    // c = c0 + (c1 - c0) * i / (n - 1), truncating; endpoints exact.
    auto Lerp = [&](int32_t i, int32_t n) -> uint16_t {
        if (n <= 1) return color0;
        const int32_t d = n - 1;
        const int32_t r = r0 + (r1 - r0) * i / d;
        const int32_t g = g0 + (g1 - g0) * i / d;
        const int32_t b = b0 + (b1 - b0) * i / d;
        return static_cast<uint16_t>((r << 11) | (g << 5) | b);
    };

    if (direction == 0) {
        // Horizontal: one constant-color column per device x.
        const int32_t n = x1d - x0d;
        int32_t px0 = (x0d > c.x0) ? x0d : c.x0;
        int32_t px1 = (x1d < c.x1) ? x1d : c.x1;
        for (int32_t dx = px0; dx < px1; ++dx) {
            VLine(t, c, dx, y0d, y1d - 1, Lerp(dx - x0d, n));
            if (service && (((dx - px0) + 1) % kServiceInterval) == 0) {
                service();
            }
        }
    } else {
        // Vertical: one constant-color row per device y.
        const int32_t n = y1d - y0d;
        int32_t py0 = (y0d > c.y0) ? y0d : c.y0;
        int32_t py1 = (y1d < c.y1) ? y1d : c.y1;
        for (int32_t dy = py0; dy < py1; ++dy) {
            HLine(t, c, x0d, x1d - 1, dy, Lerp(dy - y0d, n));
            if (service && (((dy - py0) + 1) % kServiceInterval) == 0) {
                service();
            }
        }
    }
}

// ─── Blending ───────────────────────────────────────────────────────────────

uint16_t Rasterizer2D::BlendRGB565(uint16_t src, uint16_t dst, uint8_t alpha) {
    const uint32_t srcR = (src >> 11) & 0x1F;
    const uint32_t srcG = (src >> 5)  & 0x3F;
    const uint32_t srcB =  src        & 0x1F;
    const uint32_t dstR = (dst >> 11) & 0x1F;
    const uint32_t dstG = (dst >> 5)  & 0x3F;
    const uint32_t dstB =  dst        & 0x1F;

    const uint32_t inv = 255 - alpha;
    const uint32_t r = (srcR * alpha + dstR * inv) / 255;
    const uint32_t g = (srcG * alpha + dstG * inv) / 255;
    const uint32_t b = (srcB * alpha + dstB * inv) / 255;

    return static_cast<uint16_t>((r << 11) | (g << 5) | b);
}

uint16_t Rasterizer2D::BlendAddRGB565(uint16_t src, uint16_t dst,
                                      uint8_t alpha) {
    const uint32_t srcR = (src >> 11) & 0x1F;
    const uint32_t srcG = (src >> 5)  & 0x3F;
    const uint32_t srcB =  src        & 0x1F;
    const uint32_t dstR = (dst >> 11) & 0x1F;
    const uint32_t dstG = (dst >> 5)  & 0x3F;
    const uint32_t dstB =  dst        & 0x1F;

    uint32_t r = dstR + (srcR * alpha) / 255;  if (r > 31) r = 31;
    uint32_t g = dstG + (srcG * alpha) / 255;  if (g > 63) g = 63;
    uint32_t b = dstB + (srcB * alpha) / 255;  if (b > 31) b = 31;

    return static_cast<uint16_t>((r << 11) | (g << 5) | b);
}

uint16_t Rasterizer2D::BlendMultiplyRGB565(uint16_t src, uint16_t dst,
                                           uint8_t alpha) {
    const int32_t srcR = (src >> 11) & 0x1F;
    const int32_t srcG = (src >> 5)  & 0x3F;
    const int32_t srcB =  src        & 0x1F;
    const int32_t dstR = (dst >> 11) & 0x1F;
    const int32_t dstG = (dst >> 5)  & 0x3F;
    const int32_t dstB =  dst        & 0x1F;

    const int32_t mulR = (srcR * dstR) / 31;
    const int32_t mulG = (srcG * dstG) / 63;
    const int32_t mulB = (srcB * dstB) / 31;

    const int32_t r = dstR + ((mulR - dstR) * alpha) / 255;
    const int32_t g = dstG + ((mulG - dstG) * alpha) / 255;
    const int32_t b = dstB + ((mulB - dstB) * alpha) / 255;

    return static_cast<uint16_t>((r << 11) | (g << 5) | b);
}

uint16_t Rasterizer2D::CompositeLayerPixel(uint16_t src, uint16_t dst,
                                           uint8_t blendMode,
                                           uint8_t opacity) {
    switch (blendMode) {
        case 1:  return BlendAddRGB565(src, dst, opacity);       // ADDITIVE
        case 2:  return BlendMultiplyRGB565(src, dst, opacity);  // MULTIPLY
        case 0:
        default: return BlendRGB565(src, dst, opacity);          // ALPHA
    }
}
