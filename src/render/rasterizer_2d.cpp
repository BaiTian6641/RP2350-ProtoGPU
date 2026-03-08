/**
 * @file rasterizer_2d.cpp
 * @brief GPU-side 2D rasterizer implementation.
 *
 * All primitives clip to the target framebuffer bounds. No heap allocation.
 * Optimized for RP2350 Cortex-M33 @ 150 MHz with a 128×64 or similar small FB.
 *
 * M12 — ProtoGL v0.7.2
 */

#include "rasterizer_2d.h"

#include <cstring>
#include <cstdlib>   // abs
#include <algorithm>  // std::swap, std::min, std::max

// ─── Helpers ────────────────────────────────────────────────────────────────

namespace {

/// Safely write one pixel with bounds checking.
inline void PutPixel(const Rasterizer2D::Target& t,
                     int16_t x, int16_t y, uint16_t color) {
    if (x >= 0 && x < static_cast<int16_t>(t.width) &&
        y >= 0 && y < static_cast<int16_t>(t.height)) {
        t.pixels[y * t.width + x] = color;
    }
}

/// Draw a horizontal span (clipped).
inline void HLine(const Rasterizer2D::Target& t,
                  int16_t x0, int16_t x1, int16_t y, uint16_t color) {
    if (y < 0 || y >= static_cast<int16_t>(t.height)) return;
    if (x0 > x1) std::swap(x0, x1);
    if (x1 < 0 || x0 >= static_cast<int16_t>(t.width)) return;
    if (x0 < 0) x0 = 0;
    if (x1 >= static_cast<int16_t>(t.width)) x1 = static_cast<int16_t>(t.width) - 1;

    uint16_t* row = t.pixels + y * t.width;
    for (int16_t x = x0; x <= x1; ++x) {
        row[x] = color;
    }
}

/// Draw a vertical span (clipped).
inline void VLine(const Rasterizer2D::Target& t,
                  int16_t x, int16_t y0, int16_t y1, uint16_t color) {
    if (x < 0 || x >= static_cast<int16_t>(t.width)) return;
    if (y0 > y1) std::swap(y0, y1);
    if (y1 < 0 || y0 >= static_cast<int16_t>(t.height)) return;
    if (y0 < 0) y0 = 0;
    if (y1 >= static_cast<int16_t>(t.height)) y1 = static_cast<int16_t>(t.height) - 1;

    for (int16_t y = y0; y <= y1; ++y) {
        t.pixels[y * t.width + x] = color;
    }
}

/// Plot 8 symmetric circle points (used by midpoint circle).
inline void CirclePoints(const Rasterizer2D::Target& t,
                         int16_t cx, int16_t cy, int16_t dx, int16_t dy,
                         uint16_t color) {
    PutPixel(t, cx + dx, cy + dy, color);
    PutPixel(t, cx - dx, cy + dy, color);
    PutPixel(t, cx + dx, cy - dy, color);
    PutPixel(t, cx - dx, cy - dy, color);
    PutPixel(t, cx + dy, cy + dx, color);
    PutPixel(t, cx - dy, cy + dx, color);
    PutPixel(t, cx + dy, cy - dx, color);
    PutPixel(t, cx - dy, cy - dx, color);
}

/// Fill 4 symmetric horizontal spans (filled circle).
inline void CircleHLines(const Rasterizer2D::Target& t,
                         int16_t cx, int16_t cy, int16_t dx, int16_t dy,
                         uint16_t color) {
    HLine(t, cx - dx, cx + dx, cy + dy, color);
    HLine(t, cx - dx, cx + dx, cy - dy, color);
    HLine(t, cx - dy, cx + dy, cy + dx, color);
    HLine(t, cx - dy, cx + dy, cy - dx, color);
}

}  // anonymous namespace

// ─── Implementations ────────────────────────────────────────────────────────

void Rasterizer2D::Clear(const Target& t, uint16_t color) {
    if (!t.pixels) return;
    const uint32_t count = static_cast<uint32_t>(t.width) * t.height;
    // Fast fill: use 32-bit writes for aligned case
    const uint32_t color32 = (static_cast<uint32_t>(color) << 16) | color;
    uint32_t* p32 = reinterpret_cast<uint32_t*>(t.pixels);
    const uint32_t count32 = count / 2;
    for (uint32_t i = 0; i < count32; ++i) {
        p32[i] = color32;
    }
    if (count & 1) {
        t.pixels[count - 1] = color;
    }
}

void Rasterizer2D::DrawRect(const Target& t, int16_t x, int16_t y,
                             uint16_t w, uint16_t h,
                             uint16_t color, bool filled) {
    if (!t.pixels || w == 0 || h == 0) return;

    if (filled) {
        // Clip rectangle to target bounds
        int16_t x0 = x, y0 = y;
        int16_t x1 = x + static_cast<int16_t>(w) - 1;
        int16_t y1 = y + static_cast<int16_t>(h) - 1;
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x1 >= static_cast<int16_t>(t.width))  x1 = static_cast<int16_t>(t.width) - 1;
        if (y1 >= static_cast<int16_t>(t.height)) y1 = static_cast<int16_t>(t.height) - 1;
        if (x0 > x1 || y0 > y1) return;

        for (int16_t row = y0; row <= y1; ++row) {
            HLine(t, x0, x1, row, color);
        }
    } else {
        // Outline: 4 edges
        int16_t x1 = x + static_cast<int16_t>(w) - 1;
        int16_t y1 = y + static_cast<int16_t>(h) - 1;
        HLine(t, x, x1, y, color);   // top
        HLine(t, x, x1, y1, color);  // bottom
        VLine(t, x, y, y1, color);   // left
        VLine(t, x1, y, y1, color);  // right
    }
}

void Rasterizer2D::DrawLine(const Target& t, int16_t x0, int16_t y0,
                             int16_t x1, int16_t y1, uint16_t color) {
    if (!t.pixels) return;

    // Bresenham's line algorithm
    int16_t dx = std::abs(x1 - x0);
    int16_t dy = -std::abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx + dy;

    for (;;) {
        PutPixel(t, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void Rasterizer2D::DrawCircle(const Target& t, int16_t cx, int16_t cy,
                               uint16_t radius, uint16_t color, bool filled) {
    if (!t.pixels || radius == 0) {
        if (radius == 0) PutPixel(t, cx, cy, color);
        return;
    }

    // Midpoint circle algorithm
    int16_t x = static_cast<int16_t>(radius);
    int16_t y = 0;
    int16_t d = 1 - x;

    while (x >= y) {
        if (filled) {
            CircleHLines(t, cx, cy, x, y, color);
        } else {
            CirclePoints(t, cx, cy, x, y, color);
        }
        ++y;
        if (d <= 0) {
            d += 2 * y + 1;
        } else {
            --x;
            d += 2 * (y - x) + 1;
        }
    }
}

void Rasterizer2D::DrawRoundedRect(const Target& t, int16_t x, int16_t y,
                                    uint16_t w, uint16_t h, uint16_t r,
                                    uint16_t color, bool filled) {
    if (!t.pixels || w == 0 || h == 0) return;

    // Clamp radius to half the smallest dimension
    uint16_t maxR = std::min(w, h) / 2;
    if (r > maxR) r = maxR;
    if (r == 0) {
        DrawRect(t, x, y, w, h, color, filled);
        return;
    }

    const int16_t ri = static_cast<int16_t>(r);
    const int16_t x1 = x + static_cast<int16_t>(w) - 1;
    const int16_t y1 = y + static_cast<int16_t>(h) - 1;

    if (filled) {
        // Fill center rectangle (excluding corners)
        for (int16_t row = y + ri; row <= y1 - ri; ++row) {
            HLine(t, x, x1, row, color);
        }

        // Fill top and bottom strips using midpoint quarter-circle fills
        int16_t cx_left  = x + ri;
        int16_t cx_right = x1 - ri;
        int16_t cy_top   = y + ri;
        int16_t cy_bot   = y1 - ri;

        int16_t px = ri, py = 0;
        int16_t d = 1 - px;
        while (px >= py) {
            // Top band
            HLine(t, cx_left - px, cx_right + px, cy_top - py, color);
            HLine(t, cx_left - py, cx_right + py, cy_top - px, color);
            // Bottom band
            HLine(t, cx_left - px, cx_right + px, cy_bot + py, color);
            HLine(t, cx_left - py, cx_right + py, cy_bot + px, color);

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
        HLine(t, x + ri, x1 - ri, y,  color);  // top
        HLine(t, x + ri, x1 - ri, y1, color);  // bottom
        VLine(t, x,  y + ri, y1 - ri, color);   // left
        VLine(t, x1, y + ri, y1 - ri, color);   // right

        // Quarter-circle corners (midpoint algorithm, one octant mirrored)
        int16_t cx_tl = x + ri, cy_tl = y + ri;
        int16_t cx_tr = x1 - ri, cy_tr = y + ri;
        int16_t cx_bl = x + ri, cy_bl = y1 - ri;
        int16_t cx_br = x1 - ri, cy_br = y1 - ri;

        int16_t px = ri, py = 0;
        int16_t d = 1 - px;
        while (px >= py) {
            // Top-left corner
            PutPixel(t, cx_tl - px, cy_tl - py, color);
            PutPixel(t, cx_tl - py, cy_tl - px, color);
            // Top-right corner
            PutPixel(t, cx_tr + px, cy_tr - py, color);
            PutPixel(t, cx_tr + py, cy_tr - px, color);
            // Bottom-left corner
            PutPixel(t, cx_bl - px, cy_bl + py, color);
            PutPixel(t, cx_bl - py, cy_bl + px, color);
            // Bottom-right corner
            PutPixel(t, cx_br + px, cy_br + py, color);
            PutPixel(t, cx_br + py, cy_br + px, color);

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
                                 uint16_t color) {
    if (!t.pixels) return;

    // Sort vertices by Y coordinate (y0 <= y1 <= y2)
    if (y0 > y1) { std::swap(y0, y1); std::swap(x0, x1); }
    if (y1 > y2) { std::swap(y1, y2); std::swap(x1, x2); }
    if (y0 > y1) { std::swap(y0, y1); std::swap(x0, x1); }

    if (y0 == y2) {
        // Degenerate: all on same scanline
        int16_t lo = std::min({x0, x1, x2});
        int16_t hi = std::max({x0, x1, x2});
        HLine(t, lo, hi, y0, color);
        return;
    }

    // Scanline fill using fixed-point edge interpolation
    auto FillSpan = [&](int16_t yStart, int16_t yEnd,
                        int16_t xa, int16_t ya, int16_t xb, int16_t yb,
                        int16_t xc, int16_t yc, int16_t xd, int16_t yd) {
        if (yStart == yEnd) return;
        for (int16_t row = yStart; row < yEnd; ++row) {
            // Interpolate X along edge a→b and c→d
            int32_t t1 = row - ya;
            int32_t t2 = row - yc;
            int32_t xLeft  = xa + (xb - xa) * t1 / (yb - ya);
            int32_t xRight = xc + (xd - xc) * t2 / (yd - yc);
            HLine(t, static_cast<int16_t>(xLeft), static_cast<int16_t>(xRight), row, color);
        }
    };

    // Upper half: y0 → y1 (edges 0→1 and 0→2)
    if (y0 != y1) {
        FillSpan(y0, y1, x0, y0, x1, y1, x0, y0, x2, y2);
    }

    // Lower half: y1 → y2 (edges 1→2 and 0→2)
    if (y1 != y2) {
        FillSpan(y1, y2, x1, y1, x2, y2, x0, y0, x2, y2);
    }
}

void Rasterizer2D::DrawArc(const Target& t, int16_t cx, int16_t cy,
                            uint16_t radius, int16_t startDeg, int16_t endDeg,
                            uint16_t color) {
    if (!t.pixels || radius == 0) return;

    // Normalize angles to 0–360 range
    while (startDeg < 0) startDeg += 360;
    while (endDeg < 0) endDeg += 360;
    startDeg %= 360;
    endDeg %= 360;

    // Precomputed sine table (0–90 degrees, Q15 fixed-point, 1-degree steps)
    // sin(angle) * 32768
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
        32768
    };

    auto SinQ15 = [&](int16_t deg) -> int32_t {
        deg = ((deg % 360) + 360) % 360;
        if (deg <= 90) return sinTable[deg];
        if (deg <= 180) return sinTable[180 - deg];
        if (deg <= 270) return -sinTable[deg - 180];
        return -sinTable[360 - deg];
    };

    auto CosQ15 = [&](int16_t deg) -> int32_t {
        return SinQ15(deg + 90);
    };

    // Step through each degree in the arc and plot pixels
    auto PlotArcDegree = [&](int16_t deg) {
        int32_t px = cx + (static_cast<int32_t>(radius) * CosQ15(deg) + 16384) / 32768;
        int32_t py = cy - (static_cast<int32_t>(radius) * SinQ15(deg) + 16384) / 32768;
        PutPixel(t, static_cast<int16_t>(px), static_cast<int16_t>(py), color);
    };

    if (startDeg <= endDeg) {
        for (int16_t d = startDeg; d <= endDeg; ++d) PlotArcDegree(d);
    } else {
        // Arc wraps around 0°
        for (int16_t d = startDeg; d < 360; ++d) PlotArcDegree(d);
        for (int16_t d = 0; d <= endDeg; ++d) PlotArcDegree(d);
    }
}

void Rasterizer2D::DrawSprite(const Target& t,
                               int16_t dstX, int16_t dstY,
                               const uint16_t* srcPixels,
                               uint16_t srcW, uint16_t srcH,
                               bool flipH, bool flipV) {
    if (!t.pixels || !srcPixels || srcW == 0 || srcH == 0) return;

    // Compute clipped blit region
    int16_t sx0 = 0, sy0 = 0;
    int16_t dx0 = dstX, dy0 = dstY;
    int16_t copyW = static_cast<int16_t>(srcW);
    int16_t copyH = static_cast<int16_t>(srcH);

    // Clip left
    if (dx0 < 0) { sx0 = -dx0; copyW += dx0; dx0 = 0; }
    // Clip top
    if (dy0 < 0) { sy0 = -dy0; copyH += dy0; dy0 = 0; }
    // Clip right
    if (dx0 + copyW > static_cast<int16_t>(t.width))
        copyW = static_cast<int16_t>(t.width) - dx0;
    // Clip bottom
    if (dy0 + copyH > static_cast<int16_t>(t.height))
        copyH = static_cast<int16_t>(t.height) - dy0;

    if (copyW <= 0 || copyH <= 0) return;

    for (int16_t row = 0; row < copyH; ++row) {
        int16_t srcRow = flipV ? (static_cast<int16_t>(srcH) - 1 - (sy0 + row))
                               : (sy0 + row);
        const uint16_t* srcLine = srcPixels + srcRow * srcW;
        uint16_t* dstLine = t.pixels + (dy0 + row) * t.width + dx0;

        if (!flipH) {
            // Direct copy — fast path
            std::memcpy(dstLine, srcLine + sx0,
                        static_cast<size_t>(copyW) * sizeof(uint16_t));
        } else {
            // Horizontal flip
            int16_t srcStart = static_cast<int16_t>(srcW) - 1 - sx0;
            for (int16_t col = 0; col < copyW; ++col) {
                dstLine[col] = srcLine[srcStart - col];
            }
        }
    }
}

uint16_t Rasterizer2D::BlendRGB565(uint16_t src, uint16_t dst, uint8_t alpha) {
    // Fast RGB565 alpha blend: separate R5, G6, B5 channels
    uint32_t srcR = (src >> 11) & 0x1F;
    uint32_t srcG = (src >> 5)  & 0x3F;
    uint32_t srcB =  src        & 0x1F;

    uint32_t dstR = (dst >> 11) & 0x1F;
    uint32_t dstG = (dst >> 5)  & 0x3F;
    uint32_t dstB =  dst        & 0x1F;

    uint32_t inv = 255 - alpha;
    uint32_t r = (srcR * alpha + dstR * inv) / 255;
    uint32_t g = (srcG * alpha + dstG * inv) / 255;
    uint32_t b = (srcB * alpha + dstB * inv) / 255;

    return static_cast<uint16_t>((r << 11) | (g << 5) | b);
}
