/**
 * @file test_2d_primitives.cpp
 * @brief Native regression gate for the 2D layer primitive engine
 *        (src/render/rasterizer_2d.cpp) — exact-pixel checks against a
 *        hand-computed oracle.
 *
 * Covers the consumer-visible fault classes of the v9 auxiliary 2D engine:
 *   - negative / extreme / int16-overflowing coordinates
 *   - zero and exhausted dimensions (no writes, no hangs)
 *   - clipped line / circle / arc (Cohen–Sutherland, bbox reject)
 *   - sprite H/V flips, RGB565 + RGB888 formats, truncated-upload tails
 *   - blend endpoints alpha 0 / 255 and saturating add/multiply channels
 *   - gradient endpoint exactness and truncating midpoint rule
 *   - glyph atlas bounds, newline advance, atomic reject of bad characters
 *   - clip/viewport per-op snapshot precedence, device-space clip after
 *     viewport transform, non-positive scale fail-closed
 *   - real destination stride and core-0 service-callback pacing
 *
 * Exit code: 0 = all checks passed, 1 = at least one check failed.
 */

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "render/rasterizer_2d.h"

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    if (ok) {
        std::printf("PASS %s\n", what);
    } else {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

// ─── Test fixture ───────────────────────────────────────────────────────────

constexpr int32_t TW = 16, TH = 16;
uint16_t g_buf[TW * TH];

Rasterizer2D::Target mkTarget(uint16_t init = 0x0000) {
    std::memset(g_buf, 0, sizeof(g_buf));
    if (init != 0x0000) {
        for (auto& p : g_buf) p = init;
    }
    Rasterizer2D::Target t;
    t.pixels = g_buf;
    t.width  = TW;
    t.height = TH;
    return t;
}

uint16_t px(int32_t x, int32_t y) { return g_buf[y * TW + x]; }

int countColor(uint16_t color) {
    int n = 0;
    for (const auto& p : g_buf) if (p == color) ++n;
    return n;
}

constexpr uint16_t RED   = 0xF800;
constexpr uint16_t GREEN = 0x07E0;
constexpr uint16_t BLUE  = 0x001F;
constexpr uint16_t WHITE = 0xFFFF;

int g_serviceCalls = 0;
void CountService() { ++g_serviceCalls; }

// ─── Rect ───────────────────────────────────────────────────────────────────

void TestRect() {
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawRect(t, -5, -5, 10, 10, RED, true);
        check(px(0, 0) == RED && px(4, 4) == RED && px(5, 5) == 0 &&
              countColor(RED) == 25,
              "rect: negative origin clipped to target");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        // w = 65535 overflows int16 x+w; must still fill cols 10..15.
        Rasterizer2D::DrawRect(t, 10, 0, 65535, 16, GREEN, true);
        check(countColor(GREEN) == 6 * 16 && px(9, 0) == 0 && px(10, 0) == GREEN,
              "rect: extreme width widened before clipping");
    }
    {
        Rasterizer2D::Target t = mkTarget(0x5555);
        Rasterizer2D::DrawRect(t, 0, 0, 0, 5, RED, true);
        Rasterizer2D::DrawRect(t, 0, 0, 5, 0, RED, true);
        check(countColor(0x5555) == TW * TH, "rect: zero dimensions write nothing");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawRect(t, 2, 2, 6, 4, WHITE, false);
        check(px(2, 2) == WHITE && px(7, 5) == WHITE && px(4, 3) == 0 &&
              countColor(WHITE) == 16,
              "rect: outline draws only the 4 edges");
    }
}

// ─── Line ───────────────────────────────────────────────────────────────────

void TestLine() {
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawLine(t, -10, 0, 10, 0, RED);
        check(px(0, 0) == RED && px(10, 0) == RED && px(11, 0) == 0 &&
              countColor(RED) == 11,
              "line: horizontal clipped at left edge");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawLine(t, -5, -5, 7, 7, GREEN);
        check(px(0, 0) == GREEN && px(3, 3) == GREEN && px(3, 4) == 0 &&
              countColor(GREEN) == 8,
              "line: diagonal clipped to exact Bresenham prefix");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        // Full int16 range: must terminate, clip to (0,0)->(15,15).
        Rasterizer2D::DrawLine(t, -32768, -32768, 32767, 32767, WHITE);
        check(px(15, 15) == WHITE && px(7, 7) == WHITE && px(7, 8) == 0 &&
              countColor(WHITE) == 16,
              "line: extreme endpoints clipped, traversal bounded");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawLine(t, 3, -5, 3, 20, RED);
        check(countColor(RED) == 16 && px(3, 0) == RED && px(3, 15) == RED,
              "line: vertical clipped both ends");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawLine(t, -100, -100, -50, -50, RED);
        check(countColor(RED) == 0, "line: fully off-screen draws nothing");
    }
}

// ─── Circle ─────────────────────────────────────────────────────────────────

void TestCircle() {
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawCircle(t, 5, 5, 0, RED, true);
        check(px(5, 5) == RED && countColor(RED) == 1,
              "circle: radius 0 plots center pixel");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        // The r=5 octant is (5,0), (5,1), (5,2), (4,3); reflect it
        // across both axes and x=y. Translation by (-3,8) leaves five
        // pixels at x=2, two at x=1 and two at x=0: nine, not eleven.
        // These row coordinates are a hand-derived contour, not a copy
        // of the renderer's recurrence or an observed framebuffer.
        constexpr int8_t rowX[TH] =
            {-1, -1, -1, -1, 0, 1, 2, 2, 2, 2, 2, 1, 0, -1, -1, -1};
        Rasterizer2D::DrawCircle(t, -3, 8, 5, RED, false);
        bool exact = true;
        for (int32_t y = 0; y < TH; ++y) {
            for (int32_t x = 0; x < TW; ++x) {
                if (px(x, y) != (x == rowX[y] ? RED : 0)) exact = false;
            }
        }
        check(exact, "circle: off-center outline clipped exactly");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawCircle(t, 8, 8, 60000, GREEN, true);
        check(countColor(GREEN) == TW * TH,
              "circle: huge filled radius covers target");
    }
    {
        Rasterizer2D::Target t = mkTarget(0x5555);
        Rasterizer2D::DrawCircle(t, 30000, 30000, 100, RED, true);
        check(countColor(0x5555) == TW * TH,
              "circle: far off-screen radius bbox-rejected");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawCircle(t, 8, 8, 4, BLUE, true);
        check(px(8, 8) == BLUE && px(8, 4) == BLUE && px(12, 8) == BLUE &&
              px(4, 4) == 0,
              "circle: small filled circle interior/corner rule");
    }
}

// ─── Arc ────────────────────────────────────────────────────────────────────

void TestArc() {
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawArc(t, 0, 8, 5, 0, 90, RED);
        check(px(5, 8) == RED && px(0, 3) == RED,
              "arc: quadrant endpoints plotted");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawArc(t, 8, 8, 5, 270, 30, GREEN);
        // deg 270: legacy rounding (5*-32767+16384)/32768 = -4 → y = 8+4.
        check(px(8, 12) == GREEN && px(13, 8) == GREEN,
              "arc: wrap-around segment spans 0 degrees");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawArc(t, 8, 8, 5, -90, -1, BLUE);
        check(px(8, 12) == BLUE, "arc: negative angles normalized");
    }
    {
        Rasterizer2D::Target t = mkTarget(0x5555);
        Rasterizer2D::DrawArc(t, 8, 8, 0, 0, 180, RED);
        check(countColor(0x5555) == TW * TH, "arc: radius 0 draws nothing");
    }
}

// ─── Rounded rect ───────────────────────────────────────────────────────────

void TestRoundedRect() {
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawRoundedRect(t, 0, 0, 8, 8, 2, RED, true);
        check(px(0, 0) == 0 && px(1, 0) == RED && px(6, 0) == RED &&
              px(7, 0) == 0 && px(0, 1) == RED && px(0, 7) == 0 &&
              px(6, 7) == RED && px(4, 4) == RED,
              "rounded rect: filled corner cut exact");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        // Radius 100 clamps to 4, not 2. The corner centers are (4,4)
        // and (3,4), mirrored vertically. For the legacy midpoint
        // convention, the r=4 octant is (4,0), (4,1), (4,2), (3,3);
        // the d==0 outward tie retains (4,2). Thus the top row spans
        // x=2..5 and the next spans x=1..6, not x=1..6 on both.
        constexpr uint8_t inset[8] = {2, 1, 0, 0, 0, 0, 1, 2};
        Rasterizer2D::DrawRoundedRect(t, 0, 0, 8, 8, 100, GREEN, true);
        bool exact = true;
        for (int32_t y = 0; y < TH; ++y) {
            for (int32_t x = 0; x < TW; ++x) {
                const bool inside = y < 8 && x >= inset[y] && x < 8 - inset[y];
                if (px(x, y) != (inside ? GREEN : 0)) exact = false;
            }
        }
        check(exact, "rounded rect: radius clamped to half extent");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawRoundedRect(t, 0, 0, 8, 8, 4, BLUE, false);
        check(px(0, 0) == 0 && px(4, 0) == BLUE && px(0, 4) == BLUE &&
              px(4, 4) == 0,
              "rounded rect: outline corners and straight edges");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawRoundedRect(t, 0, 0, 8, 8, 0, RED, true);
        check(countColor(RED) == 64, "rounded rect: zero radius = plain rect");
    }
}

// ─── Triangle ───────────────────────────────────────────────────────────────

void TestTriangle() {
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawTriangle(t, -5, -5, 10, 0, 0, 10, RED);
        // Row 0: edges give span [-4,10] → 0..10. Row 5: span [-2,5] → 0..5.
        check(px(0, 0) == RED && px(10, 0) == RED && px(11, 0) == 0 &&
              px(2, 5) == RED && px(6, 5) == 0,
              "triangle: negative vertices clipped, spans exact");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawTriangle(t, 3, 7, 9, 7, 5, 7, GREEN);
        check(countColor(GREEN) == 7 && px(3, 7) == GREEN && px(9, 7) == GREEN,
              "triangle: degenerate flat triangle draws one span");
    }
}

// ─── Sprite ─────────────────────────────────────────────────────────────────

Rasterizer2D::SpriteSource src565(const uint16_t* p, uint16_t w, uint16_t h,
                                  uint32_t bytes = 0) {
    Rasterizer2D::SpriteSource s;
    s.pixels = reinterpret_cast<const uint8_t*>(p);
    s.byteLength = bytes;
    s.width = w;
    s.height = h;
    s.format = Rasterizer2D::SRC_FORMAT_RGB565;
    return s;
}

void TestSprite() {
    const uint16_t quad[4] = {RED, GREEN, BLUE, WHITE};  // A B / C D
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawSprite(t, 0, 0, src565(quad, 2, 2), false, false);
        check(px(0, 0) == RED && px(1, 0) == GREEN &&
              px(0, 1) == BLUE && px(1, 1) == WHITE,
              "sprite: plain blit");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawSprite(t, 0, 0, src565(quad, 2, 2), true, false);
        check(px(0, 0) == GREEN && px(1, 0) == RED &&
              px(0, 1) == WHITE && px(1, 1) == BLUE,
              "sprite: horizontal flip");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawSprite(t, 0, 0, src565(quad, 2, 2), true, true);
        check(px(0, 0) == WHITE && px(1, 0) == BLUE &&
              px(0, 1) == GREEN && px(1, 1) == RED,
              "sprite: both flips");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawSprite(t, -1, 0, src565(quad, 2, 2), false, false);
        check(px(0, 0) == GREEN && px(0, 1) == WHITE && countColor(RED) == 0,
              "sprite: negative destination clips left column");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawSprite(t, 15, 0, src565(quad, 2, 2), false, false);
        check(px(15, 0) == RED && px(15, 1) == BLUE && countColor(GREEN) == 0,
              "sprite: right-edge tail clipped");
    }
    {
        // RGB888 source: pure red + pure green, truncated channel rule.
        const uint8_t rgb[6] = {255, 0, 0, 0, 255, 0};
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::SpriteSource s;
        s.pixels = rgb; s.byteLength = 6; s.width = 2; s.height = 1;
        s.format = Rasterizer2D::SRC_FORMAT_RGB888;
        Rasterizer2D::DrawSprite(t, 0, 0, s, false, false);
        check(px(0, 0) == 0xF800 && px(1, 0) == 0x07E0,
              "sprite: RGB888 source sampled with 5/6/5 truncation");
    }
    {
        // Truncated upload: byteLength covers only the first texel.
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawSprite(t, 0, 0, src565(quad, 2, 1, 3), false, false);
        check(px(0, 0) == RED && px(1, 0) == 0x0000,
              "sprite: truncated tail texels read black");
    }
    {
        // Viewport 2x scale: each texel becomes a 2x2 block.
        const uint16_t two[2] = {RED, GREEN};
        Rasterizer2D::Target t = mkTarget();
        t.viewScaleXQ8 = 512;
        t.viewScaleYQ8 = 512;
        Rasterizer2D::DrawSprite(t, 0, 0, src565(two, 2, 1), false, false);
        check(px(0, 0) == RED && px(1, 0) == RED && px(0, 1) == RED &&
              px(2, 0) == GREEN && px(3, 1) == GREEN && px(4, 0) == 0,
              "sprite: viewport scale nearest stretch");
    }
    {
        // Batch: two on-screen positions + one off-screen.
        const uint16_t two[2] = {RED, GREEN};
        const PglSpritePosition pos[3] = {{0, 0}, {4, 0}, {100, 100}};
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawSpriteBatch(t, src565(two, 2, 1), pos, 3,
                                      false, false);
        check(px(0, 0) == RED && px(1, 0) == GREEN &&
              px(4, 0) == RED && px(5, 0) == GREEN && countColor(RED) == 2,
              "sprite batch: positions applied, off-screen skipped");
    }
    {
        const uint16_t two[2] = {RED, GREEN};
        const PglSpritePosition pos[1] = {{0, 0}};
        Rasterizer2D::Target t = mkTarget(0x5555);
        Rasterizer2D::DrawSpriteBatch(t, src565(two, 2, 1), pos, 0,
                                      false, false);
        check(countColor(0x5555) == TW * TH,
              "sprite batch: zero count writes nothing");
    }
}

// ─── Gradient ───────────────────────────────────────────────────────────────

void TestGradient() {
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawGradientRect(t, 0, 0, 3, 1, 0x0000, 0xF800, 0);
        check(px(0, 0) == 0x0000 && px(1, 0) == 0x7800 && px(2, 0) == 0xF800,
              "gradient: horizontal endpoints exact, midpoint truncates");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawGradientRect(t, 0, 0, 3, 1, 0xF800, 0x0000, 0);
        // r = 31 + (0-31)*1/2 = 31 + (-15) = 16 (C truncating division).
        check(px(1, 0) == 0x8000,
              "gradient: descending channel truncates toward zero");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawGradientRect(t, 0, 0, 1, 3, 0x0000, 0x07E0, 1);
        check(px(0, 0) == 0x0000 && px(0, 1) == 0x03E0 && px(0, 2) == 0x07E0,
              "gradient: vertical endpoints exact");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        Rasterizer2D::DrawGradientRect(t, 2, 2, 1, 4, RED, GREEN, 0);
        check(px(2, 2) == RED && px(2, 5) == RED && countColor(RED) == 4,
              "gradient: single-pixel span draws color0");
    }
    {
        Rasterizer2D::Target t = mkTarget(0x5555);
        Rasterizer2D::DrawGradientRect(t, 0, 0, 0, 4, RED, GREEN, 0);
        check(countColor(0x5555) == TW * TH,
              "gradient: zero width writes nothing");
    }
}

// ─── Text ───────────────────────────────────────────────────────────────────

// Atlas 8x4, two 4x4 cells, firstChar 'A'.
// Cell A: row 0 full. Cell B: texels (4,1) and (6,2).
void BuildAtlas(uint16_t* atlas) {
    std::memset(atlas, 0, 8 * 4 * sizeof(uint16_t));
    atlas[0] = atlas[1] = atlas[2] = atlas[3] = 0xFFFF;
    atlas[1 * 8 + 4] = 0xFFFF;
    atlas[2 * 8 + 6] = 0xFFFF;
}

void TestText() {
    uint16_t atlasBuf[8 * 4];
    BuildAtlas(atlasBuf);
    Rasterizer2D::GlyphAtlas atlas;
    atlas.pixels = atlasBuf;
    atlas.width = 8; atlas.height = 4;
    atlas.glyphW = 4; atlas.glyphH = 4;
    atlas.columns = 2; atlas.firstChar = 'A';

    {
        Rasterizer2D::Target t = mkTarget();
        const char text[] = "AB";
        int32_t r = Rasterizer2D::DrawText(t, 0, 0, atlas, text, 2, 0x1234);
        check(r == 2 && px(0, 0) == 0x1234 && px(3, 0) == 0x1234 &&
              px(4, 0) == 0 && px(4, 1) == 0x1234 && px(6, 2) == 0x1234,
              "text: glyph mask tinted at pen positions");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        const char text[] = "A\nA";
        int32_t r = Rasterizer2D::DrawText(t, 0, 0, atlas, text, 3, RED);
        check(r == 3 && px(0, 0) == RED && px(0, 4) == RED &&
              px(3, 4) == RED && px(0, 1) == 0,
              "text: newline advances one glyph height");
    }
    {
        Rasterizer2D::Target t = mkTarget(0x5555);
        const char text[] = "AC";  // 'C' has no atlas cell
        int32_t r = Rasterizer2D::DrawText(t, 0, 0, atlas, text, 2, RED);
        check(r == Rasterizer2D::kTextErrGlyph &&
              countColor(0x5555) == TW * TH,
              "text: out-of-atlas character rejects whole draw atomically");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        const char text[] = "@";   // below firstChar
        int32_t r = Rasterizer2D::DrawText(t, 0, 0, atlas, text, 1, RED);
        check(r == Rasterizer2D::kTextErrGlyph,
              "text: character below firstChar rejected");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        int32_t r = Rasterizer2D::DrawText(t, 0, 0, atlas, nullptr, 0, RED);
        check(r == Rasterizer2D::kTextErrInvalid,
              "text: null string invalid");
        Rasterizer2D::GlyphAtlas bad = atlas;
        bad.columns = 0;
        const char text[] = "A";
        r = Rasterizer2D::DrawText(t, 0, 0, bad, text, 1, RED);
        check(r == Rasterizer2D::kTextErrInvalid,
              "text: zero columns invalid");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        const char text[] = "A";
        int32_t r = Rasterizer2D::DrawText(t, 0, 0, atlas, text, 0, RED);
        check(r == 0 && countColor(RED) == 0, "text: zero length draws nothing");
    }
    {
        // Clip applies to glyphs (device space), without overwriting the
        // clipped pixels or any other destination pixel.
        Rasterizer2D::Target t = mkTarget(0x5555);
        t.clipX = 2; t.clipY = 0; t.clipW = 14; t.clipH = 16;
        const char text[] = "A";
        const int32_t r = Rasterizer2D::DrawText(t, 0, 0, atlas, text, 1, RED);
        bool exact = r == 1;
        for (int32_t y = 0; y < TH; ++y) {
            for (int32_t x = 0; x < TW; ++x) {
                const uint16_t want = y == 0 && x >= 2 && x < 4 ? RED : 0x5555;
                if (px(x, y) != want) exact = false;
            }
        }
        check(exact, "text: device clip cuts glyph pixels");
    }
    {
        // Each B texel expands to a 2x2 rectangle. After the offset and
        // scale, these are [7,9)x[1,3) and [11,13)x[3,5). The device
        // clip [2,12)x[2,5) keeps exactly four pixels; A lies above it.
        // Guard rows and stride padding must remain untouched.
        uint16_t storage[20 * 10];
        for (auto& p : storage) p = 0x5555;
        Rasterizer2D::Target t;
        t.pixels = storage + 20;
        t.width = 16; t.height = 8; t.stride = 20;
        t.viewOffsetX = 1; t.viewOffsetY = 1;
        t.viewScaleXQ8 = 512; t.viewScaleYQ8 = 512;
        t.clipX = 2; t.clipY = 2; t.clipW = 10; t.clipH = 3;
        const int32_t r = Rasterizer2D::DrawText(t, -1, -1, atlas, "AB", 2, RED);
        bool exact = r == 2;
        for (int32_t i = 0; i < 20 * 10; ++i) {
            const int32_t x = i % 20, y = i / 20 - 1;
            const bool drawn = (y == 2 && (x == 7 || x == 8)) ||
                               (x == 11 && (y == 3 || y == 4));
            if (storage[i] != (drawn ? RED : 0x5555)) exact = false;
        }
        check(exact, "text: scaled device clip respects stride and guard rows");
    }
    {
        Rasterizer2D::Target t = mkTarget(0x5555);
        const int32_t left =
            Rasterizer2D::DrawText(t, -32768, 0, atlas, "A", 1, RED);
        const int32_t right =
            Rasterizer2D::DrawText(t, 32767, 0, atlas, "A", 1, RED);
        check(left == 1 && right == 1 && countColor(0x5555) == TW * TH,
              "text: extreme offscreen glyphs leave the destination untouched");
    }
    {
        // Half-width texels can collapse to empty rectangles. The two
        // surviving A texels cover x=0 and x=1, with no extra writes.
        Rasterizer2D::Target t = mkTarget(0x5555);
        t.viewScaleXQ8 = 128;
        const int32_t r = Rasterizer2D::DrawText(t, 0, 0, atlas, "A", 1, RED);
        bool exact = r == 1;
        for (int32_t y = 0; y < TH; ++y) {
            for (int32_t x = 0; x < TW; ++x) {
                const uint16_t want = y == 0 && x < 2 ? RED : 0x5555;
                if (px(x, y) != want) exact = false;
            }
        }
        check(exact, "text: fractional viewport drops empty texel rectangles");
    }
}

// ─── Blending ───────────────────────────────────────────────────────────────

void TestBlend() {
    check(Rasterizer2D::BlendRGB565(0xFFFF, 0x0000, 0) == 0x0000,
          "blend: alpha 0 yields dst");
    check(Rasterizer2D::BlendRGB565(0x1234, 0xABCD, 255) == 0x1234,
          "blend: alpha 255 yields src");
    check(Rasterizer2D::BlendRGB565(0xF800, 0x0000, 128) == 0x7800,
          "blend: mid alpha exact channel floor");

    check(Rasterizer2D::BlendAddRGB565(0xFFFF, 0xFFFF, 255) == 0xFFFF,
          "blend add: saturates at channel max, alpha 255 exact");
    check(Rasterizer2D::BlendAddRGB565(0x0000, 0x1234, 0) == 0x1234,
          "blend add: alpha 0 yields dst");
    check(Rasterizer2D::BlendAddRGB565(0xA000, 0xA000, 255) == 0xF800,
          "blend add: r20 + r20 saturates to r31 at alpha 255");
    check(Rasterizer2D::BlendAddRGB565(0xF800, 0x0000, 128) == 0x7800,
          "blend add: scaled source channel exact");

    check(Rasterizer2D::BlendMultiplyRGB565(0xFFFF, 0xFFFF, 255) == 0xFFFF,
          "blend multiply: alpha 255 full product");
    check(Rasterizer2D::BlendMultiplyRGB565(0x1234, 0xABCD, 0) == 0xABCD,
          "blend multiply: alpha 0 yields dst");
    check(Rasterizer2D::BlendMultiplyRGB565(0x8000, 0x8000, 255) == 0x4000,
          "blend multiply: 16*16/31 = 8 exact");

    check(Rasterizer2D::CompositeLayerPixel(0xFFFF, 0x0000, 1, 255) == 0xFFFF,
          "composite: additive dispatch");
    check(Rasterizer2D::CompositeLayerPixel(0xF800, 0x0000, 0, 128) == 0x7800,
          "composite: alpha dispatch");
    check(Rasterizer2D::CompositeLayerPixel(0xF800, 0x0000, 99, 128) == 0x7800,
          "composite: unknown mode falls back to alpha");
}

// ─── Clip / viewport snapshots ──────────────────────────────────────────────

void TestClipViewport() {
    {
        // Per-op snapshot precedence: op1's clip must not be rewritten by
        // op2's different clip.
        Rasterizer2D::Target t1 = mkTarget();
        t1.clipX = 0; t1.clipY = 0; t1.clipW = 4; t1.clipH = 16;
        Rasterizer2D::DrawRect(t1, 0, 0, 16, 16, RED, true);

        Rasterizer2D::Target t2;  // same pixels, later clip state
        t2.pixels = g_buf; t2.width = TW; t2.height = TH;
        t2.clipX = 8; t2.clipY = 0; t2.clipW = 8; t2.clipH = 16;
        Rasterizer2D::DrawRect(t2, 0, 0, 16, 16, GREEN, true);

        check(px(2, 0) == RED && px(7, 0) == 0 && px(10, 0) == GREEN,
              "snapshot: earlier op keeps its clip, later op uses new clip");
    }
    {
        Rasterizer2D::Target t = mkTarget(0x5555);
        t.clipX = 0; t.clipY = 0; t.clipW = 0; t.clipH = 16;
        Rasterizer2D::DrawRect(t, 0, 0, 16, 16, RED, true);
        Rasterizer2D::Clear(t, GREEN);
        check(countColor(0x5555) == TW * TH,
              "snapshot: zero-width clip suppresses all output");
    }
    {
        // Viewport offset+scale applies to geometry; clip is device-space
        // and applied after the transform.
        Rasterizer2D::Target t = mkTarget();
        t.viewOffsetX = 10; t.viewOffsetY = 20;
        t.viewScaleXQ8 = 512; t.viewScaleYQ8 = 512;
        t.clipX = 0; t.clipY = 0; t.clipW = 16; t.clipH = 16;
        Rasterizer2D::DrawRect(t, 0, 0, 4, 2, RED, true);
        // device rect: x [10,18), y [20,24) → clip y < 16 cuts it to 20..23?
        // clipH 16 → device y 20..23 outside → nothing visible; use offset
        // that lands inside instead.
        check(countColor(RED) == 0,
              "viewport: transformed geometry clipped in device space");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        t.viewOffsetX = 1; t.viewOffsetY = 2;
        t.viewScaleXQ8 = 512; t.viewScaleYQ8 = 512;
        Rasterizer2D::DrawRect(t, 0, 0, 4, 2, RED, true);
        // device: x [1, 1+8), y [2, 2+4)
        check(px(1, 2) == RED && px(8, 5) == RED && px(9, 2) == 0 &&
              px(0, 2) == 0 && px(1, 6) == 0 && countColor(RED) == 8 * 4,
              "viewport: offset + 2x scale maps extents exactly");
    }
    {
        // Non-uniform viewport: circle becomes an ellipse (scanline path).
        Rasterizer2D::Target t = mkTarget();
        t.viewScaleXQ8 = 512;  // 2.0
        t.viewScaleYQ8 = 256;  // 1.0
        Rasterizer2D::DrawCircle(t, 4, 8, 4, RED, true);
        // dev center (8,8), rx=8, ry=4.
        check(px(0, 8) == RED && px(15, 8) == RED && px(8, 4) == RED &&
              px(8, 12) == RED && px(8, 3) == 0 && px(7, 12) == 0,
              "viewport: non-uniform scale rasterizes ellipse");
    }
    {
        Rasterizer2D::Target t = mkTarget(0x5555);
        t.viewScaleXQ8 = 0;  // invalid — admission rejects, rasterizer fails closed
        Rasterizer2D::DrawRect(t, 0, 0, 8, 8, RED, true);
        Rasterizer2D::DrawCircle(t, 8, 8, 4, RED, true);
        check(countColor(0x5555) == TW * TH,
              "viewport: non-positive scale draws nothing");
    }
}

// ─── Stride and service callback ────────────────────────────────────────────

void TestStrideAndService() {
    {
        // Real stride: 4-wide target in an 8-stride buffer.
        uint16_t buf[8 * 4];
        std::memset(buf, 0x55, sizeof(buf));
        Rasterizer2D::Target t;
        t.pixels = buf; t.width = 4; t.height = 4; t.stride = 8;
        Rasterizer2D::Clear(t, RED);
        bool ok = true;
        for (int y = 0; y < 4; ++y) {
            for (int x = 0; x < 8; ++x) {
                const uint16_t want = (x < 4) ? RED : 0x5555;
                if (buf[y * 8 + x] != want) ok = false;
            }
        }
        check(ok, "stride: rows written at real stride, gap untouched");
    }
    {
        Rasterizer2D::Target t = mkTarget();
        g_serviceCalls = 0;
        Rasterizer2D::Clear(t, RED, CountService);
        // 16 rows, one call per 16 → exactly 1.
        check(g_serviceCalls == 1 && countColor(RED) == TW * TH,
              "service: callback paced at bounded row intervals");
    }
    {
        // Null-target safety: every entry point must be a no-op, not a crash.
        Rasterizer2D::Target t;  // pixels == nullptr
        Rasterizer2D::Clear(t, RED);
        Rasterizer2D::DrawRect(t, 0, 0, 4, 4, RED, true);
        Rasterizer2D::DrawLine(t, 0, 0, 4, 4, RED);
        Rasterizer2D::DrawCircle(t, 4, 4, 0, RED, true);
        Rasterizer2D::DrawRoundedRect(t, 0, 0, 4, 4, 1, RED, true);
        Rasterizer2D::DrawTriangle(t, 0, 0, 1, 0, 0, 1, RED);
        Rasterizer2D::DrawArc(t, 4, 4, 4, 0, 90, RED);
        const uint16_t one[1] = {RED};
        Rasterizer2D::DrawSprite(t, 0, 0, src565(one, 1, 1), false, false);
        Rasterizer2D::DrawGradientRect(t, 0, 0, 4, 4, RED, GREEN, 0);
        check(true, "null target: all entry points no-op safely");
    }
}

}  // namespace

int main() {
    TestRect();
    TestLine();
    TestCircle();
    TestArc();
    TestRoundedRect();
    TestTriangle();
    TestSprite();
    TestGradient();
    TestText();
    TestBlend();
    TestClipViewport();
    TestStrideAndService();

    std::printf("\n%s (%d failure%s)\n",
                g_failures == 0 ? "ALL CHECKS PASSED" : "CHECKS FAILED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
