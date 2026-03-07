/**
 * @file headless_selftest.cpp
 * @brief Headless self-test scene provider and HUD overlay.
 *
 * Provides built-in scene content and a bitmap-font HUD for the headless
 * self-test mode.  The rendering pipeline lives in gpu_core.cpp — this
 * module only supplies the scene data and overlays.
 *
 * Scene contents (set up by BuildScene):
 *   - Mesh 0: unit cube (8 verts, 12 faces)
 *   - Mesh 1: Utah teapot (587 verts, 1166 faces)
 *   - Material 0: warm directional light (cube)
 *   - Material 1: cool directional light (teapot)
 *   - Camera 0: perspective at z=-5, identity rotation
 *
 * Draw calls (rebuilt per frame by UpdateDrawList):
 *   - First 10 seconds: single centered rotating cube (mesh 0, mat 0)
 *   - After 10 seconds: single centered rotating teapot (mesh 1, mat 1)
 *
 * Only compiled when RP2350GPU_HEADLESS_SELFTEST is defined.
 */

#ifdef RP2350GPU_HEADLESS_SELFTEST

#include "headless_selftest.h"
#include "teapot_mesh.h"
#include "../gpu_config.h"
#include "../scene_state.h"
#include "../math/pgl_math.h"

#include <cstdio>
#include <cstring>
#include <cmath>

// ─── RGB565 Helper ──────────────────────────────────────────────────────────

static inline uint16_t Rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint16_t)(r >> 3) << 11) |
           ((uint16_t)(g >> 2) <<  5) |
           ((uint16_t)(b >> 3));
}

// ─── Minimal 5×7 Bitmap Font ────────────────────────────────────────────────
// Covers ASCII 0x20–0x7E (space to tilde). Each glyph is 5 columns wide,
// 7 rows tall, stored as 5 bytes (one per column, LSB = top row).

// clang-format off
static const uint8_t font5x7[][5] = {
    // 0x20 ' '
    {0x00,0x00,0x00,0x00,0x00},
    // 0x21 '!'
    {0x00,0x00,0x5F,0x00,0x00},
    // 0x22 '"'
    {0x00,0x07,0x00,0x07,0x00},
    // 0x23 '#'
    {0x14,0x7F,0x14,0x7F,0x14},
    // 0x24 '$'
    {0x24,0x2A,0x7F,0x2A,0x12},
    // 0x25 '%'
    {0x23,0x13,0x08,0x64,0x62},
    // 0x26 '&'
    {0x36,0x49,0x55,0x22,0x50},
    // 0x27 '''
    {0x00,0x05,0x03,0x00,0x00},
    // 0x28 '('
    {0x00,0x1C,0x22,0x41,0x00},
    // 0x29 ')'
    {0x00,0x41,0x22,0x1C,0x00},
    // 0x2A '*'
    {0x14,0x08,0x3E,0x08,0x14},
    // 0x2B '+'
    {0x08,0x08,0x3E,0x08,0x08},
    // 0x2C ','
    {0x00,0x50,0x30,0x00,0x00},
    // 0x2D '-'
    {0x08,0x08,0x08,0x08,0x08},
    // 0x2E '.'
    {0x00,0x60,0x60,0x00,0x00},
    // 0x2F '/'
    {0x20,0x10,0x08,0x04,0x02},
    // 0x30 '0'
    {0x3E,0x51,0x49,0x45,0x3E},
    // 0x31 '1'
    {0x00,0x42,0x7F,0x40,0x00},
    // 0x32 '2'
    {0x42,0x61,0x51,0x49,0x46},
    // 0x33 '3'
    {0x21,0x41,0x45,0x4B,0x31},
    // 0x34 '4'
    {0x18,0x14,0x12,0x7F,0x10},
    // 0x35 '5'
    {0x27,0x45,0x45,0x45,0x39},
    // 0x36 '6'
    {0x3C,0x4A,0x49,0x49,0x30},
    // 0x37 '7'
    {0x01,0x71,0x09,0x05,0x03},
    // 0x38 '8'
    {0x36,0x49,0x49,0x49,0x36},
    // 0x39 '9'
    {0x06,0x49,0x49,0x29,0x1E},
    // 0x3A ':'
    {0x00,0x36,0x36,0x00,0x00},
    // 0x3B ';'
    {0x00,0x56,0x36,0x00,0x00},
    // 0x3C '<'
    {0x08,0x14,0x22,0x41,0x00},
    // 0x3D '='
    {0x14,0x14,0x14,0x14,0x14},
    // 0x3E '>'
    {0x00,0x41,0x22,0x14,0x08},
    // 0x3F '?'
    {0x02,0x01,0x51,0x09,0x06},
    // 0x40 '@'
    {0x32,0x49,0x79,0x41,0x3E},
    // 0x41 'A'
    {0x7E,0x11,0x11,0x11,0x7E},
    // 0x42 'B'
    {0x7F,0x49,0x49,0x49,0x36},
    // 0x43 'C'
    {0x3E,0x41,0x41,0x41,0x22},
    // 0x44 'D'
    {0x7F,0x41,0x41,0x22,0x1C},
    // 0x45 'E'
    {0x7F,0x49,0x49,0x49,0x41},
    // 0x46 'F'
    {0x7F,0x09,0x09,0x09,0x01},
    // 0x47 'G'
    {0x3E,0x41,0x49,0x49,0x7A},
    // 0x48 'H'
    {0x7F,0x08,0x08,0x08,0x7F},
    // 0x49 'I'
    {0x00,0x41,0x7F,0x41,0x00},
    // 0x4A 'J'
    {0x20,0x40,0x41,0x3F,0x01},
    // 0x4B 'K'
    {0x7F,0x08,0x14,0x22,0x41},
    // 0x4C 'L'
    {0x7F,0x40,0x40,0x40,0x40},
    // 0x4D 'M'
    {0x7F,0x02,0x0C,0x02,0x7F},
    // 0x4E 'N'
    {0x7F,0x04,0x08,0x10,0x7F},
    // 0x4F 'O'
    {0x3E,0x41,0x41,0x41,0x3E},
    // 0x50 'P'
    {0x7F,0x09,0x09,0x09,0x06},
    // 0x51 'Q'
    {0x3E,0x41,0x51,0x21,0x5E},
    // 0x52 'R'
    {0x7F,0x09,0x19,0x29,0x46},
    // 0x53 'S'
    {0x46,0x49,0x49,0x49,0x31},
    // 0x54 'T'
    {0x01,0x01,0x7F,0x01,0x01},
    // 0x55 'U'
    {0x3F,0x40,0x40,0x40,0x3F},
    // 0x56 'V'
    {0x1F,0x20,0x40,0x20,0x1F},
    // 0x57 'W'
    {0x3F,0x40,0x38,0x40,0x3F},
    // 0x58 'X'
    {0x63,0x14,0x08,0x14,0x63},
    // 0x59 'Y'
    {0x07,0x08,0x70,0x08,0x07},
    // 0x5A 'Z'
    {0x61,0x51,0x49,0x45,0x43},
    // 0x5B '['
    {0x00,0x7F,0x41,0x41,0x00},
    // 0x5C '\'
    {0x02,0x04,0x08,0x10,0x20},
    // 0x5D ']'
    {0x00,0x41,0x41,0x7F,0x00},
    // 0x5E '^'
    {0x04,0x02,0x01,0x02,0x04},
    // 0x5F '_'
    {0x40,0x40,0x40,0x40,0x40},
    // 0x60 '`'
    {0x00,0x01,0x02,0x04,0x00},
    // 0x61 'a'
    {0x20,0x54,0x54,0x54,0x78},
    // 0x62 'b'
    {0x7F,0x48,0x44,0x44,0x38},
    // 0x63 'c'
    {0x38,0x44,0x44,0x44,0x20},
    // 0x64 'd'
    {0x38,0x44,0x44,0x48,0x7F},
    // 0x65 'e'
    {0x38,0x54,0x54,0x54,0x18},
    // 0x66 'f'
    {0x08,0x7E,0x09,0x01,0x02},
    // 0x67 'g'
    {0x0C,0x52,0x52,0x52,0x3E},
    // 0x68 'h'
    {0x7F,0x08,0x04,0x04,0x78},
    // 0x69 'i'
    {0x00,0x44,0x7D,0x40,0x00},
    // 0x6A 'j'
    {0x20,0x40,0x44,0x3D,0x00},
    // 0x6B 'k'
    {0x7F,0x10,0x28,0x44,0x00},
    // 0x6C 'l'
    {0x00,0x41,0x7F,0x40,0x00},
    // 0x6D 'm'
    {0x7C,0x04,0x18,0x04,0x78},
    // 0x6E 'n'
    {0x7C,0x08,0x04,0x04,0x78},
    // 0x6F 'o'
    {0x38,0x44,0x44,0x44,0x38},
    // 0x70 'p'
    {0x7C,0x14,0x14,0x14,0x08},
    // 0x71 'q'
    {0x08,0x14,0x14,0x18,0x7C},
    // 0x72 'r'
    {0x7C,0x08,0x04,0x04,0x08},
    // 0x73 's'
    {0x48,0x54,0x54,0x54,0x20},
    // 0x74 't'
    {0x04,0x3F,0x44,0x40,0x20},
    // 0x75 'u'
    {0x3C,0x40,0x40,0x20,0x7C},
    // 0x76 'v'
    {0x1C,0x20,0x40,0x20,0x1C},
    // 0x77 'w'
    {0x3C,0x40,0x30,0x40,0x3C},
    // 0x78 'x'
    {0x44,0x28,0x10,0x28,0x44},
    // 0x79 'y'
    {0x0C,0x50,0x50,0x50,0x3C},
    // 0x7A 'z'
    {0x44,0x64,0x54,0x4C,0x44},
    // 0x7B '{'
    {0x00,0x08,0x36,0x41,0x00},
    // 0x7C '|'
    {0x00,0x00,0x7F,0x00,0x00},
    // 0x7D '}'
    {0x00,0x41,0x36,0x08,0x00},
    // 0x7E '~'
    {0x10,0x08,0x08,0x10,0x08},
};
// clang-format on

// ─── Text Drawing Helpers ───────────────────────────────────────────────────

/// Draw a single 5×7 character at (x, y) onto an arbitrary framebuffer.
static void DrawCharTo(uint16_t* fb, uint16_t fbW, uint16_t fbH,
                       uint16_t x, uint16_t y, char ch, uint16_t color) {
    if (ch < 0x20 || ch > 0x7E) ch = '?';
    const uint8_t* glyph = font5x7[ch - 0x20];

    for (uint8_t col = 0; col < 5; ++col) {
        uint8_t bits = glyph[col];
        for (uint8_t row = 0; row < 7; ++row) {
            if (bits & (1 << row)) {
                uint16_t px = x + col;
                uint16_t py = y + row;
                if (px < fbW && py < fbH) {
                    fb[py * fbW + px] = color;
                }
            }
        }
    }
}

/// Draw a null-terminated string starting at (x, y), 6-pixel pitch.
static void DrawStringTo(uint16_t* fb, uint16_t fbW, uint16_t fbH,
                         uint16_t x, uint16_t y, const char* str,
                         uint16_t color) {
    while (*str) {
        DrawCharTo(fb, fbW, fbH, x, y, *str, color);
        x += 6;
        ++str;
    }
}

/// Draw a right-aligned string ending at xRight.
static void DrawStringRightTo(uint16_t* fb, uint16_t fbW, uint16_t fbH,
                              uint16_t xRight, uint16_t y, const char* str,
                              uint16_t color) {
    uint16_t len = 0;
    while (str[len]) ++len;
    uint16_t totalW = len * 6;
    int16_t startX = (int16_t)xRight - totalW;
    if (startX < 0) startX = 0;
    DrawStringTo(fb, fbW, fbH, (uint16_t)startX, y, str, color);
}

// ─── Simple itoa (no heap) ──────────────────────────────────────────────────

static void IntToStr(int32_t val, char* buf, uint8_t bufSize) {
    if (bufSize == 0) return;
    bool neg = (val < 0);
    if (neg) val = -val;

    char tmp[12];
    uint8_t len = 0;
    do {
        tmp[len++] = '0' + (val % 10);
        val /= 10;
    } while (val > 0 && len < sizeof(tmp));
    if (neg && len < sizeof(tmp)) tmp[len++] = '-';

    uint8_t i = 0;
    while (len > 0 && i < bufSize - 1) {
        buf[i++] = tmp[--len];
    }
    buf[i] = '\0';
}

// ─── Mesh Data — Cube ───────────────────────────────────────────────────────
// (Teapot data is in teapot_mesh.h, auto-generated from utah_teapot.obj.)

static const PglVec3 kCubeVerts[] = {
    { -0.5f, -0.5f,  0.5f },  // 0  front bottom-left
    {  0.5f, -0.5f,  0.5f },  // 1  front bottom-right
    {  0.5f,  0.5f,  0.5f },  // 2  front top-right
    { -0.5f,  0.5f,  0.5f },  // 3  front top-left
    { -0.5f, -0.5f, -0.5f },  // 4  back  bottom-left
    {  0.5f, -0.5f, -0.5f },  // 5  back  bottom-right
    {  0.5f,  0.5f, -0.5f },  // 6  back  top-right
    { -0.5f,  0.5f, -0.5f },  // 7  back  top-left
};
static const PglIndex3 kCubeIndices[] = {
    { 0, 1, 2 }, { 0, 2, 3 },   // Front  (+Z)
    { 5, 4, 7 }, { 5, 7, 6 },   // Back   (-Z)
    { 1, 5, 6 }, { 1, 6, 2 },   // Right  (+X)
    { 4, 0, 3 }, { 4, 3, 7 },   // Left   (-X)
    { 3, 2, 6 }, { 3, 6, 7 },   // Top    (+Y)
    { 4, 5, 1 }, { 4, 1, 0 },   // Bottom (-Y)
};
static constexpr uint16_t kCubeVertCount = 8;
static constexpr uint16_t kCubeFaceCount = 12;

// ─── Light Material Helper ──────────────────────────────────────────────────

static void SetupLightMaterial(MaterialSlot& mat,
                               float lx, float ly, float lz,
                               uint8_t ambR, uint8_t ambG, uint8_t ambB,
                               uint8_t difR, uint8_t difG, uint8_t difB) {
    mat.active    = true;
    mat.type      = PGL_MAT_LIGHT;
    mat.blendMode = PGL_BLEND_BASE;

    auto* p = reinterpret_cast<PglParamLight*>(mat.params);
    p->lightDirX = lx;
    p->lightDirY = ly;
    p->lightDirZ = lz;
    p->ambientR  = ambR;
    p->ambientG  = ambG;
    p->ambientB  = ambB;
    p->diffuseR  = difR;
    p->diffuseG  = difG;
    p->diffuseB  = difB;
}

// ─── Switch Timing ──────────────────────────────────────────────────────────
/// Seconds after startup to switch from cube to teapot.
static constexpr float kSwitchTimeS = 10.0f;

// ─── Scene Setup ────────────────────────────────────────────────────────────

void HeadlessSelfTest::BuildScene(SceneState* scene) {
    // ── Mesh 0: Cube ────────────────────────────────────────────────────
    {
        MeshSlot& m = scene->meshes[0];
        m.active        = true;
        m.vertexCount   = kCubeVertCount;
        m.triangleCount = kCubeFaceCount;

        m.vertices = scene->AllocVertices(kCubeVertCount);
        m.indices  = scene->AllocIndices(kCubeFaceCount);

        std::memcpy(m.vertices, kCubeVerts, kCubeVertCount * sizeof(PglVec3));
        std::memcpy(m.indices,  kCubeIndices, kCubeFaceCount * sizeof(PglIndex3));
        m.RecomputeAABB();
    }

    // ── Mesh 1: Utah Teapot ─────────────────────────────────────────────
    {
        MeshSlot& m = scene->meshes[1];
        m.active        = true;
        m.vertexCount   = kTeapotVertCount;
        m.triangleCount = kTeapotFaceCount;

        m.vertices = scene->AllocVertices(kTeapotVertCount);
        m.indices  = scene->AllocIndices(kTeapotFaceCount);

        std::memcpy(m.vertices, kTeapotVerts,   kTeapotVertCount * sizeof(PglVec3));
        std::memcpy(m.indices,  kTeapotIndices,  kTeapotFaceCount * sizeof(PglIndex3));
        m.RecomputeAABB();
    }

    // ── Material 0: Warm directional light (cube) ───────────────────────
    // Light from upper-right-front; warm orange ambient + bright gold diffuse.
    SetupLightMaterial(scene->materials[0],
                       0.5f, 0.8f, -0.6f,       // light direction
                       40, 15, 10,               // ambient RGB
                       255, 190, 80);            // diffuse RGB

    // ── Material 1: Cool directional light (teapot) ─────────────────────
    // Light from upper-left-front; cool blue ambient + teal diffuse.
    SetupLightMaterial(scene->materials[1],
                       -0.4f, 0.7f, -0.8f,      // light direction
                       12, 18, 35,               // ambient RGB
                       90, 210, 255);            // diffuse RGB

    // ── Camera 0: Perspective, looking toward +Z (identity rotation) ────
    {
        CameraSlot& cam = scene->cameras[0];
        cam.active       = true;
        cam.layoutId     = 0;
        cam.position     = { 0.0f, 0.0f, -5.0f };
        cam.rotation     = { 1.0f, 0.0f, 0.0f, 0.0f };   // identity quat
        cam.baseRotation = { 1.0f, 0.0f, 0.0f, 0.0f };
        cam.scale        = { 1.0f, 1.0f, 1.0f };
        cam.is2D         = false;
    }

    printf("[HeadlessSelfTest] Scene built: 2 meshes (cube + teapot), "
           "2 light materials, 1 camera\n");
    printf("[HeadlessSelfTest] Teapot: %u verts, %u faces\n",
           kTeapotVertCount, kTeapotFaceCount);
    printf("[HeadlessSelfTest] Cube shown for first %.0f s, then teapot\n",
           kSwitchTimeS);
}

// ─── Draw List Update ───────────────────────────────────────────────────────

/// Helper: make a PglTransform with given position, scale, and rotation.
static PglTransform MakeTransform(PglVec3 pos, PglVec3 scl, PglQuat rot) {
    PglTransform t;
    std::memset(&t, 0, sizeof(t));
    t.position            = pos;
    t.scale               = scl;
    t.rotation            = rot;
    t.baseRotation        = { 1.0f, 0.0f, 0.0f, 0.0f };
    t.scaleRotationOffset = { 1.0f, 0.0f, 0.0f, 0.0f };
    return t;
}

void HeadlessSelfTest::UpdateDrawList(SceneState* scene,
                                      float cubeAngleY, float cubeAngleX,
                                      float elapsedS) {
    // Reset draw list for this frame (preserves meshes/materials/camera)
    scene->drawCallCount = 0;
    for (uint8_t i = 0; i < GpuConfig::MAX_DRAW_CALLS; ++i) {
        scene->drawList[i] = DrawCall{};
    }

    // ── Choose mesh based on elapsed time ───────────────────────────────
    // NOTE: elapsedTimeS is derived from the RP2350 hardware timer which
    // runs from the 1 MHz reference clock, NOT clk_sys.  Overclocking to
    // 360 MHz has zero effect on this timing — no correction needed.
    const bool showTeapot = (elapsedS >= kSwitchTimeS);

    DrawCall& dc = scene->drawList[0];
    dc.enabled = true;

    if (!showTeapot) {
        // ── Phase 1 (0–10 s): Rotating cube, warm light shading ────────
        dc.meshId     = 0;   // cube
        dc.materialId = 0;   // warm directional light

        float halfY = cubeAngleY * 0.5f;
        float halfX = cubeAngleX * 0.5f;
        float halfZ = (cubeAngleY * 0.3f) * 0.5f;

        PglQuat qy = { cosf(halfY), 0.0f, sinf(halfY), 0.0f };
        PglQuat qx = { cosf(halfX), sinf(halfX), 0.0f, 0.0f };
        PglQuat qz = { cosf(halfZ), 0.0f, 0.0f, sinf(halfZ) };
        PglQuat rot = PglMath::QuatMul(PglMath::QuatMul(qy, qx), qz);

        dc.transform = MakeTransform(
            { 0.0f, 0.0f, 0.0f },
            { 3.5f, 3.5f, 3.5f },
            rot
        );
    } else {
        // ── Phase 2 (10 s+): Rotating Utah teapot, cool light shading ──
        dc.meshId     = 1;   // teapot
        dc.materialId = 1;   // cool directional light

        // Slower, stately rotation for the teapot
        float tAngle = (elapsedS - kSwitchTimeS) * 0.6f;  // ~0.6 rad/s on Y
        float halfY  = tAngle * 0.5f;
        float halfX  = tAngle * 0.15f;  // gentle tilt

        PglQuat qy = { cosf(halfY), 0.0f, sinf(halfY), 0.0f };
        PglQuat qx = { cosf(halfX), sinf(halfX), 0.0f, 0.0f };
        PglQuat rot = PglMath::QuatMul(qy, qx);

        dc.transform = MakeTransform(
            { 0.0f, 0.0f, 0.0f },
            { 0.8f, 0.8f, 0.8f },
            rot
        );
    }

    scene->drawCallCount = 1;

    // Print scene switch once
    static bool switchPrinted = false;
    if (showTeapot && !switchPrinted) {
        printf("[HeadlessSelfTest] Switched to Utah teapot at %.1f s\n", elapsedS);
        switchPrinted = true;
    }
}

// ─── HUD Overlay ────────────────────────────────────────────────────────────

void HeadlessSelfTest::DrawHUD(uint16_t* fb, uint16_t w, uint16_t h,
                               uint16_t fps, uint16_t cpuPct) {
    // FPS — top-left, yellow
    {
        char fpsStr[16];
        IntToStr(fps, fpsStr, sizeof(fpsStr));
        uint8_t len = 0;
        while (fpsStr[len]) ++len;
        if (len + 4 < (uint8_t)sizeof(fpsStr)) {
            fpsStr[len++] = 'F';
            fpsStr[len++] = 'P';
            fpsStr[len++] = 'S';
            fpsStr[len]   = '\0';
        }
        DrawStringTo(fb, w, h, 1, 1, fpsStr, Rgb565(255, 255, 100));
    }

    // CPU% — top-right, cyan
    {
        char usageStr[16];
        IntToStr(cpuPct, usageStr, sizeof(usageStr));
        uint8_t len = 0;
        while (usageStr[len]) ++len;
        if (len + 1 < (uint8_t)sizeof(usageStr)) {
            usageStr[len++] = '%';
            usageStr[len]   = '\0';
        }
        DrawStringRightTo(fb, w, h, w - 1, 1, usageStr, Rgb565(100, 255, 255));
    }
}

#endif // RP2350GPU_HEADLESS_SELFTEST
