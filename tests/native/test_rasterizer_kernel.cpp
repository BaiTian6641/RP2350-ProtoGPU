/**
 * @file test_rasterizer_kernel.cpp
 * @brief Native deterministic kernel tests for the protocol-9 rasterizer.
 *
 * Builds the REAL SceneState + Rasterizer + Triangle2D + PglMath against the
 * ProtoGC desktop backend (no parser, no Pico SDK) and drives scenes by
 * filling scene slots directly.  Every case compares rendered pixels against
 * an independently computed reference for consumer-visible faults:
 *
 *   A  dense >128-triangle tile overlap — lossless coverage (the old
 *      QuadTree/capped-128 candidate path silently dropped geometry) plus
 *      translucent submission-order source-over against a computed blend
 *      sequence
 *   B  projected-pool overflow via near-clip expansion — explicit
 *      RenderOverflow frame error, no silent drop, no OOB
 *   C1 near-plane clip — crossing triangle rasterizes the clipped band only
 *   C2 morph bounds — visible overridden geometry is not culled from the
 *      base mesh AABB
 *   C3 near-crossing AABB admission — mixed (rotated) draws whose front
 *      corners project off-screen but whose content lands on-screen are not
 *      culled (numerically searched counterexample to the old rule)
 *   D  foreshortened UV — perspective-correct texture lookup vs analytic
 *      reference (every probe also discriminates against affine)
 *   E  intersecting depth — two interpenetrating triangles resolve per-pixel
 *      against the true perspective depth reference (probes discriminate
 *      against affine interpolation)
 *   F  shared edge — top-left coverage rule: no cracks, no double-owned
 *      diagonal pixels (decisive submission order)
 *   G  stacked alpha / zero alpha / mask discard — no transparent depth
 *      writes, alpha0 identity, failed mask never occludes
 *   H  Combine blend saturation + opacity endpoints + nested handle
 *      (generation|index) extraction
 *   I  target stride/extents + camera slot ordering + per-pass scissor
 *   J  frame error surface — Ok default, invalid tile grid error + lossless
 *      fallback, lookOffset and camera scale are applied (not ignored)
 *   K  service callbacks fire at bounded slices (exact counts)
 *   L  no camera — tile pass clears to black
 *   M  declared orthographic convention — pixel-space mapping, z<=0 drop
 *
 * Reference math notes: perspective probes are computed analytically in
 * double precision with margins ≫ float32 noise; blend references replicate
 * the declared BlendAlphaRGB565 quantisation contract step by step.
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>

#include "scene_state.h"
#include "render/rasterizer.h"

// ─── Test Framework ─────────────────────────────────────────────────────────

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond, msg) do { \
    ++g_checks; \
    if (!(cond)) { \
        std::printf("FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg); \
        ++g_failures; \
    } \
} while (0)

// ─── Shared Fixtures ────────────────────────────────────────────────────────

static constexpr uint16_t W = GpuConfig::PANEL_WIDTH;    // 128
static constexpr uint16_t H = GpuConfig::PANEL_HEIGHT;   // 64
static constexpr uint16_t TILE = 16;

static SceneState g_scene;
static PhaseScratch::DepthWorkspace g_depth;
static uint16_t   g_fb[GpuConfig::FRAMEBUF_PIXELS];
static Rasterizer g_ras;

static uint16_t PackRef(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

// Mirror of the declared BlendAlphaRGB565 contract (reference oracle).
static uint16_t BlendRef(uint16_t src, uint16_t dst, float alpha) {
    if (!(alpha > 0.0f)) return dst;
    if (alpha >= 1.0f)   return src;
    const uint8_t sr = static_cast<uint8_t>(((src >> 11) & 0x1F) << 3);
    const uint8_t sg = static_cast<uint8_t>(((src >> 5)  & 0x3F) << 2);
    const uint8_t sb = static_cast<uint8_t>((src & 0x1F) << 3);
    const uint8_t dr = static_cast<uint8_t>(((dst >> 11) & 0x1F) << 3);
    const uint8_t dg = static_cast<uint8_t>(((dst >> 5)  & 0x3F) << 2);
    const uint8_t db = static_cast<uint8_t>((dst & 0x1F) << 3);
    const float inv = 1.0f - alpha;
    const uint8_t r = static_cast<uint8_t>(sr * alpha + dr * inv);
    const uint8_t g = static_cast<uint8_t>(sg * alpha + dg * inv);
    const uint8_t b = static_cast<uint8_t>(sb * alpha + db * inv);
    return PackRef(r, g, b);
}

static PglTransform IdentityTransform() {
    PglTransform t{};
    t.position            = {0, 0, 0};
    t.rotation            = {1, 0, 0, 0};
    t.scale               = {1, 1, 1};
    t.baseRotation        = {1, 0, 0, 0};
    t.scaleRotationOffset = {1, 0, 0, 0};
    t.scaleOffset         = {0, 0, 0};
    t.rotationOffset      = {0, 0, 0};
    return t;
}

static void FreshScene() {
    g_scene.Reset();
    g_ras.Initialize(&g_scene, g_depth, W, H);
}

static void AddMesh(uint16_t slot, const PglVec3* verts, uint16_t nv,
                    const PglIndex3* tris, uint16_t nt,
                    const PglVec2* uvs = nullptr,
                    const PglIndex3* uvTris = nullptr, uint16_t nuv = 0) {
    MeshSlot& m = g_scene.meshes[slot];
    m.vertices = g_scene.AllocVertices(nv);
    m.indices  = g_scene.AllocIndices(nt);
    std::memcpy(m.vertices, verts, nv * sizeof(PglVec3));
    std::memcpy(m.indices,  tris,  nt * sizeof(PglIndex3));
    if (uvs && uvTris) {
        m.uvVertices = g_scene.AllocUVVertices(nuv);
        m.uvIndices  = g_scene.AllocUVIndices(nt);
        std::memcpy(m.uvVertices, uvs,    nuv * sizeof(PglVec2));
        std::memcpy(m.uvIndices,  uvTris, nt * sizeof(PglIndex3));
        m.uvVertexCount = nuv;
    }
    m.vertexCount   = nv;
    m.triangleCount = nt;
    m.active = true;
    m.RecomputeAABB();
}

static void AddSimpleMaterial(uint16_t slot, uint8_t r, uint8_t g, uint8_t b,
                              PglBlendMode blend = PGL_BLEND_BASE,
                              float alpha = 1.0f) {
    MaterialSlot& m = g_scene.materials[slot];
    m = {};
    m.active = true;
    m.type = PGL_MAT_SIMPLE;
    m.blendMode = blend;
    m.alpha = alpha;
    const PglParamSimple p{r, g, b};
    std::memcpy(m.params, &p, sizeof(p));
    m.paramBytes = sizeof(p);
}

static void AddCamera(uint8_t slot, bool is2D = false) {
    CameraSlot& c = g_scene.cameras[slot];
    c = {};
    c.active = true;
    c.position     = {0, 0, 0};
    c.rotation     = {1, 0, 0, 0};
    c.scale        = {1, 1, 1};
    c.lookOffset   = {1, 0, 0, 0};
    c.baseRotation = {1, 0, 0, 0};
    c.is2D = is2D;
}

static DrawCall& AddDraw(uint16_t meshId, uint16_t matId) {
    DrawCall& dc = g_scene.drawList[g_scene.drawCallCount++];
    dc = {};
    dc.enabled = true;
    dc.meshId = meshId;
    dc.materialId = matId;
    dc.transform = IdentityTransform();
    return dc;
}

static void RunAllTiles(Rasterizer& ras, uint16_t* fb, void (*service)() = nullptr) {
    for (uint16_t ty = 0; ty < (H + TILE - 1) / TILE; ++ty)
        for (uint16_t tx = 0; tx < (W + TILE - 1) / TILE; ++tx)
            ras.RasterizeTile(fb, g_depth.DepthPixels(), tx, ty, TILE, TILE, service);
}

static uint16_t FB(uint16_t x, uint16_t y) { return g_fb[y * W + x]; }

// Whole-screen triangle (CCW in y-down screen space) at depth z.
static void WholeScreenTri(PglVec3 out[3], float z) {
    out[0] = {-100.0f, -100.0f, z};
    out[1] = { 500.0f, -100.0f, z};
    out[2] = {-100.0f,  500.0f, z};
}

// ─── A: dense >128 overlap — lossless coverage + submission-order alpha ─────

static void TestDenseOverlap() {
    FreshScene();
    PglVec3 big[3];

    WholeScreenTri(big, 5.0f);
    const PglIndex3 one{0, 1, 2};
    AddMesh(0, big, 3, &one, 1);                       // opaque black backstop
    AddSimpleMaterial(0, 0, 0, 0);

    PglVec3 white[3 * 160];
    PglIndex3 whiteTris[160];
    WholeScreenTri(big, 4.9f);
    for (int i = 0; i < 160; ++i) {
        white[i * 3 + 0] = big[0];
        white[i * 3 + 1] = big[1];
        white[i * 3 + 2] = big[2];
        whiteTris[i] = {static_cast<uint16_t>(i * 3),
                        static_cast<uint16_t>(i * 3 + 1),
                        static_cast<uint16_t>(i * 3 + 2)};
    }
    // Keep all 160 layers/480 vertices, but give the tail a contrasting
    // source.  Repeated white alpha=.03 reaches an RGB565 fixed point well
    // before layer 128, so a uniform-colour stack cannot detect truncation.
    AddMesh(1, white, 3 * 128, whiteTris, 128);
    AddSimpleMaterial(1, 255, 255, 255, PGL_BLEND_ALPHA, 0.03f);
    AddMesh(2, white + 3 * 128, 3 * 32, whiteTris, 32);
    AddSimpleMaterial(2, 0, 0, 255, PGL_BLEND_ALPHA, 0.5f);

    // Last-submitted opaque triangle covering only the corner of tile (0,0).
    const PglVec3 corner[3] = {
        {-2.0f, -0.9f, 2.0f}, {-1.4f, -0.9f, 2.0f}, {-1.7f, -0.5f, 2.0f}};
    AddMesh(3, corner, 3, &one, 1);
    AddSimpleMaterial(3, 0, 255, 0);

    AddDraw(0, 0);
    AddDraw(1, 1);
    AddDraw(2, 2);
    AddDraw(3, 3);
    AddCamera(0);

    g_ras.PrepareFrame(&g_scene);
    CHECK(g_ras.GetFrameError() == PglRuntime::Result::Ok, "A: frame error free");
    RunAllTiles(g_ras, g_fb);

    // Independent quantized source-over sequence: 128 white layers,
    // followed by 32 blue layers.  A capped prefix keeps its fixed-point
    // colour, while the blue tail also detects reversed translucent order.
    uint16_t ref128 = 0x0000;
    for (int i = 0; i < 128; ++i) ref128 = BlendRef(0xFFFF, ref128, 0.03f);
    uint16_t ref = ref128;
    for (int i = 0; i < 32; ++i) ref = BlendRef(0x001F, ref, 0.5f);
    uint16_t reversed = 0x0000;
    for (int i = 0; i < 32; ++i) reversed = BlendRef(0x001F, reversed, 0.5f);
    for (int i = 0; i < 128; ++i) reversed = BlendRef(0xFFFF, reversed, 0.03f);
    CHECK(ref != ref128, "A: contrasting tail detects truncation at 128");
    CHECK(ref != reversed, "A: reference detects reversed alpha order");
    CHECK(FB(64, 32) == ref, "A: 160-layer translucent stack matches reference");
    // The late-submitted corner triangle was visited (its tile saw >128
    // candidates) and won its pixel.
    CHECK(FB(9, 7) == PackRef(0, 255, 0), "A: last triangle not truncated from tile");
}

// ─── B: projected-pool overflow via clip expansion ──────────────────────────

static void TestPoolOverflow() {
    FreshScene();
    // Each source triangle crosses the near plane with 2 verts in front →
    // the clip fans out 2 projected triangles.  700 × 2 = 1400 > 1280 pool.
    PglVec3 verts[3] = {
        {0.0f, 0.0f,  1.0f},
        {0.5f, 0.0f,  1.0f},
        {0.0f, 0.5f, -1.0f}};
    PglIndex3 tris[700];
    for (int i = 0; i < 700; ++i) tris[i] = {0, 1, 2};  // shared verts
    AddMesh(0, verts, 3, tris, 700);
    AddSimpleMaterial(0, 255, 0, 0);
    AddDraw(0, 0);
    AddCamera(0);

    g_ras.PrepareFrame(&g_scene);
    CHECK(g_ras.GetFrameError() == PglRuntime::Result::RenderOverflow,
          "B: pool overflow is an explicit frame error");
    CHECK(g_ras.GetTriangleCount() == 700, "B: source triangle count");
    RunAllTiles(g_ras, g_fb);   // must not corrupt memory (ASan) or crash
    uint32_t colored = 0;
    for (uint32_t i = 0; i < W * H; ++i) colored += (g_fb[i] != 0);
    CHECK(colored > 0, "B: pooled prefix still renders");

    // Exactly 640 clipped sources fill all 1280 projected slots.  Filling
    // the last slot is not itself overflow; a later visible draw is.
    FreshScene();
    AddMesh(0, verts, 3, tris, 640);
    AddSimpleMaterial(0, 255, 0, 0);
    AddDraw(0, 0);
    AddCamera(0);
    g_ras.PrepareFrame(&g_scene);
    CHECK(g_ras.GetFrameError() == PglRuntime::Result::Ok,
          "B: exactly full projected pool is admitted");
    PglVec3 later[3];
    WholeScreenTri(later, 2.0f);
    const PglIndex3 one{0, 1, 2};
    AddMesh(1, later, 3, &one, 1);
    AddSimpleMaterial(1, 0, 255, 0);
    AddDraw(1, 1);
    g_ras.PrepareFrame(&g_scene);
    CHECK(g_ras.GetFrameError() == PglRuntime::Result::RenderOverflow,
          "B: visible draw after exact-capacity draw is explicit overflow");
    CHECK(g_ras.GetTriangleCount() == 641,
          "B: diagnostic count includes later overflowing source draw");
}

// ─── C1: near-plane clip band ───────────────────────────────────────────────

static void TestNearClip() {
    FreshScene();
    // The original behind vertex y=.25 made y=-.25*z everywhere: its
    // projection was the line y=16, not a band.  This plane instead obeys
    // y=.000125-.250125*z, so sy=15.992+.008/z.  Near z=.001
    // caps the band at sy≈23.992; the front edge is exactly sy=16.
    const PglVec3 verts[3] = {
        {-0.5f, -0.25f,  1.0f},
        { 0.5f, -0.25f,  1.0f},
        { 0.0f,  0.25025f, -1.0f}};
    const PglIndex3 one{0, 1, 2};
    AddMesh(0, verts, 3, &one, 1);
    AddSimpleMaterial(0, 255, 0, 0);
    AddDraw(0, 0);
    AddCamera(0);

    g_ras.PrepareFrame(&g_scene);
    CHECK(g_ras.GetFrameError() == PglRuntime::Result::Ok, "C1: frame error free");
    RunAllTiles(g_ras, g_fb);

    CHECK(FB(64, 20) == PackRef(255, 0, 0), "C1: clipped band covered (centre)");
    CHECK(FB(8, 20)  == PackRef(255, 0, 0), "C1: clipped band spans full width");
    CHECK(FB(120, 20) == PackRef(255, 0, 0), "C1: clipped band right edge");
    CHECK(FB(64, 28) == 0x0000, "C1: below clip band stays black");
    CHECK(FB(64, 8)  == 0x0000, "C1: above clip band stays black");
    CHECK(FB(64, 23) == PackRef(255, 0, 0),
          "C1: last pixel centre before analytic near cap is covered");
    CHECK(FB(64, 24) == 0,
          "C1: first pixel centre beyond analytic near cap is excluded");
}

// ─── C2: morph override bounds ──────────────────────────────────────────────

static void TestMorphBounds() {
    FreshScene();
    // Base mesh far off-screen (old code culled from the base AABB); the
    // morph override puts the triangle on-screen.
    const PglVec3 base[3] = {
        {1000.0f, 0.0f, 2.0f}, {1000.5f, 0.0f, 2.0f}, {1000.0f, 0.5f, 2.0f}};
    const PglIndex3 one{0, 1, 2};
    AddMesh(0, base, 3, &one, 1);
    AddSimpleMaterial(0, 0, 0, 255);
    static const PglVec3 overrideVerts[3] = {
        {-0.5f, -0.5f, 2.0f}, {0.5f, -0.5f, 2.0f}, {0.0f, 0.5f, 2.0f}};
    DrawCall& dc = AddDraw(0, 0);
    dc.hasVertexOverride = true;
    dc.overrideVertexCount = 3;
    dc.overrideVertices = const_cast<PglVec3*>(overrideVerts);
    AddCamera(0);

    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);
    CHECK(FB(64, 26) == PackRef(0, 0, 255),
          "C2: visible morphed geometry not culled from base bounds");
}

// ─── C3: near-crossing AABB admission (rotated draw) ────────────────────────
// Numerically searched counterexample to the old "cull mixed draws from the
// in-front corner projections" rule: the mesh AABB (object space) has 2 of 8
// corners in front, BOTH projecting to sx ≈ 154 (off-screen right — the old
// rule culled the whole draw), yet the triangle inside lands on-screen.

static void TestNearCrossingAABB() {
    FreshScene();
    const PglVec3 verts[5] = {
        { 0.281025f, -0.247673f,  0.325469f},   // triangle (all front)
        { 0.539602f,  1.550117f,  0.296725f},
        {-0.336621f, -0.420588f,  0.299106f},
        {-2.091045f, -0.781661f, -2.744731f},   // AABB anchors (define the box)
        { 1.253953f,  1.818664f,  0.336002f},
    };
    const PglIndex3 one{0, 1, 2};
    AddMesh(0, verts, 5, &one, 1);
    AddSimpleMaterial(0, 255, 0, 0);
    DrawCall& dc = AddDraw(0, 0);
    dc.transform.rotation = {0.9842406f, 0.0f, -0.1768347f, 0.0f};  // −20.37° about Y
    AddCamera(0);

    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);
    CHECK(FB(64, 32) == PackRef(255, 0, 0),
          "C3: mixed near-crossing draw is not culled from front-corner bounds");
}

// ─── D: foreshortened UV — perspective-correct texture lookup ───────────────

static void TestForeshortenedUV() {
    FreshScene();
    // Quad in the local z=0 plane, rotated 60° about Y, centred at (0,0,2).
    // u=(xl+1)/2, v=(yl+1)/2 across a 2×2 texture:
    //   texel(0,0)=red (1,0)=green (0,1)=blue (1,1)=white
    const PglVec3 verts[4] = {
        {-1.0f, -1.0f, 0.0f}, {1.0f, -1.0f, 0.0f},
        { 1.0f,  1.0f, 0.0f}, {-1.0f, 1.0f, 0.0f}};
    const PglIndex3 tris[2] = {{0, 1, 2}, {0, 2, 3}};
    const PglVec2 uvs[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    AddMesh(0, verts, 4, tris, 2, uvs, tris, 4);

    // 2×2 RGB565 texture, texel(x,y) at index y*2+x.
    static const uint16_t texels[4] = {0xF800, 0x07E0, 0x001F, 0xFFFF};
    TextureSlot& tex = g_scene.textures[3];
    tex = {};
    tex.active = true;
    tex.width = 2;
    tex.height = 2;
    tex.format = PGL_TEX_RGB565;
    tex.pixelDataSize = sizeof(texels);
    tex.pixels = g_scene.AllocTexturePixels(sizeof(texels));
    std::memcpy(tex.pixels, texels, sizeof(texels));

    // IMAGE material; the texture reference carries a generation byte
    // (handle = gen<<8 | index) — the renderer must extract the index.
    MaterialSlot& m = g_scene.materials[0];
    m = {};
    m.active = true;
    m.type = PGL_MAT_IMAGE;
    m.blendMode = PGL_BLEND_BASE;
    const PglParamImage ip{static_cast<uint16_t>((2u << 8) | 3u),
                           0.0f, 0.0f, 1.0f, 1.0f, 0, 0};
    std::memcpy(m.params, &ip, sizeof(ip));
    m.paramBytes = sizeof(ip);

    DrawCall& dc = AddDraw(0, 0);
    dc.transform.rotation = {0.8660254f, 0.0f, 0.5f, 0.0f};  // 60° about Y
    dc.transform.position = {0.0f, 0.0f, 2.0f};
    AddCamera(0);

    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);

    // Analytic perspective reference (see header): every probe's AFFINE
    // prediction is a DIFFERENT texel, so these assert true perspective
    // correction, not affine interpolation.
    CHECK(FB(70, 32) == 0xFFFF, "D: probe (70,32) → texel(1,1) white");
    CHECK(FB(66, 20) == 0x07E0, "D: probe (66,20) → texel(1,0) green");
    CHECK(FB(70, 40) == 0xFFFF, "D: probe (70,40) → texel(1,1) white");
    CHECK(FB(60, 32) == 0x001F, "D: probe (60,32) → texel(0,1) blue");
    CHECK(FB(64, 32) == 0xFFFF, "D: probe (64,32) → texel(1,1) white (affine: red)");
}

// ─── E: intersecting depth ──────────────────────────────────────────────────

static void TestIntersectingDepth() {
    FreshScene();
    // T1: plane z=2 (red).  T2: plane z = 2 + y_w (green) — the planes
    // intersect along y_w = 0 (screen row 31.5).  With perspective-correct
    // depth T2 wins rows ≤ 30; AFFINE depth scores those same rows for T1
    // (verified numerically — the probes discriminate).
    const PglVec3 t1[3] = {
        {-1.5f, -1.0f, 2.0f}, {1.5f, -1.0f, 2.0f}, {0.0f, 1.5f, 2.0f}};
    const PglVec3 t2[3] = {
        {-1.5f, -0.75f, 1.25f}, {1.5f, -0.75f, 1.25f}, {0.0f, 1.0f, 3.0f}};
    const PglIndex3 one{0, 1, 2};
    AddMesh(0, t1, 3, &one, 1);
    AddMesh(1, t2, 3, &one, 1);
    AddSimpleMaterial(0, 255, 0, 0);
    AddSimpleMaterial(1, 0, 255, 0);
    AddDraw(0, 0);   // red first
    AddDraw(1, 1);   // green second
    AddCamera(0);

    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);

    CHECK(FB(64, 26) == PackRef(0, 255, 0), "E: row 26 — tilted plane wins (perspective)");
    CHECK(FB(64, 30) == PackRef(0, 255, 0), "E: row 30 — tilted plane wins (perspective)");
    CHECK(FB(64, 36) == PackRef(255, 0, 0), "E: row 36 — flat plane wins");
}

// ─── F: shared edge — top-left coverage ─────────────────────────────────────

static void TestSharedEdge() {
    FreshScene();
    // Quad [32.5,80.5]×[16.5,48.5] at z=2.  Its diagonal has slope
    // 32/48=2/3, NOT 1: 3*(py-16.5)=2*(px-32.5).  The directed
    // C→A edge of red ABC goes upward and is inclusive; green ACD's
    // A→C goes downward and is exclusive.  GREEN IS SUBMITTED FIRST:
    // double ownership would give green on the strict-depth tie.
    const PglVec3 t1[3] = {
        {-0.984375f, -0.484375f, 2.0f},   // A (32.5, 16.5)
        { 0.515625f, -0.484375f, 2.0f},   // B (80.5, 16.5)
        { 0.515625f,  0.515625f, 2.0f}};  // C (80.5, 48.5)
    const PglVec3 t2[3] = {
        {-0.984375f, -0.484375f, 2.0f},   // A
        { 0.515625f,  0.515625f, 2.0f},   // C
        {-0.984375f,  0.515625f, 2.0f}};  // D (32.5, 48.5)
    const PglIndex3 one{0, 1, 2};
    AddMesh(0, t1, 3, &one, 1);
    AddMesh(1, t2, 3, &one, 1);
    AddSimpleMaterial(0, 255, 0, 0);
    AddSimpleMaterial(1, 0, 255, 0);
    AddDraw(1, 1);   // green FIRST
    AddDraw(0, 0);   // red second
    AddCamera(0);

    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);

    const uint16_t RED = PackRef(255, 0, 0), GREEN = PackRef(0, 255, 0);
    bool ok = true;
    for (uint16_t y = 16; y <= 47; ++y) {
        for (uint16_t x = 32; x <= 79; ++x) {
            const uint16_t c = FB(x, y);
            const int rel = 3 * (static_cast<int>(y) - 16) -
                            2 * (static_cast<int>(x) - 32);
            const uint16_t want = (rel <= 0) ? RED : GREEN;
            if (c != want) ok = false;
        }
    }
    CHECK(ok, "F: every quad pixel owned exactly once (no crack, no double edge)");
    CHECK(FB(47, 26) == RED, "F: exact 2/3 diagonal belongs to top-left owner");

    // With no transparent depth writes, double-owned shared edges would
    // visibly blend twice.  Uniform half-alpha red must blend exactly once
    // on every quad sample, including all exact diagonal pixel centres.
    AddSimpleMaterial(0, 255, 0, 0, PGL_BLEND_ALPHA, 0.5f);
    AddSimpleMaterial(1, 255, 0, 0, PGL_BLEND_ALPHA, 0.5f);
    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);
    bool oneBlend = true;
    for (uint16_t y = 16; y <= 47; ++y)
        for (uint16_t x = 32; x <= 79; ++x)
            oneBlend &= FB(x, y) == BlendRef(RED, 0, 0.5f);
    CHECK(oneBlend, "F: translucent quad has neither cracks nor double blends");
    CHECK(FB(80, 32) == 0 && FB(47, 48) == 0,
          "F: quad right and bottom edges are exclusive");
}

// ─── G: alpha semantics ─────────────────────────────────────────────────────

static void TestAlphaZeroNoOcclude() {
    FreshScene();
    PglVec3 big[3];
    const PglIndex3 one{0, 1, 2};
    WholeScreenTri(big, 3.0f);
    AddMesh(0, big, 3, &one, 1);                        // opaque red back
    WholeScreenTri(big, 1.0f);
    AddMesh(1, big, 3, &one, 1);                        // alpha=0 front
    WholeScreenTri(big, 2.0f);
    AddMesh(2, big, 3, &one, 1);                        // alpha=0.5 middle
    AddSimpleMaterial(0, 255, 0, 0);
    AddSimpleMaterial(1, 255, 255, 255, PGL_BLEND_ALPHA, 0.0f);
    AddSimpleMaterial(2, 0, 255, 0, PGL_BLEND_ALPHA, 0.5f);
    AddDraw(0, 0);
    AddDraw(1, 1);
    AddDraw(2, 2);
    AddCamera(0);

    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);
    // alpha=0 must be an exact identity that writes NO depth: the later
    // alpha=0.5 green still blends over red (old code's transparent depth
    // write suppressed it → plain red).
    CHECK(FB(64, 32) == BlendRef(PackRef(0, 255, 0), PackRef(255, 0, 0), 0.5f),
          "G1: alpha0 writes no depth, later translucent blends in order");
}

static void TestAlphaOneEqualsOpaque() {
    FreshScene();
    PglVec3 big[3];
    const PglIndex3 one{0, 1, 2};
    WholeScreenTri(big, 3.0f);
    AddMesh(0, big, 3, &one, 1);
    WholeScreenTri(big, 1.0f);
    AddMesh(1, big, 3, &one, 1);
    AddSimpleMaterial(0, 255, 0, 0);
    AddSimpleMaterial(1, 0, 255, 0, PGL_BLEND_ALPHA, 1.0f);
    AddDraw(0, 0);
    AddDraw(1, 1);
    AddCamera(0);

    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);
    CHECK(FB(64, 32) == PackRef(0, 255, 0), "G2: alpha=1 is bit-identical to opaque");
}

static void AddMaskMaterial(uint16_t slot, uint16_t baseHandle, uint16_t maskHandle,
                            float threshold) {
    MaterialSlot& m = g_scene.materials[slot];
    m = {};
    m.active = true;
    m.type = PGL_MAT_MASK;
    m.blendMode = PGL_BLEND_BASE;
    const PglParamMask p{baseHandle, maskHandle, threshold};
    std::memcpy(m.params, &p, sizeof(p));
    m.paramBytes = sizeof(p);
}

static void TestMaskDiscard() {
    PglVec3 big[3];
    const PglIndex3 one{0, 1, 2};

    // G3: failing mask — front triangle discards; red behind shows through.
    FreshScene();
    WholeScreenTri(big, 1.0f);
    AddMesh(0, big, 3, &one, 1);                        // front, MASK material
    WholeScreenTri(big, 2.0f);
    AddMesh(1, big, 3, &one, 1);                        // behind, red
    AddSimpleMaterial(0, 255, 255, 255);                // base (white)
    AddSimpleMaterial(1, 0, 0, 0);                      // mask (black → fails)
    AddMaskMaterial(2, (1u << 8) | 0u, (4u << 8) | 1u, 0.5f);  // gen|index handles
    AddSimpleMaterial(3, 255, 0, 0);
    AddDraw(0, 2);
    AddDraw(1, 3);
    AddCamera(0);
    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);
    CHECK(FB(64, 32) == PackRef(255, 0, 0),
          "G3: failed mask discards — no colour, no depth, no occlusion");

    // G4: passing mask — base renders and occludes.
    FreshScene();
    WholeScreenTri(big, 1.0f);
    AddMesh(0, big, 3, &one, 1);
    WholeScreenTri(big, 2.0f);
    AddMesh(1, big, 3, &one, 1);
    AddSimpleMaterial(0, 255, 255, 255);                // base
    AddSimpleMaterial(1, 255, 255, 255);                // mask (white → passes)
    AddMaskMaterial(2, 0, 1, 0.5f);
    AddSimpleMaterial(3, 255, 0, 0);
    AddDraw(0, 2);
    AddDraw(1, 3);
    AddCamera(0);
    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);
    CHECK(FB(64, 32) == PackRef(255, 255, 255),
          "G4: passing mask renders base and occludes");
}

// ─── H: Combine blend saturation, endpoints, nested handles ────────────────

static void TestBlendSaturation() {
    PglVec3 big[3];
    const PglIndex3 one{0, 1, 2};
    WholeScreenTri(big, 2.0f);

    auto runCombine = [&](uint8_t ar, uint8_t ag, uint8_t ab,
                          uint8_t br, uint8_t bg, uint8_t bb,
                          uint8_t mode, float opacity) {
        FreshScene();
        AddMesh(0, big, 3, &one, 1);
        AddSimpleMaterial(2, ar, ag, ab);               // slot 2 = A
        AddSimpleMaterial(3, br, bg, bb);               // slot 3 = B
        MaterialSlot& m = g_scene.materials[4];
        m = {};
        m.active = true;
        m.type = PGL_MAT_COMBINE;
        // Nested references carry generation bytes — extraction required.
        const PglParamCombine cp{static_cast<uint16_t>((3u << 8) | 2u),
                                 static_cast<uint16_t>((5u << 8) | 3u),
                                 mode, opacity};
        std::memcpy(m.params, &cp, sizeof(cp));
        m.paramBytes = sizeof(cp);
        AddDraw(0, 4);
        AddCamera(0);
        g_ras.PrepareFrame(&g_scene);
        RunAllTiles(g_ras, g_fb);
    };

    runCombine(200, 200, 200, 200, 200, 200, PGL_BLEND_ADD, 1.0f);
    CHECK(FB(64, 32) == 0xFFFF, "H: ADD saturates channels at 255 (opacity 1)");
    runCombine(200, 200, 200, 200, 200, 200, PGL_BLEND_ADD, 0.0f);
    CHECK(FB(64, 32) == PackRef(200, 200, 200), "H: opacity 0 → base A exactly");
    runCombine(50, 50, 50, 200, 200, 200, PGL_BLEND_SUBTRACT, 1.0f);
    CHECK(FB(64, 32) == 0x0000, "H: SUBTRACT saturates channels at 0");
}

// ─── I: target stride/extents + camera slot ordering ───────────────────────

static void TestCameraTargets() {
    FreshScene();
    PglVec3 big[3];
    const PglIndex3 one{0, 1, 2};

    WholeScreenTri(big, 2.0f);
    AddMesh(0, big, 3, &one, 1);   // red, cam0 → back buffer
    WholeScreenTri(big, 2.0f);
    AddMesh(1, big, 3, &one, 1);   // green, cam1 → layer scissor
    WholeScreenTri(big, 3.0f);
    AddMesh(2, big, 3, &one, 1);   // blue (FARTHER), cam2 → overlapping scissor
    AddSimpleMaterial(0, 255, 0, 0);
    AddSimpleMaterial(1, 0, 255, 0);
    AddSimpleMaterial(2, 0, 0, 255);
    AddDraw(0, 0);
    // All cameras render the same draw list; there is no per-camera draw
    // selection.  Separate the objects/cameras by 10000 world units so
    // each camera sees only its own triangle (the others are off-screen).
    AddDraw(1, 1).transform.position.x = 10000.0f;
    AddDraw(2, 2).transform.position.x = 20000.0f;

    // 64×64 layer target (stride 64 ≠ panel 128), canary-filled.
    LayerSlot& layer = g_scene.layers[1];
    layer = {};
    layer.active = true;
    layer.width = 64;
    layer.height = 64;
    CHECK(g_scene.AllocLayerFramebuffer(1), "I: layer FB allocated");
    for (uint32_t i = 0; i < 64u * 64u; ++i) layer.pixels[i] = 0xAAAA;

    AddCamera(0);                                   // back buffer
    AddCamera(1);
    g_scene.cameras[1].position.x = 10000.0f;
    g_scene.cameras[1].targetLayer = 1;
    g_scene.cameras[1].vpFlags = PGL_CAMERA_TARGET_SCISSOR;
    g_scene.cameras[1].vpX = 16; g_scene.cameras[1].vpY = 16;
    g_scene.cameras[1].vpW = 32; g_scene.cameras[1].vpH = 32;
    AddCamera(2);
    g_scene.cameras[2].position.x = 20000.0f;
    g_scene.cameras[2].targetLayer = 1;
    g_scene.cameras[2].vpFlags = PGL_CAMERA_TARGET_SCISSOR;
    g_scene.cameras[2].vpX = 40; g_scene.cameras[2].vpY = 40;
    g_scene.cameras[2].vpW = 16; g_scene.cameras[2].vpH = 16;

    g_ras.PrepareFrame(&g_scene);
    CHECK(g_ras.GetPreparedCameraIndex() == 0, "I: cam0 prepared first");
    RunAllTiles(g_ras, g_fb);
    CHECK(FB(64, 32) == PackRef(255, 0, 0), "I: back buffer shows cam0");

    // Slot order: cam1, then cam2.
    uint8_t order[4];
    uint8_t n = 0, camIdx = 0;
    while (g_ras.PrepareNextCameraPass(&g_scene, &camIdx) && n < 4) {
        order[n++] = camIdx;
        CameraTargetInfo ti = g_scene.ResolveCameraTarget(camIdx, g_fb, W, H);
        CHECK(ti.valid && ti.fb == layer.pixels, "I: layer target resolves");
        for (uint16_t ty = 0; ty < (H + TILE - 1) / TILE; ++ty)
            for (uint16_t tx = 0; tx < (W + TILE - 1) / TILE; ++tx)
                g_ras.RasterizeTile(ti.fb, g_depth.DepthPixels(), tx, ty, TILE, TILE, nullptr);
    }
    CHECK(n == 2 && order[0] == 1 && order[1] == 2, "I: cameras execute in slot order");

    const auto LP = [&](uint16_t x, uint16_t y) { return layer.pixels[y * 64 + x]; };
    CHECK(LP(32, 32) == PackRef(0, 255, 0), "I: cam1 scissor interior (stride 64)");
    CHECK(LP(16, 16) == PackRef(0, 255, 0), "I: scissor lower bound inclusive");
    CHECK(LP(15, 16) == 0xAAAA, "I: scissor clip left");
    CHECK(LP(48, 32) == 0xAAAA, "I: scissor exclusive upper bound");
    CHECK(LP(44, 44) == PackRef(0, 0, 255),
          "I: cam2 paints over cam1 in slot order (per-pass Z clear, declared)");
    CHECK(LP(56, 56) == 0xAAAA, "I: outside both scissors untouched");
    CHECK(LP(63, 0) == 0xAAAA, "I: layer row 0 beyond scissor untouched");
}

static void TestTargetExtents() {
    // Both targets use the full 8192-pixel workspace.  Projection remains
    // panel-space, but candidate culling, tile masks and Z addressing must
    // admit target pixels beyond the panel's width OR height.
    for (uint8_t axis = 0; axis < 2; ++axis) {
        FreshScene();
        LayerSlot& layer = g_scene.layers[1];
        layer.active = true;
        layer.width = axis == 0 ? 256 : 64;
        layer.height = axis == 0 ? 32 : 128;
        CHECK(g_scene.AllocLayerFramebuffer(1), "I: expanded target allocated");
        const PglVec3 wide[3] = {{100, -24, 2}, {156, -24, 2}, {100, -8, 2}};
        const PglVec3 tall[3] = {{-48, 48, 2}, {-16, 48, 2}, {-48, 80, 2}};
        const PglIndex3 one{0, 1, 2};
        AddMesh(0, axis == 0 ? wide : tall, 3, &one, 1);
        AddSimpleMaterial(0, 255, 0, 0);
        AddDraw(0, 0);
        AddCamera(0, true);
        g_scene.cameras[0].targetLayer = 1;
        g_ras.PrepareFrame(&g_scene);
        uint8_t camera = 255;
        CHECK(g_ras.PrepareNextCameraPass(&g_scene, &camera) && camera == 0,
              "I: expanded layer camera prepares");
        const auto target = g_scene.ResolveCameraTarget(camera, g_fb, W, H);
        for (uint16_t ty = 0; ty < (target.height + TILE - 1) / TILE; ++ty)
            for (uint16_t tx = 0; tx < (target.width + TILE - 1) / TILE; ++tx)
                g_ras.RasterizeTile(target.fb, g_depth.DepthPixels(), tx, ty, TILE, TILE);
        // Wide: (164,8),(220,8),(164,24); sample (180.5,12.5)
        // has barycentric sum 16.5/56+4.5/16 < 1.
        // Tall: (16,80),(48,80),(16,112); sample (24.5,88.5)
        // has barycentric sum (8.5+8.5)/32 < 1.
        const uint16_t x = axis == 0 ? 180 : 24;
        const uint16_t y = axis == 0 ? 12 : 88;
        CHECK(layer.pixels[uint32_t(y) * layer.width + x] == PackRef(255, 0, 0),
              "I: geometry beyond primary panel extent is not omitted");
        CHECK(g_ras.GetFrameError() == PglRuntime::Result::Ok,
              "I: expanded target uses an admissible lossless grid");
    }
}

// ─── J: frame error surface / camera fields ────────────────────────────────

static void TestInvalidGridError() {
    FreshScene();
    // 160×160 exceeds the real 8192-pixel target capacity as well as the
    // tile-grid limit.  It must be rejected, not used as a fallback oracle.
    static PhaseScratch::DepthWorkspace bigDepth;
    static uint16_t bigFb[160 * 160];
    Rasterizer ras2;
    ras2.Initialize(&g_scene, bigDepth, 160, 160);

    const PglVec3 verts[3] = {
        {0.0f, 0.0f, 2.0f}, {0.5f, 0.0f, 2.0f}, {0.0f, 0.5f, 2.0f}};
    const PglIndex3 one{0, 1, 2};
    AddMesh(0, verts, 3, &one, 1);
    AddSimpleMaterial(0, 255, 0, 0);
    AddDraw(0, 0);
    AddCamera(0);

    ras2.PrepareFrame(&g_scene);
    CHECK(ras2.GetFrameError() == PglRuntime::Result::InvalidValue,
          "J: unsupported tile grid is an explicit frame error");
    for (uint16_t ty = 0; ty < 10; ++ty)
        for (uint16_t tx = 0; tx < 10; ++tx)
            ras2.RasterizeTile(bigFb, bigDepth.DepthPixels(), tx, ty, TILE, TILE, nullptr);
    CHECK(bigFb[84 * 160 + 84] == 0,
          "J: oversized target is not admitted or painted");

    // An admissible 1600×5 target has 8000 pixels and the SAME 100 grid
    // cells as the rejected square.  This isolates mask-grid overflow from
    // target-capacity rejection, without reducing the 100-tile workload.
    static PhaseScratch::DepthWorkspace wideDepth;
    static uint16_t wideFb[1600 * 5];
    ras2.Initialize(&g_scene, wideDepth, 1600, 5);
    ras2.PrepareFrame(&g_scene);
    CHECK(ras2.GetFrameError() == PglRuntime::Result::InvalidValue,
          "J: admissible target with 100 grid cells reports invalid grid");
    for (uint16_t tx = 0; tx < 100; ++tx)
        ras2.RasterizeTile(wideFb, wideDepth.DepthPixels(), tx, 0, TILE, TILE, nullptr);
    // Projection: (800,2.5),(1000,2.5),(800,202.5).  Pixel centre
    // (804.5,3.5) is strictly inside; (799.5,3.5) is outside.
    CHECK(wideFb[3 * 1600 + 804] == PackRef(255, 0, 0),
          "J: analytic coverage fallback stays lossless");
    CHECK(wideFb[3 * 1600 + 799] == 0,
          "J: analytic fallback retains triangle boundary");
}

static void TestLookOffsetComposed() {
    FreshScene();
    // lookOffset = 180° about Y → the camera faces −Z.  The triangle at
    // world z=−2 renders; the one at z=+2 is now behind and must not.
    const PglVec3 back[3] = {
        {0.3f, 0.0f, -2.0f}, {-0.3f, 0.0f, -2.0f}, {0.0f, 0.6f, -2.0f}};
    const PglVec3 front[3] = {
        {0.7f, -0.2f, 2.0f}, {1.1f, -0.2f, 2.0f}, {0.9f, 0.2f, 2.0f}};
    const PglIndex3 one{0, 1, 2};
    AddMesh(0, back, 3, &one, 1);
    AddMesh(1, front, 3, &one, 1);
    AddSimpleMaterial(0, 255, 0, 0);
    AddSimpleMaterial(1, 0, 255, 0);
    AddDraw(0, 0);
    AddDraw(1, 1);
    AddCamera(0);
    g_scene.cameras[0].lookOffset = {0.0f, 0.0f, 1.0f, 0.0f};  // 180° about Y

    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);
    // View of the z=-2 triangle after the 180° offset: screen (54.4,32),
    // (73.6,32), (64,51.2) — front-facing, covering (64.5,40.5).
    CHECK(FB(64, 40) == PackRef(255, 0, 0),
          "J: lookOffset is composed into the view rotation");
    CHECK(FB(93, 32) == 0x0000, "J: geometry behind the offset view is culled");
}

static void TestCameraScaleApplied() {
    FreshScene();
    // Camera scale (2,1,1) doubles view-space x: the triangle lands at
    // sx ≈ 102, NOT at the unscaled sx ≈ 83.
    const PglVec3 verts[3] = {
        {0.25f, -0.1f, 1.0f}, {0.35f, -0.1f, 1.0f}, {0.30f, 0.1f, 1.0f}};
    const PglIndex3 one{0, 1, 2};
    AddMesh(0, verts, 3, &one, 1);
    AddSimpleMaterial(0, 255, 0, 0);
    AddDraw(0, 0);
    AddCamera(0);
    g_scene.cameras[0].scale = {2.0f, 1.0f, 1.0f};

    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);
    CHECK(FB(102, 29) == PackRef(255, 0, 0),
          "J: camera scale applied to view space");
    CHECK(FB(83, 27) == 0x0000, "J: unscaled position stays black");
}

// ─── K: service callbacks at bounded slices ────────────────────────────────

static int g_prepServiceCalls = 0;
static int g_tileServiceCalls = 0;
static void PrepService() { ++g_prepServiceCalls; }
static void TileService() { ++g_tileServiceCalls; }

static void TestServiceCallbacks() {
    FreshScene();
    PglVec3 big[3];
    const PglIndex3 one{0, 1, 2};
    WholeScreenTri(big, 5.0f);
    AddMesh(0, big, 3, &one, 1);
    WholeScreenTri(big, 4.9f);
    AddMesh(1, big, 3, &one, 1);
    const PglVec3 corner[3] = {
        {-2.0f, -0.9f, 2.0f}, {-1.4f, -0.9f, 2.0f}, {-1.7f, -0.5f, 2.0f}};
    AddMesh(2, corner, 3, &one, 1);
    AddSimpleMaterial(0, 0, 0, 0);
    AddSimpleMaterial(1, 255, 255, 255, PGL_BLEND_ALPHA, 0.03f);
    AddSimpleMaterial(2, 0, 255, 0);
    // 160 translucent triangles via repeated indices into 3 verts.
    PglIndex3 many[160];
    for (int i = 0; i < 160; ++i) many[i] = {0, 1, 2};
    g_scene.FreeIndices(g_scene.meshes[1].indices);
    g_scene.meshes[1].indices = g_scene.AllocIndices(160);
    std::memcpy(g_scene.meshes[1].indices, many, sizeof(many));
    g_scene.meshes[1].triangleCount = 160;
    AddDraw(0, 0);
    AddDraw(1, 1);
    AddDraw(2, 2);
    AddCamera(0);

    g_prepServiceCalls = 0;
    g_ras.SetServiceCallback(&PrepService);
    g_ras.PrepareFrame(&g_scene);
    CHECK(g_prepServiceCalls == 3, "K: preparation service per processed draw call");
    g_ras.SetServiceCallback(nullptr);

    // Tile (0,0): 1 clear-band call + 2 opaque triangles + 160 translucent.
    g_tileServiceCalls = 0;
    g_ras.RasterizeTile(g_fb, g_depth.DepthPixels(), 0, 0, TILE, TILE, &TileService);
    CHECK(g_tileServiceCalls == 1 + 2 + 160,
          "K: tile service at bounded clear/triangle slices");

    // nullptr service is a valid no-op path (core 1).
    g_ras.RasterizeTile(g_fb, g_depth.DepthPixels(), 1, 0, TILE, TILE, nullptr);
}

// ─── L: no camera clears to black ───────────────────────────────────────────

static void TestNoCameraClears() {
    FreshScene();
    PglVec3 big[3];
    const PglIndex3 one{0, 1, 2};
    WholeScreenTri(big, 2.0f);
    AddMesh(0, big, 3, &one, 1);
    AddSimpleMaterial(0, 255, 0, 0);
    AddDraw(0, 0);
    // NO camera.
    for (uint32_t i = 0; i < W * H; ++i) g_fb[i] = 0xAAAA;

    g_ras.PrepareFrame(&g_scene);
    CHECK(g_ras.GetPreparedCameraIndex() == -1, "L: no camera prepared");
    RunAllTiles(g_ras, g_fb);
    bool black = true;
    for (uint32_t i = 0; i < W * H; ++i) black &= (g_fb[i] == 0x0000);
    CHECK(black, "L: no-camera frame clears to black (no stale content)");
}

// ─── M: declared orthographic convention ────────────────────────────────────

static void TestOrthographic() {
    FreshScene();
    const PglVec3 verts[3] = {
        {0.0f, 0.0f, 1.0f}, {32.0f, 0.0f, 1.0f}, {0.0f, 32.0f, 1.0f}};
    const PglVec3 behind[3] = {
        {24.0f, -4.0f, -1.0f}, {36.0f, -4.0f, -1.0f}, {30.0f, 8.0f, -1.0f}};
    const PglIndex3 one{0, 1, 2};
    AddMesh(0, verts, 3, &one, 1);
    AddMesh(1, behind, 3, &one, 1);
    AddSimpleMaterial(0, 255, 0, 0);
    AddSimpleMaterial(1, 0, 255, 0);
    AddDraw(0, 0);
    AddDraw(1, 1);
    AddCamera(0, /*is2D=*/true);

    g_ras.PrepareFrame(&g_scene);
    RunAllTiles(g_ras, g_fb);
    // Pixel units, not normalized .5 units: (64,32),(96,32),(64,64).
    // (70.5,34.5) is inside.  (94.5,34.5) is outside the front
    // triangle (30.5+2.5>32), but inside the dropped z=-1 triangle.
    CHECK(FB(70, 34) == PackRef(255, 0, 0), "M: ortho pixel-space mapping");
    CHECK(FB(94, 34) == 0x0000, "M: z<=0 ortho triangle dropped whole");
}

// ─── main ───────────────────────────────────────────────────────────────────

int main() {
    if (!g_scene.InitSceneHeap()) {
        std::printf("FATAL: scene heap init failed\n");
        return 2;
    }
    g_scene.Reset();

    TestDenseOverlap();
    TestPoolOverflow();
    TestNearClip();
    TestMorphBounds();
    TestNearCrossingAABB();
    TestForeshortenedUV();
    TestIntersectingDepth();
    TestSharedEdge();
    TestAlphaZeroNoOcclude();
    TestAlphaOneEqualsOpaque();
    TestMaskDiscard();
    TestBlendSaturation();
    TestCameraTargets();
    TestTargetExtents();
    TestInvalidGridError();
    TestLookOffsetComposed();
    TestCameraScaleApplied();
    TestServiceCallbacks();
    TestNoCameraClears();
    TestOrthographic();

    std::printf("rasterizer kernel tests: %d checks, %d failures\n",
                g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
