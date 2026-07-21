/**
 * @file scene_state.h
 * @brief GPU-side scene resource tables, memory pools, and 2D layer state for RP2350.
 *
 * Holds all persistent resources (meshes, materials, textures, layouts) and
 * per-frame state (draw list, cameras, 2D layers, 2D draw commands).
 * Large data lives in the **scene heap** (ProtoGC `HeapAllocator`, true
 * random free — resolves the old bump-pool TODO(M5) at the bottom of this
 * file's history) so that slot metadata stays small (~40 B per mesh slot)
 * and arbitrary create/destroy order no longer fragments.
 *
 * Memory layout (approximate):
 *   Scene heap cap:       ~112 KB (was 106 KB of static pools + overhead)
 *   Pool (per-frame):      ~24 KB (static bump — hot path, stays static)
 *   Slot arrays:            ~40 KB
 *   2D layer FBs:           ~16 KB per layer (128×64 RGB565)
 *   Total scene state:    ~192 KB (with 1 active 2D layer)
 *
 * Framebuffers, Z-buffer, QuadTree, code, and stack use the rest of SRAM.
 *
 * M12 additions: LayerSlot, DrawCmd2D queue, defrag/persist status caches.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <PglTypes.h>
#include <PglShaderBytecode.h>
#include <HeapAllocator.h>
#include "gpu_config.h"

// ─── Typed Bump Pool ────────────────────────────────────────────────────────
// A simple arena allocator. Only the PER-FRAME pool remains a bump pool (by
// design: it resets wholesale every BeginFrame, so random free is pointless).
// Persistent resource data moved to the ProtoGC scene heap (2026-07-19) —
// the old TODO(M5) "true random free" is resolved there.

template <typename T, uint32_t Capacity>
struct Pool {
    T        data[Capacity];
    uint32_t used = 0;

    /// Allocate `count` contiguous elements.  Returns nullptr on OOM.
    T* Allocate(uint32_t count) {
        if (used + count > Capacity) return nullptr;
        T* ptr = &data[used];
        used += count;
        return ptr;
    }

    /// Free everything from `ptr` onward (stack-style — fine for the
    /// per-frame pool, which only ever shrinks/reset wholesale).
    void FreeFrom(T* ptr) {
        if (!ptr) return;
        uint32_t offset = static_cast<uint32_t>(ptr - data);
        if (offset < used) used = offset;
    }

    /// Reset to empty (all allocations invalidated).
    void Reset() { used = 0; }

    /// Elements still available.
    uint32_t Available() const { return Capacity - used; }

    /// Total capacity.
    static constexpr uint32_t Cap() { return Capacity; }
};

// ─── Resource Slot Structs ──────────────────────────────────────────────────
// Slots hold metadata + pointers into the shared pools.
// The command parser populates these; the rasterizer reads them.

struct MeshSlot {
    bool      active        = false;
    uint16_t  vertexCount   = 0;
    uint16_t  triangleCount = 0;
    uint16_t  uvVertexCount = 0;

    PglVec3*   vertices  = nullptr;  // → SceneState::sceneHeap
    PglIndex3* indices   = nullptr;  // → SceneState::sceneHeap
    PglVec2*   uvVertices = nullptr; // → SceneState::sceneHeap
    PglIndex3* uvIndices  = nullptr; // → SceneState::sceneHeap

    // Cached object-space AABB — computed on mesh creation/update.
    // Used for fast mesh-level frustum culling before per-triangle work.
    PglVec3 aabbMin = {0, 0, 0};
    PglVec3 aabbMax = {0, 0, 0};

    /// Recompute aabbMin/aabbMax from current vertex data.
    void RecomputeAABB() {
        if (!vertices || vertexCount == 0) return;
        PglVec3 mn = vertices[0], mx = vertices[0];
        for (uint16_t i = 1; i < vertexCount; ++i) {
            const PglVec3& v = vertices[i];
            if (v.x < mn.x) mn.x = v.x;
            if (v.y < mn.y) mn.y = v.y;
            if (v.z < mn.z) mn.z = v.z;
            if (v.x > mx.x) mx.x = v.x;
            if (v.y > mx.y) mx.y = v.y;
            if (v.z > mx.z) mx.z = v.z;
        }
        aabbMin = mn;
        aabbMax = mx;
    }
};

struct MaterialSlot {
    bool           active    = false;
    PglMaterialType type     = PGL_MAT_SIMPLE;
    PglBlendMode   blendMode = PGL_BLEND_BASE;
    uint8_t        params[60] = {};   // type-specific, copied verbatim from wire
                                      // (max: 7-stop gradient = 59 B)
};

struct TextureSlot {
    bool             active        = false;
    uint16_t         width         = 0;
    uint16_t         height        = 0;
    PglTextureFormat format        = PGL_TEX_RGB565;
    uint32_t         pixelDataSize = 0;
    uint8_t*         pixels        = nullptr;  // → SceneState::sceneHeap
};

struct PixelLayoutSlot {
    bool     active     = false;
    uint16_t pixelCount = 0;
    uint8_t  flags      = 0;

    PglRectLayoutData rectData = {};          // used if RECTANGULAR flag set
    PglVec2*          coords   = nullptr;     // → SceneState::sceneHeap
};

struct ShaderSlot {
    bool    active      = false;
    uint8_t shaderClass = 0;     // PglShaderClass
    float   intensity   = 0.0f;
    uint8_t params[20]  = {};    // class-specific, copied from PglCmdSetShader::params
    uint16_t programId  = 0;     // For PGL_SHADER_PROGRAM: index into shaderPrograms[]
};

// ─── Programmable Shader Program Storage ────────────────────────────────────
// Each loaded program holds its bytecode, constants, and current uniform values.
// Per-program SRAM: ~1.3 KB.  PGL_MAX_SHADER_PROGRAMS programs = ~21 KB.

struct ShaderProgram {
    bool     active          = false;
    uint16_t programId       = 0;
    uint8_t  uniformCount    = 0;
    uint8_t  constCount      = 0;
    uint16_t instrCount      = 0;
    uint8_t  flags           = 0;
    float    uniforms[PSB_MAX_UNIFORMS]      = {};   // Current uniform values (64 bytes)
    float    constants[PSB_MAX_CONSTANTS]     = {};   // Embedded constants (128 bytes)
    uint32_t instructions[PSB_MAX_INSTRUCTIONS] = {}; // Bytecode (1024 bytes)
    uint32_t uniformNameHashes[PSB_MAX_UNIFORMS] = {};// For name-based uniform lookup
    uint8_t  uniformTypes[PSB_MAX_UNIFORMS]  = {};    // Component counts per uniform
};

struct CameraSlot {
    bool    active   = false;
    uint8_t layoutId = 0;

    PglVec3 position      = {};
    PglQuat rotation      = {};
    PglVec3 scale         = {};
    PglQuat lookOffset    = {};
    PglQuat baseRotation  = {};
    bool    is2D          = false;

    // Screen-space post-processing shaders (applied in slot order after rasterization)
    ShaderSlot shaders[PGL_MAX_SHADERS_PER_CAMERA];
};

struct DrawCall {
    uint16_t     meshId     = 0;
    uint16_t     materialId = 0;
    bool         enabled    = false;

    PglTransform transform  = {};

    bool         hasVertexOverride  = false;
    uint16_t     overrideVertexCount = 0;
    PglVec3*     overrideVertices    = nullptr;  // → SceneState::frameVertexPool
};

// ─── 2D Layer State (M12) ──────────────────────────────────────────────────
// Each LayerSlot owns a framebuffer (128×64 RGB565 = 16 KB) that the 2D
// rasterizer draws into.  Layer 0 is reserved for the 3D pipeline's back
// buffer and is never allocated here.  Layers 1–7 are user-created via
// CMD_LAYER_CREATE.  After all draw commands execute, the compositor alpha-
// blends visible layers in ascending order onto the final display buffer.

struct LayerSlot {
    bool     active      = false;
    bool     visible     = true;
    bool     dirty       = false;     ///< Set true when any draw cmd targets this layer

    uint16_t width       = 0;
    uint16_t height      = 0;
    uint8_t  pixelFormat = 0;         ///< PglDisplayPixelFormat (usually RGB565)
    uint8_t  blendMode   = 0;         ///< PglLayerBlendMode
    uint8_t  opacity     = 255;       ///< 0 = fully transparent, 255 = fully opaque

    int16_t  offsetX     = 0;         ///< Compositing offset X (signed pixels)
    int16_t  offsetY     = 0;         ///< Compositing offset Y (signed pixels)

    uint16_t* pixels     = nullptr;   ///< Layer framebuffer (RGB565), heap-allocated
};

// ─── 2D Draw Command Queue (M12) ───────────────────────────────────────────
// During command parsing, 2D draw commands are deferred into a fixed-size queue
// instead of executing immediately.  gpu_core then processes the queue once per
// frame, routing each command to the Rasterizer2D.  This decouples parsing from
// rendering and lets us batch-sort by layer if needed.

enum DrawCmd2DType : uint8_t {
    DRAW_CMD_2D_RECT           = 0,
    DRAW_CMD_2D_LINE           = 1,
    DRAW_CMD_2D_CIRCLE         = 2,
    DRAW_CMD_2D_SPRITE         = 3,
    DRAW_CMD_2D_CLEAR          = 4,
    DRAW_CMD_2D_ROUNDED_RECT   = 5,
    DRAW_CMD_2D_ARC            = 6,
    DRAW_CMD_2D_TRIANGLE       = 7,
};

struct DrawCmd2D {
    DrawCmd2DType type;
    uint8_t       layerId;   ///< Explicit layer ID (set during enqueue, avoids fragile union peeking)

    union {
        PglCmdDrawRect2D       rect;          // 12 bytes
        PglCmdDrawLine2D       line;          // 11 bytes
        PglCmdDrawCircle2D     circle;        // 10 bytes
        PglCmdDrawSprite       sprite;        //  8 bytes
        PglCmdLayerClear       clear;         //  3 bytes
        PglCmdDrawRoundedRect  roundedRect;   // 14 bytes
        PglCmdDrawArc          arc;           // 13 bytes
        PglCmdDrawTriangle2D   triangle;      // 15 bytes  (largest member)
    };
};
// sizeof(DrawCmd2D) = 1 (type) + 1 (layerId) + 15 (largest union member) = 17 bytes
// With packing: compiler may pad to 17. 128 commands × 17 bytes ≈ 2.2 KB per-frame budget
static_assert(sizeof(DrawCmd2D) <= 20, "DrawCmd2D should be compact (≤20 bytes)");

// ─── Scene State ────────────────────────────────────────────────────────────
// The main GPU-side data structure.  Owned by gpu_core.cpp (static lifetime).

struct SceneState {

    // ── Resource slot arrays ────────────────────────────────────────────────
    MeshSlot        meshes     [GpuConfig::MAX_MESHES];
    MaterialSlot    materials  [GpuConfig::MAX_MATERIALS];
    TextureSlot     textures   [GpuConfig::MAX_TEXTURES];
    PixelLayoutSlot pixelLayouts[PGL_MAX_LAYOUTS];
    CameraSlot      cameras    [PGL_MAX_CAMERAS];
    DrawCall        drawList   [GpuConfig::MAX_DRAW_CALLS];
    uint16_t        drawCallCount = 0;

    // ── v8 generation-checked handles (PGL_CAP_HANDLE_GEN) ──────────────────
    // Per-slot generation byte recorded at CREATE time from the wire handle's
    // [generation:8 | index:8] encoding.  DESTROY/use sites must present a
    // handle whose generation matches the stored byte; legacy gen-0 handles
    // match slots created with plain indices (v7 semantics preserved).
    uint8_t         meshGeneration    [GpuConfig::MAX_MESHES]     = {};
    uint8_t         materialGeneration[GpuConfig::MAX_MATERIALS]  = {};
    uint8_t         textureGeneration [GpuConfig::MAX_TEXTURES]   = {};

    // ── Mesh content versions (F-04 frame-signature counters) ───────────────
    // Per-slot logical clock, bumped by the command parser on EVERY write to
    // a mesh slot's vertex data (CREATE_MESH initial data / re-gen overwrite,
    // UPDATE_VERTICES, UPDATE_VERTICES_DELTA, DESTROY_MESH).  The rasterizer's
    // frame signature folds these in instead of FNV-1a-hashing up to ~24 KB of
    // vertex bytes per frame — O(draws) instead of O(verts).
    // MONOTONIC: deliberately NOT cleared by Reset() — a scene rebuilt after a
    // reset must never reproduce a pre-reset signature (Reset wipes the slot
    // data these versions describe, so the versions must keep counting).
    uint32_t        meshVersion       [GpuConfig::MAX_MESHES]     = {};

    // ── Programmable shader programs ────────────────────────────────────────
    ShaderProgram   shaderPrograms[PGL_MAX_SHADER_PROGRAMS];

    // ── 2D Compositing Layers (M12) ────────────────────────────────────────
    LayerSlot       layers[PGL_MAX_LAYERS];          // layers[0] is 3D (not user-allocated)
    uint8_t         activeLayerCount = 0;            // Number of active layers (excl. layer 0)

    // ── 2D Draw Command Queue (M12, per-frame) ─────────────────────────────
    DrawCmd2D       drawCmds2D[PGL_MAX_2D_DRAW_CMDS]; // Queued 2D commands for this frame
    uint16_t        drawCmd2DCount = 0;                // Current queue depth

    // ── Defrag / Persistence / Dirty-rect Status Caches (M12, I2C readback) ─
    PglMemDefragStatusResponse   defragStatus   = {};
    PglMemPersistStatusResponse  persistStatus  = {};
    PglDirtyStatsResponse        dirtyStats     = {};
    uint16_t                     dmaFillThreshold = 64; // PGL_REG_DMA_FILL_THRESHOLD (pixels)

    // ── Frame info ──────────────────────────────────────────────────────────
    uint32_t frameNumber = 0;
    uint32_t frameTimeUs = 0;

    // ── Persistent memory: scene heap (ProtoGC) ─────────────────────────────
    // One capped HeapAllocator backs all persistent resource data (mesh
    // vertex/index/UV, texture pixels, layout coords). True random free with
    // immediate coalescing replaces the old stack-only bump pools; the cap
    // preserves the deterministic budget the static pools guaranteed.
    // NOT thread-shared: only Core 0 allocates (parser/Reset); Core 1 reads
    // pointers during tile passes (non-moving heap — stable).
    protogc::HeapAllocator sceneHeap;

    // ── Per-frame pool (reset every BeginFrame) ─────────────────────────────
    Pool<PglVec3,   GpuConfig::FRAME_VERTEX_POOL_SIZE> frameVertexPool;

    // ── GPU Memory Access State (for I2C readback) ──────────────────────────
    PglMemAllocResult lastAllocResult = {};      // Result of last CMD_MEM_ALLOC
    PglMemTierInfoResponse memTierInfo = {};     // Cached tier stats (updated periodically)

    // Memory read staging buffer for I2C readback (PGL_REG_MEM_READ_DATA).
    // Filled by CMD_MEM_READ_REQUEST or CMD_FRAMEBUFFER_CAPTURE.
    static constexpr uint32_t MEM_STAGING_SIZE = PGL_MEM_READ_MAX_SIZE;  // 4096 bytes
    uint8_t  memStagingBuffer[MEM_STAGING_SIZE] = {};
    uint32_t memStagingLength  = 0;   // Valid bytes in staging buffer
    uint32_t memStagingReadPos = 0;   // Current I2C read cursor (auto-increments)

    // ── Lifecycle ───────────────────────────────────────────────────────────

    /// Full reset — frees all scene-heap allocations (true random free),
    /// zeroes everything, resets the per-frame pool.
    void Reset() {
        // Free persistent resource data FIRST (slot pointers are needed for
        // the heap frees; the memsets below would orphan them).
        for (uint16_t i = 0; i < PGL_MAX_MESHES; ++i) {
            if (meshes[i].vertices)   (void)sceneHeap.deallocate(meshes[i].vertices);
            if (meshes[i].indices)    (void)sceneHeap.deallocate(meshes[i].indices);
            if (meshes[i].uvVertices) (void)sceneHeap.deallocate(meshes[i].uvVertices);
            if (meshes[i].uvIndices)  (void)sceneHeap.deallocate(meshes[i].uvIndices);
        }
        for (uint16_t i = 0; i < PGL_MAX_TEXTURES; ++i) {
            if (textures[i].pixels) (void)sceneHeap.deallocate(textures[i].pixels);
        }
        for (uint16_t i = 0; i < PGL_MAX_LAYOUTS; ++i) {
            if (pixelLayouts[i].coords) (void)sceneHeap.deallocate(pixelLayouts[i].coords);
        }

        std::memset(meshes,       0, sizeof(meshes));
        std::memset(materials,    0, sizeof(materials));
        std::memset(textures,     0, sizeof(textures));
        std::memset(pixelLayouts, 0, sizeof(pixelLayouts));
        std::memset(cameras,      0, sizeof(cameras));
        std::memset(drawList,     0, sizeof(drawList));
        std::memset(shaderPrograms, 0, sizeof(shaderPrograms));
        std::memset(meshGeneration,      0, sizeof(meshGeneration));
        std::memset(materialGeneration,  0, sizeof(materialGeneration));
        std::memset(textureGeneration,   0, sizeof(textureGeneration));
        drawCallCount = 0;
        frameNumber   = 0;
        frameTimeUs   = 0;

        // Free 2D layer framebuffers and reset layer slots
        for (uint8_t i = 0; i < PGL_MAX_LAYERS; ++i) {
            if (layers[i].pixels) {
                free(layers[i].pixels);
            }
        }
        std::memset(layers, 0, sizeof(layers));
        activeLayerCount = 0;
        drawCmd2DCount   = 0;
        std::memset(drawCmds2D, 0, sizeof(drawCmds2D));

        // Reset M12 status caches
        defragStatus     = {};
        persistStatus    = {};
        dirtyStats       = {};
        dmaFillThreshold = 64;

        frameVertexPool.Reset();

        // Reset memory access state
        lastAllocResult = {};
        memTierInfo     = {};
        memStagingLength  = 0;
        memStagingReadPos = 0;
    }

    /// Per-frame reset — clears draw list and frame pool; persistent resources
    /// (meshes, materials, textures, layouts, layers) survive.
    void BeginFrame(uint32_t newFrameNumber) {
        frameNumber   = newFrameNumber;
        drawCallCount = 0;
        std::memset(drawList, 0, sizeof(drawList));
        frameVertexPool.Reset();

        // Reset 2D draw queue and layer dirty flags (layers themselves persist)
        drawCmd2DCount = 0;
        for (uint8_t i = 0; i < PGL_MAX_LAYERS; ++i) {
            layers[i].dirty = false;
        }
    }

    // ── Scene heap lifecycle ────────────────────────────────────────────────

    /// One-time scene heap init — call from GpuCore::Initialize() BEFORE the
    /// first Reset()/allocation. pgc_init() is idempotent by contract.
    void InitSceneHeap() {
        pgc_init();
        sceneHeap.begin(GpuConfig::SCENE_HEAP_SEGMENT_BYTES,
                        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
                        protogc::HeapAllocator::defaultInternalForbiddenCaps(),
                        GpuConfig::SCENE_HEAP_MAX_BYTES);
    }

    // ── Pool allocation helpers (called by command_parser) ──────────────────
    // All persistent resource data comes from the capped scene heap. Growth:
    // allocate → createSegment → linkSegment → retry (Core-0-only callers,
    // so no locking is needed around the two-phase pattern).

    void* SceneHeapAlloc(size_t bytes) {
        if (void* p = sceneHeap.allocate(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)) {
            return p;
        }
        auto* seg = sceneHeap.createSegment(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!seg) return nullptr;  // platform heap OOM or scene cap reached
        sceneHeap.linkSegment(seg);
        return sceneHeap.allocate(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }

    PglVec3*   AllocVertices(uint16_t count)        { return static_cast<PglVec3*>(SceneHeapAlloc(count * sizeof(PglVec3))); }
    PglIndex3* AllocIndices(uint16_t count)         { return static_cast<PglIndex3*>(SceneHeapAlloc(count * sizeof(PglIndex3))); }
    PglVec2*   AllocUVVertices(uint16_t count)      { return static_cast<PglVec2*>(SceneHeapAlloc(count * sizeof(PglVec2))); }
    PglIndex3* AllocUVIndices(uint16_t count)       { return static_cast<PglIndex3*>(SceneHeapAlloc(count * sizeof(PglIndex3))); }
    uint8_t*   AllocTexturePixels(uint32_t bytes)   { return static_cast<uint8_t*>(SceneHeapAlloc(bytes)); }
    PglVec2*   AllocLayoutCoords(uint16_t count)    { return static_cast<PglVec2*>(SceneHeapAlloc(count * sizeof(PglVec2))); }
    PglVec3*   AllocFrameVertices(uint16_t count)   { return frameVertexPool.Allocate(count); }

    // ── 2D Layer Helpers ────────────────────────────────────────────────────

    /// Enqueue a 2D draw command.  Returns true if queued, false if full.
    bool Enqueue2DCmd(const DrawCmd2D& cmd) {
        if (drawCmd2DCount >= PGL_MAX_2D_DRAW_CMDS) return false;
        drawCmds2D[drawCmd2DCount++] = cmd;
        return true;
    }

    /// Allocate a layer framebuffer.  Returns true on success.
    bool AllocLayerFramebuffer(uint8_t layerId) {
        if (layerId == 0 || layerId >= PGL_MAX_LAYERS) return false;
        LayerSlot& l = layers[layerId];
        if (l.pixels) return true;  // Already allocated
        uint32_t bytes = uint32_t(l.width) * l.height * 2;  // RGB565
        if (bytes == 0) return false;
        l.pixels = static_cast<uint16_t*>(malloc(bytes));
        if (!l.pixels) return false;
        std::memset(l.pixels, 0, bytes);
        return true;
    }

    /// Free a layer framebuffer.
    void FreeLayerFramebuffer(uint8_t layerId) {
        if (layerId == 0 || layerId >= PGL_MAX_LAYERS) return;
        LayerSlot& l = layers[layerId];
        if (l.pixels) { free(l.pixels); l.pixels = nullptr; }
    }

    // ── Pool deallocation helpers (true random free via the scene heap) ─────

    void FreeVertices(PglVec3* p)        { if (p) (void)sceneHeap.deallocate(p); }
    void FreeIndices(PglIndex3* p)       { if (p) (void)sceneHeap.deallocate(p); }
    void FreeUVVertices(PglVec2* p)      { if (p) (void)sceneHeap.deallocate(p); }
    void FreeUVIndices(PglIndex3* p)     { if (p) (void)sceneHeap.deallocate(p); }
    void FreeTexturePixels(uint8_t* p)   { if (p) (void)sceneHeap.deallocate(p); }
    void FreeLayoutCoords(PglVec2* p)    { if (p) (void)sceneHeap.deallocate(p); }

    // ── Diagnostics ─────────────────────────────────────────────────────────

    void PrintPoolUsage() const {
        const auto hs = sceneHeap.stats();
        printf("[Scene] Heap: used %u/%u KB (segments %u KB, largest free %u B); "
               "frameVtx %u/%u\n",
               static_cast<unsigned>(hs.usedBytes / 1024),
               static_cast<unsigned>(GpuConfig::SCENE_HEAP_MAX_BYTES / 1024),
               static_cast<unsigned>(hs.segmentBytes / 1024),
               static_cast<unsigned>(hs.largestFreeBlock),
               frameVertexPool.used, GpuConfig::FRAME_VERTEX_POOL_SIZE);
    }
};
