/**
 * @file scene_state.h
 * @brief GPU-side scene resource tables, memory pools, and 2D layer state for RP2350.
 *
 * Holds all persistent resources (meshes, materials, textures, layouts) and
 * per-frame state (draw list, cameras, 2D layers, 2D draw commands).
 * Large data lives in typed bump pools so that slot metadata stays small
 * (~40 B per mesh slot) and we stay well within the RP2350's 520 KB SRAM budget.
 *
 * Memory layout (approximate):
 *   Pools (persistent):   ~130 KB
 *   Pool (per-frame):      ~24 KB
 *   Slot arrays:            ~40 KB
 *   2D layer FBs:           ~16 KB per layer (128×64 RGB565)
 *   Total scene state:    ~210 KB (with 1 active 2D layer)
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
#include "gpu_config.h"

// ─── Typed Bump Pool ────────────────────────────────────────────────────────
// A simple arena allocator.  Persistent pools survive across frames; the
// per-frame pool is reset at BeginFrame().

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

    /// Free everything from `ptr` onward (stack-style).
    /// For true random free, a free-list allocator is needed — TODO(M5).
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

    PglVec3*   vertices  = nullptr;  // → SceneState::vertexPool
    PglIndex3* indices   = nullptr;  // → SceneState::indexPool
    PglVec2*   uvVertices = nullptr; // → SceneState::uvVertexPool
    PglIndex3* uvIndices  = nullptr; // → SceneState::uvIndexPool

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
    uint8_t*         pixels        = nullptr;  // → SceneState::texturePool
};

struct PixelLayoutSlot {
    bool     active     = false;
    uint16_t pixelCount = 0;
    uint8_t  flags      = 0;

    PglRectLayoutData rectData = {};          // used if RECTANGULAR flag set
    PglVec2*          coords   = nullptr;     // → SceneState::layoutCoordPool
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

    // ── Persistent memory pools ─────────────────────────────────────────────
    Pool<PglVec3,   GpuConfig::VERTEX_POOL_SIZE>       vertexPool;
    Pool<PglIndex3, GpuConfig::INDEX_POOL_SIZE>        indexPool;
    Pool<PglVec2,   GpuConfig::UV_VERTEX_POOL_SIZE>    uvVertexPool;
    Pool<PglIndex3, GpuConfig::UV_INDEX_POOL_SIZE>     uvIndexPool;
    Pool<uint8_t,   GpuConfig::TEXTURE_POOL_SIZE>      texturePool;
    Pool<PglVec2,   GpuConfig::LAYOUT_COORD_POOL_SIZE> layoutCoordPool;

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

    /// Full reset — zeroes everything, resets all pools.
    void Reset() {
        std::memset(meshes,       0, sizeof(meshes));
        std::memset(materials,    0, sizeof(materials));
        std::memset(textures,     0, sizeof(textures));
        std::memset(pixelLayouts, 0, sizeof(pixelLayouts));
        std::memset(cameras,      0, sizeof(cameras));
        std::memset(drawList,     0, sizeof(drawList));
        std::memset(shaderPrograms, 0, sizeof(shaderPrograms));
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

        vertexPool.Reset();
        indexPool.Reset();
        uvVertexPool.Reset();
        uvIndexPool.Reset();
        texturePool.Reset();
        layoutCoordPool.Reset();
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

    // ── Pool allocation helpers (called by command_parser) ──────────────────

    PglVec3*   AllocVertices(uint16_t count)        { return vertexPool.Allocate(count); }
    PglIndex3* AllocIndices(uint16_t count)         { return indexPool.Allocate(count); }
    PglVec2*   AllocUVVertices(uint16_t count)      { return uvVertexPool.Allocate(count); }
    PglIndex3* AllocUVIndices(uint16_t count)       { return uvIndexPool.Allocate(count); }
    uint8_t*   AllocTexturePixels(uint32_t bytes)   { return texturePool.Allocate(bytes); }
    PglVec2*   AllocLayoutCoords(uint16_t count)    { return layoutCoordPool.Allocate(count); }
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

    // ── Pool deallocation helpers (simple stack-style) ──────────────────────

    void FreeVertices(PglVec3* p)        { vertexPool.FreeFrom(p); }
    void FreeIndices(PglIndex3* p)       { indexPool.FreeFrom(p); }
    void FreeUVVertices(PglVec2* p)      { uvVertexPool.FreeFrom(p); }
    void FreeUVIndices(PglIndex3* p)     { uvIndexPool.FreeFrom(p); }
    void FreeTexturePixels(uint8_t* p)   { texturePool.FreeFrom(p); }
    void FreeLayoutCoords(PglVec2* p)    { layoutCoordPool.FreeFrom(p); }

    // ── Diagnostics ─────────────────────────────────────────────────────────

    void PrintPoolUsage() const {
        printf("[Scene] Pools: vtx %u/%u  idx %u/%u  uvVtx %u/%u  uvIdx %u/%u  "
               "tex %u/%u  layout %u/%u  frameVtx %u/%u\n",
               vertexPool.used,      GpuConfig::VERTEX_POOL_SIZE,
               indexPool.used,        GpuConfig::INDEX_POOL_SIZE,
               uvVertexPool.used,     GpuConfig::UV_VERTEX_POOL_SIZE,
               uvIndexPool.used,      GpuConfig::UV_INDEX_POOL_SIZE,
               texturePool.used,      GpuConfig::TEXTURE_POOL_SIZE,
               layoutCoordPool.used,  GpuConfig::LAYOUT_COORD_POOL_SIZE,
               frameVertexPool.used,  GpuConfig::FRAME_VERTEX_POOL_SIZE);
    }
};
