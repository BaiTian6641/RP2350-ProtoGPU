/**
 * @file scene_state.h
 * @brief GPU-side scene resource tables and memory pools for RP2350.
 *
 * Holds all persistent resources (meshes, materials, textures, layouts) and
 * per-frame state (draw list, cameras).  Large data lives in typed bump pools
 * so that slot metadata stays small (~40 B per mesh slot) and we stay well
 * within the RP2350's 520 KB SRAM budget.
 *
 * Memory layout (approximate):
 *   Pools (persistent):   ~130 KB
 *   Pool (per-frame):      ~24 KB
 *   Slot arrays:            ~40 KB
 *   Total scene state:    ~194 KB
 *
 * Framebuffers, Z-buffer, QuadTree, code, and stack use the rest of SRAM.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>

#include <PglTypes.h>
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
};

struct MaterialSlot {
    bool           active    = false;
    PglMaterialType type     = PGL_MAT_SIMPLE;
    PglBlendMode   blendMode = PGL_BLEND_BASE;
    uint8_t        params[64] = {};   // type-specific, copied verbatim from wire
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
        drawCallCount = 0;
        frameNumber   = 0;
        frameTimeUs   = 0;

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
    /// (meshes, materials, textures, layouts) survive.
    void BeginFrame(uint32_t newFrameNumber) {
        frameNumber   = newFrameNumber;
        drawCallCount = 0;
        std::memset(drawList, 0, sizeof(drawList));
        frameVertexPool.Reset();
    }

    // ── Pool allocation helpers (called by command_parser) ──────────────────

    PglVec3*   AllocVertices(uint16_t count)        { return vertexPool.Allocate(count); }
    PglIndex3* AllocIndices(uint16_t count)         { return indexPool.Allocate(count); }
    PglVec2*   AllocUVVertices(uint16_t count)      { return uvVertexPool.Allocate(count); }
    PglIndex3* AllocUVIndices(uint16_t count)       { return uvIndexPool.Allocate(count); }
    uint8_t*   AllocTexturePixels(uint32_t bytes)   { return texturePool.Allocate(bytes); }
    PglVec2*   AllocLayoutCoords(uint16_t count)    { return layoutCoordPool.Allocate(count); }
    PglVec3*   AllocFrameVertices(uint16_t count)   { return frameVertexPool.Allocate(count); }

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
