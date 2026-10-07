#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <new>
#include <PglTypes.h>
#include <PglRenderCommands.h>
#include <PglShaderBytecode.h>
#include <PglRuntimeProtocol.h>
#include <HeapAllocator.h>
#include "gpu_config.h"
#include "memory/mem_assets.h"

template <typename T, uint32_t Capacity>
struct Pool {
    T data[Capacity] = {};
    uint32_t used = 0;
    T* Allocate(uint32_t count) {
        if (count > Capacity - used) return nullptr;
        T* result = data + used; used += count; return result;
    }
    void Reset() { used = 0; }
    uint32_t Available() const { return Capacity - used; }
    static constexpr uint32_t Cap() { return Capacity; }
};

struct MeshSlot {
    bool active = false;
    uint16_t vertexCount = 0, triangleCount = 0, uvVertexCount = 0;
    PglVec3* vertices = nullptr;
    PglIndex3* indices = nullptr;
    PglVec2* uvVertices = nullptr;
    PglIndex3* uvIndices = nullptr;
    void* storage = nullptr;
    uint32_t storageBytes = 0;
    uint32_t externalToken = 0xffffffffu;
    PglVec3 aabbMin = {}, aabbMax = {};
    void RecomputeAABB() {
        if (!vertices || !vertexCount) return;
        aabbMin = aabbMax = vertices[0];
        for (uint16_t i = 1; i < vertexCount; ++i) {
            const auto& v = vertices[i];
            if (v.x < aabbMin.x) aabbMin.x = v.x;
            if (v.y < aabbMin.y) aabbMin.y = v.y;
            if (v.z < aabbMin.z) aabbMin.z = v.z;
            if (v.x > aabbMax.x) aabbMax.x = v.x;
            if (v.y > aabbMax.y) aabbMax.y = v.y;
            if (v.z > aabbMax.z) aabbMax.z = v.z;
        }
    }
};

struct MaterialSlot {
    bool active = false;
    PglMaterialType type = PGL_MAT_SIMPLE;
    PglBlendMode blendMode = PGL_BLEND_BASE;
    uint8_t params[60] = {};
    uint8_t paramBytes = 0;
    float alpha = 1.0f;
};

struct TextureSlot {
    bool active = false;
    uint16_t width = 0, height = 0;
    PglTextureFormat format = PGL_TEX_RGB565;
    uint32_t pixelDataSize = 0;
    uint8_t* pixels = nullptr;
    void* storage = nullptr;
    uint32_t externalToken = 0xffffffffu;
};

struct PixelLayoutSlot {
    bool active = false;
    uint16_t pixelCount = 0;
    uint8_t flags = 0;
    PglRectLayoutData rectData = {};
    PglVec2* coords = nullptr;
};

struct ShaderSlot {
    bool active = false;
    uint8_t shaderClass = 0;
    float intensity = 0.0f;
    uint8_t params[20] = {};
    uint16_t programId = 0;
};

struct ShaderProgram {
    bool active = false;
    uint16_t programId = 0;
    uint8_t uniformCount = 0, constCount = 0;
    uint16_t instrCount = 0;
    uint8_t flags = 0;
    float uniforms[PSB_MAX_UNIFORMS] = {};
    float constants[PSB_MAX_CONSTANTS] = {};
    uint32_t instructions[PSB_MAX_INSTRUCTIONS] = {};
    uint32_t uniformNameHashes[PSB_MAX_UNIFORMS] = {};
    uint8_t uniformTypes[PSB_MAX_UNIFORMS] = {};
    bool verified = false;
    bool readsFramebuffer = false;
    uint32_t weightedCost = 0;
};

struct CameraSlot {
    bool active = false;
    uint8_t layoutId = 0;
    PglVec3 position = {};
    PglQuat rotation = {1, 0, 0, 0};
    PglVec3 scale = {1, 1, 1};
    PglQuat lookOffset = {1, 0, 0, 0}, baseRotation = {1, 0, 0, 0};
    bool is2D = false;
    uint8_t targetLayer = 0, vpFlags = 0;
    uint16_t vpX = 0, vpY = 0, vpW = 0, vpH = 0;
    ShaderSlot shaders[PGL_MAX_SHADERS_PER_CAMERA];
};

struct DrawCall {
    uint16_t meshId = 0, materialId = 0;
    bool enabled = false;
    PglTransform transform = {};
    bool hasVertexOverride = false;
    uint16_t overrideVertexCount = 0;
    PglVec3* overrideVertices = nullptr;
};

struct LayerSlot {
    bool active = false, visible = true, dirty = false;
    uint16_t width = 0, height = 0;
    uint8_t pixelFormat = PGL_PIXFMT_RGB565, blendMode = PGL_LAYER_BLEND_ALPHA, opacity = 255;
    int16_t offsetX = 0, offsetY = 0;
    uint16_t* pixels = nullptr;
    int16_t clipX = 0, clipY = 0;
    uint16_t clipW = 0, clipH = 0;
    int16_t viewOffX = 0, viewOffY = 0, viewScaleXQ8 = 256, viewScaleYQ8 = 256;
    ShaderSlot shaders[PGL_MAX_SHADERS_PER_CAMERA];
};

enum DrawCmd2DType : uint8_t {
    DRAW_CMD_2D_RECT, DRAW_CMD_2D_LINE, DRAW_CMD_2D_CIRCLE, DRAW_CMD_2D_SPRITE,
    DRAW_CMD_2D_CLEAR, DRAW_CMD_2D_ROUNDED_RECT, DRAW_CMD_2D_ARC, DRAW_CMD_2D_TRIANGLE,
    DRAW_CMD_2D_TEXT, DRAW_CMD_2D_SPRITE_BATCH, DRAW_CMD_2D_GRADIENT_RECT,
    DRAW_CMD_2D_WRITE_PIXELS
};

struct DrawCmd2D {
    DrawCmd2DType type = DRAW_CMD_2D_RECT;
    uint8_t layerId = 0;
    int16_t clipX = 0, clipY = 0;
    uint16_t clipW = 0xffff, clipH = 0xffff;
    int16_t viewOffsetX = 0, viewOffsetY = 0, viewScaleXQ8 = 256, viewScaleYQ8 = 256;
    union {
        PglCmdDrawRect2D rect;
        PglCmdDrawLine2D line;
        PglCmdDrawCircle2D circle;
        PglCmdDrawSprite sprite;
        PglCmdLayerClear clear;
        PglCmdDrawRoundedRect roundedRect;
        PglCmdDrawArc arc;
        PglCmdDrawTriangle2D triangle;
        PglCmdDrawGradientRect gradient;
        struct {
            uint16_t fontTextureId;
            int16_t x, y;
            uint8_t glyphW, glyphH, columns, firstChar;
            uint16_t color, textLength;
            const char* bytes;
        } text;
        struct { uint16_t textureId, posOffset, count; uint8_t flags; } spriteBatch;
    };
    DrawCmd2D() : rect{} {}
};
static_assert(sizeof(DrawCmd2D) <= 48, "bounded primitive snapshot");

struct CameraTargetInfo {
    uint16_t* fb = nullptr;
    uint16_t width = 0, height = 0;
    uint16_t scX0 = 0, scY0 = 0, scX1 = 0, scY1 = 0;
    bool valid = false;
};

struct ResourceUpload {
    bool active = false;
    uint8_t resourceClass = 0, preferredTier = 0;
    uint16_t resourceId = 0;
    uint32_t bytes = 0, received = 0, checksum = 0;
    uint8_t* storage = nullptr;
};

struct SceneState {
    MeshSlot meshes[GpuConfig::MAX_MESHES];
    MaterialSlot materials[GpuConfig::MAX_MATERIALS];
    TextureSlot textures[GpuConfig::MAX_TEXTURES];
    PixelLayoutSlot pixelLayouts[PGL_MAX_LAYOUTS];
    CameraSlot cameras[PGL_MAX_CAMERAS];
    DrawCall drawList[GpuConfig::MAX_DRAW_CALLS];
    uint16_t drawCallCount = 0;
    uint8_t meshGeneration[GpuConfig::MAX_MESHES] = {};
    uint8_t materialGeneration[GpuConfig::MAX_MATERIALS] = {};
    uint8_t textureGeneration[GpuConfig::MAX_TEXTURES] = {};
    bool meshEverUsed[GpuConfig::MAX_MESHES] = {};
    bool materialEverUsed[GpuConfig::MAX_MATERIALS] = {};
    bool textureEverUsed[GpuConfig::MAX_TEXTURES] = {};
    bool pendingMeshDestroy[GpuConfig::MAX_MESHES] = {};
    bool pendingMaterialDestroy[GpuConfig::MAX_MATERIALS] = {};
    bool pendingTextureDestroy[GpuConfig::MAX_TEXTURES] = {};
    uint32_t shaderStateVersion = 0, shaderFrameOps = 0;
    ShaderProgram shaderPrograms[GpuConfig::MAX_SHADER_PROGRAMS];
    LayerSlot layers[GpuConfig::MAX_LAYERS];
    uint8_t activeLayerCount = 0;
    DrawCmd2D drawCmds2D[PGL_MAX_2D_DRAW_CMDS];
    uint16_t drawCmd2DCount = 0;
    Pool<PglSpritePosition, 256> spritePosPool2D;
    uint32_t frameNumber = 0, frameTimeUs = 0;
    uint64_t elapsedTimeUs = 0;
    uint16_t renderWidth = GpuConfig::PANEL_WIDTH, renderHeight = GpuConfig::PANEL_HEIGHT;
    void* frameOwnedData[GpuConfig::MAX_DRAW_CALLS] = {};
    uint8_t frameOwnedCount = 0;
    ResourceUpload upload;
    gpumem::AssetService* externalAssets = nullptr;
    gpumem::AssetSpan assetLeases[GPU_MEM_MAX_SPANS];
    uint8_t assetLeaseCount = 0;
    PglRuntime::Result PinFrameAssets();
    void ReleaseFrameAssets();
    protogc::HeapAllocator sceneHeap;
    alignas(std::max_align_t) uint8_t sceneBacking[GpuConfig::SCENE_HEAP_MAX_BYTES] = {};

    bool InitSceneHeap() {
        return sceneHeap.beginExternal(sceneBacking, sizeof(sceneBacking),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
            protogc::HeapAllocator::defaultInternalForbiddenCaps());
    }
    void* SceneHeapAlloc(size_t bytes) {
        if (!bytes) return nullptr;
        return sceneHeap.allocate(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    void SceneHeapFree(void* pointer) { if (pointer) (void)sceneHeap.deallocate(pointer); }
    PglVec3* AllocVertices(uint16_t n) { return static_cast<PglVec3*>(SceneHeapAlloc(size_t(n) * sizeof(PglVec3))); }
    PglIndex3* AllocIndices(uint16_t n) { return static_cast<PglIndex3*>(SceneHeapAlloc(size_t(n) * sizeof(PglIndex3))); }
    PglVec2* AllocUVVertices(uint16_t n) { return static_cast<PglVec2*>(SceneHeapAlloc(size_t(n) * sizeof(PglVec2))); }
    PglIndex3* AllocUVIndices(uint16_t n) { return static_cast<PglIndex3*>(SceneHeapAlloc(size_t(n) * sizeof(PglIndex3))); }
    uint8_t* AllocTexturePixels(uint32_t n) { return static_cast<uint8_t*>(SceneHeapAlloc(n)); }
    PglVec2* AllocLayoutCoords(uint16_t n) { return static_cast<PglVec2*>(SceneHeapAlloc(size_t(n) * sizeof(PglVec2))); }
    void FreeVertices(PglVec3* p) { SceneHeapFree(p); }
    void FreeIndices(PglIndex3* p) { SceneHeapFree(p); }
    void FreeUVVertices(PglVec2* p) { SceneHeapFree(p); }
    void FreeUVIndices(PglIndex3* p) { SceneHeapFree(p); }
    void FreeTexturePixels(uint8_t* p) { SceneHeapFree(p); }
    void FreeLayoutCoords(PglVec2* p) { SceneHeapFree(p); }

    void FreeMesh(uint16_t index) {
        MeshSlot& mesh = meshes[index];
        if (mesh.externalToken != gpumem::kAssetTokenInvalid) {
            if (externalAssets) externalAssets->destroy(gpumem::assetHandleFromToken(mesh.externalToken));
        } else if (mesh.storage) SceneHeapFree(mesh.storage);
        else { SceneHeapFree(mesh.vertices); SceneHeapFree(mesh.indices); SceneHeapFree(mesh.uvVertices); SceneHeapFree(mesh.uvIndices); }
        mesh = {};
        pendingMeshDestroy[index] = false;
    }
    void FreeTexture(uint16_t index) {
        auto& texture = textures[index];
        if (texture.externalToken != gpumem::kAssetTokenInvalid) {
            if (externalAssets) externalAssets->destroy(gpumem::assetHandleFromToken(texture.externalToken));
        } else SceneHeapFree(texture.storage ? texture.storage : texture.pixels);
        texture = {};
        pendingTextureDestroy[index] = false;
    }
    void RetireResourceReads() {
        ReleaseFrameAssets();
        for (uint16_t i = 0; i < GpuConfig::MAX_MESHES; ++i) if (pendingMeshDestroy[i]) FreeMesh(i);
        for (uint16_t i = 0; i < GpuConfig::MAX_TEXTURES; ++i) if (pendingTextureDestroy[i]) FreeTexture(i);
        for (uint16_t i = 0; i < GpuConfig::MAX_MATERIALS; ++i) if (pendingMaterialDestroy[i]) {
            materials[i] = {}; pendingMaterialDestroy[i] = false;
        }
    }
    bool AllocLayerFramebuffer(uint8_t id) {
        if (!id || id >= GpuConfig::MAX_LAYERS) return false;
        auto& layer = layers[id];
        const size_t count = size_t(layer.width) * layer.height;
        if (!count || count > GpuConfig::FRAMEBUF_PIXELS || layer.pixels) return false;
        layer.pixels = static_cast<uint16_t*>(SceneHeapAlloc(count * sizeof(uint16_t)));
        if (!layer.pixels) return false;
        std::memset(layer.pixels, 0, count * sizeof(uint16_t));
        return true;
    }
    void FreeLayerFramebuffer(uint8_t id) {
        if (!id || id >= GpuConfig::MAX_LAYERS) return;
        SceneHeapFree(layers[id].pixels); layers[id].pixels = nullptr;
    }
    bool Enqueue2DCmd(const DrawCmd2D& command) {
        if (drawCmd2DCount >= PGL_MAX_2D_DRAW_CMDS) return false;
        drawCmds2D[drawCmd2DCount++] = command;
        return true;
    }
    void RetireFrameData() {
        for (uint8_t i = 0; i < frameOwnedCount; ++i) SceneHeapFree(frameOwnedData[i]);
        frameOwnedCount = 0;
    }
    void BeginFrame(uint32_t id) {
        RetireFrameData();
        frameNumber = id; drawCallCount = 0; drawCmd2DCount = 0;
        spritePosPool2D.Reset(); shaderFrameOps = 0;
        for (auto& draw : drawList) draw = {};
        for (auto& layer : layers) layer.dirty = false;
    }
    void Reset() {
        ReleaseFrameAssets();
        RetireFrameData();
        SceneHeapFree(upload.storage); upload = {};
        for (uint16_t i = 0; i < GpuConfig::MAX_MESHES; ++i) FreeMesh(i);
        for (uint16_t i = 0; i < GpuConfig::MAX_TEXTURES; ++i) FreeTexture(i);
        if(externalAssets)externalAssets->reset();
        for (auto& layout : pixelLayouts) { SceneHeapFree(layout.coords); layout = {}; }
        for (uint8_t i = 1; i < GpuConfig::MAX_LAYERS; ++i) { FreeLayerFramebuffer(i); layers[i] = {}; }
        for (auto& material : materials) material = {};
        for (auto& camera : cameras) camera = {};
        for (auto& program : shaderPrograms) program = {};
        for (auto& generation : meshGeneration) generation = 0;
        for (auto& generation : materialGeneration) generation = 0;
        for (auto& generation : textureGeneration) generation = 0;
        for (auto& used : meshEverUsed) used = false;
        for (auto& used : materialEverUsed) used = false;
        for (auto& used : textureEverUsed) used = false;
        for (auto& destroy : pendingMaterialDestroy) destroy = false;
        activeLayerCount = 0; frameTimeUs = 0; elapsedTimeUs = 0;
        layers[0] = {}; layers[0].active = true;
        layers[0].width = renderWidth; layers[0].height = renderHeight;
        layers[0].clipW = renderWidth; layers[0].clipH = renderHeight;
        ++shaderStateVersion; BeginFrame(0);
    }
    CameraTargetInfo ResolveCameraTarget(uint8_t cameraIndex, uint16_t* back,
                                          uint16_t panelW, uint16_t panelH) const {
        CameraTargetInfo target;
        if (cameraIndex >= PGL_MAX_CAMERAS) return target;
        const auto& camera = cameras[cameraIndex];
        if (!camera.targetLayer) { target.fb = back; target.width = panelW; target.height = panelH; target.valid = true; }
        else if (camera.targetLayer < GpuConfig::MAX_LAYERS) {
            const auto& layer = layers[camera.targetLayer];
            if (!layer.active || !layer.pixels || !layer.width || !layer.height) return target;
            target.fb = layer.pixels; target.width = layer.width; target.height = layer.height; target.valid = true;
        }
        if (!target.valid || uint32_t(target.width) * target.height > GpuConfig::FRAMEBUF_PIXELS) return {};
        target.scX1 = target.width; target.scY1 = target.height;
        if (camera.vpFlags & PGL_CAMERA_TARGET_SCISSOR) {
            target.scX0 = camera.vpX < target.width ? camera.vpX : target.width;
            target.scY0 = camera.vpY < target.height ? camera.vpY : target.height;
            const uint32_t x1 = uint32_t(camera.vpX) + camera.vpW, y1 = uint32_t(camera.vpY) + camera.vpH;
            target.scX1 = x1 < target.width ? static_cast<uint16_t>(x1) : target.width;
            target.scY1 = y1 < target.height ? static_cast<uint16_t>(y1) : target.height;
            if (target.scX1 < target.scX0) target.scX1 = target.scX0;
            if (target.scY1 < target.scY0) target.scY1 = target.scY0;
        }
        return target;
    }
    void PrintPoolUsage() const {
        const auto stats = sceneHeap.stats();
        std::printf("[Scene] used=%u capacity=%u largest=%u\n",
            unsigned(stats.usedBytes), unsigned(sizeof(sceneBacking)), unsigned(stats.largestFreeBlock));
    }
};
