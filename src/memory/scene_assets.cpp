#include "../scene_state.h"

namespace {
using R = PglRuntime::Result;
R AssetError(gpumem::AssetStatus status) {
    if (status == gpumem::AssetStatus::Ok) return R::Ok;
    if (status == gpumem::AssetStatus::NoBacking) return R::Unsupported;
    if (status == gpumem::AssetStatus::BackingError) return R::Io;
    if (status == gpumem::AssetStatus::InvalidHandle) return R::InvalidHandle;
    return R::Capacity;
}
size_t Align4(size_t n) { return (n + 3u) & ~size_t(3); }
}

void SceneState::ReleaseFrameAssets() {
    if (externalAssets) {
        for (uint8_t i = 0; i < assetLeaseCount; ++i)
            externalAssets->releaseSpan(assetLeases[i],false);
    }
    assetLeaseCount = 0;
    for (auto& mesh : meshes) if (mesh.externalToken != gpumem::kAssetTokenInvalid) {
        mesh.vertices = nullptr; mesh.indices = nullptr;
        mesh.uvVertices = nullptr; mesh.uvIndices = nullptr;
    }
    for (auto& texture : textures)
        if (texture.externalToken != gpumem::kAssetTokenInvalid) texture.pixels = nullptr;
}

PglRuntime::Result SceneState::PinFrameAssets() {
    if (assetLeaseCount) return R::BadState;
    bool meshReads[GpuConfig::MAX_MESHES] = {}, textureReads[GpuConfig::MAX_TEXTURES] = {};
    auto materialReads = [&](auto&& self, uint16_t handle, uint8_t depth) -> bool {
        const uint16_t index = PglHandleIndex(handle);
        if (index >= GpuConfig::MAX_MATERIALS || !materials[index].active ||
            materialGeneration[index] != PglHandleGeneration(handle) || depth > 3) return false;
        const auto& material = materials[index];
        if (material.type == PGL_MAT_IMAGE || material.type == PGL_MAT_PRERENDERED) {
            const uint16_t texture = PglRuntime::Load16(material.params), ti = PglHandleIndex(texture);
            if (ti >= GpuConfig::MAX_TEXTURES || !textures[ti].active ||
                textureGeneration[ti] != PglHandleGeneration(texture)) return false;
            textureReads[ti] = true;
        } else if (material.type == PGL_MAT_COMBINE || material.type == PGL_MAT_MASK || material.type == PGL_MAT_ANIMATOR) {
            return self(self,PglRuntime::Load16(material.params),depth+1) &&
                   self(self,PglRuntime::Load16(material.params+2),depth+1);
        }
        return true;
    };
    for (uint16_t i = 0; i < drawCallCount; ++i) if (drawList[i].enabled) {
        const auto& draw = drawList[i];
        if (draw.meshId >= GpuConfig::MAX_MESHES || !meshes[draw.meshId].active ||
            draw.materialId >= GpuConfig::MAX_MATERIALS ||
            !materialReads(materialReads,PglMakeHandle(materialGeneration[draw.materialId],draw.materialId),0))
            return R::InvalidHandle;
        meshReads[draw.meshId] = true;
    }
    for (uint16_t i = 0; i < drawCmd2DCount; ++i) {
        const auto& command = drawCmds2D[i];
        uint16_t handle;
        if (command.type == DRAW_CMD_2D_SPRITE) handle = command.sprite.textureId;
        else if (command.type == DRAW_CMD_2D_SPRITE_BATCH) handle = command.spriteBatch.textureId;
        else if (command.type == DRAW_CMD_2D_TEXT) handle = command.text.fontTextureId;
        else continue;
        const uint16_t index = PglHandleIndex(handle);
        if (index >= GpuConfig::MAX_TEXTURES || !textures[index].active ||
            textureGeneration[index] != PglHandleGeneration(handle)) return R::InvalidHandle;
        textureReads[index] = true;
    }
    auto pin = [&](uint32_t token,uint32_t bytes,uint8_t*& data) -> R {
        if (!externalAssets || !externalAssets->hasBacking()) return R::Unsupported;
        if (assetLeaseCount >= GPU_MEM_MAX_SPANS) return R::Capacity;
        auto& span = assetLeases[assetLeaseCount];
        R result = AssetError(externalAssets->prefetchSpan(gpumem::assetHandleFromToken(token),0,bytes,span));
        if (result == R::Ok) { data = span.data; ++assetLeaseCount; }
        return result;
    };
    for (uint16_t i = 0; i < GpuConfig::MAX_MESHES; ++i) if (meshReads[i]) {
        auto& mesh = meshes[i];
        if (mesh.externalToken == gpumem::kAssetTokenInvalid) continue;
        uint8_t* base = nullptr;
        R result = pin(mesh.externalToken,mesh.storageBytes,base);
        if (result != R::Ok) {
            ReleaseFrameAssets();
            return result;
        }
        const size_t io = Align4(size_t(mesh.vertexCount)*sizeof(PglVec3));
        const size_t uo = Align4(io + size_t(mesh.triangleCount)*sizeof(PglIndex3));
        mesh.vertices = reinterpret_cast<PglVec3*>(base);
        mesh.indices = reinterpret_cast<PglIndex3*>(base+io);
        if (mesh.uvVertexCount) {
            mesh.uvVertices = reinterpret_cast<PglVec2*>(base+uo);
            mesh.uvIndices = reinterpret_cast<PglIndex3*>(base+Align4(uo+size_t(mesh.uvVertexCount)*sizeof(PglVec2)));
        }
    }
    for (uint16_t i = 0; i < GpuConfig::MAX_TEXTURES; ++i) if (textureReads[i]) {
        auto& texture = textures[i];
        if (texture.externalToken == gpumem::kAssetTokenInvalid) continue;
        R result = pin(texture.externalToken,texture.pixelDataSize,texture.pixels);
        if (result != R::Ok) {
            ReleaseFrameAssets();
            return result;
        }
    }
    return R::Ok;
}
