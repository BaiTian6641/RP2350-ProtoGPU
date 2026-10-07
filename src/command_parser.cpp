#include "command_parser.h"
#include "scene_state.h"
#include "phase_scratch.h"
#include "render/pgl_shader_vm.h"
#include "render/screenspace_effects.h"
#include <PglOpcodes.h>
#include <PglCRC16.h>
#include <cstring>
#include <cmath>
#include <new>

namespace CommandParser {
namespace {
using R = PglRuntime::Result;
uint16_t errors = 0;
uint32_t errorMask = 0;

enum class Kind : uint8_t { Mesh, Material, Texture, Layout, Camera, Layer, Shader };
struct Change {
    static constexpr uint8_t HasGeneration = 1, Destroy = 2;
    void* value;
    const uint8_t* adoptedPayload;
    Kind kind;
    uint8_t index, generation, flags;
};
static_assert(GpuConfig::MAX_MESHES <= 256 && GpuConfig::MAX_MATERIALS <= 256 &&
              GpuConfig::MAX_TEXTURES <= 256 && PGL_MAX_LAYOUTS <= 256 &&
              PGL_MAX_CAMERAS <= 256 && GpuConfig::MAX_LAYERS <= 256 &&
              GpuConfig::MAX_SHADER_PROGRAMS <= 256, "change indices must fit one byte");

// Ingress remains immutable through Commit. Retain only a validated record's
// bounded byte offset and the separately owned override; expand DrawCall once.
struct PendingDraw {
    PglVec3* overrideVertices;
    uint16_t recordOffset, overrideVertexCount;
};
union Pending2D {
    DrawCmd2D value;
    Pending2D() {} // Begin each occupied value's lifetime in Queue, not all 128.
};
static_assert(PglRuntime::MaxBatchBytes <= UINT16_MAX, "pending record offsets must fit");
bool Finite(float value) {
    uint32_t bits; std::memcpy(&bits, &value, 4);
    return (bits & 0x7f800000u) != 0x7f800000u;
}
float FloatAt(const uint8_t* bytes) {
    uint32_t bits = PglRuntime::Load32(bytes); float value;
    std::memcpy(&value, &bits, 4); return value;
}
bool FiniteRange(const uint8_t* bytes, size_t floats) {
    for (size_t i = 0; i < floats; ++i) {
        const float value = FloatAt(bytes + 4 * i);
        if (!Finite(value) || value < -1000000.0f || value > 1000000.0f) return false;
    }
    return true;
}
bool Quat(const PglQuat& q) {
    if (!Finite(q.w) || !Finite(q.x) || !Finite(q.y) || !Finite(q.z)) return false;
    const float norm = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    return norm >= 0.999f && norm <= 1.001f;
}
bool Transform(const PglTransform& t) {
    return FiniteRange(reinterpret_cast<const uint8_t*>(&t), sizeof(t)/4) &&
        Quat(t.rotation) && Quat(t.baseRotation) && Quat(t.scaleRotationOffset);
}
size_t Align4(size_t n) { return (n + 3u) & ~size_t(3); }

template <typename T> bool Exact(const uint8_t* bytes, size_t length, T& value) {
    if (length != sizeof(T)) return false;
    std::memcpy(&value, bytes, sizeof(T)); return true;
}
template <typename T> bool Header(const uint8_t* bytes, size_t length, T& value) {
    if (length < sizeof(T)) return false;
    std::memcpy(&value, bytes, sizeof(T)); return true;
}

struct Transaction {
    SceneState* scene = nullptr;
    const uint8_t* records = nullptr;
    Change changes[GpuConfig::MAX_BATCH_COMMANDS];
    uint16_t changeCount = 0;
    void* reservations[GpuConfig::MAX_BATCH_COMMANDS * 2];
    uint16_t reservationCount = 0;
    gpumem::AssetToken externalReservations[GpuConfig::MAX_BATCH_COMMANDS];
    uint16_t externalReservationCount = 0;
    PendingDraw draws[GpuConfig::MAX_DRAW_CALLS];
    uint16_t drawCount = 0;
    Pending2D draws2D[PGL_MAX_2D_DRAW_CMDS];
    uint16_t drawCount2D = 0;
    PglSpritePosition spritePositions[256];
    uint16_t spriteCount = 0;
    bool referencedMeshes[GpuConfig::MAX_MESHES];
    bool referencedMaterials[GpuConfig::MAX_MATERIALS];
    bool referencedTextures[GpuConfig::MAX_TEXTURES];
    bool referencedLayers[GpuConfig::MAX_LAYERS];
    void* frameOwned[GpuConfig::MAX_DRAW_CALLS];
    uint8_t frameOwnedCount = 0;
    uint16_t overrideVertices = 0;
    ResourceUpload upload;
    bool uploadChanged = false;
    BatchInfo info;

    void Start(SceneState* target, bool resourceOnly, const uint8_t* batch) {
        scene = target; records = batch; changeCount = reservationCount = drawCount = drawCount2D = spriteCount = 0;
        frameOwnedCount = 0; overrideVertices = 0; info = {}; info.resourceOnly = resourceOnly;
        externalReservationCount = 0;
        upload = target->upload; uploadChanged = false;
        std::memset(referencedMeshes, 0, sizeof(referencedMeshes));
        std::memset(referencedMaterials, 0, sizeof(referencedMaterials));
        std::memset(referencedTextures, 0, sizeof(referencedTextures));
        std::memset(referencedLayers, 0, sizeof(referencedLayers));
    }
    void* Allocate(size_t size) {
        if (!size || reservationCount >= GpuConfig::MAX_BATCH_COMMANDS * 2) return nullptr;
        void* pointer = scene->SceneHeapAlloc(size);
        if (pointer) reservations[reservationCount++] = pointer;
        return pointer;
    }
    void Publish(void* pointer) {
        for (uint16_t i = 0; i < reservationCount; ++i) if (reservations[i] == pointer) {
            reservations[i] = nullptr; return; // Publication transfers ownership.
        }
    }
    void Finish() {
        for (uint16_t i = 0; i < reservationCount; ++i) scene->SceneHeapFree(reservations[i]);
        for (uint16_t i = 0; i < externalReservationCount; ++i)
            if (externalReservations[i] != gpumem::kAssetTokenInvalid)
                scene->externalAssets->destroy(gpumem::assetHandleFromToken(externalReservations[i]));
        externalReservationCount = 0;
        reservationCount = changeCount = 0;
    }
    R AllocateExternal(gpumem::AssetClass assetClass, uint32_t bytes, uint32_t& token) {
        if (!scene->externalAssets || !scene->externalAssets->hasBacking()) return R::Unsupported;
        if (externalReservationCount >= GpuConfig::MAX_BATCH_COMMANDS) return R::Capacity;
        gpumem::AssetHandle handle;
        const auto result = scene->externalAssets->create(assetClass, bytes, handle);
        if (result != gpumem::AssetStatus::Ok)
            return result == gpumem::AssetStatus::BackingError ? R::Io : R::Capacity;
        token = gpumem::assetTokenFromHandle(handle);
        externalReservations[externalReservationCount++] = token;
        return R::Ok;
    }
    void PublishExternal(uint32_t token) {
        for (uint16_t i = 0; i < externalReservationCount; ++i)
            if (externalReservations[i] == token) externalReservations[i] = gpumem::kAssetTokenInvalid;
    }
    Change* Find(Kind kind, uint16_t index) {
        for (uint16_t i = 0; i < changeCount; ++i) if (changes[i].kind == kind && changes[i].index == index) return &changes[i];
        return nullptr;
    }
    template <typename T> T* Base(Kind kind, uint16_t index) {
        switch (kind) {
            case Kind::Mesh: return index < GpuConfig::MAX_MESHES ? reinterpret_cast<T*>(&scene->meshes[index]) : nullptr;
            case Kind::Material: return index < GpuConfig::MAX_MATERIALS ? reinterpret_cast<T*>(&scene->materials[index]) : nullptr;
            case Kind::Texture: return index < GpuConfig::MAX_TEXTURES ? reinterpret_cast<T*>(&scene->textures[index]) : nullptr;
            case Kind::Layout: return index < PGL_MAX_LAYOUTS ? reinterpret_cast<T*>(&scene->pixelLayouts[index]) : nullptr;
            case Kind::Camera: return index < PGL_MAX_CAMERAS ? reinterpret_cast<T*>(&scene->cameras[index]) : nullptr;
            case Kind::Layer: return index < GpuConfig::MAX_LAYERS ? reinterpret_cast<T*>(&scene->layers[index]) : nullptr;
            case Kind::Shader: return index < GpuConfig::MAX_SHADER_PROGRAMS ? reinterpret_cast<T*>(&scene->shaderPrograms[index]) : nullptr;
        }
        return nullptr;
    }
    template <typename T> T* View(Kind kind, uint16_t index, bool allowDestroyed = false) {
        if (auto* change = Find(kind, index)) return (change->flags & Change::Destroy) && !allowDestroyed ? nullptr : static_cast<T*>(change->value);
        return Base<T>(kind, index);
    }
    template <typename T> T* Edit(Kind kind, uint16_t index, bool clear = false) {
        if (auto* change = Find(kind, index)) {
            if (change->flags & Change::Destroy) return nullptr;
            return static_cast<T*>(change->value);
        }
        const auto* original = Base<T>(kind, index);
        if (!original || changeCount >= GpuConfig::MAX_BATCH_COMMANDS) return nullptr;
        void* memory = Allocate(sizeof(T)); if (!memory) return nullptr;
        T* candidate = clear ? new (memory) T{} : new (memory) T(*original);
        changes[changeCount++] = {candidate, nullptr, kind, static_cast<uint8_t>(index), 0, 0};
        return candidate;
    }
    uint8_t Generation(Kind kind, uint16_t index) {
        if (auto* change = Find(kind, index); change && (change->flags & Change::HasGeneration)) return change->generation;
        if (kind == Kind::Mesh) return scene->meshGeneration[index];
        if (kind == Kind::Material) return scene->materialGeneration[index];
        return scene->textureGeneration[index];
    }
    bool Available(Kind kind, uint16_t handle) {
        const uint16_t index = PglHandleIndex(handle);
        if (index == PGL_INVALID_HANDLE_INDEX) return false;
        bool active = false;
        if (kind == Kind::Mesh) { auto* v = View<MeshSlot>(kind, index); active = v && v->active && !scene->pendingMeshDestroy[index]; }
        if (kind == Kind::Material) { auto* v = View<MaterialSlot>(kind, index); active = v && v->active && !scene->pendingMaterialDestroy[index]; }
        if (kind == Kind::Texture) { auto* v = View<TextureSlot>(kind, index); active = v && v->active && !scene->pendingTextureDestroy[index]; }
        return active && Generation(kind, index) == PglHandleGeneration(handle);
    }
    bool NewGeneration(Kind kind, uint16_t handle) {
        const uint16_t index = PglHandleIndex(handle);
        if (index == PGL_INVALID_HANDLE_INDEX || PglHandleGeneration(handle) == 0xff) return false;
        bool used; uint16_t bound;
        if (kind == Kind::Mesh) { used = index < GpuConfig::MAX_MESHES && scene->meshEverUsed[index]; bound = GpuConfig::MAX_MESHES; }
        else if (kind == Kind::Material) { used = index < GpuConfig::MAX_MATERIALS && scene->materialEverUsed[index]; bound = GpuConfig::MAX_MATERIALS; }
        else { used = index < GpuConfig::MAX_TEXTURES && scene->textureEverUsed[index]; bound = GpuConfig::MAX_TEXTURES; }
        if (index >= bound || Find(kind, index)) return false;
        return !used || PglHandleGeneration(handle) > Generation(kind, index);
    }
    void SetGeneration(Kind kind, uint16_t handle) {
        auto* change = Find(kind, PglHandleIndex(handle));
        change->generation = PglHandleGeneration(handle); change->flags |= Change::HasGeneration;
    }
    bool ReferenceTexture(uint16_t handle) {
        if (!Available(Kind::Texture, handle)) return false;
        referencedTextures[PglHandleIndex(handle)] = true; return true;
    }
    bool ReferenceMaterial(uint16_t handle, uint8_t depth = 0, uint64_t ancestors = 0) {
        if (depth > 3 || !Available(Kind::Material, handle)) return false;
        const uint8_t index = PglHandleIndex(handle);
        if (ancestors & (uint64_t(1) << index)) return false;
        ancestors |= uint64_t(1) << index;
        referencedMaterials[index] = true;
        const auto* mat = View<MaterialSlot>(Kind::Material, index);
        if (mat->type == PGL_MAT_IMAGE || mat->type == PGL_MAT_PRERENDERED) return ReferenceTexture(PglRuntime::Load16(mat->params));
        if (mat->type == PGL_MAT_COMBINE || mat->type == PGL_MAT_MASK || mat->type == PGL_MAT_ANIMATOR) {
            return ReferenceMaterial(PglRuntime::Load16(mat->params), depth + 1, ancestors) &&
                   ReferenceMaterial(PglRuntime::Load16(mat->params + 2), depth + 1, ancestors);
        }
        return true;
    }
    R CreateMesh(const uint8_t* bytes, size_t length, uint8_t* adoption = nullptr, bool cold = false) {
        PglCmdCreateMeshHeader header;
        if (!Header(bytes, length, header) || !header.vertexCount || header.vertexCount > GpuConfig::MAX_VERTICES ||
            !header.triangleCount || header.triangleCount > GpuConfig::MAX_SOURCE_TRIANGLES || (header.flags & ~PGL_MESH_HAS_UV)) return R::InvalidValue;
        const uint16_t index = PglHandleIndex(header.meshId);
        if (!NewGeneration(Kind::Mesh, header.meshId) || (index < GpuConfig::MAX_MESHES && referencedMeshes[index])) return R::InvalidHandle;
        const size_t vertexBytes = size_t(header.vertexCount) * sizeof(PglVec3), indexBytes = size_t(header.triangleCount) * sizeof(PglIndex3);
        size_t position = sizeof(header);
        if (vertexBytes + indexBytes > length - position || !FiniteRange(bytes + position, size_t(header.vertexCount)*3)) return R::BadPacket;
        const uint8_t* vertices = bytes + position; position += vertexBytes;
        const uint8_t* indices = bytes + position; position += indexBytes;
        for (size_t i = 0; i < size_t(header.triangleCount) * 3; ++i) if (PglRuntime::Load16(indices + 2*i) >= header.vertexCount) return R::InvalidValue;
        uint16_t uvCount = 0; const uint8_t* uvs = nullptr; const uint8_t* uvIndices = nullptr;
        if (header.flags & PGL_MESH_HAS_UV) {
            if (length - position < 2) return R::BadPacket;
            uvCount = PglRuntime::Load16(bytes + position); position += 2;
            if (!uvCount || uvCount > GpuConfig::MAX_VERTICES || size_t(uvCount)*8 + indexBytes > length - position) return R::BadPacket;
            uvs = bytes + position; position += size_t(uvCount)*8;
            uvIndices = bytes + position; position += indexBytes;
            if (!FiniteRange(uvs, size_t(uvCount)*2)) return R::InvalidValue;
            for (size_t i = 0; i < size_t(header.triangleCount)*3; ++i) if (PglRuntime::Load16(uvIndices + 2*i) >= uvCount) return R::InvalidValue;
        }
        if (position != length) return R::BadPacket;
        const size_t indexOffset = Align4(vertexBytes), uvOffset = Align4(indexOffset + indexBytes);
        const size_t uvIndexOffset = Align4(uvOffset + size_t(uvCount)*8);
        const size_t allocationBytes = uvCount ? uvIndexOffset + indexBytes : indexOffset + indexBytes;
        auto* mesh = Edit<MeshSlot>(Kind::Mesh, index, true); if (!mesh) return R::NoMemory;
        auto* storage = cold ? nullptr : (adoption ? adoption : static_cast<uint8_t*>(Allocate(allocationBytes)));
        if (!cold && !storage) return R::NoMemory;
        mesh->active = true; mesh->vertexCount = header.vertexCount; mesh->triangleCount = header.triangleCount;
        mesh->storage = storage; mesh->storageBytes = allocationBytes;
        mesh->vertices = reinterpret_cast<PglVec3*>(adoption || cold ? const_cast<uint8_t*>(vertices) : storage);
        mesh->indices = reinterpret_cast<PglIndex3*>(adoption || cold ? const_cast<uint8_t*>(indices) : storage + indexOffset);
        if (!adoption && !cold) { std::memcpy(mesh->vertices, vertices, vertexBytes); std::memcpy(mesh->indices, indices, indexBytes); }
        if (uvCount) {
            mesh->uvVertexCount = uvCount;
            mesh->uvVertices = reinterpret_cast<PglVec2*>(adoption || cold ? const_cast<uint8_t*>(uvs) : storage + uvOffset);
            mesh->uvIndices = reinterpret_cast<PglIndex3*>(adoption || cold ? const_cast<uint8_t*>(uvIndices) : storage + uvIndexOffset);
            if (!adoption && !cold) { std::memcpy(mesh->uvVertices, uvs, size_t(uvCount)*8); std::memcpy(mesh->uvIndices, uvIndices, indexBytes); }
        }
        if (adoption && !cold) Find(Kind::Mesh, index)->adoptedPayload = bytes;
        mesh->aabbMin = mesh->aabbMax = {FloatAt(vertices), FloatAt(vertices+4), FloatAt(vertices+8)};
        for (uint16_t i=1; i<header.vertexCount; ++i) {
            const PglVec3 v{FloatAt(vertices+size_t(i)*12),FloatAt(vertices+size_t(i)*12+4),FloatAt(vertices+size_t(i)*12+8)};
            if(v.x<mesh->aabbMin.x)mesh->aabbMin.x=v.x;
            if(v.x>mesh->aabbMax.x)mesh->aabbMax.x=v.x;
            if(v.y<mesh->aabbMin.y)mesh->aabbMin.y=v.y;
            if(v.y>mesh->aabbMax.y)mesh->aabbMax.y=v.y;
            if(v.z<mesh->aabbMin.z)mesh->aabbMin.z=v.z;
            if(v.z>mesh->aabbMax.z)mesh->aabbMax.z=v.z;
        }
        if (cold) {
            R result = AllocateExternal(gpumem::AssetClass::Mesh, allocationBytes, mesh->externalToken);
            if (result != R::Ok) return result;
            auto handle = gpumem::assetHandleFromToken(mesh->externalToken);
            auto write = [&](uint32_t offset, const void* source, uint32_t count) {
                return scene->externalAssets->write(handle,offset,source,count)==gpumem::AssetStatus::Ok;
            };
            if (!write(0,vertices,vertexBytes) || !write(indexOffset,indices,indexBytes) ||
                (uvCount && (!write(uvOffset,uvs,size_t(uvCount)*8) || !write(uvIndexOffset,uvIndices,indexBytes)))) return R::Io;
            mesh->vertices=nullptr;mesh->indices=nullptr;mesh->uvVertices=nullptr;mesh->uvIndices=nullptr;
        }
        SetGeneration(Kind::Mesh, header.meshId); return R::Ok;
    }
    R CopyMeshForUpdate(uint16_t handle, MeshSlot*& mesh) {
        if (!Available(Kind::Mesh, handle) || referencedMeshes[PglHandleIndex(handle)]) return R::InvalidHandle;
        const uint16_t index = PglHandleIndex(handle);
        mesh = Edit<MeshSlot>(Kind::Mesh, index); if (!mesh) return R::NoMemory;
        if (Find(Kind::Mesh, index)->adoptedPayload) return R::BadState;
        if (mesh->storage && mesh->storage != scene->meshes[index].storage) return R::Ok;
        const size_t vb = size_t(mesh->vertexCount)*12, ib = size_t(mesh->triangleCount)*6;
        const size_t io = Align4(vb), uo = Align4(io + ib), uio = Align4(uo + size_t(mesh->uvVertexCount)*8);
        const size_t total = mesh->uvVertexCount ? uio + ib : io + ib;
        auto* storage = static_cast<uint8_t*>(Allocate(total)); if (!storage) return R::NoMemory;
        if (mesh->externalToken != gpumem::kAssetTokenInvalid) {
            if (!scene->externalAssets || scene->externalAssets->read(gpumem::assetHandleFromToken(mesh->externalToken),0,storage,total)!=gpumem::AssetStatus::Ok) return R::Io;
            mesh->externalToken=gpumem::kAssetTokenInvalid;
        } else {
            std::memcpy(storage, mesh->vertices, vb); std::memcpy(storage + io, mesh->indices, ib);
            if (mesh->uvVertexCount) { std::memcpy(storage + uo, mesh->uvVertices, size_t(mesh->uvVertexCount)*8); std::memcpy(storage + uio, mesh->uvIndices, ib); }
        }
        mesh->storage = storage; mesh->storageBytes = total; mesh->vertices = reinterpret_cast<PglVec3*>(storage); mesh->indices = reinterpret_cast<PglIndex3*>(storage+io);
        if (mesh->uvVertexCount) { mesh->uvVertices = reinterpret_cast<PglVec2*>(storage+uo); mesh->uvIndices = reinterpret_cast<PglIndex3*>(storage+uio); }
        return R::Ok;
    }
    static size_t MaterialBase(const MaterialSlot& mat) {
        switch (mat.type) {
            case PGL_MAT_SIMPLE: return sizeof(PglParamSimple);
            case PGL_MAT_NORMAL: return 0;
            case PGL_MAT_DEPTH: return sizeof(PglParamDepth);
            case PGL_MAT_GRADIENT: return 1 + size_t(mat.params[0])*sizeof(PglGradientStop) + 9;
            case PGL_MAT_LIGHT: return sizeof(PglParamLight);
            case PGL_MAT_SIMPLEX_NOISE: return sizeof(PglParamSimplexNoise);
            case PGL_MAT_RAINBOW_NOISE: return sizeof(PglParamRainbowNoise);
            case PGL_MAT_IMAGE: return sizeof(PglParamImage);
            case PGL_MAT_COMBINE: return sizeof(PglParamCombine);
            case PGL_MAT_MASK: return sizeof(PglParamMask);
            case PGL_MAT_ANIMATOR: return sizeof(PglParamAnimator);
            case PGL_MAT_PRERENDERED: return sizeof(PglParamPreRendered);
            default: return 1000;
        }
    }
    R MaterialParams(MaterialSlot& mat, const uint8_t* bytes, size_t length) {
        if (length > sizeof(mat.params)) return R::Capacity;
        std::memset(mat.params, 0, sizeof(mat.params)); std::memcpy(mat.params, bytes, length); mat.paramBytes = length;
        size_t base = MaterialBase(mat);
        if (base > sizeof(mat.params) || (mat.blendMode != PGL_BLEND_ALPHA && length != base) ||
            (mat.blendMode == PGL_BLEND_ALPHA && length != base && length != base + 4)) return R::InvalidValue;
        mat.alpha = 1.0f;
        if (mat.blendMode == PGL_BLEND_ALPHA && length == base + 4) {
            mat.alpha = FloatAt(bytes + base);
            if (!Finite(mat.alpha) || mat.alpha < 0 || mat.alpha > 1) return R::InvalidValue;
        }
        switch (mat.type) {
            case PGL_MAT_DEPTH:
                if (!FiniteRange(bytes+6,2) || FloatAt(bytes+10) <= FloatAt(bytes+6)) return R::InvalidValue;
                break;
            case PGL_MAT_LIGHT:
                if (!FiniteRange(bytes,3)) return R::InvalidValue;
                break;
            case PGL_MAT_SIMPLEX_NOISE:
                if (!FiniteRange(bytes,4)) return R::InvalidValue;
                break;
            case PGL_MAT_RAINBOW_NOISE:
                if (!FiniteRange(bytes,2)) return R::InvalidValue;
                break;
            case PGL_MAT_IMAGE:
                if (!FiniteRange(bytes+2,4) || (bytes[18] & ~PGL_IMAGE_FILTER_BILINEAR) || bytes[19]) return R::InvalidValue;
                break;
            case PGL_MAT_COMBINE:
                if (bytes[4] > PGL_BLEND_ALPHA || !FiniteRange(bytes+5,1) || FloatAt(bytes+5) < 0 || FloatAt(bytes+5) > 1) return R::InvalidValue;
                break;
            case PGL_MAT_MASK:
                if (!FiniteRange(bytes+4,1) || FloatAt(bytes+4) < 0 || FloatAt(bytes+4) > 1) return R::InvalidValue;
                break;
            case PGL_MAT_ANIMATOR:
                if (bytes[4] > 1 || !FiniteRange(bytes+5,1) || FloatAt(bytes+5) < 0 || FloatAt(bytes+5) > 1) return R::InvalidValue;
                break;
            case PGL_MAT_GRADIENT: {
                const uint8_t count = bytes[0]; if (count < 2 || count > 7) return R::InvalidValue;
                float previous = -1.0f;
                for (uint8_t i=0;i<count;++i) { float position = FloatAt(bytes+1+size_t(i)*7); if (!Finite(position) || position < previous || position < 0 || position > 1) return R::InvalidValue; previous = position; }
                const uint8_t* tail = bytes+1+size_t(count)*7;
                if (tail[0] > 2 || !FiniteRange(tail+1,2) || FloatAt(tail+5) <= FloatAt(tail+1)) return R::InvalidValue;
                break;
            }
            default: break;
        }
        return R::Ok;
    }
    R CreateMaterial(const uint8_t* bytes, size_t length) {
        PglCmdCreateMaterialHeader h; if (!Header(bytes,length,h)) return R::BadPacket;
        if (!NewGeneration(Kind::Material,h.materialId) || referencedMaterials[PglHandleIndex(h.materialId)] || h.blendMode > PGL_BLEND_ALPHA) return R::InvalidHandle;
        auto* material = Edit<MaterialSlot>(Kind::Material,PglHandleIndex(h.materialId),true); if (!material) return R::NoMemory;
        material->active=true; material->type=static_cast<PglMaterialType>(h.materialType); material->blendMode=static_cast<PglBlendMode>(h.blendMode);
        const auto result = MaterialParams(*material,bytes+sizeof(h),length-sizeof(h)); if (result != R::Ok) return result;
        SetGeneration(Kind::Material,h.materialId); return R::Ok;
    }
    R CreateTexture(const uint8_t* bytes,size_t length,uint8_t* adoption = nullptr,bool cold = false) {
        PglCmdCreateTextureHeader h; if (!Header(bytes,length,h) || !h.width || !h.height || h.width > GpuConfig::MAX_TEXTURE_DIMENSION || h.height > GpuConfig::MAX_TEXTURE_DIMENSION || h.format > PGL_TEX_RGB888) return R::InvalidValue;
        const size_t dataBytes=size_t(h.width)*h.height*(h.format==PGL_TEX_RGB565?2:3);
        if (dataBytes > GpuConfig::TEXTURE_POOL_SIZE || length-sizeof(h)!=dataBytes) return R::Capacity;
        if (!NewGeneration(Kind::Texture,h.textureId) || referencedTextures[PglHandleIndex(h.textureId)]) return R::InvalidHandle;
        auto* texture=Edit<TextureSlot>(Kind::Texture,PglHandleIndex(h.textureId),true); if (!texture) return R::NoMemory;
        auto* pixels = cold ? nullptr : (adoption ? const_cast<uint8_t*>(bytes + sizeof(h)) : static_cast<uint8_t*>(Allocate(dataBytes)));
        if(!cold && !pixels) return R::NoMemory;
        if (!adoption && !cold) std::memcpy(pixels,bytes+sizeof(h),dataBytes);
        texture->active=true; texture->width=h.width; texture->height=h.height;
        texture->format=static_cast<PglTextureFormat>(h.format); texture->pixelDataSize=dataBytes; texture->pixels=pixels; texture->storage=cold?nullptr:(adoption?adoption:pixels);
        if (adoption && !cold) Find(Kind::Texture, PglHandleIndex(h.textureId))->adoptedPayload = bytes;
        if (cold) {
            R result = AllocateExternal(gpumem::AssetClass::Texture,dataBytes,texture->externalToken);
            if(result!=R::Ok)return result;
            if(scene->externalAssets->write(gpumem::assetHandleFromToken(texture->externalToken),0,bytes+sizeof(h),dataBytes)!=gpumem::AssetStatus::Ok)return R::Io;
        }
        SetGeneration(Kind::Texture,h.textureId); return R::Ok;
    }
    R Destroy(Kind kind,uint16_t handle) {
        if (!Available(kind,handle)) return R::InvalidHandle;
        const uint16_t index=PglHandleIndex(handle);
        void* candidate=nullptr;
        if(kind==Kind::Mesh) candidate=Edit<MeshSlot>(kind,index);
        if(kind==Kind::Material) candidate=Edit<MaterialSlot>(kind,index);
        if(kind==Kind::Texture) candidate=Edit<TextureSlot>(kind,index);
        if(!candidate) return R::NoMemory;
        Find(kind,index)->flags |= Change::Destroy; return R::Ok;
    }
    R Queue(uint8_t layer,DrawCmd2D command) {
        if(info.resourceOnly) return R::BadState;
        if(drawCount2D>=PGL_MAX_2D_DRAW_CMDS) return R::Capacity;
        auto* target=View<LayerSlot>(Kind::Layer,layer);
        if(!target || (!layer ? false : (!target->active || !target->pixels))) return R::InvalidHandle;
        referencedLayers[layer]=true;
        command.layerId=layer; command.clipX=target->clipX; command.clipY=target->clipY; command.clipW=target->clipW; command.clipH=target->clipH;
        command.viewOffsetX=target->viewOffX; command.viewOffsetY=target->viewOffY; command.viewScaleXQ8=target->viewScaleXQ8; command.viewScaleYQ8=target->viewScaleYQ8;
        ::new (static_cast<void*>(&draws2D[drawCount2D++].value)) DrawCmd2D(command); return R::Ok;
    }
    R ShaderSlotValue(ShaderSlot& slot,uint8_t shaderClass,float intensity,const uint8_t* params,uint16_t programId) {
        if(shaderClass>PGL_SHADER_PROGRAM || !Finite(intensity) || intensity<0 || intensity>1) return R::InvalidValue;
        slot={}; slot.active=shaderClass!=PGL_SHADER_NONE; slot.shaderClass=shaderClass; slot.intensity=intensity;
        slot.programId=programId; std::memcpy(slot.params,params,sizeof(slot.params));
        if(shaderClass==PGL_SHADER_PROGRAM) { auto* program=View<ShaderProgram>(Kind::Shader,programId); if(!program || !program->active || !program->verified) return R::InvalidHandle; }
        return R::Ok;
    }
    R BeginUpload(const uint8_t* bytes, size_t length) {
        PglCmdStreamBegin h;
        if (!info.resourceOnly || !Exact(bytes, length, h) || upload.active) return R::BadState;
        if (h.resourceClass > PGL_RES_CLASS_TEXTURE || !h.byteLength ||
            h.byteLength > GpuConfig::SCENE_HEAP_MAX_BYTES - 256 ||
            (h.preferredTier != 0 && h.preferredTier != 1 && h.preferredTier != 0xff)) return R::InvalidValue;
        if (h.preferredTier == 1 && (!scene->externalAssets || !scene->externalAssets->hasBacking())) return R::Unsupported;
        auto* buffer = static_cast<uint8_t*>(Allocate(size_t(h.byteLength) + 16));
        if (!buffer) return R::NoMemory;
        upload = {}; upload.active = true; upload.resourceClass = h.resourceClass;
        upload.resourceId = h.resourceId; upload.preferredTier = h.preferredTier;
        upload.bytes = h.byteLength; upload.checksum = h.checksum; upload.storage = buffer;
        uploadChanged = true; return R::Ok;
    }
    R AppendUpload(const uint8_t* bytes, size_t length) {
        PglCmdStreamDataHeader h;
        if (!info.resourceOnly || !Header(bytes, length, h) || !upload.active ||
            h.resourceClass != upload.resourceClass || h.resourceId != upload.resourceId ||
            h.offset != upload.received || !h.byteLength || length != sizeof(h) + h.byteLength ||
            h.byteLength > upload.bytes - upload.received) return R::BadPacket;
        // Append-only staging is not a published resource. On failure its
        // received cursor is not committed; a retry rewrites this same tail.
        std::memcpy(upload.storage + h.offset, bytes + sizeof(h), h.byteLength);
        upload.received += h.byteLength; uploadChanged = true; return R::Ok;
    }
    R CommitUpload(const uint8_t* bytes, size_t length) {
        PglCmdStreamCommit h;
        if (!info.resourceOnly || !Exact(bytes, length, h) || !upload.active ||
            h.resourceClass != upload.resourceClass || h.resourceId != upload.resourceId ||
            upload.received != upload.bytes || PglRuntime::PayloadChecksum(upload.storage, upload.bytes) != upload.checksum)
            return R::BadPacket;
        if (upload.bytes < 2 || PglRuntime::Load16(upload.storage) != upload.resourceId) return R::InvalidHandle;
        R result = R::Unsupported;
        const bool cold=upload.preferredTier==1;
        if (h.resourceClass == PGL_RES_CLASS_MESH) result = CreateMesh(upload.storage, upload.bytes, upload.storage,cold);
        if (h.resourceClass == PGL_RES_CLASS_TEXTURE) result = CreateTexture(upload.storage, upload.bytes, upload.storage,cold);
        if (h.resourceClass == PGL_RES_CLASS_MATERIAL) result = CreateMaterial(upload.storage, upload.bytes);
        if (result != R::Ok) return result;
        upload.active = false; upload.storage = nullptr; uploadChanged = true; return R::Ok;
    }
    R Command(uint8_t opcode,const uint8_t* bytes,size_t length) {
        switch(opcode) {
            case PGL_CMD_STREAM_BEGIN: return BeginUpload(bytes, length);
            case PGL_CMD_STREAM_DATA: return AppendUpload(bytes, length);
            case PGL_CMD_STREAM_COMMIT: return CommitUpload(bytes, length);
            case PGL_CMD_BEGIN_FRAME: { PglCmdBeginFrame p; if(!Exact(bytes,length,p))return R::BadPacket; info.frameNumber=p.frameNumber; info.frameTimeUs=p.frameTimeUs; return R::Ok; }
            case PGL_CMD_END_FRAME: { PglCmdEndFrame p; return Exact(bytes,length,p)&&p.frameNumber==info.frameNumber?R::Ok:R::BadPacket; }
            case PGL_CMD_CREATE_MESH:return CreateMesh(bytes,length);
            case PGL_CMD_CREATE_MATERIAL:return CreateMaterial(bytes,length);
            case PGL_CMD_CREATE_TEXTURE:return CreateTexture(bytes,length);
            case PGL_CMD_DESTROY_MESH: return length==2?Destroy(Kind::Mesh,PglRuntime::Load16(bytes)):R::BadPacket;
            case PGL_CMD_DESTROY_MATERIAL:return length==2?Destroy(Kind::Material,PglRuntime::Load16(bytes)):R::BadPacket;
            case PGL_CMD_DESTROY_TEXTURE:return length==2?Destroy(Kind::Texture,PglRuntime::Load16(bytes)):R::BadPacket;
            case PGL_CMD_UPDATE_VERTICES: {
                PglCmdUpdateVerticesHeader h; if(!Header(bytes,length,h))return R::BadPacket;
                auto* original=View<MeshSlot>(Kind::Mesh,PglHandleIndex(h.meshId));
                if(!original || h.vertexCount!=original->vertexCount || length!=sizeof(h)+size_t(h.vertexCount)*12 || !FiniteRange(bytes+sizeof(h),size_t(h.vertexCount)*3))return R::InvalidValue;
                MeshSlot* mesh; R r=CopyMeshForUpdate(h.meshId,mesh); if(r!=R::Ok)return r;
                std::memcpy(mesh->vertices,bytes+sizeof(h),size_t(h.vertexCount)*12); mesh->RecomputeAABB(); return R::Ok;
            }
            case PGL_CMD_UPDATE_VERTICES_DELTA: {
                PglCmdUpdateVerticesDeltaHeader h; if(!Header(bytes,length,h)||length!=sizeof(h)+size_t(h.deltaCount)*sizeof(PglVertexDelta))return R::BadPacket;
                MeshSlot* mesh; R r=CopyMeshForUpdate(h.meshId,mesh); if(r!=R::Ok)return r;
                for(uint16_t i=0;i<h.deltaCount;++i) { const uint8_t* p=bytes+sizeof(h)+size_t(i)*14;uint16_t vertex=PglRuntime::Load16(p);if(vertex>=mesh->vertexCount || !FiniteRange(p+2,3))return R::InvalidValue; mesh->vertices[vertex]={FloatAt(p+2),FloatAt(p+6),FloatAt(p+10)}; }
                mesh->RecomputeAABB(); return R::Ok;
            }
            case PGL_CMD_UPDATE_MATERIAL: {
                if(length<2)return R::BadPacket;
                uint16_t handle=PglRuntime::Load16(bytes);
                if(!Available(Kind::Material,handle)||referencedMaterials[PglHandleIndex(handle)])return R::InvalidHandle;
                auto* material=Edit<MaterialSlot>(Kind::Material,PglHandleIndex(handle)); return material?MaterialParams(*material,bytes+2,length-2):R::NoMemory;
            }
            case PGL_CMD_SET_PIXEL_LAYOUT: {
                PglCmdSetPixelLayoutHeader h;if(!Header(bytes,length,h)||h.layoutId>=PGL_MAX_LAYOUTS||!h.pixelCount||h.pixelCount>GpuConfig::FRAMEBUF_PIXELS||(h.flags&~3u))return R::InvalidValue;
                auto* layout=Edit<PixelLayoutSlot>(Kind::Layout,h.layoutId,true);if(!layout)return R::NoMemory;
                layout->active=true;layout->pixelCount=h.pixelCount;layout->flags=h.flags;
                if(h.flags&PGL_LAYOUT_RECTANGULAR) { if(length!=sizeof(h)+sizeof(PglRectLayoutData))return R::BadPacket;std::memcpy(&layout->rectData,bytes+sizeof(h),sizeof(PglRectLayoutData));const auto&r=layout->rectData;if(!r.rowCount||!r.colCount||uint32_t(r.rowCount)*r.colCount!=h.pixelCount||!FiniteRange(bytes+sizeof(h),4)||r.size.x<=0||r.size.y<=0)return R::InvalidValue; }
                else { if(h.pixelCount>GpuConfig::LAYOUT_COORD_POOL_SIZE||length!=sizeof(h)+size_t(h.pixelCount)*8||!FiniteRange(bytes+sizeof(h),size_t(h.pixelCount)*2))return R::InvalidValue;layout->coords=static_cast<PglVec2*>(Allocate(size_t(h.pixelCount)*8));if(!layout->coords)return R::NoMemory;std::memcpy(layout->coords,bytes+sizeof(h),size_t(h.pixelCount)*8); }
                return R::Ok;
            }
            case PGL_CMD_SET_CAMERA: {
                if(info.resourceOnly)return R::BadState;
                PglCmdSetCamera p;
                if(!Exact(bytes,length,p)||p.cameraId>=PGL_MAX_CAMERAS||p.pixelLayoutId>=PGL_MAX_LAYOUTS||p.is2D>1||!Quat(p.rotation)||!Quat(p.lookOffset)||!Quat(p.baseRotation)||!FiniteRange(reinterpret_cast<const uint8_t*>(&p.position),3)||!FiniteRange(reinterpret_cast<const uint8_t*>(&p.scale),3)||p.scale.x==0||p.scale.y==0||p.scale.z==0)return R::InvalidValue;
                auto* cam=Edit<CameraSlot>(Kind::Camera,p.cameraId);if(!cam)return R::NoMemory;
                cam->active=true;cam->layoutId=p.pixelLayoutId;cam->position=p.position;cam->rotation=p.rotation;cam->lookOffset=p.lookOffset;cam->baseRotation=p.baseRotation;cam->scale=p.scale;cam->is2D=p.is2D;return R::Ok;
            }
            case PGL_CMD_SET_CAMERA_TARGET: {
                if(info.resourceOnly)return R::BadState;
                PglCmdSetCameraTarget p;
                if(!Exact(bytes,length,p)||p.cameraId>=PGL_MAX_CAMERAS||p.targetLayer>=GpuConfig::MAX_LAYERS||(p.flags&~PGL_CAMERA_TARGET_SCISSOR)||p.reserved)return R::InvalidValue;
                if(p.targetLayer) { auto*l=View<LayerSlot>(Kind::Layer,p.targetLayer);if(!l||!l->active)return R::InvalidHandle;referencedLayers[p.targetLayer]=true; }
                auto*cam=Edit<CameraSlot>(Kind::Camera,p.cameraId);if(!cam)return R::NoMemory;cam->targetLayer=p.targetLayer;cam->vpFlags=p.flags;cam->vpX=p.vpX;cam->vpY=p.vpY;cam->vpW=p.vpW;cam->vpH=p.vpH;return R::Ok;
            }
            case PGL_CMD_DRAW_OBJECT: {
                if(info.resourceOnly)return R::BadState;
                PglCmdDrawObject p;
                if(!Header(bytes,length,p)||drawCount>=GpuConfig::MAX_DRAW_CALLS||(p.flags&~3u)||!Available(Kind::Mesh,p.meshId)||!ReferenceMaterial(p.materialId))return R::InvalidHandle;
                PglTransform transform={p.position,p.rotation,p.scale,p.baseRotation,p.scaleRotationOffset,p.scaleOffset,p.rotationOffset};if(!Transform(transform))return R::InvalidValue;
                PendingDraw draw{nullptr, static_cast<uint16_t>(bytes-records), 0};
                const uint16_t meshIndex=PglHandleIndex(p.meshId); referencedMeshes[meshIndex]=true;
                if(p.flags&PGL_DRAW_VERTEX_OVERRIDE) { if(length<sizeof(p)+2)return R::BadPacket;uint16_t n=PglRuntime::Load16(bytes+sizeof(p));auto*mesh=View<MeshSlot>(Kind::Mesh,meshIndex);if(n!=mesh->vertexCount||n>GpuConfig::FRAME_VERTEX_POOL_SIZE-overrideVertices||length!=sizeof(p)+2+size_t(n)*12||!FiniteRange(bytes+sizeof(p)+2,size_t(n)*3)||frameOwnedCount>=GpuConfig::MAX_DRAW_CALLS)return R::InvalidValue;auto*v=static_cast<PglVec3*>(Allocate(size_t(n)*12));if(!v)return R::NoMemory;std::memcpy(v,bytes+sizeof(p)+2,size_t(n)*12);draw.overrideVertexCount=n;draw.overrideVertices=v;frameOwned[frameOwnedCount++]=v;overrideVertices+=n; }
                else if(length!=sizeof(p))return R::BadPacket;
                draws[drawCount++]=draw;return R::Ok;
            }
            case PGL_CMD_CREATE_SHADER_PROGRAM: {
                PglCmdCreateShaderProgramHeader h;if(!Header(bytes,length,h)||h.programId>=GpuConfig::MAX_SHADER_PROGRAMS||length-sizeof(h)!=h.bytecodeSize)return R::BadPacket;
                auto*program=Edit<ShaderProgram>(Kind::Shader,h.programId,true);if(!program)return R::NoMemory;return DecodeShaderProgram(bytes+sizeof(h),h.bytecodeSize,h.programId,*program);
            }
            case PGL_CMD_DESTROY_SHADER_PROGRAM: {
                PglCmdDestroyShaderProgram p;if(!Exact(bytes,length,p)||p.programId>=GpuConfig::MAX_SHADER_PROGRAMS)return R::InvalidValue;auto*program=Edit<ShaderProgram>(Kind::Shader,p.programId);if(!program)return R::NoMemory;program->active=false;program->verified=false;return R::Ok;
            }
            case PGL_CMD_SET_SHADER_UNIFORM: {
                PglCmdSetShaderUniformHeader h;if(!Header(bytes,length,h)||h.programId>=GpuConfig::MAX_SHADER_PROGRAMS||h.uniformSlot<PSB_USER_UNIFORM_START||!h.componentCount||h.componentCount>4||h.uniformSlot+h.componentCount>PSB_MAX_UNIFORMS||length!=sizeof(h)+size_t(h.componentCount)*4||!FiniteRange(bytes+sizeof(h),h.componentCount))return R::InvalidValue;
                auto*program=Edit<ShaderProgram>(Kind::Shader,h.programId);
                if(!program||!program->active||!program->verified)return R::InvalidHandle;
                for(uint8_t i=0;i<h.componentCount;++i)
                    program->uniforms[h.uniformSlot+i]=FloatAt(bytes+sizeof(h)+4*i);
                return R::Ok;
            }
            case PGL_CMD_SET_SHADER: {
                if(info.resourceOnly)return R::BadState;
                PglCmdSetShader p;
                if(!Exact(bytes,length,p)||p.cameraId>=PGL_MAX_CAMERAS||p.shaderSlot>=PGL_MAX_SHADERS_PER_CAMERA)return R::InvalidValue;
                auto*cam=Edit<CameraSlot>(Kind::Camera,p.cameraId);
                if(!cam)return R::NoMemory;
                return ShaderSlotValue(cam->shaders[p.shaderSlot],p.shaderClass,p.intensity,p.params,0);
            }
            case PGL_CMD_BIND_SHADER_PROGRAM: {
                if(info.resourceOnly)return R::BadState;
                PglCmdBindShaderProgram p;
                if(!Exact(bytes,length,p)||p.cameraId>=PGL_MAX_CAMERAS||p.shaderSlot>=PGL_MAX_SHADERS_PER_CAMERA)return R::InvalidValue;
                auto*cam=Edit<CameraSlot>(Kind::Camera,p.cameraId);
                if(!cam)return R::NoMemory;
                uint8_t zero[20]={};
                return ShaderSlotValue(cam->shaders[p.shaderSlot],p.programId==0xffff?PGL_SHADER_NONE:PGL_SHADER_PROGRAM,p.intensity,zero,p.programId==0xffff?0:p.programId);
            }
            case PGL_CMD_LAYER_CREATE: {
                PglCmdLayerCreate p;if(!Exact(bytes,length,p)||!p.layerId||p.layerId>=GpuConfig::MAX_LAYERS||!p.width||!p.height||uint32_t(p.width)*p.height>GpuConfig::FRAMEBUF_PIXELS||p.pixelFormat!=PGL_PIXFMT_RGB565||p.blendMode>PGL_LAYER_BLEND_MULTIPLY||referencedLayers[p.layerId])return R::InvalidValue;
                auto*layer=Edit<LayerSlot>(Kind::Layer,p.layerId,true);if(!layer)return R::NoMemory;layer->pixels=static_cast<uint16_t*>(Allocate(size_t(p.width)*p.height*2));if(!layer->pixels)return R::NoMemory;
                std::memset(layer->pixels,0,size_t(p.width)*p.height*2);layer->active=true;layer->width=p.width;layer->height=p.height;layer->blendMode=p.blendMode;layer->opacity=p.opacity;layer->clipW=p.width;layer->clipH=p.height;return R::Ok;
            }
            case PGL_CMD_LAYER_DESTROY: {
                PglCmdLayerDestroy p;if(!Exact(bytes,length,p)||!p.layerId||p.layerId>=GpuConfig::MAX_LAYERS||referencedLayers[p.layerId])return R::InvalidValue;auto*layer=Edit<LayerSlot>(Kind::Layer,p.layerId);if(!layer||!layer->active)return R::InvalidHandle;Find(Kind::Layer,p.layerId)->flags |= Change::Destroy;return R::Ok;
            }
            case PGL_CMD_LAYER_SET_PROPS: {
                PglCmdLayerSetProps p;if(!Exact(bytes,length,p)||!p.layerId||p.layerId>=GpuConfig::MAX_LAYERS||p.blendMode>PGL_LAYER_BLEND_MULTIPLY)return R::InvalidValue;auto*layer=Edit<LayerSlot>(Kind::Layer,p.layerId);if(!layer||!layer->active)return R::InvalidHandle;layer->opacity=p.opacity;layer->blendMode=p.blendMode;layer->offsetX=p.offsetX;layer->offsetY=p.offsetY;return R::Ok;
            }
            case PGL_CMD_LAYER_SET_VISIBILITY: { PglCmdLayerSetVisibility p;if(!Exact(bytes,length,p)||!p.layerId||p.layerId>=GpuConfig::MAX_LAYERS||p.visible>1)return R::InvalidValue;auto*layer=Edit<LayerSlot>(Kind::Layer,p.layerId);if(!layer||!layer->active)return R::InvalidHandle;layer->visible=p.visible;return R::Ok; }
            case PGL_CMD_SET_CLIP_RECT: { PglCmdSetClipRect p;if(!Exact(bytes,length,p)||p.layerId>=GpuConfig::MAX_LAYERS)return R::InvalidValue;auto*layer=Edit<LayerSlot>(Kind::Layer,p.layerId);if(!layer||!layer->active)return R::InvalidHandle;layer->clipX=p.x;layer->clipY=p.y;layer->clipW=p.w;layer->clipH=p.h;return R::Ok; }
            case PGL_CMD_SET_VIEWPORT: { PglCmdSetViewport p;if(!Exact(bytes,length,p)||p.layerId>=GpuConfig::MAX_LAYERS||p.scaleXQ8<=0||p.scaleYQ8<=0)return R::InvalidValue;auto*layer=Edit<LayerSlot>(Kind::Layer,p.layerId);if(!layer||!layer->active)return R::InvalidHandle;layer->viewOffX=p.offsetX;layer->viewOffY=p.offsetY;layer->viewScaleXQ8=p.scaleXQ8;layer->viewScaleYQ8=p.scaleYQ8;return R::Ok; }
            case PGL_CMD_SET_LAYER_SHADER: { PglCmdSetLayerShader p;if(!Exact(bytes,length,p)||!p.layerId||p.layerId>=GpuConfig::MAX_LAYERS||p.shaderSlot>=PGL_MAX_SHADERS_PER_CAMERA)return R::InvalidValue;auto*layer=Edit<LayerSlot>(Kind::Layer,p.layerId);if(!layer||!layer->active)return R::InvalidHandle;return ShaderSlotValue(layer->shaders[p.shaderSlot],p.shaderClass,p.intensity,p.params,p.programId); }
            case PGL_CMD_DRAW_RECT_2D: { DrawCmd2D c;c.type=DRAW_CMD_2D_RECT;if(!Exact(bytes,length,c.rect)||c.rect.filled>1)return R::InvalidValue;return Queue(c.rect.layerId,c); }
            case PGL_CMD_DRAW_LINE_2D: { DrawCmd2D c;c.type=DRAW_CMD_2D_LINE;if(!Exact(bytes,length,c.line))return R::BadPacket;return Queue(c.line.layerId,c); }
            case PGL_CMD_DRAW_CIRCLE_2D: { DrawCmd2D c;c.type=DRAW_CMD_2D_CIRCLE;if(!Exact(bytes,length,c.circle)||c.circle.filled>1)return R::InvalidValue;return Queue(c.circle.layerId,c); }
            case PGL_CMD_DRAW_ROUNDED_RECT: { DrawCmd2D c;c.type=DRAW_CMD_2D_ROUNDED_RECT;if(!Exact(bytes,length,c.roundedRect)||c.roundedRect.filled>1)return R::InvalidValue;return Queue(c.roundedRect.layerId,c); }
            case PGL_CMD_DRAW_ARC: { DrawCmd2D c;c.type=DRAW_CMD_2D_ARC;if(!Exact(bytes,length,c.arc))return R::BadPacket;return Queue(c.arc.layerId,c); }
            case PGL_CMD_DRAW_TRIANGLE_2D: { DrawCmd2D c;c.type=DRAW_CMD_2D_TRIANGLE;if(!Exact(bytes,length,c.triangle))return R::BadPacket;return Queue(c.triangle.layerId,c); }
            case PGL_CMD_LAYER_CLEAR: { DrawCmd2D c;c.type=DRAW_CMD_2D_CLEAR;if(!Exact(bytes,length,c.clear))return R::BadPacket;return Queue(c.clear.layerId,c); }
            case PGL_CMD_DRAW_GRADIENT_RECT: { DrawCmd2D c;c.type=DRAW_CMD_2D_GRADIENT_RECT;if(!Exact(bytes,length,c.gradient)||c.gradient.direction>1)return R::InvalidValue;return Queue(c.gradient.layerId,c); }
            case PGL_CMD_DRAW_SPRITE: { DrawCmd2D c;c.type=DRAW_CMD_2D_SPRITE;if(!Exact(bytes,length,c.sprite)||(c.sprite.flags&~3u)||!ReferenceTexture(c.sprite.textureId))return R::InvalidValue;return Queue(c.sprite.layerId,c); }
            case PGL_CMD_DRAW_SPRITE_BATCH: { PglCmdDrawSpriteBatchHeader h;if(!Header(bytes,length,h)||(h.flags&~3u)||h.count>256-spriteCount||length!=sizeof(h)+size_t(h.count)*4||!ReferenceTexture(h.textureId))return R::InvalidValue;DrawCmd2D c;c.type=DRAW_CMD_2D_SPRITE_BATCH;c.spriteBatch={h.textureId,spriteCount,h.count,h.flags};std::memcpy(spritePositions+spriteCount,bytes+sizeof(h),size_t(h.count)*4);spriteCount+=h.count;return Queue(h.layerId,c); }
            case PGL_CMD_DRAW_TEXT: {
                PglCmdDrawTextHeader h;if(!Header(bytes,length,h)||!h.glyphWidth||!h.glyphHeight||!h.columns||length!=sizeof(h)+h.textLength||h.textLength>1024||!ReferenceTexture(h.fontTextureId))return R::InvalidValue;const auto*texture=View<TextureSlot>(Kind::Texture,PglHandleIndex(h.fontTextureId));if(texture->format!=PGL_TEX_RGB565||h.columns>texture->width/h.glyphWidth||!texture->height||h.glyphHeight>texture->height)return R::InvalidValue;
                const uint32_t cells=uint32_t(h.columns)*(texture->height/h.glyphHeight);for(uint16_t i=0;i<h.textLength;++i){uint8_t ch=bytes[sizeof(h)+i];if(ch!='\n'&&(ch<h.firstCharacter||uint32_t(ch-h.firstCharacter)>=cells))return R::InvalidValue;}
                DrawCmd2D c;c.type=DRAW_CMD_2D_TEXT;c.text={h.fontTextureId,h.x,h.y,h.glyphWidth,h.glyphHeight,h.columns,h.firstCharacter,h.color,h.textLength,reinterpret_cast<const char*>(bytes+sizeof(h))};return Queue(h.layerId,c);
            }
            default:return R::Unsupported;
        }
    }
    R CheckBudget() {
        if (info.resourceOnly) return R::Ok;
        uint64_t operations = 0;
        auto add = [&](const ShaderSlot* slots, uint32_t pixels) -> R {
            for (size_t i=0; i<PGL_MAX_SHADERS_PER_CAMERA; ++i) {
                const auto& slot=slots[i]; if (!slot.active || slot.intensity<=0) continue;
                uint64_t cost;
                if (slot.shaderClass==PGL_SHADER_PROGRAM) {
                    auto* program=View<ShaderProgram>(Kind::Shader,slot.programId);
                    if (!program || !program->active || !program->verified) return R::InvalidHandle;
                    cost=uint64_t(program->weightedCost)*pixels;
                } else {
                    uint32_t estimated=ScreenspaceShaders::EstimateSlotWeightedOps(scene,slot,pixels);
                    if (estimated==0xffffffffu) return R::InvalidValue;
                    cost=estimated;
                }
                operations+=cost; if (operations>GpuConfig::POSTFX_WORK_BUDGET) return R::Capacity;
            }
            return R::Ok;
        };
        for (uint8_t i=0; i<PGL_MAX_CAMERAS; ++i) {
            const auto* cam=View<CameraSlot>(Kind::Camera,i); if (!cam || !cam->active) continue;
            uint16_t w=scene->renderWidth,h=scene->renderHeight;
            if (cam->targetLayer) {
                auto* layer=View<LayerSlot>(Kind::Layer,cam->targetLayer);
                if (!layer || !layer->active || !layer->pixels) return R::InvalidHandle;
                w=layer->width;h=layer->height;
            }
            if (uint32_t((w+15)/16)*((h+15)/16)>64) return R::Capacity;
            uint32_t pixels=uint32_t(w)*h;
            if (cam->vpFlags&PGL_CAMERA_TARGET_SCISSOR) {
                uint32_t x0=cam->vpX<w?cam->vpX:w,y0=cam->vpY<h?cam->vpY:h;
                uint32_t x1=uint32_t(cam->vpX)+cam->vpW,y1=uint32_t(cam->vpY)+cam->vpH;
                if(x1>w)x1=w;
                if(y1>h)y1=h;
                pixels=(x1>x0&&y1>y0)?(x1-x0)*(y1-y0):0;
            }
            R result=add(cam->shaders,pixels);if(result!=R::Ok)return result;
        }
        for(uint8_t i=1;i<GpuConfig::MAX_LAYERS;++i) {
            const auto* layer=View<LayerSlot>(Kind::Layer,i);if(!layer||!layer->active)continue;
            R result=add(layer->shaders,uint32_t(layer->width)*layer->height);if(result!=R::Ok)return result;
        }
        return R::Ok;
    }
    void Commit() {
        // Layout compaction happens only after the complete transaction passes.
        // The stream allocation becomes the resource; no duplicate asset block.
        for (uint16_t i=0; i<changeCount; ++i) {
            auto& c = changes[i]; if (!c.adoptedPayload) continue;
            if (c.kind == Kind::Mesh) {
                auto& m = *static_cast<MeshSlot*>(c.value); auto* base = static_cast<uint8_t*>(m.storage);
                const size_t vb=size_t(m.vertexCount)*12, ib=size_t(m.triangleCount)*6;
                const size_t io=Align4(vb), uo=Align4(io+ib), uio=Align4(uo+size_t(m.uvVertexCount)*8);
                std::memmove(base, m.vertices, vb); std::memmove(base+io, m.indices, ib);
                if (m.uvVertexCount) { std::memmove(base+uo, m.uvVertices, size_t(m.uvVertexCount)*8); std::memmove(base+uio, m.uvIndices, ib); }
                m.vertices=reinterpret_cast<PglVec3*>(base); m.indices=reinterpret_cast<PglIndex3*>(base+io);
                if (m.uvVertexCount) { m.uvVertices=reinterpret_cast<PglVec2*>(base+uo); m.uvIndices=reinterpret_cast<PglIndex3*>(base+uio); }
            } else if (c.kind == Kind::Texture) {
                auto& t=*static_cast<TextureSlot*>(c.value); std::memmove(t.storage,t.pixels,t.pixelDataSize); t.pixels=static_cast<uint8_t*>(t.storage);
            }
        }
        if (uploadChanged) {
            bool adopted=false;
            for(uint16_t i=0;i<changeCount;++i) {
                const auto& change=changes[i];
                if(change.kind==Kind::Mesh && static_cast<MeshSlot*>(change.value)->storage==scene->upload.storage)adopted=true;
                if(change.kind==Kind::Texture && static_cast<TextureSlot*>(change.value)->storage==scene->upload.storage)adopted=true;
            }
            if(scene->upload.storage && scene->upload.storage!=upload.storage && !adopted)
                scene->SceneHeapFree(scene->upload.storage);
            scene->upload=upload; if (upload.active) Publish(upload.storage);
        }
        if(!info.resourceOnly) {
            scene->BeginFrame(info.frameNumber);scene->frameTimeUs=info.frameTimeUs;scene->elapsedTimeUs+=info.frameTimeUs;
            scene->drawCallCount=drawCount;
            for(uint16_t i=0;i<drawCount;++i) {
                const auto& pending=draws[i]; auto& draw=scene->drawList[i];
                const uint8_t* record=records+pending.recordOffset;
                draw.meshId=PglHandleIndex(PglRuntime::Load16(record));
                draw.materialId=PglHandleIndex(PglRuntime::Load16(record+2));
                draw.enabled=(record[4]&PGL_DRAW_ENABLED)!=0;
                std::memcpy(&draw.transform,record+offsetof(PglCmdDrawObject,position),sizeof(draw.transform));
                draw.hasVertexOverride=pending.overrideVertices!=nullptr;
                draw.overrideVertexCount=pending.overrideVertexCount;draw.overrideVertices=pending.overrideVertices;
            }
            scene->drawCmd2DCount=drawCount2D;for(uint16_t i=0;i<drawCount2D;++i)scene->drawCmds2D[i]=draws2D[i].value;
            scene->spritePosPool2D.used=spriteCount;std::memcpy(scene->spritePosPool2D.data,spritePositions,size_t(spriteCount)*4);
            for(uint8_t i=0;i<frameOwnedCount;++i){scene->frameOwnedData[scene->frameOwnedCount++]=frameOwned[i];Publish(frameOwned[i]);}
        }
        for(uint16_t i=0;i<changeCount;++i) {
            const auto& c=changes[i];
            switch(c.kind) {
                case Kind::Mesh: {
                    auto& old=scene->meshes[c.index];const auto& value=*static_cast<MeshSlot*>(c.value);
                    if(old.storage!=value.storage||old.externalToken!=value.externalToken)scene->FreeMesh(c.index);
                    old=value;Publish(value.storage);PublishExternal(value.externalToken);
                    scene->pendingMeshDestroy[c.index]=(c.flags&Change::Destroy)!=0;
                    if(c.flags&Change::HasGeneration){scene->meshGeneration[c.index]=c.generation;scene->meshEverUsed[c.index]=true;}
                    break;
                }
                case Kind::Material:
                    scene->materials[c.index]=*static_cast<MaterialSlot*>(c.value);
                    scene->pendingMaterialDestroy[c.index]=(c.flags&Change::Destroy)!=0;
                    if(c.flags&Change::HasGeneration){scene->materialGeneration[c.index]=c.generation;scene->materialEverUsed[c.index]=true;}
                    break;
                case Kind::Texture: {
                    auto& old=scene->textures[c.index];const auto& value=*static_cast<TextureSlot*>(c.value);
                    if(old.storage!=value.storage||old.externalToken!=value.externalToken)scene->FreeTexture(c.index);
                    old=value;Publish(value.storage);PublishExternal(value.externalToken);
                    scene->pendingTextureDestroy[c.index]=(c.flags&Change::Destroy)!=0;
                    if(c.flags&Change::HasGeneration){scene->textureGeneration[c.index]=c.generation;scene->textureEverUsed[c.index]=true;}
                    break;
                }
                case Kind::Layout: { auto& old=scene->pixelLayouts[c.index];const auto& value=*static_cast<PixelLayoutSlot*>(c.value);if(old.coords!=value.coords)scene->SceneHeapFree(old.coords);old=value;Publish(value.coords);break; }
                case Kind::Camera:scene->cameras[c.index]=*static_cast<CameraSlot*>(c.value);break;
                case Kind::Layer:{auto&old=scene->layers[c.index];const auto&value=*static_cast<LayerSlot*>(c.value);if(c.flags & Change::Destroy){scene->FreeLayerFramebuffer(c.index);old={};}else{if(old.pixels!=value.pixels)scene->FreeLayerFramebuffer(c.index);old=value;Publish(value.pixels);}break;}
                case Kind::Shader:scene->shaderPrograms[c.index]=*static_cast<ShaderProgram*>(c.value);++scene->shaderStateVersion;break;
            }
        }
        scene->activeLayerCount=0;for(uint8_t i=1;i<GpuConfig::MAX_LAYERS;++i)if(scene->layers[i].active)++scene->activeLayerCount;
        if(info.resourceOnly)scene->RetireResourceReads();
    }
};
static_assert(sizeof(Transaction) <= PhaseScratch::CapacityBytes, "parser transaction must fit phase scratch");
static_assert(sizeof(void*) != 4 || sizeof(Change) == 12, "compact target change metadata");
static_assert(sizeof(void*) != 4 || sizeof(PendingDraw) == 8, "compact target pending draw metadata");
static_assert(offsetof(PglCmdDrawObject,position)+sizeof(PglTransform)==sizeof(PglCmdDrawObject),
              "draw transform is the complete wire suffix");
}

PglRuntime::Result Parse(const uint8_t* bytes,size_t length,SceneState* scene,BatchInfo& info,bool resourceOnly) {
    auto fail=[](R result){if(errors!=0xffff)++errors;errorMask|=uint32_t(1)<<(uint16_t(result)%32);return result;};
    if(!bytes||!scene||length<sizeof(PglFrameHeader)+sizeof(PglFrameFooter)||length>PglRuntime::MaxBatchBytes)return fail(R::BadPacket);
    if(PglRuntime::Load16(bytes)!=PGL_SYNC_WORD||PglRuntime::Load32(bytes+6)!=length||PglRuntime::Load16(bytes+length-2)!=PglCRC16::Compute(bytes,length-2))return fail(R::BadPacket);
    const uint16_t count=PglRuntime::Load16(bytes+10);if(count<(resourceOnly?1:2)||count>GpuConfig::MAX_BATCH_COMMANDS)return fail(R::Capacity);
    PhaseScratch::Lease<Transaction> scratch;if(!scratch)return fail(R::Busy);
    auto& transaction=*scratch;
    transaction.Start(scene,resourceOnly,bytes);const uint32_t frame=PglRuntime::Load32(bytes+2);
    size_t offset=sizeof(PglFrameHeader);R result=R::Ok;
    for(uint16_t i=0;i<count;++i){
        if(length-2-offset<sizeof(PglCommandHeader)){result=R::BadPacket;break;}
        const uint8_t opcode=bytes[offset];const uint16_t payload=PglRuntime::Load16(bytes+offset+1);offset+=sizeof(PglCommandHeader);
        const bool badFrameGrammar = !resourceOnly && ((i==0&&opcode!=PGL_CMD_BEGIN_FRAME)||(i+1==count&&opcode!=PGL_CMD_END_FRAME)||(i>0&&opcode==PGL_CMD_BEGIN_FRAME)||(i+1<count&&opcode==PGL_CMD_END_FRAME));
        if(payload>length-2-offset||badFrameGrammar||(resourceOnly&&(opcode==PGL_CMD_BEGIN_FRAME||opcode==PGL_CMD_END_FRAME))){result=R::BadPacket;break;}
        result=transaction.Command(opcode,bytes+offset,payload);if(result!=R::Ok)break;offset+=payload;
    }
    if(result==R::Ok&&(offset!=length-2||transaction.info.frameNumber!=frame||
       (resourceOnly?frame!=0:frame==0)))result=R::BadPacket;
    if(result==R::Ok)result=transaction.CheckBudget();
    if(result==R::Ok){transaction.info.commands=count;transaction.Commit();info=transaction.info;}
    transaction.Finish();return result==R::Ok?R::Ok:fail(result);
}
uint16_t GetParserErrorCount(){return errors;}
uint32_t GetParserErrorMask(){return errorMask;}
void ClearErrors(){errors=0;errorMask=0;}
} // namespace CommandParser
