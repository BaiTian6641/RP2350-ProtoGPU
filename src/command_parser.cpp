/**
 * @file command_parser.cpp
 * @brief ProtoGL command buffer parser implementation for RP2350.
 *
 * Parses a ProtoGL wire-format frame and populates the GPU's SceneState
 * with meshes, materials, draw calls, camera state, etc.
 */

#include "command_parser.h"
#include "scene_state.h"
#include "gpu_config.h"

// Shared ProtoGL headers (from lib/ProtoGL/src/)
#include <PglTypes.h>
#include <PglOpcodes.h>
#include <PglParser.h>
#include <PglCRC16.h>

#include <cstdio>
#include <cstring>

// ─── Opcode Handlers (forward declarations) ─────────────────────────────────

static void HandleBeginFrame(const uint8_t*& ptr, SceneState* scene);
static void HandleEndFrame(const uint8_t*& ptr, SceneState* scene);
static void HandleCreateMesh(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);
static void HandleDestroyMesh(const uint8_t*& ptr, SceneState* scene);
static void HandleUpdateVertices(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);
static void HandleUpdateVerticesDelta(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);
static void HandleCreateMaterial(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);
static void HandleUpdateMaterial(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);
static void HandleDestroyMaterial(const uint8_t*& ptr, SceneState* scene);
static void HandleCreateTexture(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);
static void HandleDestroyTexture(const uint8_t*& ptr, SceneState* scene);
static void HandleSetPixelLayout(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);
static void HandleDrawObject(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);
static void HandleSetCamera(const uint8_t*& ptr, SceneState* scene);
static void HandleSetShader(const uint8_t*& ptr, SceneState* scene);

// ─── Main Parser ────────────────────────────────────────────────────────────

CommandParser::ParseResult CommandParser::Parse(
    const uint8_t* frameData, uint32_t frameLength, SceneState* scene)
{
    if (frameLength < sizeof(PglFrameHeader) + sizeof(PglFrameFooter)) {
        return ParseResult::TruncatedFrame;
    }

    // 1. Read frame header
    const uint8_t* ptr = frameData;
    PglFrameHeader hdr;
    PglReadStruct(ptr, hdr);

    // 2. Validate sync word
    if (hdr.syncWord != PGL_SYNC_WORD) {
        return ParseResult::InvalidSync;
    }

    // 3. Validate CRC-16 (over everything except the 2-byte footer)
    const uint32_t dataLen = frameLength - sizeof(PglFrameFooter);
    uint16_t computedCrc = PglCRC16::Compute(frameData, dataLen);
    uint16_t storedCrc;
    std::memcpy(&storedCrc, frameData + dataLen, sizeof(storedCrc));

    if (computedCrc != storedCrc) {
        return ParseResult::CrcError;
    }

    // 4. Clear the per-frame draw list (persistent resources like meshes stay)
    scene->BeginFrame(hdr.frameNumber);

    // 5. Iterate commands
    const uint8_t* frameEnd = frameData + dataLen;  // stop before CRC
    ParseResult result = ParseResult::Ok;

    for (uint16_t i = 0; i < hdr.commandCount && ptr < frameEnd; ++i) {
        // Read command header
        PglCommandHeader cmdHdr;
        PglReadStruct(ptr, cmdHdr);

        const uint8_t* cmdPayloadStart = ptr;

        switch (cmdHdr.opcode) {
            case PGL_CMD_BEGIN_FRAME:
                HandleBeginFrame(ptr, scene);
                break;
            case PGL_CMD_END_FRAME:
                HandleEndFrame(ptr, scene);
                break;
            case PGL_CMD_CREATE_MESH:
                HandleCreateMesh(ptr, cmdHdr.payloadLength, scene);
                break;
            case PGL_CMD_DESTROY_MESH:
                HandleDestroyMesh(ptr, scene);
                break;
            case PGL_CMD_UPDATE_VERTICES:
                HandleUpdateVertices(ptr, cmdHdr.payloadLength, scene);
                break;
            case PGL_CMD_UPDATE_VERTICES_DELTA:
                HandleUpdateVerticesDelta(ptr, cmdHdr.payloadLength, scene);
                break;
            case PGL_CMD_CREATE_MATERIAL:
                HandleCreateMaterial(ptr, cmdHdr.payloadLength, scene);
                break;
            case PGL_CMD_UPDATE_MATERIAL:
                HandleUpdateMaterial(ptr, cmdHdr.payloadLength, scene);
                break;
            case PGL_CMD_DESTROY_MATERIAL:
                HandleDestroyMaterial(ptr, scene);
                break;
            case PGL_CMD_CREATE_TEXTURE:
                HandleCreateTexture(ptr, cmdHdr.payloadLength, scene);
                break;
            case PGL_CMD_DESTROY_TEXTURE:
                HandleDestroyTexture(ptr, scene);
                break;
            case PGL_CMD_SET_PIXEL_LAYOUT:
                HandleSetPixelLayout(ptr, cmdHdr.payloadLength, scene);
                break;
            case PGL_CMD_DRAW_OBJECT:
                HandleDrawObject(ptr, cmdHdr.payloadLength, scene);
                break;
            case PGL_CMD_SET_CAMERA:
                HandleSetCamera(ptr, scene);
                break;
            case PGL_CMD_SET_SHADER:
                HandleSetShader(ptr, scene);
                break;
            default:
                // Unknown opcode — skip payload, log warning
                printf("[Parser] Unknown opcode 0x%02X, skipping %u bytes\n",
                       cmdHdr.opcode, cmdHdr.payloadLength);
                PglSkip(ptr, cmdHdr.payloadLength);
                result = ParseResult::UnknownOpcode;
                break;
        }

        // Safety: ensure ptr advanced by exactly payloadLength
        // (in case a handler read too few or too many bytes)
        ptr = cmdPayloadStart + cmdHdr.payloadLength;
    }

    return result;
}

// ─── Opcode Handlers ────────────────────────────────────────────────────────

static void HandleBeginFrame(const uint8_t*& ptr, SceneState* scene) {
    PglCmdBeginFrame cmd;
    PglReadStruct(ptr, cmd);
    scene->frameTimeUs = cmd.frameTimeUs;
}

static void HandleEndFrame(const uint8_t*& ptr, SceneState* scene) {
    PglCmdEndFrame cmd;
    PglReadStruct(ptr, cmd);
    // End-of-frame marker — scene state is now complete for this frame
    (void)scene;
}

static void HandleCreateMesh(const uint8_t*& ptr, uint16_t payloadLen,
                             SceneState* scene) {
    PglCmdCreateMeshHeader hdr;
    PglReadStruct(ptr, hdr);

    if (hdr.meshId >= GpuConfig::MAX_MESHES) {
        PglSkip(ptr, payloadLen - sizeof(hdr));
        return;
    }

    // If the slot was already active, free its old pool allocations
    MeshSlot& mesh = scene->meshes[hdr.meshId];
    if (mesh.active) {
        scene->FreeVertices(mesh.vertices);
        scene->FreeIndices(mesh.indices);
        scene->FreeUVVertices(mesh.uvVertices);
        scene->FreeUVIndices(mesh.uvIndices);
    }

    uint16_t vertsToCopy = (hdr.vertexCount <= GpuConfig::MAX_VERTICES)
                         ? hdr.vertexCount : GpuConfig::MAX_VERTICES;
    uint16_t trisToCopy  = (hdr.triangleCount <= GpuConfig::MAX_TRIANGLES)
                         ? hdr.triangleCount : GpuConfig::MAX_TRIANGLES;

    // Allocate from pools
    PglVec3*   verts = scene->AllocVertices(vertsToCopy);
    PglIndex3* idxs  = scene->AllocIndices(trisToCopy);
    if (!verts || !idxs) {
        printf("[Parser] Vertex/index pool full for mesh %u\n", hdr.meshId);
        PglSkip(ptr, payloadLen - sizeof(hdr));
        return;
    }

    mesh.active        = true;
    mesh.vertexCount   = vertsToCopy;
    mesh.triangleCount = trisToCopy;
    mesh.vertices      = verts;
    mesh.indices       = idxs;
    mesh.uvVertexCount = 0;
    mesh.uvVertices    = nullptr;
    mesh.uvIndices     = nullptr;

    // Read vertex positions into pool memory
    PglReadArray(ptr, mesh.vertices, vertsToCopy);
    if (hdr.vertexCount > vertsToCopy) {
        PglSkip(ptr, (hdr.vertexCount - vertsToCopy) * sizeof(PglVec3));
    }

    // Read triangle indices into pool memory
    PglReadArray(ptr, mesh.indices, trisToCopy);
    if (hdr.triangleCount > trisToCopy) {
        PglSkip(ptr, (hdr.triangleCount - trisToCopy) * sizeof(PglIndex3));
    }

    // UV data (optional)
    if (hdr.flags & PGL_MESH_HAS_UV) {
        uint16_t uvVertexCount = PglRead<uint16_t>(ptr);
        uint16_t uvToCopy = (uvVertexCount <= GpuConfig::MAX_VERTICES)
                          ? uvVertexCount : GpuConfig::MAX_VERTICES;

        PglVec2*   uvVerts = scene->AllocUVVertices(uvToCopy);
        PglIndex3* uvIdxs  = scene->AllocUVIndices(trisToCopy);
        if (!uvVerts || !uvIdxs) {
            printf("[Parser] UV pool full for mesh %u\n", hdr.meshId);
            // Skip remaining UV data
            PglSkip(ptr, uvVertexCount * sizeof(PglVec2)
                       + hdr.triangleCount * sizeof(PglIndex3));
        } else {
            mesh.uvVertexCount = uvToCopy;
            mesh.uvVertices    = uvVerts;
            mesh.uvIndices     = uvIdxs;

            PglReadArray(ptr, mesh.uvVertices, uvToCopy);
            if (uvVertexCount > uvToCopy) {
                PglSkip(ptr, (uvVertexCount - uvToCopy) * sizeof(PglVec2));
            }
            PglReadArray(ptr, mesh.uvIndices, trisToCopy);
            if (hdr.triangleCount > trisToCopy) {
                PglSkip(ptr, (hdr.triangleCount - trisToCopy) * sizeof(PglIndex3));
            }
        }
    }
}

static void HandleDestroyMesh(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDestroyMesh cmd;
    PglReadStruct(ptr, cmd);
    if (cmd.meshId < GpuConfig::MAX_MESHES) {
        MeshSlot& mesh = scene->meshes[cmd.meshId];
        // Free pool allocations (stack-style, best-effort)
        scene->FreeUVIndices(mesh.uvIndices);
        scene->FreeUVVertices(mesh.uvVertices);
        scene->FreeIndices(mesh.indices);
        scene->FreeVertices(mesh.vertices);
        mesh = MeshSlot{};  // zero the slot
    }
}

static void HandleUpdateVertices(const uint8_t*& ptr, uint16_t payloadLen,
                                 SceneState* scene) {
    PglCmdUpdateVerticesHeader hdr;
    PglReadStruct(ptr, hdr);

    if (hdr.meshId >= GpuConfig::MAX_MESHES || !scene->meshes[hdr.meshId].active) {
        PglSkip(ptr, hdr.vertexCount * sizeof(PglVec3));
        return;
    }

    MeshSlot& mesh = scene->meshes[hdr.meshId];
    uint16_t count = (hdr.vertexCount <= mesh.vertexCount)
                   ? hdr.vertexCount : mesh.vertexCount;
    PglReadArray(ptr, mesh.vertices, count);
    if (hdr.vertexCount > count) {
        PglSkip(ptr, (hdr.vertexCount - count) * sizeof(PglVec3));
    }
}

static void HandleUpdateVerticesDelta(const uint8_t*& ptr, uint16_t payloadLen,
                                      SceneState* scene) {
    PglCmdUpdateVerticesDeltaHeader hdr;
    PglReadStruct(ptr, hdr);

    if (hdr.meshId >= GpuConfig::MAX_MESHES || !scene->meshes[hdr.meshId].active) {
        PglSkip(ptr, hdr.deltaCount * sizeof(PglVertexDelta));
        return;
    }

    MeshSlot& mesh = scene->meshes[hdr.meshId];
    for (uint16_t i = 0; i < hdr.deltaCount; ++i) {
        PglVertexDelta delta;
        PglReadStruct(ptr, delta);
        if (delta.index < mesh.vertexCount) {
            mesh.vertices[delta.index].x += delta.x;
            mesh.vertices[delta.index].y += delta.y;
            mesh.vertices[delta.index].z += delta.z;
        }
    }
}

static void HandleCreateMaterial(const uint8_t*& ptr, uint16_t payloadLen,
                                 SceneState* scene) {
    PglCmdCreateMaterialHeader hdr;
    PglReadStruct(ptr, hdr);

    if (hdr.materialId >= GpuConfig::MAX_MATERIALS) {
        PglSkip(ptr, payloadLen - sizeof(hdr));
        return;
    }

    MaterialSlot& mat = scene->materials[hdr.materialId];
    mat.active = true;
    mat.type = static_cast<PglMaterialType>(hdr.materialType);
    mat.blendMode = static_cast<PglBlendMode>(hdr.blendMode);

    // Copy type-specific parameters into the raw param buffer
    uint16_t paramSize = payloadLen - sizeof(hdr);
    if (paramSize > sizeof(mat.params)) {
        paramSize = sizeof(mat.params);
    }
    if (paramSize > 0) {
        std::memcpy(mat.params, ptr, paramSize);
    }
    PglSkip(ptr, payloadLen - sizeof(hdr));
}

static void HandleUpdateMaterial(const uint8_t*& ptr, uint16_t payloadLen,
                                 SceneState* scene) {
    PglCmdUpdateMaterialHeader hdr;
    PglReadStruct(ptr, hdr);

    if (hdr.materialId >= GpuConfig::MAX_MATERIALS ||
        !scene->materials[hdr.materialId].active) {
        PglSkip(ptr, payloadLen - sizeof(hdr));
        return;
    }

    MaterialSlot& mat = scene->materials[hdr.materialId];
    uint16_t paramSize = payloadLen - sizeof(hdr);
    if (paramSize > sizeof(mat.params)) {
        paramSize = sizeof(mat.params);
    }
    if (paramSize > 0) {
        std::memcpy(mat.params, ptr, paramSize);
    }
    PglSkip(ptr, payloadLen - sizeof(hdr));
}

static void HandleDestroyMaterial(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDestroyMaterial cmd;
    PglReadStruct(ptr, cmd);
    if (cmd.materialId < GpuConfig::MAX_MATERIALS) {
        scene->materials[cmd.materialId].active = false;
    }
}

static void HandleCreateTexture(const uint8_t*& ptr, uint16_t payloadLen,
                                SceneState* scene) {
    PglCmdCreateTextureHeader hdr;
    PglReadStruct(ptr, hdr);

    if (hdr.textureId >= GpuConfig::MAX_TEXTURES) {
        PglSkip(ptr, payloadLen - sizeof(hdr));
        return;
    }

    TextureSlot& tex = scene->textures[hdr.textureId];

    // Free previous allocation if re-creating
    if (tex.active && tex.pixels) {
        scene->FreeTexturePixels(tex.pixels);
    }

    uint16_t bpp = (hdr.format == PGL_TEX_RGB565) ? 2 : 3;
    uint32_t pixelDataSize = static_cast<uint32_t>(hdr.width) * hdr.height * bpp;
    uint8_t* pixBuf = scene->AllocTexturePixels(pixelDataSize);
    if (!pixBuf) {
        printf("[Parser] Texture pool full for texture %u (%u bytes)\n",
               hdr.textureId, pixelDataSize);
        PglSkip(ptr, payloadLen - sizeof(hdr));
        return;
    }

    tex.active        = true;
    tex.width         = hdr.width;
    tex.height        = hdr.height;
    tex.format        = static_cast<PglTextureFormat>(hdr.format);
    tex.pixelDataSize = pixelDataSize;
    tex.pixels        = pixBuf;

    // Copy pixel data into pool memory
    std::memcpy(tex.pixels, ptr, pixelDataSize);
    PglSkip(ptr, payloadLen - sizeof(hdr));
}

static void HandleDestroyTexture(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDestroyTexture cmd;
    PglReadStruct(ptr, cmd);
    if (cmd.textureId < GpuConfig::MAX_TEXTURES) {
        TextureSlot& tex = scene->textures[cmd.textureId];
        scene->FreeTexturePixels(tex.pixels);
        tex = TextureSlot{};
    }
}

static void HandleSetPixelLayout(const uint8_t*& ptr, uint16_t payloadLen,
                                 SceneState* scene) {
    PglCmdSetPixelLayoutHeader hdr;
    PglReadStruct(ptr, hdr);

    if (hdr.layoutId >= PGL_MAX_LAYOUTS) {
        PglSkip(ptr, payloadLen - sizeof(hdr));
        return;
    }

    PixelLayoutSlot& layout = scene->pixelLayouts[hdr.layoutId];

    // Free previous coord allocation if re-creating
    if (layout.active && layout.coords) {
        scene->FreeLayoutCoords(layout.coords);
    }

    layout.active     = true;
    layout.pixelCount = hdr.pixelCount;
    layout.flags      = hdr.flags;
    layout.coords     = nullptr;

    if (hdr.flags & PGL_LAYOUT_RECTANGULAR) {
        // Rectangular layout — read fixed-size RectLayoutData
        PglReadStruct(ptr, layout.rectData);
    } else {
        // Irregular layout — allocate coords from pool
        uint16_t count = (hdr.pixelCount <= GpuConfig::FRAMEBUF_PIXELS)
                       ? hdr.pixelCount : static_cast<uint16_t>(GpuConfig::FRAMEBUF_PIXELS);
        PglVec2* coordBuf = scene->AllocLayoutCoords(count);
        if (!coordBuf) {
            printf("[Parser] Layout coord pool full for layout %u\n", hdr.layoutId);
            PglSkip(ptr, hdr.pixelCount * sizeof(PglVec2));
            return;
        }
        layout.coords = coordBuf;
        PglReadArray(ptr, layout.coords, count);
        if (hdr.pixelCount > count) {
            PglSkip(ptr, (hdr.pixelCount - count) * sizeof(PglVec2));
        }
    }
}

static void HandleDrawObject(const uint8_t*& ptr, uint16_t payloadLen,
                             SceneState* scene) {
    PglCmdDrawObject cmd;
    PglReadStruct(ptr, cmd);

    if (scene->drawCallCount >= GpuConfig::MAX_DRAW_CALLS) return;

    DrawCall& dc = scene->drawList[scene->drawCallCount];
    dc.meshId     = cmd.meshId;
    dc.materialId = cmd.materialId;
    dc.enabled    = (cmd.flags & PGL_DRAW_ENABLED) != 0;

    // Copy full transform
    dc.transform.position             = cmd.position;
    dc.transform.rotation             = cmd.rotation;
    dc.transform.scale                = cmd.scale;
    dc.transform.baseRotation         = cmd.baseRotation;
    dc.transform.scaleRotationOffset  = cmd.scaleRotationOffset;
    dc.transform.scaleOffset          = cmd.scaleOffset;
    dc.transform.rotationOffset       = cmd.rotationOffset;

    // Vertex override (morph) — uses per-frame pool (reset each BeginFrame)
    dc.hasVertexOverride  = (cmd.flags & PGL_DRAW_VERTEX_OVERRIDE) != 0;
    dc.overrideVertexCount = 0;
    dc.overrideVertices    = nullptr;
    if (dc.hasVertexOverride) {
        dc.overrideVertexCount = PglRead<uint16_t>(ptr);
        uint16_t count = (dc.overrideVertexCount <= GpuConfig::MAX_VERTICES)
                       ? dc.overrideVertexCount : GpuConfig::MAX_VERTICES;
        PglVec3* overrideBuf = scene->AllocFrameVertices(count);
        if (!overrideBuf) {
            printf("[Parser] Frame vertex pool full for draw call\n");
            PglSkip(ptr, dc.overrideVertexCount * sizeof(PglVec3));
            dc.hasVertexOverride = false;
        } else {
            dc.overrideVertices = overrideBuf;
            PglReadArray(ptr, dc.overrideVertices, count);
            if (dc.overrideVertexCount > count) {
                PglSkip(ptr, (dc.overrideVertexCount - count) * sizeof(PglVec3));
                dc.overrideVertexCount = count;
            }
        }
    }

    scene->drawCallCount++;
}

static void HandleSetCamera(const uint8_t*& ptr, SceneState* scene) {
    PglCmdSetCamera cmd;
    PglReadStruct(ptr, cmd);

    if (cmd.cameraId >= PGL_MAX_CAMERAS) return;

    CameraSlot& cam = scene->cameras[cmd.cameraId];
    cam.active      = true;
    cam.layoutId    = cmd.pixelLayoutId;
    cam.position    = cmd.position;
    cam.rotation    = cmd.rotation;
    cam.scale       = cmd.scale;
    cam.lookOffset  = cmd.lookOffset;
    cam.baseRotation = cmd.baseRotation;
    cam.is2D        = cmd.is2D != 0;
}

static void HandleSetShader(const uint8_t*& ptr, SceneState* scene) {
    PglCmdSetShader cmd;
    PglReadStruct(ptr, cmd);

    if (cmd.cameraId >= PGL_MAX_CAMERAS) return;
    CameraSlot& cam = scene->cameras[cmd.cameraId];
    if (!cam.active) return;
    if (cmd.shaderSlot >= PGL_MAX_SHADERS_PER_CAMERA) return;

    ShaderSlot& slot = cam.shaders[cmd.shaderSlot];
    slot.active      = (cmd.shaderClass != PGL_SHADER_NONE);
    slot.shaderClass = cmd.shaderClass;
    slot.intensity   = cmd.intensity;
    std::memcpy(slot.params, cmd.params, sizeof(slot.params));
}
