/**
 * @file command_parser.cpp
 * @brief ProtoGL command buffer parser implementation for RP2350.
 *
 * Parses a ProtoGL wire-format frame and populates the GPU's SceneState
 * with meshes, materials, draw calls, camera state, 2D layer commands,
 * memory defrag, persistence, and direct framebuffer writes.
 *
 * ProtoGL v0.7.2 — M12: 2D compositing, defrag, persistence, framebuffer write.
 */

#include "command_parser.h"
#include "scene_state.h"
#include "gpu_config.h"
#include "memory/mem_opi_psram.h"
#include "memory/mem_qspi_psram.h"
#include "memory/mem_tier.h"
#include "memory/mem_pool.h"
#include "memory/flash_persist.h"
#include "display/display_manager.h"

// Shared ProtoGL headers (from lib/ProtoGL/src/)
#include <PglTypes.h>
#include <PglOpcodes.h>
#include <PglParser.h>
#include <PglCRC16.h>
#include <PglShaderBytecode.h>

#include <cstdio>
#include <cstring>
#include <algorithm>

// ─── Memory Subsystem State (set via InitMemory) ────────────────────────────

static OpiPsramDriver*  s_opi   = nullptr;
static QspiPsramDriver* s_qspi  = nullptr;
static MemTierManager*  s_tier  = nullptr;

static const uint16_t*  s_frontBuffer = nullptr;
static const uint16_t*  s_backBuffer  = nullptr;
static uint32_t         s_fbPixels    = 0;

/// Simple handle-based allocation table for CMD_MEM_ALLOC / CMD_MEM_FREE.
struct MemAllocEntry {
    bool         active;
    uint8_t      tier;
    uint32_t     address;
    uint32_t     size;
    uint16_t     tag;
};
static MemAllocEntry s_allocTable[PGL_MAX_MEM_ALLOCATIONS];
static uint16_t      s_nextHandle = 0;

// ─── M11: Display & Pool Subsystem State ────────────────────────────────────

static DisplayManager*  s_displayMgr = nullptr;
static MemPoolManager*  s_poolMgr    = nullptr;

// ─── M12: Flash Persistence State ───────────────────────────────────────────

static FlashPersistManager s_flashPersist;
static bool                s_flashPersistInitialized = false;

void CommandParser::InitMemory(OpiPsramDriver* opi, QspiPsramDriver* qspi,
                                MemTierManager* tier,
                                const uint16_t* frontBuf,
                                const uint16_t* backBuf,
                                uint32_t fbPixels) {
    s_opi   = opi;
    s_qspi  = qspi;
    s_tier  = tier;
    s_frontBuffer = frontBuf;
    s_backBuffer  = backBuf;
    s_fbPixels    = fbPixels;
    s_nextHandle  = 0;
    std::memset(s_allocTable, 0, sizeof(s_allocTable));

    // Initialize flash persistence manager (M12)
    if (!s_flashPersistInitialized) {
        s_flashPersist.Initialize(tier);
        s_flashPersistInitialized = true;
    }
}

void CommandParser::UpdateFramebufferPtrs(const uint16_t* frontBuf,
                                          const uint16_t* backBuf) {
    s_frontBuffer = frontBuf;
    s_backBuffer  = backBuf;
}

void CommandParser::InitDisplayAndPools(DisplayManager* displayMgr,
                                         MemPoolManager* poolMgr) {
    s_displayMgr = displayMgr;
    s_poolMgr    = poolMgr;
}

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

// Memory access handlers (0x30 – 0x3F)
static void HandleMemWrite(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);
static void HandleMemReadRequest(const uint8_t*& ptr, SceneState* scene);
static void HandleMemSetResourceTier(const uint8_t*& ptr, SceneState* scene);
static void HandleMemAlloc(const uint8_t*& ptr, SceneState* scene);
static void HandleMemFree(const uint8_t*& ptr, SceneState* scene);
static void HandleFramebufferCapture(const uint8_t*& ptr, SceneState* scene);
static void HandleMemCopy(const uint8_t*& ptr, SceneState* scene);

// Programmable shader handlers (0x84 – 0x87)
static void HandleCreateShaderProgram(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);
static void HandleDestroyShaderProgram(const uint8_t*& ptr, SceneState* scene);
static void HandleBindShaderProgram(const uint8_t*& ptr, SceneState* scene);
static void HandleSetShaderUniform(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);

// ── M12: 2D Layer & Draw Command handlers (0xA0 – 0xAC) ────────────────────
static void HandleLayerCreate(const uint8_t*& ptr, SceneState* scene);
static void HandleLayerDestroy(const uint8_t*& ptr, SceneState* scene);
static void HandleLayerSetProps(const uint8_t*& ptr, SceneState* scene);
static void HandleDrawRect2D(const uint8_t*& ptr, SceneState* scene);
static void HandleDrawLine2D(const uint8_t*& ptr, SceneState* scene);
static void HandleDrawCircle2D(const uint8_t*& ptr, SceneState* scene);
static void HandleDrawSprite(const uint8_t*& ptr, SceneState* scene);
static void HandleLayerClear(const uint8_t*& ptr, SceneState* scene);
static void HandleDrawRoundedRect(const uint8_t*& ptr, SceneState* scene);
static void HandleDrawArc(const uint8_t*& ptr, SceneState* scene);
static void HandleDrawTriangle2D(const uint8_t*& ptr, SceneState* scene);

// ── M11: Memory pool handlers (0x38 – 0x3B) ────────────────────────────────
static void HandleMemPoolCreate(const uint8_t*& ptr, SceneState* scene);
static void HandleMemPoolAlloc(const uint8_t*& ptr, SceneState* scene);
static void HandleMemPoolFree(const uint8_t*& ptr, SceneState* scene);
static void HandleMemPoolDestroy(const uint8_t*& ptr, SceneState* scene);

// ── M11: Display driver handlers (0x90 – 0x91) ─────────────────────────────
static void HandleDisplayConfigure(const uint8_t*& ptr, SceneState* scene);
static void HandleDisplaySetRegion(const uint8_t*& ptr, SceneState* scene);

// ── M12: Memory defrag, framebuffer write, persistence (0x3C, 0x45 – 0x48) ─
static void HandleMemDefrag(const uint8_t*& ptr, SceneState* scene);
static void HandleWriteFramebuffer(const uint8_t*& ptr, uint16_t payloadLen, SceneState* scene);
static void HandlePersistResource(const uint8_t*& ptr, SceneState* scene);
static void HandleRestoreResource(const uint8_t*& ptr, SceneState* scene);
static void HandleQueryPersistence(const uint8_t*& ptr, SceneState* scene);

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

            // Memory access commands (0x30 – 0x3F)
            case PGL_CMD_MEM_WRITE:
                HandleMemWrite(ptr, cmdHdr.payloadLength, scene);
                break;
            case PGL_CMD_MEM_READ_REQUEST:
                HandleMemReadRequest(ptr, scene);
                break;
            case PGL_CMD_MEM_SET_RESOURCE_TIER:
                HandleMemSetResourceTier(ptr, scene);
                break;
            case PGL_CMD_MEM_ALLOC:
                HandleMemAlloc(ptr, scene);
                break;
            case PGL_CMD_MEM_FREE:
                HandleMemFree(ptr, scene);
                break;
            case PGL_CMD_FRAMEBUFFER_CAPTURE:
                HandleFramebufferCapture(ptr, scene);
                break;
            case PGL_CMD_MEM_COPY:
                HandleMemCopy(ptr, scene);
                break;

            // Programmable shader commands (0x84 – 0x87)
            case PGL_CMD_CREATE_SHADER_PROGRAM:
                HandleCreateShaderProgram(ptr, cmdHdr.payloadLength, scene);
                break;
            case PGL_CMD_DESTROY_SHADER_PROGRAM:
                HandleDestroyShaderProgram(ptr, scene);
                break;
            case PGL_CMD_BIND_SHADER_PROGRAM:
                HandleBindShaderProgram(ptr, scene);
                break;
            case PGL_CMD_SET_SHADER_UNIFORM:
                HandleSetShaderUniform(ptr, cmdHdr.payloadLength, scene);
                break;

            // ── M12: Memory defrag (0x3C) ───────────────────────────────
            case PGL_CMD_MEM_DEFRAG:
                HandleMemDefrag(ptr, scene);
                break;

            // ── M11: Memory pool commands (0x38 – 0x3B) ─────────────────
            case PGL_CMD_MEM_POOL_CREATE:
                HandleMemPoolCreate(ptr, scene);
                break;
            case PGL_CMD_MEM_POOL_ALLOC:
                HandleMemPoolAlloc(ptr, scene);
                break;
            case PGL_CMD_MEM_POOL_FREE:
                HandleMemPoolFree(ptr, scene);
                break;
            case PGL_CMD_MEM_POOL_DESTROY:
                HandleMemPoolDestroy(ptr, scene);
                break;

            // ── M12: Direct framebuffer write (0x45) ────────────────────
            case PGL_CMD_WRITE_FRAMEBUFFER:
                HandleWriteFramebuffer(ptr, cmdHdr.payloadLength, scene);
                break;

            // ── M12: Resource persistence (0x46 – 0x48) ────────────────
            case PGL_CMD_PERSIST_RESOURCE:
                HandlePersistResource(ptr, scene);
                break;
            case PGL_CMD_RESTORE_RESOURCE:
                HandleRestoreResource(ptr, scene);
                break;
            case PGL_CMD_QUERY_PERSISTENCE:
                HandleQueryPersistence(ptr, scene);
                break;

            // ── M12: 2D Layer lifecycle (0xA0 – 0xA2) ──────────────────
            case PGL_CMD_LAYER_CREATE:
                HandleLayerCreate(ptr, scene);
                break;
            case PGL_CMD_LAYER_DESTROY:
                HandleLayerDestroy(ptr, scene);
                break;
            case PGL_CMD_LAYER_SET_PROPS:
                HandleLayerSetProps(ptr, scene);
                break;

            // ── M12: 2D Drawing primitives (0xA3 – 0xA6) ───────────────
            case PGL_CMD_DRAW_RECT_2D:
                HandleDrawRect2D(ptr, scene);
                break;
            case PGL_CMD_DRAW_LINE_2D:
                HandleDrawLine2D(ptr, scene);
                break;
            case PGL_CMD_DRAW_CIRCLE_2D:
                HandleDrawCircle2D(ptr, scene);
                break;
            case PGL_CMD_DRAW_SPRITE:
                HandleDrawSprite(ptr, scene);
                break;

            // ── M12: 2D Utility (0xA9) ─────────────────────────────────
            case PGL_CMD_LAYER_CLEAR:
                HandleLayerClear(ptr, scene);
                break;

            // ── M12: 2D Extended primitives (0xAA – 0xAC) ──────────────
            case PGL_CMD_DRAW_ROUNDED_RECT:
                HandleDrawRoundedRect(ptr, scene);
                break;
            case PGL_CMD_DRAW_ARC:
                HandleDrawArc(ptr, scene);
                break;
            case PGL_CMD_DRAW_TRIANGLE_2D:
                HandleDrawTriangle2D(ptr, scene);
                break;

            // ── M11: Display driver commands (0x90 – 0x91) ──────────────
            case PGL_CMD_DISPLAY_CONFIGURE:
                HandleDisplayConfigure(ptr, scene);
                break;
            case PGL_CMD_DISPLAY_SET_REGION:
                HandleDisplaySetRegion(ptr, scene);
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

    // Compute and cache object-space bounding box for frustum culling
    mesh.RecomputeAABB();

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

    // Recompute cached AABB after vertex replacement
    mesh.RecomputeAABB();
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

    // Recompute cached AABB after delta updates
    mesh.RecomputeAABB();
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

// ─── Programmable Shader Handlers (0x84 – 0x87) ────────────────────────────

static void HandleCreateShaderProgram(const uint8_t*& ptr, uint16_t payloadLen,
                                       SceneState* scene) {
    PglCmdCreateShaderProgramHeader cmd;
    PglReadStruct(ptr, cmd);

    if (cmd.programId >= PGL_MAX_SHADER_PROGRAMS) {
        // Skip the bytecode blob
        ptr += cmd.bytecodeSize;
        return;
    }

    // Validate bytecode blob size
    if (cmd.bytecodeSize < sizeof(PglShaderProgramHeader)) {
        ptr += cmd.bytecodeSize;
        return;
    }

    // Read PSB header from the bytecode blob
    const uint8_t* blobStart = ptr;
    PglShaderProgramHeader psbHdr;
    std::memcpy(&psbHdr, ptr, sizeof(psbHdr));
    ptr += sizeof(psbHdr);

    // Validate magic and version
    if (psbHdr.magic != PSB_MAGIC || psbHdr.version != PSB_VERSION) {
        ptr = blobStart + cmd.bytecodeSize;
        return;
    }

    // Validate counts
    if (psbHdr.uniformCount > PSB_MAX_UNIFORMS ||
        psbHdr.constCount > PSB_MAX_CONSTANTS ||
        psbHdr.instrCount > PSB_MAX_INSTRUCTIONS) {
        ptr = blobStart + cmd.bytecodeSize;
        return;
    }

    ShaderProgram& prog = scene->shaderPrograms[cmd.programId];
    std::memset(&prog, 0, sizeof(prog));
    prog.active       = true;
    prog.programId    = cmd.programId;
    prog.uniformCount = psbHdr.uniformCount;
    prog.constCount   = psbHdr.constCount;
    prog.instrCount   = psbHdr.instrCount;
    prog.flags        = psbHdr.flags;

    // Read uniform descriptor table
    for (uint8_t i = 0; i < psbHdr.uniformCount; ++i) {
        PglUniformDescriptor desc;
        std::memcpy(&desc, ptr, sizeof(desc));
        ptr += sizeof(desc);
        prog.uniformNameHashes[desc.slot] = desc.nameHash;
        prog.uniformTypes[desc.slot]      = desc.type;
    }

    // Read constants pool
    if (psbHdr.constCount > 0) {
        std::memcpy(prog.constants, ptr, psbHdr.constCount * sizeof(float));
        ptr += psbHdr.constCount * sizeof(float);
    }

    // Read instructions
    if (psbHdr.instrCount > 0) {
        std::memcpy(prog.instructions, ptr, psbHdr.instrCount * sizeof(uint32_t));
        ptr += psbHdr.instrCount * sizeof(uint32_t);
    }

    printf("[Parser] Created shader program %u: %u uniforms, %u consts, %u instrs\n",
           cmd.programId, psbHdr.uniformCount, psbHdr.constCount, psbHdr.instrCount);
}

static void HandleDestroyShaderProgram(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDestroyShaderProgram cmd;
    PglReadStruct(ptr, cmd);

    if (cmd.programId >= PGL_MAX_SHADER_PROGRAMS) return;

    ShaderProgram& prog = scene->shaderPrograms[cmd.programId];
    std::memset(&prog, 0, sizeof(prog));

    printf("[Parser] Destroyed shader program %u\n", cmd.programId);
}

static void HandleBindShaderProgram(const uint8_t*& ptr, SceneState* scene) {
    PglCmdBindShaderProgram cmd;
    PglReadStruct(ptr, cmd);

    if (cmd.cameraId >= PGL_MAX_CAMERAS) return;
    CameraSlot& cam = scene->cameras[cmd.cameraId];
    if (!cam.active) return;
    if (cmd.shaderSlot >= PGL_MAX_SHADERS_PER_CAMERA) return;

    ShaderSlot& slot = cam.shaders[cmd.shaderSlot];
    if (cmd.programId == 0xFFFF) {
        // Unbind — clear the slot
        slot.active      = false;
        slot.shaderClass = PGL_SHADER_NONE;
        slot.intensity   = 0.0f;
        slot.programId   = 0;
    } else {
        slot.active      = true;
        slot.shaderClass = PGL_SHADER_PROGRAM;
        slot.intensity   = cmd.intensity;
        slot.programId   = cmd.programId;
    }
}

static void HandleSetShaderUniform(const uint8_t*& ptr, uint16_t payloadLen,
                                    SceneState* scene) {
    PglCmdSetShaderUniformHeader cmd;
    PglReadStruct(ptr, cmd);

    if (cmd.programId >= PGL_MAX_SHADER_PROGRAMS) {
        // Skip value data
        if (cmd.componentCount > 0 && cmd.componentCount <= 4) {
            ptr += cmd.componentCount * sizeof(float);
        }
        return;
    }

    ShaderProgram& prog = scene->shaderPrograms[cmd.programId];
    if (!prog.active) {
        if (cmd.componentCount > 0 && cmd.componentCount <= 4) {
            ptr += cmd.componentCount * sizeof(float);
        }
        return;
    }

    if (cmd.uniformSlot >= PSB_MAX_UNIFORMS || cmd.componentCount == 0 || cmd.componentCount > 4) {
        if (cmd.componentCount > 0 && cmd.componentCount <= 4) {
            ptr += cmd.componentCount * sizeof(float);
        }
        return;
    }

    // Read the uniform value(s) directly into the program's uniform table.
    // Multi-component uniforms occupy consecutive slots.
    for (uint8_t c = 0; c < cmd.componentCount; ++c) {
        uint8_t slot = cmd.uniformSlot + c;
        if (slot < PSB_MAX_UNIFORMS) {
            float val;
            std::memcpy(&val, ptr, sizeof(float));
            prog.uniforms[slot] = val;
        }
        ptr += sizeof(float);
    }
}

// ─── Memory Access Handlers (0x30 – 0x3F) ──────────────────────────────────
// Route memory commands to the appropriate tier driver (OPI PSRAM, QSPI, or
// SRAM).  These were formerly M8 stubs — now fully implemented.

/// Helper: read bytes from a given tier into a buffer.
static bool TierReadInto(uint8_t tier, uint32_t address, void* dst,
                         uint32_t size) {
    switch (tier) {
        case PGL_TIER_SRAM:
            // Direct SRAM read — address is treated as a byte offset within
            // the scene state's staging buffer (for I2C readback flows).
            // The caller handles the actual pointer arithmetic.
            return false;  // Handled at call site

        case PGL_TIER_OPI_PSRAM:
            if (s_opi && s_opi->IsInitialized()) {
                return s_opi->ReadSync(address, dst, size);
            }
            return false;

        case PGL_TIER_QSPI_PSRAM:
            if (s_qspi && s_qspi->IsInitialized()) {
                s_qspi->Read(address, dst, size);
                return true;
            }
            return false;

        default:
            return false;
    }
}

/// Helper: write bytes from a buffer to a given tier.
static bool TierWriteTo(uint8_t tier, uint32_t address, const void* src,
                        uint32_t size) {
    switch (tier) {
        case PGL_TIER_SRAM:
            return false;  // Caller must handle SRAM writes directly

        case PGL_TIER_OPI_PSRAM:
            if (s_opi && s_opi->IsInitialized()) {
                return s_opi->WriteSync(address, src, size);
            }
            return false;

        case PGL_TIER_QSPI_PSRAM:
            if (s_qspi && s_qspi->IsInitialized()) {
                s_qspi->Write(address, src, size);
                return true;
            }
            return false;

        default:
            return false;
    }
}

static void HandleMemWrite(const uint8_t*& ptr, uint16_t payloadLen,
                           SceneState* scene) {
    PglCmdMemWriteHeader hdr;
    PglReadStruct(ptr, hdr);

    const uint8_t* dataPtr = ptr;
    PglSkip(ptr, hdr.size);  // advance ptr past the data payload

    switch (hdr.tier) {
        case PGL_TIER_SRAM: {
            // SRAM write: treat address as offset into the staging buffer.
            // This allows the host to pre-fill staging data for readback or
            // write directly to a known SRAM region.
            uint32_t maxBytes = SceneState::MEM_STAGING_SIZE;
            if (hdr.address < maxBytes) {
                uint32_t copyLen = std::min(hdr.size, maxBytes - hdr.address);
                std::memcpy(scene->memStagingBuffer + hdr.address, dataPtr, copyLen);
            }
            break;
        }
        case PGL_TIER_OPI_PSRAM:
            if (s_opi && s_opi->IsInitialized()) {
                if (!s_opi->WriteSync(hdr.address, dataPtr, hdr.size)) {
                    printf("[Parser] MemWrite: OPI WriteSync failed\n");
                }
            } else {
                printf("[Parser] MemWrite: OPI tier not available\n");
            }
            break;

        case PGL_TIER_QSPI_PSRAM:
            if (s_qspi && s_qspi->IsInitialized()) {
                s_qspi->Write(hdr.address, dataPtr, hdr.size);
            } else {
                printf("[Parser] MemWrite: QSPI tier not available\n");
            }
            break;

        default:
            printf("[Parser] MemWrite: invalid tier %u\n", hdr.tier);
            break;
    }
}

static void HandleMemReadRequest(const uint8_t*& ptr, SceneState* scene) {
    PglCmdMemReadRequest cmd;
    PglReadStruct(ptr, cmd);

    // Clamp to staging buffer size
    uint16_t stageLen = cmd.size;
    if (stageLen > SceneState::MEM_STAGING_SIZE) {
        stageLen = static_cast<uint16_t>(SceneState::MEM_STAGING_SIZE);
    }

    bool ok = false;
    switch (cmd.tier) {
        case PGL_TIER_SRAM:
            // SRAM read-request: the "address" is an offset within the
            // staging buffer itself (for round-trip verification) or a
            // known SRAM region. For safety, copy from staging.
            if (cmd.address + stageLen <= SceneState::MEM_STAGING_SIZE) {
                // Data is already in the staging buffer (identity read).
                // Just adjust the staging metadata.
                ok = true;
            }
            break;

        case PGL_TIER_OPI_PSRAM:
            if (s_opi && s_opi->IsInitialized()) {
                ok = s_opi->ReadSync(cmd.address, scene->memStagingBuffer, stageLen);
            }
            break;

        case PGL_TIER_QSPI_PSRAM:
            if (s_qspi && s_qspi->IsInitialized()) {
                s_qspi->Read(cmd.address, scene->memStagingBuffer, stageLen);
                ok = true;
            }
            break;

        default:
            break;
    }

    if (ok) {
        scene->memStagingLength  = stageLen;
        scene->memStagingReadPos = 0;
    } else {
        printf("[Parser] MemReadRequest: tier %u not available\n", cmd.tier);
        scene->memStagingLength  = 0;
        scene->memStagingReadPos = 0;
    }
}

static void HandleMemSetResourceTier(const uint8_t*& ptr, SceneState* scene) {
    PglCmdSetResourceTier cmd;
    PglReadStruct(ptr, cmd);

    if (!s_tier) {
        printf("[Parser] SetResourceTier: tier manager not initialized\n");
        return;
    }

    // Map PglMemResourceClass → internal ResClass
    ResClass rc;
    switch (cmd.resourceClass) {
        case PGL_RES_CLASS_MESH:     rc = ResClass::VERTEX_DATA;    break;
        case PGL_RES_CLASS_MATERIAL: rc = ResClass::MATERIAL_PARAM; break;
        case PGL_RES_CLASS_TEXTURE:  rc = ResClass::TEXTURE;        break;
        case PGL_RES_CLASS_LAYOUT:   rc = ResClass::LAYOUT_COORDS;  break;
        default:                     rc = ResClass::COLD_MESH;      break;
    }

    // Ensure the resource is registered with the tier manager
    MemRecord* rec = s_tier->FindRecord(cmd.resourceId);
    if (!rec) {
        // Auto-register with default data size (0 — will be updated on first write)
        s_tier->Register(cmd.resourceId, rc, 0);
        rec = s_tier->FindRecord(cmd.resourceId);
    }

    if (rec) {
        // Apply pinned flag
        rec->pinned = (cmd.flags & PGL_TIER_FLAG_PINNED) != 0;

        // If a preferred tier is specified (not AUTO), initiate migration
        if (cmd.preferredTier != PGL_TIER_AUTO) {
            MemTier targetTier;
            switch (cmd.preferredTier) {
                case PGL_TIER_SRAM:       targetTier = MemTier::SRAM;      break;
                case PGL_TIER_OPI_PSRAM:  targetTier = MemTier::OPI_PSRAM; break;
                case PGL_TIER_QSPI_PSRAM: targetTier = MemTier::QSPI_XIP;  break;
                default:                  targetTier = MemTier::SRAM;      break;
            }

            if (static_cast<uint8_t>(targetTier) <
                static_cast<uint8_t>(rec->currentTier)) {
                s_tier->Promote(*rec, targetTier);
            } else if (static_cast<uint8_t>(targetTier) >
                       static_cast<uint8_t>(rec->currentTier)) {
                s_tier->Demote(*rec, targetTier);
            }
        }
    }

    (void)scene;
}

static void HandleMemAlloc(const uint8_t*& ptr, SceneState* scene) {
    PglCmdMemAlloc cmd;
    PglReadStruct(ptr, cmd);

    // Validate tier
    if (cmd.tier == PGL_TIER_AUTO || cmd.tier > PGL_TIER_QSPI_PSRAM) {
        scene->lastAllocResult.handle  = PGL_INVALID_MEM_HANDLE;
        scene->lastAllocResult.address = 0;
        scene->lastAllocResult.status  = PGL_ALLOC_INVALID_TIER;
        return;
    }

    // Find a free handle slot
    PglMemHandle handle = PGL_INVALID_MEM_HANDLE;
    for (uint16_t i = 0; i < PGL_MAX_MEM_ALLOCATIONS; ++i) {
        uint16_t idx = (s_nextHandle + i) % PGL_MAX_MEM_ALLOCATIONS;
        if (!s_allocTable[idx].active) {
            handle = idx;
            s_nextHandle = (idx + 1) % PGL_MAX_MEM_ALLOCATIONS;
            break;
        }
    }

    if (handle == PGL_INVALID_MEM_HANDLE) {
        scene->lastAllocResult.handle  = PGL_INVALID_MEM_HANDLE;
        scene->lastAllocResult.address = 0;
        scene->lastAllocResult.status  = PGL_ALLOC_HANDLE_EXHAUSTED;
        return;
    }

    // Route to tier allocator
    uint32_t allocAddr = 0;
    bool ok = false;

    switch (cmd.tier) {
        case PGL_TIER_SRAM:
            // SRAM allocation: not supported via this command path.
            // SRAM resources are managed through the SceneState pools.
            scene->lastAllocResult.handle  = PGL_INVALID_MEM_HANDLE;
            scene->lastAllocResult.address = 0;
            scene->lastAllocResult.status  = PGL_ALLOC_TIER_DISABLED;
            return;

        case PGL_TIER_OPI_PSRAM:
            if (s_opi && s_opi->IsInitialized()) {
                allocAddr = s_opi->Alloc(cmd.size, 4);
                ok = (allocAddr != 0xFFFFFFFF);
            }
            break;

        case PGL_TIER_QSPI_PSRAM:
            if (s_qspi && s_qspi->IsInitialized()) {
                allocAddr = s_qspi->Alloc(cmd.size, 4);
                ok = (allocAddr != 0xFFFFFFFF);
            }
            break;

        default:
            break;
    }

    if (!ok) {
        // Check if tier is disabled vs out-of-memory
        bool tierEnabled = false;
        if (cmd.tier == PGL_TIER_OPI_PSRAM && s_opi && s_opi->IsInitialized())
            tierEnabled = true;
        if (cmd.tier == PGL_TIER_QSPI_PSRAM && s_qspi && s_qspi->IsInitialized())
            tierEnabled = true;

        scene->lastAllocResult.handle  = PGL_INVALID_MEM_HANDLE;
        scene->lastAllocResult.address = 0;
        scene->lastAllocResult.status  = tierEnabled
                                       ? PGL_ALLOC_OUT_OF_MEMORY
                                       : PGL_ALLOC_TIER_DISABLED;
        return;
    }

    // Record the allocation
    s_allocTable[handle].active  = true;
    s_allocTable[handle].tier    = cmd.tier;
    s_allocTable[handle].address = allocAddr;
    s_allocTable[handle].size    = cmd.size;
    s_allocTable[handle].tag     = cmd.tag;

    scene->lastAllocResult.handle  = handle;
    scene->lastAllocResult.address = allocAddr;
    scene->lastAllocResult.status  = PGL_ALLOC_OK;
}

static void HandleMemFree(const uint8_t*& ptr, SceneState* scene) {
    PglCmdMemFree cmd;
    PglReadStruct(ptr, cmd);

    if (cmd.handle >= PGL_MAX_MEM_ALLOCATIONS ||
        !s_allocTable[cmd.handle].active) {
        printf("[Parser] MemFree: invalid handle %u\n", cmd.handle);
        return;
    }

    // Mark the handle as free.
    // Note: With bump allocators, individual deallocation doesn't return memory
    // to the pool. The space is reclaimed only on FreeAll(). This is acceptable
    // for the GPU's allocation pattern (bulk load → render → reset).
    s_allocTable[cmd.handle].active = false;

    (void)scene;
}

static void HandleFramebufferCapture(const uint8_t*& ptr, SceneState* scene) {
    PglCmdFramebufferCapture cmd;
    PglReadStruct(ptr, cmd);

    // Select source buffer
    const uint16_t* srcBuf = (cmd.bufferSelect == 0) ? s_frontBuffer
                                                      : s_backBuffer;
    if (!srcBuf) {
        printf("[Parser] FramebufferCapture: buffer not available\n");
        scene->memStagingLength = 0;
        return;
    }

    uint32_t fbBytes = s_fbPixels * sizeof(uint16_t);  // RGB565

    if (cmd.format == 0) {
        // RGB565 native — direct copy
        uint32_t copyLen = std::min(fbBytes,
                                    static_cast<uint32_t>(SceneState::MEM_STAGING_SIZE));
        std::memcpy(scene->memStagingBuffer, srcBuf, copyLen);
        scene->memStagingLength  = copyLen;
        scene->memStagingReadPos = 0;
    } else {
        // RGB888 expansion: 2 bytes → 3 bytes per pixel
        uint32_t maxPixels = SceneState::MEM_STAGING_SIZE / 3;
        uint32_t pixelCount = std::min(s_fbPixels, maxPixels);
        uint8_t* dst = scene->memStagingBuffer;

        for (uint32_t i = 0; i < pixelCount; ++i) {
            uint16_t c = srcBuf[i];
            // RGB565 → RGB888 (5-6-5 → 8-8-8)
            dst[0] = static_cast<uint8_t>(((c >> 11) & 0x1F) * 255 / 31);
            dst[1] = static_cast<uint8_t>(((c >>  5) & 0x3F) * 255 / 63);
            dst[2] = static_cast<uint8_t>(((c      ) & 0x1F) * 255 / 31);
            dst += 3;
        }

        scene->memStagingLength  = pixelCount * 3;
        scene->memStagingReadPos = 0;
    }
}

static void HandleMemCopy(const uint8_t*& ptr, SceneState* scene) {
    PglCmdMemCopy cmd;
    PglReadStruct(ptr, cmd);

    if (cmd.size == 0) return;

    // Use the staging buffer as a temporary transfer buffer for cross-tier copies.
    // We process in chunks of MEM_STAGING_SIZE.
    uint32_t remaining = cmd.size;
    uint32_t srcOff = cmd.srcAddress;
    uint32_t dstOff = cmd.dstAddress;

    while (remaining > 0) {
        uint32_t chunk = std::min(remaining,
                                  static_cast<uint32_t>(SceneState::MEM_STAGING_SIZE));

        // Read from source tier into staging buffer
        bool readOk = false;
        if (cmd.srcTier == PGL_TIER_SRAM) {
            // SRAM source is not supported for cross-tier copy
            // (SRAM addresses are managed by scene state pools, not this path)
            printf("[Parser] MemCopy: SRAM source not supported\n");
            break;
        } else {
            readOk = TierReadInto(cmd.srcTier, srcOff,
                                  scene->memStagingBuffer, chunk);
        }

        if (!readOk) {
            printf("[Parser] MemCopy: read failed (srcTier=%u)\n", cmd.srcTier);
            break;
        }

        // Write from staging buffer to destination tier
        bool writeOk = false;
        if (cmd.dstTier == PGL_TIER_SRAM) {
            // SRAM destination: data is now in staging buffer for host readback.
            // (No further copy needed — staging IS the SRAM buffer.)
            writeOk = true;
        } else {
            writeOk = TierWriteTo(cmd.dstTier, dstOff,
                                  scene->memStagingBuffer, chunk);
        }

        if (!writeOk) {
            printf("[Parser] MemCopy: write failed (dstTier=%u)\n", cmd.dstTier);
            break;
        }

        remaining -= chunk;
        srcOff += chunk;
        dstOff += chunk;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// M12 Handlers: 2D Layer Lifecycle (0xA0 – 0xA2)
// ═══════════════════════════════════════════════════════════════════════════

static void HandleLayerCreate(const uint8_t*& ptr, SceneState* scene) {
    PglCmdLayerCreate cmd;
    PglReadStruct(ptr, cmd);

    if (cmd.layerId == PGL_LAYER_3D || cmd.layerId >= PGL_MAX_LAYERS) {
        printf("[Parser] LayerCreate: invalid layer %u\n", cmd.layerId);
        return;
    }

    LayerSlot& layer = scene->layers[cmd.layerId];

    // If already active with different dimensions, free the old buffer
    if (layer.active && layer.pixels &&
        (layer.width != cmd.width || layer.height != cmd.height)) {
        scene->FreeLayerFramebuffer(cmd.layerId);
    }

    layer.active      = true;
    layer.visible     = true;
    layer.dirty       = false;
    layer.width       = cmd.width;
    layer.height      = cmd.height;
    layer.pixelFormat = cmd.pixelFormat;
    layer.blendMode   = cmd.blendMode;
    layer.opacity     = cmd.opacity;
    layer.offsetX     = 0;
    layer.offsetY     = 0;

    if (!scene->AllocLayerFramebuffer(cmd.layerId)) {
        printf("[Parser] LayerCreate: failed to allocate FB for layer %u (%ux%u)\n",
               cmd.layerId, cmd.width, cmd.height);
        layer.active = false;
        return;
    }

    scene->activeLayerCount = 0;
    for (uint8_t i = 1; i < PGL_MAX_LAYERS; ++i) {
        if (scene->layers[i].active) scene->activeLayerCount++;
    }

    printf("[Parser] LayerCreate: layer %u (%ux%u, blend=%u, opacity=%u)\n",
           cmd.layerId, cmd.width, cmd.height, cmd.blendMode, cmd.opacity);
}

static void HandleLayerDestroy(const uint8_t*& ptr, SceneState* scene) {
    PglCmdLayerDestroy cmd;
    PglReadStruct(ptr, cmd);

    if (cmd.layerId == PGL_LAYER_3D || cmd.layerId >= PGL_MAX_LAYERS) {
        printf("[Parser] LayerDestroy: invalid layer %u\n", cmd.layerId);
        return;
    }

    scene->FreeLayerFramebuffer(cmd.layerId);
    scene->layers[cmd.layerId] = {};  // Zero the slot

    scene->activeLayerCount = 0;
    for (uint8_t i = 1; i < PGL_MAX_LAYERS; ++i) {
        if (scene->layers[i].active) scene->activeLayerCount++;
    }
}

static void HandleLayerSetProps(const uint8_t*& ptr, SceneState* scene) {
    PglCmdLayerSetProps cmd;
    PglReadStruct(ptr, cmd);

    if (cmd.layerId >= PGL_MAX_LAYERS || !scene->layers[cmd.layerId].active) {
        printf("[Parser] LayerSetProps: invalid/inactive layer %u\n", cmd.layerId);
        return;
    }

    LayerSlot& layer = scene->layers[cmd.layerId];
    layer.opacity   = cmd.opacity;
    layer.blendMode = cmd.blendMode;
    layer.offsetX   = cmd.offsetX;
    layer.offsetY   = cmd.offsetY;
}

// ═══════════════════════════════════════════════════════════════════════════
// M12 Handlers: 2D Drawing Primitives (0xA3 – 0xA6, 0xA9 – 0xAC)
// ═══════════════════════════════════════════════════════════════════════════

/// Helper: validate layer ID and enqueue a 2D draw command.
static bool Enqueue2D(SceneState* scene, uint8_t layerId, DrawCmd2DType type,
                      const void* payload, size_t payloadSize) {
    if (layerId >= PGL_MAX_LAYERS || !scene->layers[layerId].active) {
        printf("[Parser] 2D cmd type %u: invalid/inactive layer %u\n", type, layerId);
        return false;
    }
    DrawCmd2D cmd;
    cmd.type    = type;
    cmd.layerId = layerId;  // Store explicit layerId for Process2DDrawQueue
    std::memcpy(&cmd.rect, payload, payloadSize);  // union starts at .rect
    scene->layers[layerId].dirty = true;
    return scene->Enqueue2DCmd(cmd);
}

static void HandleDrawRect2D(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDrawRect2D cmd;
    PglReadStruct(ptr, cmd);
    Enqueue2D(scene, cmd.layerId, DRAW_CMD_2D_RECT, &cmd, sizeof(cmd));
}

static void HandleDrawLine2D(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDrawLine2D cmd;
    PglReadStruct(ptr, cmd);
    Enqueue2D(scene, cmd.layerId, DRAW_CMD_2D_LINE, &cmd, sizeof(cmd));
}

static void HandleDrawCircle2D(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDrawCircle2D cmd;
    PglReadStruct(ptr, cmd);
    Enqueue2D(scene, cmd.layerId, DRAW_CMD_2D_CIRCLE, &cmd, sizeof(cmd));
}

static void HandleDrawSprite(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDrawSprite cmd;
    PglReadStruct(ptr, cmd);
    Enqueue2D(scene, cmd.layerId, DRAW_CMD_2D_SPRITE, &cmd, sizeof(cmd));
}

static void HandleLayerClear(const uint8_t*& ptr, SceneState* scene) {
    PglCmdLayerClear cmd;
    PglReadStruct(ptr, cmd);
    Enqueue2D(scene, cmd.layerId, DRAW_CMD_2D_CLEAR, &cmd, sizeof(cmd));
}

static void HandleDrawRoundedRect(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDrawRoundedRect cmd;
    PglReadStruct(ptr, cmd);
    Enqueue2D(scene, cmd.layerId, DRAW_CMD_2D_ROUNDED_RECT, &cmd, sizeof(cmd));
}

static void HandleDrawArc(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDrawArc cmd;
    PglReadStruct(ptr, cmd);
    Enqueue2D(scene, cmd.layerId, DRAW_CMD_2D_ARC, &cmd, sizeof(cmd));
}

static void HandleDrawTriangle2D(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDrawTriangle2D cmd;
    PglReadStruct(ptr, cmd);
    Enqueue2D(scene, cmd.layerId, DRAW_CMD_2D_TRIANGLE, &cmd, sizeof(cmd));
}

// ═══════════════════════════════════════════════════════════════════════════
// M11 Handlers: Memory Pool Commands (0x38 – 0x3B)
// ═══════════════════════════════════════════════════════════════════════════

static void HandleMemPoolCreate(const uint8_t*& ptr, SceneState* scene) {
    PglCmdMemPoolCreate cmd;
    PglReadStruct(ptr, cmd);

    if (!s_poolMgr) {
        printf("[Parser] MemPoolCreate: pool manager not initialized\n");
        scene->lastAllocResult.handle  = 0xFFFF;
        scene->lastAllocResult.address = 0;
        scene->lastAllocResult.status  = 0xFF;
        return;
    }

    uint16_t handle = s_poolMgr->CreatePool(cmd.tier, cmd.blockSize,
                                              cmd.blockCount, cmd.tag);

    // Report result via I2C MEM_ALLOC_RESULT register
    scene->lastAllocResult.handle  = handle;
    scene->lastAllocResult.address = 0;  // Pool handle, not address
    scene->lastAllocResult.status  = (handle != 0xFFFF) ? 0x00 : 0xFF;

    printf("[Parser] MemPoolCreate: tier=%u blkSize=%u blkCount=%u tag=0x%04X → handle=%u\n",
           cmd.tier, cmd.blockSize, cmd.blockCount, cmd.tag, handle);
}

static void HandleMemPoolAlloc(const uint8_t*& ptr, SceneState* scene) {
    PglCmdMemPoolAlloc cmd;
    PglReadStruct(ptr, cmd);

    if (!s_poolMgr) {
        scene->lastAllocResult.handle  = cmd.poolHandle;
        scene->lastAllocResult.address = 0xFFFF;
        scene->lastAllocResult.status  = 0xFF;
        return;
    }

    uint16_t blockIdx = s_poolMgr->Alloc(cmd.poolHandle);

    // Report block index in the address field
    scene->lastAllocResult.handle  = cmd.poolHandle;
    scene->lastAllocResult.address = blockIdx;
    scene->lastAllocResult.status  = (blockIdx != 0xFFFF) ? 0x00 : 0x01;  // 0x01 = exhausted
}

static void HandleMemPoolFree(const uint8_t*& ptr, SceneState* scene) {
    PglCmdMemPoolFree cmd;
    PglReadStruct(ptr, cmd);

    if (!s_poolMgr) return;

    bool ok = s_poolMgr->Free(cmd.poolHandle, cmd.blockIndex);
    if (!ok) {
        printf("[Parser] MemPoolFree: failed pool=%u block=%u\n",
               cmd.poolHandle, cmd.blockIndex);
    }
    (void)scene;
}

static void HandleMemPoolDestroy(const uint8_t*& ptr, SceneState* scene) {
    PglCmdMemPoolDestroy cmd;
    PglReadStruct(ptr, cmd);

    if (!s_poolMgr) return;

    s_poolMgr->DestroyPool(cmd.poolHandle);
    (void)scene;
}

// ═══════════════════════════════════════════════════════════════════════════
// M11 Handlers: Display Driver Commands (0x90 – 0x91)
// ═══════════════════════════════════════════════════════════════════════════

static void HandleDisplayConfigure(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDisplayConfigure cmd;
    PglReadStruct(ptr, cmd);

    if (!s_displayMgr) {
        printf("[Parser] DisplayConfigure: display manager not initialized\n");
        return;
    }

    bool ok = s_displayMgr->ConfigureDisplay(cmd);
    if (!ok) {
        printf("[Parser] DisplayConfigure: failed for slot %u type 0x%02X\n",
               cmd.displayId, cmd.displayType);
    }
    (void)scene;
}

static void HandleDisplaySetRegion(const uint8_t*& ptr, SceneState* scene) {
    PglCmdDisplaySetRegion cmd;
    PglReadStruct(ptr, cmd);

    if (!s_displayMgr) {
        printf("[Parser] DisplaySetRegion: display manager not initialized\n");
        return;
    }

    s_displayMgr->SetRegion(cmd);
    (void)scene;
}

// ═══════════════════════════════════════════════════════════════════════════
// M12 Handlers: Memory Defrag (0x3C)
// ═══════════════════════════════════════════════════════════════════════════

static void HandleMemDefrag(const uint8_t*& ptr, SceneState* scene) {
    PglCmdMemDefrag cmd;
    PglReadStruct(ptr, cmd);

    // Update status to "active"
    scene->defragStatus.state = PGL_DEFRAG_ACTIVE;
    scene->defragStatus.tier  = cmd.tier;
    scene->defragStatus.movedKB = 0;

    if (!s_tier) {
        printf("[Parser] MemDefrag: no tier manager — skipping\n");
        scene->defragStatus.state = PGL_DEFRAG_COMPLETED;
        return;
    }

    // Map wire tier value to MemTier enum
    // PGL_TIER_AUTO (0xFF) → MemTier::NONE (defrag all tiers)
    MemTier targetTier;
    switch (cmd.tier) {
        case 0:    targetTier = MemTier::SRAM;   break;
        case 1:    targetTier = MemTier::QSPI_A; break;
        case 2:    targetTier = MemTier::QSPI_B; break;
        default:   targetTier = MemTier::NONE;    break;  // auto = all
    }

    uint16_t movedKB = 0;
    uint16_t fragmentCount = 0;
    uint16_t largestFreeKB = 0;

    bool completed = s_tier->Defragment(
        targetTier, cmd.mode, cmd.maxMoveKB,
        movedKB, fragmentCount, largestFreeKB);

    scene->defragStatus.movedKB       = movedKB;
    scene->defragStatus.fragmentCount = fragmentCount;
    scene->defragStatus.largestFreeKB = largestFreeKB;
    scene->defragStatus.state = completed
        ? PGL_DEFRAG_COMPLETED : PGL_DEFRAG_ACTIVE;

    printf("[Parser] MemDefrag: tier=%u mode=%u maxMoveKB=%u → moved=%u KB, "
           "fragments=%u, largestFree=%u KB, %s\n",
           cmd.tier, cmd.mode, cmd.maxMoveKB, movedKB,
           fragmentCount, largestFreeKB,
           completed ? "completed" : "in-progress");
}

// ═══════════════════════════════════════════════════════════════════════════
// M12 Handlers: Direct Framebuffer Write (0x45)
// ═══════════════════════════════════════════════════════════════════════════

static void HandleWriteFramebuffer(const uint8_t*& ptr, uint16_t payloadLen,
                                   SceneState* scene) {
    PglCmdWriteFramebufferHeader hdr;
    PglReadStruct(ptr, hdr);

    const uint32_t pixelDataBytes = payloadLen - sizeof(hdr);
    const uint32_t expectedBytes  = uint32_t(hdr.w) * hdr.h * 2;  // RGB565

    if (pixelDataBytes < expectedBytes) {
        printf("[Parser] WriteFramebuffer: truncated data (%u < %u)\n",
               pixelDataBytes, expectedBytes);
        PglSkip(ptr, pixelDataBytes);
        return;
    }

    // Determine target buffer
    uint16_t* dst    = nullptr;
    uint16_t  dstW   = 0;
    uint16_t  dstH   = 0;

    if (hdr.layerId == 0xFF) {
        // Write directly to 3D back buffer
        dst  = const_cast<uint16_t*>(s_backBuffer);
        dstW = GpuConfig::PANEL_WIDTH;
        dstH = GpuConfig::PANEL_HEIGHT;
    } else if (hdr.layerId < PGL_MAX_LAYERS &&
               scene->layers[hdr.layerId].active &&
               scene->layers[hdr.layerId].pixels) {
        LayerSlot& layer = scene->layers[hdr.layerId];
        dst  = layer.pixels;
        dstW = layer.width;
        dstH = layer.height;
        layer.dirty = true;
    } else {
        printf("[Parser] WriteFramebuffer: invalid layer %u\n", hdr.layerId);
        PglSkip(ptr, pixelDataBytes);
        return;
    }

    // Copy pixel data with clipping
    const uint16_t* srcPixels = reinterpret_cast<const uint16_t*>(ptr);

    for (uint16_t row = 0; row < hdr.h; ++row) {
        int32_t dstY = hdr.y + row;
        if (dstY < 0 || dstY >= dstH) { srcPixels += hdr.w; continue; }

        int32_t sx = 0;
        int32_t dx = hdr.x;
        int32_t copyW = hdr.w;

        // Left clipping
        if (dx < 0) { sx = -dx; copyW += dx; dx = 0; }
        // Right clipping
        if (dx + copyW > dstW) { copyW = dstW - dx; }

        if (copyW > 0) {
            std::memcpy(&dst[dstY * dstW + dx],
                        &srcPixels[sx],
                        copyW * sizeof(uint16_t));
        }
        srcPixels += hdr.w;
    }

    PglSkip(ptr, pixelDataBytes);
}

// ═══════════════════════════════════════════════════════════════════════════
// M12 Handlers: Resource Persistence (0x46 – 0x48)
// ═══════════════════════════════════════════════════════════════════════════

static void HandlePersistResource(const uint8_t*& ptr, SceneState* scene) {
    PglCmdPersistResource cmd;
    PglReadStruct(ptr, cmd);

    if (!s_flashPersistInitialized) {
        printf("[Parser] PersistResource: flash persistence not initialized\n");
        scene->persistStatus.state = PGL_PERSIST_ERROR;
        PglSkip(ptr, 0);
        return;
    }

    // Look up resource data via tier manager
    const void* dataPtr = nullptr;
    uint32_t dataSize = 0;

    if (s_tier) {
        MemRecord* rec = s_tier->FindRecord(cmd.resourceId);
        if (rec && rec->sramPtr && rec->dataSize > 0) {
            dataPtr  = rec->sramPtr;
            dataSize = rec->dataSize;
        }
    }

    if (!dataPtr || dataSize == 0) {
        printf("[Parser] PersistResource: resource %u:%u not found in memory\n",
               cmd.resourceClass, cmd.resourceId);
        scene->persistStatus.state          = PGL_PERSIST_ERROR;
        scene->persistStatus.lastResourceId = cmd.resourceId;
        return;
    }

    bool queued = s_flashPersist.PersistResource(
        cmd.resourceClass, cmd.resourceId, cmd.flags,
        dataPtr, dataSize);

    scene->persistStatus.lastResourceId = cmd.resourceId;
    if (queued) {
        scene->persistStatus.state = PGL_PERSIST_WRITING;
        printf("[Parser] PersistResource: queued class=%u id=%u size=%lu\n",
               cmd.resourceClass, cmd.resourceId, (unsigned long)dataSize);
    } else {
        scene->persistStatus.state = PGL_PERSIST_ERROR;
        printf("[Parser] PersistResource: failed to queue class=%u id=%u\n",
               cmd.resourceClass, cmd.resourceId);
    }
}

static void HandleRestoreResource(const uint8_t*& ptr, SceneState* scene) {
    PglCmdRestoreResource cmd;
    PglReadStruct(ptr, cmd);

    if (!s_flashPersistInitialized) {
        printf("[Parser] RestoreResource: flash persistence not initialized\n");
        scene->persistStatus.state = PGL_PERSIST_ERROR;
        return;
    }

    const void* restored = s_flashPersist.RestoreResource(
        cmd.resourceClass, cmd.resourceId);

    scene->persistStatus.lastResourceId = cmd.resourceId;
    if (restored) {
        scene->persistStatus.state = PGL_PERSIST_IDLE;
        printf("[Parser] RestoreResource: restored class=%u id=%u\n",
               cmd.resourceClass, cmd.resourceId);
    } else {
        scene->persistStatus.state = PGL_PERSIST_ERROR;
        printf("[Parser] RestoreResource: failed class=%u id=%u\n",
               cmd.resourceClass, cmd.resourceId);
    }
}

static void HandleQueryPersistence(const uint8_t*& ptr, SceneState* scene) {
    PglCmdQueryPersistence cmd;
    PglReadStruct(ptr, cmd);

    if (s_flashPersistInitialized) {
        s_flashPersist.GetStatus(scene->persistStatus);

        // If querying a specific resource, check if it's persisted
        if (cmd.resourceClass != 0xFF) {
            bool persisted = s_flashPersist.IsResourcePersisted(
                cmd.resourceClass, cmd.resourceId);
            printf("[Parser] QueryPersistence: class=%u id=%u → %s\n",
                   cmd.resourceClass, cmd.resourceId,
                   persisted ? "persisted" : "not found");
        } else {
            printf("[Parser] QueryPersistence: manifest has %u entries\n",
                   scene->persistStatus.manifestEntries);
        }
    } else {
        printf("[Parser] QueryPersistence: not initialized\n");
        scene->persistStatus.state = PGL_PERSIST_IDLE;
    }
}
