// PSB1 blob builder shared by the native shader tests.
//
// Builds exact-layout PSB1 blobs (header + uniform descriptors + constants +
// packed instructions) including deliberately malformed variants.  Test-only
// utility; not part of the firmware or ProtoGL API.

#pragma once

#include <cstdint>
#include <cstring>

#include <PglShaderBytecode.h>

struct PsbBlobBuilder {
    uint8_t data[PSB_MAX_PROGRAM_SIZE + 16];  // headroom for trailing-garbage cases
    size_t  size = 0;

    void Append(const void* p, size_t n) {
        std::memcpy(data + size, p, n);
        size += n;
    }

    void Header(uint8_t flags, uint8_t constCount, uint8_t uniformCount,
                uint16_t instrCount,
                uint32_t magic = PSB_MAGIC, uint8_t version = PSB_VERSION,
                uint16_t reserved = 0) {
        PglShaderProgramHeader h{};
        h.magic        = magic;
        h.version      = version;
        h.flags        = flags;
        h.constCount   = constCount;
        h.uniformCount = uniformCount;
        h.instrCount   = instrCount;
        h.nameHash     = 0;
        h.reserved     = reserved;
        Append(&h, sizeof(h));
    }

    void Uniform(uint32_t nameHash, uint8_t type, uint8_t slot, uint16_t defOff) {
        PglUniformDescriptor d{};
        d.nameHash           = nameHash;
        d.type               = type;
        d.slot               = slot;
        d.defaultValueOffset = defOff;
        Append(&d, sizeof(d));
    }

    void Const(float f) { Append(&f, sizeof(f)); }

    void ConstBits(uint32_t bits) { Append(&bits, sizeof(bits)); }  // NaN/inf injection

    void Instr(uint8_t opcode, uint8_t dst, uint8_t srcA, uint8_t srcB) {
        const uint32_t w = static_cast<uint32_t>(opcode)
                         | (static_cast<uint32_t>(dst)  << 8)
                         | (static_cast<uint32_t>(srcA) << 16)
                         | (static_cast<uint32_t>(srcB) << 24);
        Append(&w, sizeof(w));
    }

    void End() { Instr(PSB_OP_END, PSB_OP_UNUSED, PSB_OP_UNUSED, PSB_OP_UNUSED); }
};

// Standard valid passthrough program: gl_FragColor = input pixel.
//   MOV r28,r4 / MOV r29,r5 / MOV r30,r6 / END
inline size_t PsbBuildPassthrough(uint8_t* out) {
    PsbBlobBuilder b;
    b.Header(0, 0, 0, 4);
    b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, PSB_REG_IN_R, PSB_OP_UNUSED);
    b.Instr(PSB_OP_MOV, PSB_REG_OUT_G, PSB_REG_IN_G, PSB_OP_UNUSED);
    b.Instr(PSB_OP_MOV, PSB_REG_OUT_B, PSB_REG_IN_B, PSB_OP_UNUSED);
    b.End();
    std::memcpy(out, b.data, b.size);
    return b.size;
}

// Invert program (no TEX2D): gl_FragColor.rgb = 1 - in.rgb
//   SUB r28,lit1,r4 / SUB r29,lit1,r5 / SUB r30,lit1,r6 / END
inline size_t PsbBuildInvert(uint8_t* out) {
    constexpr uint8_t LIT1 = PSB_OP_LITERAL_BASE + 2;  // 1.0
    PsbBlobBuilder b;
    b.Header(0, 0, 0, 4);
    b.Instr(PSB_OP_SUB, PSB_REG_OUT_R, LIT1, PSB_REG_IN_R);
    b.Instr(PSB_OP_SUB, PSB_REG_OUT_G, LIT1, PSB_REG_IN_G);
    b.Instr(PSB_OP_SUB, PSB_REG_OUT_B, LIT1, PSB_REG_IN_B);
    b.End();
    std::memcpy(out, b.data, b.size);
    return b.size;
}

//   r8 = fragX + 1.0 ; r9 = fragY ; r8 /= u0 ; r9 /= u1 ;
//   TEX2D r16, r8 ; MOV r28,r16 / r29,r17 / r30,r18 ; END
// (u0/u1 are the auto-bound resolution uniforms.)
inline size_t PsbBuildTexShift(uint8_t* out) {
    PsbBlobBuilder b;
    constexpr uint8_t LIT1  = PSB_OP_LITERAL_BASE + 2;      // 1.0
    constexpr uint8_t U_RES = PSB_OP_UNIFORM_BASE + 0;      // u_resolution.x
    constexpr uint8_t V_RES = PSB_OP_UNIFORM_BASE + 1;      // u_resolution.y
    b.Header(PSB_FLAG_NEEDS_SCRATCH_COPY, 0, 0, 9);
    b.Instr(PSB_OP_ADD,   8, PSB_REG_FRAG_X, LIT1);
    b.Instr(PSB_OP_MOV,   9, PSB_REG_FRAG_Y, PSB_OP_UNUSED);
    b.Instr(PSB_OP_DIV,   8, 8, U_RES);
    b.Instr(PSB_OP_DIV,   9, 9, V_RES);
    b.Instr(PSB_OP_TEX2D, 16, 8, PSB_OP_UNUSED);
    b.Instr(PSB_OP_MOV,   PSB_REG_OUT_R, 16, PSB_OP_UNUSED);
    b.Instr(PSB_OP_MOV,   PSB_REG_OUT_G, 17, PSB_OP_UNUSED);
    b.Instr(PSB_OP_MOV,   PSB_REG_OUT_B, 18, PSB_OP_UNUSED);
    b.End();
    std::memcpy(out, b.data, b.size);
    return b.size;
}
