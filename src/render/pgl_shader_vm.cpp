/**
 * @file pgl_shader_vm.cpp
 * @brief PGL Shader VM — bytecode interpreter implementation.
 *
 * Core interpreter loop with ~50 opcodes covering arithmetic, math functions,
 * clamping/interpolation, geometric operations, texture sampling, and load.
 *
 * All math operations are dispatched through PglShaderBackend, which provides
 * platform-portable implementations selected at compile time:
 *   - PGL_BACKEND_SCALAR_FLOAT  — standard C <cmath> (default)
 *   - PGL_BACKEND_CM33_FPV5     — Cortex-M33 FPv5 FMA / VSQRT
 *   - PGL_BACKEND_SOFT_FLOAT    — integer-only approximations (RISC-V no FPU)
 *
 * Design rules:
 *   - Fixed-width 4-byte instructions for fast sequential decode
 *   - Jump-table dispatch (S-03): one handler function per opcode in a
 *     256-entry constexpr table indexed directly by the opcode byte;
 *     unknown opcodes fail closed to a shared skip handler (the old flat
 *     switch's default case)
 *   - No heap allocation — register file is on the stack
 *   - Each handler resolves exactly the operands its opcode reads, inlined
 *     via PsbResolveOperand() — unary ops no longer decode srcB (S-03)
 *   - Read-all-then-write: every handler reads ALL of its sources (including
 *     regs[dst] for the 3-operand forms) into locals BEFORE storing the
 *     result — the PGLSL register allocator's LIFO temp scheme depends on it.
 *
 * Performance: ~8 cycles per instruction → 40-instruction shader ≈ 320 cycles/pixel
 *              8192 pixels × 320 = 2.6M cycles ≈ 0.017 ms @ 150 MHz.
 */

#include "pgl_shader_vm.h"
#include "../scene_state.h"

#include <PglShaderBytecode.h>
#include <PglShaderBackend.h>

#include <array>
#include <cstring>

// Namespace alias for brevity in the opcode handlers
namespace BE = PglShaderBackend;

namespace {

// ─── Execution context (one per Execute() call) ─────────────────────────────
// Bundles everything a handler may need so handlers stay plain function
// pointers (no captures, no heap, no per-instruction re-fetch from prog).

struct PsbVmContext {
    float*          regs;        // 32-register file (lives in PglShaderVM)
    const float*    uniforms;    // PSB_MAX_UNIFORMS entries
    const float*    constants;   // PSB_MAX_CONSTANTS entries
    const uint16_t* fb;          // framebuffer for TEX2D sampling
    uint16_t        fbW;
    uint16_t        fbH;
};

// Handler contract: dst/srcA/srcB are the raw 8-bit operand fields; each
// handler decodes and resolves exactly the operands its opcode actually
// reads — nothing more.  Return false to continue, true to halt (PSB_OP_END).
typedef bool (*PsbOpHandler)(PsbVmContext& ctx,
                             uint8_t dst, uint8_t srcA, uint8_t srcB);

// Guarded scalar store (the old loop's shared epilogue): a dst operand that
// does not encode a register (uniform/constant/literal/unused byte) still
// computes the result but discards it.
inline void PsbWriteDst(PsbVmContext& ctx, uint8_t dst, float value) {
    if (dst <= PSB_OP_REG_END) ctx.regs[dst] = value;
}

// ─── Scalar handler generators ──────────────────────────────────────────────
// One function per opcode so the dispatch table can index them directly.
//   UNARY   — resolves srcA only (S-03: no eager srcB decode)
//   BINARY  — resolves srcA + srcB
//   TERNARY — additionally reads regs[dst & 0x1F] as the third source BEFORE
//             the store (FMA addend / clamp hi / mix t / smoothstep x)

#define PSB_HANDLER_UNARY(opName, expression)                                   \
    bool Op##opName(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t) {    \
        const float a = PsbResolveOperand(srcA, ctx.regs, ctx.uniforms,         \
                                          ctx.constants);                       \
        PsbWriteDst(ctx, dst, (expression));                                    \
        return false;                                                           \
    }

#define PSB_HANDLER_BINARY(opName, expression)                                  \
    bool Op##opName(PsbVmContext& ctx, uint8_t dst, uint8_t srcA,               \
                    uint8_t srcB) {                                             \
        const float a = PsbResolveOperand(srcA, ctx.regs, ctx.uniforms,         \
                                          ctx.constants);                       \
        const float b = PsbResolveOperand(srcB, ctx.regs, ctx.uniforms,         \
                                          ctx.constants);                       \
        PsbWriteDst(ctx, dst, (expression));                                    \
        return false;                                                           \
    }

#define PSB_HANDLER_TERNARY(opName, expression)                                 \
    bool Op##opName(PsbVmContext& ctx, uint8_t dst, uint8_t srcA,               \
                    uint8_t srcB) {                                             \
        const float a = PsbResolveOperand(srcA, ctx.regs, ctx.uniforms,         \
                                          ctx.constants);                       \
        const float b = PsbResolveOperand(srcB, ctx.regs, ctx.uniforms,         \
                                          ctx.constants);                       \
        const float c = ctx.regs[dst & 0x1F];                                   \
        PsbWriteDst(ctx, dst, (expression));                                    \
        return false;                                                           \
    }

// ── Special ─────────────────────────────────────────────────────────────────

bool OpNop(PsbVmContext&, uint8_t, uint8_t, uint8_t)     { return false; }
bool OpEnd(PsbVmContext&, uint8_t, uint8_t, uint8_t)     { return true;  }
// Unknown/unassigned opcode — skip (fail closed, same as the old default case)
bool OpUnknown(PsbVmContext&, uint8_t, uint8_t, uint8_t) { return false; }

// ── Arithmetic ──────────────────────────────────────────────────────────────

PSB_HANDLER_UNARY  (Mov,   a)                 // 0x01 dst = srcA
PSB_HANDLER_BINARY (Add,   BE::Add(a, b))     // 0x02
PSB_HANDLER_BINARY (Sub,   BE::Sub(a, b))     // 0x03
PSB_HANDLER_BINARY (Mul,   BE::Mul(a, b))     // 0x04
PSB_HANDLER_BINARY (Div,   BE::Div(a, b))     // 0x05
PSB_HANDLER_TERNARY(Fma,   BE::Fma(a, b, c))  // 0x06 dst = srcA*srcB + dst
PSB_HANDLER_UNARY  (Neg,   BE::Neg(a))        // 0x07

// ── Math functions ──────────────────────────────────────────────────────────

PSB_HANDLER_UNARY  (Sin,   BE::Sin(a))        // 0x10
PSB_HANDLER_UNARY  (Cos,   BE::Cos(a))        // 0x11
PSB_HANDLER_UNARY  (Tan,   BE::Tan(a))        // 0x12
PSB_HANDLER_UNARY  (Asin,  BE::Asin(a))       // 0x13
PSB_HANDLER_UNARY  (Acos,  BE::Acos(a))       // 0x14
PSB_HANDLER_UNARY  (Atan,  BE::Atan(a))       // 0x15
PSB_HANDLER_BINARY (Atan2, BE::Atan2(a, b))   // 0x16
PSB_HANDLER_BINARY (Pow,   BE::Pow(a, b))     // 0x17
PSB_HANDLER_UNARY  (Exp,   BE::Exp(a))        // 0x18
PSB_HANDLER_UNARY  (Log,   BE::Log(a))        // 0x19
PSB_HANDLER_UNARY  (Sqrt,  BE::Sqrt(a))       // 0x1A
PSB_HANDLER_UNARY  (Rsqrt, BE::Rsqrt(a))      // 0x1B
PSB_HANDLER_UNARY  (Abs,   BE::Abs(a))        // 0x1C
PSB_HANDLER_UNARY  (Sign,  BE::Sign(a))       // 0x1D
PSB_HANDLER_UNARY  (Floor, BE::Floor(a))      // 0x1E
PSB_HANDLER_UNARY  (Ceil,  BE::Ceil(a))       // 0x1F
PSB_HANDLER_UNARY  (Fract, BE::Fract(a))      // 0x20
PSB_HANDLER_BINARY (Mod,   BE::Mod(a, b))     // 0x21

// ── Clamping / interpolation ────────────────────────────────────────────────

PSB_HANDLER_BINARY (Min,   BE::Min(a, b))          // 0x30
PSB_HANDLER_BINARY (Max,   BE::Max(a, b))          // 0x31
PSB_HANDLER_TERNARY(Clamp, BE::Clamp(a, b, c))     // 0x32 clamp(a, lo=b, hi=dst)
PSB_HANDLER_TERNARY(Mix,   BE::Mix(a, b, c))       // 0x33 mix(a, b, t=dst)
PSB_HANDLER_BINARY (Step,  BE::Step(a, b))         // 0x34
PSB_HANDLER_TERNARY(Sstep, BE::Smoothstep(a, b, c))// 0x35 smoothstep(a, b, x=dst)

// ── Geometric (vector ops on consecutive registers) ─────────────────────────
// Operand bytes are raw register indices here (masked & 0x1F) — they are NOT
// PsbResolveOperand-resolved.  Multi-result ops write regs directly.

bool OpDot2(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t srcB) {
    const uint8_t ai = srcA & 0x1F;
    const uint8_t bi = srcB & 0x1F;
    PsbWriteDst(ctx, dst, BE::Dot2(ctx.regs[ai], ctx.regs[ai + 1],
                                   ctx.regs[bi], ctx.regs[bi + 1]));
    return false;
}

bool OpDot3(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t srcB) {
    const uint8_t ai = srcA & 0x1F;
    const uint8_t bi = srcB & 0x1F;
    PsbWriteDst(ctx, dst, BE::Dot3(ctx.regs[ai], ctx.regs[ai + 1], ctx.regs[ai + 2],
                                   ctx.regs[bi], ctx.regs[bi + 1], ctx.regs[bi + 2]));
    return false;
}

bool OpLen2(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t) {
    const uint8_t ai = srcA & 0x1F;
    PsbWriteDst(ctx, dst, BE::Len2(ctx.regs[ai], ctx.regs[ai + 1]));
    return false;
}

bool OpLen3(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t) {
    const uint8_t ai = srcA & 0x1F;
    PsbWriteDst(ctx, dst, BE::Len3(ctx.regs[ai], ctx.regs[ai + 1],
                                   ctx.regs[ai + 2]));
    return false;
}

bool OpNorm2(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t) {
    const uint8_t ai = srcA & 0x1F;
    const uint8_t di = dst & 0x1F;
    BE::Norm2(ctx.regs[ai], ctx.regs[ai + 1],
              ctx.regs[di], ctx.regs[di + 1]);
    return false;
}

bool OpNorm3(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t) {
    const uint8_t ai = srcA & 0x1F;
    const uint8_t di = dst & 0x1F;
    BE::Norm3(ctx.regs[ai], ctx.regs[ai + 1], ctx.regs[ai + 2],
              ctx.regs[di], ctx.regs[di + 1], ctx.regs[di + 2]);
    return false;
}

bool OpCross(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t srcB) {
    const uint8_t ai = srcA & 0x1F;
    const uint8_t bi = srcB & 0x1F;
    const uint8_t di = dst & 0x1F;
    BE::Cross(ctx.regs[ai], ctx.regs[ai + 1], ctx.regs[ai + 2],
              ctx.regs[bi], ctx.regs[bi + 1], ctx.regs[bi + 2],
              ctx.regs[di], ctx.regs[di + 1], ctx.regs[di + 2]);
    return false;
}

bool OpDist2(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t srcB) {
    const uint8_t ai = srcA & 0x1F;
    const uint8_t bi = srcB & 0x1F;
    PsbWriteDst(ctx, dst, BE::Dist2(ctx.regs[ai], ctx.regs[ai + 1],
                                    ctx.regs[bi], ctx.regs[bi + 1]));
    return false;
}

// ── Texture sampling ────────────────────────────────────────────────────────

bool OpTex2D(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t) {
    // Sample framebuffer at (srcA, srcA+1) as UV coords [0,1]
    // Write RGBA to dst..dst+3
    const uint8_t ai = srcA & 0x1F;
    const uint8_t di = dst & 0x1F;
    float texR, texG, texB;
    BE::TexSample(ctx.fb, ctx.fbW, ctx.fbH,
                  ctx.regs[ai], ctx.regs[ai + 1], texR, texG, texB);
    ctx.regs[di]     = texR;
    ctx.regs[di + 1] = texG;
    ctx.regs[di + 2] = texB;
    ctx.regs[di + 3] = 1.0f;
    return false;
}

// ── Load (srcA is a raw pool index — NOT an encoded operand) ────────────────

bool OpLconst(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t) {
    PsbWriteDst(ctx, dst,
                (srcA < PSB_MAX_CONSTANTS) ? ctx.constants[srcA] : 0.0f);
    return false;
}

bool OpLuni(PsbVmContext& ctx, uint8_t dst, uint8_t srcA, uint8_t) {
    PsbWriteDst(ctx, dst,
                (srcA < PSB_MAX_UNIFORMS) ? ctx.uniforms[srcA] : 0.0f);
    return false;
}

// ─── Dispatch table (S-03) ──────────────────────────────────────────────────
// 256 entries indexed directly by the opcode byte — no range checks in the
// hot loop.  Unassigned opcodes fail closed to OpUnknown (skip), exactly like
// the old switch's default case.  Built entirely at compile time
// (256 × 4 B = 1 KB of flash on the RP2350).

constexpr std::array<PsbOpHandler, 256> PsbBuildDispatchTable() {
    std::array<PsbOpHandler, 256> t{};
    for (PsbOpHandler& h : t) h = OpUnknown;

    // Special
    t[PSB_OP_NOP]    = OpNop;

    // Arithmetic
    t[PSB_OP_MOV]    = OpMov;
    t[PSB_OP_ADD]    = OpAdd;
    t[PSB_OP_SUB]    = OpSub;
    t[PSB_OP_MUL]    = OpMul;
    t[PSB_OP_DIV]    = OpDiv;
    t[PSB_OP_FMA]    = OpFma;
    t[PSB_OP_NEG]    = OpNeg;

    // Math functions
    t[PSB_OP_SIN]    = OpSin;
    t[PSB_OP_COS]    = OpCos;
    t[PSB_OP_TAN]    = OpTan;
    t[PSB_OP_ASIN]   = OpAsin;
    t[PSB_OP_ACOS]   = OpAcos;
    t[PSB_OP_ATAN]   = OpAtan;
    t[PSB_OP_ATAN2]  = OpAtan2;
    t[PSB_OP_POW]    = OpPow;
    t[PSB_OP_EXP]    = OpExp;
    t[PSB_OP_LOG]    = OpLog;
    t[PSB_OP_SQRT]   = OpSqrt;
    t[PSB_OP_RSQRT]  = OpRsqrt;
    t[PSB_OP_ABS]    = OpAbs;
    t[PSB_OP_SIGN]   = OpSign;
    t[PSB_OP_FLOOR]  = OpFloor;
    t[PSB_OP_CEIL]   = OpCeil;
    t[PSB_OP_FRACT]  = OpFract;
    t[PSB_OP_MOD]    = OpMod;

    // Clamping / interpolation
    t[PSB_OP_MIN]    = OpMin;
    t[PSB_OP_MAX]    = OpMax;
    t[PSB_OP_CLAMP]  = OpClamp;
    t[PSB_OP_MIX]    = OpMix;
    t[PSB_OP_STEP]   = OpStep;
    t[PSB_OP_SSTEP]  = OpSstep;

    // Geometric
    t[PSB_OP_DOT2]   = OpDot2;
    t[PSB_OP_DOT3]   = OpDot3;
    t[PSB_OP_LEN2]   = OpLen2;
    t[PSB_OP_LEN3]   = OpLen3;
    t[PSB_OP_NORM2]  = OpNorm2;
    t[PSB_OP_NORM3]  = OpNorm3;
    t[PSB_OP_CROSS]  = OpCross;
    t[PSB_OP_DIST2]  = OpDist2;

    // Texture sampling
    t[PSB_OP_TEX2D]  = OpTex2D;

    // Load
    t[PSB_OP_LCONST] = OpLconst;
    t[PSB_OP_LUNI]   = OpLuni;

    // Halt
    t[PSB_OP_END]    = OpEnd;

    return t;
}

constexpr std::array<PsbOpHandler, 256> kPsbDispatch = PsbBuildDispatchTable();

}  // namespace

// ─── VM Execute ─────────────────────────────────────────────────────────────

void PglShaderVM::Execute(const ShaderProgram& prog,
                           float fragX, float fragY,
                           float inR, float inG, float inB,
                           const uint16_t* fb, uint16_t w, uint16_t h,
                           float& outR, float& outG, float& outB) {

    // ── Auto-load built-in registers ────────────────────────────────────
    regs_[PSB_REG_FRAG_X] = fragX;
    regs_[PSB_REG_FRAG_Y] = fragY;
    regs_[PSB_REG_FRAG_Z] = 0.0f;
    regs_[PSB_REG_FRAG_W] = 1.0f;
    regs_[PSB_REG_IN_R]   = inR;
    regs_[PSB_REG_IN_G]   = inG;
    regs_[PSB_REG_IN_B]   = inB;
    regs_[PSB_REG_IN_A]   = 1.0f;

    // Zero user temporaries to avoid undefined behaviour
    for (int i = PSB_REG_USER_START; i <= PSB_REG_USER_END; ++i)
        regs_[i] = 0.0f;

    // Default output = input (passthrough if shader doesn't write)
    regs_[PSB_REG_OUT_R] = inR;
    regs_[PSB_REG_OUT_G] = inG;
    regs_[PSB_REG_OUT_B] = inB;
    regs_[PSB_REG_OUT_A] = 1.0f;

    // ── Main interpreter loop: fetch → decode → dispatch via jump table ─
    PsbVmContext ctx;
    ctx.regs      = regs_;
    ctx.uniforms  = prog.uniforms;
    ctx.constants = prog.constants;
    ctx.fb        = fb;
    ctx.fbW       = w;
    ctx.fbH       = h;

    const uint16_t instrCount = prog.instrCount;

    for (uint16_t pc = 0; pc < instrCount; ++pc) {
        const uint32_t raw = prog.instructions[pc];

        // Decode 4-byte instruction: [opcode][dst][srcA][srcB]
        const uint8_t opcode = static_cast<uint8_t>(raw & 0xFF);
        const uint8_t dstOp  = static_cast<uint8_t>((raw >> 8) & 0xFF);
        const uint8_t srcAOp = static_cast<uint8_t>((raw >> 16) & 0xFF);
        const uint8_t srcBOp = static_cast<uint8_t>((raw >> 24) & 0xFF);

        // Handlers return true only for PSB_OP_END (halt).
        if (kPsbDispatch[opcode](ctx, dstOp, srcAOp, srcBOp))
            break;
    }

    // ── Read output from gl_FragColor registers ─────────────────────────
    outR = regs_[PSB_REG_OUT_R];
    outG = regs_[PSB_REG_OUT_G];
    outB = regs_[PSB_REG_OUT_B];
}

