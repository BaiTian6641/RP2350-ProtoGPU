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
 *     256-entry constexpr table indexed directly by the opcode byte
 *   - No heap allocation — register file is on the stack
 *   - Each handler resolves exactly the operands its opcode reads, inlined
 *     via PsbResolveOperand() — unary ops no longer decode srcB (S-03)
 *   - Read-all-then-write: every handler reads ALL of its sources (including
 *     regs[dst] for the 3-operand forms) into locals BEFORE storing the
 *     result — the PGLSL register allocator's LIFO temp scheme depends on it.
 *
 * Verification (P05-08): unknown opcodes, malformed operands and out-of-range
 * vector register bases are rejected ONCE by DecodeShaderProgram() at upload;
 * Execute() below runs only verified programs and performs no per-pixel
 * re-validation.  The dispatch table's OpUnknown entry is retained purely as
 * fail-closed defence in depth.
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
#include <cmath>

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

void PglShaderVM::Execute(const ShaderProgram& prog, const float* uniforms,
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
    ctx.uniforms  = uniforms;
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
    // Non-finite containment (P05-08): intermediate overflow (e.g. MUL of
    // large constants, EXP(x>88)) may leave NaN/±inf in the output
    // registers; a shader must never poison the framebuffer, so non-finite
    // channels are contained to 0.0 here — once per pixel, not per upload.
    outR = std::isfinite(regs_[PSB_REG_OUT_R]) ? regs_[PSB_REG_OUT_R] : 0.0f;
    outG = std::isfinite(regs_[PSB_REG_OUT_G]) ? regs_[PSB_REG_OUT_G] : 0.0f;
    outB = std::isfinite(regs_[PSB_REG_OUT_B]) ? regs_[PSB_REG_OUT_B] : 0.0f;
}

// ═══════════════════════════════════════════════════════════════════════════
// ── PSB1 blob decode + verification (P05-08) ────────────────────────────
// ═══════════════════════════════════════════════════════════════════════════

namespace {

// Per-opcode verification/cost rule.  Vector widths >1 require REGISTER
// operands whose base + width stays inside the 32-register file.
struct PsbOpRule {
    uint8_t dstVec;   // 0 = no destination (operand must be UNUSED), else register vector width written
    uint8_t srcAVec;  // 0 = operand unused, 1 = readable scalar operand, 2..4 = register vector width
    uint8_t srcBVec;  // same encoding as srcAVec
    uint8_t weight;   // weighted ops charged per executed pixel (frame budget accounting)
};

constexpr std::array<PsbOpRule, 256> PsbBuildRuleTable() {
    // {dstVec, srcAVec, srcBVec, weight}; {} = invalid opcode (rejected)
    std::array<PsbOpRule, 256> t{};

    t[PSB_OP_NOP]   = {0, 0, 0, 0};
    t[PSB_OP_END]   = {0, 0, 0, 0};

    t[PSB_OP_MOV]   = {1, 1, 0, 1};
    t[PSB_OP_ADD]   = {1, 1, 1, 2};
    t[PSB_OP_SUB]   = {1, 1, 1, 2};
    t[PSB_OP_MUL]   = {1, 1, 1, 2};
    t[PSB_OP_DIV]   = {1, 1, 1, 2};
    t[PSB_OP_FMA]   = {1, 1, 1, 2};  // also reads old dst (register dst enforced)
    t[PSB_OP_NEG]   = {1, 1, 0, 1};

    t[PSB_OP_SIN]   = {1, 1, 0, 4};
    t[PSB_OP_COS]   = {1, 1, 0, 4};
    t[PSB_OP_TAN]   = {1, 1, 0, 4};
    t[PSB_OP_ASIN]  = {1, 1, 0, 4};
    t[PSB_OP_ACOS]  = {1, 1, 0, 4};
    t[PSB_OP_ATAN]  = {1, 1, 0, 4};
    t[PSB_OP_ATAN2] = {1, 1, 1, 4};
    t[PSB_OP_POW]   = {1, 1, 1, 8};
    t[PSB_OP_EXP]   = {1, 1, 0, 4};
    t[PSB_OP_LOG]   = {1, 1, 0, 4};
    t[PSB_OP_SQRT]  = {1, 1, 0, 4};
    t[PSB_OP_RSQRT] = {1, 1, 0, 4};
    t[PSB_OP_ABS]   = {1, 1, 0, 1};
    t[PSB_OP_SIGN]  = {1, 1, 0, 1};
    t[PSB_OP_FLOOR] = {1, 1, 0, 1};
    t[PSB_OP_CEIL]  = {1, 1, 0, 1};
    t[PSB_OP_FRACT] = {1, 1, 0, 1};
    t[PSB_OP_MOD]   = {1, 1, 1, 2};

    t[PSB_OP_MIN]   = {1, 1, 1, 1};
    t[PSB_OP_MAX]   = {1, 1, 1, 1};
    t[PSB_OP_CLAMP] = {1, 1, 1, 3};  // reads old dst as hi
    t[PSB_OP_MIX]   = {1, 1, 1, 3};  // reads old dst as t
    t[PSB_OP_STEP]  = {1, 1, 1, 1};
    t[PSB_OP_SSTEP] = {1, 1, 1, 3};  // reads old dst as x

    t[PSB_OP_DOT2]  = {1, 2, 2, 3};
    t[PSB_OP_DOT3]  = {1, 3, 3, 4};
    t[PSB_OP_LEN2]  = {1, 2, 0, 5};
    t[PSB_OP_LEN3]  = {1, 3, 0, 6};
    t[PSB_OP_NORM2] = {2, 2, 0, 6};
    t[PSB_OP_NORM3] = {3, 3, 0, 8};
    t[PSB_OP_CROSS] = {3, 3, 3, 6};
    t[PSB_OP_DIST2] = {1, 2, 2, 5};

    t[PSB_OP_TEX2D] = {4, 2, 0, 8};

    // LCONST/LUNI carry a RAW pool index in srcA (not an encoded operand);
    // validated against the declared pool counts in the decode loop below.
    t[PSB_OP_LCONST] = {1, 1, 0, 1};
    t[PSB_OP_LUNI]   = {1, 1, 0, 1};

    return t;
}

constexpr std::array<PsbOpRule, 256> kPsbOpRules = PsbBuildRuleTable();

/// Validate a READABLE SCALAR operand byte against the declared pools.
bool PsbValidScalarOperand(uint8_t op, uint8_t constCount) {
    if (op <= PSB_OP_REG_END)     return true;                     // r0–r31
    if (op <= PSB_OP_UNIFORM_END) return true;                     // u0–u15 (undeclared reads as 0.0)
    if (op <= PSB_OP_CONST_END)   return (op - PSB_OP_CONST_BASE) < constCount;
    if (op <= PSB_OP_LITERAL_END) return true;                     // inline literal
    return false;                                                  // 0x60–0xFE and 0xFF: invalid to READ
}

/// Validate one operand field against its rule (0 = must be UNUSED,
/// 1 = readable scalar, 2..4 = register vector base).
bool PsbValidOperand(uint8_t op, uint8_t vecWidth, uint8_t constCount) {
    if (vecWidth == 0) return op == PSB_OP_UNUSED;
    if (vecWidth == 1) return PsbValidScalarOperand(op, constCount);
    // Vector: must be a register whose consecutive range stays in r0–r31.
    return op <= PSB_OP_REG_END &&
           static_cast<uint16_t>(op) + vecWidth <= PSB_NUM_REGISTERS;
}

}  // namespace

PglRuntime::Result DecodeShaderProgram(const uint8_t* blob, size_t bytes,
                                       uint16_t programId,
                                       ShaderProgram& destination) {
    using R = PglRuntime::Result;

    // Fail-closed: any rejection leaves an INACTIVE destination slot.
    destination = ShaderProgram{};

    if (!blob) return R::BadPacket;
    if (programId >= GpuConfig::MAX_SHADER_PROGRAMS) return R::InvalidHandle;
    if (bytes < sizeof(PglShaderProgramHeader) || bytes > PSB_MAX_PROGRAM_SIZE)
        return R::BadPacket;

    PglShaderProgramHeader hdr;
    std::memcpy(&hdr, blob, sizeof(hdr));

    if (hdr.magic != PSB_MAGIC || hdr.version != PSB_VERSION) return R::Incompatible;
    if (hdr.reserved != 0) return R::InvalidValue;
    if (hdr.flags & ~PSB_FLAG_VALID_MASK) return R::InvalidValue;
    if (hdr.uniformCount > PSB_MAX_UNIFORMS ||
        hdr.constCount   > PSB_MAX_CONSTANTS ||
        hdr.instrCount   > PSB_MAX_INSTRUCTIONS ||
        hdr.instrCount   == 0)                          return R::InvalidValue;

    // Exact layout: header + uniform table + constants + instructions.
    const size_t uniformBytes = static_cast<size_t>(hdr.uniformCount) * sizeof(PglUniformDescriptor);
    const size_t constBytes   = static_cast<size_t>(hdr.constCount)   * sizeof(float);
    const size_t instrBytes   = static_cast<size_t>(hdr.instrCount)   * sizeof(uint32_t);
    if (bytes != sizeof(hdr) + uniformBytes + constBytes + instrBytes)
        return R::BadPacket;

    const uint8_t* pUniforms = blob + sizeof(hdr);
    const uint8_t* pConsts   = pUniforms + uniformBytes;
    const uint8_t* pInstrs   = pConsts + constBytes;

    ShaderProgram tmp;  // zero-initialised (default member initialisers)
    tmp.programId    = programId;
    tmp.uniformCount = hdr.uniformCount;
    tmp.constCount   = hdr.constCount;
    tmp.instrCount   = hdr.instrCount;
    tmp.flags        = hdr.flags;  // informational only — readsFramebuffer below is authoritative

    // ── Constants pool: bounded AND finite ──────────────────────────────
    for (uint8_t i = 0; i < hdr.constCount; ++i) {
        float c;
        std::memcpy(&c, pConsts + i * sizeof(float), sizeof(float));
        if (!std::isfinite(c)) return R::InvalidValue;
        tmp.constants[i] = c;
    }

    // ── Uniform descriptors: user slots only, typed, non-overlapping ────
    // Slots 0–2 are runtime auto-bound (resolution/time) and may not be
    // declared.  Declared defaults are applied to the uniform table now.
    uint32_t usedSlots = 0x7u;  // bits 0..2 = auto-bound
    for (uint8_t i = 0; i < hdr.uniformCount; ++i) {
        PglUniformDescriptor desc;
        std::memcpy(&desc, pUniforms + i * sizeof(desc), sizeof(desc));

        if (desc.type > PSB_UNIFORM_VEC4) return R::InvalidValue;
        const uint8_t comps = static_cast<uint8_t>(desc.type) + 1;
        if (desc.slot < PSB_USER_UNIFORM_START) return R::InvalidValue;
        if (static_cast<uint16_t>(desc.slot) + comps > PSB_MAX_UNIFORMS)
            return R::InvalidValue;
        const uint32_t mask = ((1u << comps) - 1u) << desc.slot;
        if (usedSlots & mask) return R::InvalidValue;  // overlapping descriptors
        usedSlots |= mask;

        tmp.uniformNameHashes[desc.slot] = desc.nameHash;
        tmp.uniformTypes[desc.slot]      = desc.type;

        if (desc.defaultValueOffset != PSB_UNIFORM_NO_DEFAULT) {
            const uint32_t off = desc.defaultValueOffset;
            if ((off & 3u) != 0 ||
                off + static_cast<uint32_t>(comps) * sizeof(float) > constBytes)
                return R::InvalidValue;
            std::memcpy(&tmp.uniforms[desc.slot], pConsts + off,
                        comps * sizeof(float));  // constants already finite-checked
        }
        // PSB_UNIFORM_NO_DEFAULT → components stay 0.0f (defined default)
    }

    // ── Instruction stream: opcodes, operand classes, vector ranges ─────
    uint32_t weightedCost = 0;
    bool readsFramebuffer = false;

    for (uint16_t pc = 0; pc < hdr.instrCount; ++pc) {
        uint32_t raw;
        std::memcpy(&raw, pInstrs + pc * sizeof(uint32_t), sizeof(uint32_t));

        const uint8_t opcode = static_cast<uint8_t>(raw & 0xFF);
        const uint8_t dst    = static_cast<uint8_t>((raw >> 8) & 0xFF);
        const uint8_t srcA   = static_cast<uint8_t>((raw >> 16) & 0xFF);
        const uint8_t srcB   = static_cast<uint8_t>((raw >> 24) & 0xFF);

        const PsbOpRule& rule = kPsbOpRules[opcode];
        if (rule.dstVec == 0 && rule.srcAVec == 0 && rule.srcBVec == 0 &&
            rule.weight == 0 && opcode != PSB_OP_NOP && opcode != PSB_OP_END)
            return R::InvalidValue;  // unknown opcode — rejected BEFORE execution

        // Destination: no-dst ops require UNUSED; otherwise a register whose
        // (possibly vector) consecutive range stays inside the register file.
        if (rule.dstVec == 0) {
            if (dst != PSB_OP_UNUSED) return R::InvalidValue;
        } else {
            if (dst > PSB_OP_REG_END ||
                static_cast<uint16_t>(dst) + rule.dstVec > PSB_NUM_REGISTERS)
                return R::InvalidValue;
        }

        if (!PsbValidOperand(srcA, rule.srcAVec, hdr.constCount))
            return R::InvalidValue;
        if (!PsbValidOperand(srcB, rule.srcBVec, hdr.constCount))
            return R::InvalidValue;

        // Raw pool-index operands (LCONST/LUNI): bounded by DECLARED counts.
        if (opcode == PSB_OP_LCONST && srcA >= hdr.constCount) return R::InvalidValue;
        if (opcode == PSB_OP_LUNI   && srcA >= PSB_MAX_UNIFORMS) return R::InvalidValue;

        // END semantics: exactly one END, as the LAST instruction — no
        // unreachable trailing code, no fall-off without END.
        if (opcode == PSB_OP_END && pc != hdr.instrCount - 1) return R::InvalidValue;

        if (opcode == PSB_OP_TEX2D) readsFramebuffer = true;
        weightedCost += rule.weight;
        tmp.instructions[pc] = raw;
    }

    if ((tmp.instructions[hdr.instrCount - 1] & 0xFF) != PSB_OP_END)
        return R::InvalidValue;  // last instruction must be END

    // Derived framebuffer-read requirement (never the host flag): a program
    // containing TEX2D must have asked for the snapshot — TEX-without-flag
    // is malformed and rejected at upload, not patched over at run time.
    if (readsFramebuffer && !(hdr.flags & PSB_FLAG_NEEDS_SCRATCH_COPY))
        return R::InvalidValue;

    // ── Commit: publish only a fully verified program ───────────────────
    tmp.active           = true;
    tmp.verified         = true;
    tmp.readsFramebuffer = readsFramebuffer;
    tmp.weightedCost     = weightedCost;

    destination = tmp;
    return R::Ok;
}

