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
 *   - Flat switch dispatch (compiler generates jump table)
 *   - No heap allocation — register file is on the stack
 *   - Operand resolution inlined via PsbResolveOperand()
 *
 * Performance: ~8 cycles per instruction → 40-instruction shader ≈ 320 cycles/pixel
 *              8192 pixels × 320 = 2.6M cycles ≈ 0.017 ms @ 150 MHz.
 */

#include "pgl_shader_vm.h"
#include "../scene_state.h"

#include <PglShaderBytecode.h>
#include <PglShaderBackend.h>

#include <cstring>

// Namespace alias for brevity in the interpreter loop
namespace BE = PglShaderBackend;

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

    // Alias pointers for operand resolution
    const float* uniforms  = prog.uniforms;
    const float* constants = prog.constants;

    // ── Main interpreter loop ───────────────────────────────────────────
    const uint16_t instrCount = prog.instrCount;

    for (uint16_t pc = 0; pc < instrCount; ++pc) {
        const uint32_t raw = prog.instructions[pc];

        // Decode 4-byte instruction: [opcode][dst][srcA][srcB]
        const uint8_t opcode = static_cast<uint8_t>(raw & 0xFF);
        const uint8_t dstOp  = static_cast<uint8_t>((raw >> 8) & 0xFF);
        const uint8_t srcAOp = static_cast<uint8_t>((raw >> 16) & 0xFF);
        const uint8_t srcBOp = static_cast<uint8_t>((raw >> 24) & 0xFF);

        // Resolve source operands (only when will be needed — computed eagerly
        // for simplicity; the compiler should optimise unused ones away).
        const float a = PsbResolveOperand(srcAOp, regs_, uniforms, constants);
        const float b = PsbResolveOperand(srcBOp, regs_, uniforms, constants);

        // Destination must be a register (0x00–0x1F)
        // If dstOp > 0x1F we still compute but discard (no write).

        float result = 0.0f;

        switch (opcode) {
            // ── Special ─────────────────────────────────────────────────
            case PSB_OP_NOP:
                continue;

            case PSB_OP_END:
                goto done;

            // ── Arithmetic ──────────────────────────────────────────────
            case PSB_OP_MOV:   result = a;                          break;
            case PSB_OP_ADD:   result = BE::Add(a, b);              break;
            case PSB_OP_SUB:   result = BE::Sub(a, b);              break;
            case PSB_OP_MUL:   result = BE::Mul(a, b);              break;
            case PSB_OP_DIV:   result = BE::Div(a, b);              break;
            case PSB_OP_FMA:   result = BE::Fma(a, b, regs_[dstOp & 0x1F]); break;
            case PSB_OP_NEG:   result = BE::Neg(a);                 break;

            // ── Math Functions ──────────────────────────────────────────
            case PSB_OP_SIN:   result = BE::Sin(a);                 break;
            case PSB_OP_COS:   result = BE::Cos(a);                 break;
            case PSB_OP_TAN:   result = BE::Tan(a);                 break;
            case PSB_OP_ASIN:  result = BE::Asin(a);                break;
            case PSB_OP_ACOS:  result = BE::Acos(a);                break;
            case PSB_OP_ATAN:  result = BE::Atan(a);                break;
            case PSB_OP_ATAN2: result = BE::Atan2(a, b);            break;
            case PSB_OP_POW:   result = BE::Pow(a, b);              break;
            case PSB_OP_EXP:   result = BE::Exp(a);                 break;
            case PSB_OP_LOG:   result = BE::Log(a);                 break;
            case PSB_OP_SQRT:  result = BE::Sqrt(a);                break;
            case PSB_OP_RSQRT: result = BE::Rsqrt(a);               break;
            case PSB_OP_ABS:   result = BE::Abs(a);                 break;
            case PSB_OP_SIGN:  result = BE::Sign(a);                break;
            case PSB_OP_FLOOR: result = BE::Floor(a);               break;
            case PSB_OP_CEIL:  result = BE::Ceil(a);                break;
            case PSB_OP_FRACT: result = BE::Fract(a);               break;
            case PSB_OP_MOD:   result = BE::Mod(a, b);              break;

            // ── Clamping / Interpolation ────────────────────────────────
            case PSB_OP_MIN:   result = BE::Min(a, b);              break;
            case PSB_OP_MAX:   result = BE::Max(a, b);              break;
            case PSB_OP_CLAMP: {
                // 3-operand: clamp(srcA, lo=srcB, hi=dst)
                float hi = regs_[dstOp & 0x1F];
                result = BE::Clamp(a, b, hi);
                break;
            }
            case PSB_OP_MIX: {
                // 3-operand: mix(srcA, srcB, t=dst)
                float t = regs_[dstOp & 0x1F];
                result = BE::Mix(a, b, t);
                break;
            }
            case PSB_OP_STEP:  result = BE::Step(a, b);             break;
            case PSB_OP_SSTEP: {
                // 3-operand: smoothstep(edge0=srcA, edge1=srcB, x=dst)
                float x = regs_[dstOp & 0x1F];
                result = BE::Smoothstep(a, b, x);
                break;
            }

            // ── Geometric (vector operations on consecutive registers) ──
            case PSB_OP_DOT2: {
                uint8_t ai = srcAOp & 0x1F;
                uint8_t bi = srcBOp & 0x1F;
                result = BE::Dot2(regs_[ai], regs_[ai+1],
                                  regs_[bi], regs_[bi+1]);
                break;
            }
            case PSB_OP_DOT3: {
                uint8_t ai = srcAOp & 0x1F;
                uint8_t bi = srcBOp & 0x1F;
                result = BE::Dot3(regs_[ai], regs_[ai+1], regs_[ai+2],
                                  regs_[bi], regs_[bi+1], regs_[bi+2]);
                break;
            }
            case PSB_OP_LEN2: {
                uint8_t ai = srcAOp & 0x1F;
                result = BE::Len2(regs_[ai], regs_[ai+1]);
                break;
            }
            case PSB_OP_LEN3: {
                uint8_t ai = srcAOp & 0x1F;
                result = BE::Len3(regs_[ai], regs_[ai+1], regs_[ai+2]);
                break;
            }
            case PSB_OP_NORM2: {
                uint8_t ai = srcAOp & 0x1F;
                uint8_t di = dstOp & 0x1F;
                BE::Norm2(regs_[ai], regs_[ai+1],
                          regs_[di], regs_[di+1]);
                continue;  // already wrote to regs
            }
            case PSB_OP_NORM3: {
                uint8_t ai = srcAOp & 0x1F;
                uint8_t di = dstOp & 0x1F;
                BE::Norm3(regs_[ai], regs_[ai+1], regs_[ai+2],
                          regs_[di], regs_[di+1], regs_[di+2]);
                continue;
            }
            case PSB_OP_CROSS: {
                uint8_t ai = srcAOp & 0x1F;
                uint8_t bi = srcBOp & 0x1F;
                uint8_t di = dstOp & 0x1F;
                BE::Cross(regs_[ai], regs_[ai+1], regs_[ai+2],
                          regs_[bi], regs_[bi+1], regs_[bi+2],
                          regs_[di], regs_[di+1], regs_[di+2]);
                continue;
            }
            case PSB_OP_DIST2: {
                uint8_t ai = srcAOp & 0x1F;
                uint8_t bi = srcBOp & 0x1F;
                result = BE::Dist2(regs_[ai], regs_[ai+1],
                                   regs_[bi], regs_[bi+1]);
                break;
            }

            // ── Texture Sampling ────────────────────────────────────────
            case PSB_OP_TEX2D: {
                // Sample framebuffer at (srcA, srcA+1) as UV coords [0,1]
                // Write RGBA to dst..dst+3
                uint8_t ai = srcAOp & 0x1F;
                uint8_t di = dstOp & 0x1F;
                float texR, texG, texB;
                BE::TexSample(fb, w, h, regs_[ai], regs_[ai + 1],
                              texR, texG, texB);
                regs_[di]     = texR;
                regs_[di + 1] = texG;
                regs_[di + 2] = texB;
                regs_[di + 3] = 1.0f;
                continue;
            }

            // ── Load ────────────────────────────────────────────────────
            case PSB_OP_LCONST: {
                uint8_t idx = srcAOp;
                result = (idx < PSB_MAX_CONSTANTS) ? constants[idx] : 0.0f;
                break;
            }
            case PSB_OP_LUNI: {
                uint8_t idx = srcAOp;
                result = (idx < PSB_MAX_UNIFORMS) ? uniforms[idx] : 0.0f;
                break;
            }

            default:
                // Unknown opcode — skip
                continue;
        }

        // Write result to destination register
        if (dstOp <= PSB_OP_REG_END) {
            regs_[dstOp] = result;
        }
    }

done:
    // ── Read output from gl_FragColor registers ─────────────────────────
    outR = regs_[PSB_REG_OUT_R];
    outG = regs_[PSB_REG_OUT_G];
    outB = regs_[PSB_REG_OUT_B];
}

