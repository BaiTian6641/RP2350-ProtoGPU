/**
 * @file pgl_shader_vm.h
 * @brief PGL Shader Virtual Machine — bytecode interpreter for screen-space shaders.
 *
 * Executes verified PSB1 (PGL Shader Bytecode) programs per-pixel on the
 * RP2350.  The VM uses a 32-register float file with fixed assignments for
 * built-in variables (gl_FragCoord, pixel colour, gl_FragColor) and 20 user
 * temporaries.
 *
 * Execution trust model (P05-08):
 *   - Programs are decoded and fully verified ONCE at upload time by
 *     DecodeShaderProgram() below (blob/table/count bounds, opcodes, operand
 *     classes, writable and consecutive vector register ranges, flags,
 *     reserved fields, finite constants/defaults, TEX2D snapshot derivation).
 *   - Execute() runs ONLY programs whose ShaderProgram::verified flag was set
 *     by a successful decode; it performs NO per-pixel re-validation.
 *   - Every accepted instruction has defined behaviour for all finite inputs;
 *     non-finite intermediate or final results are contained (finite-sanitised
 *     at the output stage) so a shader can never poison the framebuffer.
 *
 * The PSB1 stage is strictly a finite screen-space RGB post-process: there is
 * no vertex/compute stage and the output alpha channel is unused.
 *
 * See docs/TinyGPU_Implementation_and_Agent_Handoff_Plan.md for lifetime policy.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <PglRuntimeProtocol.h>  // PglRuntime::Result

// Forward declaration — full definition in scene_state.h
struct ShaderProgram;

class PglShaderVM {
public:
    /**
     * @brief Execute a verified shader program for one pixel.
     *
     * Before calling, the VM auto-loads built-in registers:
     *   r0=fragX, r1=fragY, r2=0, r3=1, r4=inR, r5=inG, r6=inB, r7=1
     * After execution, output is read from r28–r31 (gl_FragColor) and
     * sanitised: a non-finite channel (NaN/±inf, e.g. from overflow) is
     * contained to 0.0 so the caller always receives finite RGB.
     *
     * @param prog   VERIFIED shader program (DecodeShaderProgram output).
     *               Passing an unverified program is a caller contract
     *               violation; the VM deliberately does not re-verify here.
     * @param uniforms Immutable PSB_MAX_UNIFORMS entries for this execution;
     *                 may be the resident bank or a pass-local auto-uniform bank.
     * @param fragX  Pixel X coordinate (0-based)
     * @param fragY  Pixel Y coordinate (0-based)
     * @param inR    Input red   (0.0–1.0, from current framebuffer pixel)
     * @param inG    Input green (0.0–1.0)
     * @param inB    Input blue  (0.0–1.0)
     * @param fb     Framebuffer pointer (RGB565, dense w×h snapshot, for
     *               texture2D sampling; never dereferenced when the verified
     *               program contains no TEX2D)
     * @param w      Framebuffer width in pixels (== snapshot stride)
     * @param h      Framebuffer height in pixels
     * @param outR   [out] Output red   (finite; caller clamps to 0.0–1.0)
     * @param outG   [out] Output green (finite)
     * @param outB   [out] Output blue  (finite)
     */
    void Execute(const ShaderProgram& prog, const float* uniforms,
                 float fragX, float fragY,
                 float inR, float inG, float inB,
                 const uint16_t* fb, uint16_t w, uint16_t h,
                 float& outR, float& outG, float& outB);

private:
    float regs_[32];  // Register file (128 bytes, one VM instance per band)
};

/**
 * @brief Decode and fully verify a PSB1 bytecode blob into a ShaderProgram.
 *
 * This is the ONLY accepted upload path (parser admission and commit).  The
 * blob layout must be exact:
 *   PglShaderProgramHeader | uniformCount × PglUniformDescriptor
 *   | constCount × float   | instrCount × uint32 (packed instructions)
 *
 * Verification rejects (consumer-visible upload failure, no partial state):
 *   - null/truncated/oversized blobs, size mismatch vs declared counts
 *     (BadPacket), bad magic/version (Incompatible);
 *   - counts above the profile limits, reserved/flag bits outside
 *     PSB_FLAG_VALID_MASK, zero instructions (InvalidValue);
 *   - uniform descriptors with out-of-range/auto slots (0–2 are runtime-bound),
 *     unknown types, overlapping slot ranges, or out-of-pool default offsets
 *     (InvalidValue); declared defaults ARE applied to the uniform table
 *     (PSB_UNIFORM_NO_DEFAULT → 0.0f);
 *   - non-finite constants (InvalidValue);
 *   - unknown opcodes, operands of an invalid class, readable operands in
 *     ignored positions (must be PSB_OP_UNUSED), non-register destinations,
 *     vector register bases whose consecutive range would exceed r31,
 *     LCONST/LUNI pool indices out of range (InvalidValue);
 *   - END missing from the last instruction or appearing earlier
 *     (InvalidValue);
 *   - TEX2D present without PSB_FLAG_NEEDS_SCRATCH_COPY (InvalidValue).
 *
 * On success `destination` is fully replaced (active, verified=true,
 * readsFramebuffer derived from the instruction stream — never from the host
 * flag — and weightedCost derived for budget admission).  On failure
 * `destination` is zeroed (inactive), so a failed upload can never leave a
 * half-loaded program behind.
 *
 * @param blob        PSB1 blob bytes (exactly `bytes` long)
 * @param bytes       Blob length (≤ PSB_MAX_PROGRAM_SIZE)
 * @param programId   Destination program slot (must be < the firmware profile
 *                    cap GpuConfig::MAX_SHADER_PROGRAMS)
 * @param destination [out] Program storage to fill
 * @return PglRuntime::Result::Ok on success, error code otherwise.
 */
PglRuntime::Result DecodeShaderProgram(const uint8_t* blob, size_t bytes,
                                       uint16_t programId,
                                       ShaderProgram& destination);
