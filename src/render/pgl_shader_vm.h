/**
 * @file pgl_shader_vm.h
 * @brief PGL Shader Virtual Machine — bytecode interpreter for screen-space shaders.
 *
 * Executes compiled PSB (PGL Shader Bytecode) programs per-pixel on the RP2350.
 * The VM uses a 32-register float file with fixed assignments for built-in
 * variables (gl_FragCoord, pixel colour, gl_FragColor) and 20 user temporaries.
 *
 * Performance target: < 0.05 ms for a 40-instruction shader on 128×64 @ 150 MHz.
 *
 * See docs/Shader_System_Design.md §6 for architecture details.
 */

#pragma once

#include <cstdint>

// Forward declaration — full definition in scene_state.h
struct ShaderProgram;

class PglShaderVM {
public:
    /**
     * @brief Execute a shader program for one pixel.
     *
     * Before calling, the VM auto-loads built-in registers:
     *   r0=fragX, r1=fragY, r2=0, r3=1, r4=inR, r5=inG, r6=inB, r7=1
     * After execution, output is read from r28–r31 (gl_FragColor).
     *
     * @param prog   Compiled shader program (instructions + constants + uniforms)
     * @param fragX  Pixel X coordinate (0-based)
     * @param fragY  Pixel Y coordinate (0-based)
     * @param inR    Input red   (0.0–1.0, from current framebuffer pixel)
     * @param inG    Input green (0.0–1.0)
     * @param inB    Input blue  (0.0–1.0)
     * @param fb     Framebuffer pointer (RGB565, for texture2D sampling)
     * @param w      Framebuffer width in pixels
     * @param h      Framebuffer height in pixels
     * @param outR   [out] Output red   (0.0–1.0)
     * @param outG   [out] Output green (0.0–1.0)
     * @param outB   [out] Output blue  (0.0–1.0)
     */
    void Execute(const ShaderProgram& prog,
                 float fragX, float fragY,
                 float inR, float inG, float inB,
                 const uint16_t* fb, uint16_t w, uint16_t h,
                 float& outR, float& outG, float& outB);

private:
    float regs_[32];  // Register file (128 bytes, stack-allocated per pixel)
};

