/**
 * @file screenspace_effects.h
 * @brief GPU-side screen-space post-processing shader engine (P05-07/P05-08).
 *
 * One unified, target-scoped engine applies shader slots to a resolved render
 * target (back buffer OR layer framebuffer) after rasterization and before
 * compositing.  The caller resolves the target — framebuffer base, logical
 * dimensions, stride, and scissor rect — so camera effects bind to their
 * camera's actual target/scissor and layer effects to that layer's buffer.
 *
 * Shader classes (PglShaderClass):
 *   0x01 CONVOLUTION   — configurable 1D/2D blur kernel (reads immutable
 *                         snapshot; radius bounded by
 *                         GpuConfig::MAX_CONVOLUTION_RADIUS)
 *   0x02 DISPLACEMENT  — coordinate warp with optional chromatic split
 *                         (reads immutable snapshot)
 *   0x03 COLOR_ADJUST  — per-pixel colour transform; edge feather/edge
 *                         detect read the immutable snapshot
 *   0x04 PROGRAM       — verified PSB1 bytecode on PglShaderVM; TEX2D samples
 *                         the immutable snapshot (requirement DERIVED at
 *                         decode, never trusted from host flags)
 *
 * Correctness contract:
 *   - Slot order is compositional: a neighbour-reading slot snapshots the
 *     target AFTER the previous slot committed, so later slots see earlier
 *     results.
 *   - intensity semantics per PglCmdSetShader: 0.0 = exact byte-for-byte
 *     bypass (slot skipped, zero cost), 1.0 = full effect, in-between =
 *     per-pixel mix with the source pixel.  Non-finite intensity rejects.
 *   - Both band workers write DISJOINT row ranges and read only immutable
 *     sources (snapshot or own-row pixel), so serial execution (sim) is
 *     byte-identical to dual-core execution (firmware).
 *   - Only the core-0 band ever invokes the service callback, and only at
 *     bounded row slices (every kServiceSliceRows rows) plus as the
 *     PairDispatch idle function.
 *   - Weighted-op accounting: every accepted slot's cost is estimated and
 *     accumulated into SceneState::shaderFrameOps (reset by the caller each
 *     frame); exceeding GpuConfig::POSTFX_WORK_BUDGET stops further passes
 *     deterministically with Result::Capacity.  Use EstimateSlotWeightedOps
 *     for admission-time rejection.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <PglRuntimeProtocol.h>  // PglRuntime::Result

struct SceneState;
struct ShaderSlot;

namespace ScreenspaceShaders {

/**
 * @brief Estimate the weighted ops one slot consumes over `pixelCount` pixels.
 *
 * Returns 0 for inactive/disabled/bypassed (intensity <= 0) slots.
 * Returns UINT32_MAX for structurally invalid slots: unknown shader class or
 * colour operation, convolution radius above GpuConfig::MAX_CONVOLUTION_RADIUS,
 * non-finite intensity, or a PROGRAM slot whose programId is out of the
 * firmware profile cap or whose program is missing/unverified.  Admission
 * must treat UINT32_MAX as "reject the upload".
 *
 * The frame budget is GpuConfig::POSTFX_WORK_BUDGET weighted ops across ALL
 * camera + layer passes of one frame.
 */
uint32_t EstimateSlotWeightedOps(const SceneState* scene,
                                 const ShaderSlot& slot,
                                 uint32_t pixelCount);

/**
 * @brief Apply active shader slots, in order, to a resolved target region.
 *
 * @param scene        Scene state (programs, per-frame op accumulator)
 * @param slots        Shader slot array (camera or layer slots)
 * @param count        Number of entries in `slots`
 * @param framebuffer  Target base pointer; logical pixel (x,y) lives at
 *                     framebuffer[y*stride + x]
 * @param width,height Logical target dimensions in pixels
 * @param stride       Target row stride in pixels (>= width)
 * @param x0,y0,x1,y1  Scissor rect (exclusive x1/y1) inside the target;
 *                     only these pixels are written.  Neighbour/texture reads
 *                     may span the whole width×height image.
 * @param scratch      Snapshot workspace; must cover width*height pixels when
 *                     any applied slot reads neighbours/textures (may alias
 *                     retired depth storage).  May be null otherwise.
 * @param scratchPixels Capacity of `scratch` in pixels
 * @param elapsedSeconds Finite wall time in seconds (animated shaders)
 * @param service      Runtime service callback; invoked from core 0 only, at
 *                     bounded row slices (may be null)
 * @return PglRuntime::Result::Ok when every valid slot applied; the first
 *         encountered error otherwise (InvalidValue for malformed slots /
 *         arguments, Capacity for budget exhaustion or undersized scratch).
 *         Invalid slots are skipped; budget exhaustion stops processing.
 */
PglRuntime::Result ApplyShaderSlots(SceneState* scene,
                                    const ShaderSlot* slots, size_t count,
                                    uint16_t* framebuffer,
                                    uint16_t width, uint16_t height,
                                    uint16_t stride,
                                    uint16_t x0, uint16_t y0,
                                    uint16_t x1, uint16_t y1,
                                    uint16_t* scratch, size_t scratchPixels,
                                    float elapsedSeconds,
                                    void (*service)());

}  // namespace ScreenspaceShaders
