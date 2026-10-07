// Screenspace post-FX engine — native functional check (P05-07/P05-08).
//
// Exercises ApplyShaderSlots against real framebuffers: intensity 0/1 and
// blend semantics, scissor + stride targeting, immutable snapshot neighbour
// reads (deterministic across the two band workers), slot-order composition,
// weighted-op budget exhaustion, and invalid slot containment.
//
// No mocks: runs the real effects engine and VM on native RGB565 buffers.
// Build/run via tests/shader_vm/run_shader_checks.sh.

#include "../src/render/screenspace_effects.h"
#include "../src/render/pgl_shader_vm.h"
#include "../src/scene_state.h"

#include "psb_blob_builder.h"

#include <PglShaderBackend.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <cstring>

namespace {

int g_failures = 0;
void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_failures;
}

using PglRuntime::Result;
namespace BE = PglShaderBackend;

constexpr uint16_t W = 16, H = 8;         // logical target
constexpr uint16_t STRIDE = 16;           // dense target
constexpr size_t   PIXELS = size_t(W) * H;

uint16_t g_fb[PIXELS];
uint16_t g_orig[PIXELS];
uint16_t g_scratch[PIXELS];

uint16_t Px(int r5, int g6, int b5) { return BE::PackRGB565i(
    static_cast<uint8_t>(r5), static_cast<uint8_t>(g6), static_cast<uint8_t>(b5)); }

void FillPattern() {
    for (uint16_t y = 0; y < H; ++y)
        for (uint16_t x = 0; x < W; ++x)
            g_fb[y * W + x] = Px((x * x + y) & 31, (x + y * y) & 63, (x + y) & 31);
    std::memcpy(g_orig, g_fb, sizeof(g_fb));
}

bool FbEqualsOrig() { return std::memcmp(g_fb, g_orig, sizeof(g_fb)) == 0; }

ShaderSlot InvertSlot(float intensity) {
    ShaderSlot s{};
    s.active = true;
    s.shaderClass = PGL_SHADER_COLOR_ADJUST;
    s.intensity = intensity;
    PglShaderParamsColorAdjust cp{};
    cp.operation = PGL_COLOR_INVERT;
    std::memcpy(s.params, &cp, sizeof(cp));
    return s;
}

ShaderSlot ConvolutionSlot(uint8_t radius, float intensity) {
    ShaderSlot s{};
    s.active = true;
    s.shaderClass = PGL_SHADER_CONVOLUTION;
    s.intensity = intensity;
    PglShaderParamsConvolution cp{};
    cp.kernelShape = PGL_KERNEL_BOX;
    cp.radius = radius;
    cp.separable = 0;
    cp.angle = 0.0f;        // horizontal
    cp.anglePeriod = 0.0f;
    cp.sigma = 0.0f;
    std::memcpy(s.params, &cp, sizeof(cp));
    return s;
}

SceneState& TheScene() {
    static SceneState scene;  // static: large object
    return scene;
}

void ResetOps() { TheScene().shaderFrameOps = 0; }

Result Apply(ShaderSlot* slots, size_t count,
             uint16_t* fb, uint16_t w, uint16_t h, uint16_t stride,
             uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
             uint16_t* scratch, size_t scratchPixels) {
    return ScreenspaceShaders::ApplyShaderSlots(&TheScene(), slots, count,
                                                fb, w, h, stride,
                                                x0, y0, x1, y1,
                                                scratch, scratchPixels,
                                                0.5f, nullptr);
}

int g_serviceCalls = 0;
void CountService() { ++g_serviceCalls; }

}  // namespace

int main() {
    // ── E1: intensity 0 is an exact bypass ──────────────────────────────
    {
        FillPattern(); ResetOps();
        ShaderSlot s = InvertSlot(0.0f);
        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        check(r == Result::Ok && FbEqualsOrig(),
              "fx: intensity 0 bypasses byte-exactly");
        check(TheScene().shaderFrameOps == 0, "fx: bypassed slot costs nothing");
    }

    // ── E2: intensity 1 inverts every pixel ─────────────────────────────
    {
        FillPattern(); ResetOps();
        ShaderSlot s = InvertSlot(1.0f);
        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        bool ok = (r == Result::Ok);
        for (size_t i = 0; i < PIXELS && ok; ++i) {
            const uint16_t o = g_orig[i];
            ok = g_fb[i] == Px(31 - BE::R5(o), 63 - BE::G6(o), 31 - BE::B5(o));
        }
        check(ok, "fx: intensity 1 invert applies to every pixel");
        check(TheScene().shaderFrameOps == 2u * PIXELS,
              "fx: invert charged 2 weighted ops per pixel");
    }

    // ── E3: intensity 0.5 blends with the source pixel ──────────────────
    {
        FillPattern(); ResetOps();
        ShaderSlot s = InvertSlot(0.5f);
        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        bool ok = (r == Result::Ok);
        for (size_t i = 0; i < PIXELS && ok; ++i) {
            const uint16_t o = g_orig[i];
            const int or5 = BE::R5(o), og6 = BE::G6(o), ob5 = BE::B5(o);
            const int er = or5 + static_cast<int>((31 - or5 - or5) * 0.5f);
            const int eg = og6 + static_cast<int>((63 - og6 - og6) * 0.5f);
            const int eb = ob5 + static_cast<int>((31 - ob5 - ob5) * 0.5f);
            ok = g_fb[i] == Px(er, eg, eb);
        }
        check(ok, "fx: intensity 0.5 mixes source and effect per pixel");
    }

    // ── E4: scissor confines writes ─────────────────────────────────────
    {
        FillPattern(); ResetOps();
        ShaderSlot s = InvertSlot(1.0f);
        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 4, 2, 12, 6, g_scratch, PIXELS);
        bool ok = (r == Result::Ok);
        for (uint16_t y = 0; y < H && ok; ++y)
            for (uint16_t x = 0; x < W && ok; ++x) {
                const uint16_t o = g_orig[y * W + x];
                const bool inside = (x >= 4 && x < 12 && y >= 2 && y < 6);
                const uint16_t want = inside
                    ? Px(31 - BE::R5(o), 63 - BE::G6(o), 31 - BE::B5(o)) : o;
                ok = g_fb[y * W + x] == want;
            }
        check(ok, "fx: scissor rect confines effect writes");
        check(TheScene().shaderFrameOps == 2u * 8u * 4u,
              "fx: cost charged on scissor pixels only");
    }

    // ── E5: strided target ──────────────────────────────────────────────
    {
        constexpr uint16_t SW = 8, SH = 4, SSTRIDE = 20;
        uint16_t sfb[SH * SSTRIDE];
        for (uint16_t i = 0; i < SH * SSTRIDE; ++i) sfb[i] = Px(i & 31, i & 63, (i * 3) & 31);
        uint16_t sorig[SH * SSTRIDE];
        std::memcpy(sorig, sfb, sizeof(sorig));

        ResetOps();
        ShaderSlot s = InvertSlot(1.0f);
        Result r = Apply(&s, 1, sfb, SW, SH, SSTRIDE, 0, 0, SW, SH,
                         g_scratch, PIXELS);
        bool ok = (r == Result::Ok);
        for (uint16_t y = 0; y < SH && ok; ++y)
            for (uint16_t x = 0; x < SSTRIDE && ok; ++x) {
                const uint16_t o = sorig[y * SSTRIDE + x];
                const uint16_t want = (x < SW)
                    ? Px(31 - BE::R5(o), 63 - BE::G6(o), 31 - BE::B5(o)) : o;
                ok = sfb[y * SSTRIDE + x] == want;
            }
        check(ok, "fx: strided target writes logical pixels, padding untouched");
    }

    // ── E6: convolution reads the immutable snapshot ────────────────────
    {
        FillPattern(); ResetOps();
        ShaderSlot s = ConvolutionSlot(2, 1.0f);  // horizontal box blur, radius 2
        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        bool ok = (r == Result::Ok);
        // Reference: average of ORIGINAL neighbours (in-range only), box weights.
        for (uint16_t y = 0; y < H && ok; ++y)
            for (uint16_t x = 0; x < W && ok; ++x) {
                int sr = 0, sg = 0, sb = 0, n = 0;
                for (int j = -2; j <= 2; ++j) {
                    const int xx = static_cast<int>(x) + j;
                    if (xx < 0 || xx >= W) continue;
                    const uint16_t o = g_orig[y * W + xx];
                    sr += BE::R5(o); sg += BE::G6(o); sb += BE::B5(o); ++n;
                }
                ok = g_fb[y * W + x] == Px(sr / n, sg / n, sb / n);
            }
        check(ok, "fx: convolution blur computed from pre-pass snapshot");
        check(TheScene().shaderFrameOps == (2u * 2 + 1u) * 2u * PIXELS,
              "fx: convolution charged (2r+1)*2 ops per pixel");
    }

    // ── E7: displacement samples the snapshot ───────────────────────────
    {
        FillPattern(); ResetOps();
        ShaderSlot s{};
        s.active = true;
        s.shaderClass = PGL_SHADER_DISPLACEMENT;
        s.intensity = 1.0f;
        PglShaderParamsDisplacement dp{};
        dp.axis = PGL_AXIS_X;
        dp.perChannel = 0;
        dp.amplitude = 2;
        dp.waveform = PGL_WAVE_SAWTOOTH;
        dp.period = 0.0f;
        dp.frequency = 10.0f;
        std::memcpy(s.params, &dp, sizeof(dp));

        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        bool ok = (r == Result::Ok);
        for (uint16_t y = 0; y < H && ok; ++y)
            for (uint16_t x = 0; x < W && ok; ++x) {
                // Mirror the engine's deterministic offset formula.
                const float s5 = sinf(static_cast<float>(y));  // coordY = y/(10/freq)
                int off = static_cast<int>(1.0f + (s5 + 1.0f) * 0.5f * (2.0f - 1.0f));
                off = off < 1 ? 1 : (off > 2 ? 2 : off);
                const int sx = static_cast<int>(x) + off;
                const uint16_t want = (sx < W) ? g_orig[y * W + sx] : 0;
                ok = g_fb[y * W + x] == want;
            }
        check(ok, "fx: displacement samples pre-pass snapshot at offset coords");
    }

    // ── E8: PSB invert program through the engine ───────────────────────
    {
        FillPattern(); ResetOps();
        uint8_t blob[PSB_MAX_PROGRAM_SIZE];
        const size_t n = PsbBuildInvert(blob);
        Result dr = DecodeShaderProgram(blob, n, 0, TheScene().shaderPrograms[0]);
        check(dr == Result::Ok, "fx: invert program uploaded via decode");

        ShaderSlot s{};
        s.active = true;
        s.shaderClass = PGL_SHADER_PROGRAM;
        s.intensity = 1.0f;
        s.programId = 0;

        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        bool ok = (r == Result::Ok);
        for (size_t i = 0; i < PIXELS && ok; ++i) {
            const uint16_t o = g_orig[i];
            const float ir = BE::R5(o) / 31.0f, ig = BE::G6(o) / 63.0f, ib = BE::B5(o) / 31.0f;
            const uint16_t want = BE::PackRGB565(1.0f - ir, 1.0f - ig, 1.0f - ib);
            ok = g_fb[i] == want;
        }
        check(ok, "fx: PSB invert program applied per pixel");
        check(TheScene().shaderFrameOps == 6u * PIXELS,
              "fx: PSB pass charged decoded weighted cost (3 SUBs) per pixel");
    }

    // ── E9: TEX2D program samples the snapshot ──────────────────────────
    {
        FillPattern(); ResetOps();
        uint8_t blob[PSB_MAX_PROGRAM_SIZE];
        const size_t n = PsbBuildTexShift(blob);
        Result dr = DecodeShaderProgram(blob, n, 0, TheScene().shaderPrograms[0]);
        check(dr == Result::Ok && TheScene().shaderPrograms[0].readsFramebuffer,
              "fx: TEX2D shift program uploaded with derived snapshot need");
        ShaderSlot s{};
        s.active = true;
        s.shaderClass = PGL_SHADER_PROGRAM;
        s.intensity = 1.0f;
        s.programId = 0;

        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        bool ok = (r == Result::Ok);
        for (uint16_t y = 0; y < H && ok; ++y)
            for (uint16_t x = 0; x < W && ok; ++x) {
                const uint16_t sx = (x + 1 < W) ? static_cast<uint16_t>(x + 1)
                                                : static_cast<uint16_t>(W - 1);
                ok = g_fb[y * W + x] == g_orig[y * W + sx];
            }
        check(ok, "fx: TEX2D reads immutable snapshot (x+1 texel, clamped at edge)");
    }

    // ── E10: auto-bound uniforms (u_resolution, u_time) ─────────────────
    {
        FillPattern(); ResetOps();
        PsbBlobBuilder b;
        b.Header(0, 0, 0, 4);
        b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, PSB_OP_UNIFORM_BASE + 2, PSB_OP_UNUSED); // u_time
        b.Instr(PSB_OP_MOV, PSB_REG_OUT_G, PSB_OP_LITERAL_BASE + 1, PSB_OP_UNUSED); // 0.5
        b.Instr(PSB_OP_MOV, PSB_REG_OUT_B, PSB_OP_LITERAL_BASE + 0, PSB_OP_UNUSED); // 0.0
        b.End();
        Result dr = DecodeShaderProgram(b.data, b.size, 0, TheScene().shaderPrograms[0]);
        check(dr == Result::Ok, "fx: u_time program uploaded");

        ShaderSlot s{};
        s.active = true;
        s.shaderClass = PGL_SHADER_PROGRAM;
        s.intensity = 1.0f;
        s.programId = 0;

        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        // elapsedSeconds = 0.5 (Apply helper): outR=0.5 → r5 = int(0.5*31+0.5) = 16
        const uint16_t want = Px(16, 32, 0);
        bool ok = (r == Result::Ok);
        for (size_t i = 0; i < PIXELS && ok; ++i) ok = g_fb[i] == want;
        check(ok, "fx: auto-bound u_time uniform reaches the VM (elapsed seconds)");
    }

    // ── E11: weighted-op budget exhaustion is contained ─────────────────
    {
        FillPattern();
        ShaderSlot s = ConvolutionSlot(GpuConfig::MAX_CONVOLUTION_RADIUS, 1.0f);
        const uint32_t cost = ScreenspaceShaders::EstimateSlotWeightedOps(
            &TheScene(), s, PIXELS);
        check(cost == (2u * GpuConfig::MAX_CONVOLUTION_RADIUS + 1u) * 2u * PIXELS,
              "fx: estimator reports max-radius convolution cost");
        TheScene().shaderFrameOps = GpuConfig::POSTFX_WORK_BUDGET - cost + 1;
        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        check(r == Result::Capacity && FbEqualsOrig(),
              "fx: budget exhaustion returns Capacity without touching the target");
        check(TheScene().shaderFrameOps == GpuConfig::POSTFX_WORK_BUDGET - cost + 1,
              "fx: exhausted pass does not consume budget");
    }

    // ── E12/E13: invalid slots rejected and contained ───────────────────
    {
        FillPattern(); ResetOps();
        ShaderSlot s{};
        s.active = true;
        s.shaderClass = 0x7F;  // unknown class
        s.intensity = 1.0f;
        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        check(r == Result::InvalidValue && FbEqualsOrig(),
              "fx: unknown shader class rejected, target untouched");
        check(ScreenspaceShaders::EstimateSlotWeightedOps(&TheScene(), s, PIXELS) == UINT32_MAX,
              "fx: estimator rejects unknown class");

        TheScene().shaderPrograms[1] = ShaderProgram{};  // inactive/unverified
        ShaderSlot p{};
        p.active = true;
        p.shaderClass = PGL_SHADER_PROGRAM;
        p.intensity = 1.0f;
        p.programId = 1;
        r = Apply(&p, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        check(r == Result::InvalidValue && FbEqualsOrig(),
              "fx: unverified program slot rejected, target untouched");
        check(ScreenspaceShaders::EstimateSlotWeightedOps(&TheScene(), p, PIXELS) == UINT32_MAX,
              "fx: estimator rejects unverified program");

        ShaderSlot oob = p;
        oob.programId = GpuConfig::MAX_SHADER_PROGRAMS;
        check(ScreenspaceShaders::EstimateSlotWeightedOps(&TheScene(), oob, PIXELS) == UINT32_MAX,
              "fx: estimator rejects out-of-profile programId");
    }

    // ── E14: snapshot workspace bounds ──────────────────────────────────
    {
        FillPattern(); ResetOps();
        ShaderSlot s = ConvolutionSlot(2, 1.0f);
        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H,
                         g_scratch, 10);  // far too small for 16×8
        check(r == Result::Capacity && FbEqualsOrig(),
              "fx: undersized scratch rejects snapshot pass, target untouched");
        r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, nullptr, 0);
        check(r == Result::Capacity && FbEqualsOrig(),
              "fx: null scratch rejects snapshot pass, target untouched");
    }

    // ── E15: argument validation ────────────────────────────────────────
    {
        FillPattern(); ResetOps();
        ShaderSlot s = InvertSlot(1.0f);
        check(Apply(&s, 1, g_fb, W, H, W - 1, 0, 0, W, H, g_scratch, PIXELS)
                  == Result::InvalidValue, "fx: stride < width rejected");
        check(Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W + 1, H, g_scratch, PIXELS)
                  == Result::InvalidValue, "fx: scissor beyond target rejected");
        check(Apply(&s, 1, nullptr, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS)
                  == Result::InvalidValue, "fx: null framebuffer rejected");
        check(Apply(&s, 1, g_fb, W, H, STRIDE, 3, 0, 3, H, g_scratch, PIXELS)
                  == Result::Ok && FbEqualsOrig(), "fx: empty scissor is a no-op");
        check(ScreenspaceShaders::ApplyShaderSlots(&TheScene(), &s, 1,
                  g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS,
                  std::numeric_limits<float>::quiet_NaN(), nullptr) == Result::InvalidValue,
              "fx: non-finite elapsed time rejected");
    }

    // ── E16: slot-order composition (invert twice = identity) ───────────
    {
        FillPattern(); ResetOps();
        ShaderSlot slots[2] = { InvertSlot(1.0f), InvertSlot(1.0f) };
        Result r = Apply(slots, 2, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        check(r == Result::Ok && FbEqualsOrig(),
              "fx: two full-strength invert slots compose to identity");
    }

    // ── E17: intensity edge cases ───────────────────────────────────────
    {
        FillPattern(); ResetOps();
        ShaderSlot s = InvertSlot(std::numeric_limits<float>::quiet_NaN());
        Result r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        check(r == Result::InvalidValue && FbEqualsOrig(),
              "fx: NaN intensity rejected, target untouched");

        s = InvertSlot(2.0f);  // clamps to full effect
        r = Apply(&s, 1, g_fb, W, H, STRIDE, 0, 0, W, H, g_scratch, PIXELS);
        bool ok = (r == Result::Ok);
        for (size_t i = 0; i < PIXELS && ok; ++i) {
            const uint16_t o = g_orig[i];
            ok = g_fb[i] == Px(31 - BE::R5(o), 63 - BE::G6(o), 31 - BE::B5(o));
        }
        check(ok, "fx: intensity > 1 clamps to full effect");
    }

    // ── E18: estimator edge values ──────────────────────────────────────
    {
        ShaderSlot s = ConvolutionSlot(GpuConfig::MAX_CONVOLUTION_RADIUS + 1, 1.0f);
        check(ScreenspaceShaders::EstimateSlotWeightedOps(&TheScene(), s, PIXELS) == UINT32_MAX,
              "fx: estimator rejects radius above profile cap");
        s = InvertSlot(0.0f);
        check(ScreenspaceShaders::EstimateSlotWeightedOps(&TheScene(), s, PIXELS) == 0,
              "fx: estimator reports zero for bypassed slot");
        s.active = false;
        check(ScreenspaceShaders::EstimateSlotWeightedOps(&TheScene(), s, PIXELS) == 0,
              "fx: estimator reports zero for inactive slot");
    }

    // ── E19: service callback fires from bounded slices (core-0 band) ───
    {
        FillPattern(); ResetOps();
        g_serviceCalls = 0;
        ShaderSlot s = ConvolutionSlot(1, 1.0f);
        Result r = ScreenspaceShaders::ApplyShaderSlots(&TheScene(), &s, 1,
                                                        g_fb, W, H, STRIDE,
                                                        0, 0, W, H,
                                                        g_scratch, PIXELS,
                                                        0.5f, CountService);
        // Native PairDispatch runs serially; the core-0 band covers rows
        // [0, H/2) = 4 rows → exactly one bounded service slice.
        check(r == Result::Ok && g_serviceCalls == 1,
              "fx: service callback fires once per 4-row slice on the core-0 band");
        check(TheScene().shaderFrameOps == (2u * 1 + 1u) * 2u * PIXELS,
              "fx: radius-1 convolution charged (2r+1)*2 ops per pixel");
    }

    std::printf("\n%s (%d failures)\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
}
