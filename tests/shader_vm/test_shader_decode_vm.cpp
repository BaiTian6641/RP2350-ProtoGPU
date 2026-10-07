// PSB1 blob decode/verification + VM execution semantics — native functional
// check (P05-08).  Exercises DecodeShaderProgram's rejection of malformed
// blobs (consumer-visible upload failures) and the VM's defined arithmetic /
// uniform-default / END / nonfinite-containment behaviour.
//
// No mocks: blobs are decoded into a real ShaderProgram and executed by the
// real PglShaderVM.  Build/run via tests/shader_vm/run_shader_checks.sh.

#include "../src/render/pgl_shader_vm.h"
#include "../src/scene_state.h"   // ShaderProgram, GpuConfig caps

#include "psb_blob_builder.h"

#include <PglShaderCompiler.h>    // host PGLSL compiler (real upload path)

#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

int g_failures = 0;
void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_failures;
}

using PglRuntime::Result;

Result Decode(const PsbBlobBuilder& b, ShaderProgram& prog, uint16_t id = 0) {
    return DecodeShaderProgram(b.data, b.size, id, prog);
}

// 4-instruction passthrough with tweakable header/instruction bytes.
PsbBlobBuilder Passthrough(uint8_t flags = 0, uint16_t reserved = 0,
                           uint32_t magic = PSB_MAGIC, uint8_t version = PSB_VERSION) {
    PsbBlobBuilder b;
    b.Header(flags, 0, 0, 4, magic, version, reserved);
    b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, PSB_REG_IN_R, PSB_OP_UNUSED);
    b.Instr(PSB_OP_MOV, PSB_REG_OUT_G, PSB_REG_IN_G, PSB_OP_UNUSED);
    b.Instr(PSB_OP_MOV, PSB_REG_OUT_B, PSB_REG_IN_B, PSB_OP_UNUSED);
    b.End();
    return b;
}

}  // namespace

int main() {
    // ── D1: valid passthrough blob ──────────────────────────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b = Passthrough();
        check(Decode(b, prog) == Result::Ok, "decode: valid passthrough accepted");
    }

    // ── D2/D3: bad magic / version ──────────────────────────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b = Passthrough(0, 0, 0xDEADBEEF);
        check(Decode(b, prog) == Result::Incompatible && !prog.active,
              "decode: bad magic rejected (Incompatible), destination stays inactive");
        prog = ShaderProgram{};
        b = Passthrough(0, 0, PSB_MAGIC, 99);
        check(Decode(b, prog) == Result::Incompatible, "decode: bad version rejected");
    }

    // ── D4/D5: reserved field / unknown flag bits ───────────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b = Passthrough(0, 1);
        check(Decode(b, prog) == Result::InvalidValue, "decode: nonzero reserved rejected");
        prog = ShaderProgram{};
        b = Passthrough(0x02);
        check(Decode(b, prog) == Result::InvalidValue, "decode: unknown flag bit rejected");
    }

    // ── D6: blob size mismatches ────────────────────────────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b = Passthrough();
        check(DecodeShaderProgram(b.data, b.size + 1, 0, prog) == Result::BadPacket,
              "decode: trailing byte rejected");
        check(DecodeShaderProgram(b.data, b.size - 1, 0, prog) == Result::BadPacket,
              "decode: truncated blob rejected");
        check(DecodeShaderProgram(b.data, 8, 0, prog) == Result::BadPacket,
              "decode: sub-header blob rejected");
        check(DecodeShaderProgram(nullptr, b.size, 0, prog) == Result::BadPacket,
              "decode: null blob rejected");
        // Declared counts disagree with blob length.
        PsbBlobBuilder c;
        c.Header(0, 1, 0, 1);   // claims 1 constant + 1 instruction
        c.End();                // but only the instruction is present
        check(Decode(c, prog) == Result::BadPacket,
              "decode: count/size mismatch rejected");
    }

    // ── D7/D8: count bounds ─────────────────────────────────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b;
        b.Header(0, 0, 0, 0);   // zero instructions
        check(Decode(b, prog) == Result::InvalidValue, "decode: zero instructions rejected");

        PsbBlobBuilder c;
        c.Header(0, PSB_MAX_CONSTANTS + 1, 0, 1);
        for (int i = 0; i < PSB_MAX_CONSTANTS + 1; ++i) c.Const(0.0f);
        c.End();
        check(Decode(c, prog) == Result::InvalidValue,
              "decode: constCount above pool rejected");
    }

    // ── D9–D12: opcode / operand class / writable dst ───────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b;
        b.Header(0, 0, 0, 2);
        b.Instr(0x99, 8, PSB_OP_LITERAL_BASE, PSB_OP_UNUSED);  // unknown opcode
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: unknown opcode rejected before execution");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 0, 0, 2);
        b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, PSB_REG_IN_R, 0x00);  // srcB must be UNUSED
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: ignored operand position must be PSB_OP_UNUSED");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 0, 0, 2);
        b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, 0x60, PSB_OP_UNUSED);  // 0x60: no operand class
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: operand outside all classes rejected");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 0, 0, 2);
        b.Instr(PSB_OP_MOV, PSB_OP_LITERAL_BASE, PSB_REG_IN_R, PSB_OP_UNUSED);  // dst literal
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: non-register destination rejected");
    }

    // ── D13/D14: consecutive vector register ranges ─────────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b;
        b.Header(0, 0, 0, 2);
        b.Instr(PSB_OP_LEN3, 8, 30, PSB_OP_UNUSED);  // r30..r32 overruns r31
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: vector source base r30 len3 rejected");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 0, 0, 2);
        b.Instr(PSB_OP_LEN3, 8, 29, PSB_OP_UNUSED);  // r29..r31 exactly fits
        b.End();
        check(Decode(b, prog) == Result::Ok,
              "decode: vector source base r29 len3 accepted");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 0, 0, 2);
        b.Instr(PSB_OP_DOT2, 8, PSB_OP_UNIFORM_BASE + 1, 12);  // vector src must be REGISTER
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: uniform operand in vector source rejected");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(PSB_FLAG_NEEDS_SCRATCH_COPY, 0, 0, 2);
        b.Instr(PSB_OP_TEX2D, 29, 8, PSB_OP_UNUSED);  // r29..r32 overruns
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: TEX2D destination r29 (needs r29..r32) rejected");
    }

    // ── D15: TEX2D requires the snapshot flag ───────────────────────────
    {
        ShaderProgram prog;
        uint8_t blob[PSB_MAX_PROGRAM_SIZE];
        const size_t n = PsbBuildTexShift(blob);
        check(DecodeShaderProgram(blob, n, 0, prog) == Result::Ok &&
              prog.readsFramebuffer,
              "decode: TEX2D program accepted with derived framebuffer read");

        // Same program with the host flag stripped → must be rejected.
        blob[5] = 0;  // header.flags offset: magic(4) version(1) flags(1)
        prog = ShaderProgram{};
        check(DecodeShaderProgram(blob, n, 0, prog) == Result::InvalidValue,
              "decode: TEX2D without PSB_FLAG_NEEDS_SCRATCH_COPY rejected");
    }

    // ── D16: LCONST/LUNI pool bounds ────────────────────────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b;
        b.Header(0, 3, 0, 2);
        b.Const(1.0f); b.Const(2.0f); b.Const(3.0f);
        b.Instr(PSB_OP_LCONST, 8, 3, PSB_OP_UNUSED);  // index 3 >= constCount 3
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: LCONST beyond declared constants rejected");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 3, 0, 2);
        b.Const(1.0f); b.Const(2.0f); b.Const(3.0f);
        b.Instr(PSB_OP_LCONST, 8, 2, PSB_OP_UNUSED);
        b.End();
        check(Decode(b, prog) == Result::Ok, "decode: LCONST inside pool accepted");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 0, 0, 2);
        b.Instr(PSB_OP_LUNI, 8, PSB_MAX_UNIFORMS, PSB_OP_UNUSED);
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: LUNI beyond uniform table rejected");
    }

    // ── D17: END semantics ──────────────────────────────────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b;
        b.Header(0, 0, 0, 2);
        b.End();
        b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, PSB_REG_IN_R, PSB_OP_UNUSED);  // unreachable
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: END before last instruction rejected");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 0, 0, 1);
        b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, PSB_REG_IN_R, PSB_OP_UNUSED);  // no END at all
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: missing terminator END rejected");
    }

    // ── D18: non-finite constants ───────────────────────────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b;
        b.Header(0, 1, 0, 2);
        b.ConstBits(0x7FC00000u);  // quiet NaN
        b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, PSB_OP_CONST_BASE, PSB_OP_UNUSED);
        b.End();
        check(Decode(b, prog) == Result::InvalidValue, "decode: NaN constant rejected");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 1, 0, 2);
        b.ConstBits(0x7F800000u);  // +inf
        b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, PSB_OP_CONST_BASE, PSB_OP_UNUSED);
        b.End();
        check(Decode(b, prog) == Result::InvalidValue, "decode: infinite constant rejected");
    }

    // ── D19–D23: uniform descriptors ────────────────────────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b;
        b.Header(0, 0, 1, 1);
        b.Uniform(0xAAAA, PSB_UNIFORM_FLOAT, 2, PSB_UNIFORM_NO_DEFAULT);  // auto slot
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: descriptor on auto-bound slot rejected");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 0, 2, 1);
        b.Uniform(0xAAAA, PSB_UNIFORM_VEC2,  3, PSB_UNIFORM_NO_DEFAULT);  // slots 3,4
        b.Uniform(0xBBBB, PSB_UNIFORM_FLOAT, 4, PSB_UNIFORM_NO_DEFAULT);  // overlaps 4
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: overlapping uniform slots rejected");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 0, 2, 1);
        b.Uniform(0xAAAA, PSB_UNIFORM_VEC2,  3, PSB_UNIFORM_NO_DEFAULT);  // slots 3,4
        b.Uniform(0xBBBB, PSB_UNIFORM_FLOAT, 5, PSB_UNIFORM_NO_DEFAULT);  // slot 5 free
        b.End();
        check(Decode(b, prog) == Result::Ok,
              "decode: non-overlapping vec2+float descriptors accepted");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 0, 1, 1);
        b.Uniform(0xAAAA, PSB_UNIFORM_VEC4, 13, PSB_UNIFORM_NO_DEFAULT);  // 13..16 overrun
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: vec4 descriptor overflowing uniform table rejected");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 0, 1, 1);
        b.Uniform(0xAAAA, 4 /*unknown type*/, 3, PSB_UNIFORM_NO_DEFAULT);
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: unknown uniform type rejected");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 1, 1, 1);
        b.Uniform(0xAAAA, PSB_UNIFORM_FLOAT, 3, 2);   // misaligned default offset
        b.Const(1.0f);
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: misaligned default offset rejected");

        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 1, 1, 1);
        b.Uniform(0xAAAA, PSB_UNIFORM_FLOAT, 3, 4);   // past the 1-float pool
        b.Const(1.0f);
        b.End();
        check(Decode(b, prog) == Result::InvalidValue,
              "decode: default offset beyond constants pool rejected");

        // Valid defaults are APPLIED to the uniform table.
        prog = ShaderProgram{};
        b = PsbBlobBuilder{};
        b.Header(0, 2, 1, 1);
        b.Uniform(0xAAAA, PSB_UNIFORM_VEC2, 3, 0);    // defaults = constants[0..1]
        b.Const(0.25f); b.Const(0.5f);
        b.End();
        check(Decode(b, prog) == Result::Ok &&
              prog.uniforms[3] == 0.25f && prog.uniforms[4] == 0.5f,
              "decode: declared uniform defaults applied from constants pool");
    }

    // ── D24: profile cap on program slots ───────────────────────────────
    {
        ShaderProgram prog;
        PsbBlobBuilder b = Passthrough();
        check(Decode(b, prog, GpuConfig::MAX_SHADER_PROGRAMS) == Result::InvalidHandle,
              "decode: programId at/above firmware profile cap rejected");
    }

    // ── V1–V7: VM execution semantics on verified programs ──────────────
    {
        ShaderProgram prog;
        PglShaderVM vm;
        float r, g, bl;

        // V1: passthrough
        uint8_t blob[PSB_MAX_PROGRAM_SIZE];
        size_t n = PsbBuildPassthrough(blob);
        check(DecodeShaderProgram(blob, n, 0, prog) == Result::Ok, "vm: passthrough decodes");
        vm.Execute(prog, prog.uniforms, 3.0f, 4.0f, 0.25f, 0.5f, 0.75f, nullptr, 16, 8, r, g, bl);
        check(r == 0.25f && g == 0.5f && bl == 0.75f, "vm: passthrough copies input colour");

        // V2: END-only program → default output = input (passthrough default)
        {
            PsbBlobBuilder b;
            b.Header(0, 0, 0, 1);
            b.End();
            prog = ShaderProgram{};
            check(Decode(b, prog) == Result::Ok, "vm: END-only program decodes");
            vm.Execute(prog, prog.uniforms, 0.0f, 0.0f, 0.25f, 0.5f, 0.75f, nullptr, 16, 8, r, g, bl);
            check(r == 0.25f && g == 0.5f && bl == 0.75f,
                  "vm: END-only program passes input through unchanged");
        }

        // V3: invert arithmetic
        n = PsbBuildInvert(blob);
        prog = ShaderProgram{};
        check(DecodeShaderProgram(blob, n, 0, prog) == Result::Ok, "vm: invert decodes");
        vm.Execute(prog, prog.uniforms, 0.0f, 0.0f, 0.25f, 0.5f, 0.75f, nullptr, 16, 8, r, g, bl);
        check(r == 0.75f && g == 0.5f && bl == 0.25f, "vm: invert computes 1-c");

        // V4: encoded constant operand
        {
            PsbBlobBuilder b;
            b.Header(0, 1, 0, 2);
            b.Const(0.5f);
            b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, PSB_OP_CONST_BASE, PSB_OP_UNUSED);
            b.End();
            prog = ShaderProgram{};
            check(Decode(b, prog) == Result::Ok, "vm: constant-operand program decodes");
            vm.Execute(prog, prog.uniforms, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, nullptr, 16, 8, r, g, bl);
            check(r == 0.5f, "vm: encoded constant operand resolves");
        }

        // V5: non-finite result containment (1e38 * 1e38 overflows to inf)
        {
            PsbBlobBuilder b;
            b.Header(0, 1, 0, 4);
            b.Const(1.0e38f);
            b.Instr(PSB_OP_LCONST, 8, 0, PSB_OP_UNUSED);
            b.Instr(PSB_OP_MUL, 8, 8, 8);                 // r8 = 1e38^2 → +inf
            b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, 8, PSB_OP_UNUSED);
            b.End();
            prog = ShaderProgram{};
            check(Decode(b, prog) == Result::Ok, "vm: overflow program decodes");
            vm.Execute(prog, prog.uniforms, 0.0f, 0.0f, 0.25f, 0.5f, 0.75f, nullptr, 16, 8, r, g, bl);
            check(std::isfinite(r) && r == 0.0f && g == 0.5f && bl == 0.75f,
                  "vm: non-finite output contained to 0.0, other channels intact");
        }

        // V6: END halts (MOV before END executed, output written)
        // Covered by V3/V4 structure; here NOP is a defined no-op:
        {
            PsbBlobBuilder b;
            b.Header(0, 0, 0, 2);
            b.Instr(PSB_OP_NOP, PSB_OP_UNUSED, PSB_OP_UNUSED, PSB_OP_UNUSED);
            b.End();
            prog = ShaderProgram{};
            check(Decode(b, prog) == Result::Ok, "vm: NOP program decodes");
            vm.Execute(prog, prog.uniforms, 0.0f, 0.0f, 0.25f, 0.5f, 0.75f, nullptr, 16, 8, r, g, bl);
            check(r == 0.25f && g == 0.5f && bl == 0.75f, "vm: NOP preserves default output");
        }

        // V7: declared uniform defaults readable via encoded uniform operand
        {
            PsbBlobBuilder b;
            b.Header(0, 2, 1, 4);
            b.Uniform(0xAAAA, PSB_UNIFORM_VEC2, 3, 0);
            b.Const(0.25f); b.Const(0.5f);
            b.Instr(PSB_OP_MOV, PSB_REG_OUT_R, PSB_OP_UNIFORM_BASE + 3, PSB_OP_UNUSED);
            b.Instr(PSB_OP_MOV, PSB_REG_OUT_G, PSB_OP_UNIFORM_BASE + 4, PSB_OP_UNUSED);
            b.Instr(PSB_OP_MOV, PSB_REG_OUT_B, PSB_OP_LITERAL_BASE, PSB_OP_UNUSED);
            b.End();
            prog = ShaderProgram{};
            check(Decode(b, prog) == Result::Ok, "vm: default-uniform program decodes");
            vm.Execute(prog, prog.uniforms, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, nullptr, 16, 8, r, g, bl);
            check(r == 0.25f && g == 0.5f && bl == 0.0f,
                  "vm: encoded uniform operand reads the applied default");
        }
    }

    // ── C1: host PGLSL compiler output passes strict verification ───────
    {
        const char* kInvertSrc = R"pglsl(
void main() {
    vec2 uv = gl_FragCoord.xy / u_resolution;
    vec4 color = texture2D(u_framebuffer, uv);
    gl_FragColor = vec4(vec3(1.0) - color.rgb, 1.0);
}
)pglsl";
        auto res = PglShaderCompiler::Compile(kInvertSrc, std::strlen(kInvertSrc));
        check(res.success, "compile: stock-style invert PGLSL compiles");
        if (res.success) {
            ShaderProgram prog;
            check(DecodeShaderProgram(res.bytecode, res.bytecodeSize, 0, prog) == Result::Ok &&
                  prog.readsFramebuffer && prog.verified,
                  "compile→decode: texture2D invert accepted, TEX2D derived");
        }

        // Two vec4 uniforms exercise component-count slot advance (3..6, 7..10);
        // the old slot=3+index allocator would overlap them and fail decode.
        const char* kVec4Src = R"pglsl(
uniform vec4 u_a;
uniform vec4 u_b;
void main() {
    gl_FragColor = vec4(u_a.rgb + u_b.rgb, 1.0);
}
)pglsl";
        res = PglShaderCompiler::Compile(kVec4Src, std::strlen(kVec4Src));
        check(res.success, "compile: two vec4 uniforms compile");
        if (res.success) {
            ShaderProgram prog;
            check(DecodeShaderProgram(res.bytecode, res.bytecodeSize, 0, prog) == Result::Ok,
                  "compile→decode: vec4 uniform slots non-overlapping, accepted");
        }
    }

    std::printf("\n%s (%d failures)\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
}
