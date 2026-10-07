// Deterministic transport-state regressions for the PglSpiTarget pure logic
// (src/transport/pgl_spi_target.h, Detail namespace). These exercise the
// shipped commit/reject/absorb/pack/snapshot code paths with scripted CS-rise
// captures — the same structures the CS IRQ produces on hardware.
//
// Covered acceptance cases:
//  * exact byte/word order (MSB prepack, sentinel byte-word decode)
//  * wire prefix classification: 0x80 control (prefix+31) vs 0x81 bulk
//    (prefix+logical), including the 32-byte bulk vs control-while-armed
//    regression — the chained prefix scratch keeps them unambiguous
//  * CS abort with clock stop, partial zero bits, and all-extra clocks
//  * control while a bulk is armed (incl. <31-byte armed reconstruction)
//  * read-record transactions never consuming a reservation
//  * unknown/unarmed prefixes never committing partial data
//  * bounded tail absorb never writing past the armed length (canary)
//  * immutable double-buffered TX snapshots with idle activation

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "transport/pgl_spi_target.h"

namespace {

int g_failures = 0;
int g_checks = 0;

void Check(bool ok, const char* what) {
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL: %s\n", what);
    }
}

using PglRuntime::Record;
using PglRuntime::Result;
using namespace PglSpiTargetDetail;

// dmaBytes = payload-channel bytes; residue is appended as words.
Capture MakeCapture(uint32_t dmaBytes, bool dstWasBulk, uint8_t prefix, bool prefixValid = true) {
    Capture c{};
    c.dmaBytes = dmaBytes;
    c.dstWasBulk = dstWasBulk;
    c.prefix = prefix;
    c.prefixValid = prefixValid;
    return c;
}

// ─── Arm validation ─────────────────────────────────────────────────────────
void TestValidateArm() {
    Check(ValidateArm(64, 64, 1) == Result::Ok, "arm single lane ok");
    Check(ValidateArm(64, 64, 4) == Result::Ok, "arm quad lane ok");
    Check(ValidateArm(64, 64, 0) == Result::InvalidValue, "arm lanes 0 rejected");
    Check(ValidateArm(64, 64, 2) == Result::InvalidValue, "arm lanes 2 rejected");
    Check(ValidateArm(64, 64, 5) == Result::InvalidValue, "arm lanes 5 rejected");
    Check(ValidateArm(64, 0, 1) == Result::InvalidValue, "arm zero length rejected");
    Check(ValidateArm(64, PglRuntime::MaxBatchBytes + 1, 1) == Result::InvalidValue,
          "arm oversize rejected");
    Check(ValidateArm(32, 32, 1) == Result::Ok,
          "arm exactly-32 bulk accepted (prefix scratch disambiguates)");
    Check(ValidateArm(16, 24, 1) == Result::Capacity, "arm capacity below length rejected");
    Check(ValidateArm(31, 31, 4) == Result::Ok, "arm 31 quad ok");
    Check(ValidateArm(30,19,1)==Result::Capacity,"short ingress must reserve full control-body capacity");
    Check(ValidateArm(31,19,1)==Result::Ok,"short logical resource batch remains armable");
    Check(ValidateArm(33, 33, 1) == Result::Ok, "arm 33 ok");
}

// ─── MSB prepack / record wire order ────────────────────────────────────────
void TestPrepack() {
    Check(PackMsbWord(0xab) == 0xab000000u, "prepack byte to bits31..24");
    Check(PackMsbWord(0x00) == 0u && PackMsbWord(0xff) == 0xff000000u, "prepack extremes");

    TxSnapshots snap;
    Record r{};
    r.info = PglRuntime::Info::Status;
    r.result = Result::Ok;
    r.session = 0x11223344;
    const uint32_t* w = snap.Publish(r, false);
    uint8_t bytes[PglRuntime::RecordBytes];
    Check(PglRuntime::EncodeRecord(r, bytes, sizeof(bytes)), "encode record");
    bool orderOk = true;
    for (size_t i = 0; i < PglRuntime::RecordBytes; ++i)
        if (w[i] != PackMsbWord(bytes[i])) orderOk = false;
    Check(orderOk, "prepacked word order matches record byte order");
    Check(bytes[0] == 'P' && bytes[1] == 'G' && bytes[2] == 'L' && bytes[3] == 'R',
          "record magic little-endian on wire");
    Check(w[0] == uint32_t('P') << 24, "first wire word carries first byte MSB-first");

    // The streamed snapshot decodes as a valid record (CRC valid at publish).
    uint8_t back[PglRuntime::RecordBytes];
    for (size_t i = 0; i < PglRuntime::RecordBytes; ++i) back[i] = uint8_t(w[i] >> 24);
    Record decoded{};
    Check(PglRuntime::DecodeRecord(back, sizeof(back), decoded) == Result::Ok,
          "streamed snapshot is a valid CRC record");
    Check(decoded.session == 0x11223344, "session survives encode/prepack");
}

// ─── Double-buffer immutability / idle activation ───────────────────────────
void TestSnapshotActivation() {
    // Record session (bytes 8..11) distinguishes snapshots; word 8 differs.
    TxSnapshots snap;
    Record a{};
    a.info = PglRuntime::Info::Status;
    a.session = 1;
    const uint32_t* first = snap.Publish(a, false);
    Check(first[8] == PackMsbWord(1), "snapshot A active with session 1");

    Record b{};
    b.info = PglRuntime::Info::Status;
    b.session = 2;
    const uint32_t* during = snap.Publish(b, true);  // transaction in flight
    Check(during == first && first[8] == PackMsbWord(1) && snap.pendingSwap,
          "publish while live leaves active snapshot immutable");
    const uint32_t* after = snap.EndTransaction();
    Check(after != first && !snap.pendingSwap, "pending snapshot activates at transaction end");
    Check(after[8] == PackMsbWord(2), "activated snapshot carries new content");

    Record c{};
    c.info = PglRuntime::Info::Fence;
    c.session = 3;
    snap.Publish(c, true);
    Check(after[8] == PackMsbWord(2), "second publish while live still immutable");
    const uint32_t* latest = snap.EndTransaction();
    Check(latest != after && latest[8] == PackMsbWord(3), "latest pending snapshot wins");
}

// ─── Sentinel tail decode, residue classes, bounded writes ──────────────────
void TestAbsorb() {
    uint8_t buf[40];
    uint8_t excess[kMaxCapturedWords];
    std::memset(buf, 0xcc, sizeof(buf));

    // Complete stream finishing in the FIFO: dma 29 + 2 tail bytes + clean residue.
    {
        Capture c = MakeCapture(29, false, PglRuntime::WriteControl);
        c.words[0] = kByteSentinel | 0x10;
        c.words[1] = kByteSentinel | 0x11;
        c.words[2] = 0;  // residue: clean
        c.wordCount = 3;
        AbsorbResult r = AbsorbCapturedBytes(c, buf, 31, excess, kMaxCapturedWords);
        Check(r.totalBytes == 31 && r.placedBytes == 31 && !r.partial && r.excessCount == 0,
              "clean 31-byte control body absorb");
        Check(buf[29] == 0x10 && buf[30] == 0x11, "tail bytes appended in order");
        Check(buf[31] == 0xcc && buf[39] == 0xcc, "no write past dst limit");
    }
    // Residue value 1 (sentinel only) is also a clean byte boundary.
    {
        Capture c = MakeCapture(5, false, PglRuntime::WriteControl);
        c.words[0] = 1;
        c.wordCount = 1;
        AbsorbResult r = AbsorbCapturedBytes(c, buf, 31, excess, kMaxCapturedWords);
        Check(!r.partial && r.totalBytes == 5, "residue 1 is clean byte boundary");
    }
    // Partial all-zero bit cells: k zero data bits leave residue (1<<k) > 1.
    for (uint8_t k = 1; k <= 7; ++k) {
        Capture c = MakeCapture(3, false, PglRuntime::WriteControl);
        c.words[0] = uint16_t(1u << k);
        c.wordCount = 1;
        AbsorbResult r = AbsorbCapturedBytes(c, buf, 31, excess, kMaxCapturedWords);
        Check(r.partial, "all-zero partial bits detected");
    }
    // Partial with data bits set, and a quad nibble residue (1<<4|nibble).
    {
        Capture c = MakeCapture(0, true, PglRuntime::WriteBulk);
        c.words[0] = uint16_t((1u << 3) | 0x5);  // sentinel + 3 bits
        c.wordCount = 1;
        Check(AbsorbCapturedBytes(c, buf, 40, excess, kMaxCapturedWords).partial,
              "partial bits with data detected");
        c.words[0] = uint16_t((1u << 4) | 0x9);  // quad: one nibble after sentinel
        Check(AbsorbCapturedBytes(c, buf, 40, excess, kMaxCapturedWords).partial,
              "quad half-byte partial detected");
    }
    // Complete byte blocked behind a full FIFO, pushed by exec: still data.
    {
        Capture c = MakeCapture(30, false, PglRuntime::WriteControl);
        c.words[0] = kByteSentinel | 0x77;  // blocked complete byte
        c.words[1] = 0;                     // then clean residue
        c.wordCount = 2;
        AbsorbResult r = AbsorbCapturedBytes(c, buf, 31, excess, kMaxCapturedWords);
        Check(r.totalBytes == 31 && !r.partial && buf[30] == 0x77, "blocked byte recovered");
    }
    // Extra clocks beyond the armed count: excess captured, dst untouched past limit.
    {
        std::memset(buf, 0xcc, sizeof(buf));
        Capture c = MakeCapture(31, false, PglRuntime::WriteControl);
        c.words[0] = kByteSentinel | 0xee;
        c.words[1] = 0;
        c.wordCount = 2;
        AbsorbResult r = AbsorbCapturedBytes(c, buf, 31, excess, kMaxCapturedWords);
        Check(r.totalBytes == 32 && r.excessCount == 1 && excess[0] == 0xee,
              "extra byte captured as excess");
        Check(buf[31] == 0xcc, "armed region not overwritten by extra clocks");
    }
    // Byte word after the residue: inconsistent capture.
    {
        Capture c = MakeCapture(0, false, PglRuntime::WriteControl);
        c.words[0] = 1;
        c.words[1] = kByteSentinel | 0x01;
        c.wordCount = 2;
        Check(AbsorbCapturedBytes(c, buf, 31, excess, kMaxCapturedWords).inconsistent,
              "byte after residue flagged inconsistent");
    }
}

Decision DecideFrom(Capture& c, uint32_t logical, uint8_t* dst, uint32_t limit) {
    uint8_t excess[kMaxCapturedWords];
    AbsorbResult ab = AbsorbCapturedBytes(c, dst, limit, excess, kMaxCapturedWords);
    return DecideTransaction(ab, c.prefixValid, c.prefix, c.dstWasBulk, logical);
}

// ─── Commit/reject matrix (prefix from the chained scratch channel) ─────────
void TestDecide() {
    uint8_t body[31];  // control body lands at controlBuf+1; prefix is separate
    std::memset(body, 0, sizeof(body));

    // Clean control: prefix 0x80 + exactly 31 following bytes.
    {
        Capture c = MakeCapture(31, false, PglRuntime::WriteControl);
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 0, body, 31);
        Check(d.kind == TransactionKind::Control && d.commitBytes == 32 && !d.overflow,
              "clean control commits");
    }
    // Short control (CS abort after prefix + 30 bytes).
    {
        Capture c = MakeCapture(30, false, PglRuntime::WriteControl);
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 0, body, 31);
        Check(d.kind == TransactionKind::Rejected && !d.overflow, "short control rejected");
    }
    // Control with a 32nd body byte (all-extra clocks).
    {
        Capture c = MakeCapture(31, false, PglRuntime::WriteControl);
        c.words[0] = kByteSentinel | 0x00;
        c.words[1] = 0; c.wordCount = 2;
        Decision d = DecideFrom(c, 0, body, 31);
        Check(d.kind == TransactionKind::Rejected && d.overflow, "extra-clock control rejected");
    }
    // Clock stop mid-byte (partial), e.g. aborted after 10 bytes + 3 clocks.
    {
        Capture c = MakeCapture(10, false, PglRuntime::WriteControl);
        c.words[0] = (1u << 3); c.wordCount = 1;
        Decision d = DecideFrom(c, 0, body, 31);
        Check(d.kind == TransactionKind::Rejected && d.partial, "clock-stop partial rejected");
    }
    // Unknown prefix never commits even with a clean count.
    {
        Capture c = MakeCapture(31, false, 0x40);
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 0, body, 31);
        Check(d.kind == TransactionKind::Rejected, "unknown prefix rejected");
    }
    // Read: prefix 0x00; dummy RX bytes and their length are ignored.
    {
        Capture c = MakeCapture(31, false, PglRuntime::ReadRecord);
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 0, body, 31);
        Check(d.kind == TransactionKind::Read && !d.keepReservation, "read completes cleanly");
    }
    // Read with extra dummy clocks: still a read, reservation untouched.
    {
        Capture c = MakeCapture(31, false, PglRuntime::ReadRecord);
        c.words[0] = kByteSentinel | 0x00;
        c.words[1] = kByteSentinel | 0x00;
        c.words[2] = 0; c.wordCount = 3;
        Decision d = DecideFrom(c, 0, body, 31);
        Check(d.kind == TransactionKind::Read, "read ignores extra dummy clocks safely");
    }
    // Unarmed bulk: prefix captured, PIO drained the payload, nothing commits.
    {
        Capture c = MakeCapture(0, false, PglRuntime::WriteBulk);
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 0, body, 31);
        Check(d.kind == TransactionKind::Rejected && !d.keepReservation,
              "unarmed bulk rejected without commit");
    }
    // CS pulse with no complete prefix.
    {
        Capture c = MakeCapture(0, false, 0, false);
        c.words[0] = 1; c.wordCount = 1;  // sentinel residue only
        Decision d = DecideFrom(c, 0, body, 31);
        Check(d.kind == TransactionKind::Empty, "CS pulse is an empty transaction");
    }
    // Data bytes without a completed prefix: impossible wire state, reject.
    {
        Capture c = MakeCapture(5, false, 0, false);
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 0, body, 31);
        Check(d.kind == TransactionKind::Rejected, "data without prefix rejected");
    }

    // Armed bulk: exact clean payload commits and consumes the reservation.
    uint8_t bulk[128];
    std::memset(bulk, 0, sizeof(bulk));
    bulk[0] = 0x55;
    {
        Capture c = MakeCapture(100, true, PglRuntime::WriteBulk);
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 100, bulk, 100);
        Check(d.kind == TransactionKind::Bulk && d.commitBytes == 100 && !d.keepReservation,
              "armed bulk commits exactly");
    }
    // 32-byte bulk vs control-while-armed regression: prefix disambiguates.
    {
        Capture c = MakeCapture(32, true, PglRuntime::WriteBulk);
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 32, bulk, 32);
        Check(d.kind == TransactionKind::Bulk && d.commitBytes == 32 && !d.keepReservation,
              "32-byte bulk commits as bulk (prefix 0x81)");
    }
    {
        Capture c = MakeCapture(31, true, PglRuntime::WriteControl);  // control, armed len 32
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 32, bulk, 32);
        Check(d.kind == TransactionKind::Control && d.commitBytes == 32 && d.keepReservation,
              "control while armed at length 32 commits as control (prefix 0x80)");
    }
    // A bulk payload that begins with 0x80 is still a bulk (prefix decides).
    {
        uint8_t b80[64];
        std::memset(b80, 0, sizeof(b80));
        b80[0] = 0x80;
        Capture c = MakeCapture(64, true, PglRuntime::WriteBulk);
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 64, b80, 64);
        Check(d.kind == TransactionKind::Bulk, "bulk payload starting 0x80 not misread");
    }
    // Bulk with extra clocks: reject, reservation retained for host retry.
    {
        Capture c = MakeCapture(100, true, PglRuntime::WriteBulk);
        c.words[0] = kByteSentinel | 0x01;
        c.words[1] = 0; c.wordCount = 2;
        Decision d = DecideFrom(c, 100, bulk, 100);
        Check(d.kind == TransactionKind::Rejected && d.overflow && d.keepReservation,
              "overlong bulk rejected, reservation kept");
    }
    // Bulk aborted short (CS rise at 40 bytes): reject, reservation retained.
    {
        Capture c = MakeCapture(40, true, PglRuntime::WriteBulk);
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 100, bulk, 100);
        Check(d.kind == TransactionKind::Rejected && !d.overflow && d.keepReservation,
              "short bulk rejected, reservation kept");
    }
    // Read while armed: reservation untouched.
    {
        Capture c = MakeCapture(64, true, PglRuntime::ReadRecord);  // dummies in bulk buffer
        c.words[0] = 0; c.wordCount = 1;
        Decision d = DecideFrom(c, 100, bulk, 100);
        Check(d.kind == TransactionKind::Read && d.keepReservation,
              "read while armed keeps reservation");
    }
    // Partial bulk residue never commits even at the exact byte count.
    {
        Capture c = MakeCapture(100, true, PglRuntime::WriteBulk);
        c.words[0] = (1u << 2); c.wordCount = 1;
        Decision d = DecideFrom(c, 100, bulk, 100);
        Check(d.kind == TransactionKind::Rejected && d.partial && d.keepReservation,
              "exact-count bulk with partial tail rejected");
    }
}

// ─── Control-while-armed reconstruction across dst + excess ─────────────────
void TestGather() {
    // Armed length 24 (< 31): 24 body bytes DMA'd into the bulk buffer, the
    // final 7 control body bytes captured in the excess tail.
    uint8_t bulk[24];
    uint8_t excess[kMaxCapturedWords];
    for (uint8_t i = 0; i < 24; ++i) bulk[i] = i;
    for (uint8_t i = 0; i < 7; ++i) excess[i] = uint8_t(24 + i);
    uint8_t ctrlBody[PglRuntime::ControlBytes - 1];
    GatherReceived(ctrlBody, PglRuntime::ControlBytes - 1, bulk, 24, excess, 7);
    bool ok = true;
    for (uint8_t i = 0; i < 31; ++i) if (ctrlBody[i] != i) ok = false;
    Check(ok, "control body reconstructed across armed buffer and tail");

    // A matching decision for that capture: total 31, 7 excess, prefix 0x80.
    Capture c = MakeCapture(24, true, PglRuntime::WriteControl);
    for (uint8_t i = 0; i < 7; ++i) c.words[i] = uint16_t(kByteSentinel | uint8_t(24 + i));
    c.words[7] = 0;
    c.wordCount = 8;
    uint8_t ex2[kMaxCapturedWords];
    AbsorbResult ab = AbsorbCapturedBytes(c, bulk, 24, ex2, kMaxCapturedWords);
    Check(ab.totalBytes == 31 && ab.placedBytes == 24 && ab.excessCount == 7 && !ab.partial,
          "small-armed control absorbs across the tail");
    Decision d = DecideTransaction(ab, true, PglRuntime::WriteControl, true, 24);
    Check(d.kind == TransactionKind::Control && d.keepReservation,
          "control while armed below 31 bytes commits from reconstruction");
}

// ─── Authoritative kind routing (consumer invariant, not wiring) ────────────
void TestKindRouting() {
    // A 32-byte bulk whose payload is a byte-valid control record (RESET with
    // a valid CRC) must still classify as Bulk: kind comes from the captured
    // wire prefix, never from payload contents. This simulates the parent's
    // routing rule: kind==Control → DecodeControl/execute; kind==Bulk → queue.
    uint8_t payload[PglRuntime::ControlBytes];
    PglRuntime::Control hostile{};
    hostile.command = PglRuntime::Command::Reset;
    hostile.session = 1;
    Check(PglRuntime::EncodeControl(hostile, payload, sizeof(payload)), "craft valid control bytes");
    PglRuntime::Control decoded{};
    Check(PglRuntime::DecodeControl(payload, sizeof(payload), decoded) == Result::Ok &&
          decoded.command == PglRuntime::Command::Reset,
          "crafted bulk payload really is a valid RESET control record");

    uint8_t excess[kMaxCapturedWords];
    Capture c = MakeCapture(32, true, PglRuntime::WriteBulk);
    c.words[0] = 0; c.wordCount = 1;
    AbsorbResult ab = AbsorbCapturedBytes(c, payload, 32, excess, kMaxCapturedWords);
    CommitView v = MakeCommitView(DecideTransaction(ab, true, PglRuntime::WriteBulk, true, 32));
    Check(v.kind == TransactionKind::Bulk && v.hasBytes && v.length == 32,
          "hostile control-shaped payload classified Bulk by wire prefix");
    Check(v.kind != TransactionKind::Control, "bulk never routed to control execution path");

    // The mirror: a control-while-armed at armed length 32 routes to control.
    Capture c2 = MakeCapture(31, true, PglRuntime::WriteControl);
    c2.words[0] = 0; c2.wordCount = 1;
    AbsorbResult ab2 = AbsorbCapturedBytes(c2, payload, 32, excess, kMaxCapturedWords);
    CommitView v2 = MakeCommitView(DecideTransaction(ab2, true, PglRuntime::WriteControl, true, 32));
    Check(v2.kind == TransactionKind::Control && v2.hasBytes && v2.length == 32,
          "control-while-armed at length 32 routes to control");
    // The smallest resource envelope may have fewer bytes than a control
    // body. DMA must still consume that entire body without a full-FIFO stall.
    uint8_t shortDst[33]={}, shortExcess[kMaxCapturedWords]={};
    shortDst[31]=0xa5;shortDst[32]=0x5a;
    auto shortControl=MakeCapture(31,true,PglRuntime::WriteControl);
    shortControl.words[0]=0;shortControl.wordCount=1;
    auto shortBytes=AbsorbCapturedBytes(shortControl,shortDst,PhysicalBodyBytes(19),shortExcess,kMaxCapturedWords);
    auto shortDecision=DecideTransaction(shortBytes,true,PglRuntime::WriteControl,true,19);
    Check(shortDecision.kind==TransactionKind::Control && shortDecision.keepReservation &&
          shortDst[31]==0xa5 && shortDst[32]==0x5a,"short armed control completes without exceeding physical destination");
    auto shortBulk=MakeCapture(19,true,PglRuntime::WriteBulk);
    shortBulk.words[0]=0;shortBulk.wordCount=1;
    shortBytes=AbsorbCapturedBytes(shortBulk,shortDst,PhysicalBodyBytes(19),shortExcess,kMaxCapturedWords);
    shortDecision=DecideTransaction(shortBytes,true,PglRuntime::WriteBulk,true,19);
    Check(shortDecision.kind==TransactionKind::Bulk && shortDecision.commitBytes==19,
          "physical control capacity never inflates logical bulk length");

    // View field discipline: Read/Empty/Rejected expose no bytes.
    for (TransactionKind k : {TransactionKind::Read, TransactionKind::Empty,
                              TransactionKind::Rejected}) {
        Decision dd{};
        dd.kind = k;
        CommitView vv = MakeCommitView(dd);
        Check(!vv.hasBytes && vv.length == 0, "non-commit kinds expose no bytes");
    }
}

} // namespace

int main() {
    TestValidateArm();
    TestPrepack();
    TestSnapshotActivation();
    TestAbsorb();
    TestDecide();
    TestGather();
    TestKindRouting();
    std::printf("spi-target transport-state checks: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
