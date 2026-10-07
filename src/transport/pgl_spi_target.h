#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <PglRuntimeProtocol.h>

#if defined(PICO_ON_DEVICE)
#include "../hardware_resources.h"  // lease types for the device implementation
#else
class HardwareResources;
#endif

// ─── PglSpiTarget — PIO mode-0 SPI target (RP2350 side of the runtime link) ─
//
// Wire contract (PglRuntimeProtocol.h v9, P06):
//  * Every transaction is one single-lane command byte on D0, MSB first,
//    mode 0 (SCK idles low, RP samples MOSI on the rising edge, changes MISO
//    on the falling edge). No address phase.
//  * 0x80 WriteControl: 32 wire bytes total including the prefix, all
//    single-lane; the 32 bytes are exactly PglRuntime::EncodeControl output.
//  * 0x00 ReadRecord: command byte only (8 clocks), then 64 RX-only bytes
//    streamed from an immutable prepacked snapshot. MISO (D1) is driven only
//    after the command phase of a read; it is an input at all other times.
//  * 0x81 WriteBulk: prefix byte, then the reserved logical payload at the
//    reserved width. Single lane: bytes MSB first on D0. Quad: two clocks per
//    byte, first clock carries bits7..4 (D3=bit7 … D0=bit4), second clock
//    bits3..0. The bulk prefix is outside the logical payload and never lands
//    in the payload buffer.
//  * Last prefix bit selects the following RX data width in PIO: 0 → control
//    (1-bit IN), 1 → the reserved bulk program (IN 4 or IN 1, patched while
//    the SM is idle). Control and bulk are therefore distinguishable while a
//    bulk reservation is armed, with no CPU reaction inside a clock edge.
//  * Minimum CS gap 64 µs (PglRuntime::MinimumTransactionGapUs), CS setup and
//    hold ≥ 1 µs (≫ the 2-flop input synchronizers at any admitted clk_sys),
//    initial SCK 1 MHz. Conservatively allowing 8 clk_sys cycles per SCK
//    half-period gives a sampling bound of clk_sys/16 (4.6875 MHz at75MHz)
//    [INFERENCE from instruction counts], not a qualified external rate.
//    Any rate above 1 MHz requires separate physical qualification.
//
// Robustness design:
//  * RX shifts a sentinel 1 bit into the ISR before each byte, then 8 single
//    bits or 2 quad nibbles, then pushes manually. Every complete byte word
//    therefore has bit8 set (word = 0x100|byte) and DMA drains the low 8 bits
//    of each pushed word. At CS rise the ISR residue is captured: value 0 or
//    1 means a clean byte boundary; any larger value — including (1<<k) from
//    k all-zero partial bits — is a partial bit cell and aborts the packet.
//  * The prefix is stream word 0 for every kind; a chained 1-transfer DMA
//    channel parks it in a driver-owned scratch before the payload channel
//    starts, so the wire prefix byte (0x80/0x81/0x00) itself always
//    classifies the transaction — a 32-byte bulk (prefix+32 = 33 wire bytes)
//    and a control (prefix+31 = 32 wire bytes) are never ambiguous, and the
//    prefix never lands in the payload buffer.
//  * CS rising edge (GPIO IRQ, core 0) disables both SMs, RP2350-E5-safely
//    stops all three DMA channels (EN cleared on every member of the prefix→
//    payload chain before any abort), captures the DMA count deltas, the
//    ≤8-deep joined RX FIFO tail and the ISR residue (bounded: ≤10 words),
//    releases MISO, drops READY and publishes exactly one capture. No CRC or
//    parsing runs in IRQ context.
//  * Payload DMA runs in NORMAL mode with the exact armed byte count, so it
//    can never write past the armed region: extra host clocks stall the SM's
//    blocking push once the FIFO fills and are counted as overflow, short
//    clocks leave a count delta; both reject without committing.
//  * The TX SM pushes one marker word into its own RX FIFO only when it
//    drives MISO, recorded in the capture as a read cross-check (the wire
//    prefix remains authoritative).
//
// Ownership: control RX storage, the prefix scratch, the capture buffers and
// the two 256-byte prepacked TX snapshots are driver-owned. Bulk payload
// buffers are borrowed from the parent (Arm) and are written only by the
// armed DMA plus the bounded FIFO tail, never beyond the armed logical
// length. An active TX snapshot is immutable; PublishRecord double-buffers
// and activates at idle.
//
// Control and read transactions remain available even when a very short bulk
// is armed. The physical RX destination reserves at least 31 body bytes; only
// the exact logical bulk count can commit.

namespace PglSpiTargetDetail {

constexpr uint8_t kRxPioBlock = 1;   // PIO1: receiver program (30 words, 1 SM)
constexpr uint8_t kTxPioBlock = 2;   // PIO2: status/read program (17 words, 1 SM)
constexpr uint8_t kRxProgramWords = 30;
constexpr uint8_t kTxProgramWords = 17;
// Index of the idle-patched bulk-dispatch JMP inside pgl_spi_rx, and its
// legal targets (single-lane byte loop / quad byte loop / unarmed drain).
constexpr uint8_t kRxPatchIndex = 10;
constexpr uint8_t kRxTargetSingle = 11;
constexpr uint8_t kRxTargetQuad = 19;
constexpr uint8_t kRxTargetDrain = 27;

constexpr uint16_t kByteSentinel = 0x100;  // bit8 set on every complete-byte word
constexpr uint8_t kMaxCapturedWords = 10;  // 8-deep joined RX FIFO + 2 exec pushes
constexpr uint32_t kStallTimeoutUs = 500000;  // bounded expiry of clock-stopped transactions

// One prepacked TX FIFO word: byte in bits31..24, emitted MSB first by
// `out pins, 1` with left shift and autopull 8.
inline uint32_t PackMsbWord(uint8_t byte) { return uint32_t(byte) << 24; }

inline uint32_t PhysicalBodyBytes(uint32_t logicalBytes) {
    return logicalBytes < PglRuntime::ControlBytes-1 ?
        uint32_t(PglRuntime::ControlBytes-1) : logicalBytes;
}
// The physical region also holds a control body while bulk is armed.
// Logical lengths 1..MaxBatchBytes remain legal, including 32.
inline PglRuntime::Result ValidateArm(size_t capacity, uint32_t logicalBytes, uint8_t lanes) {
    if (lanes != 1 && lanes != 4) return PglRuntime::Result::InvalidValue;
    if (!logicalBytes || logicalBytes > PglRuntime::MaxBatchBytes) return PglRuntime::Result::InvalidValue;
    if (capacity < PhysicalBodyBytes(logicalBytes)) return PglRuntime::Result::Capacity;
    return PglRuntime::Result::Ok;
}

// Bounded capture taken at CS rise (IRQ) or at a stall deadline (Poll).
struct Capture {
    uint32_t dmaBytes = 0;                    // payload-channel transfers into the armed dst
    uint16_t words[kMaxCapturedWords] = {};   // drained FIFO words then exec-pushed ISR residue(s)
    uint8_t wordCount = 0;
    bool truncated = false;                   // defensive: more words than the capture holds
    bool prefixValid = false;                 // the 1-transfer prefix channel completed
    uint8_t prefix = 0;                       // driver-owned prefix scratch contents
    bool misoDriven = false;                  // TX SM pushed its read marker (cross-check)
    bool dstWasBulk = false;                  // armed destination was the parent's bulk buffer
};

// Result of folding a capture's drained words into the armed destination.
struct AbsorbResult {
    uint32_t totalBytes = 0;   // dmaBytes + every complete-byte word (placed + excess)
    uint32_t placedBytes = 0;  // bytes that landed inside the dst limit
    uint8_t excessCount = 0;   // complete bytes beyond the dst limit (in excess[])
    bool partial = false;      // ISR residue > 1: partial bit cell (incl. all-zero bits)
    bool inconsistent = false; // byte words after the residue / repeated residue
};

// Append complete-byte words to dst (bounded by dstLimit total bytes,
// including the DMA bytes already there); overflow bytes go to excess.
// Never writes past dstLimit or excessCap. The first word without the
// sentinel is the ISR residue: 0 or 1 is a clean byte boundary, anything
// larger is a partial bit cell (an all-zero partial cell leaves (1<<k) > 1).
inline AbsorbResult AbsorbCapturedBytes(const Capture& cap, uint8_t* dst, uint32_t dstLimit,
                                        uint8_t* excess, uint8_t excessCap) {
    AbsorbResult r{};
    r.totalBytes = cap.dmaBytes;
    r.placedBytes = cap.dmaBytes;
    bool residueSeen = false;
    for (uint8_t i = 0; i < cap.wordCount; ++i) {
        const uint16_t w = cap.words[i];
        if (w & kByteSentinel) {
            if (residueSeen) { r.inconsistent = true; continue; }
            if (r.placedBytes < dstLimit && dst) dst[r.placedBytes++] = w & 0xffu;
            else if (r.excessCount < excessCap && excess) excess[r.excessCount++] = w & 0xffu;
            else r.inconsistent = true;  // capture larger than the bounded sinks
            ++r.totalBytes;
        } else if (!residueSeen) {
            residueSeen = true;
            if (w > 1) r.partial = true;
        } else if (w > 1) {
            r.inconsistent = true;  // a second nonzero residue cannot happen
        }
    }
    if (cap.truncated) r.inconsistent = true;
    return r;
}

enum class TransactionKind : uint8_t { Empty = 0, Read, Control, Bulk, Rejected };

struct Decision {
    TransactionKind kind = TransactionKind::Empty;
    uint32_t commitBytes = 0;      // Control: 32; Bulk: logicalBytes
    bool overflow = false;         // host clocked more than the armed/expected count
    bool partial = false;          // partial bit cell at CS rise
    bool keepReservation = false;  // armed bulk reservation survives (re-arm)
    uint32_t receivedBytes = 0;    // diagnostic: complete bytes observed
};

// Pure commit/reject decision. The captured wire prefix (stream word 0,
// parked in the driver-owned scratch by the chained 1-transfer channel) is
// authoritative; `ab` covers only the bytes after the prefix. Rules:
//  * partial/inconsistent capture → Rejected, nothing commits.
//  * no complete prefix and no data → Empty (CS pulse / <8 clean clocks);
//    data without a prefix is impossible and rejects.
//  * 0x00 → Read: the immutable snapshot was streamed; RX dummy bytes and
//    their length are ignored and the reservation is untouched.
//  * 0x80 → Control commits on exactly 31 following bytes (32 wire bytes
//    including the prefix); short/overlong reject without committing.
//  * 0x81 → Bulk commits only against an armed reservation with exactly the
//    reserved logical byte count (33 wire bytes for a 32-byte payload — never
//    aliased with control); otherwise Rejected, reservation kept.
//  * any other prefix → Rejected; partial data never commits.
inline Decision DecideTransaction(const AbsorbResult& ab, bool prefixValid, uint8_t prefix,
                                  bool dstWasBulk, uint32_t logicalBytes) {
    Decision d{};
    d.receivedBytes = ab.totalBytes;
    d.partial = ab.partial;
    if (ab.partial || ab.inconsistent) {
        d.kind = TransactionKind::Rejected;
        d.overflow = ab.inconsistent && !ab.partial;
        d.keepReservation = dstWasBulk;
        return d;
    }
    if (!prefixValid) {
        d.kind = ab.totalBytes == 0 ? TransactionKind::Empty : TransactionKind::Rejected;
        d.keepReservation = dstWasBulk;
        return d;
    }
    if (prefix == PglRuntime::ReadRecord) {  // host-side CRC covers truncation
        d.kind = TransactionKind::Read;
        d.keepReservation = dstWasBulk;
        return d;
    }
    if (prefix == PglRuntime::WriteControl) {
        // Exactly 31 bytes follow the prefix. When armed, part of the body
        // may sit in the excess tail (armed length < 31) — that is
        // reconstruction material, not overflow; totalBytes already counts it.
        if (ab.totalBytes == PglRuntime::ControlBytes - 1) {
            d.kind = TransactionKind::Control;
            d.commitBytes = PglRuntime::ControlBytes;
            d.keepReservation = dstWasBulk;  // control never consumes a reservation
        } else {
            d.kind = TransactionKind::Rejected;
            d.overflow = ab.totalBytes > PglRuntime::ControlBytes - 1;
            d.keepReservation = dstWasBulk;
        }
        return d;
    }
    if (prefix == PglRuntime::WriteBulk) {
        if (dstWasBulk && ab.totalBytes == logicalBytes && ab.excessCount == 0) {
            d.kind = TransactionKind::Bulk;
            d.commitBytes = logicalBytes;
            return d;  // reservation consumed
        }
        d.kind = TransactionKind::Rejected;
        d.overflow = dstWasBulk && ab.totalBytes > logicalBytes;
        d.keepReservation = dstWasBulk;
        return d;
    }
    d.kind = TransactionKind::Rejected;
    d.keepReservation = dstWasBulk;
    return d;
}

// Public commit view derived from a Decision — the single source of truth for
// the RxTransaction fields handed to the parent. Bytes are exposed only for
// committed Control (driver controlBuf) and Bulk (armed parent buffer);
// Read/Empty/Rejected carry no bytes. The kind is authoritative: it comes
// from the captured wire prefix, never from payload contents or counts, so a
// 32-byte bulk whose payload mimics a valid control record must never be
// routed as control.
struct CommitView {
    TransactionKind kind = TransactionKind::Empty;
    uint32_t length = 0;
    bool hasBytes = false;
    bool overflow = false;
    bool partial = false;
};

inline CommitView MakeCommitView(const Decision& d) {
    CommitView v{};
    v.kind = d.kind;
    v.overflow = d.overflow;
    v.partial = d.partial;
    if (d.kind == TransactionKind::Control || d.kind == TransactionKind::Bulk) {
        v.length = d.commitBytes;
        v.hasBytes = true;
    }
    return v;
}

// Reassemble the first n received bytes when they straddle the armed dst and
// the excess tail (control-while-armed with logicalBytes < 31).
inline void GatherReceived(uint8_t* out, uint32_t n, const uint8_t* dst, uint32_t placedBytes,
                           const uint8_t* excess, uint8_t excessCount) {
    for (uint32_t i = 0; i < n; ++i) {
        if (i < placedBytes) out[i] = dst[i];
        else if (i - placedBytes < excessCount) out[i] = excess[i - placedBytes];
        else out[i] = 0;  // unreachable for a committed control; zero-fill defensively
    }
}

// Double-buffered immutable TX snapshot. Publish encodes (CRC16 included) and
// prepacks into the inactive buffer; the active buffer is never touched while
// a transaction can be streaming it.
struct TxSnapshots {
    uint32_t words[2][PglRuntime::RecordBytes] = {};
    uint8_t active = 0;
    bool pendingSwap = false;

    const uint32_t* Publish(const PglRuntime::Record& record, bool transactionLive) {
        const uint8_t next = active ^ 1;
        uint8_t bytes[PglRuntime::RecordBytes];
        PglRuntime::EncodeRecord(record, bytes, sizeof(bytes));
        for (size_t i = 0; i < PglRuntime::RecordBytes; ++i) words[next][i] = PackMsbWord(bytes[i]);
        if (transactionLive) {
            pendingSwap = true;  // activate at CS rise / idle, never mid-stream
            return words[active];
        }
        active = next;
        pendingSwap = false;
        return words[active];
    }

    const uint32_t* EndTransaction() {
        if (pendingSwap) {
            active ^= 1;
            pendingSwap = false;
        }
        return words[active];
    }

    const uint32_t* Active() const { return words[active]; }
};

} // namespace PglSpiTargetDetail

class PglSpiTarget {
public:
    struct RxTransaction {
        // Authoritative classification from the captured wire prefix
        // (Detail::MakeCommitView). Parent MUST route on kind and never
        // infer it from bytes/length/payload contents.
        PglSpiTargetDetail::TransactionKind kind = PglSpiTargetDetail::TransactionKind::Empty;
        const uint8_t* bytes = nullptr;  // Control: driver-owned 32 B; Bulk: armed parent buffer
        size_t length = 0;               // 0 for Read/Empty/Rejected
        bool overflow = false;
        bool partial = false;
    };

    // Claims GPIO0..5+22, PIO1 SM+RX program, PIO2 SM+TX program and three DMA
    // channels (prefix chain head, payload, TX) from the central registry;
    // any failure rolls all leases back.
    // Must run on core 0 (the CS GPIO IRQ is registered on the calling core).
    PglRuntime::Result Init(HardwareResources& resources);
    // Record a bulk reservation. Lanes 1 or 4; capacity must cover
    // max(logicalBytes,31), so control stays available for the shortest bulk.
    // Logical length 1..MaxBatchBytes, prefix excluded. Valid only while a transaction
    // is held or the link is not armed (returns Busy otherwise) — the safe
    // pattern is Arm() then ReleaseTransaction() for the held control.
    // Never disturbs a live CS.
    PglRuntime::Result Arm(uint8_t* buffer, size_t capacity, uint32_t logicalBytes, uint8_t lanes);
    PglRuntime::Result Disarm();  // same arming-window rules as Arm()
    // Encode + prepack into the inactive snapshot; activates at idle.
    PglRuntime::Result PublishRecord(const PglRuntime::Record& record);
    // Returns Pending with no completed capture, else Ok with exactly one
    // held transaction. Every completed transaction is held until
    // ReleaseTransaction(); the link is not re-armed (READY stays low) until
    // then. Read completion arrives as {kind=Read, bytes=nullptr, length=0}.
    PglRuntime::Result PopTransaction(RxTransaction& out);
    PglRuntime::Result ReleaseTransaction();
    // Busy while CS is live, a bulk is armed, or a transaction is held.
    PglRuntime::Result Resume(uint32_t systemClockHz);  // preserves divider/ceiling policy
    PglRuntime::Result Quiesce();
    void SetReady(bool ready);      // parent gate; READY = gate ∧ armed ∧ !quiesced
    void Poll(uint64_t nowUs);      // bounded expiry of clock-stopped CS-low transactions
    void Shutdown();                // force-quiesce, then release every lease

    bool BulkArmed() const { return reservation_.active; }
    bool Quiesced() const { return quiesced_; }
    uint32_t RejectedCount() const { return rejectedCount_; }
    uint32_t DroppedCount() const { return droppedCount_; }
    uint32_t StallAbortCount() const { return stallAbortCount_; }

private:
    struct Reservation {
        uint8_t* buffer = nullptr;
        size_t capacity = 0;
        uint32_t logicalBytes = 0;
        uint8_t lanes = 0;
        bool active = false;
    };

#if defined(PICO_ON_DEVICE)
    void Rearm();             // DMA/SM/FIFO prepare for the next transaction, READY truthful
    void DisableSmsAndDma();  // stop every chain member before E5-safe abort
    void Finalize(bool stalled);  // CS-rise/stall capture; publishes one event
    void UpdateReady();
    static void IrqTrampoline();

    HardwareResources* resources_ = nullptr;
    HardwareResources::PioLease rxLease_{}, txLease_{};
    int rxPreDma_ = -1, rxDma_ = -1, txDma_ = -1;
#endif
    Reservation reservation_{};
    uint8_t controlBuf_[PglRuntime::ControlBytes] = {};
    uint8_t prefixScratch_ = 0;  // chained 1-transfer DMA lands the prefix here
    uint8_t excessBuf_[PglSpiTargetDetail::kMaxCapturedWords] = {};
    PglSpiTargetDetail::TxSnapshots snapshots_{};
    PglSpiTargetDetail::Capture capture_{};
    volatile bool capturePending_ = false;
    volatile bool transactionArmed_ = false;  // DMA/FIFO prepared, SMs running, READY truthful
    uint64_t gpioMask_ = 0;
    bool armedDstWasBulk_ = false;
    uint32_t armedRxCount_ = 0;
    bool heldTransaction_ = false;
    RxTransaction heldView_{};
    bool parentReady_ = false;
    bool quiesced_ = false;
    bool initialized_ = false;
    bool csLowSeen_ = false;
    uint64_t csLowSinceUs_ = 0;
    uint32_t rejectedCount_ = 0, droppedCount_ = 0, stallAbortCount_ = 0;
};

#if defined(PICO_ON_DEVICE)
// Process-wide instance (the CS GPIO raw IRQ trampoline is a free function).
PglSpiTarget& PglSpiTargetInstance();
#endif
