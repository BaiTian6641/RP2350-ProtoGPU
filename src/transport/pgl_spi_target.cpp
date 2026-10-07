#include "pgl_spi_target.h"

#if defined(PICO_ON_DEVICE)

#include "../gpu_config.h"
#include "../hardware_resources.h"

#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include "pgl_spi_target.pio.h"

namespace {
PglSpiTarget* g_irqInstance = nullptr;

constexpr uint8_t kDataBase = GpuConfig::HOST_DATA_BASE;   // D0..D3 = GPIO0..3
constexpr uint8_t kSck = GpuConfig::HOST_SCK;              // GPIO4
constexpr uint8_t kCs = GpuConfig::HOST_CS;                // GPIO5
constexpr uint8_t kReady = GpuConfig::HOST_READY;          // GPIO22
constexpr uint8_t kMiso = kDataBase + 1;                   // D1 = GPIO1
} // namespace

PglSpiTarget& PglSpiTargetInstance() {
    static PglSpiTarget instance;
    return instance;
}

// CS rising edge (core 0, shared raw handler): finalize the transaction.
// Bounded work only: stop SMs/DMA, capture count + ≤10 FIFO/residue words,
// release MISO, drop READY, publish one capture. No CRC/parsing here.
void PglSpiTarget::IrqTrampoline() {
    PglSpiTarget* self = g_irqInstance;
    if (!self) return;
    const uint32_t events = gpio_get_irq_event_mask(kCs);
    if (!(events & GPIO_IRQ_EDGE_RISE)) return;
    gpio_acknowledge_irq(kCs, GPIO_IRQ_EDGE_RISE);
    self->Finalize(false);
}

void PglSpiTarget::UpdateReady() {
    gpio_put(kReady, parentReady_ && transactionArmed_ && !quiesced_ && initialized_);
}

// Bounded stop. RP2350-E5: clear EN on every channel — including every member
// of the prefix→payload chain — before aborting any of them.
void PglSpiTarget::DisableSmsAndDma() {
    if (rxLease_.pio) pio_sm_set_enabled(rxLease_.pio, rxLease_.sm, false);
    if (txLease_.pio) pio_sm_set_enabled(txLease_.pio, txLease_.sm, false);
    if (rxPreDma_ >= 0) hw_clear_bits(&dma_hw->ch[rxPreDma_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    if (rxDma_ >= 0) hw_clear_bits(&dma_hw->ch[rxDma_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    if (txDma_ >= 0) hw_clear_bits(&dma_hw->ch[txDma_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    if (rxPreDma_ >= 0) { dma_channel_abort(rxPreDma_); dma_channel_cleanup(rxPreDma_); }
    if (rxDma_ >= 0) { dma_channel_abort(rxDma_); dma_channel_cleanup(rxDma_); }
    if (txDma_ >= 0) { dma_channel_abort(txDma_); dma_channel_cleanup(txDma_); }
}

void PglSpiTarget::Finalize(bool stalled) {
    {
        const uint32_t save = save_and_disable_interrupts();
        if (!transactionArmed_) {
            restore_interrupts(save);
            return;
        }
        transactionArmed_ = false;  // exactly one capture per transaction
        restore_interrupts(save);
    }
    if (stalled) ++stallAbortCount_;
    gpio_put(kReady, false);  // READY is never high while unprepared

    PglSpiTargetDetail::Capture cap{};
    cap.dstWasBulk = armedDstWasBulk_;

    PIO rxPio = rxLease_.pio;
    PIO txPio = txLease_.pio;
    const uint rxSm = rxLease_.sm;
    const uint txSm = txLease_.sm;
    pio_sm_set_enabled(rxPio, rxSm, false);
    pio_sm_set_enabled(txPio, txSm, false);

    // RP2350-E5: EN cleared on the chained prefix channel AND the payload/TX
    // channels before any abort; aborts are hardware-bounded safe points.
    hw_clear_bits(&dma_hw->ch[rxPreDma_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    hw_clear_bits(&dma_hw->ch[rxDma_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    hw_clear_bits(&dma_hw->ch[txDma_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    dma_channel_abort(rxPreDma_);
    dma_channel_abort(rxDma_);
    dma_channel_abort(txDma_);

    const uint32_t preRemaining = dma_hw->ch[rxPreDma_].transfer_count & DMA_CH0_TRANS_COUNT_COUNT_BITS;
    cap.prefixValid = preRemaining == 0;  // the 1-transfer prefix channel completed
    cap.prefix = prefixScratch_;
    const uint32_t remaining = dma_hw->ch[rxDma_].transfer_count & DMA_CH0_TRANS_COUNT_COUNT_BITS;
    cap.dmaBytes = armedRxCount_ - remaining;

    // Joined RX FIFO tail: ≤8 complete-byte words.
    while (cap.wordCount < 8 && !pio_sm_is_rx_fifo_empty(rxPio, rxSm))
        cap.words[cap.wordCount++] = static_cast<uint16_t>(pio_sm_get(rxPio, rxSm));
    // ISR residue: a complete byte blocked on a full FIFO, then the sentinel
    // residue word (0/1 clean, >1 partial bits). FIFO has space after draining.
    for (uint8_t i = 0; i < 2 && cap.wordCount < PglSpiTargetDetail::kMaxCapturedWords; ++i) {
        pio_sm_exec_wait_blocking(rxPio, rxSm, pio_encode_push(false, false));
        cap.words[cap.wordCount++] = static_cast<uint16_t>(pio_sm_get(rxPio, rxSm));
    }
    cap.truncated = !pio_sm_is_rx_fifo_empty(rxPio, rxSm);  // defensive; cannot happen at depth 8+2

    // TX SM read marker, then release MISO (driver owns the pad until here).
    cap.misoDriven = !pio_sm_is_rx_fifo_empty(txPio, txSm);
    while (!pio_sm_is_rx_fifo_empty(txPio, txSm)) (void)pio_sm_get(txPio, txSm);
    pio_sm_exec_wait_blocking(txPio, txSm, pio_encode_set(pio_pindirs, 0));

    const uint32_t save = save_and_disable_interrupts();
    if (capturePending_) {
        ++droppedCount_;  // host clocked a new transaction while one was held
    } else {
        capture_ = cap;             // body first, pending flag last; both sides
        capturePending_ = true;     // run under masked IRQs on one core
    }
    restore_interrupts(save);
}

// Prepare DMA/FIFOs/SMs for the next transaction, then let READY rise.
// Runs only with CS inactive and both SMs disabled.
void PglSpiTarget::Rearm() {
    PIO rxPio = rxLease_.pio;
    PIO txPio = txLease_.pio;
    const uint rxSm = rxLease_.sm;
    const uint txSm = txLease_.sm;

    dma_channel_cleanup(rxPreDma_);
    dma_channel_cleanup(rxDma_);
    dma_channel_cleanup(txDma_);

    // Stale words from an aborted transfer must never be sampled again.
    pio_sm_clear_fifos(rxPio, rxSm);
    pio_sm_clear_fifos(txPio, txSm);
    // Clears shift counters and the ISR; preserves PC/X/Y (Y stays 1).
    pio_sm_restart(rxPio, rxSm);
    pio_sm_restart(txPio, txSm);
    pio_sm_exec_wait_blocking(rxPio, rxSm, pio_encode_jmp(rxLease_.offset));
    pio_sm_exec_wait_blocking(txPio, txSm, pio_encode_jmp(txLease_.offset));

    // Patch the bulk dispatch while the SM is idle: reserved lane width, or
    // the non-committing drain when no bulk is armed.
    const uint8_t target = !reservation_.active ? PglSpiTargetDetail::kRxTargetDrain
                           : reservation_.lanes == 4 ? PglSpiTargetDetail::kRxTargetQuad
                                                     : PglSpiTargetDetail::kRxTargetSingle;
    rxPio->instr_mem[rxLease_.offset + PglSpiTargetDetail::kRxPatchIndex] =
        pio_encode_jmp(rxLease_.offset + target);

    // RX DMA chain: channel A takes exactly stream word 0 (the prefix) into
    // the driver-owned scratch, then chains to channel B which takes the
    // exact following-byte count into the armed destination (NORMAL mode, so
    // it can never write past the armed region; extra host clocks stall the
    // SM's blocking push instead). The prefix is never in the payload.
    armedDstWasBulk_ = reservation_.active;
    armedRxCount_ = PglSpiTargetDetail::PhysicalBodyBytes(
        reservation_.active ? reservation_.logicalBytes : 0);
    dma_channel_config pc = dma_channel_get_default_config(rxPreDma_);
    channel_config_set_transfer_data_size(&pc, DMA_SIZE_8);  // low 8 bits of each word
    channel_config_set_read_increment(&pc, false);
    channel_config_set_write_increment(&pc, false);
    channel_config_set_dreq(&pc, pio_get_dreq(rxPio, rxSm, false));
    channel_config_set_chain_to(&pc, rxDma_);
    dma_channel_configure(rxPreDma_, &pc, &prefixScratch_, &rxPio->rxf[rxSm], 1, false);

    dma_channel_config rc = dma_channel_get_default_config(rxDma_);
    channel_config_set_transfer_data_size(&rc, DMA_SIZE_8);
    channel_config_set_read_increment(&rc, false);
    channel_config_set_write_increment(&rc, true);
    channel_config_set_dreq(&rc, pio_get_dreq(rxPio, rxSm, false));
    channel_config_set_chain_to(&rc, rxDma_);  // self = no further chain
    dma_channel_configure(rxDma_, &rc,
                          reservation_.active ? reservation_.buffer : controlBuf_ + 1,
                          &rxPio->rxf[rxSm], armedRxCount_, false);
    dma_channel_start(rxPreDma_);  // B starts only when A's prefix transfer chains

    // TX DMA: immutable prepacked snapshot, whole 32-bit words, autopull 8.
    dma_channel_config tc = dma_channel_get_default_config(txDma_);
    channel_config_set_transfer_data_size(&tc, DMA_SIZE_32);
    channel_config_set_read_increment(&tc, true);
    channel_config_set_write_increment(&tc, false);
    channel_config_set_dreq(&tc, pio_get_dreq(txPio, txSm, true));
    dma_channel_configure(txDma_, &tc, &txPio->txf[txSm], snapshots_.Active(),
                          PglRuntime::RecordBytes, true);

    pio_sm_set_enabled(txPio, txSm, true);
    pio_sm_set_enabled(rxPio, rxSm, true);
    transactionArmed_ = true;
    UpdateReady();
}

PglRuntime::Result PglSpiTarget::Init(HardwareResources& resources) {
    if (initialized_) return PglRuntime::Result::BadState;

    gpioMask_ = (uint64_t(1) << 6) - 1;        // D0..D3, SCK, CS
    gpioMask_ |= uint64_t(1) << kReady;
    PglRuntime::Result rc = resources.ClaimGpios(gpioMask_, HardwareResources::Owner::Host);
    if (rc != PglRuntime::Result::Ok) return rc;
    bool rxOk = false, txOk = false;
    rc = resources.ClaimPio(PglSpiTargetDetail::kRxPioBlock, &pgl_spi_rx_program,
                            HardwareResources::Owner::Host, rxLease_);
    rxOk = rc == PglRuntime::Result::Ok;
    if (rxOk) {
        rc = resources.ClaimPio(PglSpiTargetDetail::kTxPioBlock, &pgl_spi_tx_program,
                                HardwareResources::Owner::Host, txLease_);
        txOk = rc == PglRuntime::Result::Ok;
    }
    if (txOk) rc = resources.ClaimDma(HardwareResources::Owner::Host, rxPreDma_);
    if (txOk && rc == PglRuntime::Result::Ok) rc = resources.ClaimDma(HardwareResources::Owner::Host, rxDma_);
    if (txOk && rc == PglRuntime::Result::Ok) rc = resources.ClaimDma(HardwareResources::Owner::Host, txDma_);
    if (rc != PglRuntime::Result::Ok) {  // central lease rollback
        if (rxPreDma_ >= 0) resources.ReleaseDma(rxPreDma_, HardwareResources::Owner::Host);
        if (rxDma_ >= 0) resources.ReleaseDma(rxDma_, HardwareResources::Owner::Host);
        if (txDma_ >= 0) resources.ReleaseDma(txDma_, HardwareResources::Owner::Host);
        if (txOk) resources.ReleasePio(txLease_);
        if (rxOk) resources.ReleasePio(rxLease_);
        resources.ReleaseGpios(gpioMask_, HardwareResources::Owner::Host);
        return rc;
    }

    // Program layout sanity: the idle-patched dispatch must be a JMP whose
    // targets are the single/quad byte loops and the drain loop.
    const uint16_t* prog = pgl_spi_rx_program.instructions;
    if (pgl_spi_rx_program.length != PglSpiTargetDetail::kRxProgramWords ||
        pgl_spi_tx_program.length != PglSpiTargetDetail::kTxProgramWords ||
        prog[PglSpiTargetDetail::kRxPatchIndex] != pio_encode_jmp(PglSpiTargetDetail::kRxTargetSingle) ||
        (prog[PglSpiTargetDetail::kRxTargetSingle] & 0xe000u) != 0x4000u ||   // in
        (prog[PglSpiTargetDetail::kRxTargetQuad] & 0xe000u) != 0x4000u ||     // in
        (prog[PglSpiTargetDetail::kRxTargetDrain] & 0xe000u) != 0x2000u) {    // wait
        resources.ReleaseDma(rxPreDma_, HardwareResources::Owner::Host);
        resources.ReleaseDma(rxDma_, HardwareResources::Owner::Host);
        resources.ReleaseDma(txDma_, HardwareResources::Owner::Host);
        resources.ReleasePio(txLease_);
        resources.ReleasePio(rxLease_);
        resources.ReleaseGpios(gpioMask_, HardwareResources::Owner::Host);
        return PglRuntime::Result::CorruptImage;
    }

    resources_ = &resources;

    // Pins: PIO1 owns D0/D2/D3/SCK/CS muxes, PIO2 owns D1 (MISO) mux; every
    // pad input remains visible to both blocks. CS idles high via the internal
    // pull-up (the board still needs the external pull-up prerequisite).
    for (uint8_t pin = kDataBase; pin <= kCs; ++pin) gpio_set_dir(pin, GPIO_IN);
    pio_gpio_init(rxLease_.pio, kDataBase + 0);
    pio_gpio_init(rxLease_.pio, kDataBase + 2);
    pio_gpio_init(rxLease_.pio, kDataBase + 3);
    pio_gpio_init(rxLease_.pio, kSck);
    pio_gpio_init(rxLease_.pio, kCs);
    pio_gpio_init(txLease_.pio, kMiso);
    gpio_pull_up(kCs);

    gpio_init(kReady);
    gpio_set_dir(kReady, GPIO_OUT);
    gpio_put(kReady, false);

    // RX SM: IN base D0, JMP pin D0, left shift (MSB first), manual push,
    // joined RX FIFO (8 words), clk_sys synchronous sampling (divider 1).
    pio_sm_config rc_cfg = pgl_spi_rx_program_get_default_config(rxLease_.offset);
    sm_config_set_in_pins(&rc_cfg, kDataBase);
    sm_config_set_jmp_pin(&rc_cfg, kDataBase);
    sm_config_set_in_shift(&rc_cfg, false, false, 32);
    sm_config_set_fifo_join(&rc_cfg, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&rc_cfg, 1.0f);
    pio_sm_init(rxLease_.pio, rxLease_.sm, rxLease_.offset, &rc_cfg);
    pio_sm_set_consecutive_pindirs(rxLease_.pio, rxLease_.sm, kDataBase, 6, false);
    pio_sm_exec_wait_blocking(rxLease_.pio, rxLease_.sm, pio_encode_set(pio_y, 1));  // sentinel source

    // TX SM: OUT/SET base D1 (count 1), JMP pin D0, left shift, autopull 8,
    // default 4+4 FIFOs (RX FIFO carries the per-read marker word).
    pio_sm_config tc_cfg = pgl_spi_tx_program_get_default_config(txLease_.offset);
    sm_config_set_out_pins(&tc_cfg, kMiso, 1);
    sm_config_set_set_pins(&tc_cfg, kMiso, 1);
    sm_config_set_jmp_pin(&tc_cfg, kDataBase);
    sm_config_set_out_shift(&tc_cfg, false, true, 8);
    sm_config_set_clkdiv(&tc_cfg, 1.0f);
    pio_sm_init(txLease_.pio, txLease_.sm, txLease_.offset, &tc_cfg);
    pio_sm_set_consecutive_pindirs(txLease_.pio, txLease_.sm, kMiso, 1, false);

    // Valid-CRC default snapshot: a truthful NotReady status until the parent
    // publishes its first record.
    PglRuntime::Record initial{};
    initial.info = PglRuntime::Info::Status;
    initial.result = PglRuntime::Result::NotReady;
    snapshots_.Publish(initial, false);

    g_irqInstance = this;
    gpio_add_raw_irq_handler(kCs, &PglSpiTarget::IrqTrampoline);
    gpio_set_irq_enabled(kCs, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    initialized_ = true;
    Rearm();  // READY stays low until SetReady(true): parentReady_ is false
    return PglRuntime::Result::Ok;
}

PglRuntime::Result PglSpiTarget::Arm(uint8_t* buffer, size_t capacity,
                                     uint32_t logicalBytes, uint8_t lanes) {
    if (!initialized_) return PglRuntime::Result::NotReady;
    if (quiesced_) return PglRuntime::Result::BadState;
    if (!buffer) return PglRuntime::Result::InvalidValue;
    if (reservation_.active) return PglRuntime::Result::Busy;
    // Safe window only: a held transaction (re-arm follows Release) or the
    // link not yet armed. Never reconfigures under a possibly-live CS.
    if (!heldTransaction_ && transactionArmed_) return PglRuntime::Result::Busy;
    const PglRuntime::Result rc = PglSpiTargetDetail::ValidateArm(capacity, logicalBytes, lanes);
    if (rc != PglRuntime::Result::Ok) return rc;
    reservation_.buffer = buffer;
    reservation_.capacity = capacity;
    reservation_.logicalBytes = logicalBytes;
    reservation_.lanes = lanes;
    reservation_.active = true;
    return PglRuntime::Result::Ok;
}

PglRuntime::Result PglSpiTarget::Disarm() {
    if (!initialized_) return PglRuntime::Result::NotReady;
    if (quiesced_) return PglRuntime::Result::BadState;
    if (!heldTransaction_ && transactionArmed_) return PglRuntime::Result::Busy;
    reservation_.active = false;
    return PglRuntime::Result::Ok;
}

PglRuntime::Result PglSpiTarget::PublishRecord(const PglRuntime::Record& record) {
    if (!initialized_) return PglRuntime::Result::NotReady;
    const bool live = capturePending_ || heldTransaction_ ||
                      (transactionArmed_ && !gpio_get(kCs));
    snapshots_.Publish(record, live);
    return PglRuntime::Result::Ok;
}

PglRuntime::Result PglSpiTarget::PopTransaction(RxTransaction& out) {
    if (!initialized_ || quiesced_) return PglRuntime::Result::NotReady;
    if (heldTransaction_) {
        out = heldView_;
        return PglRuntime::Result::Ok;
    }
    PglSpiTargetDetail::Capture cap;
    {
        const uint32_t save = save_and_disable_interrupts();
        if (!capturePending_) {
            restore_interrupts(save);
            return PglRuntime::Result::Pending;
        }
        cap = capture_;
        capturePending_ = false;
        restore_interrupts(save);
    }
    snapshots_.EndTransaction();  // the link is idle: activate a pending snapshot

    // The prefix went to the driver scratch; following bytes went to the
    // armed destination (control body starts at controlBuf_[1]).
    uint8_t* dst = cap.dstWasBulk ? reservation_.buffer : controlBuf_ + 1;
    const uint32_t limit = PglSpiTargetDetail::PhysicalBodyBytes(
        cap.dstWasBulk ? reservation_.logicalBytes : 0);
    const PglSpiTargetDetail::AbsorbResult ab = PglSpiTargetDetail::AbsorbCapturedBytes(
        cap, dst, limit, excessBuf_, PglSpiTargetDetail::kMaxCapturedWords);
    const PglSpiTargetDetail::Decision d = PglSpiTargetDetail::DecideTransaction(
        ab, cap.prefixValid, cap.prefix, cap.dstWasBulk, reservation_.logicalBytes);
    const PglSpiTargetDetail::CommitView view = PglSpiTargetDetail::MakeCommitView(d);
    heldView_ = RxTransaction{};
    heldView_.kind = view.kind;  // authoritative: wire prefix, never payload contents
    heldView_.overflow = view.overflow;
    heldView_.partial = view.partial;
    switch (view.kind) {
        case PglSpiTargetDetail::TransactionKind::Control:
            controlBuf_[0] = PglRuntime::WriteControl;  // verified captured prefix
            if (cap.dstWasBulk) {  // control while armed: reassemble body from dst + tail
                PglSpiTargetDetail::GatherReceived(controlBuf_ + 1, PglRuntime::ControlBytes - 1, dst,
                                                   ab.placedBytes, excessBuf_, ab.excessCount);
            }
            heldView_.bytes = controlBuf_;
            heldView_.length = view.length;
            break;
        case PglSpiTargetDetail::TransactionKind::Bulk:
            heldView_.bytes = reservation_.buffer;
            heldView_.length = view.length;
            reservation_.active = false;  // consumed
            break;
        case PglSpiTargetDetail::TransactionKind::Rejected:
            ++rejectedCount_;
            break;
        case PglSpiTargetDetail::TransactionKind::Read:
        case PglSpiTargetDetail::TransactionKind::Empty:
            break;
    }
    heldTransaction_ = true;
    out = heldView_;
    return PglRuntime::Result::Ok;
}

PglRuntime::Result PglSpiTarget::ReleaseTransaction() {
    if (!initialized_ || quiesced_) return PglRuntime::Result::NotReady;
    if (!heldTransaction_) return PglRuntime::Result::BadState;
    heldTransaction_ = false;
    {
        // A host that clocked without READY left a stale capture; drop it
        // rather than attributing it to the freshly armed configuration.
        const uint32_t save = save_and_disable_interrupts();
        if (capturePending_) {
            capturePending_ = false;
            ++droppedCount_;
        }
        restore_interrupts(save);
    }
    Rearm();
    return PglRuntime::Result::Ok;
}

PglRuntime::Result PglSpiTarget::Quiesce() {
    if (!initialized_) return PglRuntime::Result::BadState;
    if (quiesced_) return PglRuntime::Result::Ok;
    if (transactionArmed_ && !gpio_get(kCs)) return PglRuntime::Result::Busy;   // live CS
    if (heldTransaction_ || capturePending_) return PglRuntime::Result::Busy;
    if (reservation_.active) return PglRuntime::Result::Busy;                   // armed bulk
    gpio_put(kReady, false);
    DisableSmsAndDma();
    transactionArmed_ = false;
    quiesced_ = true;
    return PglRuntime::Result::Ok;
}

PglRuntime::Result PglSpiTarget::Resume(uint32_t systemClockHz) {
    if (!initialized_) return PglRuntime::Result::BadState;
    if (!quiesced_) return PglRuntime::Result::BadState;
    if (!systemClockHz || clock_get_hz(clk_sys) != systemClockHz)
        return PglRuntime::Result::InvalidValue;
    // Divider stays 1.0 across all profiles: both programs run from clk_sys.
    // The initial 1 MHz host rate leaves >=37.5 sys cycles per half-period
    // even at 75 MHz; no faster externally qualified rate is implied.
    // Restart divider phase while the SMs are idle.
    pio_sm_clkdiv_restart(rxLease_.pio, rxLease_.sm);
    pio_sm_clkdiv_restart(txLease_.pio, txLease_.sm);
    quiesced_ = false;
    Rearm();
    return PglRuntime::Result::Ok;
}

void PglSpiTarget::SetReady(bool ready) {
    parentReady_ = ready;
    if (initialized_) UpdateReady();
}

void PglSpiTarget::Poll(uint64_t nowUs) {
    if (!initialized_ || quiesced_ || !transactionArmed_) return;
    if (gpio_get(kCs)) {
        csLowSeen_ = false;
        return;
    }
    if (!csLowSeen_) {
        csLowSeen_ = true;
        csLowSinceUs_ = nowUs;
        return;
    }
    // Bounded expiry: a clock-stopped transaction aborts into the same
    // capture path as a CS rise and is rejected/finalized deterministically.
    if (nowUs - csLowSinceUs_ > PglSpiTargetDetail::kStallTimeoutUs) {
        csLowSeen_ = false;
        Finalize(true);
    }
}

void PglSpiTarget::Shutdown() {
    if (!initialized_) return;
    gpio_put(kReady, false);
    gpio_set_irq_enabled(kCs, GPIO_IRQ_EDGE_RISE, false);
    gpio_remove_raw_irq_handler(kCs, &PglSpiTarget::IrqTrampoline);
    g_irqInstance = nullptr;
    DisableSmsAndDma();
    resources_->ReleaseDma(rxPreDma_, HardwareResources::Owner::Host);
    resources_->ReleaseDma(rxDma_, HardwareResources::Owner::Host);
    resources_->ReleaseDma(txDma_, HardwareResources::Owner::Host);
    resources_->ReleasePio(rxLease_);
    resources_->ReleasePio(txLease_);
    resources_->ReleaseGpios(gpioMask_, HardwareResources::Owner::Host);
    // Pads to a safe electrical state: inputs, CS pull-up retained.
    for (uint8_t pin = kDataBase; pin <= kCs; ++pin) {
        gpio_set_function(pin, GPIO_FUNC_SIO);
        gpio_set_dir(pin, GPIO_IN);
    }
    gpio_set_function(kReady, GPIO_FUNC_SIO);
    gpio_set_dir(kReady, GPIO_IN);
    resources_ = nullptr;
    reservation_ = Reservation{};
    capturePending_ = false;
    transactionArmed_ = false;
    heldTransaction_ = false;
    parentReady_ = false;
    quiesced_ = false;
    initialized_ = false;
}

#endif // PICO_ON_DEVICE
