/**
 * @file hub75_driver.cpp
 * @brief HUB75 backend — autonomous whole-scan PIO/DMA engine (P07).
 *
 * Ownership model (docs plan §display-buffer ownership):
 *   RGB source     : FREE -> BORROWED(Present) -> CONVERTING -> released
 *   Encoded scan   : FREE -> BUILDING -> ARMED -> ACTIVE (repeats) -> FREE
 *
 * The engine scans the ACTIVE bank continuously with no per-record CPU work:
 * the data SM paces from a DREQ-driven DMA channel, the row SM handshakes
 * data-ready/latch via PIO IRQ flags 1/2, and one whole-scan-complete flag
 * (0) reaches the CPU, whose shared handler re-arms both DMA channels and
 * applies armed bank switches at that exact gate.  Flag 3 is the quiesce
 * park request (IRQ_FORCE): the row SM stalls blank at the scan boundary so
 * ClockQuiesce gates with OE high; Resume() recomputes all dividers/dwells.
 */

#include "hub75_driver.h"

#if defined(PICO_ON_DEVICE)

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/time.h"

#include "hub75.pio.h"

#include "hardware/sync.h"
#include "pico/stdlib.h"
#include <cstring>
#include <new>

namespace {
Hub75PanelDriver* g_hub75Instance = nullptr;

/// Pico2-safe HUB75 pin group: RGB 6..11, CLK 12, LAT 13, OE 14, ADDR 15..19.
constexpr uint64_t kGpioMask =
    (uint64_t(0x3fff) << GpuConfig::HUB75_R1_PIN);  // pins 6..19

/// Bounded wait for one scan boundary during Quiesce (worst-case scan at the
/// lowest supported clock is a few ms; 20 ms is generous but finite).
constexpr uint32_t kQuiesceTimeoutUs = 20000;
} // namespace

// ─── IRQ ────────────────────────────────────────────────────────────────────

void Hub75PanelDriver::ScanIrqHandler() {
    Hub75PanelDriver* self = g_hub75Instance;
    if (!self || !self->dataLease_.pio) return;
    // Shared handler: acknowledge only our leased flag, never others.
    if (!pio_interrupt_get(self->dataLease_.pio, 0)) return;
    pio_interrupt_clear(self->dataLease_.pio, 0);
    self->OnScanComplete();
}

void Hub75PanelDriver::OnScanComplete() {
    const uint64_t now = time_us_64();
    lastScanUs_ = now;
    ++scanCount_;
    if (faulted_ || quiesced_) return;  // DMA/SMs stopped; nothing to re-arm

    // ── Whole-scan gate ──────────────────────────────────────────────────
    // The data channel completed its count and the data SM consumed every
    // byte of the finished scan (it now stalls on autopull with CLK low);
    // the row SM consumed all 256 control words.  Neither a prefetched FIFO
    // word nor a DMA descriptor can reference the retiring bank after this
    // point, so the switch below can never mix rows/planes of two frames.
    // Completion belongs to the scan that just finished, not the new bank
    // about to start. Keep the source lease until every new row was latched.
    if (phase_ == Phase::Scanning) {
        PushEvent(PglRuntime::Completion::Displayed, PglRuntime::Result::Ok, now, true);
        phase_ = Phase::Idle;
    }
    uint8_t dataBank = activeDataBank_;
    if (armedDataBank_ != 0xff) {
        dataBank = armedDataBank_;
        armedDataBank_ = 0xff;
        phase_ = Phase::Scanning;
    }
    uint8_t ctrlBank = activeCtrlBank_;
    if (armedCtrlBank_ != 0xff) {
        ctrlBank = armedCtrlBank_;
        armedCtrlBank_ = 0xff;
    }
    activeDataBank_ = dataBank;
    activeCtrlBank_ = ctrlBank;

    // Re-arm both channels for the next scan (bounded, constant work).
    dma_channel_set_read_addr(dmaData_, DataBank(dataBank), false);
    dma_channel_set_trans_count(dmaData_, scanBytes_, true);
    dma_channel_set_read_addr(dmaCtrl_, CtrlBank(ctrlBank), false);
    dma_channel_set_trans_count(dmaCtrl_, Hub75::kRecordCount, true);
    ++totalScans_;

}

// ─── Helpers ────────────────────────────────────────────────────────────────

void Hub75PanelDriver::PushEvent(PglRuntime::Completion completion,
                                 PglRuntime::Result result, uint64_t timestampUs,
                                 bool sourceReleased) {
    const uint32_t irqState = save_and_disable_interrupts();  // main+ISR safe
    const uint8_t next = uint8_t((eventHead_ + 1) & 3u);
    if (next == eventTail_) eventTail_ = uint8_t((eventTail_ + 1) & 3u);  // drop oldest
    DisplayEvent& e = events_[eventHead_];
    e.frame = pendingFrame_;
    e.completion = completion;
    e.result = result;
    e.timestampUs = timestampUs;
    e.sourceReleased = sourceReleased;
    eventHead_ = next;
    restore_interrupts(irqState);
}

void Hub75PanelDriver::ForceBlank() {
    faulted_ = true;
    PIO pio = dataLease_.pio;
    if (!pio) return;
    const uint32_t irqState = save_and_disable_interrupts();
    pio_set_irqn_source_enabled(pio, 0, pis_interrupt0, false);
    if (dmaData_ >= 0) hw_clear_bits(&dma_hw->ch[dmaData_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    if (dmaCtrl_ >= 0) hw_clear_bits(&dma_hw->ch[dmaCtrl_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    if (dmaData_ >= 0) dma_channel_abort(dmaData_);
    if (dmaCtrl_ >= 0) dma_channel_abort(dmaCtrl_);
    pio_sm_set_enabled(pio, dataLease_.sm, false);
    pio_sm_set_enabled(pio, rowLease_.sm, false);
    // Abort must blank independently of stalled SM execution: the frozen SM
    // pin levels are untrusted, so drive OE high / CLK / LAT low via SIO.
    gpio_set_function(GpuConfig::HUB75_OE_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(GpuConfig::HUB75_OE_PIN, GPIO_OUT);
    gpio_put(GpuConfig::HUB75_OE_PIN, 1);
    gpio_set_function(GpuConfig::HUB75_CLK_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(GpuConfig::HUB75_CLK_PIN, GPIO_OUT);
    gpio_put(GpuConfig::HUB75_CLK_PIN, 0);
    gpio_set_function(GpuConfig::HUB75_LAT_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(GpuConfig::HUB75_LAT_PIN, GPIO_OUT);
    gpio_put(GpuConfig::HUB75_LAT_PIN, 0);
    restore_interrupts(irqState);
}

void Hub75PanelDriver::ReleaseAllClaims() {
    if (!resources_) return;
    if (dmaData_ >= 0) hw_clear_bits(&dma_hw->ch[dmaData_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    if (dmaCtrl_ >= 0) hw_clear_bits(&dma_hw->ch[dmaCtrl_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    if (dataLease_.pio) {
        pio_interrupt_clear(dataLease_.pio, 0);
        pio_interrupt_clear(dataLease_.pio, 1);
        pio_interrupt_clear(dataLease_.pio, 2);
        pio_interrupt_clear(dataLease_.pio, 3);
    }
    if (dmaData_ >= 0) resources_->ReleaseDma(dmaData_, HardwareResources::Owner::Display);
    if (dmaCtrl_ >= 0) resources_->ReleaseDma(dmaCtrl_, HardwareResources::Owner::Display);
    resources_->ReleasePioIrqs(pioBlock_, kIrqFlagMask, HardwareResources::Owner::Display);
    resources_->ReleasePio(dataLease_);
    resources_->ReleasePio(rowLease_);
    resources_->ReleaseGpios(kGpioMask, HardwareResources::Owner::Display);
}

// ─── Init / Shutdown ────────────────────────────────────────────────────────

PglRuntime::Result Hub75PanelDriver::Init(HardwareResources& resources,
                                          const DisplayConfig& config,
                                          void* workspace, size_t bytes) {
    using Result = PglRuntime::Result;
    using Owner = HardwareResources::Owner;
    if (inited_) return Result::BadState;
    if (!Hub75::SupportedWidth(config.width) || config.height != Hub75::kPanelHeight)
        return Result::Unsupported;
    if (!workspace || (reinterpret_cast<uintptr_t>(workspace) & 3u)) return Result::InvalidValue;
    if (config.type != DisplayType::Hub75) return Result::InvalidValue;
    if (bytes < Hub75::RequiredWorkspaceBytes(config.width)) return Result::Capacity;

    resources_ = &resources;
    config_ = config;
    brightness_ = config.brightness;
    scanBytes_ = Hub75::ScanDataBytes(config.width);
    dataBanks_ = static_cast<uint8_t*>(workspace);
    ctrlBanks_ = dataBanks_ + 2 * scanBytes_;
    clockHz_ = clock_get_hz(clk_sys);

    // ── Dynamic leases with full rollback on any failure ────────────────
    Result r = resources_->ClaimGpios(kGpioMask, Owner::Display);
    if (r != Result::Ok) { resources_ = nullptr; return r; }
    gpio_init(GpuConfig::HUB75_OE_PIN);
    gpio_put(GpuConfig::HUB75_OE_PIN, 1);
    gpio_set_dir(GpuConfig::HUB75_OE_PIN, GPIO_OUT);
    r = resources_->ClaimPioIrqs(pioBlock_, kIrqFlagMask, Owner::Display);
    if (r != Result::Ok) { ReleaseAllClaims(); resources_ = nullptr; return r; }
    r = resources_->ClaimPio(pioBlock_, &hub75_data_program, Owner::Display, dataLease_);
    if (r != Result::Ok) { ReleaseAllClaims(); resources_ = nullptr; return r; }
    r = resources_->ClaimPio(pioBlock_, &hub75_row_program, Owner::Display, rowLease_);
    if (r != Result::Ok) { ReleaseAllClaims(); resources_ = nullptr; return r; }
    if (dataLease_.pio != rowLease_.pio) {  // both SMs must share one block
        ReleaseAllClaims(); resources_ = nullptr; return Result::Capacity;
    }
    r = resources_->ClaimDma(Owner::Display, dmaData_);
    if (r != Result::Ok) { ReleaseAllClaims(); resources_ = nullptr; return r; }
    r = resources_->ClaimDma(Owner::Display, dmaCtrl_);
    if (r != Result::Ok) { ReleaseAllClaims(); resources_ = nullptr; return r; }

    PIO pio = dataLease_.pio;

    // Immutable initial content: black data banks, current-brightness dwells.
    std::memset(dataBanks_, 0, 2 * scanBytes_);
    Hub75::EncodeControlBank(CtrlBank(0), brightness_, clockHz_);
    Hub75::EncodeControlBank(CtrlBank(1), brightness_, clockHz_);

    // ── Data SM: byte stream -> RGB pins, CLK side-set ──────────────────
    {
        pio_sm_config c = hub75_data_program_get_default_config(dataLease_.offset);
        sm_config_set_wrap(&c, dataLease_.offset + hub75_data_wrap_target,
                           dataLease_.offset + hub75_data_wrap);
        sm_config_set_sideset_pins(&c, GpuConfig::HUB75_CLK_PIN);
        sm_config_set_out_pins(&c, GpuConfig::HUB75_R1_PIN, 6);
        sm_config_set_set_pins(&c, GpuConfig::HUB75_CLK_PIN, 1);
        sm_config_set_out_shift(&c, true, true, 6);  // right, autopull threshold 6
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        const uint32_t div = Hub75::PixelClockDivFrac8(clockHz_);
        sm_config_set_clkdiv_int_frac8(&c, uint16_t(div >> 8), uint8_t(div & 0xff));
        pio_sm_init(pio, dataLease_.sm, dataLease_.offset, &c);
        pio_sm_set_pins_with_mask(pio, dataLease_.sm, 0,
                                 0x7fu << GpuConfig::HUB75_R1_PIN);
        pio_sm_set_consecutive_pindirs(pio, dataLease_.sm, GpuConfig::HUB75_R1_PIN, 7, true);
    }
    // ── Row SM: address/latch/OE ─────────────────────────────────────────
    {
        pio_sm_config c = hub75_row_program_get_default_config(rowLease_.offset);
        sm_config_set_wrap(&c, rowLease_.offset + hub75_row_wrap_target,
                           rowLease_.offset + hub75_row_wrap);
        sm_config_set_sideset_pins(&c, GpuConfig::HUB75_LAT_PIN);  // 2 pins: LAT, OE
        sm_config_set_out_pins(&c, GpuConfig::HUB75_ADDR_A, 5);
        sm_config_set_set_pins(&c, GpuConfig::HUB75_LAT_PIN, 2);
        sm_config_set_out_shift(&c, true, false, 32);  // right, explicit pull
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&c, 1.0f);                // dwell counts = clk_sys cycles
        pio_sm_init(pio, rowLease_.sm, rowLease_.offset, &c);
        // PIO's own OE value is blank BEFORE its outputs are connected.
        pio_sm_set_pins_with_mask(pio, rowLease_.sm, 1u << GpuConfig::HUB75_OE_PIN,
                                 (3u << GpuConfig::HUB75_LAT_PIN) |
                                 (31u << GpuConfig::HUB75_ADDR_A));
        pio_sm_set_consecutive_pindirs(pio, rowLease_.sm, GpuConfig::HUB75_LAT_PIN, 7, true);
    }
    // Only connect the pads once each SM's own output state is safely set.
    for (uint8_t pin = GpuConfig::HUB75_R1_PIN; pin <= GpuConfig::HUB75_ADDR_E; ++pin)
        pio_gpio_init(pio, pin);

    pio_sm_clear_fifos(pio, dataLease_.sm);
    pio_sm_clear_fifos(pio, rowLease_.sm);
    for (uint8_t f = 0; f < 4; ++f) pio_interrupt_clear(pio, f);

    // Preload count headers (consumed once by each program's entry pull).
    pio_sm_put(pio, dataLease_.sm, config_.width - 1);        // pixels-1
    pio_sm_put(pio, rowLease_.sm, Hub75::kRecordCount - 1);   // records-1 = 255

    // ── DMA: byte data stream + 32-bit control stream, DREQ-paced ───────
    {
        dma_channel_config c = dma_channel_get_default_config(dmaData_);
        channel_config_set_chain_to(&c, dmaData_);   // self = chaining disabled
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
        channel_config_set_dreq(&c, pio_get_dreq(pio, dataLease_.sm, true));
        dma_channel_configure(dmaData_, &c, &pio->txf[dataLease_.sm],
                              DataBank(0), scanBytes_, false);
    }
    {
        dma_channel_config c = dma_channel_get_default_config(dmaCtrl_);
        channel_config_set_chain_to(&c, dmaCtrl_);   // self = chaining disabled
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_dreq(&c, pio_get_dreq(pio, rowLease_.sm, true));
        dma_channel_configure(dmaCtrl_, &c, &pio->txf[rowLease_.sm],
                              CtrlBank(0), Hub75::kRecordCount, false);
    }

    // ── Shared CPU handler for the whole-scan-complete flag ─────────────
    g_hub75Instance = this;
    const uint irqNum = pio_get_irq_num(pio, 0);
    irq_add_shared_handler(irqNum, &Hub75PanelDriver::ScanIrqHandler,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(irqNum, true);
    pio_set_irqn_source_enabled(pio, 0, pis_interrupt0, true);

    inited_ = true;
    quiesced_ = false;
    faulted_ = false;
    phase_ = Phase::Idle;
    activeDataBank_ = activeCtrlBank_ = 0;
    armedDataBank_ = armedCtrlBank_ = 0xff;
    eventHead_ = eventTail_ = 0;

    // DMA first so the FIFOs fill, then both SMs phase-aligned.
    dma_start_channel_mask((1u << dmaData_) | (1u << dmaCtrl_));
    pio_enable_sm_mask_in_sync(pio, (1u << dataLease_.sm) | (1u << rowLease_.sm));
    lastScanUs_ = time_us_64();
    return Result::Ok;
}

void Hub75PanelDriver::Shutdown() {
    if (!inited_) return;
    Quiesce();
    PIO pio = dataLease_.pio;
    irq_remove_handler(pio_get_irq_num(pio, 0), &Hub75PanelDriver::ScanIrqHandler);
    g_hub75Instance = nullptr;
    if (sampler_) { sampler_->~DisplayPixelSampler(); sampler_ = nullptr; }
    const bool borrowed = phase_ != Phase::Idle;
    phase_ = Phase::Idle;
    if (borrowed)
        PushEvent(PglRuntime::Completion::Cancelled, PglRuntime::Result::Cancelled,
                  time_us_64(), true);
    ReleaseAllClaims();
    resources_ = nullptr;
    dataBanks_ = ctrlBanks_ = nullptr;
    inited_ = quiesced_ = faulted_ = false;
    scanCount_ = 0;
}

// ─── Present / conversion ───────────────────────────────────────────────────

PglRuntime::Result Hub75PanelDriver::Present(const DisplaySurface& surface,
                                             const DisplayMapping& mapping) {
    using Result = PglRuntime::Result;
    if (!inited_) return Result::NotReady;
    if (quiesced_ || faulted_) return Result::BadState;
    if (phase_ != Phase::Idle || eventHead_ != eventTail_) return Result::Busy;

    surface_ = surface;
    mapping_ = mapping;
    // One sampler, validated once for the whole conversion.
    sampler_ = new (samplerStorage_) DisplayPixelSampler(surface_, mapping_, config_);
    if (!sampler_->Valid()) {
        sampler_->~DisplayPixelSampler();
        sampler_ = nullptr;
        return Result::InvalidValue;
    }
    cursor_ = Hub75::EncodeCursor{};
    pendingFrame_ = surface.frame;
    phase_ = Phase::Encoding;
    return Result::Ok;
}

void Hub75PanelDriver::PollRefresh() {
    if (!inited_ || quiesced_ || faulted_) return;

    if (phase_ == Phase::Encoding) {
        // Bounded conversion slice: kEncodeSliceRecords records (<= 1 KiB)
        // per call; no per-plane CPU dwell, no blocking.
        const uint8_t bank = activeDataBank_ ^ 1;
        Hub75::EncodeSlice(*sampler_, config_.width, DataBank(bank), cursor_,
                           kEncodeSliceRecords);
        if (cursor_.Done()) {
            sampler_->~DisplayPixelSampler();
            sampler_ = nullptr;             // RGB source fully consumed here
            const uint32_t irqState = save_and_disable_interrupts();
            phase_ = Phase::Armed;
            __dmb();                        // publish fully written immutable bank
            armedDataBank_ = bank;
            restore_interrupts(irqState);
        }
    }

    // Watchdog: a stalled scan must blank, never hold a lit row.
    const uint32_t irqState = save_and_disable_interrupts();
    const uint64_t lastScan = lastScanUs_;
    restore_interrupts(irqState);
    if (time_us_64() - lastScan > kScanWatchdogUs) {
        ++forcedBlanks_;
        ForceBlank();
        armedDataBank_ = 0xff;
        if (phase_ == Phase::Armed || phase_ == Phase::Scanning) {
            PushEvent(PglRuntime::Completion::Failed, PglRuntime::Result::Io,
                      time_us_64(), true);
            phase_ = Phase::Idle;
        } else if (phase_ == Phase::Encoding) {
            if (sampler_) { sampler_->~DisplayPixelSampler(); sampler_ = nullptr; }
            PushEvent(PglRuntime::Completion::Failed, PglRuntime::Result::Io,
                      time_us_64(), true);
            phase_ = Phase::Idle;
        }
    }
}

bool Hub75PanelDriver::PopEvent(DisplayEvent& event) {
    const uint32_t irqState = save_and_disable_interrupts();
    const bool available = eventTail_ != eventHead_;
    if (available) {
        event = events_[eventTail_];
        eventTail_ = uint8_t((eventTail_ + 1) & 3u);
    }
    restore_interrupts(irqState);
    return available;
}

// ─── Clock quiesce / resume ─────────────────────────────────────────────────

PglRuntime::Result Hub75PanelDriver::Quiesce() {
    using Result = PglRuntime::Result;
    if (!inited_) return Result::BadState;
    if (quiesced_) return Result::Ok;

    PIO pio = dataLease_.pio;
    // Park request: the row SM stalls at WAIT 0 IRQ 3 (side 2 = OE high) at
    // the next whole-scan boundary and cannot advance past it, so the OE
    // gate is race-free even if this context is preempted afterwards.
    const uint32_t irqState = save_and_disable_interrupts();
    const uint32_t startScan = scanCount_;
    pio->irq_force = (1u << 3);
    restore_interrupts(irqState);
    const absolute_time_t deadline = make_timeout_time_us(kQuiesceTimeoutUs);
    while (scanCount_ == startScan) {
        if (time_reached(deadline)) {
            ++forcedBlanks_;
            ForceBlank();        // fault path: abort + SIO blank
            quiesced_ = true;
            return Result::Timeout;  // safe blank is not a successful scan drain
        }
        tight_loop_contents();
    }
    pio_set_irqn_source_enabled(pio, 0, pis_interrupt0, false);
    hw_clear_bits(&dma_hw->ch[dmaData_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    hw_clear_bits(&dma_hw->ch[dmaCtrl_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    dma_channel_abort(dmaData_);
    dma_channel_abort(dmaCtrl_);
    pio_sm_set_enabled(pio, dataLease_.sm, false);
    pio_sm_set_enabled(pio, rowLease_.sm, false);
    pio_interrupt_clear(pio, 0);
    quiesced_ = true;
    return Result::Ok;
}

PglRuntime::Result Hub75PanelDriver::Resume(uint32_t systemClockHz) {
    using Result = PglRuntime::Result;
    if (!inited_) return Result::BadState;
    if (!quiesced_ && !faulted_) return Result::BadState;  // Quiesce() must gate first
    const uint32_t divider = Hub75::PixelClockDivFrac8(systemClockHz);
    if (!systemClockHz || divider < 256u || divider > 0xffffffu)
        return Result::InvalidValue;

    clockHz_ = systemClockHz;
    PIO pio = dataLease_.pio;

    // If a fault blank drove pins via SIO, hand them back to the PIO block.
    for (uint8_t f = 0; f < 4; ++f) pio_interrupt_clear(pio, f);

    // Fresh SM state (PC at program offset) with recomputed dividers.
    {
        pio_sm_config c = hub75_data_program_get_default_config(dataLease_.offset);
        sm_config_set_wrap(&c, dataLease_.offset + hub75_data_wrap_target,
                           dataLease_.offset + hub75_data_wrap);
        sm_config_set_sideset_pins(&c, GpuConfig::HUB75_CLK_PIN);
        sm_config_set_out_pins(&c, GpuConfig::HUB75_R1_PIN, 6);
        sm_config_set_set_pins(&c, GpuConfig::HUB75_CLK_PIN, 1);
        sm_config_set_out_shift(&c, true, true, 6);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        const uint32_t div = Hub75::PixelClockDivFrac8(systemClockHz);
        sm_config_set_clkdiv_int_frac8(&c, uint16_t(div >> 8), uint8_t(div & 0xff));
        pio_sm_init(pio, dataLease_.sm, dataLease_.offset, &c);
        pio_sm_set_pins_with_mask(pio, dataLease_.sm, 0,
                                 0x7fu << GpuConfig::HUB75_R1_PIN);
        pio_sm_set_consecutive_pindirs(pio, dataLease_.sm, GpuConfig::HUB75_R1_PIN, 7, true);
    }
    {
        pio_sm_config c = hub75_row_program_get_default_config(rowLease_.offset);
        sm_config_set_wrap(&c, rowLease_.offset + hub75_row_wrap_target,
                           rowLease_.offset + hub75_row_wrap);
        sm_config_set_sideset_pins(&c, GpuConfig::HUB75_LAT_PIN);
        sm_config_set_out_pins(&c, GpuConfig::HUB75_ADDR_A, 5);
        sm_config_set_set_pins(&c, GpuConfig::HUB75_LAT_PIN, 2);
        sm_config_set_out_shift(&c, true, false, 32);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&c, 1.0f);
        pio_sm_init(pio, rowLease_.sm, rowLease_.offset, &c);
        pio_sm_set_pins_with_mask(pio, rowLease_.sm, 1u << GpuConfig::HUB75_OE_PIN,
                                 (3u << GpuConfig::HUB75_LAT_PIN) |
                                 (31u << GpuConfig::HUB75_ADDR_A));
        pio_sm_set_consecutive_pindirs(pio, rowLease_.sm, GpuConfig::HUB75_LAT_PIN, 7, true);
    }
    for (uint8_t pin = GpuConfig::HUB75_R1_PIN; pin <= GpuConfig::HUB75_ADDR_E; ++pin)
        pio_gpio_init(pio, pin);

    // Dwell banks are content-free plane weights: rebuild both at the new
    // clock (data banks are clock-independent and stay valid).
    Hub75::EncodeControlBank(CtrlBank(0), brightness_, systemClockHz);
    Hub75::EncodeControlBank(CtrlBank(1), brightness_, systemClockHz);

    pio_sm_clear_fifos(pio, dataLease_.sm);
    pio_sm_clear_fifos(pio, rowLease_.sm);
    pio_sm_put(pio, dataLease_.sm, config_.width - 1);
    pio_sm_put(pio, rowLease_.sm, Hub75::kRecordCount - 1);

    {
        dma_channel_config c = dma_channel_get_default_config(dmaData_);
        channel_config_set_chain_to(&c, dmaData_);   // self = chaining disabled
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
        channel_config_set_dreq(&c, pio_get_dreq(pio, dataLease_.sm, true));
        dma_channel_configure(dmaData_, &c, &pio->txf[dataLease_.sm],
                              DataBank(activeDataBank_), scanBytes_, false);
    }
    {
        dma_channel_config c = dma_channel_get_default_config(dmaCtrl_);
        channel_config_set_chain_to(&c, dmaCtrl_);   // self = chaining disabled
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_dreq(&c, pio_get_dreq(pio, rowLease_.sm, true));
        dma_channel_configure(dmaCtrl_, &c, &pio->txf[rowLease_.sm],
                              CtrlBank(activeCtrlBank_), Hub75::kRecordCount, false);
    }

    faulted_ = false;
    quiesced_ = false;
    pio_set_irqn_source_enabled(pio, 0, pis_interrupt0, true);
    dma_start_channel_mask((1u << dmaData_) | (1u << dmaCtrl_));
    pio_enable_sm_mask_in_sync(pio, (1u << dataLease_.sm) | (1u << rowLease_.sm));
    lastScanUs_ = time_us_64();
    return Result::Ok;
}

// ─── Brightness / caps ──────────────────────────────────────────────────────

PglRuntime::Result Hub75PanelDriver::SetBrightness(uint8_t brightness) {
    using Result = PglRuntime::Result;
    if (!inited_) return Result::BadState;
    const uint32_t irqState = save_and_disable_interrupts();
    brightness_ = brightness;
    if (!quiesced_ && !faulted_) {
        // An already armed bank may become active in the ISR. Protect the
        // bounded rebuild and publication together, including repeated calls.
        const uint8_t freeBank = activeCtrlBank_ ^ 1;
        Hub75::EncodeControlBank(CtrlBank(freeBank), brightness_, clockHz_);
        __dmb();
        armedCtrlBank_ = freeBank;
    }
    restore_interrupts(irqState);
    return Result::Ok;
}

DisplayCapabilities Hub75PanelDriver::GetCaps() const {
    DisplayCapabilities caps = {};
    caps.type = DisplayType::Hub75;
    if (inited_) {   // disabled: zero resource claims
        caps.width = config_.width;
        caps.height = config_.height;
        caps.displayCompletion = true;   // whole-scan activation is observed
        caps.pioStateMachines = 2;
        caps.dmaChannels = 2;
        caps.pioInstructions = 22;       // hub75_data 9 + hub75_row 13
        caps.workspaceBytes = Hub75::RequiredWorkspaceBytes(config_.width);
    }
    return caps;
}

// ─── Factory ────────────────────────────────────────────────────────────────

DisplayDriver& Hub75Backend() {
    static Hub75PanelDriver instance;
    return instance;
}

#endif // PICO_ON_DEVICE
