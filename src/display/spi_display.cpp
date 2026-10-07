/**
 * @file spi_display.cpp
 * @brief PIO+DMA SSD1331 96x64 RGB565 OLED backend (P07).
 *
 * Transfer integrity rules implemented here:
 *   - command arguments always travel with DC low; only GDDRAM pixel data
 *     is sent with DC high;
 *   - every CS/DC change is separated from SCLK activity by >= 1 us guards
 *     (vendor minima: CS setup 75 ns / hold 60 ns, DC setup/hold 40 ns);
 *   - a frame's source/workspace lease ends only after the DMA channel
 *     drained AND the rearmed PIO_FDEBUG_TXSTALL bit proved the shifter hit
 *     its known idle instruction (DMA completion is not shifter completion);
 *   - with no TE/FR line the completion level is Transferred, never
 *     Displayed;
 *   - display-on (AF) happens only after known-black GDDRAM was uploaded,
 *     so Init alone lights the panel with black — no scene dependency.
 */

#include "spi_display.h"

#if defined(PICO_ON_DEVICE)

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "hardware/timer.h"

#include "spi_display.pio.h"

#include <cstring>
#include <new>
#include <initializer_list>

#include "pico/stdlib.h"
namespace {

constexpr uint8_t kPinSck  = GpuConfig::SSD1331_SCK_PIN;   // 6  (PIO side-set)
constexpr uint8_t kPinMosi = GpuConfig::SSD1331_MOSI_PIN;  // 7  (PIO out)
constexpr uint8_t kPinCs   = GpuConfig::SSD1331_CS_PIN;    // 9  (SIO)
constexpr uint8_t kPinRst  = GpuConfig::SSD1331_RST_PIN;   // 10 (SIO)
constexpr uint8_t kPinDc   = GpuConfig::SSD1331_DC_PIN;    // 11 (SIO)

constexpr uint64_t kGpioMask = (uint64_t(1) << kPinSck) | (uint64_t(1) << kPinMosi) |
                               (uint64_t(1) << kPinCs) | (uint64_t(1) << kPinRst) |
                               (uint64_t(1) << kPinDc);

constexpr uint32_t kCsGuardUs = 1;       ///< >= CS setup 75 ns / hold 60 ns
constexpr uint32_t kShifterIdleTimeoutUs = 100;
constexpr uint32_t kInitTransferTimeoutUs = 100000;

// Reference init (module-dependent analog values marked; see SSD1331 Rev 1.2
// §9 and the OutputHostFacts report).  All bytes DC low.
constexpr uint8_t kInitCommands[] = {
    0xae,                   // display off (configure while off)
    0xad, 0x8e,             // master config: bit0 = 0 after reset
    0xa8, 0x3f,             // multiplex = 63
    0xa0, 0x72,             // remap: horiz inc, rev column, RGB order,
                            //        rev COM, odd/even split, 65K format
    0xa1, 0x00,             // display start line
    0xa2, 0x00,             // display offset
    0xa4,                   // normal display (from GDDRAM)
    0xb1, 0x31,             // phase1 = 1, phase2 = 3 DCLK (module-dependent)
    0x87, 0x06,             // master current 7/16 (module-dependent)
    0x15, 0x00, 0x5f,       // column window 0..95
    0x75, 0x00, 0x3f,       // row window 0..63
};
constexpr uint8_t kCmdDisplayOn[]  = { 0xaf };
constexpr uint8_t kCmdDisplayOff[] = { 0xae };

} // namespace

// ─── Low-level helpers ──────────────────────────────────────────────────────

bool Ssd1331PioDriver::WaitShifterIdle(uint32_t timeoutUs) {
    PIO pio = lease_.pio;
    const uint32_t bit = 1u << (PIO_FDEBUG_TXSTALL_LSB + lease_.sm);
    pio->fdebug = bit;   // W1C: rearm so a new stall proves shifter drained
    const absolute_time_t deadline = make_timeout_time_us(timeoutUs);
    while (!(pio->fdebug & bit)) {
        if (time_reached(deadline)) return false;
        tight_loop_contents();
    }
    return true;  // stalled at OUT with SCLK low: serial line fully idle
}

bool Ssd1331PioDriver::WriteBytes(const uint8_t* bytes, size_t count) {
    // DC level is preset by the caller (low for commands+arguments).
    gpio_put(kPinCs, 0);
    busy_wait_us(kCsGuardUs);
    const absolute_time_t deadline = make_timeout_time_us(kShifterIdleTimeoutUs * 10);
    bool idle = true;
    for (size_t i = 0; i < count; ++i) {
        while (pio_sm_is_tx_fifo_full(lease_.pio, lease_.sm)) {
            if (time_reached(deadline)) { idle = false; break; }
            tight_loop_contents();
        }
        if (!idle) break;
        pio_sm_put(lease_.pio, lease_.sm, 0x01010101u * bytes[i]);
    }
    if (idle) idle = WaitShifterIdle(kShifterIdleTimeoutUs);
    if (!idle) StopTransfer();
    busy_wait_us(kCsGuardUs);   // CS hold after the final SCLK edge
    gpio_put(kPinCs, 1);
    busy_wait_us(kCsGuardUs);
    return idle;
}

void Ssd1331PioDriver::StartDmaTransfer() {
    gpio_put(kPinDc, 1);                 // GDDRAM data
    busy_wait_us(kCsGuardUs);
    gpio_put(kPinCs, 0);
    busy_wait_us(kCsGuardUs);
    dma_channel_set_read_addr(dma_, frameBuffer_, false);
    dma_channel_set_trans_count(dma_, SpiDisplay::kFrameBytes, true);
    transferStartUs_ = time_us_64();
    phase_ = Phase::Transferring;
}

void Ssd1331PioDriver::FinishTransfer() {
    busy_wait_us(kCsGuardUs);
    gpio_put(kPinCs, 1);
    busy_wait_us(kCsGuardUs);
    gpio_put(kPinDc, 0);
    phase_ = Phase::Idle;
    // No TE/FR line: this is a completed GDDRAM write, not visible scanout.
    PushEvent(PglRuntime::Completion::Transferred, PglRuntime::Result::Ok, time_us_64(), true);
    if (pendingBrightness_) {
        pendingBrightness_ = false;
        if (ApplyBrightness(brightness_, true) != PglRuntime::Result::Ok) {
            faulted_ = true;
            StopTransfer();
        }
    }
}

void Ssd1331PioDriver::PushEvent(PglRuntime::Completion completion,
                                 PglRuntime::Result result, uint64_t timestampUs,
                                 bool sourceReleased) {
    const uint8_t next = uint8_t((eventHead_ + 1) & 3u);
    if (next == eventTail_) eventTail_ = uint8_t((eventTail_ + 1) & 3u);
    DisplayEvent& e = events_[eventHead_];
    e.frame = pendingFrame_;
    e.completion = completion;
    e.result = result;
    e.timestampUs = timestampUs;
    e.sourceReleased = sourceReleased;
    eventHead_ = next;
}

void Ssd1331PioDriver::StopTransfer() {
    if (dma_ >= 0) {
        hw_clear_bits(&dma_hw->ch[dma_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
        dma_channel_abort(dma_);
    }
    if (lease_.pio) pio_sm_set_enabled(lease_.pio, lease_.sm, false);
    gpio_put(kPinSck, 0);
    gpio_set_function(kPinSck, GPIO_FUNC_SIO);
    gpio_set_dir(kPinSck, GPIO_OUT);
    busy_wait_us(kCsGuardUs);
    gpio_put(kPinCs, 1);
    busy_wait_us(kCsGuardUs);
    gpio_put(kPinDc, 0);
}

void Ssd1331PioDriver::ConfigureStateMachine() {
    PIO pio = lease_.pio;
    pio_sm_config c = spi_display_program_get_default_config(lease_.offset);
    sm_config_set_sideset_pins(&c, kPinSck);
    sm_config_set_out_pins(&c, kPinMosi, 1);
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac8(&c, uint16_t(SpiDisplay::ClockDivInt(clockHz_)), 0);
    pio_sm_init(pio, lease_.sm, lease_.offset, &c);
    pio_sm_set_pins_with_mask(pio, lease_.sm, 0, (1u << kPinSck) | (1u << kPinMosi));
    pio_sm_set_consecutive_pindirs(pio, lease_.sm, kPinSck, 2, true);
    pio_gpio_init(pio, kPinSck);
    pio_gpio_init(pio, kPinMosi);
    pio_sm_set_enabled(pio, lease_.sm, true);
}

void Ssd1331PioDriver::ReleaseAllClaims() {
    if (!resources_) return;
    if (lease_.pio || dma_ >= 0) StopTransfer();
    gpio_put(kPinRst, 0);  // reset/off even when initialization rolled back
    if (dma_ >= 0) resources_->ReleaseDma(dma_, HardwareResources::Owner::Display);
    resources_->ReleasePio(lease_);
    resources_->ReleaseGpios(kGpioMask, HardwareResources::Owner::Display);
}

// ─── Init / Shutdown ────────────────────────────────────────────────────────

PglRuntime::Result Ssd1331PioDriver::Init(HardwareResources& resources,
                                          const DisplayConfig& config,
                                          void* workspace, size_t bytes) {
    using Result = PglRuntime::Result;
    using Owner = HardwareResources::Owner;
    if (inited_) return Result::BadState;
    if (config.width != SpiDisplay::kWidth || config.height != SpiDisplay::kHeight)
        return Result::Unsupported;
    if (config.type != DisplayType::SpiSsd1331) return Result::InvalidValue;
    if (!workspace) return Result::InvalidValue;
    if (bytes < SpiDisplay::kFrameBytes) return Result::Capacity;

    resources_ = &resources;
    config_ = config;
    brightness_ = config.brightness;
    frameBuffer_ = static_cast<uint8_t*>(workspace);
    clockHz_ = clock_get_hz(clk_sys);

    Result r = resources_->ClaimGpios(kGpioMask, Owner::Display);
    if (r != Result::Ok) { resources_ = nullptr; return r; }
    // Preload safe SIO output values before connecting output drivers.
    for (uint8_t pin : { kPinCs, kPinRst, kPinDc }) {
        gpio_init(pin);
        gpio_put(pin, pin != kPinDc);
        gpio_set_dir(pin, GPIO_OUT);
    }
    r = resources_->ClaimPio(pioBlock_, &spi_display_program, Owner::Display, lease_);
    if (r != Result::Ok) { ReleaseAllClaims(); resources_ = nullptr; return r; }
    r = resources_->ClaimDma(Owner::Display, dma_);
    if (r != Result::Ok) { ReleaseAllClaims(); resources_ = nullptr; return r; }


    PIO pio = lease_.pio;
    ConfigureStateMachine();
    {
        dma_channel_config c = dma_channel_get_default_config(dma_);
        channel_config_set_chain_to(&c, dma_);   // self = chaining disabled
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
        channel_config_set_dreq(&c, pio_get_dreq(pio, lease_.sm, true));
        dma_channel_configure(dma_, &c, &pio->txf[lease_.sm],
                              frameBuffer_, SpiDisplay::kFrameBytes, false);
    }

    // Bounded hardware reset: RES low >= 3 us (we hold 5 us).
    gpio_put(kPinRst, 0);
    busy_wait_us(5);
    gpio_put(kPinRst, 1);
    busy_wait_us(10);

    if (!WriteBytes(kInitCommands, sizeof(kInitCommands))) {
        ReleaseAllClaims(); resources_ = nullptr; return Result::Io;
    }
    if (brightness_ > 0 && ApplyBrightness(brightness_, false) != Result::Ok) {
        ReleaseAllClaims(); resources_ = nullptr; return Result::Io;
    }

    // Upload known-black GDDRAM before any visible enable.
    std::memset(frameBuffer_, 0, SpiDisplay::kFrameBytes);
    gpio_put(kPinDc, 1);
    busy_wait_us(kCsGuardUs);
    gpio_put(kPinCs, 0);
    busy_wait_us(kCsGuardUs);
    dma_channel_set_trans_count(dma_, SpiDisplay::kFrameBytes, true);
    const absolute_time_t deadline = make_timeout_time_us(kInitTransferTimeoutUs);
    bool ok = true;
    while (dma_channel_is_busy(dma_)) {
        if (time_reached(deadline)) { ok = false; break; }
        tight_loop_contents();
    }
    if (ok) ok = WaitShifterIdle(kShifterIdleTimeoutUs);
    if (!ok) {
        ReleaseAllClaims(); resources_ = nullptr; return Result::Io;
    }
    busy_wait_us(kCsGuardUs);
    gpio_put(kPinCs, 1);
    busy_wait_us(kCsGuardUs);
    gpio_put(kPinDc, 0);

    // Visible enable only after known-black RAM (brightness 0 stays off).
    if (brightness_ > 0 && !WriteBytes(kCmdDisplayOn, sizeof(kCmdDisplayOn))) {
        ReleaseAllClaims(); resources_ = nullptr; return Result::Io;
    }

    inited_ = true;
    quiesced_ = false;
    faulted_ = false;
    phase_ = Phase::Idle;
    pendingBrightness_ = false;
    eventHead_ = eventTail_ = 0;
    return Result::Ok;
}

void Ssd1331PioDriver::Shutdown() {
    if (!inited_) return;
    Quiesce();
    // Leave control pins in a safe driven state, then release every lease.
    gpio_put(kPinCs, 1);
    gpio_put(kPinDc, 0);
    gpio_put(kPinRst, 1);
    ReleaseAllClaims();
    resources_ = nullptr;
    frameBuffer_ = nullptr;
    inited_ = quiesced_ = faulted_ = false;
}

// ─── Present / transfer engine ──────────────────────────────────────────────

PglRuntime::Result Ssd1331PioDriver::Present(const DisplaySurface& surface,
                                             const DisplayMapping& mapping) {
    using Result = PglRuntime::Result;
    if (!inited_) return Result::NotReady;
    if (quiesced_ || faulted_) return Result::BadState;
    if (phase_ != Phase::Idle) return Result::Busy;

    surface_ = surface;
    mapping_ = mapping;
    sampler_ = new (samplerStorage_) DisplayPixelSampler(surface_, mapping_, config_);
    if (!sampler_->Valid()) {
        sampler_->~DisplayPixelSampler();
        sampler_ = nullptr;
        return Result::InvalidValue;
    }
    packCursor_ = 0;
    pendingFrame_ = surface.frame;
    phase_ = Phase::Packing;
    return Result::Ok;
}

void Ssd1331PioDriver::PollRefresh() {
    if (!inited_ || quiesced_ || faulted_) return;

    if (phase_ == Phase::Packing) {
        constexpr uint32_t kTotal = uint32_t(SpiDisplay::kWidth) * SpiDisplay::kHeight;
        const uint32_t n = (kTotal - packCursor_ < kPackSlicePixels)
                               ? (kTotal - packCursor_) : kPackSlicePixels;
        SpiDisplay::PackRgb565BE(*sampler_, frameBuffer_ + 2 * packCursor_,
                                 packCursor_, n);
        packCursor_ += n;
        if (packCursor_ >= kTotal) {
            sampler_->~DisplayPixelSampler();
            sampler_ = nullptr;          // RGB source fully consumed here
            StartDmaTransfer();
        }
        return;
    }

    if (phase_ == Phase::Transferring) {
        const uint64_t now = time_us_64();
        if (dma_channel_is_busy(dma_)) {
            if (now - transferStartUs_ > kTransferTimeoutUs) {
                StopTransfer();
                faulted_ = true;
                phase_ = Phase::Idle;
                PushEvent(PglRuntime::Completion::Failed, PglRuntime::Result::Timeout,
                          now, true);
            }
            return;
        }
        // DMA depletion is not shifter completion: prove the rearmed TXSTALL
        // at the known idle OUT (SCLK low) with a bounded per-call wait.
        if (!WaitShifterIdle(50)) {
            if (now - transferStartUs_ > kTransferTimeoutUs) {
                faulted_ = true;
                StopTransfer();
                phase_ = Phase::Idle;
                PushEvent(PglRuntime::Completion::Failed, PglRuntime::Result::Timeout,
                          now, true);
            }
            return;  // retry on the next call
        }
        FinishTransfer();
    }
}

bool Ssd1331PioDriver::PopEvent(DisplayEvent& event) {
    if (eventTail_ == eventHead_) return false;
    event = events_[eventTail_];
    eventTail_ = uint8_t((eventTail_ + 1) & 3u);
    return true;
}

// ─── Clock quiesce / resume ─────────────────────────────────────────────────

PglRuntime::Result Ssd1331PioDriver::Quiesce() {
    using Result = PglRuntime::Result;
    if (!inited_) return Result::BadState;
    if (quiesced_) return Result::Ok;

    Result drainResult = Result::Ok;
    if (phase_ == Phase::Packing) {
        // Source still borrowed: cancel and release it before clock surgery.
        if (sampler_) { sampler_->~DisplayPixelSampler(); sampler_ = nullptr; }
        phase_ = Phase::Idle;
        PushEvent(PglRuntime::Completion::Cancelled, PglRuntime::Result::Cancelled,
                  time_us_64(), true);
    }
    if (phase_ == Phase::Transferring) {
        // Bounded drain: finish the in-flight frame rather than corrupting
        // the bus mid-byte.
        const absolute_time_t deadline = make_timeout_time_us(kTransferTimeoutUs);
        bool ok = true;
        while (dma_channel_is_busy(dma_)) {
            if (time_reached(deadline)) { ok = false; break; }
            tight_loop_contents();
        }
        if (ok) ok = WaitShifterIdle(kShifterIdleTimeoutUs);
        if (!ok) {
            StopTransfer();
            faulted_ = true;
            drainResult = Result::Timeout;
            PushEvent(PglRuntime::Completion::Failed, PglRuntime::Result::Timeout,
                      time_us_64(), true);
        } else {
            FinishTransfer();
        }
    }
    phase_ = Phase::Idle;
    // Freeze idle SCLK low; abort disables EN before releasing DMA ownership.
    StopTransfer();
    quiesced_ = true;
    return drainResult;
}

PglRuntime::Result Ssd1331PioDriver::Resume(uint32_t systemClockHz) {
    using Result = PglRuntime::Result;
    if (!inited_) return Result::BadState;
    if (!quiesced_) return Result::BadState;
    if (!systemClockHz || SpiDisplay::ClockDivInt(systemClockHz) > 65535u)
        return Result::InvalidValue;
    clockHz_ = systemClockHz;
    // Reinitialize at the known low entry after a timeout/abort too: no stale
    // OSR/FIFO bytes or fractional divider phase survive a resumed transfer.
    ConfigureStateMachine();
    faulted_ = false;
    quiesced_ = false;
    pendingBrightness_ = false;
    const Result result = ApplyBrightness(brightness_, true);
    if (result != Result::Ok) { faulted_ = true; StopTransfer(); }
    return result;
}

// ─── Brightness / caps ──────────────────────────────────────────────────────

PglRuntime::Result Ssd1331PioDriver::ApplyBrightness(uint8_t brightness, bool ensureOn) {
    using Result = PglRuntime::Result;
    if (brightness == 0) {
        return WriteBytes(kCmdDisplayOff, sizeof(kCmdDisplayOff)) ? Result::Ok : Result::Io;
    }
    // Scale the reference contrast (0x7f mid, module-dependent) by brightness.
    const uint8_t level = uint8_t((uint32_t(0x7f) * brightness + 127u) / 255u);
    if (!ensureOn) {   // init path: visible enable waits for known-black RAM
        const uint8_t cmds[] = { 0x81, level, 0x82, level, 0x83, level };
        return WriteBytes(cmds, sizeof(cmds)) ? Result::Ok : Result::Io;
    }
    const uint8_t cmds[] = { 0x81, level, 0x82, level, 0x83, level, 0xaf };
    return WriteBytes(cmds, sizeof(cmds)) ? Result::Ok : Result::Io;
}

PglRuntime::Result Ssd1331PioDriver::SetBrightness(uint8_t brightness) {
    using Result = PglRuntime::Result;
    if (!inited_) return Result::BadState;
    brightness_ = brightness;
    if (quiesced_ || faulted_) { pendingBrightness_ = true; return Result::Ok; }
    if (phase_ != Phase::Idle) {                    // never interleave commands
        pendingBrightness_ = true;                  // with an open data window
        return Result::Ok;
    }
    const Result result = ApplyBrightness(brightness, true);
    if (result != Result::Ok) { faulted_ = true; StopTransfer(); }
    return result;
}

DisplayCapabilities Ssd1331PioDriver::GetCaps() const {
    DisplayCapabilities caps = {};
    caps.type = DisplayType::SpiSsd1331;
    if (inited_) {   // disabled: zero resource claims
        caps.width = SpiDisplay::kWidth;
        caps.height = SpiDisplay::kHeight;
        caps.displayCompletion = false;  // no TE: Transferred only
        caps.pioStateMachines = 1;
        caps.dmaChannels = 1;
        caps.pioInstructions = 2;
        caps.workspaceBytes = SpiDisplay::kFrameBytes;
    }
    return caps;
}

// ─── Factory ────────────────────────────────────────────────────────────────

DisplayDriver& SpiDisplayBackend() {
    static Ssd1331PioDriver instance;
    return instance;
}

#endif // PICO_ON_DEVICE
