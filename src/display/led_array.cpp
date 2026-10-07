/**
 * @file led_array.cpp
 * @brief WS2812B-V5/W LED array DisplayDriver (PIO + DMA) for RP2350 (P08).
 *
 * Resource cost: 1 PIO SM, 4 PIO instructions, 1 DMA channel, 1 GPIO
 * (GPU_CONFIG LED_DATA_PIN = 6), 0 IRQ lines (completion is polled through
 * PollRefresh with bounded deadlines; no exclusive DMA IRQ handler exists).
 * Workspace: pixelCount * 4 bytes of the caller-owned shared 68 KiB buffer;
 * no heap, no per-driver static stream buffer, no per-sample copying.
 *
 * Frame protocol:
 *   Present -> bounded encode slices (128 px/step) into the workspace
 *   -> DREQ-paced DMA of the immutable GRB word stream -> rearm FDEBUG
 *   TXSTALL and wait for the stall at the low `out` (shift completion, not
 *   just DMA completion) -> >= 300 us low reset window -> only then the
 *   Displayed event with sourceReleased. The RGB565 source is read only
 *   during encode slices, so sourceReleased always follows the last CPU
 *   read. Underflow/fault forces the line low, waits the reset window and
 *   re-streams a known-black frame (or reports Faulted); the line is never
 *   left high.
 *
 * Retained-state note: a WS2812 strip keeps its last colors through an MCU
 * reset; GPIO reset alone does NOT blank it. Init therefore streams one
 * full black frame before reporting Ok. Guaranteed fail-dark across power
 * faults requires power gating of the strip — that remains a physical
 * prerequisite, not something this driver claims.
 */

#include "led_array.h"

namespace LedArray {

uint32_t EncodeGrbSlice(const DisplayPixelSampler& sampler, uint8_t brightness,
                        uint32_t firstPixel, uint32_t count, uint32_t* outWords) {
    for (uint32_t i = 0; i < count; ++i) {
        uint16_t c = sampler.Get(firstPixel + i);
        if (brightness != 255) c = ScaleDisplayBrightness(c, brightness);
        outWords[i] = EncodeGrbWord(c);
    }
    return count;
}

} // namespace LedArray

#if defined(PICO_ON_DEVICE)

#include "led_array.pio.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/regs/pio.h"

#include <cstring>
#include <new>

namespace {

using PglRuntime::Result;
using PglRuntime::Completion;

constexpr uint8_t  kPinData          = GpuConfig::LED_DATA_PIN;  // 6
constexpr uint32_t kEncodeSlicePx    = 128;    // bounded CPU slice per step
constexpr uint64_t kStallWaitUs      = 300;    // joined FIFO + OSR: <= 9 * 30 us
constexpr uint64_t kQuiesceMarginUs  = 2000;

class LedArrayDriver final : public DisplayDriver {
public:
    LedArrayDriver() = default;

    // ── DisplayDriver interface ─────────────────────────────────────────────

    Result Init(HardwareResources& resources, const DisplayConfig& config,
                void* workspace, size_t bytes) override {
        if (state_ != State::Inactive) return Result::BadState;
        if (!workspace || (reinterpret_cast<uintptr_t>(workspace) & 3u) || !bytes) {
            return Result::InvalidValue;
        }
        // Validate BEFORE claiming anything: over-capacity or bad config must
        // fail with pins untouched.
        uint32_t pixels = 0;
        Result r = LedArray::ValidateConfig(config, bytes, pixels);
        if (r != Result::Ok) return r;

        const uint64_t pinMask = uint64_t(1) << kPinData;
        r = resources.ClaimGpios(pinMask, HardwareResources::Owner::Display);
        if (r != Result::Ok) return r;
        gpioClaimed_ = true;

        r = Result::Capacity;
        for (uint8_t block = 0; block < 3 && r != Result::Ok; ++block) {
            r = resources.ClaimPio(block, &ws2812b_v5_program,
                                   HardwareResources::Owner::Display, pioLease_);
        }
        if (r != Result::Ok) { ReleaseLeases(resources); return r; }

        r = resources.ClaimDma(HardwareResources::Owner::Display, dmaChannel_);
        if (r != Result::Ok) { ReleaseLeases(resources); return r; }

        res_           = &resources;
        cfg_           = config;
        words_         = static_cast<uint32_t*>(workspace);
        workspaceBytes_= bytes;
        pixelCount_    = pixels;

        LedArray::ClockDivider div{};
        if (!LedArray::ComputeClockDivider(clock_get_hz(clk_sys), div)) {
            ReleaseLeases(resources);
            return Result::InvalidValue;  // waveform would leave the V5/W windows
        }
        ConfigureStateMachine(div);
        busy_wait_us_32(LedArray::kResetLowUs);

        // Startup known black: a retained strip shows stale colors until it
        // receives new data, so push one full black stream through the real
        // DMA path before reporting ready.
        r = StreamBlackSync();
        if (r != Result::Ok) { ReleaseLeases(resources); return r; }

        idleSinceUs_ = time_us_64();
        state_       = State::Idle;
        quiesced_    = false;
        eventPending_ = false;
        return Result::Ok;
    }

    Result Present(const DisplaySurface& surface, const DisplayMapping& mapping) override {
        if (state_ == State::Inactive) return Result::NotReady;
        if (state_ == State::Faulted)  return Result::BadState;
        if (quiesced_) return Result::BadState;
        if (state_ != State::Idle || eventPending_) return Result::Busy;
        if (!ValidDisplaySurface(surface)) return Result::InvalidValue;

        // Borrow the surface: keep copies of the descriptors (coordinate span
        // and pixels stay immutable until sourceReleased) and build the
        // sampler once — validation and rectangle steps happen here only.
        surface_ = surface;
        mapping_ = mapping;
        new (samplerStorage_) DisplayPixelSampler(surface_, mapping_, cfg_);
        samplerLive_ = true;
        if (!sampler()->Valid()) { DropSampler(); return Result::InvalidValue; }

        frame_        = surface.frame;
        encodeCursor_ = 0;
        frameBrightness_ = cfg_.brightness;
        state_        = State::Encoding;
        StepEncoding();  // first bounded slice runs synchronously
        return Result::Ok;
    }

    void PollRefresh() override {
        switch (state_) {
        case State::Encoding:  StepEncoding();  break;
        case State::Streaming: StepStreaming(); break;
        case State::ResetLow:
            if (time_us_64() - idleSinceUs_ >= LedArray::kResetLowUs) {
                state_ = State::Idle;
                Emit(Completion::Displayed, Result::Ok);
            }
            break;
        default: break;
        }
    }

    bool PopEvent(DisplayEvent& event) override {
        if (!eventPending_) return false;
        event = event_;
        eventPending_ = false;
        return true;
    }

    Result Quiesce() override {
        if (state_ == State::Inactive) return Result::BadState;
        if (state_ == State::Faulted) { quiesced_ = true; return Result::BadState; }
        if (quiesced_) return Result::Ok;
        if (state_ == State::Idle) {
            WaitResetWindow();
            pio_sm_set_enabled(pioLease_.pio, pioLease_.sm, false);
            quiesced_ = true;
            return Result::Ok;
        }
        // Bounded drain: finish the active frame so the clock hook finds the
        // line low in the reset window.
        const uint64_t deadline = time_us_64() + LedArray::StreamBudgetUs(pixelCount_)
                                + kStallWaitUs + LedArray::kResetLowUs + kQuiesceMarginUs;
        while (state_ != State::Idle) {
            if (state_ == State::Faulted) { quiesced_ = true; return Result::Timeout; }
            if (time_us_64() >= deadline) { EnterFault(); quiesced_ = true; return Result::Timeout; }
            if (state_ == State::Encoding)       StepEncoding();
            else if (state_ == State::Streaming) StepStreaming();
            else {
                WaitResetWindow();
                state_ = State::Idle;
                Emit(Completion::Displayed, Result::Ok);
            }
        }
        pio_sm_set_enabled(pioLease_.pio, pioLease_.sm, false);
        quiesced_ = true;
        return Result::Ok;
    }
    Result Resume(uint32_t systemClockHz) override {
        if (state_ == State::Inactive) return Result::BadState;
        if (!quiesced_) return Result::BadState;
        if (state_ == State::Faulted) return Result::BadState;
        if (state_ != State::Idle) return Result::Busy;  // divider changes only at idle-low
        LedArray::ClockDivider div{};
        if (!LedArray::ComputeClockDivider(systemClockHz, div)) return Result::InvalidValue;
        pio_sm_set_clkdiv_int_frac8(pioLease_.pio, pioLease_.sm, div.integer, div.fraction);
        pio_sm_clkdiv_restart(pioLease_.pio, pioLease_.sm);  // restart phase while idle-low
        pio_sm_set_enabled(pioLease_.pio, pioLease_.sm, true);
        quiesced_ = false;
        return Result::Ok;
    }

    Result SetBrightness(uint8_t brightness) override {
        if (state_ == State::Inactive) return Result::BadState;
        cfg_.brightness = brightness;  // applies from the next Present; 0/255 exact in the encoder
        return Result::Ok;
    }

    DisplayCapabilities GetCaps() const override {
        DisplayCapabilities caps{};
        caps.type              = DisplayType::LedV5;
        if (state_ == State::Inactive) return caps;
        caps.width             = state_ == State::Inactive ? 0 : cfg_.width;
        caps.height            = state_ == State::Inactive ? 0 : cfg_.height;
        caps.displayCompletion = true;   // reports Displayed after the reset window
        caps.pioStateMachines  = 1;
        caps.dmaChannels       = 1;
        caps.pioInstructions   = LedArray::kPioInstructions;  // 4
        caps.workspaceBytes    = uint32_t(pixelCount_) * LedArray::kBytesPerPixel;
        return caps;
    }

    void Shutdown() override {
        if (state_ == State::Inactive) return;
        if (state_ == State::Encoding || state_ == State::Streaming ||
            state_ == State::ResetLow) {
            AbortTransfer();
            ForceLineLow();
            busy_wait_us_32(LedArray::kResetLowUs);  // leave the strip in reset
            Emit(Completion::Cancelled, Result::Cancelled);
        }
        DropSampler();
        // Orderly shutdown must overwrite retained colors; a low GPIO alone
        // only resets the receiver. A hardware power fault still needs gating.
        AbortTransfer();
        ForceLineLow();
        busy_wait_us_32(LedArray::kResetLowUs);
        LedArray::ClockDivider div{};
        if (LedArray::ComputeClockDivider(clock_get_hz(clk_sys), div)) {
            ConfigureStateMachine(div);
            StreamBlackSync();  // bounded; lease release forces low on failure
        }
        if (res_) ReleaseLeases(*res_);
        state_ = State::Inactive;
    }

private:
    enum class State : uint8_t { Inactive, Idle, Encoding, Streaming, ResetLow, Faulted };

    DisplayPixelSampler* sampler() {
        return reinterpret_cast<DisplayPixelSampler*>(samplerStorage_);
    }
    void DropSampler() {
        if (samplerLive_) {
            sampler()->~DisplayPixelSampler();
            samplerLive_ = false;
        }
    }


    void ConfigureStateMachine(const LedArray::ClockDivider& div) {
        // Preload PIO's value/direction before selecting its pad function.
        pio_sm_config c = ws2812b_v5_program_get_default_config(pioLease_.offset);
        sm_config_set_sideset_pins(&c, kPinData);
        sm_config_set_out_shift(&c, false /*shift left = MSB first*/,
                                true /*autopull*/, LedArray::kBitsPerPixel);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv_int_frac8(&c, div.integer, div.fraction);
        pio_sm_init(pioLease_.pio, pioLease_.sm, pioLease_.offset, &c);
        pio_sm_set_pins_with_mask(pioLease_.pio, pioLease_.sm, 0, 1u << kPinData);
        pio_sm_set_consecutive_pindirs(pioLease_.pio, pioLease_.sm, kPinData, 1, true);
        pio_gpio_init(pioLease_.pio, kPinData);
        pio_sm_set_enabled(pioLease_.pio, pioLease_.sm, true);
        // The SM immediately stalls in `out` with side-set 0: line low.
    }

    void StepEncoding() {
        const uint32_t remaining = pixelCount_ - encodeCursor_;
        const uint32_t n = remaining < kEncodeSlicePx ? remaining : kEncodeSlicePx;
        LedArray::EncodeGrbSlice(*sampler(), frameBrightness_, encodeCursor_, n,
                                 words_ + encodeCursor_);
        encodeCursor_ += n;
        if (encodeCursor_ == pixelCount_) {
            DropSampler();
            StartTransfer();
        }
    }

    void StartTransfer() {
        pio_sm_clear_fifos(pioLease_.pio, pioLease_.sm);
        dma_channel_config c = dma_channel_get_default_config(dmaChannel_);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(pioLease_.pio, pioLease_.sm, true));
        dma_channel_configure(dmaChannel_, &c, &pioLease_.pio->txf[pioLease_.sm],
                              words_, pixelCount_, true);
        streamDeadline_ = time_us_64() + LedArray::StreamBudgetUs(pixelCount_);
        state_ = State::Streaming;
    }

    void StepStreaming() {
        if (dma_channel_is_busy(dmaChannel_)) {
            if (time_us_64() >= streamDeadline_) EnterFault();  // starved: go known-black
            return;
        }
        // DMA done is not shift completion: rearm TXSTALL and wait for the
        // stall at the low `out` instruction.
        if (!WaitShifterIdle(time_us_64() + kStallWaitUs)) { EnterFault(); return; }
        idleSinceUs_ = time_us_64();  // line is now held low by the stalled SM
        state_ = State::ResetLow;
    }

    static bool WaitShifterIdle(PIO pio, uint sm, uint64_t deadlineUs) {
        const uint32_t bit = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
        pio->fdebug = bit;  // W1C rearm
        while (!(pio->fdebug & bit)) {
            if (time_us_64() >= deadlineUs) return false;
            tight_loop_contents();
        }
        return true;
    }
    bool WaitShifterIdle(uint64_t deadlineUs) {
        return WaitShifterIdle(pioLease_.pio, pioLease_.sm, deadlineUs);
    }

    void WaitResetWindow() {
        const uint64_t elapsed = time_us_64() - idleSinceUs_;
        if (elapsed < LedArray::kResetLowUs) {
            busy_wait_us_32(uint32_t(LedArray::kResetLowUs - elapsed));
        }
    }

    Result StreamBlackSync() {
        std::memset(words_, 0, size_t(pixelCount_) * sizeof(uint32_t));
        StartTransfer();
        while (state_ == State::Streaming) {
            if (dma_channel_is_busy(dmaChannel_)) {
                if (time_us_64() >= streamDeadline_) {
                    AbortTransfer();
                    ForceLineLow();
                    return Result::Timeout;
                }
                tight_loop_contents();
                continue;
            }
            if (!WaitShifterIdle(time_us_64() + kStallWaitUs)) {
                ForceLineLow();
                return Result::Io;
            }
            state_ = State::ResetLow;
        }
        idleSinceUs_ = time_us_64();
        busy_wait_us_32(LedArray::kResetLowUs);
        return Result::Ok;
    }

    void AbortTransfer() {
        // Single owned channel, no chain: clear EN (RP2350-E5 chain rule is
        // satisfied vacuously), then abort and wait for the bus to go quiet.
        if (dmaChannel_ >= 0) {
            hw_clear_bits(&dma_hw->ch[dmaChannel_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
            dma_channel_abort(dmaChannel_);
        }
    }

    void ForceLineLow() {
        if (pioLease_.pio) pio_sm_set_enabled(pioLease_.pio, pioLease_.sm, false);
        gpio_set_function(kPinData, GPIO_FUNC_SIO);
        gpio_set_dir(kPinData, GPIO_OUT);
        gpio_put(kPinData, false);
    }

    void EnterFault() {
        AbortTransfer();
        DropSampler();
        ForceLineLow();
        busy_wait_us_32(LedArray::kResetLowUs);  // known-low reset window first
        // Known-black recovery attempt through the real path.
        LedArray::ClockDivider div{};
        const bool clockOk = LedArray::ComputeClockDivider(clock_get_hz(clk_sys), div);
        bool recovered = false;
        if (clockOk) {
            ConfigureStateMachine(div);  // re-takes PIO ownership of the pin
            recovered = StreamBlackSync() == Result::Ok;
        }
        if (!recovered) ForceLineLow();
        state_ = recovered ? State::Idle : State::Faulted;
        Emit(Completion::Failed, Result::Io);
    }

    void Emit(Completion completion, Result result) {
        // The source is only ever read during encode slices; any event is
        // emitted after the last read (or after an abort with no further
        // reads), so sourceReleased is always truthful here.
        event_.frame          = frame_;
        event_.completion     = completion;
        event_.result         = result;
        event_.timestampUs    = time_us_64();
        event_.sourceReleased = true;
        eventPending_         = true;
    }

    void ReleaseLeases(HardwareResources& resources) {
        if (dmaChannel_ >= 0) {
            AbortTransfer();
            resources.ReleaseDma(dmaChannel_, HardwareResources::Owner::Display);
        }
        if (gpioClaimed_) ForceLineLow();
        if (pioLease_.pio) resources.ReleasePio(pioLease_);
        DropSampler();
        if (gpioClaimed_) {
            resources.ReleaseGpios(uint64_t(1) << kPinData, HardwareResources::Owner::Display);
            gpioClaimed_ = false;
        }
        res_ = nullptr;
        state_ = State::Inactive;  // no resources held: driver is back to inactive
    }

    HardwareResources* res_ = nullptr;
    DisplayConfig   cfg_{};
    DisplaySurface  surface_{};
    DisplayMapping  mapping_{};
    uint32_t*       words_          = nullptr;
    size_t          workspaceBytes_ = 0;
    uint32_t        pixelCount_     = 0;
    uint32_t        encodeCursor_   = 0;
    uint32_t        frame_          = 0;
    uint64_t        idleSinceUs_    = 0;
    uint64_t        streamDeadline_ = 0;
    HardwareResources::PioLease pioLease_{};
    int             dmaChannel_     = -1;
    bool            gpioClaimed_    = false;
    bool            samplerLive_    = false;
    bool            quiesced_       = false;
    uint8_t         frameBrightness_ = 255;
    State           state_          = State::Inactive;
    DisplayEvent    event_{};
    bool            eventPending_   = false;
    alignas(DisplayPixelSampler) uint8_t samplerStorage_[sizeof(DisplayPixelSampler)]{};
};

LedArrayDriver gLedArrayDriver;

} // namespace

DisplayDriver& LedArrayBackend() { return gLedArrayDriver; }

#endif // PICO_ON_DEVICE
