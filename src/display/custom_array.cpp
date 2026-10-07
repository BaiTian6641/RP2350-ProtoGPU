/**
 * @file custom_array.cpp
 * @brief Custom 8-bit parallel RGB888 DisplayDriver (PIO + DMA) for RP2350 (P08).
 *
 * Resource cost: 1 PIO SM, 2 PIO instructions, 1 DMA channel, 11 GPIOs
 * (DATA0..7 = 6..13, CLK = 14, LATCH = 15, OE = 16), 0 IRQ lines (completion
 * polled through PollRefresh with bounded deadlines). Workspace:
 * WordCount(pixels) * 4 bytes of the caller-owned shared 68 KiB buffer;
 * no heap, no per-driver static stream buffer, no per-sample copying.
 *
 * Frame protocol:
 *   Present blanks OE, then bounded encode slices (128 px/step) fill the
 *   immutable padded byte stream (leading zero pad + R,G,B bytes)
 *   -> DREQ-paced DMA of the complete byte count -> rearm FDEBUG TXSTALL and
 *   wait for the stall at the low `out` (the final clock has then actually
 *   completed, CLK low) -> LATCH pulse -> OE assert (active low) -> only
 *   then the Displayed event with sourceReleased. OE stays blanked through
 *   the whole transfer and every blocked wait, so starvation or abort can
 *   never expose a partial frame. The RGB565 source is read only during
 *   encode slices, so sourceReleased always follows the last CPU read.
 *   DMA/abort never releases the active workspace early: the buffer is
 *   reused only after the DMA channel is fully stopped.
 *
 * Clock hooks (Resume) are accepted only at the idle/latch boundary
 * (state Idle, SM stalled with CLK low), never mid-frame.
 */

#include "custom_array.h"

namespace CustomArray {

void EncodeRgb888Slice(const DisplayPixelSampler& sampler, uint8_t brightness,
                       uint32_t firstPixel, uint32_t count, uint8_t* out) {
    for (uint32_t i = 0; i < count; ++i) {
        uint16_t c = sampler.Get(firstPixel + i);
        if (brightness != 255) c = ScaleDisplayBrightness(c, brightness);
        uint8_t r, g, b;
        ExpandRgb888(c, r, g, b);
        out[3 * i + 0] = r;
        out[3 * i + 1] = g;
        out[3 * i + 2] = b;
    }
}

} // namespace CustomArray

#if defined(PICO_ON_DEVICE)

#include "custom_array.pio.h"

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

constexpr uint8_t  kPinDataBase      = GpuConfig::CUSTOM_DATA_BASE;  // 6..13
constexpr uint8_t  kPinClk           = GpuConfig::CUSTOM_CLK;        // 14
constexpr uint8_t  kPinLatch         = GpuConfig::CUSTOM_LATCH;      // 15
constexpr uint8_t  kPinOe            = GpuConfig::CUSTOM_OE;         // 16 (active low)
constexpr uint32_t kEncodeSlicePx    = 128;
constexpr uint64_t kStallWaitUs      = 100;
constexpr uint64_t kQuiesceMarginUs  = 2000;
constexpr uint32_t kLatchPulseUs     = 1;   // panel latch minimum is a physical prerequisite

class CustomArrayDriver final : public DisplayDriver {
public:
    CustomArrayDriver() = default;

    // ── DisplayDriver interface ─────────────────────────────────────────────

    Result Init(HardwareResources& resources, const DisplayConfig& config,
                void* workspace, size_t bytes) override {
        if (state_ != State::Inactive) return Result::BadState;
        if (!workspace || (reinterpret_cast<uintptr_t>(workspace) & 3u) || !bytes) {
            return Result::InvalidValue;
        }
        // Validate BEFORE claiming anything.
        uint32_t pixels = 0;
        Result r = CustomArray::ValidateConfig(config, bytes, pixels);
        if (r != Result::Ok) return r;

        r = resources.ClaimGpios(kGpioMask, HardwareResources::Owner::Display);
        if (r != Result::Ok) return r;
        gpioClaimed_ = true;
        // Fail-dark before any later resource/configuration failure.
        gpio_init(kPinOe);
        gpio_put(kPinOe, true);
        gpio_set_dir(kPinOe, GPIO_OUT);
        gpio_init(kPinLatch);
        gpio_put(kPinLatch, false);
        gpio_set_dir(kPinLatch, GPIO_OUT);

        r = Result::Capacity;
        for (uint8_t block = 0; block < 3 && r != Result::Ok; ++block) {
            r = resources.ClaimPio(block, &custom_rgb888_program,
                                   HardwareResources::Owner::Display, pioLease_);
        }
        if (r != Result::Ok) { ReleaseLeases(resources); return r; }

        r = resources.ClaimDma(HardwareResources::Owner::Display, dmaChannel_);
        if (r != Result::Ok) { ReleaseLeases(resources); return r; }

        res_            = &resources;
        cfg_            = config;
        stream_         = static_cast<uint8_t*>(workspace);
        workspaceBytes_ = bytes;
        pixelCount_     = pixels;
        padBytes_       = CustomArray::LeadingPadBytes(pixels);
        wordCount_      = CustomArray::WordCount(pixels);

        CustomArray::ClockDivider div{};
        if (!CustomArray::ComputeClockDivider(clock_get_hz(clk_sys), div)) {
            ReleaseLeases(resources);
            return Result::InvalidValue;
        }


        ConfigureStateMachine(div);  // SM stalls at `out` side 0: CLK low

        // Startup known black: commit one full black frame through the real
        // DMA + latch path, then enable outputs showing black.
        r = StreamBlackSync();
        if (r != Result::Ok) { ReleaseLeases(resources); return r; }

        state_ = State::Idle;
        quiesced_ = false;
        eventPending_ = false;
        return Result::Ok;
    }

    Result Present(const DisplaySurface& surface, const DisplayMapping& mapping) override {
        if (state_ == State::Inactive) return Result::NotReady;
        if (quiesced_ || state_ == State::Faulted) return Result::BadState;
        if (state_ != State::Idle || eventPending_) return Result::Busy;
        if (!ValidDisplaySurface(surface)) return Result::InvalidValue;


        surface_ = surface;
        mapping_ = mapping;
        new (samplerStorage_) DisplayPixelSampler(surface_, mapping_, cfg_);
        samplerLive_ = true;
        if (!sampler()->Valid()) { DropSampler(); return Result::InvalidValue; }
        gpio_put(kPinOe, true);

        frame_        = surface.frame;
        frameBrightness_ = cfg_.brightness;
        encodeCursor_ = 0;
        std::memset(stream_, 0, padBytes_);  // leading zero pad
        state_        = State::Encoding;
        StepEncoding();  // first bounded slice runs synchronously
        return Result::Ok;
    }

    void PollRefresh() override {
        switch (state_) {
        case State::Encoding:  StepEncoding();  break;
        case State::Streaming: StepStreaming(); break;
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
        gpio_put(kPinOe, true);
        if (state_ == State::Idle) {
            pio_sm_set_enabled(pioLease_.pio, pioLease_.sm, false);
            quiesced_ = true;
            return Result::Ok;
        }
        const uint64_t deadline = time_us_64() + CustomArray::StreamBudgetUs(pixelCount_)
                                + kStallWaitUs + kQuiesceMarginUs;
        while (state_ != State::Idle) {
            if (state_ == State::Faulted) { quiesced_ = true; return Result::Timeout; }
            if (time_us_64() >= deadline) { EnterFault(); quiesced_ = true; return Result::Timeout; }
            if (state_ == State::Encoding) StepEncoding();
            else                           StepStreaming();
        }
        gpio_put(kPinOe, true);
        pio_sm_set_enabled(pioLease_.pio, pioLease_.sm, false);
        quiesced_ = true;
        return Result::Ok;
    }

    Result Resume(uint32_t systemClockHz) override {
        if (state_ == State::Inactive) return Result::BadState;
        if (!quiesced_) return Result::BadState;
        if (state_ == State::Faulted) return Result::BadState;
        if (state_ != State::Idle) return Result::Busy;  // only at the idle/latch boundary
        CustomArray::ClockDivider div{};
        if (!CustomArray::ComputeClockDivider(systemClockHz, div)) return Result::InvalidValue;
        pio_sm_set_clkdiv_int_frac8(pioLease_.pio, pioLease_.sm, div.integer, div.fraction);
        pio_sm_clkdiv_restart(pioLease_.pio, pioLease_.sm);  // restart phase while CLK is low
        pio_sm_set_enabled(pioLease_.pio, pioLease_.sm, true);
        quiesced_ = false;
        gpio_put(kPinOe, cfg_.brightness == 0 || !committed_);
        return Result::Ok;
    }

    Result SetBrightness(uint8_t brightness) override {
        if (state_ == State::Inactive) return Result::BadState;
        cfg_.brightness = brightness;  // applies from the next Present; 0/255 exact in the encoder
        if (brightness == 0) gpio_put(kPinOe, true);
        return Result::Ok;
    }

    DisplayCapabilities GetCaps() const override {
        DisplayCapabilities caps{};
        caps.type              = DisplayType::CustomRgb888;
        if (state_ == State::Inactive) return caps;
        caps.width             = state_ == State::Inactive ? 0 : cfg_.width;
        caps.height            = state_ == State::Inactive ? 0 : cfg_.height;
        caps.displayCompletion = true;   // reports Displayed after the real latch
        caps.pioStateMachines  = 1;
        caps.dmaChannels       = 1;
        caps.pioInstructions   = CustomArray::kPioInstructions;  // 2
        caps.workspaceBytes    = wordCount_ * 4u;
        return caps;
    }

    void Shutdown() override {
        if (state_ == State::Inactive) return;
        if (state_ == State::Encoding || state_ == State::Streaming) {
            AbortTransfer();  // DMA fully stopped before the workspace is released
            Emit(Completion::Cancelled, Result::Cancelled);
        }
        // Blank independently of stalled PIO, including a mid-high abort.
        DropSampler();
        gpio_put(kPinOe, true);
        gpio_put(kPinLatch, false);
        ForceClockLow();
        if (res_) ReleaseLeases(*res_);
        state_ = State::Inactive;
    }

private:
    enum class State : uint8_t { Inactive, Idle, Encoding, Streaming, Faulted };

    static constexpr uint64_t kGpioMask =
        (uint64_t(0xFF) << kPinDataBase) | (uint64_t(1) << kPinClk) |
        (uint64_t(1) << kPinLatch) | (uint64_t(1) << kPinOe);

    DisplayPixelSampler* sampler() {
        return reinterpret_cast<DisplayPixelSampler*>(samplerStorage_);
    }
    void DropSampler() {
        if (samplerLive_) {
            sampler()->~DisplayPixelSampler();
            samplerLive_ = false;
        }
    }


    void ConfigureStateMachine(const CustomArray::ClockDivider& div) {
        pio_sm_config c = custom_rgb888_program_get_default_config(pioLease_.offset);
        sm_config_set_out_pins(&c, kPinDataBase, 8);
        sm_config_set_sideset_pins(&c, kPinClk);
        sm_config_set_out_shift(&c, true /*shift right: LSB of each word first*/,
                                true /*autopull*/, 32);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv_int_frac8(&c, div.integer, div.fraction);
        pio_sm_init(pioLease_.pio, pioLease_.sm, pioLease_.offset, &c);
        pio_sm_set_pins_with_mask(pioLease_.pio, pioLease_.sm, 0,
                                 (0xffu << kPinDataBase) | (1u << kPinClk));
        pio_sm_set_consecutive_pindirs(pioLease_.pio, pioLease_.sm, kPinDataBase, 9, true);
        for (uint8_t pin = kPinDataBase; pin <= kPinClk; ++pin)
            pio_gpio_init(pioLease_.pio, pin);
        pio_sm_set_enabled(pioLease_.pio, pioLease_.sm, true);
        // The SM immediately stalls in `out pins, 8` with side-set 0: CLK low.
    }

    void StepEncoding() {
        const uint32_t remaining = pixelCount_ - encodeCursor_;
        const uint32_t n = remaining < kEncodeSlicePx ? remaining : kEncodeSlicePx;
        CustomArray::EncodeRgb888Slice(*sampler(), frameBrightness_, encodeCursor_, n,
                                       stream_ + padBytes_ + size_t(encodeCursor_) * 3u);
        encodeCursor_ += n;
        if (encodeCursor_ == pixelCount_) { DropSampler(); StartTransfer(); }
    }

    void StartTransfer() {
        pio_sm_clear_fifos(pioLease_.pio, pioLease_.sm);
        dma_channel_config c = dma_channel_get_default_config(dmaChannel_);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(pioLease_.pio, pioLease_.sm, true));
        dma_channel_configure(dmaChannel_, &c, &pioLease_.pio->txf[pioLease_.sm],
                              stream_, wordCount_, true);
        streamDeadline_ = time_us_64() + CustomArray::StreamBudgetUs(pixelCount_);
        state_ = State::Streaming;
    }

    void StepStreaming() {
        if (dma_channel_is_busy(dmaChannel_)) {
            if (time_us_64() >= streamDeadline_) EnterFault();  // OE is blank: safe
            return;
        }
        // DMA done is not shift completion: rearm TXSTALL and wait for the
        // stall at the low `out`; the final clock has then completed.
        if (!WaitShifterIdle(time_us_64() + kStallWaitUs)) { EnterFault(); return; }
        CommitLatch();
        state_ = State::Idle;
        Emit(Completion::Displayed, Result::Ok);
    }

    void CommitLatch() {
        // CLK confirmed low; the complete byte count has been clocked.
        gpio_put(kPinLatch, true);
        busy_wait_us_32(kLatchPulseUs);
        gpio_put(kPinLatch, false);
        busy_wait_us_32(kLatchPulseUs);
        committed_ = true;
        gpio_put(kPinOe, cfg_.brightness == 0);  // brightness zero stays fail-dark
    }

    bool WaitShifterIdle(uint64_t deadlineUs) {
        const uint32_t bit = 1u << (PIO_FDEBUG_TXSTALL_LSB + pioLease_.sm);
        pioLease_.pio->fdebug = bit;  // W1C rearm
        while (!(pioLease_.pio->fdebug & bit)) {
            if (time_us_64() >= deadlineUs) return false;
            tight_loop_contents();
        }
        return true;
    }

    Result StreamBlackSync() {
        std::memset(stream_, 0, size_t(wordCount_) * 4u);
        StartTransfer();
        while (state_ == State::Streaming) {
            if (dma_channel_is_busy(dmaChannel_)) {
                if (time_us_64() >= streamDeadline_) {
                    AbortTransfer();
                    ForceClockLow();
                    return Result::Timeout;
                }
                tight_loop_contents();
                continue;
            }
            if (!WaitShifterIdle(time_us_64() + kStallWaitUs)) {
                ForceClockLow();
                return Result::Io;
            }
            state_ = State::Idle;  // provisional; CommitLatch below
        }
        CommitLatch();  // outputs enabled showing known black
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

    void ForceClockLow() {
        if (pioLease_.pio) pio_sm_set_enabled(pioLease_.pio, pioLease_.sm, false);
        gpio_set_function(kPinClk, GPIO_FUNC_SIO);
        gpio_set_dir(kPinClk, GPIO_OUT);
        gpio_put(kPinClk, false);
    }

    void EnterFault() {
        // The panel stays blank: OE was deasserted before the transfer began
        gpio_put(kPinOe, true);
        committed_ = false;
        DropSampler();
        // and the latch never ran for this frame.
        AbortTransfer();
        ForceClockLow();
        // Recover to the known stalled-idle state (CLK low, OE blank).
        CustomArray::ClockDivider div{};
        if (CustomArray::ComputeClockDivider(clock_get_hz(clk_sys), div)) {
            ConfigureStateMachine(div);
            state_ = State::Idle;
        } else {
            state_ = State::Faulted;
        }
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
        DropSampler();
        if (gpioClaimed_) {
            gpio_put(kPinOe, true);
            gpio_put(kPinLatch, false);
            ForceClockLow();
        }
        if (pioLease_.pio) resources.ReleasePio(pioLease_);
        if (gpioClaimed_) {
            resources.ReleaseGpios(kGpioMask, HardwareResources::Owner::Display);
            gpioClaimed_ = false;
        }
        res_ = nullptr;
        state_ = State::Inactive;  // no resources held: driver is back to inactive
    }

    HardwareResources* res_ = nullptr;
    DisplayConfig   cfg_{};
    DisplaySurface  surface_{};
    DisplayMapping  mapping_{};
    uint8_t*        stream_         = nullptr;
    size_t          workspaceBytes_ = 0;
    uint32_t        pixelCount_     = 0;
    uint32_t        padBytes_       = 0;
    uint32_t        wordCount_      = 0;
    uint32_t        encodeCursor_   = 0;
    uint32_t        frame_          = 0;
    uint64_t        streamDeadline_ = 0;
    HardwareResources::PioLease pioLease_{};
    int             dmaChannel_     = -1;
    bool            gpioClaimed_    = false;
    bool            samplerLive_    = false;
    bool            quiesced_       = false;
    bool            committed_      = false;
    uint8_t         frameBrightness_ = 255;
    State           state_          = State::Inactive;
    DisplayEvent    event_{};
    bool            eventPending_   = false;
    alignas(DisplayPixelSampler) uint8_t samplerStorage_[sizeof(DisplayPixelSampler)]{};
};

CustomArrayDriver gCustomArrayDriver;

} // namespace

DisplayDriver& CustomArrayBackend() { return gCustomArrayDriver; }

#endif // PICO_ON_DEVICE
