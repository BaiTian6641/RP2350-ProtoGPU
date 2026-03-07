/**
 * @file gpu_clock.cpp
 * @brief Dynamic clock management implementation for RP2350 GPU.
 *
 * Uses the Pico SDK PLL / VREG APIs for glitch-free frequency transitions.
 * The set_sys_clock_pll() function internally:
 *   1. Switches clk_sys to 48 MHz USB PLL (glitchless mux)
 *   2. Reinitialises pll_sys with new VCO/post-divider parameters
 *   3. Switches clk_sys back to the reconfigured PLL
 *   4. Leaves clk_peri on USB PLL (I2C/SPI timing unaffected)
 *
 * PIO state machines derive their execution clock from clk_sys, so their
 * clock dividers must be recalculated after a frequency change to maintain
 * the correct bit timing for HUB75 and Octal SPI.
 */

#include "gpu_clock.h"
#include "gpu_config.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/pll.h"
#include "hardware/pio.h"
#include "hardware/uart.h"

#include <cstdio>

// ─── State ──────────────────────────────────────────────────────────────────

static uint16_t currentMHz_      = 0;
static uint16_t requestedMHz_    = 0;   // Pre-throttle target
static bool     throttled_       = false;
static int      currentProfile_  = -1;

// PIO SM original clock dividers (captured at Init, scaled on freq change)
// Stored as fixed-point 16.8 (integer.fraction)
struct PioSmState {
    PIO   pio;
    uint  sm;
    float baseDivider;    // Divider at GpuConfig::SYSTEM_CLOCK_MHZ
    bool  tracked;
};

// Track up to 4 PIO SMs (HUB75 uses 1, Octal SPI uses 1, future uses 2)
static PioSmState trackedSMs_[4] = {};
static uint8_t trackedCount_ = 0;

// ─── Helpers ────────────────────────────────────────────────────────────────

static void RecalculatePioClocks(uint16_t newMHz) {
    // PIO SM divider should scale proportionally to maintain the same bit rate:
    //   new_div = base_div × (newMHz / baseMHz)
    // Because the PIO clock is clk_sys, a faster sys_clock needs a larger divider.
    float ratio = static_cast<float>(newMHz) / static_cast<float>(GpuConfig::SYSTEM_CLOCK_MHZ);

    for (uint8_t i = 0; i < trackedCount_; ++i) {
        PioSmState& s = trackedSMs_[i];
        if (!s.tracked) continue;

        float newDiv = s.baseDivider * ratio;
        if (newDiv < 1.0f) newDiv = 1.0f;

        pio_sm_set_enabled(s.pio, s.sm, false);
        pio_sm_set_clkdiv(s.pio, s.sm, newDiv);
        pio_sm_set_enabled(s.pio, s.sm, true);
    }
}

static void RefreshUartBaudAfterClockChange() {
#if defined(PICO_DEFAULT_UART) && defined(PICO_DEFAULT_UART_BAUD_RATE)
    uart_inst_t* uartInst = (PICO_DEFAULT_UART == 0) ? uart0 : uart1;
    uart_set_baudrate(uartInst, PICO_DEFAULT_UART_BAUD_RATE);
#endif
}

// ─── API Implementation ─────────────────────────────────────────────────────

void GpuClock::Initialize() {
    currentMHz_   = static_cast<uint16_t>(clock_get_hz(clk_sys) / 1000000);
    requestedMHz_ = currentMHz_;
    throttled_    = false;
    trackedCount_ = 0;

    // Find matching profile for current clock
    currentProfile_ = FindProfile(currentMHz_);

    printf("[CLOCK] Initialised at %u MHz (profile %d)\n", currentMHz_, currentProfile_);
}

uint16_t GpuClock::SetFrequency(uint16_t targetMHz, uint8_t voltageLevel,
                                 bool reconfigurePIO) {
    // Find the closest known-good profile
    int profileIdx = FindProfile(targetMHz);
    if (profileIdx < 0) {
        // Find the closest profile that doesn't exceed the target
        int bestIdx = 0;
        for (int i = kProfileCount - 1; i >= 0; --i) {
            if (kProfiles[i].freqMHz <= targetMHz) {
                bestIdx = i;
                break;
            }
        }
        profileIdx = bestIdx;
    }

    const ClockProfile& prof = kProfiles[profileIdx];

    // Nothing to do if already at this frequency
    if (prof.freqMHz == currentMHz_ && !throttled_) {
        return currentMHz_;
    }

    printf("[CLOCK] Changing: %u MHz → %u MHz (VREG=0x%02X, VCO=%u, PD1=%u, PD2=%u)\n",
           currentMHz_, prof.freqMHz,
           voltageLevel ? voltageLevel : prof.vregLevel,
           prof.vcoMHz, prof.postDiv1, prof.postDiv2);

    // Step 1: Set voltage (raise before increasing clock, lower after decreasing)
    uint8_t vreg = voltageLevel ? voltageLevel : prof.vregLevel;
    bool goingUp = prof.freqMHz > currentMHz_;

    if (goingUp) {
        vreg_set_voltage(static_cast<vreg_voltage>(vreg));
        sleep_ms(10);  // Voltage settling time
    }

    // Step 2: Set system clock via PLL (glitch-free internally)
    set_sys_clock_pll(static_cast<uint32_t>(prof.vcoMHz) * 1000000u,
                      prof.postDiv1, prof.postDiv2);

    // Step 3: Lower voltage if slowing down
    if (!goingUp) {
        vreg_set_voltage(static_cast<vreg_voltage>(vreg));
    }

    // Step 4: Update state
    currentMHz_     = prof.freqMHz;
    currentProfile_ = profileIdx;

    // Step 5: Recalculate PIO clock dividers if requested
    if (reconfigurePIO) {
        RecalculatePioClocks(currentMHz_);
    }

    // Step 6: Refresh UART baud divider so stdio serial stays stable
    // after clk_sys/clock tree changes.
    RefreshUartBaudAfterClockChange();

    printf("[CLOCK] Now running at %u MHz (verified: %lu Hz)\n",
           currentMHz_, (unsigned long)clock_get_hz(clk_sys));

    return currentMHz_;
}

uint16_t GpuClock::GetCurrentMHz() {
    return currentMHz_;
}

bool GpuClock::ThermalCheck(float currentTempC) {
    if (currentTempC >= kShutdownThreshold) {
        // Emergency: drop to minimum clock
        if (currentMHz_ != kProfiles[0].freqMHz) {
            printf("[CLOCK] THERMAL EMERGENCY: %.1f°C — dropping to %u MHz\n",
                   currentTempC, kProfiles[0].freqMHz);
            SetFrequency(kProfiles[0].freqMHz);
            throttled_ = true;
            return true;
        }
        return false;
    }

    if (currentTempC >= kThrottleThreshold && !throttled_) {
        // Throttle: step down one profile
        int targetProfile = (currentProfile_ > 0) ? currentProfile_ - 1 : 0;
        printf("[CLOCK] THERMAL THROTTLE: %.1f°C — stepping down to %u MHz\n",
               currentTempC, kProfiles[targetProfile].freqMHz);
        SetFrequency(kProfiles[targetProfile].freqMHz);
        throttled_ = true;
        return true;
    }

    if (throttled_ && currentTempC < kRecoverThreshold) {
        // Recover: restore original requested frequency
        printf("[CLOCK] THERMAL RECOVER: %.1f°C — restoring %u MHz\n",
               currentTempC, requestedMHz_);
        SetFrequency(requestedMHz_);
        throttled_ = false;
        return true;
    }

    return false;
}

int GpuClock::FindProfile(uint16_t targetMHz) {
    for (int i = 0; i < kProfileCount; ++i) {
        if (kProfiles[i].freqMHz == targetMHz) return i;
    }
    return -1;
}

bool GpuClock::IsThrottled() {
    return throttled_;
}
