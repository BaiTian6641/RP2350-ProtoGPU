/**
 * @file gpu_clock.h
 * @brief Dynamic clock management for the RP2350 GPU.
 *
 * Provides runtime frequency adjustment with automatic voltage scaling,
 * PIO clock divider recalculation, and thermal throttling.
 *
 * RP2350 PLL System Architecture:
 *   XOSC (12 MHz) → PLL SYS → clk_sys (default 150 MHz)
 *   VCO range: 750–1600 MHz.  Post-dividers: 1–7 each.
 *   Output = XOSC × FBDIV / (PD1 × PD2)
 *
 * Known Stable Overclock Points:
 *   150 MHz — default, guaranteed, 1.10V
 *   200 MHz — widely stable, 1.10V
 *   250 MHz — very reliable, 1.10–1.15V
 *   300 MHz — generally stable, 1.15–1.20V (recommended max for ProtoGL)
 *   350 MHz — silicon lottery, 1.25–1.30V
 *   400 MHz — unreliable, needs >1.30V + voltage limit bypass
 *
 * Peripheral Impact:
 *   PIO:  derives from clk_sys — MUST recalculate SM clock dividers
 *   I2C:  derives from clk_peri (48 MHz USB PLL) — unaffected by default
 *   ADC:  derives from clk_adc (48 MHz USB PLL) — unaffected
 *   DMA:  derives from clk_sys — throughput scales automatically
 */

#pragma once

#include <cstdint>

namespace GpuClock {

// ─── Overclock Profile ──────────────────────────────────────────────────────

struct ClockProfile {
    uint16_t freqMHz;     // Target system clock
    uint8_t  vregLevel;   // VREG voltage enum (e.g., VREG_VOLTAGE_1_20)
    uint16_t vcoMHz;      // PLL VCO frequency
    uint8_t  postDiv1;    // PLL post-divider 1
    uint8_t  postDiv2;    // PLL post-divider 2
};

/// Pre-validated clock profiles — guaranteed to produce exact frequencies
/// with valid PLL parameters (VCO in 750–1600 MHz range).
static constexpr ClockProfile kProfiles[] = {
    // freq   vreg  vco  pd1  pd2
    {  150,   0x0B, 1500, 5, 2 },  // VREG_VOLTAGE_1_10 — default
    {  200,   0x0B, 1200, 6, 1 },  // VREG_VOLTAGE_1_10 — stock voltage OK
    {  250,   0x0C, 1500, 6, 1 },  // VREG_VOLTAGE_1_15 — margined
    {  266,   0x0C, 1596, 6, 1 },  // VREG_VOLTAGE_1_15
    {  300,   0x0D, 1500, 5, 1 },  // VREG_VOLTAGE_1_20 — recommended max
    {  360,   0x0F, 1440, 4, 1 },  // VREG_VOLTAGE_1_30 — aggressive overclock
};

static constexpr uint8_t kProfileCount = sizeof(kProfiles) / sizeof(kProfiles[0]);

// ─── Thermal Throttle Thresholds ────────────────────────────────────────────

static constexpr float kThrottleThreshold  = 80.0f;  // °C — begin throttle-down
static constexpr float kRecoverThreshold   = 65.0f;  // °C — resume full clock
static constexpr float kShutdownThreshold  = 95.0f;  // °C — emergency min clock

// ─── API ────────────────────────────────────────────────────────────────────

/**
 * @brief Initialise the clock manager.
 *
 * Call once at startup, BEFORE PIO state machines are started.
 * Sets the initial clock from gpu_config.h SYSTEM_CLOCK_MHZ.
 */
void Initialize();

/**
 * @brief Set the system clock to a new frequency.
 *
 * Adjusts VREG voltage, reconfigures PLL, and (optionally) recalculates
 * PIO clock dividers for HUB75 and Octal SPI.
 *
 * @param targetMHz       Desired frequency in MHz.
 * @param voltageLevel    VREG voltage enum (0 = auto-select from profile table).
 * @param reconfigurePIO  If true, pauses PIO SMs, recalculates dividers, resumes.
 * @return Actual achieved frequency in MHz (may differ if target is unsupported).
 */
uint16_t SetFrequency(uint16_t targetMHz, uint8_t voltageLevel = 0,
                      bool reconfigurePIO = true);

/**
 * @brief Get the current system clock frequency in MHz.
 */
uint16_t GetCurrentMHz();

/**
 * @brief Thermal check — call once per frame from Core 0.
 *
 * Reads the on-chip temperature sensor and automatically throttles
 * the clock if the die temperature exceeds kThrottleThreshold.
 * Restores original frequency when temperature drops below kRecoverThreshold.
 *
 * @param currentTempC  Current die temperature (pass in from I2CSlave::ReadTemperature()).
 * @return true if clock was changed this call.
 */
bool ThermalCheck(float currentTempC);

/**
 * @brief Get the profile index for a given frequency.
 * @return Profile index, or -1 if not found.
 */
int FindProfile(uint16_t targetMHz);

/**
 * @brief Check if thermal throttling is currently active.
 */
bool IsThrottled();

}  // namespace GpuClock
