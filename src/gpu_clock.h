/**
 * @file gpu_clock.h
 * @brief Deferred, gate-checked clock profile management for the RP2350 GPU (P09).
 *
 * Published profile table. Boot stays at 150 MHz / 1.1 V. Frequencies above
 * 150 MHz are explicit opt-in profiles at 1.2 V (VSEL setpoint, not measured
 * DVDD); the 300 MHz profile records the integration owner's tested board
 * result, and 336 MHz is an explicitly requested 48×7 profile. Neither is a
 * vendor qualification:
 *
 *   ID  MHz  VCO    pd1/pd2  VSEL
 *    0  150  1500   5/2      1.1 V (baseline)
 *    1  100  1200   6/2      1.1 V
 *    2   75  1500   5/4      1.1 V
 *    3  125  1500   6/2      1.1 V
 *    4  240  1200   5/1      1.2 V (48×5)
 *    5  288  1152   4/1      1.2 V (48×6)
 *    6  250  1500   6/1      1.2 V
 *    7  300  1200   4/1      1.2 V (user's board reference)
 *    8  336  1344   4/1      1.2 V (48×7, requested)
 *
 * Model:
 *  - RequestProfile() is DEFERRED: it only records the logical requested
 *    profile. The physical switch happens in TryApply(), which the parent
 *    calls at its maintenance point. If any gate is false TryApply returns
 *    Busy and changes NOTHING — it never silently applies.
 *  - Both requested and actual profile are observable, plus the thermal
 *    override, transition state and the last result (Snapshot).
 *  - Thermal policy is OFF until explicitly configured (typed thresholds
 *    with hysteresis validity). An override never loses the requested
 *    profile; throttle/recovery changes go through the same TryApply gate.
 *    A critical fault does NOT touch clocks mid-waveform: it raises a flag
 *    the parent consumes to perform safe blank/reset itself.
 *
 * Target transition (performed inside TryApply, core 0, all gates drained):
 *   0. verify the voltage limit is enabled and the fixed clock tree is in
 *      place: clk_ref XOSC 12 MHz; clk_peri/HSTX 48 MHz from PLL_USB;
 *      clk_usb/ADC 48 MHz from PLL_USB; system clock is the only changing domain
 *   1. establish the required bounded VSEL BEFORE any upward clock change
 *      (1.1 V <=150 MHz, 1.2 V >150 MHz); never disable the limit
 *   2. (increase) commit conservative flash M0 CLKDIV + dummy uncached
 *      flash read + DSB/ISB fence  [FLASH_LOCAL only]
 *   3. hooks.qmiPrepare(newHz)        — peer gpumem::qmiPsramClockPrepare
 *   4. SRAM-resident quiescent PLL switch (clk_sys → USB PLL 48 MHz,
 *      re-lock pll_sys, clk_sys → pll_sys); verify resulting Hz
 *   5. tick-generator re-arm/verify   — TICKS run from clk_ref (12 MHz,
 *      untouched), so TIMER/SysTick wall time and deadlines are preserved
 *   6. hooks.qmiFinalize(actualHz)    — peer gpumem::qmiPsramClockFinalize;
 *      NOTE its psram_reinitialize() → flash_start_xip() restores the
 *      boot2 M0 divider, which is why step 7 is mandatory
 *   7. re-commit flash M0 CLKDIV + dummy uncached read + fence
 *      [FLASH_LOCAL only; RAM_HOST never dereferences absent flash]
 *   8. hooks.retimeClients(actualHz)  — display Resume, device I2C retime,
 *      transport ceilings (parent-registered)
 *   9. (decrease) set 1.1 V only after the slower clock/client retime
 *
 * Native tests inject a Platform; the state machine, gating, thermal
 * hysteresis and ordering logic under test is the real shipping code.
 */

#pragma once

#include <cstdint>
#include <PglRuntimeProtocol.h>
namespace GpuClock {

// ─── Profile table ──────────────────────────────────────────────────────────

struct Profile {
    uint8_t  id;
    uint16_t freqMHz;
    uint32_t vcoHz;       ///< PLL VCO (750–1600 MHz, per RP2350 datasheet)
    uint8_t  postDiv1;    ///< 1..7, >= postDiv2
    uint8_t  postDiv2;    ///< 1..7
    uint16_t coreMillivolts; ///< required VSEL setpoint, not measured DVDD
};

constexpr uint8_t kProfileCount       = 9;
constexpr uint8_t kProfileBaseline150 = 0;
constexpr uint8_t kProfile100         = 1;
constexpr uint8_t kProfile75          = 2;
constexpr uint8_t kProfile125         = 3;
constexpr uint8_t kProfile240         = 4;
constexpr uint8_t kProfile288         = 5;
constexpr uint8_t kProfile250         = 6;
constexpr uint8_t kProfile300         = 7;
constexpr uint8_t kProfile336         = 8;
constexpr uint8_t kProfileInvalid     = 0xFF;

inline constexpr Profile kProfiles[kProfileCount] = {
    // id                  freq  VCO Hz        pd1  pd2  VSEL setpoint
    { kProfileBaseline150,  150, 1500000000u,   5,   2,   1100 },  // fbdiv 125
    { kProfile100,          100, 1200000000u,   6,   2,   1100 },  // fbdiv 100
    { kProfile75,            75, 1500000000u,   5,   4,   1100 },  // fbdiv 125
    { kProfile125,          125, 1500000000u,   6,   2,   1100 },  // fbdiv 125
    { kProfile240,          240, 1200000000u,   5,   1,   1200 },  // fbdiv 100
    { kProfile288,          288, 1152000000u,   4,   1,   1200 },  // fbdiv 96
    { kProfile250,          250, 1500000000u,   6,   1,   1200 },  // fbdiv 125
    { kProfile300,          300, 1200000000u,   4,   1,   1200 },  // fbdiv 100
    { kProfile336,          336, 1344000000u,   4,   1,   1200 },  // fbdiv 112
};
static_assert(PglRuntime::ClockProfileCount == kProfileCount &&
              PglRuntime::ClockProfileMask == 0x1ffu, "host/firmware clock profile IDs");

// VCO range/post-divider limits from the RP2350 datasheet PLL chapter;
// output equality guarantees the tuple produces the exact published rate.
static_assert(kProfiles[0].vcoHz / (kProfiles[0].postDiv1 * kProfiles[0].postDiv2) == 150000000u &&
              kProfiles[1].vcoHz / (kProfiles[1].postDiv1 * kProfiles[1].postDiv2) == 100000000u &&
              kProfiles[2].vcoHz / (kProfiles[2].postDiv1 * kProfiles[2].postDiv2) == 75000000u &&
              kProfiles[3].vcoHz / (kProfiles[3].postDiv1 * kProfiles[3].postDiv2) == 125000000u &&
              kProfiles[4].vcoHz / (kProfiles[4].postDiv1 * kProfiles[4].postDiv2) == 240000000u &&
              kProfiles[5].vcoHz / (kProfiles[5].postDiv1 * kProfiles[5].postDiv2) == 288000000u &&
              kProfiles[6].vcoHz / (kProfiles[6].postDiv1 * kProfiles[6].postDiv2) == 250000000u &&
              kProfiles[7].vcoHz / (kProfiles[7].postDiv1 * kProfiles[7].postDiv2) == 300000000u &&
              kProfiles[8].vcoHz / (kProfiles[8].postDiv1 * kProfiles[8].postDiv2) == 336000000u,
              "profile tuples must produce the exact published rates");
static_assert(kProfiles[0].vcoHz >= 750000000u && kProfiles[0].vcoHz <= 1600000000u &&
              kProfiles[1].vcoHz >= 750000000u && kProfiles[1].vcoHz <= 1600000000u &&
              kProfiles[2].vcoHz >= 750000000u && kProfiles[2].vcoHz <= 1600000000u &&
              kProfiles[3].vcoHz >= 750000000u && kProfiles[3].vcoHz <= 1600000000u &&
              kProfiles[4].vcoHz >= 750000000u && kProfiles[4].vcoHz <= 1600000000u &&
              kProfiles[5].vcoHz >= 750000000u && kProfiles[5].vcoHz <= 1600000000u &&
              kProfiles[6].vcoHz >= 750000000u && kProfiles[6].vcoHz <= 1600000000u &&
              kProfiles[7].vcoHz >= 750000000u && kProfiles[7].vcoHz <= 1600000000u &&
              kProfiles[8].vcoHz >= 750000000u && kProfiles[8].vcoHz <= 1600000000u,
              "VCO outside datasheet range");
static_assert(kProfiles[0].postDiv1 >= kProfiles[0].postDiv2 &&
              kProfiles[1].postDiv1 >= kProfiles[1].postDiv2 &&
              kProfiles[2].postDiv1 >= kProfiles[2].postDiv2 &&
              kProfiles[3].postDiv1 >= kProfiles[3].postDiv2 &&
              kProfiles[4].postDiv1 >= kProfiles[4].postDiv2 &&
              kProfiles[5].postDiv1 >= kProfiles[5].postDiv2 &&
              kProfiles[6].postDiv1 >= kProfiles[6].postDiv2 &&
              kProfiles[7].postDiv1 >= kProfiles[7].postDiv2 &&
              kProfiles[8].postDiv1 >= kProfiles[8].postDiv2,
              "postDiv1 must be >= postDiv2 (PLL application note)");

/// Conservative flash M0 SCK ceiling: clk_sys/4 at the 150 MHz baseline
/// (37.5 MHz). Keeping the sys:SCK ratio at >= 4 also keeps the boot2
/// RXDELAY sample point at the same fraction of the SCK period for every
/// profile. Real NOR qualification remains HIL-blocked.
constexpr uint32_t kFlashSafeSckHz = 37500000u;
constexpr uint8_t  kFlashMinClkDiv = 4;
constexpr uint32_t kNominalCoreMillivolts = 1100, kOverclockCoreMillivolts = 1200;
constexpr uint32_t kPeripheralFixedHz = 48000000, kReferenceFixedHz = 12000000;

bool            IsValidProfile(uint8_t id);
const Profile&  GetProfile(uint8_t id);  ///< IsValidProfile(id) must hold.
/// QMI M0 CLKDIV safe for any clk_sys <= hz. Pure logic (unit-tested).
/// ceil(sysHz / kFlashSafeSckHz) clamped to [kFlashMinClkDiv, 255]; with
/// kFlashMinClkDiv == 4 the sys:SCK ratio never drops below 4, so flash SCK
/// stays <= 37.5 MHz AND the boot2 RXDELAY sample point keeps the same
/// fraction of the SCK period at every supported profile.
inline constexpr uint8_t FlashClkDivFor(uint32_t sysHz) {
    uint32_t div = (sysHz + kFlashSafeSckHz - 1u) / kFlashSafeSckHz;
    if (div < kFlashMinClkDiv) div = kFlashMinClkDiv;
    if (div > 255u) div = 255u;
    return static_cast<uint8_t>(div);
}
static_assert(FlashClkDivFor(150000000u) == 4, "150 MHz -> M0 div 4");
static_assert(FlashClkDivFor(100000000u) == 4, "100 MHz -> M0 div 4");
static_assert(FlashClkDivFor(75000000u) == 4, "75 MHz -> M0 div 4");
static_assert(FlashClkDivFor(125000000u) == 4, "125 MHz -> M0 div 4");
static_assert(FlashClkDivFor(240000000u) == 7, "240 MHz -> M0 div 7");
static_assert(FlashClkDivFor(288000000u) == 8, "288 MHz -> M0 div 8");
static_assert(FlashClkDivFor(250000000u) == 7, "250 MHz -> M0 div 7");
static_assert(FlashClkDivFor(300000000u) == 8, "300 MHz -> M0 div 8");
static_assert(FlashClkDivFor(336000000u) == 9, "336 MHz -> M0 div 9");

// ─── Observable state ───────────────────────────────────────────────────────

enum class Transition : uint8_t {
    Idle = 0,           ///< no pending change
    AwaitingSafePoint,  ///< requested/effective target != actual
    Applying,           ///< inside TryApply (never observable single-threaded)
    Fault,              ///< clock verify failed; parent must safe-blank/reset
};

enum class Override : uint8_t {
    None = 0,
    ThermalThrottle,    ///< policy-driven lower profile; requested preserved
};

struct Snapshot {
    uint8_t   requested;        ///< logical requested profile id
    uint8_t   actual;           ///< applied profile id (kProfileInvalid if unknown/Fault)
    uint32_t  actualHz;         ///< measured clk_sys after last apply
    Override  override;         ///< active override (requested NOT overwritten)
    Transition transition;
    PglRuntime::Result lastResult;
    bool      thermalEnabled;
    int16_t   lastTemperatureC;
    bool      criticalFaultPending;  ///< parent: safe blank + reset requested
};

// ─── Gates and hooks (parent-owned integration points) ─────────────────────

/// Maintenance-point preconditions. All must hold for a physical switch.
struct SafeGates {
    bool workersParked     = false;  ///< core1 parked in RAM, render jobs done
    bool hostIdle          = false;  ///< transport quiesced, credits frozen
    bool reservationArmed  = false;  ///< any armed bulk reservation blocks
    bool displayDrained    = false;  ///< Output.Quiesce() done (blank/latch safe)
    bool devicesDrained    = false;  ///< DeviceService::IsIdle()
    bool memoryDrained     = false;  ///< no DMA/XIP/PSRAM user active
};

/// Pure logic: all gates satisfied (unit-tested).
bool GatesSafe(const SafeGates& g);

/// Parent steps executed at the correct points of the transition. All may be
/// null (feature absent). Returning false aborts (prepare) or flags (finalize
/// / retime) the apply — see TryApply contract.
struct ApplyHooks {
    void* ctx = nullptr;
    /// Before PLL switch (step 2). Typically gpumem::qmiPsramClockPrepare.
    bool (*qmiPrepare)(void* ctx, uint32_t newHz) = nullptr;
    /// After PLL settle + tick rearm (step 5). gpumem::qmiPsramClockFinalize.
    bool (*qmiFinalize)(void* ctx, uint32_t actualHz) = nullptr;
    /// Last step: retime every clk_sys-derived client (display Resume(newHz),
    /// DeviceService::RetimeForClock, transport rate ceilings).
    bool (*retimeClients)(void* ctx, uint32_t actualHz) = nullptr;
};

// ─── Platform seam (target hardware vs native logic tests) ─────────────────

struct Platform {
    void* ctx = nullptr;
    /// Current measured clk_sys in Hz.
    uint32_t (*sysClockHz)(void* ctx) = nullptr;
    /// Quiescent PLL switch to vco/pd1/pd2. On target this is SRAM-resident
    /// and performs no flash fetch; must leave clk_peri on the 48 MHz USB
    /// PLL and clk_ref untouched.
    bool (*switchSysClockPll)(void* ctx, uint32_t vcoHz, uint8_t pd1, uint8_t pd2) = nullptr;
    /// Set required core VSEL setpoint and verify the register completed. Must
    /// reject values outside 1.1/1.2 V and never disable the voltage limit.
    bool (*setCoreMillivolts)(void* ctx, uint16_t millivolts) = nullptr;
    /// Actual VSEL setpoint derived from hardware, 0 if unavailable/unknown.
    uint16_t (*coreMillivolts)(void* ctx) = nullptr;
    /// Commit conservative flash M0 CLKDIV + dummy uncached flash read +
    /// DSB/ISB fence. On RAM_HOST (PICO_NO_FLASH) this must be a no-op that
    /// NEVER dereferences the absent flash. Returns false on failure.
    bool (*commitFlashDivisor)(void* ctx, uint8_t divisor) = nullptr;
    /// Verify/re-arm every TICKS generator at clk_ref/1MHz so wall time and
    /// absolute deadlines are preserved across the switch.
    bool (*preserveTimebase)(void* ctx) = nullptr;
    /// Wall clock for transition bookkeeping (us).
    uint64_t (*timeUs)(void* ctx) = nullptr;
    /// Current actual clocks. Fixed domains must be verified before any switch.
    uint32_t (*peripheralHz)(void* ctx) = nullptr;
    uint32_t (*referenceHz)(void* ctx) = nullptr;
    uint32_t (*usbHz)(void* ctx) = nullptr;
    uint32_t (*adcHz)(void* ctx) = nullptr;
    uint32_t (*hstxHz)(void* ctx) = nullptr;
    /// Verify hardware routing: ref=12 MHz XOSC; peri/HSTX/USB/ADC=48 MHz
    /// PLL_USB. The fixed domains must not be sourced from or track clk_sys.
    bool (*verifyFixedDomains)(void* ctx) = nullptr;
};

#if defined(PICO_ON_DEVICE)
/// Real RP2350 platform (SRAM-safe PLL switch, QMI M0 commit, TICKS re-arm).
Platform TargetPlatform();
/// Zero-arg init for firmware: uses TargetPlatform().
void Initialize();
#endif

// ─── Lifecycle ──────────────────────────────────────────────────────────────

/// Initialize with an explicit platform (native tests, or firmware passing
/// TargetPlatform()). Reads the current clock and maps it to a profile;
/// an unlisted boot clock maps to actual = kProfileInvalid with requested =
/// baseline (apply will normalize at the first safe point).
void Initialize(const Platform& platform);

// ─── Requests (deferred) ────────────────────────────────────────────────────

/// Record a new logical requested profile. Never touches hardware.
/// Ok            — accepted (idempotent: re-requesting the effective target
///                 while no transition pending is a no-op Ok)
/// InvalidValue  — unknown profile id
/// Busy          — a physical apply is in progress (TryApply on the stack)
PglRuntime::Result RequestProfile(uint8_t profileId);

/// Attempt the pending physical switch at the parent's maintenance point.
/// Voltage follows the profile: it is raised before any upward clock change
/// and reduced only after a downward clock change has completed safely.
///  - No pending change            → Ok (no-op)
///  - Gates not all true           → Busy, NOTHING changed
///  - qmiPrepare/retime failure    → Io, clock unchanged (prepare) or
///                                   applied-but-flagged (see Snapshot)
///  - Clock verify failure         → Io, transition = Fault (parent resets)
///  - Success                      → Ok, actual updated and published
PglRuntime::Result TryApply(const SafeGates& gates, const ApplyHooks& hooks);

/// Fill one published descriptor/actual-domain record. Current selector
/// 0xff uses actual profile when known, otherwise requested. Unknown/invalid
/// selectors produce InvalidValue; current-unknown produces the requested
/// descriptor with Available clear and all actual values zero.
PglRuntime::Result GetClockConfiguration(uint8_t selector,
                                         PglRuntime::ClockConfiguration& out);

/// Effective target: override profile when throttled, else requested.
uint8_t EffectiveTarget();

Snapshot GetSnapshot();

// ─── Thermal policy (OFF until configured + calibrated) ────────────────────

struct ThermalPolicy {
    bool     enabled = false;
    int16_t  throttleOnC = 0;      ///< engage override at/above
    int16_t  recoverC = 0;         ///< release override at/below (< throttleOnC)
    int16_t  criticalC = 0;        ///< request parent safe blank/reset at/above
    uint8_t  throttleProfile = kProfile75;  ///< valid profile, <= baseline heat
};

constexpr int16_t kThermalMinC = -40;
constexpr int16_t kThermalMaxC = 125;
constexpr int16_t kThermalMinHysteresisC = 2;

/// Validate + install policy. InvalidValue on bad ordering/range/profile;
/// the previous policy is retained on failure. Disabling clears any active
/// thermal override (requested profile is restored through the normal gate).
PglRuntime::Result ConfigureThermal(const ThermalPolicy& policy);

/// Feed a temperature sample (from the ADC device service). With the policy
/// disabled this only records the sample. Enabled: engages/releases the
/// override with hysteresis (change deferred to TryApply) and raises the
/// critical-fault flag at/above criticalC. Never applies clocks directly.
PglRuntime::Result ThermalSample(int16_t temperatureC);

/// Parent polls after a critical sample; true exactly once per fault until
/// the parent has performed its safe blank/reset and calls ClearCriticalFault().
bool ConsumeCriticalFault();
/// Parent calls after its safe blank/reset completed (post-reset re-init
/// also clears implicitly via Initialize()).
void ClearCriticalFault();

// ─── Idle wait ──────────────────────────────────────────────────────────────

/// Safe check-to-sleep closure. On target: disables interrupts, re-checks
/// hasPendingWork, enters WFI only when truly idle, restores interrupts
/// (an interrupt arriving between check and WFI still wakes the CPU; the
/// handler runs after restore — no lost-wakeup race). Returns true if the
/// CPU actually entered WFI. Core 0 only; parent supplies the predicate
/// (frame credits, ingress FIFO, device queue, pending clock request...).
/// Native builds never sleep (returns false) — logic tests assert the
/// predicate is consulted and no sleep occurs while work is pending.
bool SleepUntilEvent(void* ctx, bool (*hasPendingWork)(void* ctx));

}  // namespace GpuClock
