/**
 * @file gpu_clock.cpp
 * @brief Deferred clock profile management implementation (P09).
 *
 * Frequency and bounded VSEL sequencing are platform-independent and tested
 * natively. The physical transition (regulator VSEL, QMI M0 divisor commit,
 * SRAM-resident PLL switch, TICKS re-arm, fixed-domain verification) is
 * RP2350 on-device only, injected through GpuClock::Platform. The voltage
 * limit and over-temperature protection remain enabled.
 */

#include "gpu_clock.h"

#if defined(PICO_ON_DEVICE)
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/resets.h"
#include "hardware/sync.h"
#include "hardware/ticks.h"
#include "hardware/vreg.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/ticks.h"
#include "hardware/regs/qmi.h"
#include "hardware/regs/addressmap.h"
#endif

namespace GpuClock {

// ─── Profile table ──────────────────────────────────────────────────────────


bool IsValidProfile(uint8_t id) { return id < kProfileCount; }

const Profile& GetProfile(uint8_t id) { return kProfiles[id]; }


bool GatesSafe(const SafeGates& g) {
    return g.workersParked && g.hostIdle && !g.reservationArmed &&
           g.displayDrained && g.devicesDrained && g.memoryDrained;
}

// ─── State ──────────────────────────────────────────────────────────────────

static Platform  platform_;
static uint8_t   requested_      = kProfileBaseline150;
static uint8_t   actual_         = kProfileInvalid;
static uint32_t  actualHz_       = 0;
static Override  override_       = Override::None;
static Transition transition_    = Transition::Idle;
static PglRuntime::Result lastResult_ = PglRuntime::Result::Ok;
static ThermalPolicy policy_{};
static int16_t   lastTempC_      = 0;
static bool      criticalPending_ = false;

static bool FixedDomainsVerified() {
    return !platform_.verifyFixedDomains || platform_.verifyFixedDomains(platform_.ctx);
}
static uint32_t ClockHz(uint32_t (*clockFn)(void*)) {
    return clockFn ? clockFn(platform_.ctx) : 0;
}
static bool SetCoreVoltage(uint16_t millivolts) {
    if (!platform_.setCoreMillivolts ||
        !platform_.setCoreMillivolts(platform_.ctx, millivolts)) return false;
    return !platform_.coreMillivolts || platform_.coreMillivolts(platform_.ctx) == millivolts;
}
static uint16_t ActualCoreMillivolts() {
    return platform_.coreMillivolts ? platform_.coreMillivolts(platform_.ctx) : 0;
}
static bool ClockConfigurationFor(uint8_t target, bool knownActual,
                                  PglRuntime::ClockConfiguration& value) {
    if (!IsValidProfile(target)) return false;
    const auto& profile = GetProfile(target);
    value = {};
    value.profileId = target;
    value.flags = uint8_t(PglRuntime::ClockConfigurationFlag::Available);
    if (profile.freqMHz > 150) value.flags |= uint8_t(PglRuntime::ClockConfigurationFlag::Overclock);
    if (target == kProfile300) value.flags |= uint8_t(PglRuntime::ClockConfigurationFlag::UserBoardReference);
    value.coreMillivolts = profile.coreMillivolts;
    value.frequencyHz = profile.freqMHz * 1000000u;
    value.vcoHz = profile.vcoHz;
    value.postDiv1 = profile.postDiv1;
    value.postDiv2 = profile.postDiv2;
    value.supportedProfilesMask = PglRuntime::ClockProfileMask;
    if (knownActual) {
        value.actualCoreMillivolts = ActualCoreMillivolts();
        value.systemHz = actualHz_;
        value.peripheralHz = ClockHz(platform_.peripheralHz);
        value.referenceHz = ClockHz(platform_.referenceHz);
        value.usbHz = ClockHz(platform_.usbHz);
        value.adcHz = ClockHz(platform_.adcHz);
        value.hstxHz = ClockHz(platform_.hstxHz);
        if (FixedDomainsVerified()) {
            value.domainFlags |= uint16_t(PglRuntime::ClockDomainFlag::VerifiedFixedDomains);
        }
    }
    return true;
}

static int ProfileForHz(uint32_t hz) {
    for (uint8_t i = 0; i < kProfileCount; ++i) {
        if (kProfiles[i].freqMHz * 1000000u == hz) return static_cast<int>(i);
    }
    return -1;
}

static void MarkPendingIfTargetDiffers() {
    if (transition_ == Transition::Idle && EffectiveTarget() != actual_) {
        transition_ = Transition::AwaitingSafePoint;
    }
}

// ─── Lifecycle ──────────────────────────────────────────────────────────────

void Initialize(const Platform& platform) {
    platform_   = platform;
    requested_  = kProfileBaseline150;
    override_   = Override::None;
    policy_     = ThermalPolicy{};
    lastTempC_  = 0;
    criticalPending_ = false;
    lastResult_ = PglRuntime::Result::Ok;

    actualHz_ = platform_.sysClockHz ? platform_.sysClockHz(platform_.ctx) : 0;
    const int boot = ProfileForHz(actualHz_);
    if (boot >= 0) {
        actual_     = static_cast<uint8_t>(boot);
        requested_  = actual_;
        transition_ = Transition::Idle;
    } else {
        // Unlisted boot clock: actual unknown, normalize to baseline at the
        // first safe point.
        actual_     = kProfileInvalid;
        transition_ = Transition::AwaitingSafePoint;
    }
}

#if defined(PICO_ON_DEVICE)
void Initialize() { Initialize(TargetPlatform()); }
#endif

// ─── Requests ───────────────────────────────────────────────────────────────

uint8_t EffectiveTarget() {
    return (override_ == Override::ThermalThrottle) ? policy_.throttleProfile
                                                    : requested_;
}

PglRuntime::Result RequestProfile(uint8_t profileId) {
    if (!IsValidProfile(profileId)) {
        lastResult_ = PglRuntime::Result::InvalidValue;
        return lastResult_;
    }
    if (transition_ == Transition::Applying) {
        lastResult_ = PglRuntime::Result::Busy;
        return lastResult_;
    }
    requested_ = profileId;  // requested state survives thermal overrides
    MarkPendingIfTargetDiffers();
    lastResult_ = PglRuntime::Result::Ok;
    return lastResult_;
}

PglRuntime::Result TryApply(const SafeGates& gates, const ApplyHooks& hooks) {
    using Result = PglRuntime::Result;

    if (!platform_.sysClockHz || !platform_.switchSysClockPll) {
        lastResult_ = Result::BadState;
        return lastResult_;
    }
    if (transition_ == Transition::Fault) {
        lastResult_ = Result::BadState;  // parent must safe blank/reset
        return lastResult_;
    }
    if (transition_ == Transition::Applying) {
        lastResult_ = Result::Busy;
        return lastResult_;
    }
    const uint8_t target = EffectiveTarget();
    if (target == actual_ && actual_ != kProfileInvalid) {
        transition_ = Transition::Idle;  // nothing pending (idempotent no-op)
        lastResult_ = Result::Ok;
        return lastResult_;
    }
    if (!GatesSafe(gates)) {
        lastResult_ = Result::Busy;      // NEVER silently apply
        return lastResult_;
    }

    transition_ = Transition::Applying;
    const Profile& p = GetProfile(target);
    const uint32_t targetHz = static_cast<uint32_t>(p.freqMHz) * 1000000u;
    const uint32_t beforeHz = platform_.sysClockHz(platform_.ctx);
    const bool increasing = targetHz > beforeHz;
    const uint16_t targetVoltage = p.coreMillivolts;
    const uint8_t flashDiv = FlashClkDivFor(targetHz);
    const bool voltageIncrease = ActualCoreMillivolts() && ActualCoreMillivolts() < targetVoltage;
    const bool voltageDecrease = ActualCoreMillivolts() > targetVoltage;
    void* ctx = platform_.ctx;

    if (!FixedDomainsVerified()) {
        transition_ = Transition::AwaitingSafePoint;
        lastResult_ = Result::Io;
        return lastResult_;
    }
    if (voltageIncrease && !SetCoreVoltage(targetVoltage)) {
        transition_ = Transition::AwaitingSafePoint;  // regulator must precede speed
        lastResult_ = Result::Io;
        return lastResult_;
    }

    // Step 1: (increase) conservative flash M0 divisor + dummy uncached
    // flash read + DSB/ISB fence BEFORE raising clk_sys. No-op on RAM_HOST.
    if (increasing && platform_.commitFlashDivisor &&
        !platform_.commitFlashDivisor(ctx, flashDiv)) {
        transition_ = Transition::AwaitingSafePoint;  // clock unchanged
        lastResult_ = Result::Io;
        return lastResult_;
    }

    // Step 2: PSRAM M1 pre-retime (peer gpumem::qmiPsramClockPrepare).
    if (hooks.qmiPrepare && !hooks.qmiPrepare(hooks.ctx, targetHz)) {
        transition_ = Transition::AwaitingSafePoint;  // clock unchanged
        lastResult_ = Result::Io;
        return lastResult_;
    }

    // Step 3: quiescent PLL switch (SRAM-resident on target).
    if (!platform_.switchSysClockPll(ctx, p.vcoHz, p.postDiv1, p.postDiv2)) {
        transition_ = Transition::Fault;  // intermediate state unknown
        actual_     = kProfileInvalid;
        actualHz_   = platform_.sysClockHz(ctx);
        lastResult_ = Result::Io;
        return lastResult_;
    }

    // Verify the actual clock against the profile tuple.
    const uint32_t measured = platform_.sysClockHz(ctx);
    if (measured != targetHz) {
        transition_ = Transition::Fault;
        actual_     = kProfileInvalid;
        actualHz_   = measured;
        lastResult_ = Result::Io;
        return lastResult_;
    }
    actualHz_ = measured;

    // Step 4: preserve wall time/deadlines (TICKS re-arm verify).
    if (platform_.preserveTimebase && !platform_.preserveTimebase(ctx)) {
        transition_ = Transition::Fault;  // deadlines no longer trustworthy
        actual_     = kProfileInvalid;
        lastResult_ = Result::Io;
        return lastResult_;
    }

    Result applied = Result::Ok;

    // Step 5: PSRAM M1 full retime (peer gpumem::qmiPsramClockFinalize).
    // Its psram_reinitialize() -> flash_start_xip() restores the boot2 M0
    // divider, so step 6 is mandatory after it. A failure here means the
    // clock IS applied but PSRAM timing is stale — publish Io and let the
    // parent deinit PSRAM; never pretend the apply failed after the fact.
    if (hooks.qmiFinalize && !hooks.qmiFinalize(hooks.ctx, actualHz_)) {
        applied = Result::Io;
    }

    // Step 6: re-commit flash M0 divisor + dummy uncached read + fence.
    if (platform_.commitFlashDivisor &&
        !platform_.commitFlashDivisor(ctx, flashDiv)) {
        transition_ = Transition::Fault;  // flash timing not guaranteed
        actual_     = kProfileInvalid;
        lastResult_ = Result::Io;
        return lastResult_;
    }

    // Step 7: retime every clk_sys-derived client (display Resume, device
    // I2C baud, transport rate ceilings).
    if (hooks.retimeClients && !hooks.retimeClients(hooks.ctx, actualHz_)) {
        applied = Result::Io;
    }

    actual_     = target;
    transition_ = Transition::Idle;
    // have been verified/retimed.
    if (voltageDecrease && !SetCoreVoltage(targetVoltage)) applied = Result::Io;
    lastResult_ = applied;
    return lastResult_;
}

PglRuntime::Result GetClockConfiguration(uint8_t selector,
                                         PglRuntime::ClockConfiguration& out) {
    using Result = PglRuntime::Result;
    const bool current = selector == 0xff;
    uint8_t target;
    bool knownActual = false;
    if (current) {
        if (actual_ != kProfileInvalid) {
            target = actual_;
            knownActual = true;
        } else target = requested_;
    } else target = selector;
    if (!ClockConfigurationFor(target, knownActual, out)) {
        lastResult_ = Result::InvalidValue;
        return lastResult_;
    }
    if (current && !knownActual) {
        out.flags &= ~uint8_t(PglRuntime::ClockConfigurationFlag::Available);
        out.actualCoreMillivolts = out.systemHz = out.peripheralHz = out.referenceHz = 0;
        out.usbHz = out.adcHz = out.hstxHz = 0;
        out.domainFlags = 0;
    }
    lastResult_ = Result::Ok;
    return lastResult_;
}

Snapshot GetSnapshot() {
    Snapshot s;
    s.requested             = requested_;
    s.actual                = actual_;
    s.actualHz              = actualHz_;
    s.override              = override_;
    s.transition            = transition_;
    s.lastResult            = lastResult_;
    s.thermalEnabled        = policy_.enabled;
    s.lastTemperatureC      = lastTempC_;
    s.criticalFaultPending  = criticalPending_;
    return s;
}

// ─── Thermal policy ─────────────────────────────────────────────────────────

PglRuntime::Result ConfigureThermal(const ThermalPolicy& policy) {
    using Result = PglRuntime::Result;
    if (policy.enabled) {
        const bool rangesOk =
            policy.throttleOnC >= kThermalMinC && policy.throttleOnC <= kThermalMaxC &&
            policy.recoverC    >= kThermalMinC && policy.recoverC    <= kThermalMaxC &&
            policy.criticalC   >= kThermalMinC && policy.criticalC   <= kThermalMaxC;
        const bool orderingOk =
            policy.recoverC < policy.throttleOnC &&
            policy.throttleOnC - policy.recoverC >= kThermalMinHysteresisC &&
            policy.throttleOnC < policy.criticalC;
        if (!rangesOk || !orderingOk || !IsValidProfile(policy.throttleProfile)) {
            return Result::InvalidValue;  // previous policy retained
        }
    }
    policy_ = policy;
    if (!policy_.enabled && override_ == Override::ThermalThrottle) {
        override_ = Override::None;  // requested profile restored via the gate
        MarkPendingIfTargetDiffers();
    }
    return Result::Ok;
}

PglRuntime::Result ThermalSample(int16_t temperatureC) {
    lastTempC_ = temperatureC;
    if (!policy_.enabled) return PglRuntime::Result::Ok;  // off until configured

    if (temperatureC >= policy_.criticalC) {
        // NO clock change mid-waveform: parent performs safe blank/reset.
        criticalPending_ = true;
        return PglRuntime::Result::Ok;
    }
    if (override_ == Override::None && temperatureC >= policy_.throttleOnC) {
        override_ = Override::ThermalThrottle;  // requested_ untouched
        MarkPendingIfTargetDiffers();
    } else if (override_ == Override::ThermalThrottle &&
               temperatureC <= policy_.recoverC) {
        override_ = Override::None;
        MarkPendingIfTargetDiffers();
    }
    // Between recoverC and throttleOnC: hysteresis holds the current state.
    return PglRuntime::Result::Ok;
}

bool ConsumeCriticalFault() {
    const bool pending = criticalPending_;
    criticalPending_ = false;
    return pending;
}

void ClearCriticalFault() { criticalPending_ = false; }

// ─── Idle wait ──────────────────────────────────────────────────────────────

#if defined(PICO_ON_DEVICE)
bool SleepUntilEvent(void* ctx, bool (*hasPendingWork)(void* ctx)) {
    // Check-to-sleep closure: with interrupts masked, an event arriving
    // between the predicate check and WFI still wakes the core immediately
    // (WFI completes), and the handler runs after restore — no lost wakeup.
    const uint32_t save = save_and_disable_interrupts();
    const bool idle = !hasPendingWork || !hasPendingWork(ctx);
    if (idle) __wfi();
    restore_interrupts(save);
    return idle;
}
#else
bool SleepUntilEvent(void* ctx, bool (*hasPendingWork)(void* ctx)) {
    // Native builds never sleep; the predicate contract is still exercised.
    const bool idle = !hasPendingWork || !hasPendingWork(ctx);
    (void)idle;
    return false;
}
#endif

// ─── Target platform ────────────────────────────────────────────────────────

#if defined(PICO_ON_DEVICE)

namespace {

/// SRAM-resident quiescent PLL switch. Mirrors the SDK's set_sys_clock_pll()
/// clk_sys sequence with raw register writes only (every helper is a static
/// inline), so no flash text executes during the switch window. XIP remains
/// fetchable throughout because the M0 divisor committed before the call
/// keeps flash SCK <= clk_sys/4 at the 48 MHz interim and the final target.
/// clk_ref (12 MHz XOSC, drives all TICKS generators) is left untouched.
/// clk_peri/HSTX are explicitly retained on the independent 48 MHz PLL_USB.
void __not_in_flash_func(SwitchSysClockSram)(uint32_t vcoHz, uint8_t pd1, uint8_t pd2) {
    clock_hw_t* sys = &clocks_hw->clk[clk_sys];

    // 1. clk_sys glitchless mux back to clk_ref, then aux mux = USB PLL and
    //    reselect aux (glitch-safe aux change requires ref selection first).
    hw_clear_bits(&sys->ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (!(sys->selected & 1u)) {}
    hw_write_masked(&sys->ctrl,
        (CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB) |
        (CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX << CLOCKS_CLK_SYS_CTRL_SRC_LSB),
        CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS | CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (!(sys->selected & (1u << CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX))) {}
    sys->div = 1u << CLOCKS_CLK_SYS_DIV_INT_LSB;

    // 2. Re-lock pll_sys at the requested point (inline pll_init sequence).
    pll_hw_t* pll = pll_sys_hw;
    const uint32_t fbdiv = vcoHz / (XOSC_HZ / PLL_SYS_REFDIV);
    if (fbdiv < 16u || fbdiv > 320u) return;
    const uint32_t pdiv = (static_cast<uint32_t>(pd1) << PLL_PRIM_POSTDIV1_LSB) |
                          (static_cast<uint32_t>(pd2) << PLL_PRIM_POSTDIV2_LSB);
    reset_unreset_block_num_wait_blocking(RESET_PLL_SYS);
    pll->cs = PLL_SYS_REFDIV;
    pll->fbdiv_int = fbdiv;
    hw_clear_bits(&pll->pwr, PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS);
    while (!(pll->cs & PLL_CS_LOCK_BITS)) {}
    pll->prim = pdiv;
    hw_clear_bits(&pll->pwr, PLL_PWR_POSTDIVPD_BITS);

    // 3. Switch clk_sys to the re-locked pll_sys (ref selection first, as in
    //    clock_configure_internal, to avoid aux-mux glitches).
    hw_clear_bits(&sys->ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (!(sys->selected & 1u)) {}
    hw_write_masked(&sys->ctrl,
        (CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB) |
        (CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX << CLOCKS_CLK_SYS_CTRL_SRC_LSB),
        CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS | CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (!(sys->selected & (1u << CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX))) {}
}

uint32_t TargetSysClockHz(void*) {
    return clock_get_hz(clk_sys);
}

bool TargetSwitchPll(void*, uint32_t vcoHz, uint8_t pd1, uint8_t pd2) {
    SwitchSysClockSram(vcoHz, pd1, pd2);
    // Publish the new rate to the SDK bookkeeping (RAM write; XIP is safe at
    // this point — clk_sys is locked at the new frequency with the committed
    // flash divisor). Dependents using clock_get_hz(clk_sys) — I2C baud,
    // PIO divider math in retime hooks — see the real rate.
    clock_set_reported_hz(clk_sys, vcoHz / (static_cast<uint32_t>(pd1) * pd2));
    return true;
}

bool TargetCommitFlashDivisor(void*, uint8_t divisor) {
#if defined(PICO_NO_FLASH) && PICO_NO_FLASH
    // RAM_HOST: no flash exists — never dereference the absent device.
    (void)divisor;
    return true;
#else
    if (divisor < 2u) return false;
    // Only CLKDIV may change while the QMI is active; the register spec
    // requires a real (uncached) memory access plus barriers to commit the
    // new divisor before the system clock changes.
    hw_write_masked(&qmi_hw->m[0].timing,
                    static_cast<uint32_t>(divisor) << QMI_M0_TIMING_CLKDIV_LSB,
                    QMI_M0_TIMING_CLKDIV_BITS);
    (void)*reinterpret_cast<const volatile uint32_t*>(XIP_NOCACHE_NOALLOC_BASE);
    __dsb();
    __isb();
    return true;
#endif
}

bool TargetPreserveTimebase(void*) {
    // TICKS generators are clocked from clk_ref (12 MHz XOSC), which the
    // transition never touches — TIMER0 (time_us_64), TIMER1 and both
    // SysTicks keep wall time across the switch. Defensively re-arm every
    // generator whose divisor drifted or that stopped, so absolute deadlines
    // and watchdog cadence stay exact.
    const uint32_t cycles = clock_get_hz(clk_ref) / 1000000u;
    if (cycles == 0u) return false;
    for (uint32_t i = 0; i < static_cast<uint32_t>(TICK_COUNT); ++i) {
        if (ticks_hw->ticks[i].cycles != cycles ||
            !(ticks_hw->ticks[i].ctrl & TICKS_PROC0_CTRL_RUNNING_BITS)) {
            tick_start(static_cast<tick_gen_num_t>(i), cycles);
        }
    }
    return true;
}

uint64_t TargetTimeUs(void*) {
    return time_us_64();
}

uint16_t TargetCoreMillivolts(void*) {
    switch (vreg_get_voltage()) {
        case VREG_VOLTAGE_1_10: return 1100;
        case VREG_VOLTAGE_1_20: return 1200;
        default: return 0;
    }
}

bool TargetSetCoreMillivolts(void*, uint16_t millivolts) {
    // The SDK wrapper preserves HT_TH/protection fields and never disables the
    // voltage limit. Public policy is intentionally narrower: no >1.2 V.
    if (millivolts != 1100 && millivolts != 1200) return false;
    const enum vreg_voltage requested = millivolts == 1100 ? VREG_VOLTAGE_1_10 : VREG_VOLTAGE_1_20;
    vreg_set_voltage(requested);
    return vreg_get_voltage() == requested && TargetCoreMillivolts(nullptr) == millivolts;
}

bool TargetVerifyFixedDomains(void*) {
    const auto* ref = &clocks_hw->clk[clk_ref];
    const auto* peri = &clocks_hw->clk[clk_peri];
    const auto* usb = &clocks_hw->clk[clk_usb];
    const auto* adc = &clocks_hw->clk[clk_adc];
    const auto* hstx = &clocks_hw->clk[clk_hstx];
    return
        ((ref->ctrl & CLOCKS_CLK_REF_CTRL_SRC_BITS) ==
            (CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC << CLOCKS_CLK_REF_CTRL_SRC_LSB)) &&
        ((ref->div & CLOCKS_CLK_REF_DIV_INT_BITS) == (1u << CLOCKS_CLK_REF_DIV_INT_LSB)) &&
        ((peri->ctrl & CLOCKS_CLK_PERI_CTRL_AUXSRC_BITS) ==
            (CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB << CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB)) &&
        ((peri->div & CLOCKS_CLK_PERI_DIV_INT_BITS) == (1u << CLOCKS_CLK_PERI_DIV_INT_LSB)) &&
        ((usb->ctrl & CLOCKS_CLK_USB_CTRL_AUXSRC_BITS) ==
            (CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB << CLOCKS_CLK_USB_CTRL_AUXSRC_LSB)) &&
        ((usb->div & CLOCKS_CLK_USB_DIV_INT_BITS) == (1u << CLOCKS_CLK_USB_DIV_INT_LSB)) &&
        ((adc->ctrl & CLOCKS_CLK_ADC_CTRL_AUXSRC_BITS) ==
            (CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB << CLOCKS_CLK_ADC_CTRL_AUXSRC_LSB)) &&
        ((adc->div & CLOCKS_CLK_ADC_DIV_INT_BITS) == (1u << CLOCKS_CLK_ADC_DIV_INT_LSB)) &&
        ((hstx->ctrl & CLOCKS_CLK_HSTX_CTRL_AUXSRC_BITS) ==
            (CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB << CLOCKS_CLK_HSTX_CTRL_AUXSRC_LSB)) &&
        ((hstx->div & CLOCKS_CLK_HSTX_DIV_INT_BITS) == (1u << CLOCKS_CLK_HSTX_DIV_INT_LSB)) &&
        clock_get_hz(clk_peri) == kPeripheralFixedHz && clock_get_hz(clk_hstx) == kPeripheralFixedHz &&
        clock_get_hz(clk_usb) == kPeripheralFixedHz && clock_get_hz(clk_adc) == kPeripheralFixedHz &&
        clock_get_hz(clk_ref) == kReferenceFixedHz;
}

}  // namespace

Platform TargetPlatform() {
    Platform p;
    p.ctx                = nullptr;
    p.sysClockHz         = &TargetSysClockHz;
    p.switchSysClockPll  = &TargetSwitchPll;
    p.setCoreMillivolts  = &TargetSetCoreMillivolts;
    p.coreMillivolts     = &TargetCoreMillivolts;
    p.commitFlashDivisor = &TargetCommitFlashDivisor;
    p.preserveTimebase   = &TargetPreserveTimebase;
    p.timeUs             = &TargetTimeUs;
    p.peripheralHz       = [](void*) { return clock_get_hz(clk_peri); };
    p.referenceHz        = [](void*) { return clock_get_hz(clk_ref); };
    p.usbHz              = [](void*) { return clock_get_hz(clk_usb); };
    p.adcHz              = [](void*) { return clock_get_hz(clk_adc); };
    p.hstxHz             = [](void*) { return clock_get_hz(clk_hstx); };
    p.verifyFixedDomains = &TargetVerifyFixedDomains;
    return p;
}

#endif  // PICO_ON_DEVICE

}  // namespace GpuClock
