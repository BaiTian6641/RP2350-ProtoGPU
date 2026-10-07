/**
 * @file mem_qmi_psram.h
 * @brief Optional bounded QMI PSRAM asset backing for RP2350 (P11).
 *
 * REAL driver, no stub: the implementation exists only for RP2350 on-device
 * builds (PICO_ON_DEVICE && PICO_RP2350) against the pinned Pico SDK 2.3.1.
 * Native/SRAM-only builds get NO pretend device — qmiPsramBacking() is only
 * available in target builds; native tests exercise AssetService over a
 * plain SRAM span instead.
 *
 * Contract (all facts from the pinned SDK sources + RP-008373-DS-3 errata;
 * see the wave memory-facts report):
 *
 *   - Selected-CS init ONLY. The caller picks exactly one CS1 pad
 *     (GPU_QMI_PSRAM_CS_GPIO, default 8; RP2350A allows 0/8/19, RP2350B also
 *     47). No auto-detection, no pin probing: the build MUST set
 *     PICO_RUNTIME_SKIP_INIT_PSRAM=1 and PICO_AUTO_DETECT_PSRAM=0,
 *     PICO_AUTO_DETECT_PSRAM_CS=0, PICO_AUTO_DETECT_PSRAM_SIZE=0 — enforced
 *     at compile time in the .cpp.
 *   - Timing: qmiPsramInit() must run AFTER the host has released the boot
 *     pads (runtime HELLO + explicit release acknowledgement) and BEFORE
 *     core1, display, DMA or any active XIP/PSRAM user exists. On RAM_HOST
 *     (PICO_NO_FLASH) the QMI pad state is established with flash_start_xip()
 *     only at that point.
 *   - FLASH_LOCAL M0 caveat: psram_reinitialize() internally calls
 *     flash_start_xip(), which restores the ROM-SAVED boot2/M0 XIP
 *     configuration. That restore may revert a runtime-retimed flash M0
 *     divider, so callers that retime flash M0 MUST install
 *     QmiPsramConfig::postXipRestoreHook (or reapply their qualified M0
 *     divider + dummy access + DSB/ISB immediately after qmiPsramInit /
 *     qmiPsramClockFinalize return, before flash traffic at speed resumes).
 *   - Canonical CPU access: the uncached, no-alloc CS1 alias 0x15000000
 *     (XIP_NOCACHE_NOALLOC_BASE + 16 MiB). This is the ONE writable mapping
 *     the service uses; callers must not create cached writable aliases of
 *     the same range. Because every access is uncached, no XIP cache
 *     maintenance is required on this path and no coherence with DMA/cached
 *     views is assumed or implied.
 *   - Compatibility: the SDK init sends QPI-enable 0x35, quad read 0xEB
 *     with 6 dummy cycles, quad write 0x38, PAGEBREAK=1024. The selected
 *     part must be an APS6404/APS6408-class device (SDK KGD 0x5D mapping,
 *     capacities 2/4/8/16 MiB). Anything else is UnsupportedPart.
 *   - RP2350-E14 (A2): keep PICO_RP2350_A2_SUPPORTED=1; the SDK's
 *     psram_reinitialize() applies the selected-pad isolation fix.
 *
 * Absence semantics: no chip / unrecognized ID => Absent or UnsupportedPart,
 * zero capacity, pin + QMI leases released, SRAM-only runtime unaffected.
 *
 * Ownership: at most ONE successful qmiPsramInit() per boot
 * (AlreadyInitialized afterwards). When QmiPsramConfig::resources is set,
 * the backend takes real HardwareResources leases (GPIO pin + QMI window 1,
 * Owner::Memory) and a conflicting claim (e.g. HUB75 profile pins 6..19,
 * which include GPIO8) rejects initialization before any pad is driven.
 */

#pragma once

#include <cstdint>
#include <cstddef>

#include "mem_assets.h"

class HardwareResources;  // parent-owned lease registry (src/hardware_resources.h)

namespace gpumem {

#ifndef GPU_QMI_PSRAM_CS_GPIO
/// Selected CS1 pad. 8 keeps it free on the SPI profile (SSD1331 DC=11) and
/// conflicts with the HUB75 profile (colors 6..11) — that combination MUST be
/// rejected by the board profile / HardwareResources claim, never probed.
#define GPU_QMI_PSRAM_CS_GPIO 8u
#endif

// ─── Configuration ──────────────────────────────────────────────────────────

/// Exact-part timing inputs (from the selected part's datasheet). Defaults
/// below target the APS6404-family part class the SDK recognizes (KGD 0x5D,
/// 0xEB/0x38 quad formats), deliberately conservative versus the SDK's
/// 133 MHz / 8000 ns / 18 ns: 32 MHz SCK, 7 us max select (a further
/// whole-transaction overrun margin is subtracted inside qmiPsramInit), and
/// 50 ns min deselect. The SDK half-SCK receive delay is representable only
/// through224 MHz clk_sys at this ceiling; active PSRAM rejects the selectable
/// 240/250/288/300/336 MHz profiles before a clock switch.
struct QmiPsramTiming {
    uint32_t maxClockHz;     ///< part SCK ceiling (Hz)
    uint32_t maxSelectNs;    ///< part max CS-low time (refresh-limited), ns
    uint32_t minDeselectNs;  ///< part min CS-high time between transfers, ns
};

/// Encoded M1 timing, shared by target admission and native boundary tests.
/// RXDELAY uses the pinned SDK's half-SCK sampling policy (half-sys-cycle
/// units), not an unqualified clamp to its three-bit field. Consequently
/// 32 MHz PSRAM cannot currently be admitted above 224 MHz clk_sys.
struct QmiPsramTimingFields {
    uint32_t divisor;
    uint32_t rxDelay;
    uint32_t maxSelect;
    uint32_t minDeselect;
};

inline bool qmiPsramComputeTiming(uint32_t sysHz, const QmiPsramTiming& timing,
                                  QmiPsramTimingFields& fields) {
    if (!sysHz || !timing.maxClockHz || timing.maxClockHz > 32000000u ||
        !timing.maxSelectNs || timing.maxSelectNs > 7000u ||
        timing.minDeselectNs < 50u) return false;
    const uint64_t divisor =
        (uint64_t(sysHz) + timing.maxClockHz - 1u) / timing.maxClockHz;
    // RXDELAY=divisor samples halfway through SCK, as in SDK 2.3.1.
    // Explicit field checks are required even with SDK parameter assertions off.
    if (!divisor || divisor > 7u) return false;
    const uint64_t marginNs =
        (40ull * divisor * 1000000000ull + sysHz - 1u) / sysHz;
    if (timing.maxSelectNs <= marginNs) return false;
    const uint64_t maxSelect =
        ((timing.maxSelectNs - marginNs) * sysHz) / (64ull * 1000000000ull);
    const uint64_t deselectCycles =
        (uint64_t(timing.minDeselectNs) * sysHz + 999999999ull) / 1000000000ull;
    const uint64_t halfSckCycles = (divisor + 1u) / 2u;
    const uint64_t minDeselect =
        deselectCycles > halfSckCycles ? deselectCycles - halfSckCycles : 0;
    if (!maxSelect || maxSelect > 63u || minDeselect > 31u) return false;
    fields = {uint32_t(divisor), uint32_t(divisor), uint32_t(maxSelect),
              uint32_t(minDeselect)};
    return true;
}

struct QmiPsramConfig {
    uint8_t csGpio = static_cast<uint8_t>(GPU_QMI_PSRAM_CS_GPIO);
    QmiPsramTiming timing = {
        32u * 1000u * 1000u,  ///< conservative APS6404-family SCK ceiling
        7000u,                ///< max select (ns); overrun margin subtracted inside
        50u,                  ///< min deselect (ns)
    };
    /// Optional parent lease registry. When set, the CS GPIO and QMI window 1
    /// are claimed (Owner::Memory) before any pad state changes and released
    /// on failure/deinit. When null, the caller guarantees sole ownership.
    HardwareResources* resources = nullptr;
    /// Optional hook invoked after psram_reinitialize() returns, including
    /// an error return, during init and clock finalize while still quiescent.
    /// Its internal flash_start_xip() restores the ROM-saved boot2/M0 XIP
    /// configuration, which may revert a runtime-retimed flash M0 divider.
    /// The hook MUST reapply the caller-qualified M0 divider plus a real
    /// (uncached) flash access and DSB/ISB before flash/XIP traffic resumes.
    /// When null, the caller accepts the ROM-saved M0 state or reapplies its
    /// divider immediately after either API returns, including failures.
    void (*postXipRestoreHook)(void* ctx) = nullptr;
    void* hookCtx = nullptr;
};

enum class QmiPsramStatus : uint8_t {
    Ready = 0,
    AlreadyInitialized,  ///< one owner/channel: a second init is rejected
    UnsupportedPin,      ///< csGpio is not a CS1-capable pad on this package
    LeaseConflict,       ///< HardwareResources denied the pin/QMI window claim
    Absent,              ///< no part answered the RDID exchange (capacity 0)
    UnsupportedPart,     ///< answered, but not an allowed APS6404-class size
    ParamError,          ///< timing cannot be represented with a safe MAX_SELECT
    InitFailed,          ///< SDK reinitialize/verify failed
};

// ─── Lifecycle (RP2350 on-device builds only) ───────────────────────────────

/// One-time selected-CS init. Preconditions (caller-enforced):
///   - host boot pads released and acknowledged;
///   - both cores quiesced w.r.t. flash/PSRAM, no active DMA/XIP users,
///     interrupts that touch XIP disabled (SDK documents psram_detect_size /
///     psram_reinitialize as unsafe otherwise);
///   - board profile has resolved CS availability (HUB75 + CS8 => reject).
/// Never called at startup automatically.
QmiPsramStatus qmiPsramInit(const QmiPsramConfig& config);

/// Release capacity, leases and the CS pad (input + pull-up = inactive).
/// After deinit the uncached CS1 window must not be accessed.
void qmiPsramDeinit();

bool     qmiPsramIsReady();
uint32_t qmiPsramCapacityBytes();

/// Backing ops for AssetService::bind(). Capacity 0 / failing ops when not
/// ready, so the service reports NoBacking on an absent device.
AssetBacking qmiPsramBacking();

/// Compile-time/checkable pin validity for the current package
/// (RP2350A: 0, 8, 19; RP2350B adds 47).
bool qmiPsramCsPinAllowed(uint8_t csGpio);

// ─── Clock retiming hooks (P11-05) ─────────────────────────────────────────
//
// QMI timing derives from clk_sys. Only CLKDIV may change while the QMI is
// active; all other M1 timing fields require QMI idle, so both hooks must be
// called at the same quiescent maintenance point as init (no core1/display/
// DMA/XIP activity, no interrupt handler reaching flash/PSRAM).
//
// qmiPsramClockPrepare(): call BEFORE raising clk_sys. If the current M1
// divisor would exceed the part's SCK ceiling at the new clock, the divisor
// is raised first, followed by a real uncached dummy read plus DSB/ISB so
// the new divisor is effective before the clock changes (QMI register
// requirement). This hook never writes M0 registers itself.
//
/// qmiPsramClockFinalize(): call AFTER clk_sys settled at its new value.
/// Recomputes all M1 timing fields (including the MAX_SELECT 64-cycle-unit
/// field, whose real nanoseconds change with clk_sys) and applies them via
/// psram_reinitialize(), which interrupts and restores XIP. That restore
/// returns flash M0 to the ROM-saved boot2 configuration and CAN revert a
/// runtime M0 retime: QmiPsramConfig::postXipRestoreHook runs immediately
/// after, still quiescent, so the flash owner can reapply its qualified M0
/// divider + fence before Finalize returns. The caller's clock code must
/// keep flash M0 safe at every intermediate frequency (raise its divisor
/// before a clk_sys increase) — that ordering is outside this module.
//
// Both return false when the device is not ready (no state change).
bool qmiPsramClockPrepare(uint32_t newSysClockHz);
bool qmiPsramClockFinalize(uint32_t actualSysClockHz);

} // namespace gpumem
