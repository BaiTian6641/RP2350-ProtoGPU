/**
 * @file mem_qmi_psram.cpp
 * @brief Optional QMI PSRAM asset backing — RP2350 on-device only.
 *
 * This TU deliberately compiles to NOTHING off-target: there is no native
 * fake QMI device. Native tests exercise AssetService over a plain SRAM
 * span through the same AssetBacking interface.
 *
 * Hard build requirements (compile-time enforced below):
 *   PICO_RUNTIME_SKIP_INIT_PSRAM=1   — no SDK startup PSRAM init; the
 *                                      selected CS pad stays untouched until
 *                                      host boot-pad release is proven.
 *   PICO_AUTO_DETECT_PSRAM=0, PICO_AUTO_DETECT_PSRAM_CS=0,
 *   PICO_AUTO_DETECT_PSRAM_SIZE=0    — no broad CS probing, ever.
 *   PICO_RP2350_A2_SUPPORTED=1       — keeps the SDK's RP2350-E14 selected-pad
 *                                      isolation workaround active.
 */

#include "mem_qmi_psram.h"

#if defined(PICO_ON_DEVICE) && defined(PICO_RP2350)

#include "hardware/psram.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/address_mapped.h"
#include "hardware/structs/qmi.h"
#include "hardware/regs/qmi.h"
#include "hardware/regs/addressmap.h"

#include "hardware_resources.h"

#if !defined(PICO_RUNTIME_SKIP_INIT_PSRAM) || !PICO_RUNTIME_SKIP_INIT_PSRAM
#error "mem_qmi_psram requires PICO_RUNTIME_SKIP_INIT_PSRAM=1: explicit selected-CS init only, after host boot-pad release"
#endif
#if (defined(PICO_AUTO_DETECT_PSRAM) && PICO_AUTO_DETECT_PSRAM) || \
    (defined(PICO_AUTO_DETECT_PSRAM_CS) && PICO_AUTO_DETECT_PSRAM_CS) || \
    (defined(PICO_AUTO_DETECT_PSRAM_SIZE) && PICO_AUTO_DETECT_PSRAM_SIZE)
#error "mem_qmi_psram forbids all PSRAM auto-detection switches (no broad CS probing)"
#endif

namespace gpumem {

namespace {

// ─── Canonical mappings (RP2350 address map) ────────────────────────────────
// CS1 cached window: 0x11000000 (XIP_BASE + 16 MiB). The asset service uses
// ONLY the uncached/no-alloc alias 0x15000000 — one writable mapping, so no
// XIP-cache maintenance is needed on this path and no cached/DMA coherence
// is assumed. ATRANS panes stay at identity; capacity is enforced in
// software bounds checks (runtime PSRAM init does not bound the window).
static_assert(XIP_BASE == 0x10000000u, "unexpected XIP base");
static_assert(XIP_NOCACHE_NOALLOC_BASE == 0x14000000u, "unexpected uncached base");
constexpr uintptr_t kCs1WindowOffset = 0x01000000u;  // CS0 -> CS1 window stride
constexpr uintptr_t kCs1UncachedBase = XIP_NOCACHE_NOALLOC_BASE + kCs1WindowOffset;  // 0x15000000

// ─── Module state (SRAM; single owner) ──────────────────────────────────────
bool              sReady = false;
uint32_t          sCapacity = 0;
QmiPsramConfig    sConfig{};
uint32_t          sDivisor = 0;
uint32_t          sPrevCsGpio = 0;
flash_devinfo_size_t sPrevCsSize = FLASH_DEVINFO_SIZE_NONE;
HardwareResources* sResources = nullptr;

bool capacityAllowed(uint32_t bytes) {
    // SDK APS6404-class EID mapping yields exactly these capacities.
    switch (bytes) {
        case 2u * 1024u * 1024u:
        case 4u * 1024u * 1024u:
        case 8u * 1024u * 1024u:
        case 16u * 1024u * 1024u:
            return true;
        default:
            return false;
    }
}

/// Compute and admit every encoded field before touching SDK timing state.
/// Using exact rational cycle counts avoids the SDK's truncated femtosecond
/// period and explicitly refuses RXDELAY overflow (SDK checks may be off).
int configureTimingConservative(uint32_t sysClockHz, const QmiPsramTiming& timing,
                                QmiPsramTimingFields& fields) {
    if (!qmiPsramComputeTiming(sysClockHz, timing, fields) ||
        clock_get_hz(clk_sys) != sysClockHz) return PICO_ERROR_INVALID_ARG;
    return psram_set_params(fields.divisor, fields.rxDelay, fields.maxSelect,
                            fields.minDeselect);
}

void releasePadAndLeases() {
    if (sResources) {
        sResources->ReleaseQmiWindow(1, HardwareResources::Owner::Memory);
        sResources->ReleaseGpios(1ull << sConfig.csGpio,
                                 HardwareResources::Owner::Memory);
        sResources = nullptr;
    }
    // CS idle-high, not driven: input with pull-up (hardware keeps the pad
    // inactive across reset; this only undoes our XIP_CS1 function select).
    gpio_set_function(sConfig.csGpio, GPIO_FUNC_NULL);
    gpio_set_dir(sConfig.csGpio, false);
    gpio_pull_up(sConfig.csGpio);
}

void rollbackDevinfo() {
    flash_devinfo_set_cs_size(1, sPrevCsSize);
    flash_devinfo_set_cs_gpio(1, sPrevCsGpio);
}

// ─── Backing ops: complete-range CPU copies over the uncached alias ─────────
// Word interior + byte head/tail handles arbitrary (un)alignment. Every
// access is uncached; no cache maintenance and no DMA involvement.

void copyBytes(uint8_t* dst, const uint8_t* src, uint32_t bytes) {
    while (bytes > 0 && (reinterpret_cast<uintptr_t>(src) & 3u) != 0) {
        *dst++ = *src++;
        --bytes;
    }
    if ((reinterpret_cast<uintptr_t>(dst) & 3u) == 0) {
        uint32_t* d32 = reinterpret_cast<uint32_t*>(dst);
        const uint32_t* s32 = reinterpret_cast<const uint32_t*>(src);
        while (bytes >= 4) {
            *d32++ = *s32++;
            bytes -= 4;
        }
        dst = reinterpret_cast<uint8_t*>(d32);
        src = reinterpret_cast<const uint8_t*>(s32);
    }
    while (bytes > 0) {
        *dst++ = *src++;
        --bytes;
    }
}

bool backingRead(void* /*ctx*/, uint32_t offset, void* dst, uint32_t bytes) {
    if (!sReady || !dst || bytes == 0) return false;
    if (offset > sCapacity || bytes > sCapacity - offset) return false;
    copyBytes(static_cast<uint8_t*>(dst),
              reinterpret_cast<const uint8_t*>(kCs1UncachedBase + offset), bytes);
    return true;
}

bool backingWrite(void* /*ctx*/, uint32_t offset, const void* src, uint32_t bytes) {
    if (!sReady || !src || bytes == 0) return false;
    if (offset > sCapacity || bytes > sCapacity - offset) return false;
    copyBytes(reinterpret_cast<uint8_t*>(kCs1UncachedBase + offset),
              static_cast<const uint8_t*>(src), bytes);
    return true;
}

} // namespace

// ─── Public API ─────────────────────────────────────────────────────────────

bool qmiPsramCsPinAllowed(uint8_t csGpio) {
#if defined(PICO_RP2350A) && PICO_RP2350A
    return csGpio == 0u || csGpio == 8u || csGpio == 19u;
#else
    return csGpio == 0u || csGpio == 8u || csGpio == 19u || csGpio == 47u;
#endif
}

QmiPsramStatus qmiPsramInit(const QmiPsramConfig& config) {
    if (sReady) return QmiPsramStatus::AlreadyInitialized;
    if (!qmiPsramCsPinAllowed(config.csGpio)) return QmiPsramStatus::UnsupportedPin;
    if (config.timing.maxClockHz == 0) return QmiPsramStatus::ParamError;

    sConfig = config;

    // Real resource leases first: a conflicting board profile (e.g. HUB75 on
    // pins 6..19 claiming GPIO8) rejects here, before any pad is driven.
    if (config.resources) {
        if (config.resources->ClaimQmiWindow(1, HardwareResources::Owner::Memory) !=
            PglRuntime::Result::Ok) {
            return QmiPsramStatus::LeaseConflict;
        }
        if (config.resources->ClaimGpios(1ull << config.csGpio,
                                         HardwareResources::Owner::Memory) !=
            PglRuntime::Result::Ok) {
            config.resources->ReleaseQmiWindow(1, HardwareResources::Owner::Memory);
            return QmiPsramStatus::LeaseConflict;
        }
        sResources = config.resources;
    }

    sPrevCsGpio = flash_devinfo_get_cs_gpio(1);
    sPrevCsSize = flash_devinfo_get_cs_size(1);

    // Selected CS1 pad only — never iterate candidate pins.
    flash_devinfo_set_cs_gpio(1, config.csGpio);
    gpio_set_function(config.csGpio, GPIO_FUNC_XIP_CS1);

#if defined(PICO_NO_FLASH) && PICO_NO_FLASH
    // RAM_HOST: the QMI pad state is established ONLY now — after the caller
    // proved host boot-pad release. ROM exit sequences touch CS0 as well;
    // that is expected and safe at this quiescent point.
    flash_start_xip();
#endif

    // RDID exchange on CS1 (KGD/EID decode for APS6404-class parts).
    const size_t detected = psram_detect_size();
    if (detected == 0 || detected > 0xFFFFFFFFu) {
        rollbackDevinfo();
        releasePadAndLeases();
        return QmiPsramStatus::Absent;  // zero capacity, leases released
    }
    if (!capacityAllowed(static_cast<uint32_t>(detected))) {
        rollbackDevinfo();
        releasePadAndLeases();
        return QmiPsramStatus::UnsupportedPart;
    }

    flash_devinfo_set_cs_size(1, flash_devinfo_bytes_to_size(
                                 static_cast<uint32_t>(detected)));

    const uint32_t sysHz = clock_get_hz(clk_sys);
    QmiPsramTimingFields fields{};
    if (configureTimingConservative(sysHz, config.timing, fields) != PICO_OK) {
        rollbackDevinfo();
        releasePadAndLeases();
        return QmiPsramStatus::ParamError;
    }

    // Applies M1 timing/format (QPI enable 0x35, quad read 0xEB/6 dummy,
    // quad write 0x38, COOLDOWN=1, PAGEBREAK=1024, XIP_CTRL_WRITABLE_M1),
    // installs the SRAM CS1 setup callback and re-enters XIP. RP2350-E14 (A2)
    // pad isolation is handled inside the SDK.
    const int rc = psram_reinitialize();
    // Even a failed reinitialization may have restored ROM-saved flash M0.
    if (config.postXipRestoreHook) config.postXipRestoreHook(config.hookCtx);
    if (rc != PICO_OK || !psram_is_available() ||
        psram_get_size() != detected) {
        rollbackDevinfo();
        releasePadAndLeases();
        return QmiPsramStatus::InitFailed;
    }

    sCapacity = static_cast<uint32_t>(detected);
    sDivisor = fields.divisor;
    sReady = true;
    return QmiPsramStatus::Ready;
}

void qmiPsramDeinit() {
    if (!sReady) return;
    sReady = false;
    sCapacity = 0;
    sDivisor = 0;
    rollbackDevinfo();
    releasePadAndLeases();
    // The M1 window mapping remains in the QMI registers but is never
    // accessed again: the service reports zero capacity. A later
    // flash_start_xip() re-runs the registered CS1 setup harmlessly with
    // size NONE until reboot clears devinfo (boot-RAM state, not OTP).
}

bool qmiPsramIsReady() { return sReady; }

uint32_t qmiPsramCapacityBytes() { return sReady ? sCapacity : 0; }

AssetBacking qmiPsramBacking() {
    AssetBacking backing;
    backing.ctx = nullptr;
    backing.capacityBytes = qmiPsramCapacityBytes();
    backing.read = &backingRead;
    backing.write = &backingWrite;
    return backing;
}

bool qmiPsramClockPrepare(uint32_t newSysClockHz) {
    if (!sReady || newSysClockHz == 0) return false;
    QmiPsramTimingFields fields{};
    if (!qmiPsramComputeTiming(newSysClockHz, sConfig.timing, fields)) return false;
    const uint32_t newDiv = fields.divisor;
    if (newDiv <= sDivisor) return true;  // current divisor stays within ceiling

    // Only CLKDIV may change while the QMI is active. Raise it AHEAD of the
    // clk_sys increase, then force a real (uncached) bus access plus
    // barriers so the new divisor is effective before the clock switches.
    hw_write_masked(&qmi_hw->m[1].timing,
                    newDiv << QMI_M1_TIMING_CLKDIV_LSB,
                    QMI_M1_TIMING_CLKDIV_BITS);
    (void)*reinterpret_cast<const volatile uint32_t*>(kCs1UncachedBase);
    __dsb();
    __isb();
    sDivisor = newDiv;
    return true;
}

bool qmiPsramClockFinalize(uint32_t actualSysClockHz) {
    if (!sReady || actualSysClockHz == 0) return false;
    // QMI is idle (caller: same quiescent point as init). Recompute every M1
    // timing field at the ACTUAL new clock — MAX_SELECT's 64-cycle units are
    // clk_sys-relative, so the field must change to keep the same ns bound.
    QmiPsramTimingFields fields{};
    if (configureTimingConservative(actualSysClockHz, sConfig.timing, fields) != PICO_OK) {
        return false;  // previous (conservative, higher) divisor stays in force
    }
    const int rc = psram_reinitialize();
    // Restore M0 even on an SDK error: flash_start_xip may already have run.
    if (sConfig.postXipRestoreHook) sConfig.postXipRestoreHook(sConfig.hookCtx);
    if (rc != PICO_OK || !psram_is_available() ||
        psram_get_size() != sCapacity) {
        return false;
    }
    sDivisor = fields.divisor;
    return true;
}

} // namespace gpumem

#endif // PICO_ON_DEVICE && PICO_RP2350
