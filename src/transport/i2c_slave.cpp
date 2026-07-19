/**
 * @file i2c_slave.cpp
 * @brief I2C slave handler implementation for RP2350.
 *
 * Uses the Pico SDK's pico_i2c_slave library for IRQ-driven I2C slave
 * communication.  The ESP32-S3 host sends register-addressed writes and
 * reads to configure the GPU and query its status / capabilities.
 *
 * I2C Protocol:
 *   Write: [I2C_ADDR+W][REG_ADDR][DATA_0][DATA_1]...
 *   Read:  [I2C_ADDR+W][REG_ADDR] → [I2C_ADDR+R][DATA_0][DATA_1]...
 *
 * The slave handler uses a state machine:
 *   - On RECEIVE: first byte = register address, subsequent bytes = write data
 *   - On REQUEST: respond with register data from the last addressed register
 *   - On FINISH:  commit the write or reset state
 */

#include "i2c_slave.h"
#include "../gpu_config.h"
#include "../scene_state.h"
#include "../display/display_manager.h"
#include "../memory/mem_pool.h"

// Shared ProtoGL headers
#include <PglTypes.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "pico/i2c_slave.h"

#include <cstring>
#include <cstdio>

// ─── State ──────────────────────────────────────────────────────────────────

static i2c_inst_t* i2c_inst = nullptr;

/// Current register values
static volatile uint8_t  currentBrightness = 128;
static volatile uint16_t panelWidth  = GpuConfig::PANEL_WIDTH;
static volatile uint16_t panelHeight = GpuConfig::PANEL_HEIGHT;
static volatile uint8_t  scanRate    = GpuConfig::SCAN_ROWS;
static volatile uint8_t  gammaTable  = 0;
static volatile bool     resetRequested = false;
static volatile bool     clearRequested = false;

/// Clock change request (set by I2C write, consumed by main loop)
static volatile uint16_t clockRequestMHz  = 0;
static volatile uint8_t  clockRequestVolt = 0;
static volatile uint8_t  clockRequestFlags = 0;

/// Pointer to the GPU scene state (set by SetSceneState, used for memory registers)
static SceneState* g_sceneState = nullptr;

/// M11: Display and pool manager pointers (set by SetDisplayAndPools)
static DisplayManager*  g_displayMgr = nullptr;
static MemPoolManager*  g_poolMgr    = nullptr;

/// M11: Currently selected pool handle for PGL_REG_MEM_POOL_STATUS
static uint16_t g_selectedPoolHandle = 0;

/// Status response (updated by UpdateStatus)
static PglStatusResponse statusResponse{};

/// Extended status response (updated by UpdateExtendedStatus + UpdateVramStatus)
static PglExtendedStatusResponse extendedStatus{};

/// Capability response (filled at init, VRAM flags updated after probe)
static PglCapabilityResponse capabilityResponse{};

// ─── I2C Transaction State Machine ──────────────────────────────────────────

/// Internal state for the current I2C transaction
static volatile uint8_t  currentRegister = 0;     // register address from first write byte
static volatile bool     registerAddressed = false; // true after first byte of write
static volatile uint8_t  writeBuffer[32];           // accumulates write data
static volatile uint8_t  writeLen = 0;
static volatile uint8_t  readBuffer[32];            // response data for reads
static volatile uint8_t  readLen = 0;
static volatile uint8_t  readIdx = 0;               // current read position

// ─── Capability Initialization ──────────────────────────────────────────────

static void BuildCapabilityResponse() {
    capabilityResponse.protoVersion = 5;  // ProtoGL v0.5
    capabilityResponse.gpuArch     = PGL_ARCH_ARM_CM33;
    capabilityResponse.coreCount   = 2;
    capabilityResponse.coreFreqMHz = static_cast<uint8_t>(clock_get_hz(clk_sys) / 1000000);
    capabilityResponse.sramKB      = 520;
    capabilityResponse.maxVertices  = GpuConfig::MAX_VERTICES;
    capabilityResponse.maxTriangles = GpuConfig::MAX_TRIANGLES;
    capabilityResponse.maxMeshes    = GpuConfig::MAX_MESHES;
    capabilityResponse.maxMaterials = GpuConfig::MAX_MATERIALS;
    capabilityResponse.maxTextures  = GpuConfig::MAX_TEXTURES;

    // Base flags — always present on RP2350 Cortex-M33
    uint8_t flags = PGL_CAP_HW_FLOAT | PGL_CAP_UNALIGNED_ACCESS | PGL_CAP_DSP
                  | PGL_CAP_DYNAMIC_CLOCK | PGL_CAP_TEMP_SENSOR;

    // VRAM detection flags are set after PSRAM probe (see ProbeExternalVram())
    capabilityResponse.flags = flags;
}

// ─── VRAM Detection ─────────────────────────────────────────────────────────
// Reports which external QSPI VRAM channels are configured in gpu_config.h.
//
// In the unified QspiVram model (mem_qspi_vram.h) there is no 8-bit OPI bus
// and no QMI CS1 XIP chip: both external tiers are QSPI channels on the
// single PIO2-driven QspiVramDriver.  Real chip detection (per-chip-select
// RDID probe: MRAM 0x4B / PSRAM 0x9F) happens inside
// QspiVramDriver::InitChannel(), which runs later in GpuCore::Initialize().
// At this early boot stage we can only report compile-time configuration;
// capacities stay 0 until the driver probe completes.  Authoritative
// per-channel totals then flow to the host each frame via
// PGL_REG_MEM_TIER_INFO (updated in gpu_core.cpp).

static uint8_t detectedVramFlags = 0;

/// QSPI chip profile placeholder — runtime chip profiles now live in the
/// QspiVramDriver per chip-select (see QspiVramDriver::GetChipConfig()).
/// Retained for modules that still query the legacy accessor; always
/// reports "no chip detected" at this pre-driver-init stage.
static GpuConfig::QspiChipProfile g_qspiChipProfile = {};

/// Accessor for other modules (mem_tier.h, mem_qspi_vram.h).
const GpuConfig::QspiChipProfile& GetQspiChipProfile() {
    return g_qspiChipProfile;
}

/**
 * @brief Check whether QSPI Channel A (Tier 1) is configured.
 *
 * Channel A replaces the old PIO2 OPI bus in the QspiVram model.
 * Per-chip-select RDID verification is deferred to
 * QspiVramDriver::InitChannel() — here we only validate compile-time config.
 *
 * Returns true if Channel A hardware is expected to be present.
 */
static bool ProbeQspiChannelA() {
    if (!GpuConfig::QspiVramEnabled()) return false;
    if (GpuConfig::QSPI_A_CHIP_COUNT == 0) return false;
    if (GpuConfig::QSPI_A_CS0_PIN == 0xFF) return false;

    printf("[VRAM] QSPI Channel A: configured (%u chip(s), CS0=%u, CS1=%u) "
           "— RDID/capacity deferred to QspiVramDriver init\n",
           GpuConfig::QSPI_A_CHIP_COUNT,
           GpuConfig::QSPI_A_CS0_PIN,
           GpuConfig::QSPI_A_CS1_PIN);
    return true;
}

/**
 * @brief Check whether QSPI Channel B (Tier 2) is configured.
 *
 * Channel B replaces the old QMI CS1 XIP path — XIP memory-mapping no
 * longer exists; Channel B is a second PIO2 QSPI bus on the same unified
 * QspiVramDriver.  Only present in DUAL_CHANNEL mode.
 *
 * Returns true if Channel B hardware is expected to be present.
 */
static bool ProbeQspiChannelB() {
    if (!GpuConfig::QspiChannelBEnabled()) return false;
    if (GpuConfig::QSPI_B_CHIP_COUNT == 0) return false;
    if (GpuConfig::QSPI_B_CS0_PIN == 0xFF) return false;

    printf("[VRAM] QSPI Channel B: configured (%u chip(s), CS0=%u, CS1=%u) "
           "— RDID/capacity deferred to QspiVramDriver init\n",
           GpuConfig::QSPI_B_CHIP_COUNT,
           GpuConfig::QSPI_B_CS0_PIN,
           GpuConfig::QSPI_B_CS1_PIN);
    return true;
}

// ─── QMI CS1 XIP Probe (REMOVED) ────────────────────────────────────────────
// The old QMI direct-mode CS1 RDID probe (QmiProbeEnterDirect /
// QmiProbeTransfer / ProbeQspiCs1) has no analog in the QspiVram model:
// there is no XIP memory-mapped VRAM anymore.  Equivalent per-chip-select
// RDID auto-detection is built into QspiVramDriver::InitChannel().

/// Probe all external VRAM at boot.  Updates capability flags.
static void ProbeExternalVram() {
    detectedVramFlags = 0;

    if (ProbeQspiChannelA()) {
        // Reported in the legacy "OPI" wire fields (= Tier 1 PIO2 memory).
        detectedVramFlags |= PGL_VRAM_OPI_DETECTED;
        capabilityResponse.flags |= PGL_CAP_OPI_VRAM;

        // Capacity is per-chip auto-detected (MRAM 128 KB vs PSRAM 8 MB) —
        // unknown until QspiVramDriver::InitChannel() probes RDID.  Report 0
        // here; per-frame authoritative values flow via PGL_REG_MEM_TIER_INFO.
        extendedStatus.opiVramTotalKB = 0;
        extendedStatus.opiVramFreeKB  = 0;
    }

    if (ProbeQspiChannelB()) {
        // Reported in the legacy "QSPI" wire fields (= Tier 2).
        detectedVramFlags |= PGL_VRAM_QSPI_DETECTED;
        capabilityResponse.flags |= PGL_CAP_QSPI_VRAM;

        extendedStatus.qspiVramTotalKB = 0;
        extendedStatus.qspiVramFreeKB  = 0;

        // Chip type per chip-select is auto-detected by the driver at init.
        extendedStatus.qspiChipType = PGL_QSPI_CHIP_NONE;
    }

    extendedStatus.vramTierFlags = detectedVramFlags;
    printf("[VRAM] Tier flags: 0x%02X (QSPI-A=%s, QSPI-B=%s)\n",
           detectedVramFlags,
           (detectedVramFlags & PGL_VRAM_OPI_DETECTED)  ? "yes" : "no",
           (detectedVramFlags & PGL_VRAM_QSPI_DETECTED) ? "yes" : "no");
}

// ─── Temperature Sensor ─────────────────────────────────────────────────────

static bool adcInitialised = false;

static void InitTemperatureSensor() {
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adcInitialised = true;
    printf("[TEMP] ADC temperature sensor enabled (channel %u)\n",
           ADC_TEMPERATURE_CHANNEL_NUM);
}

// ─── Register Handlers ──────────────────────────────────────────────────────

/**
 * @brief Process a completed register write transaction.
 *
 * Called when the I2C master finishes writing data to a register.
 * The register address is in `currentRegister` and the data is in
 * `writeBuffer[0..writeLen-1]`.
 */
static void ProcessRegisterWrite() {
    uint8_t reg = currentRegister;
    uint8_t len = writeLen;
    const volatile uint8_t* data = writeBuffer;

    switch (static_cast<PglI2CRegister>(reg)) {
        case PGL_REG_SET_BRIGHTNESS:
            if (len >= 1) currentBrightness = data[0];
            break;

        case PGL_REG_SET_PANEL_CONFIG:
            if (len >= 4) {
                uint16_t w, h;
                memcpy(&w, (const void*)&data[0], 2);
                memcpy(&h, (const void*)&data[2], 2);
                panelWidth = w;
                panelHeight = h;
            }
            break;

        case PGL_REG_SET_SCAN_RATE:
            if (len >= 1) scanRate = data[0];
            break;

        case PGL_REG_CLEAR_DISPLAY:
            clearRequested = true;
            break;

        case PGL_REG_SET_GAMMA_TABLE:
            if (len >= 1) gammaTable = data[0];
            break;

        case PGL_REG_RESET_GPU:
            resetRequested = true;
            break;

        case PGL_REG_SET_CLOCK_FREQ:
            if (len >= sizeof(PglClockRequest)) {
                PglClockRequest req;
                memcpy(&req, (const void*)data, sizeof(req));
                clockRequestMHz   = req.targetMHz;
                clockRequestVolt  = req.voltageLevel;
                clockRequestFlags = req.flags;
            }
            break;

        case PGL_REG_MEM_READ_ADDR:
            // Host writes PglMemReadSetup to set the staging read address.
            // The actual data staging is handled by CMD_MEM_READ_REQUEST in
            // the command parser.  This register just resets the read cursor
            // so the host can re-read staged data via PGL_REG_MEM_READ_DATA.
            if (g_sceneState) {
                g_sceneState->memStagingReadPos = 0;
            }
            break;

        // ── M12: Writable registers ────────────────────────────────────
        case PGL_REG_DMA_FILL_THRESHOLD:
            // Host writes 2-byte LE threshold value
            if (g_sceneState && writeLen >= 2) {
                uint16_t val;
                memcpy(&val, (const void*)writeBuffer, sizeof(uint16_t));
                g_sceneState->dmaFillThreshold = val;
            }
            break;

        // ── M11: Display & Pool select registers ────────────────────────
        case PGL_REG_DISPLAY_MODE:
            // Write: uint8_t displayId → select active display slot for queries
            if (g_displayMgr && writeLen >= 1) {
                g_displayMgr->selectedSlot = data[0];
            }
            break;

        case PGL_REG_MEM_POOL_STATUS:
            // Write: uint16_t poolHandle → select pool for status query
            if (writeLen >= 2) {
                memcpy(&g_selectedPoolHandle, (const void*)data, sizeof(uint16_t));
            }
            break;

        default:
            break;
    }
}

/**
 * @brief Prepare the read response buffer for the current register.
 *
 * Called when the I2C master starts a read transaction. Copies the response
 * data into `readBuffer` and sets `readLen`.
 */
static void PrepareRegisterRead() {
    uint8_t reg = currentRegister;

    switch (static_cast<PglI2CRegister>(reg)) {
        case PGL_REG_STATUS_REQUEST:
            memcpy((void*)readBuffer, (const void*)&statusResponse,
                   sizeof(statusResponse));
            readLen = sizeof(statusResponse);
            break;

        case PGL_REG_CAPABILITY_QUERY:
            memcpy((void*)readBuffer, (const void*)&capabilityResponse,
                   sizeof(capabilityResponse));
            readLen = sizeof(capabilityResponse);
            break;

        case PGL_REG_SET_BRIGHTNESS:
            readBuffer[0] = currentBrightness;
            readLen = 1;
            break;

        case PGL_REG_SET_SCAN_RATE:
            readBuffer[0] = scanRate;
            readLen = 1;
            break;

        case PGL_REG_EXTENDED_STATUS:
            memcpy((void*)readBuffer, (const void*)&extendedStatus,
                   sizeof(extendedStatus));
            readLen = sizeof(extendedStatus);
            break;

        case PGL_REG_MEM_TIER_INFO:
            // Return cached memory tier info (updated each frame in gpu_core.cpp)
            if (g_sceneState) {
                memcpy((void*)readBuffer,
                       (const void*)&g_sceneState->memTierInfo,
                       sizeof(PglMemTierInfoResponse));
                readLen = sizeof(PglMemTierInfoResponse);
            } else {
                readLen = 0;
            }
            break;

        case PGL_REG_MEM_READ_DATA:
            // Return the next 32-byte chunk from the memory staging buffer.
            // Auto-increments the read position.
            if (g_sceneState && g_sceneState->memStagingLength > 0) {
                uint32_t pos = g_sceneState->memStagingReadPos;
                uint32_t remaining = g_sceneState->memStagingLength - pos;
                uint8_t chunk = (remaining < PGL_MEM_READ_CHUNK_SIZE)
                              ? static_cast<uint8_t>(remaining)
                              : PGL_MEM_READ_CHUNK_SIZE;
                memcpy((void*)readBuffer,
                       (const void*)&g_sceneState->memStagingBuffer[pos],
                       chunk);
                readLen = chunk;
                g_sceneState->memStagingReadPos += chunk;
            } else {
                readLen = 0;
            }
            break;

        case PGL_REG_MEM_ALLOC_RESULT:
            // Return the result of the last CMD_MEM_ALLOC
            if (g_sceneState) {
                memcpy((void*)readBuffer,
                       (const void*)&g_sceneState->lastAllocResult,
                       sizeof(PglMemAllocResult));
                readLen = sizeof(PglMemAllocResult);
            } else {
                readLen = 0;
            }
            break;

        // ── M11: Display Mode, Caps, Pool Status ────────────────────────
        case PGL_REG_DISPLAY_MODE:
            // Read: returns current active display type (PglDisplayType) for selected slot
            if (g_displayMgr) {
                readBuffer[0] = g_displayMgr->GetDisplayType(g_displayMgr->selectedSlot);
                readLen = 1;
            } else {
                readBuffer[0] = PGL_DISPLAY_NONE;
                readLen = 1;
            }
            break;

        case PGL_REG_DISPLAY_CAPS:
            // Read: PglDisplayCaps (16 bytes) for selected display slot
            if (g_displayMgr) {
                PglDisplayCaps caps;
                g_displayMgr->GetCaps(g_displayMgr->selectedSlot, caps);
                memcpy((void*)readBuffer, (const void*)&caps, sizeof(PglDisplayCaps));
                readLen = sizeof(PglDisplayCaps);
            } else {
                memset((void*)readBuffer, 0, sizeof(PglDisplayCaps));
                readLen = sizeof(PglDisplayCaps);
            }
            break;

        case PGL_REG_MEM_POOL_STATUS:
            // Read: PglMemPoolStatusResponse (12 bytes) for selected pool
            if (g_poolMgr) {
                PglMemPoolStatusResponse resp;
                g_poolMgr->GetStatus(g_selectedPoolHandle, resp);
                memcpy((void*)readBuffer, (const void*)&resp, sizeof(resp));
                readLen = sizeof(resp);
            } else {
                PglMemPoolStatusResponse resp = {};
                resp.status = PGL_POOL_INVALID_HANDLE;
                memcpy((void*)readBuffer, (const void*)&resp, sizeof(resp));
                readLen = sizeof(resp);
            }
            break;

        // ── M12: Defrag, Persistence, Dirty Stats, DMA Threshold ────────
        case PGL_REG_MEM_DEFRAG_STATUS:
            if (g_sceneState) {
                memcpy((void*)readBuffer,
                       (const void*)&g_sceneState->defragStatus,
                       sizeof(PglMemDefragStatusResponse));
                readLen = sizeof(PglMemDefragStatusResponse);
            } else {
                readLen = 0;
            }
            break;

        case PGL_REG_MEM_PERSIST_STATUS:
            if (g_sceneState) {
                memcpy((void*)readBuffer,
                       (const void*)&g_sceneState->persistStatus,
                       sizeof(PglMemPersistStatusResponse));
                readLen = sizeof(PglMemPersistStatusResponse);
            } else {
                readLen = 0;
            }
            break;

        case PGL_REG_DIRTY_STATS:
            if (g_sceneState) {
                memcpy((void*)readBuffer,
                       (const void*)&g_sceneState->dirtyStats,
                       sizeof(PglDirtyStatsResponse));
                readLen = sizeof(PglDirtyStatsResponse);
            } else {
                readLen = 0;
            }
            break;

        case PGL_REG_DMA_FILL_THRESHOLD:
            // Read: returns current threshold as 2-byte LE value
            if (g_sceneState) {
                memcpy((void*)readBuffer,
                       (const void*)&g_sceneState->dmaFillThreshold,
                       sizeof(uint16_t));
                readLen = sizeof(uint16_t);
            } else {
                readLen = 0;
            }
            break;

        default:
            readLen = 0;
            break;
    }

    readIdx = 0;
}

// ─── I2C Slave IRQ Handler ──────────────────────────────────────────────────

/**
 * @brief Callback invoked by the Pico SDK i2c_slave library on I2C events.
 *
 * Called from IRQ context. Must be fast and non-blocking.
 *
 * Events:
 *   I2C_SLAVE_RECEIVE — master is writing data to us
 *   I2C_SLAVE_REQUEST — master is reading data from us
 *   I2C_SLAVE_FINISH  — transaction complete (STOP condition)
 */
static void i2c_slave_handler(i2c_inst_t* i2c, i2c_slave_event_t event) {
    (void)i2c;

    switch (event) {
        case I2C_SLAVE_RECEIVE: {
            // Read one byte from the master
            uint8_t byte = i2c_read_byte_raw(i2c_inst);

            if (!registerAddressed) {
                // First byte of write = register address
                currentRegister = byte;
                registerAddressed = true;
                writeLen = 0;
            } else {
                // Subsequent bytes = register data
                if (writeLen < sizeof(writeBuffer)) {
                    writeBuffer[writeLen++] = byte;
                }
            }
            break;
        }

        case I2C_SLAVE_REQUEST: {
            // Master requests a read byte
            if (readIdx == 0) {
                // First read byte — prepare the response
                PrepareRegisterRead();
            }

            if (readIdx < readLen) {
                i2c_write_byte_raw(i2c_inst, readBuffer[readIdx++]);
            } else {
                // Read past end of response — send 0xFF
                i2c_write_byte_raw(i2c_inst, 0xFF);
            }
            break;
        }

        case I2C_SLAVE_FINISH: {
            // STOP condition — commit any pending write
            if (registerAddressed && writeLen > 0) {
                ProcessRegisterWrite();
            }

            // Reset state for next transaction
            registerAddressed = false;
            writeLen = 0;
            readLen = 0;
            readIdx = 0;
            break;
        }
    }
}

// ─── Initialization ─────────────────────────────────────────────────────────

bool I2CSlave::Initialize() {
    // Initialize on-chip temperature sensor (ADC — not I2C dependent)
    InitTemperatureSensor();

    // Build static capability response (needed even without I2C for self-test)
    BuildCapabilityResponse();

    // Probe external VRAM regardless of I2C availability.
    // The probe only reads compile-time QspiVram channel configuration (no
    // bus traffic — RDID happens later in QspiVramDriver::InitChannel()), so
    // it works even when I2C pins are disabled.
    ProbeExternalVram();

    // ── Skip I2C bus init when pins are disabled (0xFF) ──
    if (GpuConfig::I2C_SDA_PIN == 0xFF || GpuConfig::I2C_SCL_PIN == 0xFF) {
        printf("[I2C] Disabled (no I2C pins assigned — Pico 2 mode)\n");
        return false;
    }

    // Select the I2C instance
    i2c_inst = (GpuConfig::I2C_INSTANCE == 0) ? i2c0 : i2c1;

    // Initialize I2C peripheral (baudrate is set by master; slave follows)
    i2c_init(i2c_inst, 400 * 1000);

    // Configure GPIO functions for I2C
    gpio_set_function(GpuConfig::I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(GpuConfig::I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(GpuConfig::I2C_SDA_PIN);
    gpio_pull_up(GpuConfig::I2C_SCL_PIN);

    // Initialize the I2C slave with our handler callback
    // This sets the slave address and installs the IRQ handler
    i2c_slave_init(i2c_inst, GpuConfig::I2C_ADDRESS, &i2c_slave_handler);

    // Clear initial state
    registerAddressed = false;
    writeLen = 0;
    readLen = 0;
    readIdx = 0;
    memset((void*)&statusResponse, 0, sizeof(statusResponse));

    // Pre-fill extended status with clock info
    extendedStatus.currentClockMHz = static_cast<uint16_t>(
        clock_get_hz(clk_sys) / 1000000);

    printf("[I2C] Slave initialized at address 0x%02X on i2c%u (SDA=%u, SCL=%u)\n",
           GpuConfig::I2C_ADDRESS, GpuConfig::I2C_INSTANCE,
           GpuConfig::I2C_SDA_PIN, GpuConfig::I2C_SCL_PIN);

    return true;
}

// ─── Polling ────────────────────────────────────────────────────────────────

void I2CSlave::Poll() {
    // The i2c_slave library is fully IRQ-driven, so no polling is needed.
    // This method exists for compatibility with the polling-based interface
    // and may be used for future non-IRQ extensions.
}

// ─── Status Update ──────────────────────────────────────────────────────────

void I2CSlave::UpdateStatus(uint16_t fps, uint16_t droppedFrames,
                            uint32_t freeMemory, int8_t temperature,
                            bool renderBusy) {
    // These writes are atomic enough for 16-bit fields on Cortex-M33.
    // For full atomicity, a critical section could be used, but the
    // status response is informational and occasional tearing is acceptable.
    statusResponse.currentFPS    = fps;
    statusResponse.droppedFrames = droppedFrames;
    statusResponse.freeMemory16  = static_cast<uint16_t>(freeMemory / 16);
    statusResponse.temperature   = temperature;
    statusResponse.flags         = renderBusy ? PGL_STATUS_RENDER_BUSY : 0;
}

// ─── Getters ────────────────────────────────────────────────────────────────

uint8_t I2CSlave::GetBrightness() {
    return currentBrightness;
}

bool I2CSlave::ConsumeResetRequest() {
    bool req = resetRequested;
    resetRequested = false;
    return req;
}

// ─── Extended Status ────────────────────────────────────────────────────────

void I2CSlave::UpdateExtendedStatus(uint16_t fps, uint16_t droppedFrames,
                                    uint8_t gpuUsage, uint8_t core0Usage,
                                    uint8_t core1Usage, uint8_t flags,
                                    int16_t temperatureQ8,
                                    uint16_t currentClockMHz,
                                    uint16_t sramFreeKB,
                                    uint16_t frameTimeUs, uint16_t rasterTimeUs,
                                    uint16_t transferTimeUs,
                                    uint16_t hub75RefreshHz) {
    extendedStatus.currentFPS       = fps;
    extendedStatus.droppedFrames    = droppedFrames;
    extendedStatus.gpuUsagePercent  = gpuUsage;
    extendedStatus.core0UsagePercent = core0Usage;
    extendedStatus.core1UsagePercent = core1Usage;
    extendedStatus.flags            = flags;
    extendedStatus.temperatureQ8    = temperatureQ8;
    extendedStatus.currentClockMHz  = currentClockMHz;
    extendedStatus.sramFreeKB       = sramFreeKB;
    extendedStatus.frameTimeUs      = frameTimeUs;
    extendedStatus.rasterTimeUs     = rasterTimeUs;
    extendedStatus.transferTimeUs   = transferTimeUs;
    extendedStatus.hub75RefreshHz   = hub75RefreshHz;
}

void I2CSlave::UpdateVramStatus(uint16_t opiTotalKB, uint16_t opiFreeKB,
                                uint16_t qspiTotalKB, uint16_t qspiFreeKB,
                                uint8_t tierFlags) {
    extendedStatus.opiVramTotalKB  = opiTotalKB;
    extendedStatus.opiVramFreeKB   = opiFreeKB;
    extendedStatus.qspiVramTotalKB = qspiTotalKB;
    extendedStatus.qspiVramFreeKB  = qspiFreeKB;
    extendedStatus.vramTierFlags   = tierFlags;
}

// ─── Temperature Sensor ─────────────────────────────────────────────────────

float I2CSlave::ReadTemperature() {
    if (!adcInitialised) return 0.0f;

    adc_select_input(ADC_TEMPERATURE_CHANNEL_NUM);
    uint16_t raw = adc_read();
    float voltage = raw * 3.3f / 4096.0f;
    return 27.0f - (voltage - 0.706f) / 0.001721f;
}

// ─── Clock Change Request ───────────────────────────────────────────────────

uint16_t I2CSlave::ConsumeClockRequest(uint8_t* outVoltage, uint8_t* outFlags) {
    uint16_t mhz = clockRequestMHz;
    if (mhz == 0) return 0;

    if (outVoltage) *outVoltage = clockRequestVolt;
    if (outFlags)   *outFlags   = clockRequestFlags;

    // Clear request
    clockRequestMHz   = 0;
    clockRequestVolt  = 0;
    clockRequestFlags = 0;

    return mhz;
}

// ─── Scene State Binding ────────────────────────────────────────────────────

void I2CSlave::SetSceneState(SceneState* scene) {
    g_sceneState = scene;
}

void I2CSlave::SetDisplayAndPools(DisplayManager* displayMgr, MemPoolManager* poolMgr) {
    g_displayMgr = displayMgr;
    g_poolMgr    = poolMgr;
}

// ─── Shutdown ───────────────────────────────────────────────────────────────

void I2CSlave::Shutdown() {
    if (i2c_inst) {
        // Deinit the slave (disables IRQ handler)
        i2c_slave_deinit(i2c_inst);
        i2c_deinit(i2c_inst);
        i2c_inst = nullptr;

        printf("[I2C] Shutdown complete\n");
    }
}
