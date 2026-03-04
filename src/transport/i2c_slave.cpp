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

// Shared ProtoGL headers
#include <PglTypes.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/xip_ctrl.h"
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
// Probes OPI PSRAM (PIO2) and QSPI CS1 memory (QMI) at boot to determine what
// external memory is physically present.  CS1 auto-detection uses real SPI
// RDID transactions via the QMI direct-mode interface:
//   1. MRAM RDID (0x4B + mode byte 0xFF) → MR10Q010 profile
//   2. PSRAM RDID (0x9F)                  → APS6408L / ESP-PSRAM profile
// Results are stored in the capability response flags and extended status.
// Full driver initialization uses the results from these probes.

static uint8_t detectedVramFlags = 0;

/// Runtime-detected QSPI CS1 chip profile (set by ProbeQspiCs1).
static GpuConfig::QspiChipProfile g_qspiChipProfile = {};

/// Accessor for other modules (mem_tier.h, mem_qspi_psram.h).
const GpuConfig::QspiChipProfile& GetQspiChipProfile() {
    return g_qspiChipProfile;
}

/**
 * @brief Probe PIO2 external memory (OPI PSRAM or QSPI MRAM).
 *
 * Checks PIO2_MEM_MODE from gpu_config.h and validates pin configuration.
 * The actual device verification (RDID) happens during OpiPsramDriver::Initialize()
 * in gpu_core.cpp.  At this early probe stage, we check compile-time config and
 * pin validity to determine if the hardware *should* be present.
 *
 * Returns true if the PIO2 memory hardware is expected to be present.
 */
static bool ProbePio2Memory() {
    if (GpuConfig::PIO2_MEM_MODE == GpuConfig::Pio2MemMode::NONE) return false;

    switch (GpuConfig::PIO2_MEM_MODE) {
    case GpuConfig::Pio2MemMode::OPI_PSRAM: {
        // OPI PSRAM (APS6408L): verify CS pin is configured.
        // Full RDID verification via the 8-bit PIO bus happens during
        // OpiPsramDriver::Initialize() where the PIO program is loaded.
        bool detected = (GpuConfig::PIO2_MEM_CS0_PIN != 0xFF);
        if (detected) {
            printf("[VRAM] PIO2 OPI PSRAM probe: CS=%u configured "
                   "(capacity=%lu KB, RDID deferred to driver init)\n",
                   GpuConfig::PIO2_MEM_CS0_PIN,
                   (unsigned long)(GpuConfig::OPI_PSRAM_CAPACITY / 1024));
        }
        return detected;
    }

    case GpuConfig::Pio2MemMode::DUAL_QSPI_MRAM: {
        // Dual QSPI MRAM (2× MR10Q010): verify both CS pins.
        // RDID via PIO2 QSPI deferred to OpiPsramDriver::Initialize().
        bool cs0ok = (GpuConfig::PIO2_MEM_CS0_PIN != 0xFF);
        bool cs1ok = (GpuConfig::QSPI_MRAM_CS1_PIN != 0xFF);
        bool detected = cs0ok && cs1ok;
        if (detected) {
            printf("[VRAM] PIO2 Dual QSPI MRAM probe: 2 × MR10Q010 "
                   "(CS0=%u, CS1=%u, total=%lu KB)\n",
                   GpuConfig::PIO2_MEM_CS0_PIN,
                   GpuConfig::QSPI_MRAM_CS1_PIN,
                   (unsigned long)(GpuConfig::QSPI_MRAM_CHIP_CAPACITY * 2 / 1024));
        } else if (cs0ok) {
            printf("[VRAM] PIO2 Dual QSPI MRAM probe: only CS0 detected, "
                   "operating as single MRAM (128 KB)\n");
            detected = true;
        }
        return detected;
    }

    case GpuConfig::Pio2MemMode::SINGLE_QSPI_MRAM: {
        // Single QSPI MRAM (MR10Q010): verify CS0 pin.
        bool detected = (GpuConfig::PIO2_MEM_CS0_PIN != 0xFF);
        if (detected) {
            printf("[VRAM] PIO2 Single QSPI MRAM probe: MR10Q010 "
                   "(CS0=%u, capacity=%lu KB)\n",
                   GpuConfig::PIO2_MEM_CS0_PIN,
                   (unsigned long)(GpuConfig::QSPI_MRAM_CHIP_CAPACITY / 1024));
        }
        return detected;
    }

    default:
        return false;
    }
}

/**
 * @brief Auto-detect chip on QMI CS1 (MRAM or PSRAM).
 *
 * Uses real QMI direct-mode SPI transactions to read chip IDs:
 *   1. Try MRAM RDID (cmd 0x4B, mode byte 0xFF, read 5 bytes).
 *      If response matches 0x076B111111 → MR10Q010 MRAM (128 KB).
 *   2. Try PSRAM RDID (cmd 0x9F, read 3 bytes).
 *      If MFR byte == 0x0D → APS6408L (8 MB AP Memory).
 *      If MFR byte == 0x5D → ESP-PSRAM64H (8 MB Espressif).
 *   3. If neither matched, report UNKNOWN_DEVICE.
 *
 * The detected chip profile is stored in g_qspiChipProfile for use by the
 * QSPI driver and memory tier manager.
 */

// ─── QMI Direct-Mode SPI Helpers ─────────────────────────────────────────
// These are lightweight SPI functions for RDID probes only.
// The full QMI configuration is done by QspiPsramDriver::Initialize().

/// Timeout for QMI direct-mode busy-waits (~2 ms at 150 MHz).
static constexpr uint32_t QMI_PROBE_TIMEOUT = 300000;

/// Track whether QMI probe timed out (used to early-exit the probe sequence).
static volatile bool qmiProbeTimedOut = false;

static void QmiProbeEnterDirect() {
    qmiProbeTimedOut = false;
    // Temporarily disable XIP caching to enter direct mode
    xip_ctrl_hw->ctrl &= ~(XIP_CTRL_EN_SECURE_BITS | XIP_CTRL_EN_NONSECURE_BITS);
    // Enable QMI direct-mode CS1
    qmi_hw->direct_csr = QMI_DIRECT_CSR_EN_BITS
                        | QMI_DIRECT_CSR_CLKDIV_BITS  // max divider for safe probe
                        | (1u << QMI_DIRECT_CSR_ASSERT_CS1N_LSB);
    // Wait for direct mode to be ready (with timeout)
    uint32_t timeout = QMI_PROBE_TIMEOUT;
    while (!(qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS) && --timeout) {
        tight_loop_contents();
    }
    if (timeout == 0) {
        printf("[VRAM] QMI direct-mode enter timed out\n");
        qmiProbeTimedOut = true;
    }
}

static void QmiProbeExitDirect() {
    // Deassert CS, disable direct mode
    qmi_hw->direct_csr = 0;
    // Re-enable XIP caching
    xip_ctrl_hw->ctrl |= (XIP_CTRL_EN_SECURE_BITS | XIP_CTRL_EN_NONSECURE_BITS);
    // Fence to ensure XIP is active before returning
    __compiler_memory_barrier();
}

/// Transfer one byte via QMI direct mode (full-duplex SPI).
/// If `isTx` is true, sends `txByte`; otherwise sends 0xFF and returns the response.
/// Returns 0xFF on timeout (no device present).
static uint8_t QmiProbeTransfer(uint8_t txByte, bool isTx) {
    if (qmiProbeTimedOut) return 0xFF;

    // Wait for TX ready (with timeout)
    uint32_t timeout = QMI_PROBE_TIMEOUT;
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) && --timeout) {
        tight_loop_contents();
    }
    if (timeout == 0) { qmiProbeTimedOut = true; return 0xFF; }

    if (isTx) {
        // Transmit: OE enabled, 1-bit width, 8 bits, push RX
        qmi_hw->direct_tx = QMI_DIRECT_TX_OE_BITS
                           | (0u << QMI_DIRECT_TX_IWIDTH_LSB)   // single-bit
                           | txByte;
    } else {
        // Receive: OE disabled (tristate MOSI), read MISO
        qmi_hw->direct_tx = (0u << QMI_DIRECT_TX_IWIDTH_LSB) | 0xFF;
    }

    // Wait for RX data (with timeout)
    timeout = QMI_PROBE_TIMEOUT;
    while (!(qmi_hw->direct_csr & QMI_DIRECT_CSR_RXFULL_BITS) && --timeout) {
        tight_loop_contents();
    }
    if (timeout == 0) { qmiProbeTimedOut = true; return 0xFF; }

    return static_cast<uint8_t>(qmi_hw->direct_rx);
}

static bool ProbeQspiCs1() {
    // Always attempt runtime QSPI CS1 auto-detection regardless of the
    // QSPI_CS1_ENABLED compile-time flag.  The QMI peripheral on RP2350
    // can safely probe CS1 even when nothing is connected — the RDID
    // commands will return 0xFF (no response), which we handle gracefully.
    // This enables VRAMless mode: the firmware auto-detects whatever memory
    // is physically present at boot without needing recompilation.
    //
    // Note: QSPI_CS1_ENABLED is still respected for compile-time optimisation
    // hints (e.g., the tier manager's weight tables), but it no longer gates
    // the runtime probe.

    QmiProbeEnterDirect();

    // ── Step 1: Try MRAM RDID (0x4B + mode byte 0xFF → 5 response bytes) ──
    QmiProbeTransfer(0x4B, true);   // RDID command
    QmiProbeTransfer(0xFF, true);   // Mode byte

    uint8_t rdidBytes[5];
    for (int i = 0; i < 5; ++i) {
        rdidBytes[i] = QmiProbeTransfer(0xFF, false);
    }

    // Reconstruct 40-bit device ID (big-endian)
    uint64_t mramRdid = 0;
    for (int i = 0; i < 5; ++i) {
        mramRdid = (mramRdid << 8) | rdidBytes[i];
    }

    QmiProbeExitDirect();

    if (mramRdid == GpuConfig::QSPI_RDID_MR10Q010) {
        g_qspiChipProfile = GpuConfig::PROFILE_MR10Q010;
        printf("[VRAM] QSPI CS1 auto-detect: MR10Q010 MRAM "
               "(128 KB, 104 MHz, no random-access penalty)\n");
        return true;
    }

    // ── Step 2: Try PSRAM RDID (0x9F → 3 response bytes: MFR, KGD, EID) ──
    QmiProbeEnterDirect();

    QmiProbeTransfer(0x9F, true);   // RDID command

    uint8_t mfrId   = QmiProbeTransfer(0xFF, false);
    uint8_t kgdByte = QmiProbeTransfer(0xFF, false);
    uint8_t eidByte = QmiProbeTransfer(0xFF, false);

    uint64_t psramRdid = ((uint64_t)mfrId << 16) | ((uint64_t)kgdByte << 8) | eidByte;

    QmiProbeExitDirect();

    if (mfrId == 0x0D) {
        // AP Memory — APS6408L-SQR (8 MB QSPI PSRAM)
        g_qspiChipProfile = GpuConfig::PROFILE_APS6408L;
        g_qspiChipProfile.deviceId = psramRdid;
        printf("[VRAM] QSPI CS1 auto-detect: APS6408L PSRAM "
               "(8 MB, 133 MHz, has random-access penalty)\n");
        return true;
    }

    if (mfrId == 0x5D) {
        // Espressif — ESP-PSRAM64H (8 MB)
        g_qspiChipProfile = GpuConfig::PROFILE_ESP_PSRAM;
        g_qspiChipProfile.deviceId = psramRdid;
        printf("[VRAM] QSPI CS1 auto-detect: ESP-PSRAM64H "
               "(8 MB, 84 MHz, has random-access penalty)\n");
        return true;
    }

    // ── Step 3: No known device responded ─────────────────────────────
    if (mramRdid != 0 && mramRdid != 0xFFFFFFFFFFULL) {
        // Something responded but with an unrecognized ID
        printf("[VRAM] QSPI CS1 auto-detect: unknown device "
               "(MRAM RDID=0x%010llX, PSRAM MFR=0x%02X)\n",
               (unsigned long long)mramRdid, mfrId);
        // Default to MRAM profile for safety (lower risk of bus contention)
        g_qspiChipProfile = GpuConfig::PROFILE_MR10Q010;
        g_qspiChipProfile.type = static_cast<decltype(g_qspiChipProfile.type)>(
            PGL_QSPI_CHIP_UNKNOWN);
        return true;
    }

    printf("[VRAM] QSPI CS1 auto-detect: no device responded\n");
    return false;
}

/// Probe all external VRAM at boot.  Updates capability flags.
static void ProbeExternalVram() {
    detectedVramFlags = 0;

    if (ProbePio2Memory()) {
        detectedVramFlags |= PGL_VRAM_OPI_DETECTED;
        capabilityResponse.flags |= PGL_CAP_OPI_VRAM;

        // Use mode-aware capacity from gpu_config.h
        uint32_t pio2Capacity = GpuConfig::Pio2MemCapacity();
        extendedStatus.opiVramTotalKB = static_cast<uint16_t>(pio2Capacity / 1024);
        extendedStatus.opiVramFreeKB = extendedStatus.opiVramTotalKB;

        // Report PIO2 mode details
        const char* modeStr =
            (GpuConfig::PIO2_MEM_MODE == GpuConfig::Pio2MemMode::OPI_PSRAM)         ? "OPI PSRAM" :
            (GpuConfig::PIO2_MEM_MODE == GpuConfig::Pio2MemMode::DUAL_QSPI_MRAM)   ? "Dual QSPI MRAM" :
            (GpuConfig::PIO2_MEM_MODE == GpuConfig::Pio2MemMode::SINGLE_QSPI_MRAM) ? "Single QSPI MRAM" :
            "unknown";
        printf("[VRAM] PIO2 mode: %s (%lu KB, random-penalty: %s, non-volatile: %s)\n",
               modeStr,
               (unsigned long)(pio2Capacity / 1024),
               GpuConfig::Pio2IsMram() ? "NO" : "yes",
               GpuConfig::Pio2IsMram() ? "yes" : "no");
    }

    if (ProbeQspiCs1()) {
        detectedVramFlags |= PGL_VRAM_QSPI_DETECTED;
        capabilityResponse.flags |= PGL_CAP_QSPI_VRAM;

        // Use runtime-detected capacity from the chip profile
        extendedStatus.qspiVramTotalKB = static_cast<uint16_t>(
            g_qspiChipProfile.capacityBytes / 1024);
        extendedStatus.qspiVramFreeKB = extendedStatus.qspiVramTotalKB;

        // Report detected chip type to host (replaces 'reserved' byte)
        extendedStatus.qspiChipType = static_cast<uint8_t>(g_qspiChipProfile.type);

        printf("[VRAM] QSPI CS1 chip: %s (random-access penalty: %s, non-volatile: %s)\n",
               g_qspiChipProfile.hasRandomAccessPenalty ? "PSRAM" : "MRAM",
               g_qspiChipProfile.hasRandomAccessPenalty ? "yes" : "NO",
               g_qspiChipProfile.isNonVolatile ? "yes" : "no");
    }

    extendedStatus.vramTierFlags = detectedVramFlags;
    printf("[VRAM] Tier flags: 0x%02X (OPI=%s, QSPI=%s)\n",
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

    // ── Skip I2C + VRAM probe when pins are disabled (0xFF) ──
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

    // Probe external VRAM (updates capability flags + extended status)
    ProbeExternalVram();

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
