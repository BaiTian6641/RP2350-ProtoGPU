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

/// Status response (updated by UpdateStatus)
static PglStatusResponse statusResponse{};

/// Capability response (constant — filled at init)
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
    capabilityResponse.protoVersion = 3;  // ProtoGL v0.3
    capabilityResponse.gpuArch     = PGL_ARCH_ARM_CM33;
    capabilityResponse.coreCount   = 2;
    capabilityResponse.coreFreqMHz = static_cast<uint8_t>(GpuConfig::SYSTEM_CLOCK_MHZ);
    capabilityResponse.sramKB      = 520;
    capabilityResponse.maxVertices  = GpuConfig::MAX_VERTICES;
    capabilityResponse.maxTriangles = GpuConfig::MAX_TRIANGLES;
    capabilityResponse.maxMeshes    = GpuConfig::MAX_MESHES;
    capabilityResponse.maxMaterials = GpuConfig::MAX_MATERIALS;
    capabilityResponse.maxTextures  = GpuConfig::MAX_TEXTURES;
    capabilityResponse.flags       = PGL_CAP_HW_FLOAT | PGL_CAP_UNALIGNED_ACCESS;
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
    // Select the I2C instance
    i2c_inst = (GpuConfig::I2C_INSTANCE == 0) ? i2c0 : i2c1;

    // Initialize I2C peripheral (baudrate is set by master; slave follows)
    i2c_init(i2c_inst, 400 * 1000);

    // Configure GPIO functions for I2C
    gpio_set_function(GpuConfig::I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(GpuConfig::I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(GpuConfig::I2C_SDA_PIN);
    gpio_pull_up(GpuConfig::I2C_SCL_PIN);

    // Build static capability response
    BuildCapabilityResponse();

    // Initialize the I2C slave with our handler callback
    // This sets the slave address and installs the IRQ handler
    i2c_slave_init(i2c_inst, GpuConfig::I2C_ADDRESS, &i2c_slave_handler);

    // Clear initial state
    registerAddressed = false;
    writeLen = 0;
    readLen = 0;
    readIdx = 0;
    memset((void*)&statusResponse, 0, sizeof(statusResponse));

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
