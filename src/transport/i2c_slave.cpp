/**
 * @file i2c_slave.cpp
 * @brief I2C slave handler implementation for RP2350.
 *
 * TODO(M1/Week 5): Full implementation. This skeleton maps out the
 * correct Pico SDK I2C slave API calls and register dispatch logic.
 */

#include "i2c_slave.h"
#include "../gpu_config.h"

// Shared ProtoGL headers
#include <PglTypes.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include <cstring>

// ─── State ──────────────────────────────────────────────────────────────────

static i2c_inst_t* i2c_inst = nullptr;

/// Current register values
static uint8_t  currentBrightness = 128;
static uint16_t panelWidth  = GpuConfig::PANEL_WIDTH;
static uint16_t panelHeight = GpuConfig::PANEL_HEIGHT;
static uint8_t  scanRate    = GpuConfig::SCAN_ROWS;
static uint8_t  gammaTable  = 0;
static bool     resetRequested = false;

/// Status response (updated by UpdateStatus)
static PglStatusResponse statusResponse{};

/// Capability response (constant — filled at init)
static PglCapabilityResponse capabilityResponse{};

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

// ─── Initialization ─────────────────────────────────────────────────────────

bool I2CSlave::Initialize() {
    // Select the I2C instance
    i2c_inst = (GpuConfig::I2C_INSTANCE == 0) ? i2c0 : i2c1;

    // Initialize I2C at 400 kHz (fast mode)
    i2c_init(i2c_inst, 400 * 1000);

    // Configure GPIO functions
    gpio_set_function(GpuConfig::I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(GpuConfig::I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(GpuConfig::I2C_SDA_PIN);
    gpio_pull_up(GpuConfig::I2C_SCL_PIN);

    // Set slave address
    // TODO(M1/Week 5): Pico SDK's i2c_slave API requires the pico_i2c_slave
    // library to be linked. Use i2c_slave_init() once available, or implement
    // the handler using raw I2C hardware registers:
    //
    // i2c_slave_init(i2c_inst, GpuConfig::I2C_ADDRESS, &i2c_slave_handler);

    // Build static capability response
    BuildCapabilityResponse();

    return true;
}

// ─── I2C Slave Handler ──────────────────────────────────────────────────────

/// Called when the host writes data to a register.
static void HandleRegisterWrite(uint8_t reg, const uint8_t* data, uint8_t len) {
    switch (static_cast<PglI2CRegister>(reg)) {
        case PGL_REG_SET_BRIGHTNESS:
            if (len >= 1) currentBrightness = data[0];
            break;

        case PGL_REG_SET_PANEL_CONFIG:
            if (len >= 4) {
                std::memcpy(&panelWidth, &data[0], 2);
                std::memcpy(&panelHeight, &data[2], 2);
            }
            break;

        case PGL_REG_SET_SCAN_RATE:
            if (len >= 1) scanRate = data[0];
            break;

        case PGL_REG_CLEAR_DISPLAY:
            // TODO: Signal rasterizer to clear framebuffer
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

/// Called when the host reads from a register.
static void HandleRegisterRead(uint8_t reg, uint8_t* response, uint8_t* responseLen) {
    switch (static_cast<PglI2CRegister>(reg)) {
        case PGL_REG_STATUS_REQUEST:
            std::memcpy(response, &statusResponse, sizeof(statusResponse));
            *responseLen = sizeof(statusResponse);
            break;

        case PGL_REG_CAPABILITY_QUERY:
            std::memcpy(response, &capabilityResponse, sizeof(capabilityResponse));
            *responseLen = sizeof(capabilityResponse);
            break;

        default:
            *responseLen = 0;
            break;
    }
}

// ─── Polling ────────────────────────────────────────────────────────────────

void I2CSlave::Poll() {
    // TODO(M1/Week 5): Implement polling or IRQ-based handling.
    // In the Pico SDK i2c_slave model, the handler callbacks are invoked
    // from the I2C IRQ. This Poll() function may not be needed if using
    // the IRQ-driven approach. Keep as a stub for now.
}

// ─── Status Update ──────────────────────────────────────────────────────────

void I2CSlave::UpdateStatus(uint16_t fps, uint16_t droppedFrames,
                            uint32_t freeMemory, int8_t temperature,
                            bool renderBusy) {
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
        i2c_deinit(i2c_inst);
        i2c_inst = nullptr;
    }
}
