/**
 * @file gpu_core.cpp
 * @brief GPU core lifecycle implementation — RP2350 dual-core.
 */

#include "gpu_core.h"
#include "gpu_config.h"
#include "scene_state.h"
#include "command_parser.h"
#include "display/hub75_driver.h"
#include "transport/octal_spi_rx.h"
#include "transport/i2c_slave.h"
#include "render/rasterizer.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

#include <cstdio>

// ─── GPU State ──────────────────────────────────────────────────────────────

/// Double framebuffers (RGB565)
static uint16_t framebufferA[GpuConfig::FRAMEBUF_PIXELS];
static uint16_t framebufferB[GpuConfig::FRAMEBUF_PIXELS];

/// Pointers for double-buffering: display reads front, rasterizer writes back
static uint16_t* frontBuffer = framebufferA;
static uint16_t* backBuffer  = framebufferB;

/// Z-buffer (shared between cores — each core writes distinct Y ranges)
static float zBuffer[GpuConfig::FRAMEBUF_PIXELS];

/// Scene and render state
static SceneState   sceneState;
static Rasterizer   rasterizer;
static uint32_t     frameCount = 0;
static uint32_t     lastFrameTimeUs = 0;

// ─── Initialization ─────────────────────────────────────────────────────────

bool GpuCore::Initialize() {
    printf("[GpuCore] Initializing subsystems...\n");

    // 1. RDY pin — output, initially low (not ready)
    gpio_init(GpuConfig::RDY_PIN);
    gpio_set_dir(GpuConfig::RDY_PIN, GPIO_OUT);
    gpio_put(GpuConfig::RDY_PIN, 0);

    // 2. HUB75 display driver (PIO + DMA)
    if (!Hub75Driver::Initialize(frontBuffer)) {
        printf("[GpuCore] ERROR: HUB75 init failed\n");
        return false;
    }
    printf("[GpuCore] HUB75 display initialized (%ux%u, 1/%u scan)\n",
           GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT, GpuConfig::SCAN_ROWS);

    // 3. Octal SPI receiver (PIO + DMA)
    if (!OctalSpiRx::Initialize()) {
        printf("[GpuCore] ERROR: Octal SPI init failed\n");
        return false;
    }
    printf("[GpuCore] Octal SPI receiver initialized\n");

    // 4. I2C slave
    if (!I2CSlave::Initialize()) {
        printf("[GpuCore] ERROR: I2C slave init failed\n");
        return false;
    }
    printf("[GpuCore] I2C slave initialized at address 0x%02X\n", GpuConfig::I2C_ADDRESS);

    // 5. Scene state (zeroes resource tables)
    sceneState.Reset();
    printf("[GpuCore] Scene state reset (max %u meshes, %u materials)\n",
           GpuConfig::MAX_MESHES, GpuConfig::MAX_MATERIALS);

    // 6. Rasterizer (references to framebuffer, Z-buffer, scene)
    rasterizer.Initialize(&sceneState, zBuffer,
                          GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT);
    printf("[GpuCore] Rasterizer initialized\n");

    // 7. Clear both framebuffers
    memset(framebufferA, 0, sizeof(framebufferA));
    memset(framebufferB, 0, sizeof(framebufferB));

    // 8. Assert RDY — we're ready to receive data
    gpio_put(GpuConfig::RDY_PIN, 1);
    printf("[GpuCore] GPU ready. RDY pin asserted.\n");

    return true;
}

// ─── Core 0 Main Loop ──────────────────────────────────────────────────────

void GpuCore::Core0Main() {
    while (true) {
        // 1. Check for new frame data in SPI ring buffer
        const uint8_t* frameData = nullptr;
        uint32_t frameLength = 0;

        if (!OctalSpiRx::TryGetFrame(&frameData, &frameLength)) {
            // No complete frame yet — brief yield
            tight_loop_contents();
            continue;
        }

        // 2. Parse command buffer → update scene state
        CommandParser::ParseResult result = CommandParser::Parse(
            frameData, frameLength, &sceneState);

        // Release the ring buffer region
        OctalSpiRx::ConsumeFrame();

        if (result != CommandParser::ParseResult::Ok) {
            // CRC error or malformed — skip this frame, use last good state
            printf("[GpuCore] Frame parse error %d — dropped\n",
                   static_cast<int>(result));
            continue;
        }

        // 3. Transform + project + build QuadTree (single-threaded)
        rasterizer.PrepareFrame(&sceneState);

        // 4. Signal Core 1: start rasterizing bottom half
        multicore_fifo_push_blocking(FIFO_CMD_START_RENDER);

        // 5. Rasterize top half (Y = 0 to H/2 - 1)
        const uint16_t halfHeight = GpuConfig::PANEL_HEIGHT / 2;
        rasterizer.RasterizeRange(backBuffer, 0, halfHeight);

        // 6. Wait for Core 1 to finish bottom half
        uint32_t response = multicore_fifo_pop_blocking();
        (void)response;  // should be FIFO_CMD_RENDER_DONE

        // 7. Swap framebuffers
        uint16_t* temp = frontBuffer;
        frontBuffer = backBuffer;
        backBuffer = temp;
        Hub75Driver::SetFramebuffer(frontBuffer);

        // 8. Update flow control
        uint32_t freeBytes = OctalSpiRx::GetFreeBytes();
        uint32_t threshold = GpuConfig::SPI_RING_BUFFER_SIZE
                           * GpuConfig::RDY_ASSERT_THRESHOLD / 100;
        gpio_put(GpuConfig::RDY_PIN, freeBytes >= threshold ? 1 : 0);

        frameCount++;
    }
}

// ─── Core 1 Main Loop ──────────────────────────────────────────────────────

void GpuCore::Core1Main() {
    const uint16_t halfHeight = GpuConfig::PANEL_HEIGHT / 2;

    while (true) {
        // Wait for Core 0 to signal: QuadTree is ready, start rasterizing
        uint32_t cmd = multicore_fifo_pop_blocking();

        if (cmd == FIFO_CMD_START_RENDER) {
            // Rasterize bottom half (Y = H/2 to H-1)
            rasterizer.RasterizeRange(backBuffer, halfHeight, GpuConfig::PANEL_HEIGHT);

            // Signal completion to Core 0
            multicore_fifo_push_blocking(FIFO_CMD_RENDER_DONE);
        }
    }
}
