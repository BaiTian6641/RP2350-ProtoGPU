/**
 * @file gpu_core.cpp
 * @brief GPU core lifecycle implementation — RP2350 dual-core.
 *
 * Core 0: SPI receive → parse → prepare frame → rasterize top half → swap → HUB75 refresh
 * Core 1: Rasterize bottom half (FIFO-triggered)
 */

#include "gpu_core.h"
#include "gpu_config.h"
#include "scene_state.h"
#include "command_parser.h"
#include "display/hub75_driver.h"
#include "transport/octal_spi_rx.h"
#include "transport/i2c_slave.h"
#include "render/rasterizer.h"
#include "render/screenspace_effects.h"
#include "profiling/perf_counters.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

#include <cstdio>
#include <cstring>
#include <cmath>

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
static uint32_t     droppedFrames = 0;
static uint32_t     lastFpsTimeUs = 0;
static uint16_t     measuredFps = 0;
static float        elapsedTimeS = 0.0f;   // accumulated wall time for animated effects

// ─── Initialization ─────────────────────────────────────────────────────────

bool GpuCore::Initialize() {
    printf("[GpuCore] Initializing subsystems...\n");

    // 1. RDY pin — output, initially low (not ready)
    gpio_init(GpuConfig::RDY_PIN);
    gpio_set_dir(GpuConfig::RDY_PIN, GPIO_OUT);
    gpio_put(GpuConfig::RDY_PIN, 0);

    // 2. Clear both framebuffers
    memset(framebufferA, 0, sizeof(framebufferA));
    memset(framebufferB, 0, sizeof(framebufferB));

    // 3. Fill front buffer with a test pattern for initial bringup
    Hub75Driver::FillTestPattern(frontBuffer, 1);  // RGB gradient

    // 4. HUB75 display driver (PIO + DMA)
    if (!Hub75Driver::Initialize(frontBuffer)) {
        printf("[GpuCore] ERROR: HUB75 init failed\n");
        return false;
    }
    printf("[GpuCore] HUB75 display initialized (%ux%u, 1/%u scan, %u-bit BCM)\n",
           GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT,
           GpuConfig::SCAN_ROWS, GpuConfig::COLOR_DEPTH);

    // 5. Octal SPI receiver (PIO + DMA)
    if (!OctalSpiRx::Initialize()) {
        printf("[GpuCore] ERROR: Octal SPI init failed\n");
        return false;
    }
    printf("[GpuCore] Octal SPI receiver initialized\n");

    // 6. I2C slave (IRQ-driven)
    if (!I2CSlave::Initialize()) {
        printf("[GpuCore] ERROR: I2C slave init failed\n");
        return false;
    }
    printf("[GpuCore] I2C slave initialized at address 0x%02X\n",
           GpuConfig::I2C_ADDRESS);

    // 7. Scene state (zeroes resource tables)
    sceneState.Reset();
    printf("[GpuCore] Scene state reset (max %u meshes, %u materials)\n",
           GpuConfig::MAX_MESHES, GpuConfig::MAX_MATERIALS);

    // 8. Rasterizer (references to framebuffer, Z-buffer, scene)
    rasterizer.Initialize(&sceneState, zBuffer,
                          GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT);
    printf("[GpuCore] Rasterizer initialized\n");

    // 9. Performance counters (DWT cycle counter)
    PerfCounters::Initialize();

    // 10. Assert RDY — we're ready to receive data
    gpio_put(GpuConfig::RDY_PIN, 1);
    printf("[GpuCore] GPU ready. RDY pin asserted.\n");

    return true;
}

// ─── FPS Measurement ────────────────────────────────────────────────────────

static uint32_t lastPerfReportFrameCount = 0;
static constexpr uint32_t PERF_REPORT_INTERVAL = 60;  // print every 60 frames

static void UpdateFpsMeasurement() {
    uint32_t now = time_us_32();
    if (lastFpsTimeUs == 0) {
        lastFpsTimeUs = now;
        return;
    }

    // Update FPS every second
    uint32_t elapsed = now - lastFpsTimeUs;
    if (elapsed >= 1000000) {
        measuredFps = static_cast<uint16_t>(
            (uint64_t)frameCount * 1000000u / elapsed);
        frameCount = 0;
        lastFpsTimeUs = now;
    }
}

// ─── Core 0 Main Loop ──────────────────────────────────────────────────────

void GpuCore::Core0Main() {
    printf("[GpuCore] Core 0 entering main loop\n");

    while (true) {
        // ── HUB75 refresh (cooperative, one BCM plane per call) ──
        // We call PollRefresh() every iteration to keep the display updating.
        // Each call takes ~12 µs, so this doesn't block the main loop.
        Hub75Driver::PollRefresh();

        // ── Sync brightness from I2C ──
        Hub75Driver::SetBrightness(I2CSlave::GetBrightness());

        // ── Check for GPU reset request from I2C ──
        if (I2CSlave::ConsumeResetRequest()) {
            printf("[GpuCore] GPU reset requested via I2C\n");
            sceneState.Reset();
            memset(frontBuffer, 0, GpuConfig::FRAMEBUF_PIXELS * 2);
            memset(backBuffer, 0, GpuConfig::FRAMEBUF_PIXELS * 2);
        }

        // ── Check for new frame data in SPI ring buffer ──
        const uint8_t* frameData = nullptr;
        uint32_t frameLength = 0;

        PerfCounters::Begin(PerfStage::SpiReceive);
        bool hasFrame = OctalSpiRx::TryGetFrame(&frameData, &frameLength);
        PerfCounters::End(PerfStage::SpiReceive);

        if (!hasFrame) {
            // No complete frame yet — continue refreshing display
            continue;
        }

        // ── BEGIN FRAME PROFILING ──
        PerfCounters::Begin(PerfStage::FrameTotal);

        // ── Parse command buffer → update scene state ──
        PerfCounters::Begin(PerfStage::Parse);
        CommandParser::ParseResult result = CommandParser::Parse(
            frameData, frameLength, &sceneState);

        // Release the ring buffer region
        OctalSpiRx::ConsumeFrame();
        PerfCounters::End(PerfStage::Parse);

        if (result != CommandParser::ParseResult::Ok) {
            // CRC error or malformed — skip this frame, use last good state
            droppedFrames++;
            printf("[GpuCore] Frame parse error %d — dropped (total: %u)\n",
                   static_cast<int>(result), droppedFrames);
            PerfCounters::End(PerfStage::FrameTotal);  // match Begin above
            continue;
        }

        // ── Transform + project + build QuadTree (single-threaded) ──
        PerfCounters::Begin(PerfStage::Transform);
        rasterizer.PrepareFrame(&sceneState);
        PerfCounters::End(PerfStage::Transform);

        // ── Frame signature caching: skip raster if scene unchanged ──
        // When PrepareFrame detects an identical draw list + camera state,
        // it sets frameSkipped=true and returns immediately.  We skip
        // rasterization and reuse the previous back buffer contents.
        if (rasterizer.IsFrameSkipped()) {
            // Still count the frame and update stats
            PerfCounters::End(PerfStage::FrameTotal);  // match Begin above
            frameCount++;
            UpdateFpsMeasurement();
            I2CSlave::UpdateStatus(
                measuredFps,
                static_cast<uint16_t>(droppedFrames),
                OctalSpiRx::GetFreeBytes(),
                0, false);
            continue;
        }

        // ── Signal Core 1: start rasterizing bottom half ──
        // Set elapsed time for animated material evaluation (noise, rainbow).
        rasterizer.SetElapsedTime(elapsedTimeS);
        multicore_fifo_push_blocking(FIFO_CMD_START_RENDER);

        // ── Rasterize top half (Y = 0 to H/2 - 1) ──
        PerfCounters::Begin(PerfStage::RasterTop);
        const uint16_t halfHeight = GpuConfig::PANEL_HEIGHT / 2;
        rasterizer.RasterizeRange(backBuffer, 0, halfHeight);
        PerfCounters::End(PerfStage::RasterTop);

        // ── Wait for Core 1 to finish bottom half ──
        // Poll instead of blocking so we can keep refreshing the HUB75 display.
        // multicore_fifo_rvalid() returns true when data is available in the FIFO.
        while (!multicore_fifo_rvalid()) {
            Hub75Driver::PollRefresh();
        }
        uint32_t response = multicore_fifo_pop_blocking();
        (void)response;  // should be FIFO_CMD_RENDER_DONE

        // ── Accumulate elapsed time for animated shaders ──
        // Wrap at 3600 s (1 hour) to prevent float32 precision loss.
        // Animated shaders use periodic functions (sin/cos), so wrapping is safe.
        elapsedTimeS += sceneState.frameTimeUs / 1000000.0f;
        if (elapsedTimeS > 3600.0f) {
            elapsedTimeS = fmodf(elapsedTimeS, 3600.0f);
        }

        // ── Apply screen-space post-processing shaders ──
        PerfCounters::Begin(PerfStage::Shaders);
        // Reuse the Z-buffer as scratch (32 KB float → reinterpret as uint16_t).
        // Z data is stale after rasterization, so this is safe.
        ScreenspaceShaders::ApplyShaders(
            backBuffer,
            reinterpret_cast<uint16_t*>(zBuffer),
            GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT,
            &sceneState,
            elapsedTimeS);
        PerfCounters::End(PerfStage::Shaders);

        // ── Swap framebuffers ──
        PerfCounters::Begin(PerfStage::Swap);
        uint16_t* temp = frontBuffer;
        frontBuffer = backBuffer;
        backBuffer = temp;
        Hub75Driver::SetFramebuffer(frontBuffer);

        // ── Update flow control (RDY pin) with hysteresis ──
        uint32_t freeBytes = OctalSpiRx::GetFreeBytes();
        bool rdyHigh = gpio_get(GpuConfig::RDY_PIN);
        if (rdyHigh) {
            // Currently asserted — deassert when free space drops below 25%
            uint32_t deassertThreshold = GpuConfig::SPI_RING_BUFFER_SIZE
                                       * GpuConfig::RDY_DEASSERT_THRESHOLD / 100;
            if (freeBytes < deassertThreshold) {
                gpio_put(GpuConfig::RDY_PIN, 0);
            }
        } else {
            // Currently deasserted — reassert when free space rises above 50%
            uint32_t assertThreshold = GpuConfig::SPI_RING_BUFFER_SIZE
                                     * GpuConfig::RDY_ASSERT_THRESHOLD / 100;
            if (freeBytes >= assertThreshold) {
                gpio_put(GpuConfig::RDY_PIN, 1);
            }
        }

        // ── Update stats ──
        PerfCounters::End(PerfStage::Swap);
        PerfCounters::End(PerfStage::FrameTotal);
        frameCount++;
        UpdateFpsMeasurement();

        // ── Update I2C status response ──
        I2CSlave::UpdateStatus(
            measuredFps,
            static_cast<uint16_t>(droppedFrames),
            OctalSpiRx::GetFreeBytes(),
            0,    // temperature: not measured yet
            false // renderBusy: cleared after render complete
        );

        // ── Periodic profiling report ──
        lastPerfReportFrameCount++;
        if (lastPerfReportFrameCount >= PERF_REPORT_INTERVAL) {
            PerfCounters::PrintReport();
            PerfCounters::ResetAll();
            lastPerfReportFrameCount = 0;
        }
    }
}

// ─── Core 1 Main Loop ──────────────────────────────────────────────────────

void GpuCore::Core1Main() {
    const uint16_t halfHeight = GpuConfig::PANEL_HEIGHT / 2;
    printf("[GpuCore] Core 1 entering render loop\n");

    while (true) {
        // Wait for Core 0 to signal: QuadTree is ready, start rasterizing
        uint32_t cmd = multicore_fifo_pop_blocking();

        if (cmd == FIFO_CMD_START_RENDER) {
            // Rasterize bottom half (Y = H/2 to H-1)
            PerfCounters::Begin(PerfStage::RasterBottom);
            rasterizer.RasterizeRange(backBuffer, halfHeight,
                                      GpuConfig::PANEL_HEIGHT);
            PerfCounters::End(PerfStage::RasterBottom);

            // Signal completion to Core 0
            multicore_fifo_push_blocking(FIFO_CMD_RENDER_DONE);
        }
    }
}
