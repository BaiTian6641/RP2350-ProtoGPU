/**
 * @file gpu_core.cpp
 * @brief GPU core lifecycle implementation — RP2350 dual-core.
 *
 * Core 0: SPI receive → parse → prepare frame → tile raster (work-stealing) → swap → HUB75 refresh
 * Core 1: Tile raster (work-stealing, Morton Z-order, FIFO-triggered)
 *
 * Rasterisation is tile-based: the 128×64 panel is divided into 32 tiles of
 * 16×16 pixels, dispatched via PglTileScheduler.  Both cores pull tiles from
 * a lock-free atomic counter, giving natural load balancing.
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
#include "memory/mem_opi_psram.h"
#include "memory/mem_qspi_psram.h"
#include "memory/mem_tier.h"
#include "scheduler/pgl_tile_scheduler.h"

// Free function defined in i2c_slave.cpp — returns the auto-detected CS1 chip profile
extern const GpuConfig::QspiChipProfile& GetQspiChipProfile();

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include <cstdio>
#include <cstring>

// ─── GPU State ──────────────────────────────────────────────────────────────

/// Double framebuffers (RGB565)
static uint16_t framebufferA[GpuConfig::FRAMEBUF_PIXELS];
static uint16_t framebufferB[GpuConfig::FRAMEBUF_PIXELS];

/// Pointers for double-buffering: display reads front, rasterizer writes back
static uint16_t* frontBuffer = framebufferA;
static uint16_t* backBuffer  = framebufferB;

/// Z-buffer (shared between cores — each core writes distinct tiles)
static float zBuffer[GpuConfig::FRAMEBUF_PIXELS];

/// Scene and render state
static SceneState   sceneState;
static Rasterizer   rasterizer;
static uint32_t     frameCount = 0;
static uint32_t     droppedFrames = 0;
static uint32_t     lastFpsTimeUs = 0;
static uint16_t     measuredFps = 0;
static float        elapsedTimeS = 0.0f;   // accumulated wall time for animated effects

/// Memory subsystem
static OpiPsramDriver  opiDriver;
static QspiPsramDriver qspiDriver;
static MemTierManager  memTierManager;

/// Tile-based dual-core scheduler (bare-metal, work-stealing)
static PglTileScheduler tileScheduler;

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

    // 10. Tile scheduler (bare-metal, work-stealing, dual-core)
    tileScheduler.Initialize();

    // 11. Memory subsystem (PIO2 external + QSPI CS1 + tier manager)
    {
        // Initialize PIO2 external memory driver (if configured)
        OpiPsramDriver* opiPtr = nullptr;
        if (GpuConfig::PIO2_MEM_MODE != GpuConfig::Pio2MemMode::NONE) {
            OpiPsramConfig opiCfg = {};
            opiCfg.mode           = static_cast<Pio2MemMode>(
                                        static_cast<uint8_t>(GpuConfig::PIO2_MEM_MODE));
            opiCfg.pioInstance    = 2;  // PIO2
            opiCfg.dataBasePin    = GpuConfig::PIO2_MEM_DATA_BASE_PIN;
            opiCfg.dataPinCount   = GpuConfig::Pio2DataPinCount();
            opiCfg.clkPin         = GpuConfig::PIO2_MEM_CLK_PIN;
            opiCfg.cs0Pin         = GpuConfig::PIO2_MEM_CS0_PIN;
            opiCfg.cs1Pin         = GpuConfig::QSPI_MRAM_CS1_PIN;
            opiCfg.readLatency    = GpuConfig::Pio2IsMram()
                                  ? GpuConfig::QSPI_MRAM_PIO_READ_LAT
                                  : GpuConfig::OPI_PSRAM_READ_LATENCY;
            opiCfg.clockMHz       = GpuConfig::Pio2IsMram()
                                  ? GpuConfig::QSPI_MRAM_PIO_CLOCK_MHZ
                                  : GpuConfig::OPI_PSRAM_CLOCK_MHZ;
            opiCfg.capacityBytes  = GpuConfig::Pio2MemCapacity();
            opiCfg.chipCapacity   = GpuConfig::Pio2IsMram()
                                  ? GpuConfig::QSPI_MRAM_CHIP_CAPACITY : 0;
            opiCfg.chipCount      = (GpuConfig::PIO2_MEM_MODE ==
                                     GpuConfig::Pio2MemMode::DUAL_QSPI_MRAM) ? 2 : 1;
            opiCfg.isMram              = GpuConfig::Pio2IsMram();
            opiCfg.hasRandomAccessPenalty = !GpuConfig::Pio2IsMram();
            opiCfg.isNonVolatile       = GpuConfig::Pio2IsMram();
            opiCfg.needsWrenBeforeWrite = GpuConfig::Pio2IsMram();
            opiCfg.expectedRdid        = GpuConfig::Pio2IsMram()
                                       ? GpuConfig::QSPI_MRAM_PIO_RDID : 0;

            if (opiDriver.Initialize(opiCfg)) {
                opiPtr = &opiDriver;
                printf("[GpuCore] PIO2 external memory initialized (%lu KB, %s)\n",
                       (unsigned long)(opiCfg.capacityBytes / 1024),
                       opiCfg.isMram ? "MRAM" : "PSRAM");
            } else {
                printf("[GpuCore] WARNING: PIO2 external memory init failed\n");
            }
        }

        // Initialize QSPI CS1 driver (if auto-detected by ProbeQspiCs1)
        QspiPsramDriver* qspiPtr = nullptr;
        {
            const auto& chipProfile = GetQspiChipProfile();
            if (chipProfile.type != GpuConfig::QspiChipType::NONE) {
                QspiPsramConfig qspiCfg = {};
                qspiCfg.xipBase        = GpuConfig::QSPI_CS1_XIP_BASE;
                qspiCfg.capacityBytes  = chipProfile.capacityBytes;
                qspiCfg.clockMHz       = chipProfile.maxClockMHz;
                qspiCfg.readLatency    = chipProfile.readLatencyDummy;
                qspiCfg.chipType       = static_cast<uint8_t>(chipProfile.type);
                qspiCfg.hasRandomAccessPenalty = chipProfile.hasRandomAccessPenalty;
                qspiCfg.isNonVolatile  = chipProfile.isNonVolatile;
                qspiCfg.needsWrenBeforeWrite = chipProfile.needsWrenBeforeWrite;
                qspiCfg.needsRefresh   = chipProfile.needsRefresh;
                qspiCfg.expectedRdid   = chipProfile.deviceId;

                if (qspiDriver.Initialize(qspiCfg)) {
                    qspiPtr = &qspiDriver;
                    printf("[GpuCore] QSPI CS1 memory initialized (%lu KB, %s)\n",
                           (unsigned long)(qspiCfg.capacityBytes / 1024),
                           chipProfile.hasRandomAccessPenalty ? "PSRAM" : "MRAM");
                } else {
                    printf("[GpuCore] WARNING: QSPI CS1 memory init failed\n");
                }
            }
        }

        // Initialize tier manager
        MemTierConfig tierCfg = {};
        tierCfg.sramCacheBudget     = GpuConfig::MEM_TIER_SRAM_CACHE_BUDGET;
        tierCfg.cacheLineSize       = GpuConfig::MEM_TIER_CACHE_LINE_SIZE;
        tierCfg.alphaWeight         = GpuConfig::MEM_TIER_ALPHA_WEIGHT;
        tierCfg.betaScore           = GpuConfig::MEM_TIER_BETA_SCORE;
        tierCfg.demotionThreshold   = GpuConfig::MEM_TIER_DEMOTION_THRESHOLD;
        tierCfg.promotionHysteresis = GpuConfig::MEM_TIER_PROMOTION_HYSTERESIS;
        tierCfg.opiCapacity  = opiPtr  ? opiPtr->GetCapacity()  : 0;
        tierCfg.pio2IsMram   = opiPtr  ? opiPtr->IsMram()       : false;
        tierCfg.pio2HasRandomAccessPenalty = opiPtr ? opiPtr->HasRandomAccessPenalty() : true;
        tierCfg.qspiCapacity = qspiPtr ? qspiPtr->GetCapacity() : 0;
        tierCfg.qspiXipBase  = GpuConfig::QSPI_CS1_XIP_BASE;
        tierCfg.qspiHasRandomAccessPenalty = qspiPtr ? qspiPtr->HasRandomAccessPenalty() : true;
        tierCfg.qspiIsNonVolatile          = qspiPtr ? qspiPtr->IsNonVolatile()          : false;

        memTierManager.Initialize(tierCfg, opiPtr, qspiPtr);

        // Wire memory subsystem into the command parser
        CommandParser::InitMemory(
            opiPtr, qspiPtr, &memTierManager,
            frontBuffer, backBuffer,
            GpuConfig::FRAMEBUF_PIXELS);

        printf("[GpuCore] Memory subsystem initialized\n");
    }

    // 12. Assert RDY — we're ready to receive data
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

        // ── Memory tier: begin-of-frame bookkeeping ──
        // Decay access scores, evaluate promotion/demotion.
        memTierManager.BeginFrame();

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

        // ── Set elapsed time for animated material evaluation (noise, rainbow) ──
        rasterizer.SetElapsedTime(elapsedTimeS);

        // ── Memory tier: prefetch PIO2-resident data into SRAM cache ──
        // Build ID arrays from the draw list for prefetch.
        // (Lightweight — just indexes into the existing draw list.)
        {
            static uint16_t meshIds[GpuConfig::MAX_DRAW_CALLS];
            static uint16_t matIds[GpuConfig::MAX_DRAW_CALLS];
            for (uint16_t i = 0; i < sceneState.drawCallCount; ++i) {
                meshIds[i] = sceneState.drawList[i].meshId;
                matIds[i]  = sceneState.drawList[i].materialId;
            }
            memTierManager.PrefetchForDrawList(
                meshIds, matIds, nullptr, sceneState.drawCallCount);
        }

        // ── Rasterize via tile scheduler (32 tiles, work-stealing) ──
        // Both cores pull 16×16 tiles from an atomic counter in Morton order.
        // Each tile queries the QuadTree once (256-pixel amortisation).
        PerfCounters::Begin(PerfStage::RasterTop);   // reuse stage for "raster total"
        tileScheduler.DispatchTilePass(
            &rasterizer, backBuffer, zBuffer,
            GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT,
            Hub75Driver::PollRefresh);
        PerfCounters::End(PerfStage::RasterTop);

        // ── Accumulate elapsed time for animated shaders ──
        // Wrap at 3600 s (1 hour) to prevent float32 precision loss.
        // Animated shaders use periodic functions (sin/cos), so wrapping is safe.
        elapsedTimeS += sceneState.frameTimeUs / 1000000.0f;
        if (elapsedTimeS > 3600.0f) {
            elapsedTimeS -= 3600.0f;
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

        // ── Update command parser's framebuffer pointers after swap ──
        CommandParser::UpdateFramebufferPtrs(frontBuffer, backBuffer);

        // ── Memory tier: end-of-frame flush ──
        // Write back dirty SRAM cache lines to external memory.
        memTierManager.EndFrame();

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

        // ── Update memory tier info for I2C readback ──
        {
            PglMemTierInfoResponse& ti = sceneState.memTierInfo;
            // Tier 0 — SRAM
            ti.sramTotalKB = 520;  // RP2350 has 520 KB SRAM
            ti.sramFreeKB  = static_cast<uint16_t>(
                520 - (memTierManager.GetSramCacheUsed() / 1024));
            // Tier 1 — PIO2
            ti.opiTotalKB = static_cast<uint16_t>(
                opiDriver.IsInitialized() ? opiDriver.GetCapacity() / 1024 : 0);
            ti.opiFreeKB  = static_cast<uint16_t>(
                opiDriver.IsInitialized() ? opiDriver.Available() / 1024 : 0);
            ti.opiEnabled = opiDriver.IsInitialized() ? 1 : 0;
            // Tier 2 — QSPI CS1
            ti.qspiTotalKB = static_cast<uint16_t>(
                qspiDriver.IsInitialized() ? qspiDriver.GetCapacity() / 1024 : 0);
            ti.qspiFreeKB  = static_cast<uint16_t>(
                qspiDriver.IsInitialized() ? qspiDriver.Available() / 1024 : 0);
            ti.qspiEnabled = qspiDriver.IsInitialized() ? 1 : 0;
            // Cache stats
            // (cachedEntries, cacheHitRate left as-is for now)
        }

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
    printf("[GpuCore] Core 1 entering tile scheduler loop\n");
    tileScheduler.Core1Main();  // never returns — work-stealing tile loop
}
