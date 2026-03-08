/**
 * @file gpu_core.cpp
 * @brief GPU core lifecycle implementation — RP2350 dual-core.
 *
 * Both normal and headless self-test modes share the SAME ProtoGL graphics
 * pipeline:  SceneState → Rasterizer.PrepareFrame() → PglTileScheduler
 * (dual-core work-stealing, 32 tiles in Morton order) → display.
 *
 * Normal mode:
 *   Display  = HUB75 128×64 LED panel
 *   Scene    = received via Octal SPI from host and parsed by CommandParser
 *   Extras   = I2C slave, VRAM tiering, screenspace shaders, perf counters
 *
 * Headless self-test (RP2350GPU_HEADLESS_SELFTEST):
 *   Display  = SSD1331 96×64 OLED (center-cropped from 96×96 render output)
 *   Scene    = built-in 2×3 rotating gradient cubes (HeadlessSelfTest module)
 *   Excluded = HUB75, Octal SPI, I2C, VRAM drivers, command parser
 *   Shared   = SceneState, Rasterizer, TileScheduler, dual-core operation
 */

// ─── Common includes (both modes) ──────────────────────────────────────────

#include "gpu_core.h"
#include "gpu_config.h"
#include "scene_state.h"
#include "render/rasterizer.h"
#include "scheduler/pgl_tile_scheduler.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#include <cstdio>
#include <cstring>

// ─── Mode-specific includes ────────────────────────────────────────────────

#ifdef RP2350GPU_HEADLESS_SELFTEST
#include "selftest/ssd1331_driver.h"
#include "selftest/headless_selftest.h"
#else
#include "command_parser.h"
#include "display/hub75_driver.h"
#include "transport/octal_spi_rx.h"
#include "transport/i2c_slave.h"
#include "render/screenspace_effects.h"
#include "render/rasterizer_2d.h"
#include "profiling/perf_counters.h"
#include "memory/mem_qspi_vram.h"
#include "memory/mem_tier.h"
#include "memory/mem_pool.h"
#include "display/display_manager.h"
#include "display/hub75_display_driver.h"
#include "display/i2c_hud_driver.h"
#include "memory/flash_persist.h"
#include "selftest/gpu_selftest.h"
extern const GpuConfig::QspiChipProfile& GetQspiChipProfile();
#endif

// ═══════════════════════════════════════════════════════════════════════════
// Common GPU state — shared ProtoGL pipeline for both modes.
// Renders at 128×64 (PANEL_WIDTH × PANEL_HEIGHT) regardless of output.
// ═══════════════════════════════════════════════════════════════════════════

/// Runtime-detected VRAM mode (set during Initialize)
static GpuCore::VramMode g_vramMode = GpuCore::VramMode::SRAM_ONLY;

/// Double framebuffers (RGB565, 128×64 = 16 KB each)
static uint16_t framebufferA[GpuConfig::FRAMEBUF_PIXELS];
static uint16_t framebufferB[GpuConfig::FRAMEBUF_PIXELS];
static uint16_t* frontBuffer = framebufferA;
static uint16_t* backBuffer  = framebufferB;

/// Z-buffer (shared between cores — each core writes distinct tiles).
/// 128×64 = 16 KB.  Cleared to 0xFFFF by Rasterizer::PrepareFrame().
static uint16_t zBuffer[GpuConfig::FRAMEBUF_PIXELS];

/// Scene state (meshes, materials, cameras, draw calls, resource pools ~194 KB)
static SceneState sceneState;

/// Rasterizer (transform → project → tile rasterize)
static Rasterizer rasterizer;

/// Tile scheduler (dual-core work-stealing, 16×16 tiles, Morton Z-order)
static PglTileScheduler tileScheduler;

/// Common frame timing
static uint32_t frameCount    = 0;
static uint32_t lastFpsTimeUs = 0;
static uint16_t measuredFps   = 0;
static float    elapsedTimeS  = 0.0f;

// ═══════════════════════════════════════════════════════════════════════════
// Normal-mode-only state (transport, VRAM, profiling)
// ═══════════════════════════════════════════════════════════════════════════

#ifndef RP2350GPU_HEADLESS_SELFTEST

static QspiVramDriver*  g_vramDriverPtr = nullptr;

static QspiVramDriver   vramDriver;
static MemTierManager   memTierManager;

/// M11: Display abstraction layer
static DisplayManager      displayManager;
static Hub75DisplayDriver  hub75Display;
static I2cHudDriver        hudDisplay;

/// M11: Memory pool manager
static MemPoolManager      memPoolManager;

/// M12: Flash persistence manager
static FlashPersistManager flashPersist;

static uint32_t droppedFrames = 0;

static uint32_t lastPerfReportFrameCount = 0;
static constexpr uint32_t PERF_REPORT_INTERVAL = 60;

#endif // !RP2350GPU_HEADLESS_SELFTEST

// ═══════════════════════════════════════════════════════════════════════════
// Headless-mode-only state (SSD1331 output buffer)
// ═══════════════════════════════════════════════════════════════════════════

#ifdef RP2350GPU_HEADLESS_SELFTEST

/// Output buffer for SSD1331 (96×64, center-cropped from 96×96 pipeline output)
static uint16_t g_ssd1331Fb[GpuConfig::SSD1331_WIDTH * GpuConfig::SSD1331_HEIGHT];

#endif // RP2350GPU_HEADLESS_SELFTEST

// ═══════════════════════════════════════════════════════════════════════════
// M12: 2D Layer Processing — execute queued draw commands, then composite.
// ═══════════════════════════════════════════════════════════════════════════

#ifndef RP2350GPU_HEADLESS_SELFTEST

/// Process the queued 2D draw commands, dispatching each to Rasterizer2D.
static void Process2DDrawQueue(SceneState* scene) {
    if (scene->drawCmd2DCount == 0) return;

    for (uint16_t i = 0; i < scene->drawCmd2DCount; ++i) {
        const DrawCmd2D& cmd = scene->drawCmds2D[i];

        // Use the explicit layerId field set during enqueue
        uint8_t layerId = cmd.layerId;

        if (layerId >= PGL_MAX_LAYERS || !scene->layers[layerId].active ||
            !scene->layers[layerId].pixels) {
            continue;
        }

        const LayerSlot& layer = scene->layers[layerId];
        Rasterizer2D::Target target;
        target.pixels = layer.pixels;
        target.width  = layer.width;
        target.height = layer.height;

        switch (cmd.type) {
            case DRAW_CMD_2D_RECT:
                Rasterizer2D::DrawRect(target, cmd.rect.x, cmd.rect.y,
                                       cmd.rect.w, cmd.rect.h,
                                       cmd.rect.color, cmd.rect.filled != 0);
                break;
            case DRAW_CMD_2D_LINE:
                Rasterizer2D::DrawLine(target, cmd.line.x0, cmd.line.y0,
                                       cmd.line.x1, cmd.line.y1,
                                       cmd.line.color);
                break;
            case DRAW_CMD_2D_CIRCLE:
                Rasterizer2D::DrawCircle(target, cmd.circle.cx, cmd.circle.cy,
                                         cmd.circle.radius, cmd.circle.color,
                                         cmd.circle.filled != 0);
                break;
            case DRAW_CMD_2D_SPRITE: {
                // Look up texture in scene state for source pixels
                uint16_t texId = cmd.sprite.textureId;
                if (texId < GpuConfig::MAX_TEXTURES &&
                    scene->textures[texId].active &&
                    scene->textures[texId].pixels) {
                    Rasterizer2D::DrawSprite(
                        target, cmd.sprite.x, cmd.sprite.y,
                        reinterpret_cast<const uint16_t*>(scene->textures[texId].pixels),
                        scene->textures[texId].width,
                        scene->textures[texId].height,
                        (cmd.sprite.flags & PGL_SPRITE_FLIP_H) != 0,
                        (cmd.sprite.flags & PGL_SPRITE_FLIP_V) != 0);
                }
                break;
            }
            case DRAW_CMD_2D_CLEAR:
                Rasterizer2D::Clear(target, cmd.clear.color);
                break;
            case DRAW_CMD_2D_ROUNDED_RECT:
                Rasterizer2D::DrawRoundedRect(target,
                    cmd.roundedRect.x, cmd.roundedRect.y,
                    cmd.roundedRect.w, cmd.roundedRect.h,
                    cmd.roundedRect.radius, cmd.roundedRect.color,
                    cmd.roundedRect.filled != 0);
                break;
            case DRAW_CMD_2D_ARC:
                Rasterizer2D::DrawArc(target, cmd.arc.cx, cmd.arc.cy,
                                      cmd.arc.radius,
                                      cmd.arc.startAngleDeg, cmd.arc.endAngleDeg,
                                      cmd.arc.color);
                break;
            case DRAW_CMD_2D_TRIANGLE:
                Rasterizer2D::DrawTriangle(target,
                    cmd.triangle.x0, cmd.triangle.y0,
                    cmd.triangle.x1, cmd.triangle.y1,
                    cmd.triangle.x2, cmd.triangle.y2,
                    cmd.triangle.color);
                break;
        }
    }
}

/// Composite 2D layers over the 3D back buffer in ascending layer order.
/// Uses alpha blending based on each layer's blend mode and opacity.
static void CompositeLayers(uint16_t* backBuf, uint16_t panelW, uint16_t panelH,
                            SceneState* scene) {
    if (scene->activeLayerCount == 0) return;

    for (uint8_t layerId = 1; layerId < PGL_MAX_LAYERS; ++layerId) {
        const LayerSlot& layer = scene->layers[layerId];
        if (!layer.active || !layer.visible || !layer.pixels || layer.opacity == 0) {
            continue;
        }

        // Compute overlap region between layer (with offset) and back buffer
        int32_t srcStartX = 0, srcStartY = 0;
        int32_t dstStartX = layer.offsetX, dstStartY = layer.offsetY;
        int32_t copyW = layer.width, copyH = layer.height;

        // Left/top clipping
        if (dstStartX < 0) { srcStartX = -dstStartX; copyW += dstStartX; dstStartX = 0; }
        if (dstStartY < 0) { srcStartY = -dstStartY; copyH += dstStartY; dstStartY = 0; }
        // Right/bottom clipping
        if (dstStartX + copyW > panelW) copyW = panelW - dstStartX;
        if (dstStartY + copyH > panelH) copyH = panelH - dstStartY;

        if (copyW <= 0 || copyH <= 0) continue;

        // Fully opaque fast path — memcpy rows
        if (layer.opacity == 255 && layer.blendMode == PGL_LAYER_BLEND_ALPHA) {
            for (int32_t row = 0; row < copyH; ++row) {
                uint16_t* dst = &backBuf[(dstStartY + row) * panelW + dstStartX];
                const uint16_t* src = &layer.pixels[(srcStartY + row) * layer.width + srcStartX];
                std::memcpy(dst, src, copyW * sizeof(uint16_t));
            }
            continue;
        }

        // Per-pixel blending path
        const uint8_t alpha = layer.opacity;
        for (int32_t row = 0; row < copyH; ++row) {
            uint16_t* dst = &backBuf[(dstStartY + row) * panelW + dstStartX];
            const uint16_t* src = &layer.pixels[(srcStartY + row) * layer.width + srcStartX];

            for (int32_t col = 0; col < copyW; ++col) {
                uint16_t srcPx = src[col];
                uint16_t dstPx = dst[col];

                switch (layer.blendMode) {
                    case PGL_LAYER_BLEND_ALPHA:
                    default:
                        dst[col] = Rasterizer2D::BlendRGB565(srcPx, dstPx, alpha);
                        break;

                    case PGL_LAYER_BLEND_ADDITIVE: {
                        // Additive: clamp(src + dst)
                        uint16_t sr = (srcPx >> 11) & 0x1F, sg = (srcPx >> 5) & 0x3F, sb = srcPx & 0x1F;
                        uint16_t dr = (dstPx >> 11) & 0x1F, dg = (dstPx >> 5) & 0x3F, db = dstPx & 0x1F;
                        sr = (sr * alpha) >> 8; sg = (sg * alpha) >> 8; sb = (sb * alpha) >> 8;
                        uint16_t r = (sr + dr > 31) ? 31 : sr + dr;
                        uint16_t g = (sg + dg > 63) ? 63 : sg + dg;
                        uint16_t b = (sb + db > 31) ? 31 : sb + db;
                        dst[col] = (r << 11) | (g << 5) | b;
                        break;
                    }
                    case PGL_LAYER_BLEND_MULTIPLY: {
                        // Multiply: (src * dst) / max_channel
                        uint16_t sr = (srcPx >> 11) & 0x1F, sg = (srcPx >> 5) & 0x3F, sb = srcPx & 0x1F;
                        uint16_t dr = (dstPx >> 11) & 0x1F, dg = (dstPx >> 5) & 0x3F, db = dstPx & 0x1F;
                        uint16_t r = (sr * dr) / 31;
                        uint16_t g = (sg * dg) / 63;
                        uint16_t b = (sb * db) / 31;
                        // Apply opacity as lerp between result and original dst
                        r = dr + (((int16_t)r - dr) * alpha >> 8);
                        g = dg + (((int16_t)g - dg) * alpha >> 8);
                        b = db + (((int16_t)b - db) * alpha >> 8);
                        dst[col] = (r << 11) | (g << 5) | b;
                        break;
                    }
                }
            }
        }
    }
}

#endif // !RP2350GPU_HEADLESS_SELFTEST

// ─── Initialization ─────────────────────────────────────────────────────────

bool GpuCore::Initialize() {
    printf("[GpuCore] Initializing subsystems...\n");

    // Clear both framebuffers
    memset(framebufferA, 0, sizeof(framebufferA));
    memset(framebufferB, 0, sizeof(framebufferB));

    // ── Display output (mode-specific) ──────────────────────────────────

#ifdef RP2350GPU_HEADLESS_SELFTEST
    // Headless: SSD1331 OLED
    printf("[GpuCore] Headless mode — initializing SSD1331 OLED...\n");
    if (!Ssd1331::Initialize()) {
        printf("[GpuCore] ERROR: SSD1331 init failed\n");
        return false;
    }
    printf("[GpuCore] SSD1331 initialized (%ux%u, SPI0 @ %lu Hz)\n",
           GpuConfig::SSD1331_WIDTH, GpuConfig::SSD1331_HEIGHT,
           (unsigned long)GpuConfig::SSD1331_SPI_BAUD);

#else
    // Normal: DIR + IRQ pins, HUB75 + Octal SPI (bidir) + I2C

    // DIR pin — input from host (high = host TX, low = GPU TX)
    gpio_init(GpuConfig::DIR_PIN);
    gpio_set_dir(GpuConfig::DIR_PIN, GPIO_IN);

    // IRQ pin — output, active-low, initially deasserted (high)
    gpio_init(GpuConfig::IRQ_PIN);
    gpio_set_dir(GpuConfig::IRQ_PIN, GPIO_OUT);
    gpio_put(GpuConfig::IRQ_PIN, 1);  // deasserted (active-low)

    // Fill front buffer with test pattern for initial bringup
    Hub75Driver::FillTestPattern(frontBuffer, 1);

    // HUB75 display driver (PIO + DMA)
    if (!Hub75Driver::Initialize(frontBuffer)) {
        printf("[GpuCore] ERROR: HUB75 init failed\n");
        return false;
    }
    printf("[GpuCore] HUB75 display initialized (%ux%u, 1/%u scan, %u-bit BCM)\n",
           GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT,
           GpuConfig::SCAN_ROWS, GpuConfig::COLOR_DEPTH);

    // Display manager — register HUB75 as primary display (slot 0)
    displayManager.Init();
    displayManager.RegisterDriver(0, &hub75Display);
    printf("[GpuCore] DisplayManager initialized — HUB75 registered in slot 0\n");

    // I2C HUD OLED (slot 1) — non-fatal
    if (hudDisplay.Init(nullptr)) {
        displayManager.RegisterDriver(1, &hudDisplay);
        printf("[GpuCore] I2C HUD initialized on slot 1 (addr 0x%02X)\n",
               I2cHudDriver::DEFAULT_I2C_ADDR);
    } else {
        printf("[GpuCore] WARNING: I2C HUD init failed (non-fatal)\n");
    }

    // Octal SPI transceiver (PIO + DMA, bidirectional)
    if (!OctalSpiRx::Initialize()) {
        printf("[GpuCore] ERROR: Octal SPI init failed\n");
        return false;
    }
    printf("[GpuCore] Octal SPI transceiver initialized (bidir)\n");

    // I2C slave (IRQ-driven) — non-fatal
    if (!I2CSlave::Initialize()) {
        printf("[GpuCore] WARNING: I2C slave not available (non-fatal)\n");
    } else {
        printf("[GpuCore] I2C slave initialized at address 0x%02X\n",
               GpuConfig::I2C_ADDRESS);
    }

#endif // RP2350GPU_HEADLESS_SELFTEST

    // ── Common: Scene state ─────────────────────────────────────────────

    sceneState.Reset();
#ifndef RP2350GPU_HEADLESS_SELFTEST
    I2CSlave::SetSceneState(&sceneState);
#endif
    printf("[GpuCore] Scene state reset (max %u meshes, %u materials)\n",
           GpuConfig::MAX_MESHES, GpuConfig::MAX_MATERIALS);

    // ── Common: Rasterizer ──────────────────────────────────────────────

    rasterizer.Initialize(&sceneState, zBuffer,
                          GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT);
    printf("[GpuCore] Rasterizer initialized (%ux%u)\n",
           GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT);

    // ── Common: Tile scheduler (dual-core work-stealing) ────────────────

    tileScheduler.Initialize();
    printf("[GpuCore] Tile scheduler initialized (%u tiles, %ux%u px each)\n",
           TileConfig::TILE_COUNT, TileConfig::TILE_W, TileConfig::TILE_H);

    // ── Mode-specific post-init ─────────────────────────────────────────

#ifdef RP2350GPU_HEADLESS_SELFTEST
    // Build self-test scene: six rotating gradient cubes in a 2×3 grid,
    // perspective camera at z=-7
    HeadlessSelfTest::BuildScene(&sceneState);
    g_vramMode = VramMode::SRAM_ONLY;
    printf("[GpuCore] Headless scene built — pipeline ready.\n");
    printf("[GpuCore] Render: %ux%u (%u tiles, dual-core) → center-crop → %ux%u (SSD1331)\n",
           GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT,
           TileConfig::TILE_COUNT,
           GpuConfig::SSD1331_WIDTH, GpuConfig::SSD1331_HEIGHT);

#else
    // Performance counters (DWT cycle counter)
    PerfCounters::Initialize();

    // Memory subsystem (QSPI VRAM channels + tier manager)
    {
        // Initialize QSPI VRAM driver (dual-channel PIO2)
        QspiVramDriver* vramPtr = nullptr;

        if (GpuConfig::QspiVramEnabled()) {
            // Channel A
            if (GpuConfig::QSPI_A_CHIP_COUNT > 0) {
                QspiVramChannelConfig cfgA = {};
                cfgA.channel       = QspiChannel::A;
                cfgA.pioInstance   = 2;
                cfgA.dataBasePin   = GpuConfig::QSPI_A_DATA_BASE_PIN;
                cfgA.dataPinCount  = GpuConfig::QSPI_A_DATA_PIN_COUNT;
                cfgA.clkPin        = GpuConfig::QSPI_A_CLK_PIN;
                cfgA.cs0Pin        = GpuConfig::QSPI_A_CS0_PIN;
                cfgA.cs1Pin        = GpuConfig::QSPI_A_CS1_PIN;
                cfgA.chipCount     = GpuConfig::QSPI_A_CHIP_COUNT;

                // TODO: Populate cfgA.chips[] from auto-detect probe at boot
                // For now, use built-in profiles from gpu_config.h

                if (vramDriver.InitChannel(cfgA)) {
                    vramPtr = &vramDriver;
                    printf("[GpuCore] QSPI VRAM Channel A initialized\n");
                } else {
                    printf("[GpuCore] WARNING: QSPI VRAM Channel A init failed\n");
                }
            }

            // Channel B (only in DUAL_CHANNEL mode)
            if (GpuConfig::QspiChannelBEnabled() &&
                GpuConfig::QSPI_B_CHIP_COUNT > 0) {
                QspiVramChannelConfig cfgB = {};
                cfgB.channel       = QspiChannel::B;
                cfgB.pioInstance   = 2;
                cfgB.dataBasePin   = GpuConfig::QSPI_B_DATA_BASE_PIN;
                cfgB.dataPinCount  = GpuConfig::QSPI_B_DATA_PIN_COUNT;
                cfgB.clkPin        = GpuConfig::QSPI_B_CLK_PIN;
                cfgB.cs0Pin        = GpuConfig::QSPI_B_CS0_PIN;
                cfgB.cs1Pin        = GpuConfig::QSPI_B_CS1_PIN;
                cfgB.chipCount     = GpuConfig::QSPI_B_CHIP_COUNT;

                // TODO: Populate cfgB.chips[] from auto-detect probe at boot

                if (vramDriver.InitChannel(cfgB)) {
                    if (!vramPtr) vramPtr = &vramDriver;
                    printf("[GpuCore] QSPI VRAM Channel B initialized\n");
                } else {
                    printf("[GpuCore] WARNING: QSPI VRAM Channel B init failed\n");
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

        if (vramPtr) {
            tierCfg.qspiACapacity = vramPtr->GetCapacity(QspiChannel::A);
            tierCfg.qspiAIsMram   = vramPtr->IsMram(QspiChannel::A);
            tierCfg.qspiAHasRandomAccessPenalty =
                vramPtr->HasRandomAccessPenalty(QspiChannel::A);

            tierCfg.qspiBCapacity = vramPtr->GetCapacity(QspiChannel::B);
            tierCfg.qspiBIsMram   = vramPtr->IsMram(QspiChannel::B);
            tierCfg.qspiBHasRandomAccessPenalty =
                vramPtr->HasRandomAccessPenalty(QspiChannel::B);
            tierCfg.qspiBIsNonVolatile =
                vramPtr->IsNonVolatile(QspiChannel::B);
        }

        // VRAMless mode detection
        {
            bool hasA = vramPtr && vramPtr->IsChannelInitialized(QspiChannel::A);
            bool hasB = vramPtr && vramPtr->IsChannelInitialized(QspiChannel::B);

            if (hasA && hasB) {
                g_vramMode = VramMode::DUAL_CHANNEL;
            } else if (hasA) {
                g_vramMode = VramMode::QSPI_A_ONLY;
            } else if (hasB) {
                g_vramMode = VramMode::QSPI_B_ONLY;
            } else {
                g_vramMode = VramMode::SRAM_ONLY;
            }

            g_vramDriverPtr = vramPtr;

            if (g_vramMode == VramMode::SRAM_ONLY) {
                tierCfg.sramCacheBudget = 0;
                printf("[GpuCore] *** VRAMless mode — all resources in internal SRAM ***\n");
                printf("[GpuCore] SRAM cache budget: 0 (no external memory detected)\n");
                printf("[GpuCore] Memory layout (SRAM only):\n");
                printf("[GpuCore]   Framebuffers (x2):  %u KB\n",
                       (unsigned)(GpuConfig::FRAMEBUF_SIZE * 2 / 1024));
                printf("[GpuCore]   Z-buffer:           %u KB\n",
                       (unsigned)(GpuConfig::FRAMEBUF_PIXELS * 2 / 1024));
                printf("[GpuCore]   Vertex pool:        %u KB\n",
                       (unsigned)(GpuConfig::VERTEX_POOL_SIZE * 12 / 1024));
                printf("[GpuCore]   Index pool:         %u KB\n",
                       (unsigned)(GpuConfig::INDEX_POOL_SIZE * 6 / 1024));
                printf("[GpuCore]   Texture pool:       %u KB\n",
                       (unsigned)(GpuConfig::TEXTURE_POOL_SIZE / 1024));
                printf("[GpuCore]   Layout coords:      %u KB\n",
                       (unsigned)(GpuConfig::LAYOUT_COORD_POOL_SIZE * 8 / 1024));
            } else {
                const char* modeStr =
                    (g_vramMode == VramMode::DUAL_CHANNEL)  ? "DUAL (QSPI-A + QSPI-B)" :
                    (g_vramMode == VramMode::QSPI_A_ONLY)   ? "QSPI-A only" :
                    (g_vramMode == VramMode::QSPI_B_ONLY)   ? "QSPI-B only" : "?";
                printf("[GpuCore] VRAM mode: %s\n", modeStr);
                printf("[GpuCore] SRAM cache budget: %u KB\n",
                       (unsigned)(tierCfg.sramCacheBudget / 1024));
            }
        }

        memTierManager.Initialize(tierCfg, vramPtr);

        CommandParser::InitMemory(
            vramPtr, vramPtr, &memTierManager,
            frontBuffer, backBuffer,
            GpuConfig::FRAMEBUF_PIXELS);

        printf("[GpuCore] Memory subsystem initialized\n");
    }

    // Wire display manager and memory pools to command parser and I2C
    CommandParser::InitDisplayAndPools(&displayManager, &memPoolManager);
    I2CSlave::SetDisplayAndPools(&displayManager, &memPoolManager);
    printf("[GpuCore] Display and pool managers wired to parser + I2C\n");

    // Flash persistence (M12) — load manifest, auto-restore persisted resources
    if (flashPersist.Initialize(&memTierManager)) {
        uint16_t restored = flashPersist.AutoRestore();
        if (restored > 0) {
            printf("[GpuCore] Auto-restored %u resources from flash\n", restored);
        }
    } else {
        printf("[GpuCore] WARNING: Flash persistence init failed (non-fatal)\n");
    }

    // Deassert IRQ — GPU initialized and ready
    gpio_put(GpuConfig::IRQ_PIN, 1);  // deasserted (high)
    printf("[GpuCore] GPU ready. IRQ deasserted.\n");

#endif // !RP2350GPU_HEADLESS_SELFTEST

    return true;
}

// ─── FPS Measurement (common) ───────────────────────────────────────────────

static void UpdateFpsMeasurement() {
    uint32_t now = time_us_32();
    if (lastFpsTimeUs == 0) {
        lastFpsTimeUs = now;
        return;
    }
    uint32_t elapsed = now - lastFpsTimeUs;
    if (elapsed >= 1000000) {
        measuredFps = static_cast<uint16_t>(
            (uint64_t)frameCount * 1000000u / elapsed);
        frameCount = 0;
        lastFpsTimeUs = now;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Core 0 Main Loop
// ═══════════════════════════════════════════════════════════════════════════

#ifdef RP2350GPU_HEADLESS_SELFTEST

// ─── Headless Core 0: self-generated scene → standard pipeline → SSD1331 ──

void GpuCore::Core0Main() {
    printf("[GpuCore] Core 0 entering headless render loop\n");
    printf("[GpuCore] Scene: rotating cube (10 s) then Utah teapot, with Lambert shading\n");
    printf("[GpuCore] Pipeline: PrepareFrame → DispatchTilePass (%u tiles, dual-core) → center-crop → SSD1331\n",
           TileConfig::TILE_COUNT);
    // NOTE: All timing (sleep_ms, busy_wait_us, time_us_32) uses the RP2350
    // hardware timer at 1 MHz (from reference clock), NOT clk_sys.  The
    // 360 MHz core overclock has zero effect on delays or timestamps.
    printf("[GpuCore] Timing: hardware timer (1 MHz ref clock), unaffected by clk_sys OC\n");

    float cubeAngleY = 0.0f;
    float cubeAngleX = 0.0f;

    uint32_t fpsFrames  = 0;
    uint32_t lastFpsUs  = time_us_32();
    uint16_t displayFps = 0;
    uint16_t displayCpu = 0;
    uint32_t busyUs     = 0;

    while (true) {
        uint32_t frameStartUs = time_us_32();

        // ── 1. Update draw list with rotating cube angles ───────────────
        HeadlessSelfTest::UpdateDrawList(&sceneState, cubeAngleY, cubeAngleX,
                                             elapsedTimeS);

        // ── 2. PrepareFrame: transform → project → Z-clear → QuadTree ──
        //    Standard ProtoGL pipeline — identical to normal mode.
        rasterizer.PrepareFrame(&sceneState);

        if (!rasterizer.IsFrameSkipped()) {
            // ── 3. Set elapsed time for animated material evaluation ────
            rasterizer.SetElapsedTime(elapsedTimeS);

            // ── 4. Tile rasterization (dual-core, work-stealing) ────────
            //    TILE_COUNT tiles (16×16) in Morton Z-order.  Both cores
            //    share an atomic tile counter.  This is the SAME scheduler
            //    as the normal HUB75 pipeline — no code differences.
            tileScheduler.DispatchTilePass(
                &rasterizer, backBuffer, zBuffer,
                GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT,
                nullptr);  // no HUB75 refresh needed

            // ── 5. Swap framebuffers ────────────────────────────────────
            uint16_t* temp = frontBuffer;
            frontBuffer = backBuffer;
            backBuffer  = temp;
        }

        // ── 6. Center-crop 96×96 → 96×64 for SSD1331 output ────────────
        //    The pipeline renders at 96×96 (square, matching SSD1331 width).
        //    We extract the center 64 rows vertically for the 96×64 display.
        {
            constexpr uint16_t srcW = GpuConfig::PANEL_WIDTH;    // 96
            constexpr uint16_t srcH = GpuConfig::PANEL_HEIGHT;   // 96
            constexpr uint16_t dstW = GpuConfig::SSD1331_WIDTH;  // 96
            constexpr uint16_t dstH = GpuConfig::SSD1331_HEIGHT; // 64
            constexpr uint16_t xOff = (srcW - dstW) / 2;         // 0
            constexpr uint16_t yOff = (srcH - dstH) / 2;         // 16

            for (uint16_t y = 0; y < dstH; ++y) {
                std::memcpy(&g_ssd1331Fb[y * dstW],
                            &frontBuffer[(y + yOff) * srcW + xOff],
                            dstW * sizeof(uint16_t));
            }
        }

        // ── 7. Draw HUD overlay (FPS + CPU %) on SSD1331 buffer ────────
        HeadlessSelfTest::DrawHUD(g_ssd1331Fb,
                                  GpuConfig::SSD1331_WIDTH,
                                  GpuConfig::SSD1331_HEIGHT,
                                  displayFps, displayCpu);

        // ── 8. Push to SSD1331 OLED ─────────────────────────────────────
        Ssd1331::PushFramebuffer(g_ssd1331Fb);

        // ── 9. Advance rotation angles ──────────────────────────────────
        cubeAngleY += 0.04f;
        cubeAngleX += 0.015f;
        if (cubeAngleY > 6.2832f) cubeAngleY -= 6.2832f;
        if (cubeAngleX > 6.2832f) cubeAngleX -= 6.2832f;

        // ── 10. Timing & FPS measurement ────────────────────────────────
        uint32_t frameEndUs  = time_us_32();
        uint32_t frameTimeUs = frameEndUs - frameStartUs;

        elapsedTimeS += frameTimeUs / 1000000.0f;
        if (elapsedTimeS > 3600.0f) elapsedTimeS -= 3600.0f;

        busyUs += frameTimeUs;
        fpsFrames++;
        frameCount++;

        uint32_t elapsed = frameEndUs - lastFpsUs;
        if (elapsed >= 1000000) {
            displayFps = (uint16_t)((uint64_t)fpsFrames * 1000000u / elapsed);
            displayCpu = (uint16_t)((uint64_t)busyUs * 100u / elapsed);
            if (displayCpu > 100) displayCpu = 100;

            printf("[GpuCore] Headless FPS: %u, CPU: %u%%, Frame: %lu us, Tris: %lu\n",
                   displayFps, displayCpu,
                   (unsigned long)frameTimeUs,
                   (unsigned long)rasterizer.GetTriangleCount());

            fpsFrames = 0;
            busyUs    = 0;
            lastFpsUs = frameEndUs;
        }
    }
}

#else // !RP2350GPU_HEADLESS_SELFTEST

// ─── Normal Core 0: SPI receive → parse → prepare → tile raster → HUB75 ──

void GpuCore::Core0Main() {
    printf("[GpuCore] Core 0 entering main loop\n");

    while (true) {
        // ── HUB75 refresh (cooperative, one BCM plane per call) ──
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
            continue;
        }

        // ── BEGIN FRAME PROFILING ──
        PerfCounters::Begin(PerfStage::FrameTotal);

        // ── Parse command buffer → update scene state ──
        PerfCounters::Begin(PerfStage::Parse);
        CommandParser::ParseResult result = CommandParser::Parse(
            frameData, frameLength, &sceneState);
        OctalSpiRx::ConsumeFrame();
        PerfCounters::End(PerfStage::Parse);

        if (result != CommandParser::ParseResult::Ok &&
            result != CommandParser::ParseResult::UnknownOpcode) {
            droppedFrames++;
            printf("[GpuCore] Frame parse error %d — dropped (total: %u)\n",
                   static_cast<int>(result), droppedFrames);
            PerfCounters::End(PerfStage::FrameTotal);
            continue;
        }

        // ── Memory tier: begin-of-frame bookkeeping ──
        memTierManager.BeginFrame();

        // ── Transform + project + build QuadTree ──
        PerfCounters::Begin(PerfStage::Transform);
        rasterizer.PrepareFrame(&sceneState);
        PerfCounters::End(PerfStage::Transform);

        // ── Frame signature caching ──
        if (rasterizer.IsFrameSkipped()) {
            PerfCounters::End(PerfStage::FrameTotal);
            frameCount++;
            UpdateFpsMeasurement();
            I2CSlave::UpdateStatus(
                measuredFps,
                static_cast<uint16_t>(droppedFrames),
                OctalSpiRx::GetFreeBytes(),
                0, false);
            continue;
        }

        // ── Set elapsed time for animated material evaluation ──
        rasterizer.SetElapsedTime(elapsedTimeS);

        // ── Memory tier: prefetch PIO2-resident data ──
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

        // ── Tile rasterization (dual-core, work-stealing) ──
        PerfCounters::Begin(PerfStage::RasterTop);
        tileScheduler.DispatchTilePass(
            &rasterizer, backBuffer, zBuffer,
            GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT,
            Hub75Driver::PollRefresh);
        PerfCounters::End(PerfStage::RasterTop);

        // ── Accumulate elapsed time ──
        elapsedTimeS += sceneState.frameTimeUs / 1000000.0f;
        if (elapsedTimeS > 3600.0f) elapsedTimeS -= 3600.0f;

        // ── Screenspace post-processing shaders ──
        PerfCounters::Begin(PerfStage::Shaders);
        ScreenspaceShaders::ApplyShaders(
            backBuffer, zBuffer,
            GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT,
            &sceneState, elapsedTimeS);
        PerfCounters::End(PerfStage::Shaders);

        // ── 2D layer rendering + compositing (M12) ──
        // Process queued 2D draw commands, then composite layers over 3D output.
        if (sceneState.drawCmd2DCount > 0 || sceneState.activeLayerCount > 0) {
            Process2DDrawQueue(&sceneState);
            CompositeLayers(backBuffer, GpuConfig::PANEL_WIDTH,
                            GpuConfig::PANEL_HEIGHT, &sceneState);
        }

        // ── Swap framebuffers ──
        PerfCounters::Begin(PerfStage::Swap);
        uint16_t* temp = frontBuffer;
        frontBuffer = backBuffer;
        backBuffer = temp;
        Hub75Driver::SetFramebuffer(frontBuffer);
        displayManager.SetPrimaryFramebuffer(frontBuffer);
        CommandParser::UpdateFramebufferPtrs(frontBuffer, backBuffer);

        // ── Memory tier: end-of-frame flush ──
        memTierManager.EndFrame();

        // ── Flash persistence: process writeback queue (one per frame) ──
        flashPersist.ProcessWritebackQueue();

        // ── Update flow control (IRQ pin) with hysteresis ──
        // IRQ is active-low: asserted (0) = backpressure, deasserted (1) = OK
        uint32_t freeBytes = OctalSpiRx::GetFreeBytes();
        bool irqAsserted = !gpio_get(GpuConfig::IRQ_PIN);  // active-low
        if (!irqAsserted) {
            // Currently OK — check if we need to assert backpressure
            uint32_t deassertThreshold = GpuConfig::SPI_RING_BUFFER_SIZE
                                       * GpuConfig::IRQ_DEASSERT_THRESHOLD / 100;
            if (freeBytes < deassertThreshold) {
                gpio_put(GpuConfig::IRQ_PIN, 0);  // assert IRQ (backpressure)
            }
        } else {
            // Currently backpressure — check if we can release
            uint32_t assertThreshold = GpuConfig::SPI_RING_BUFFER_SIZE
                                     * GpuConfig::IRQ_ASSERT_THRESHOLD / 100;
            if (freeBytes >= assertThreshold) {
                gpio_put(GpuConfig::IRQ_PIN, 1);  // deassert IRQ (OK)
            }
        }

        // ── Update stats ──
        PerfCounters::End(PerfStage::Swap);
        PerfCounters::End(PerfStage::FrameTotal);
        frameCount++;
        UpdateFpsMeasurement();

        // ── Update I2C status ──
        I2CSlave::UpdateStatus(
            measuredFps,
            static_cast<uint16_t>(droppedFrames),
            OctalSpiRx::GetFreeBytes(),
            0, false);

        // ── Update I2C extended status ──
        {
            float tempC = I2CSlave::ReadTemperature();
            int16_t tempQ8 = static_cast<int16_t>(tempC * 256.0f);
            uint16_t clkMHz = static_cast<uint16_t>(clock_get_hz(clk_sys) / 1000000);
            uint16_t sramFree = static_cast<uint16_t>(
                520 - (memTierManager.GetSramCacheUsed() / 1024));

            I2CSlave::UpdateExtendedStatus(
                measuredFps,
                static_cast<uint16_t>(droppedFrames),
                0, 0, 0, 0,
                tempQ8, clkMHz, sramFree,
                static_cast<uint16_t>(PerfCounters::stages[static_cast<uint8_t>(PerfStage::FrameTotal)].LastUs()),
                static_cast<uint16_t>(PerfCounters::stages[static_cast<uint8_t>(PerfStage::RasterTop)].LastUs()),
                static_cast<uint16_t>(PerfCounters::stages[static_cast<uint8_t>(PerfStage::SpiReceive)].LastUs()),
                0);
        }

        // ── Update memory tier info for I2C readback ──
        {
            PglMemTierInfoResponse& ti = sceneState.memTierInfo;
            ti.sramTotalKB = 520;
            ti.sramFreeKB  = static_cast<uint16_t>(
                520 - (memTierManager.GetSramCacheUsed() / 1024));
            // Channel A (QSPI-A)
            bool chAInit = vramDriver.IsChannelInitialized(QspiChannel::A);
            ti.opiTotalKB = static_cast<uint16_t>(
                chAInit ? vramDriver.GetCapacity(QspiChannel::A) / 1024 : 0);
            ti.opiFreeKB  = static_cast<uint16_t>(
                chAInit ? vramDriver.Available(QspiChannel::A) / 1024 : 0);
            ti.opiEnabled = chAInit ? 1 : 0;
            // Channel B (QSPI-B)
            bool chBInit = vramDriver.IsChannelInitialized(QspiChannel::B);
            ti.qspiTotalKB = static_cast<uint16_t>(
                chBInit ? vramDriver.GetCapacity(QspiChannel::B) / 1024 : 0);
            ti.qspiFreeKB  = static_cast<uint16_t>(
                chBInit ? vramDriver.Available(QspiChannel::B) / 1024 : 0);
            ti.qspiEnabled = chBInit ? 1 : 0;
        }

        // ── Update HUD OLED (one page per frame) ──
        if (displayManager.GetDriver(1) != nullptr) {
            uint16_t vramUsed = static_cast<uint16_t>(
                memTierManager.GetSramCacheUsed() / 1024);
            uint16_t vramTotal = 520;  // SRAM total KB
            uint16_t frameUs = static_cast<uint16_t>(
                PerfCounters::stages[static_cast<uint8_t>(
                    PerfStage::FrameTotal)].LastUs());
            float htempC = I2CSlave::ReadTemperature();
            int8_t htempCi = static_cast<int8_t>(htempC);
            uint8_t cpuPct = (measuredFps > 0 && frameUs > 0)
                ? static_cast<uint8_t>(
                      (uint32_t)frameUs * measuredFps / 10000)
                : 0;
            if (cpuPct > 100) cpuPct = 100;
            hudDisplay.RenderStatus(
                measuredFps, vramUsed, vramTotal,
                cpuPct, htempCi, frameUs);
            hudDisplay.PollRefresh();  // one 128-byte page per frame
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

#endif // RP2350GPU_HEADLESS_SELFTEST

// ═══════════════════════════════════════════════════════════════════════════
// Core 1 Main Loop — shared by both modes (tile scheduler work-stealing)
// ═══════════════════════════════════════════════════════════════════════════

void GpuCore::Core1Main() {
    printf("[GpuCore] Core 1 entering tile scheduler loop\n");
    tileScheduler.Core1Main();  // never returns
}

// ═══════════════════════════════════════════════════════════════════════════
// VRAM Mode & Driver Access
// ═══════════════════════════════════════════════════════════════════════════

GpuCore::VramMode GpuCore::GetVramMode() {
    return g_vramMode;
}

void GpuCore::GetVramDriver(QspiVramDriver** vramOut) {
#ifdef RP2350GPU_HEADLESS_SELFTEST
    if (vramOut) *vramOut = nullptr;
#else
    if (vramOut) *vramOut = g_vramDriverPtr;
#endif
}

void GpuCore::GetVramDrivers(QspiVramDriver** opiOut, QspiVramDriver** qspiOut) {
#ifdef RP2350GPU_HEADLESS_SELFTEST
    if (opiOut)  *opiOut  = nullptr;
    if (qspiOut) *qspiOut = nullptr;
#else
    if (opiOut)  *opiOut  = g_vramDriverPtr;
    if (qspiOut) *qspiOut = g_vramDriverPtr;
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// Host Wait & HUB75 Self-Test (normal mode only)
// ═══════════════════════════════════════════════════════════════════════════

#ifdef RP2350GPU_HEADLESS_SELFTEST

bool GpuCore::WaitForHost(uint32_t) { return false; }
void GpuCore::RunSelfTest() {}

#else

bool GpuCore::WaitForHost(uint32_t timeoutMs) {
    printf("[GpuCore] Waiting up to %lu ms for host SPI data...\n", (unsigned long)timeoutMs);

    uint32_t startUs = time_us_32();
    uint32_t timeoutUs = timeoutMs * 1000;

    while ((time_us_32() - startUs) < timeoutUs) {
        Hub75Driver::PollRefresh();
        uint32_t freeBytes = OctalSpiRx::GetFreeBytes();
        if (freeBytes < GpuConfig::SPI_RING_BUFFER_SIZE) {
            uint32_t received = GpuConfig::SPI_RING_BUFFER_SIZE - freeBytes;
            printf("[GpuCore] Host SPI data detected (%lu bytes)\n",
                   (unsigned long)received);
            return true;
        }
    }

    printf("[GpuCore] No host SPI data detected after %lu ms\n", (unsigned long)timeoutMs);
    return false;
}

void GpuCore::RunSelfTest() {
    printf("\n[GpuCore] === Entering Self-Test Mode ===\n");
    printf("[GpuCore] VRAM mode: %s\n",
           g_vramMode == VramMode::SRAM_ONLY    ? "SRAM_ONLY (VRAMless)" :
           g_vramMode == VramMode::QSPI_A_ONLY  ? "QSPI_A_ONLY" :
           g_vramMode == VramMode::QSPI_B_ONLY  ? "QSPI_B_ONLY" :
           g_vramMode == VramMode::DUAL_CHANNEL  ? "DUAL_CHANNEL" : "?");

    GpuSelfTest::SelfTestResult result = GpuSelfTest::RunAll(
        frontBuffer, backBuffer, zBuffer,
        g_vramDriverPtr, g_vramDriverPtr,
        GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT);

    printf("[GpuCore] Self-test complete. Holding display (waiting for host)...\n");

    while (true) {
        Hub75Driver::PollRefresh();

        uint32_t freeBytes = OctalSpiRx::GetFreeBytes();
        if (freeBytes < GpuConfig::SPI_RING_BUFFER_SIZE) {
            printf("[GpuCore] Host SPI data detected — exiting self-test mode\n");
            const uint8_t* dummy = nullptr;
            uint32_t dummyLen = 0;
            if (OctalSpiRx::TryGetFrame(&dummy, &dummyLen)) {
                OctalSpiRx::ConsumeFrame();
            }
            break;
        }

        Hub75Driver::SetBrightness(I2CSlave::GetBrightness());
    }
}

#endif // RP2350GPU_HEADLESS_SELFTEST
