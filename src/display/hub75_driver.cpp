/**
 * @file hub75_driver.cpp
 * @brief PIO-driven HUB75 display driver implementation for RP2350.
 *
 * Architecture:
 *   SM0 (hub75_data) — shifts out pixel data with side-set CLK
 *   SM1 (hub75_row)  — controls LAT, OE, row address (A-E)
 *   DMA channel 0    — feeds SM0 from BCM plane buffer (pixel data)
 *   DMA channel 1    — auto-chains to restart after each row (control channel)
 *
 * BCM (Binary Code Modulation):
 *   For COLOR_DEPTH bits of color depth, we make COLOR_DEPTH passes per row pair.
 *   Each pass shifts the bit-plane for that BCM weight, then OE is held for
 *   2^bit_weight PIO cycles.  This gives perceived brightness proportional to
 *   the binary value.
 *
 * Refresh loop:
 *   For each row pair (0..SCAN_ROWS-1):
 *     For each BCM bit (0..COLOR_DEPTH-1):
 *       1. Extract bit-plane data from RGB565 framebuffer
 *       2. DMA the bit-plane to SM0 (hub75_data)
 *       3. SM1 (hub75_row): blank → latch → set row addr → OE for 2^bit cycles
 *
 * The refresh runs from a PIO IRQ handler to be essentially zero-CPU once
 * started.  At frame swap, only the framebuffer pointer is updated atomically.
 */

#include "hub75_driver.h"
#include "../gpu_config.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

// Generated PIO headers (from hub75.pio via pico_generate_pio_header)
#include "hub75.pio.h"

#include <cstring>
#include <cstdio>

// ─── Constants ──────────────────────────────────────────────────────────────

static constexpr uint16_t PANEL_W     = GpuConfig::PANEL_WIDTH;   // 128
static constexpr uint16_t PANEL_H     = GpuConfig::PANEL_HEIGHT;  // 64
static constexpr uint8_t  SCAN_ROWS   = GpuConfig::SCAN_ROWS;     // 32
static constexpr uint8_t  COLOR_DEPTH = GpuConfig::COLOR_DEPTH;   // 8

/// One pixel = one 32-bit word with 6 data bits [5:0] = R1 G1 B1 R2 G2 B2
/// Row-pair buffer: PANEL_W pixels × 4 bytes = 512 bytes per row per BCM bit
static uint32_t bcmRowBuffer[PANEL_W];

// ─── PIO / DMA State ────────────────────────────────────────────────────────

static PIO   pio_hw_inst      = pio0;
static uint  sm_data          = 0;        // SM index for hub75_data
static uint  sm_row           = 1;        // SM index for hub75_row
static uint  data_prog_offset = 0;
static uint  row_prog_offset  = 0;
static int   dma_data_chan     = -1;      // DMA channel feeding SM0

static volatile const uint16_t* active_framebuffer = nullptr;
static uint8_t  currentBrightness = 255;

// ─── Refresh State (driven from main-loop poll or timer) ────────────────────

static volatile uint8_t  currentRow   = 0;   // 0..SCAN_ROWS-1
static volatile uint8_t  currentBit   = 0;   // 0..COLOR_DEPTH-1
static volatile uint32_t refreshCount = 0;
static volatile uint32_t lastRefreshUs = 0;
static volatile uint32_t measuredRefreshHz = 0;

// ─── BCM Bit-Plane Extraction ───────────────────────────────────────────────

/**
 * @brief Extract a single BCM bit plane from the RGB565 framebuffer.
 *
 * For a given row pair (topRow and bottomRow = topRow + SCAN_ROWS) and
 * a given BCM bit index, produce PANEL_W 32-bit words where bits [5:0] are:
 *   bit 0 = R1 (top row red, this BCM bit)
 *   bit 1 = G1 (top row green)
 *   bit 2 = B1 (top row blue)
 *   bit 3 = R2 (bottom row red)
 *   bit 4 = G2 (bottom row green)
 *   bit 5 = B2 (bottom row blue)
 *
 * RGB565 layout: RRRRR GGGGGG BBBBB
 *   R = bits[15:11] (5 bits → map to BCM bits 0-7 by shifting)
 *   G = bits[10:5]  (6 bits → map to BCM bits 0-7 by shifting)
 *   B = bits[4:0]   (5 bits → map to BCM bits 0-7 by shifting)
 *
 * For 5-bit channels → 8-bit BCM: replicate top bits (5→8 by shift + OR top 3)
 * For 6-bit green → 8-bit BCM:    replicate top bits (6→8 by shift + OR top 2)
 */
static void ExtractBcmPlane(const uint16_t* fb, uint8_t row, uint8_t bit,
                            uint32_t* dest) {
    const uint16_t* topRowPtr = fb + (row * PANEL_W);
    const uint16_t* botRowPtr = fb + ((row + SCAN_ROWS) * PANEL_W);

    for (uint16_t x = 0; x < PANEL_W; x++) {
        uint16_t topPx = topRowPtr[x];
        uint16_t botPx = botRowPtr[x];

        // Extract 8-bit channels from RGB565
        // R: bits[15:11] → 5 bits, shift left 3 for 8-bit
        // G: bits[10:5]  → 6 bits, shift left 2 for 8-bit
        // B: bits[4:0]   → 5 bits, shift left 3 for 8-bit
        uint8_t tr = (topPx >> 8) & 0xF8;  // top row red (top 5 bits → 8-bit)
        uint8_t tg = (topPx >> 3) & 0xFC;  // top row green (top 6 bits → 8-bit)
        uint8_t tb = (topPx << 3) & 0xF8;  // top row blue (top 5 bits → 8-bit)

        uint8_t br = (botPx >> 8) & 0xF8;
        uint8_t bg = (botPx >> 3) & 0xFC;
        uint8_t bb = (botPx << 3) & 0xF8;

        // Apply brightness scaling (linear scale, not gamma corrected here)
        if (currentBrightness < 255) {
            tr = (uint16_t)tr * currentBrightness >> 8;
            tg = (uint16_t)tg * currentBrightness >> 8;
            tb = (uint16_t)tb * currentBrightness >> 8;
            br = (uint16_t)br * currentBrightness >> 8;
            bg = (uint16_t)bg * currentBrightness >> 8;
            bb = (uint16_t)bb * currentBrightness >> 8;
        }

        // Extract the specific BCM bit from each 8-bit channel
        uint32_t pixel = 0;
        pixel |= ((tr >> bit) & 1) << 0;  // R1
        pixel |= ((tg >> bit) & 1) << 1;  // G1
        pixel |= ((tb >> bit) & 1) << 2;  // B1
        pixel |= ((br >> bit) & 1) << 3;  // R2
        pixel |= ((bg >> bit) & 1) << 4;  // G2
        pixel |= ((bb >> bit) & 1) << 5;  // B2

        dest[x] = pixel;
    }
}

// ─── PIO Initialization Helpers ─────────────────────────────────────────────

static void InitDataSM() {
    // Load hub75_data PIO program
    data_prog_offset = pio_add_program(pio_hw_inst, &hub75_data_program);
    sm_data = pio_claim_unused_sm(pio_hw_inst, true);

    pio_sm_config c = hub75_data_program_get_default_config(data_prog_offset);

    // OUT pins: R1, G1, B1, R2, G2, B2 (6 consecutive starting at R1_PIN)
    sm_config_set_out_pins(&c, GpuConfig::HUB75_R1_PIN, 6);

    // Side-set pin: CLK (1 pin)
    sm_config_set_sideset_pins(&c, GpuConfig::HUB75_CLK_PIN);

    // OUT shift: shift right, autopull at 32 bits (one pixel per word)
    sm_config_set_out_shift(&c, true, true, 32);

    // FIFO: join to TX for deeper buffer (8 entries instead of 4)
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // Clock divider: 150 MHz / 2 = 75 MHz pixel clock
    // (each pixel = pull + out + nop = 3 PIO cycles at 75 MHz ≈ 25 Mpixel/s)
    // Divide by 2 for safety at startup; can reduce for faster refresh.
    float clkdiv = 2.0f;
    sm_config_set_clkdiv(&c, clkdiv);

    // Configure pin directions for PIO control
    pio_sm_set_consecutive_pindirs(pio_hw_inst, sm_data,
                                   GpuConfig::HUB75_R1_PIN, 6, true);  // data out
    pio_sm_set_consecutive_pindirs(pio_hw_inst, sm_data,
                                   GpuConfig::HUB75_CLK_PIN, 1, true); // CLK out

    // Assign PIO function to GPIO pins
    for (int i = 0; i < 6; i++) {
        pio_gpio_init(pio_hw_inst, GpuConfig::HUB75_R1_PIN + i);
    }
    pio_gpio_init(pio_hw_inst, GpuConfig::HUB75_CLK_PIN);

    // Initialize but don't start yet
    pio_sm_init(pio_hw_inst, sm_data, data_prog_offset +
                hub75_data_offset_entry_point, &c);
}

static void InitRowSM() {
    // Load hub75_row PIO program
    row_prog_offset = pio_add_program(pio_hw_inst, &hub75_row_program);
    sm_row = pio_claim_unused_sm(pio_hw_inst, true);

    pio_sm_config c = hub75_row_program_get_default_config(row_prog_offset);

    // SET pins: LAT, OE (2 consecutive pins)
    sm_config_set_set_pins(&c, GpuConfig::HUB75_LAT_PIN, 2);

    // OUT pins: Address A-E (5 consecutive pins)
    sm_config_set_out_pins(&c, GpuConfig::HUB75_ADDR_A, 5);

    // OUT shift: shift right, no autopull (manual PULL in PIO program)
    sm_config_set_out_shift(&c, true, false, 32);

    // Same clock as data SM
    float clkdiv = 2.0f;
    sm_config_set_clkdiv(&c, clkdiv);

    // Configure pin directions
    pio_sm_set_consecutive_pindirs(pio_hw_inst, sm_row,
                                   GpuConfig::HUB75_LAT_PIN, 2, true);  // LAT, OE
    pio_sm_set_consecutive_pindirs(pio_hw_inst, sm_row,
                                   GpuConfig::HUB75_ADDR_A, 5, true);   // ADDR A-E

    // Assign PIO function to GPIO pins
    pio_gpio_init(pio_hw_inst, GpuConfig::HUB75_LAT_PIN);
    pio_gpio_init(pio_hw_inst, GpuConfig::HUB75_OE_PIN);
    for (int i = 0; i < 5; i++) {
        pio_gpio_init(pio_hw_inst, GpuConfig::HUB75_ADDR_A + i);
    }

    // Initialize but don't start yet
    pio_sm_init(pio_hw_inst, sm_row, row_prog_offset +
                hub75_row_offset_entry_point, &c);
}

// ─── DMA Setup ──────────────────────────────────────────────────────────────

static void InitDMA() {
    dma_data_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(dma_data_chan);

    // 32-bit transfers
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

    // DREQ: pace to PIO TX FIFO of SM0
    channel_config_set_dreq(&c, pio_get_dreq(pio_hw_inst, sm_data, true));

    // Read from RAM (increment), write to PIO TX FIFO (no increment)
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    // Configure channel (but don't start — we trigger per row)
    dma_channel_configure(
        dma_data_chan,
        &c,
        &pio_hw_inst->txf[sm_data],   // write to PIO TX FIFO
        bcmRowBuffer,                    // read from BCM row buffer
        PANEL_W,                         // transfer PANEL_W words per row
        false                            // don't start yet
    );
}

// ─── Refresh Engine ─────────────────────────────────────────────────────────

/**
 * @brief Drive one BCM plane for one row pair.
 *
 * Called from the polling loop or IRQ. Sequence:
 *   1. Extract BCM bit-plane from framebuffer into bcmRowBuffer
 *   2. Trigger DMA → SM0 (shifts out pixel data)
 *   3. Feed SM1 (row address + OE delay)
 *   4. Wait for DMA completion
 *   5. Wait for SM1 OE cycle to complete (IRQ 0)
 */
static void DriveOnePlane(uint8_t row, uint8_t bit) {
    const uint16_t* fb = active_framebuffer;
    if (!fb) return;

    // 1. Extract bit-plane
    ExtractBcmPlane(fb, row, bit, bcmRowBuffer);

    // 2. Start DMA transfer to SM0
    dma_channel_set_read_addr(dma_data_chan, bcmRowBuffer, false);
    dma_channel_set_trans_count(dma_data_chan, PANEL_W, true);  // start

    // 3. Feed SM1: row address
    pio_sm_put_blocking(pio_hw_inst, sm_row, row);

    // 4. Feed SM1: OE delay count (BCM weighting)
    // Higher bits get exponentially longer OE time.
    // Bit 0 → 1 cycle, bit 1 → 2, bit 2 → 4, ..., bit 7 → 128 cycles.
    // Scale up for visible brightness. Base unit = 4 PIO cycles.
    uint32_t oe_delay = (1u << bit) * 4;
    if (oe_delay > 0) oe_delay--;  // loop is X+1 iterations
    pio_sm_put_blocking(pio_hw_inst, sm_row, oe_delay);

    // 5. Wait for DMA to finish feeding SM0
    dma_channel_wait_for_finish_blocking(dma_data_chan);

    // 6. Wait for SM1 to finish OE cycle (IRQ 0)
    // Clear IRQ 0 flag first, then wait
    pio_interrupt_clear(pio_hw_inst, 0);
    // The IRQ may already be set if the OE loop was short.
    // Spin-wait for IRQ 0 to be set by hub75_row SM.
    while (!(pio_hw_inst->irq & (1u << 0))) {
        tight_loop_contents();
    }
    pio_interrupt_clear(pio_hw_inst, 0);
}

/**
 * @brief Run one complete refresh of the entire panel.
 *
 * Iterates all SCAN_ROWS row pairs × COLOR_DEPTH BCM bits.
 * Total operations: 32 × 8 = 256 plane transfers per refresh.
 *
 * This should be called in a tight loop or from a timer interrupt.
 * At 150 MHz with clkdiv=2: ~75 MHz PIO clock.
 * Per row: 128 pixels × 3 clocks = 384 PIO cycles = ~5.1 µs shift time.
 * Plus OE time: average across 8 BCM bits ≈ 128 cycles × 4 = 512 cycles ≈ 6.8 µs
 * Per scan row (8 planes): ~95 µs
 * Full refresh (32 rows): ~3.0 ms → ~333 Hz refresh rate.
 */
static void RefreshEntirePanel() {
    for (uint8_t row = 0; row < SCAN_ROWS; row++) {
        for (uint8_t bit = 0; bit < COLOR_DEPTH; bit++) {
            DriveOnePlane(row, bit);
        }
    }

    refreshCount++;

    // Measure refresh rate every 32 refreshes
    if ((refreshCount & 0x1F) == 0) {
        uint32_t now = time_us_32();
        if (lastRefreshUs != 0) {
            uint32_t elapsed = now - lastRefreshUs;
            // 32 refreshes in 'elapsed' microseconds
            measuredRefreshHz = 32000000u / elapsed;
        }
        lastRefreshUs = now;
    }
}

// ─── Public API ─────────────────────────────────────────────────────────────

bool Hub75Driver::Initialize(const uint16_t* initialFramebuffer) {
    active_framebuffer = initialFramebuffer;

    // --- GPIO setup (direct control for pins not managed by PIO) ---
    // Address pins A-E are set up later by PIO
    // LAT, OE initially via GPIO to hold display off
    gpio_init(GpuConfig::HUB75_OE_PIN);
    gpio_set_dir(GpuConfig::HUB75_OE_PIN, GPIO_OUT);
    gpio_put(GpuConfig::HUB75_OE_PIN, 1);  // OE active low → start disabled

    gpio_init(GpuConfig::HUB75_LAT_PIN);
    gpio_set_dir(GpuConfig::HUB75_LAT_PIN, GPIO_OUT);
    gpio_put(GpuConfig::HUB75_LAT_PIN, 0);

    // --- PIO setup ---
    InitDataSM();
    InitRowSM();

    // --- DMA setup ---
    InitDMA();

    // -- Start both SMs ---
    pio_sm_set_enabled(pio_hw_inst, sm_data, true);
    pio_sm_set_enabled(pio_hw_inst, sm_row, true);

    printf("[HUB75] Initialized: %ux%u, 1/%u scan, %u-bit BCM, PIO0 SM%u+SM%u\n",
           PANEL_W, PANEL_H, SCAN_ROWS, COLOR_DEPTH, sm_data, sm_row);

    return true;
}

void Hub75Driver::SetFramebuffer(const uint16_t* framebuffer) {
    // Atomic pointer update — picked up at next row-0 / bit-0 boundary
    active_framebuffer = framebuffer;
}

void Hub75Driver::SetBrightness(uint8_t b) {
    currentBrightness = b;
    // Brightness is applied per-pixel during ExtractBcmPlane().
    // No PIO reconfiguration needed.
}

uint32_t Hub75Driver::GetRefreshRate() {
    return measuredRefreshHz;
}

void Hub75Driver::Shutdown() {
    // Stop state machines
    pio_sm_set_enabled(pio_hw_inst, sm_data, false);
    pio_sm_set_enabled(pio_hw_inst, sm_row, false);

    // Stop DMA
    if (dma_data_chan >= 0) {
        dma_channel_abort(dma_data_chan);
        dma_channel_unclaim(dma_data_chan);
        dma_data_chan = -1;
    }

    // Unclaim SMs
    pio_sm_unclaim(pio_hw_inst, sm_data);
    pio_sm_unclaim(pio_hw_inst, sm_row);

    // Remove PIO programs
    pio_remove_program(pio_hw_inst, &hub75_data_program, data_prog_offset);
    pio_remove_program(pio_hw_inst, &hub75_row_program, row_prog_offset);

    // Blank display
    gpio_init(GpuConfig::HUB75_OE_PIN);
    gpio_set_dir(GpuConfig::HUB75_OE_PIN, GPIO_OUT);
    gpio_put(GpuConfig::HUB75_OE_PIN, 1);  // OE high = display off

    printf("[HUB75] Shutdown complete\n");
}

// ─── Refresh Poll (called from Core 0 or Core 1 between renders) ───────────

/**
 * @brief Non-blocking refresh. Call this frequently from the main loop.
 *
 * Each call drives one BCM plane for one row pair, then returns.
 * After SCAN_ROWS × COLOR_DEPTH calls, one full refresh is complete.
 * This approach avoids blocking the main loop for the full refresh.
 */
void Hub75Driver::PollRefresh() {
    DriveOnePlane(currentRow, currentBit);

    currentBit++;
    if (currentBit >= COLOR_DEPTH) {
        currentBit = 0;
        currentRow++;
        if (currentRow >= SCAN_ROWS) {
            currentRow = 0;
            refreshCount++;

            // Measure refresh rate
            if ((refreshCount & 0x1F) == 0) {
                uint32_t now = time_us_32();
                if (lastRefreshUs != 0) {
                    uint32_t elapsed = now - lastRefreshUs;
                    measuredRefreshHz = 32000000u / elapsed;
                }
                lastRefreshUs = now;
            }
        }
    }
}

// ─── Test Patterns ──────────────────────────────────────────────────────────

void Hub75Driver::FillTestPattern(uint16_t* fb, uint8_t pattern) {
    switch (pattern) {
        case 0: {
            // Solid red
            for (uint32_t i = 0; i < PANEL_W * PANEL_H; i++) {
                fb[i] = 0xF800;  // RGB565 red
            }
            break;
        }
        case 1: {
            // RGB gradient: R varies with X, G with Y, B with X+Y
            for (uint16_t y = 0; y < PANEL_H; y++) {
                for (uint16_t x = 0; x < PANEL_W; x++) {
                    uint8_t r = (x * 255) / PANEL_W;
                    uint8_t g = (y * 255) / PANEL_H;
                    uint8_t b = ((x + y) * 255) / (PANEL_W + PANEL_H);
                    fb[y * PANEL_W + x] = ((r >> 3) << 11)
                                        | ((g >> 2) << 5)
                                        | (b >> 3);
                }
            }
            break;
        }
        case 2: {
            // Checkerboard (8x8 tiles)
            for (uint16_t y = 0; y < PANEL_H; y++) {
                for (uint16_t x = 0; x < PANEL_W; x++) {
                    bool black = ((x / 8) + (y / 8)) & 1;
                    fb[y * PANEL_W + x] = black ? 0x0000 : 0xFFFF;
                }
            }
            break;
        }
        case 3: {
            // Color bars: 8 vertical bars (R, G, B, Y, C, M, W, BLK)
            uint16_t colors[8] = {
                0xF800, 0x07E0, 0x001F, 0xFFE0,
                0x07FF, 0xF81F, 0xFFFF, 0x0000
            };
            uint16_t barWidth = PANEL_W / 8;
            for (uint16_t y = 0; y < PANEL_H; y++) {
                for (uint16_t x = 0; x < PANEL_W; x++) {
                    uint8_t bar = x / barWidth;
                    if (bar > 7) bar = 7;
                    fb[y * PANEL_W + x] = colors[bar];
                }
            }
            break;
        }
        default:
            // Black
            memset(fb, 0, PANEL_W * PANEL_H * 2);
            break;
    }
}
