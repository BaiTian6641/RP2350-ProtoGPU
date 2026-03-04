/**
 * @file hub75_driver.cpp
 * @brief PIO-driven HUB75 display driver implementation for RP2350.
 *
 * TODO(M1/Week 3): Full implementation. This is the skeleton with the
 * correct structure and Pico SDK API calls mapped out.
 */

#include "hub75_driver.h"
#include "../gpu_config.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// Generated PIO header (from hub75.pio via pico_generate_pio_header)
#include "hub75.pio.h"

// ─── State ──────────────────────────────────────────────────────────────────

static PIO         pio_instance = pio0;
static uint        sm = 0;               // PIO state machine index
static int         dma_channel = -1;
static int         dma_channel_ctrl = -1; // Control channel for chain restart
static const uint16_t* active_framebuffer = nullptr;
static uint8_t     brightness = 255;

/// BCM plane buffers: color depth × scan rows × row width
/// Each BCM plane holds 1-bit-per-channel data for one bit weight.
/// For 8-bit depth: 8 planes. Each plane row = PANEL_WIDTH * 6 bits (R1G1B1R2G2B2).
///
/// TODO(M1/Week 3): Implement BCM plane extraction from RGB565 framebuffer.
///                  This runs in the DMA IRQ or a background scanline loop.

// ─── Initialization ─────────────────────────────────────────────────────────

bool Hub75Driver::Initialize(const uint16_t* initialFramebuffer) {
    active_framebuffer = initialFramebuffer;

    // --- GPIO setup ---
    // Data pins: R1, G1, B1, R2, G2, B2 (consecutive for PIO side-set or OUT)
    const uint8_t dataPins[] = {
        GpuConfig::HUB75_R1_PIN, GpuConfig::HUB75_G1_PIN, GpuConfig::HUB75_B1_PIN,
        GpuConfig::HUB75_R2_PIN, GpuConfig::HUB75_G2_PIN, GpuConfig::HUB75_B2_PIN,
    };
    for (uint8_t pin : dataPins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
    }

    // Address pins (A-E)
    const uint8_t addrPins[] = {
        GpuConfig::HUB75_ADDR_A, GpuConfig::HUB75_ADDR_B,
        GpuConfig::HUB75_ADDR_C, GpuConfig::HUB75_ADDR_D,
        GpuConfig::HUB75_ADDR_E,
    };
    for (uint8_t pin : addrPins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
    }

    // Control pins: CLK, LAT, OE
    gpio_init(GpuConfig::HUB75_CLK_PIN);
    gpio_set_dir(GpuConfig::HUB75_CLK_PIN, GPIO_OUT);
    gpio_init(GpuConfig::HUB75_LAT_PIN);
    gpio_set_dir(GpuConfig::HUB75_LAT_PIN, GPIO_OUT);
    gpio_init(GpuConfig::HUB75_OE_PIN);
    gpio_set_dir(GpuConfig::HUB75_OE_PIN, GPIO_OUT);
    gpio_put(GpuConfig::HUB75_OE_PIN, 1);  // OE active low → start disabled

    // --- PIO setup ---
    // TODO(M1/Week 3): Load the PIO program from hub75.pio
    // uint offset = pio_add_program(pio_instance, &hub75_program);
    // sm = pio_claim_unused_sm(pio_instance, true);
    // hub75_program_init(pio_instance, sm, offset, GpuConfig::HUB75_R1_PIN);

    // --- DMA setup ---
    // TODO(M1/Week 3): Configure DMA channel to feed PIO TX FIFO
    // dma_channel = dma_claim_unused_channel(true);
    // dma_channel_config c = dma_channel_get_default_config(dma_channel);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    // channel_config_set_dreq(&c, pio_get_dreq(pio_instance, sm, true));
    // channel_config_set_read_increment(&c, true);
    // channel_config_set_write_increment(&c, false);
    //
    // Configure chain restart for continuous refresh:
    // dma_channel_ctrl = dma_claim_unused_channel(true);
    // ... chain ctrl channel to restart data channel after each frame scan

    return true;
}

// ─── Framebuffer Swap ───────────────────────────────────────────────────────

void Hub75Driver::SetFramebuffer(const uint16_t* framebuffer) {
    // Atomic pointer update — the DMA IRQ handler picks this up
    // at the next scanline-0 boundary.
    active_framebuffer = framebuffer;
}

// ─── Brightness ─────────────────────────────────────────────────────────────

void Hub75Driver::SetBrightness(uint8_t b) {
    brightness = b;
    // TODO(M1/Week 3): Adjust OE pulse width in the BCM timing table
    //                  based on brightness value. 0 = always OE high (off),
    //                  255 = maximum OE low time per BCM plane.
}

// ─── Refresh Rate ───────────────────────────────────────────────────────────

uint32_t Hub75Driver::GetRefreshRate() {
    // TODO(M1/Week 3): Measure from DMA completion IRQ timestamps
    return 0;
}

// ─── Shutdown ───────────────────────────────────────────────────────────────

void Hub75Driver::Shutdown() {
    // TODO(M1): Stop DMA, unclaim PIO SM, disable OE
    gpio_put(GpuConfig::HUB75_OE_PIN, 1);  // OE high = display off
}
