#pragma once
#include <cstdint>
#include <cstddef>

#ifndef PROTOGPU_RAM_HOST
#define PROTOGPU_RAM_HOST 0
#endif
#ifndef PROTOGPU_ENABLE_PSRAM
#define PROTOGPU_ENABLE_PSRAM 0
#endif
#ifndef PROTOGPU_DEFAULT_DISPLAY
#define PROTOGPU_DEFAULT_DISPLAY 3
#endif

namespace GpuConfig {
constexpr uint32_t SYSTEM_CLOCK_MHZ = 150;
constexpr uint16_t PANEL_WIDTH = 128, PANEL_HEIGHT = 64;
constexpr uint32_t FRAMEBUF_PIXELS = uint32_t(PANEL_WIDTH) * PANEL_HEIGHT;
constexpr uint32_t FRAMEBUF_SIZE = FRAMEBUF_PIXELS * sizeof(uint16_t);
constexpr uint16_t MAX_VERTICES = 1024;
constexpr uint16_t MAX_SOURCE_TRIANGLES = 1280;
constexpr uint16_t MAX_TRIANGLES = 1280;
constexpr uint16_t MAX_MESHES = 64, MAX_MATERIALS = 64;
constexpr uint8_t MAX_TEXTURES = 16, MAX_DRAW_CALLS = 64;
constexpr uint8_t MAX_LAYERS = 8, MAX_SHADER_PROGRAMS = 4;
constexpr uint16_t FRAME_VERTEX_POOL_SIZE = 1024;
constexpr uint32_t SCENE_HEAP_MAX_BYTES = 64 * 1024;
constexpr uint32_t SCENE_HEAP_SEGMENT_BYTES = SCENE_HEAP_MAX_BYTES;
constexpr uint32_t TEXTURE_POOL_SIZE = 32 * 1024;
constexpr uint16_t MAX_TEXTURE_DIMENSION = 512;
constexpr uint16_t LAYOUT_COORD_POOL_SIZE = 2048;
constexpr uint32_t POSTFX_WORK_BUDGET = 2 * 1024 * 1024;
constexpr uint8_t MAX_CONVOLUTION_RADIUS = 4;
constexpr uint32_t SRAM_RESERVE_BYTES = 32 * 1024;
constexpr size_t DISPLAY_WORKSPACE_BYTES = 67584; // exact two packed128×64 HUB scans+records
constexpr uint16_t MAX_BATCH_COMMANDS = 256;
constexpr uint8_t FRAME_HISTORY = 8;

// Pico2-safe pin map. GP23/24/25/29 are board-connected, not free headers.
constexpr uint64_t GPIO_AVAILABLE_MASK = ((uint64_t(1) << 30) - 1) &
    ~((uint64_t(1) << 23) | (uint64_t(1) << 24) | (uint64_t(1) << 25) | (uint64_t(1) << 29));
constexpr uint8_t HOST_DATA_BASE = 0, HOST_SCK = 4, HOST_CS = 5;
constexpr uint8_t HOST_READY = 22, HOST_IRQ = 27;
constexpr uint32_t HOST_INITIAL_SCK_HZ = 1000000;
constexpr uint8_t DEBUG_UART = 0, DEBUG_TX = 28;

constexpr uint8_t HUB75_R1_PIN = 6, HUB75_G1_PIN = 7, HUB75_B1_PIN = 8;
constexpr uint8_t HUB75_R2_PIN = 9, HUB75_G2_PIN = 10, HUB75_B2_PIN = 11;
constexpr uint8_t HUB75_CLK_PIN = 12, HUB75_LAT_PIN = 13, HUB75_OE_PIN = 14;
constexpr uint8_t HUB75_ADDR_A = 15, HUB75_ADDR_B = 16, HUB75_ADDR_C = 17;
constexpr uint8_t HUB75_ADDR_D = 18, HUB75_ADDR_E = 19;
constexpr uint8_t SCAN_ROWS = 32, COLOR_DEPTH = 8;
constexpr uint32_t HUB75_PIXEL_CLOCK_HZ = 12000000;
constexpr uint32_t HUB75_BASE_DWELL_NS = 160;

constexpr uint8_t SSD1331_SCK_PIN = 6, SSD1331_MOSI_PIN = 7;
constexpr uint8_t SSD1331_CS_PIN = 9, SSD1331_RST_PIN = 10, SSD1331_DC_PIN = 11;
constexpr uint16_t SSD1331_WIDTH = 96, SSD1331_HEIGHT = 64;
constexpr uint32_t SSD1331_SPI_BAUD = 4000000;
constexpr uint8_t LED_DATA_PIN = 6;
constexpr uint16_t LED_DEFAULT_WIDTH = 16, LED_DEFAULT_HEIGHT = 16;
constexpr uint32_t LED_RESET_US = 300;
constexpr uint32_t LED_PIO_CLOCK_HZ = 6400000;
constexpr uint8_t CUSTOM_DATA_BASE = 6, CUSTOM_CLK = 14, CUSTOM_LATCH = 15, CUSTOM_OE = 16;
constexpr uint32_t CUSTOM_PIXEL_CLOCK_HZ = 4000000;

constexpr uint8_t DEVICE_I2C_INSTANCE = 0, DEVICE_I2C_SDA = 20, DEVICE_I2C_SCL = 21;
constexpr uint8_t DEVICE_I2C_ADDRESS = 0x3d;
constexpr uint32_t DEVICE_I2C_BAUD = 400000;
constexpr uint32_t DEVICE_TIMEOUT_US = 1000;
constexpr uint8_t DEVICE_GPIO_PIN = 26;
constexpr uint8_t PSRAM_CS_PIN = 8;
constexpr uint32_t PSRAM_MAX_CLOCK_HZ = 32000000;
constexpr uint32_t PSRAM_MAX_SELECT_NS = 7000, PSRAM_MIN_DESELECT_NS = 50;
constexpr uint16_t MAX_EXTERNAL_ASSETS = 96;
constexpr uint32_t MAX_ACTIVE_ASSET_BYTES = 32 * 1024;

constexpr char BOARD_ID[] = "PGL-PICO2";
constexpr bool RAM_HOST = PROTOGPU_RAM_HOST != 0;
constexpr bool PSRAM_ENABLED = PROTOGPU_ENABLE_PSRAM != 0;
constexpr uint8_t DEFAULT_DISPLAY = PROTOGPU_DEFAULT_DISPLAY;

static_assert(MAX_MESHES <= 255 && MAX_MATERIALS <= 255, "slot255 is reserved");
static_assert(HOST_SCK == HOST_DATA_BASE + 4 && HOST_CS == HOST_DATA_BASE + 5, "PIO host pin grouping");
static_assert(HUB75_LAT_PIN + 1 == HUB75_OE_PIN, "HUB75 LAT/OE group");
static_assert(HUB75_ADDR_A + 4 == HUB75_ADDR_E, "HUB75 row-address group");
static_assert(SRAM_RESERVE_BYTES >= 32 * 1024, "explicit SRAM margin");
} // namespace GpuConfig
