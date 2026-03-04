# ProtoGL GPU Firmware — RP2350

Phase 1 GPU implementation targeting the **RP2350** (dual-core ARM Cortex-M33
@ 150 MHz, 520 KB tightly-coupled SRAM).

## Features

- PIO-driven HUB75 display output (zero CPU overhead)
- PIO-driven Octal SPI receiver (8-bit parallel, up to 80 MHz)
- I2C slave (hardware I2C0 at address 0x3C)
- Dual-core symmetric screen-space rasterizer
- Full ProtoGL command parser with CRC-16 validation
- Double-buffered framebuffer with atomic swap

## Hardware Requirements

- RP2350-based board (Raspberry Pi Pico 2 or custom)
- HUB75 LED matrix panel (128×64 or 64×64, 1/32 scan)
- ESP32-S3 host connected via:
  - Octal SPI: 8 data pins + CLK + CS
  - I2C: SDA + SCL
  - Flow control: RDY GPIO

## Pin Assignments

> **Note:** Pin numbers are configurable in `src/gpu_config.h`.
> Defaults below are provisional — adjust for your PCB layout.

| Function | RP2350 GPIO | Direction |
|---|---|---|
| SPI D0–D7 | GPIO 0–7 | Input |
| SPI CLK | GPIO 8 | Input |
| SPI CS | GPIO 9 | Input |
| I2C SDA | GPIO 14 | Bidir |
| I2C SCL | GPIO 15 | Input |
| RDY | GPIO 10 | Output |
| HUB75 R1 | GPIO 16 | Output |
| HUB75 G1 | GPIO 17 | Output |
| HUB75 B1 | GPIO 18 | Output |
| HUB75 R2 | GPIO 19 | Output |
| HUB75 G2 | GPIO 20 | Output |
| HUB75 B2 | GPIO 21 | Output |
| HUB75 ADDR A | GPIO 22 | Output |
| HUB75 ADDR B | GPIO 23 | Output |
| HUB75 ADDR C | GPIO 24 | Output |
| HUB75 ADDR D | GPIO 25 | Output |
| HUB75 ADDR E | GPIO 26 | Output |
| HUB75 CLK | GPIO 27 | Output |
| HUB75 LAT | GPIO 28 | Output |
| HUB75 OE | GPIO 29 | Output |

## Building

### Prerequisites

- [Pico SDK](https://github.com/raspberrypi/pico-sdk) (v2.0+ for RP2350 support)
- CMake 3.20+
- ARM GCC toolchain (`arm-none-eabi-gcc`)

### Build Steps

```bash
# Set PICO_SDK_PATH if not already in environment
export PICO_SDK_PATH=/path/to/pico-sdk

mkdir build && cd build
cmake .. -DPICO_BOARD=pico2
make -j$(nproc)
```

### Flashing

1. Hold BOOTSEL on the RP2350 board
2. Connect USB
3. Drag `protogl_gpu.uf2` to the RPI-RP2 drive

## Architecture

```
Core 0                          Core 1
┌──────────────────┐            ┌──────────────────┐
│ PIO Octal SPI RX │            │ (idle / waiting) │
│       ↓          │            │                  │
│ Ring Buffer      │            │                  │
│       ↓          │            │                  │
│ Command Parser   │            │                  │
│       ↓          │            │                  │
│ Scene State      │            │                  │
│       ↓          │            │                  │
│ Transform + Proj │            │                  │
│       ↓          │            │                  │
│ QuadTree Rebuild │            │                  │
│       ↓          │            │       ↓          │
│ Raster top half  │──FIFO──→  │ Raster bot half  │
│       ↓          │            │       ↓          │
│   ← barrier ←   │←──FIFO──── │   → barrier →    │
│       ↓          │            │                  │
│ Framebuf swap    │            │                  │
│       ↓          │            │                  │
│ PIO HUB75 DMA   │            │                  │
└──────────────────┘            └──────────────────┘
```

## Memory Budget (520 KB SRAM)

| Subsystem | Size | Notes |
|---|---|---|
| Framebuffer ×2 | 32 KB | 128×64 RGB565, double-buffered |
| Z-Buffer | 32 KB | 128×64 float32 |
| QuadTree/Nodes | 100 KB | Pre-allocated node pool |
| Mesh storage | 48 KB | 2048 vertices + 1024 triangles |
| Material table | 16 KB | 256 material slots |
| SPI ring buffer | 32 KB | DMA receive ring |
| Draw list | 4 KB | 64 draw calls × ~64 B each |
| Texture cache | 64 KB | Small textures for Image material |
| **Subtotal** | **328 KB** | |
| Pico-SDK + stack | ~60 KB | Runtime, PIO, interrupts |
| **Free** | **~132 KB** | 25% headroom |
