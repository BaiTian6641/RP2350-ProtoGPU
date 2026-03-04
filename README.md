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
│ Tile Raster      │──FIFO──→  │ Tile Raster      │
│ (work-stealing)  │            │ (work-stealing)  │
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
| Triangle2D pool | 80 KB | 1024 projected triangles × ~80 B |
| QuadTree nodes | 22 KB | 512 nodes × ~44 B |
| Transform scratch | 24 KB | 2048 transformed vertices × 12 B |
| Scene state pools | 138 KB | Vertex, index, UV, texture, layout coord pools |
| Slot arrays | 40 KB | Mesh, material, texture, layout, camera, draw slots |
| SPI ring buffer | 32 KB | DMA receive ring |
| **Subtotal** | **~400 KB** | |
| Pico-SDK + stack | ~60 KB | Runtime, PIO, interrupts |
| **Free** | **~60 KB** | Headroom |
| SRAM cache arena | (64 KB) | Tier 1 OPI PSRAM cache (optional, reduces free to 0) |

---

## Tiered Memory Architecture

The GPU supports up to three memory tiers for resource storage. **Rasterization-critical
data (framebuffer, Z-buffer, QuadTree) is ALWAYS in internal SRAM.** External memory is
only for expanding texture, material, and mesh capacity without degrading render speed.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          MEMORY TIERS                                       │
│                                                                             │
│  Tier 0 ─ Internal SRAM (520 KB, 1-cycle)                                  │
│  ├── Framebuffers, Z-buffer, QuadTree       (ALWAYS here, pinned)          │
│  ├── Active vertices/indices                 (hot mesh data)               │
│  ├── SRAM cache arena (64 KB)                (cache lines for OPI data)    │
│  └── Hot textures & material params          (promoted by score+weight)    │
│                                                                             │
│  Tier 1 ─ OPI PSRAM via PIO2 (8–16 MB, ~150 MB/s, indirect)               │
│  ├── Large textures, texture atlases                                        │
│  ├── Cold mesh geometry (inactive objects)                                  │
│  ├── Material parameter banks                                               │
│  └── DMA-prefetched into SRAM cache before rasterization                   │
│                                                                             │
│  Tier 2 ─ QSPI MRAM via QMI CS1 (MR10Q010, 128 KB, ~52 MB/s, XIP)      │
│  ├── Lookup tables (gamma, CIE, noise permutation)                         │
│  ├── Font atlases                                                           │
│  ├── Material parameter banks, small textures                              │
│  ┬── Animation keyframe data                                                │
│  └── Read via normal pointers — QMI HW cache (4 KB) handles caching       │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Placement Policy (Score + Weight)

Each resource tracks two metrics:

| Metric | Meaning | Updated |
|---|---|---|
| **weight** | Rendering pipeline impact (0–255). Higher = more critical to visual quality. | Set at resource creation (based on type) |
| **score** | Access frequency (0–255). Higher = more accesses per frame. | Updated at runtime per frame |

Combined priority: `priority = α × weight + β × score` (default α=3, β=1)

| Weight | Score | Tier Assigned | Rationale |
|---|---|---|---|
| HIGH | HIGH | **Tier 0 — SRAM** | Hot critical path: active textures, vertices |
| HIGH | LOW | **Tier 1 — OPI + cache** | Critical but infrequent: DMA prefetch before use |
| LOW | HIGH | **Tier 2 — QSPI XIP** | Frequent reads, non-critical: HW cache handles it |
| LOW | LOW | **Tier 2 — QSPI XIP** | Cold storage: LUTs, inactive data |

**Hard constraint:** Framebuffer, Z-buffer, and QuadTree are **pinned to SRAM** regardless
of score/weight. The tier system only governs resource data (textures, materials, meshes,
lookup tables).

### Resource Class Weights

| Resource Class | Base Weight | Typical Tier |
|---|---|---|
| `FRAMEBUFFER` | 255 (pinned) | SRAM only |
| `Z_BUFFER` | 255 (pinned) | SRAM only |
| `QUADTREE` | 255 (pinned) | SRAM only |
| `VERTEX_DATA` | 200 | SRAM (active), OPI (cold) |
| `INDEX_DATA` | 200 | SRAM (active), OPI (cold) |
| `MATERIAL_PARAM` | 160 | SRAM (hot), OPI (cold) |
| `UV_DATA` | 140 | SRAM or OPI |
| `TEXTURE` | 128 | SRAM (small/hot), OPI (large) |
| `LAYOUT_COORDS` | 100 | SRAM or QSPI |
| `LOOKUP_TABLE` | 60 | QSPI XIP (best for random reads) |
| `FONT_ATLAS` | 40 | QSPI XIP |
| `COLD_MESH` | 20 | QSPI XIP or OPI |

### Prefetch Pipeline

OPI PSRAM data is DMA-prefetched into SRAM cache lines **between command parsing and
rasterization** (overlapped with QuadTree rebuild on Core 0):

```
Parse commands → Scan draw list → DMA prefetch OPI → QuadTree rebuild → Rasterize
                                  ^^^^^^^^^^^^^^^^    ^^^^^^^^^^^^^^^^^
                                  (these overlap — DMA is zero-CPU)
```

### PIO Block Allocation

| PIO Block | Usage | State Machines | DMA Channels |
|---|---|---|---|
| PIO0 | HUB75 display driver | SM0 (data) + SM1 (row) | 2 |
| PIO1 | Octal SPI receiver | SM0 | 1 |
| PIO2 | OPI PSRAM interface | SM0 (cmd/write) + SM1 (read) | 2 |
| **Total** | | **6 / 12 SM** | **5 / 12 DMA** |

### External Memory GPIO Assignment (provisional)

| Function | RP2350 GPIO | Direction | Notes |
|---|---|---|---|
| OPI PSRAM CS# | GPIO 11 | Output | Active low |
| OPI PSRAM CLK | GPIO 12 | Output | Up to 150 MHz |
| OPI PSRAM DQ0–DQ7 | GPIO 34–41 | Bidir | QFN-80 package required |
| QSPI MRAM | QMI CS1 pins | — | MR10Q010 (128 KB, dual-supply 3.3V/1.8V) |

> **Note:** GPIO 34+ requires the RP2350 QFN-80 package. The QFN-60 (Pico 2 default)
> only exposes GPIO 0–29 + QSPI pins. For QFN-60, use QSPI MRAM only (no OPI PIO2).

### Graceful Degradation

External memory is **optional**. The firmware detects which tiers are present at boot
and adjusts limits accordingly:

| Configuration | Total Resources | Notes |
|---|---|---|
| SRAM only (default) | 256 meshes, 64 textures | Current Phase 1 target |
| SRAM + QSPI MRAM 128 KB | + LUTs, fonts, material params, small textures | Custom board with MR10Q010 on CS1 |
| SRAM + OPI 8 MB | + large textures, cold meshes | Custom board with PIO2 PSRAM |
| SRAM + QSPI MRAM + OPI | Full expanded mode | Maximum capacity custom board |

### Host Memory Access

The host can directly read/write GPU device memory across all tiers via **7 SPI commands**
(0x30–0x3F) and **4 I2C registers** (0x0C–0x0F). This enables:

- Bulk texture/data upload to MRAM/OPI (bypassing the resource command path)
- Framebuffer capture for screenshots (via I2C readback, ~0.33 s for 16 KB at 400 kHz)
- Runtime memory pressure monitoring (per-tier capacity/usage/cache hit rate)
- Explicit resource tier placement and pinning
- GPU-internal memory copying between tiers

The `command_parser.cpp` includes stub handlers for all 7 memory commands. Full
implementations depend on M8 OPI/QSPI drivers and `MemTierManager`. The `scene_state.h`
has been extended with a 4 KB staging buffer, `lastAllocResult`, and `memTierInfo` fields.

See `GPU_API_Design.md` §9 for the complete API design and `ProtoGL_API_Spec.md` §4.4–4.5
for wire-format tables.
