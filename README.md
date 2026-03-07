# ProtoGL GPU Firmware — RP2350

Phase 1 GPU implementation targeting the **RP2350** (dual-core ARM Cortex-M33
@ 150 MHz, 520 KB tightly-coupled SRAM).

## Features

- PIO-driven HUB75 display output (zero CPU overhead)
- PIO-driven bidirectional Octal SPI (8-bit parallel, up to 80 MHz, half-duplex) — data plane
- I2C slave (hardware I2C0 at address 0x3C) — management / control plane (device ID, status, config)
- Dual-core symmetric screen-space rasterizer
- Full ProtoGL command parser with CRC-16 validation
- Double-buffered framebuffer with atomic swap

## Hardware Requirements

- RP2350-based board (Raspberry Pi Pico 2 or custom)
- HUB75 LED matrix panel (128×64 or 64×64, 1/32 scan)
- ESP32-S3 host connected via:
  - Octal SPI: 8 data pins + CLK + CS + DIR (bidirectional data plane)
  - I2C: SDA + SCL (management / control plane)
  - Async notification: IRQ GPIO (active-low)

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
| DIR | GPIO 10 | Output |
| IRQ | GPIO 13 | Output |
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
│ PIO Octal SPI    │            │ (idle / waiting) │
│ (bidir, SM0+SM1) │            │                  │
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
| Z-Buffer | 16 KB | 128×64 uint16 (IEEE-754 upper bits, order-preserving) |
| Triangle2D pool | 50 KB | 512 projected triangles × 100 B |
| QuadTree nodes + bounds | 19 KB | 256 nodes × ~44 B + 512 entity bounds × 16 B |
| Transform scratch | 12 KB | 1024 transformed vertices × 12 B |
| Scene state pools | 138 KB | Vertex, index, UV, texture, layout coord pools |
| Slot arrays | 40 KB | Mesh, material, texture, layout, camera, draw slots |
| SPI ring buffer | 32 KB | DMA receive ring |
| Staging buffer | 16 KB | Bulk upload staging |
| Alloc table | 4 KB | Memory tier allocator |
| **Subtotal** | **~359 KB** | |
| Pico-SDK + stack | ~20 KB | Runtime, PIO, interrupts, Core 1 stack |
| **Free** | **~133 KB** | Headroom |
| SRAM cache arena | (64 KB) | QSPI VRAM cache (optional, reduces free to ~69 KB) |

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
│  ├── SRAM cache arena (64 KB)                (cache lines for QSPI VRAM)  │
│  └── Hot textures & material params          (promoted by score+weight)    │
│                                                                             │
│  Tier 1 ─ QSPI-A VRAM via PIO2 SM0+SM1 (up to 2 chips, RP2350B)          │
│  ├── Large textures, texture atlases                                        │
│  ├── Cold mesh geometry (inactive objects)                                  │
│  ├── Material parameter banks                                               │
│  └── DMA-prefetched into SRAM cache before rasterization                   │
│                                                                             │
│  Tier 2 ─ QSPI-B VRAM via PIO2 SM2+SM3 (up to 2 chips, RP2350B)          │
│  ├── Lookup tables (gamma, CIE, noise permutation)                         │
│  ├── Font atlases                                                           │
│  ├── Material parameter banks, small textures                              │
│  ┬── Animation keyframe data                                                │
│  └── DMA-prefetched into SRAM cache before rasterization                   │
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
| HIGH | LOW | **Tier 1 — QSPI-A + cache** | Critical but infrequent: DMA prefetch before use |
| LOW | HIGH | **Tier 2 — QSPI-B** | Frequent reads, DMA-prefetchable |
| LOW | LOW | **Tier 2 — QSPI-B** | Cold storage: LUTs, inactive data |

**Hard constraint:** Framebuffer, Z-buffer, and QuadTree are **pinned to SRAM** regardless
of score/weight. The tier system only governs resource data (textures, materials, meshes,
lookup tables).

### Resource Class Weights

| Resource Class | Base Weight | Typical Tier |
|---|---|---|
| `FRAMEBUFFER` | 255 (pinned) | SRAM only |
| `Z_BUFFER` | 255 (pinned) | SRAM only |
| `QUADTREE` | 255 (pinned) | SRAM only |
| `VERTEX_DATA` | 200 | SRAM (active), QSPI-A (cold) |
| `INDEX_DATA` | 200 | SRAM (active), QSPI-A (cold) |
| `MATERIAL_PARAM` | 160 | SRAM (hot), QSPI-A (cold) |
| `UV_DATA` | 140 | SRAM or QSPI-A |
| `TEXTURE` | 128 | SRAM (small/hot), QSPI-A (large) |
| `LAYOUT_COORDS` | 100 | SRAM or QSPI |
| `LOOKUP_TABLE` | 60 | QSPI-B (best for random reads) |
| `FONT_ATLAS` | 40 | QSPI-B (prefer MRAM) |
| `COLD_MESH` | 20 | QSPI-B or QSPI-A |

### Prefetch Pipeline

External QSPI VRAM data is DMA-prefetched into SRAM cache lines **between command parsing and
rasterization** (overlapped with QuadTree rebuild on Core 0):

```
Parse commands → Scan draw list → DMA prefetch VRAM → QuadTree rebuild → Rasterize
                                  ^^^^^^^^^^^^^^^^    ^^^^^^^^^^^^^^^^^
                                  (these overlap — DMA is zero-CPU)
```

### PIO Block Allocation

| PIO Block | Usage | State Machines | DMA Channels |
|---|---|---|---|
| PIO0 | HUB75 display driver | SM0 (data) + SM1 (row) | 2 |
| PIO1 | Octal SPI (bidir) | SM0 (RX) + SM1 (TX) | 2 |
| PIO2 | QSPI VRAM (dual channel) | SM0+SM1 (Ch-A) + SM2+SM3 (Ch-B) | 4 |
| **Total** | | **9 / 12 SM** | **8 / 12 DMA** |

### External Memory GPIO Assignment (provisional)

| Function | RP2350 GPIO | Direction | Notes |
|---|---|---|---|
| QSPI-A CS0 | GPIO 11 | Output | Active low |
| QSPI-A CLK | GPIO 12 | Output | Up to 150 MHz |
| QSPI-A DQ0–DQ3 | GPIO 34–37 | Bidir | QFN-80 package required |
| QSPI-A CS1 | GPIO 38 | Output | Second chip select |
| QSPI-B DQ0–DQ3 | GPIO 39–42 | Bidir | QFN-80 package required |
| QSPI-B CLK | GPIO 43 | Output | Up to 150 MHz |
| QSPI-B CS0 | GPIO 44 | Output | Active low |
| QSPI-B CS1 | GPIO 45 | Output | Second chip select |

> **Note:** All QSPI VRAM pins require the RP2350B QFN-80 package. The RP2350A (QFN-60)
> only exposes GPIO 0–29 — no external VRAM support (SRAM-only operation).

### Graceful Degradation

External memory is **optional**. The firmware detects which tiers are present at boot
and adjusts limits accordingly:

| Configuration | Total Resources | Notes |
|---|---|---|
| SRAM only (default) | 256 meshes, 64 textures | Current Phase 1 target |
| SRAM + QSPI-A (1–2 chips) | + large textures, cold meshes | RP2350B board with Channel A |
| SRAM + QSPI-A + QSPI-B | Full expanded mode (up to 4 chips) | RP2350B board with both channels |

### Host Memory Access

The host can directly read/write GPU device memory across all tiers via **7 SPI commands**
(0x30–0x3F) and **4 I2C registers** (0x0C–0x0F). This enables:

- Bulk texture/data upload to QSPI VRAM (bypassing the resource command path)
- Framebuffer capture for screenshots (via I2C readback, ~0.33 s for 16 KB at 400 kHz)
- Runtime memory pressure monitoring (per-tier capacity/usage/cache hit rate)
- Explicit resource tier placement and pinning
- GPU-internal memory copying between tiers

The `command_parser.cpp` includes stub handlers for all 7 memory commands. Full
implementations depend on M8 QSPI VRAM drivers and `MemTierManager`. The `scene_state.h`
has been extended with a 4 KB staging buffer, `lastAllocResult`, and `memTierInfo` fields.

See `GPU_API_Design.md` §9 for the complete API design and `ProtoGL_API_Spec.md` §4.4–4.5
for wire-format tables.
