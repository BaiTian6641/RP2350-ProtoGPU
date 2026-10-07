# ProtoGPU — RP2350 firmware and ProtoGL host API

RP2350 graphics-coprocessor firmware for an ESP32-S3 host. The host retains application, animation and networking work; ProtoGPU performs object/camera transforms, clipping, material evaluation, 3D rasterization, postprocessing, 2D layers and physical output. No required nonvolatile graphics RAM, external PSRAM, overclock or RP-side FreeRTOS.

## Implementation and tracking

- [Architecture and agent handoff plan](docs/TinyGPU_Implementation_and_Agent_Handoff_Plan.md)
- [Fine-grained implementation tracker](docs/Implementation_Tracker.md)
- [ProtoGL API, runtime protocol and Arduino examples](ProtoGL/README.md)
- [Protocol-9 changelog and qualification limits](CHANGELOG.md)

The protocol-9 software cutover is implemented. Native scenarios execute the actual firmware parser, frame renderer and two-worker scheduler. Arm builds generate both `FLASH_LOCAL` and `RAM_HOST` images, manifests and enforced SRAM ledgers. These are **software/build results, not physical qualification**: no RP/S3 board or display is connected in this environment.

Future application integration target: [`BaiTian6641/ProtoTracer-ESP32S3-Port`](https://github.com/BaiTian6641/ProtoTracer-ESP32S3-Port), SSH URL `git@github.com:BaiTian6641/ProtoTracer-ESP32S3-Port.git`. ProtoTracer adaptation is explicitly deferred; it is not cloned or modified here. Firmware/API solidity is the current scope.

## Source dependencies

`ProtoGL/`, `ProtoGC/` and `third_party/pico-sdk/` are in-tree submodules; dependency/tool versions are recorded in [dependencies.lock.json](dependencies.lock.json). Shared firmware includes use `ProtoGL/src` and `ProtoGC/src`, not sibling checkouts.

```bash
git submodule update --init ProtoGL ProtoGC third_party/pico-sdk
```

Co-development edits are separate repository changes inside the ProtoGL and ProtoGC submodules. Commit/push those changes first, then record the parent firmware and submodule pointers as a compatible pair. Base revision pins do not include uncommitted API changes; the release bundler hashes and includes the actual paired host sources. Do not initialize ProtoGL's historical nested firmware entry to work on this runtime.

For Arduino IDE, install the `ProtoGL/` folder as the ProtoGL library; it now includes `library.properties`, `src/` and the two sketches. The exercised host compilation uses Arduino-ESP32 3.3.6/IDF5.5.2 for ESP32-S3. Example wiring/output assumptions are documented alongside each sketch.

## Build profiles

Requirements: CMake 3.20+, Pico SDK 2.3.1 and Arm embedded GCC 14.2.1. Set `PICO_TOOLCHAIN_PATH` if the compiler is not on PATH.

```bash
cmake -S . -B build/flash -DPICO_BOARD=pico2 \
  -DBOOT_STORAGE=FLASH_LOCAL -DDEFAULT_DISPLAY=SPI -DPSRAM=OFF
cmake --build build/flash --parallel

cmake -S . -B build/ram -DPICO_BOARD=pico2 \
  -DBOOT_STORAGE=RAM_HOST -DDEFAULT_DISPLAY=SPI -DPSRAM=OFF
cmake --build build/ram --parallel
```

| Selector | Implemented choices |
|---|---|
| `BOOT_STORAGE` | `FLASH_LOCAL`: SDK normal NOR boot; `RAM_HOST`: SDK `no_flash` SRAM image with RP2350 `IMAGE_DEF`, uploaded by the host ROM-UART loader |
| `DEFAULT_DISPLAY` | `SPI`, `HUB`, `LED`, `CUSTOM`, `NONE`; one active output, runtime reconfiguration at a drained boundary |
| `PSRAM` | `OFF` minimum; `ON` selected-CS QMI asset backing, explicit host boot-pad release before initialization |
| `HOSTLESS_DEMO` | `OFF` normal host-controlled runtime; `ON` bounded command-encoded rotating-cube diagnostic, using the same renderer/output path |

RAM code uses `-Os`, flash code `-O2`; LTO remains enabled except SDK wrapper/TLS translation units requiring ordinary symbols/literal-pool isolation. Every successful build emits ELF/map/bin, the appropriate SDK extra outputs, `pgl_image_manifest.json`, a 32-byte host manifest, `pgl_image_report.txt` and `pgl_budget.json`. Packaging fails if the linked profile leaves less than **32 KiB total unallocated SRAM**, including both 4 KiB stacks. This is static headroom, not measured stack high-water or contiguous scene-heap availability.

FLASH_LOCAL UF2 installs through BOOTSEL. RAM_HOST is not a generic function-pointer jump: use ProtoGL's echo-paced RP2350 ROM-UART image loader and validate runtime build/profile identity after boot. Pico2 includes NOR; direct QSPI boot-UART wiring requires the documented reset/strap/IO isolation. A successful compilation is not proof that a board can safely use those pads.

Final SPI-default static ledgers, bytes (both4KiB stacks included):

| Profile | Image bytes | SRAM used | Unallocated SRAM |
|---|---:|---:|---:|
| FLASH_LOCAL, PSRAM OFF | 158708 | 390196 | 142284 |
| RAM_HOST, PSRAM OFF | 112860 | 496292 | 36188 |
| FLASH_LOCAL, PSRAM ON | 163868 | 391348 | 141132 |
| RAM_HOST, PSRAM ON | 115340 | 498828 | 33652 |

The exact output workspace is now67,584 bytes. RAM_HOST HUB/LED/CUSTOM/NONE defaults link/package with36,172 bytes unallocated and the hostless FLASH_LOCAL diagnostic with142,268. All ledgers clear the32KiB floor; these margins are not measured runtime stack safety or power/current qualification.

## Bounded runtime contract

| Resource | Firmware limit |
|---|---:|
| Logical framebuffer / depth pixels | 8192 |
| Tile grid | 64 cells, 16×16 pixels each |
| Transformed vertices | 1024 |
| Source / projected triangles | 1280 / 1280, including clipping expansion |
| Mesh / material / texture slots | 64 / 64 / 16 |
| Cameras / 3D draws / queued 2D operations | 4 / 64 / 128 |
| Layers / shader programs | 8 / 4 |
| Scene allocation arena | 64 KiB |
| Ingress slots / maximum batch | 2 / 16 KiB |
| Weighted post-FX work per frame | 2 Mi operations |

Aggregate scene/resources/layer allocation and complete external-asset staging can fail before execution; maxima are not independent promises that every resource can be filled simultaneously. Projected-pool overflow is an explicit failed frame, never missing geometry presented as success. The original 587-vertex/1166-triangle teapot and full-frame three-program post-FX workload are retained.

- Bare-metal event/service loop; both cores claim disjoint tiles through one tagged scheduler. Clock transitions park the worker and drain host/output/device/memory users.
- Fresh nonzero sessions, generation-checked resources, contiguous streamed uploads and transactional preflight/commit. Rendering/conversion readers retain immutable inputs until release.
- One executing frame and at most one queued batch; no arbitrary dropping or coalescing of mixed resource mutations. Transfer sequences and nonzero frame IDs increase within a session; start a new session before wrap. Exact retries retain the same sequence and bytes.
- Separate accepted/rendered/transferred/displayed fences. SSD1331 has no TE: only `Transferred`, not fabricated `Displayed`. `NONE` is explicit render-only output.
- Protocol-9 control32/read64/bulk envelopes on real PIO mode-0 SPI. Single-lane command/control, negotiated one/four-lane bulk data; initial SCK1 MHz, minimum64 µs CS gap, active-high READY before every host transaction. Control ACK and terminal bulk ACK are distinct.
- Exact clock profiles 150/100/75/125/240/288/250/300/336 MHz. Profiles above150 MHz are explicit requests at a **1.2 V regulator setpoint** (never higher, never bypassing the voltage limit); 300 MHz records the user's board-tested reference and 336 MHz is the requested 48×7 profile. `clk_sys` varies through PLL_SYS; reference/timers stay at12 MHz and UART/SPI/USB/ADC/HSTX stay at48 MHz from PLL_USB. Optional active PSRAM remains ≤150 MHz until part-specific receive calibration is supplied. `QueryClock` and `QueryClockConfiguration` expose requested/actual state, required/actual setpoint and domain rates; qualified power/timing still requires hardware measurements.
- Optional typed cold mesh/texture backing uses complete pinned SRAM spans consumed by the real renderer. Code, depth, active output and CPU hot reads remain internal; no per-texel PSRAM transaction or fake pointer mapping.
- Verified shader instructions/constants stay resident and immutable; only pass uniforms are snapshotted. A rejected worker-band job fails the frame, never reports successful postprocessing.
- Future custom-GPGPU kernels are extension work, not an advertised feature: protocol 10 can reserve a small compute capability/work budget using immutable inputs, disjoint outputs, bounded scheduler jobs and explicit cancellation. They must share the already-drained 13 KiB preparation and 16 KiB world/depth lifetimes plus scene-heap headroom, not claim a second persistent arena or bypass display/DMA/clock maintenance.

## Pico2-safe reference wiring

This is an engineering reference, **not an approved product PCB**. GPIO numbers below are RP pins; configure corresponding host pins explicitly in ProtoGL. Examples map S3 GPIO14 to RP GP22 READY. Ordinary GP1 is runtime MISO/D1, not the dedicated QSPI SD1 ROM-UART strap pad.

| Function | RP GPIO |
|---|---|
| Runtime host D0..D3, SCK, CS | 0..3, 4, 5 |
| READY / IRQ (active-low record notification) | 22 / 27 |
| Debug UART0 TX, no RX | 28 |
| SSD1331 PIO SCK/MOSI/CS/RST/DC | 6 / 7 / 9 / 10 / 11, ≤4 MHz |
| HUB75 R1/G1/B1/R2/G2/B2, CLK/LAT/OE, A..E | 6..11, 12/13/14, 15..19 |
| WS2812B-V5/W GRB data | 6, calculated6.4 MHz PIO timing, ≥300 µs reset |
| Custom RGB888 parallel data, CLK/LATCH/OE | 6..13, 14/15/16 |
| Attached I2C0 SDA/SCL, fixed address | 20/21, `0x3D`, ≤1 ms request slice |
| Attached explicit-drive GPIO | 26 |
| Optional QMI CS1 | 8; conflicts with HUB and CUSTOM reference routing |

Pico2 GP23/24/25/29 are board-connected and excluded from external claims. Hardware leases arbitrate GPIO, PIO programs/SMs/flags, DMA, bus roles and QMI windows. HUB75/custom/PSRAM conflicts reject explicitly. Actual panel controller, logic levels, buffering, reset pulls, power gating and timing captures remain hardware prerequisites.

## Verification entrypoints

```bash
bash tests/native/run_kernel_tests.sh
bash tests/native/run_pipeline_tests.sh
bash sim/run_2d_primitives_check.sh
bash sim/run_f04_check.sh
bash tests/scheduler/run_scheduler_check.sh
bash tests/transport/run_transport_check.sh
bash tests/memory/run_tests.sh
bash tests/shader_vm/run_shader_checks.sh
bash tests/display/run_tests.sh
bash tests/clock/run_tests.sh
bash tests/devices/run_tests.sh
bash ProtoGL/tests/syntax_check/run_link.sh
bash ProtoGL/tests/syntax_check/run_boot.sh
bash sim/build_sim.sh
```

`sim/run_golden.sh` preserves historical references and diagnoses intentional raster changes; it never automatically regenerates them. Consumer geometry/depth/alpha/rollback/lifetime oracles are the correctness gates for changed semantics. Native timing is desktop timing, not RP frame-rate/power or S3 CPU-offload evidence. Actual Arduino-ESP32 3.3.6/IDF5.5.2 example objects are cross-compiled separately; this does not establish flashing or hardware operation.

After building both final profiles:

```bash
PGL_RAM_BUILD=build/ram PGL_FLASH_BUILD=build/flash \
  python3 -m unittest discover -s tests/packaging -v
python3 tools/pgl_pair_bundle.py --repo . \
  --firmware-manifest build/ram/pgl_image_manifest.json build/flash/pgl_image_manifest.json \
  --firmware-artifacts \
    build/ram/protogl_gpu.bin build/ram/protogl_gpu.elf build/ram/protogl_gpu.elf.map \
    build/flash/protogl_gpu.bin build/flash/protogl_gpu.elf build/flash/protogl_gpu.elf.map \
  --manifest-bin build/ram/pgl_image_manifest.bin build/flash/pgl_image_manifest.bin \
  --out-dir build/release-pair
python3 tools/pgl_asset_embed.py image --bin build/ram/protogl_gpu.bin \
  --manifest build/ram/pgl_image_manifest.json --manifest-bin build/ram/pgl_image_manifest.bin \
  --name ProtoGPUImage --out-dir build/embedded-image
```

Release bundles include exact images, ELF/map/source hashes, the dependency lock, an Arduino-layout host library, exact firmware/allocator/tool inputs and native replay/checks with profile reproduction commands. The SDK/toolchain are pinned acquisition prerequisites, not duplicated in the bundle. Integrity is not authentication. Physical boot/output/transport/PSRAM/current/stack/sustained-load gates remain open in the tracker.

The exercised working-tree bundle is `build/release-protocol9-final/`: `firmware/RAM_HOST`, `firmware/FLASH_LOCAL`, Arduino library `host/`, and exact replay/build snapshot `source/`. Both generated reproduction scripts were run against that snapshot with the pinned SDK/GCC supplied; source/build identities and SRAM ledgers match. Rebuilt image/ELF/map digests may differ with compilation paths/link layout; the manifests identify each exact artifact, not a bit-for-bit rebuild claim. Native full-frame post-FX replay also ran from the bundled sources.
