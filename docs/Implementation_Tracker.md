# ProtoGPU Implementation Tracker and Agent Dispatch Contract

Version 1.2 · 7 October 2026 · Protocol-9 software implementation and qualification ledger

## 1. How to use this ledger

This is the implementation task authority. Architecture decisions live in the companion plan; do not duplicate or silently change them here. Package numbers identify ownership/gates, not a requirement to serialize independent work. Dependencies below are the scheduling authority.

Status vocabulary:

- DONE: the stated deliverable has evidence. Source inspection can finish an inventory task, never a hardware gate.
- READY: prerequisites satisfied; may be claimed by an implementation agent.
- WAITING: a listed dependency is incomplete; do not dispatch yet.
- ACTIVE: one named owner has claimed the task and its write scope.
- BLOCKED: a specific missing input/access prevents completion; record a blocker and continue independent work.
- OPTIONAL: outside minimum delivery unless the corresponding expansion is selected; still requires its own acceptance evidence before advertising it.
- SOFTWARE_DONE: real implementation plus exercised native/target-build evidence; not closure of the row's physical acceptance or its gate.
- DEFERRED: explicitly removed from this job by the user; original acceptance retained for later work.

Current state: protocol-9 firmware and ProtoGL software are implemented and exercised. Both Arm boot images, concrete PIO backends, selected-CS optional QMI backing, native rendering and real Arduino3.3.6 adapter objects are built. Physical gates remain OPEN. Each row retains its original acceptance; SOFTWARE_DONE reports only reachable source/native/build completion.

Latest user scope: [`git@github.com:BaiTian6641/ProtoTracer-ESP32S3-Port.git`](https://github.com/BaiTian6641/ProtoTracer-ESP32S3-Port) is the future application integration target. ProtoTracer adaptation and application-coupled P00-06/P12-01..05 are DEFERRED; no clone or edits were made there. Source/API/firmware work proceeds independently of application/HIL prerequisites. SOFTWARE_DONE dependencies unlock further software; physical acceptance still requires the relevant gate.

## 2. Initial blockers and evidence

| ID | Missing prerequisite / observed limitation | Affected completion | Clearing action |
|---|---|---|---|
| B01 | CLEARED: in-tree ProtoGL/ProtoGC/SDK and toolchains restored; lock manifest and compatible protocol-9 native/Arm pair exercised | Baseline/build/source dependency blocker closed | Record actual modified host hashes and exact image manifests when publishing a pair; base submodule revisions alone do not capture working-tree edits |
| B02 | Missing approved target schematic/module/part inventory and captures; Pico2-safe reference routing is implemented, not a product PCB approval | Physical electrical, boot, output and device gates | Supply actual RP/S3 board, isolation/reset circuit, panel/LED/custom receiver/attached part models and measured traces |
| B03 | CLEARED: complete vendor boot/XIP/UART/QMI details recovered and checked against Pico SDK2.3.1, RP2350 datasheet release8 | Document-retrieval blocker closed, hardware still unqualified | Retain primary vendor citations; no physical timing inference from documentation |
| B04 | No accessible RP/S3/output hardware or serial debug device; product scene/FPS/current targets are not supplied | Runtime stack high-water, power, sustained load and offload measurements | Execute G03/G05..G12 on the approved hardware; application integration remains deferred by the user |

| Evidence ID | What was actually observed | Limit |
|---|---|---|
| E00-SOURCE | Reviewed README, previous plan, active CMake/config/main, firmware subsystem source, simulator scripts; two read-only subsystem audits completed | Source facts only; no source edits, firmware build, scheduler race proof, or hardware run |
| E00-PROTOGL | Public ProtoGL tree `c0570d8dee80c06325f14b91ef262e471fa440ee` inspected; actual headers are protocol v8, with V9 feature flags and i80 host transport | Research identity, not an established compatible dependency lock |
| E00-SMOKE | `bash sim/build_sim.sh` exited 1: `ERROR: expected ProtoGL/ProtoGC sources at /home/polar/ProtoGL/src` | Dependency guard stopped before compilation/rendering; no passing baseline or image comparison claimed |
| E00-DOC | Transient planning-artifact validator passed: 105 unique contiguous tasks in 13 packages, acyclic/resolved dependencies, 13 gates, complete mandatory-release reachability, optional PSRAM excluded from minimum gating, 7 valid local links, existing documented shell entrypoints and balanced code fences | Documentation consistency only; does not qualify firmware/hardware or imply any implementation gate passed |
| E01-SUBMODULE | Cloned `git@github.com:BaiTian6641/ProtoGL.git` into `ProtoGL/` as a submodule at `c0570d8dee80c06325f14b91ef262e471fa440ee`; firmware CMake and native-check scripts now select `ProtoGL/src` | API source is locally editable with its own upstream history; full firmware/dependency compatibility and Arduino hardware operation remain unqualified |
| E01-API | `bash ProtoGL/tests/syntax_check/run_roundtrip.sh` compiled and executed successfully: `RESULT: PASS (wire-format round-trip)`; camera/object records, v8 generation handles and tampered-frame CRC rejection exercised | Native shared API behavior only; no Arduino/ESP32 peripheral or RP firmware execution |
| E01-FIRMWARE | Updated `bash sim/build_sim.sh` advanced past local ProtoGL and exited 1 at `ERROR: expected ProtoGL/ProtoGC sources at /home/polar/ProtoGC/src` | B01 remains open for the allocator dependency; compilation/rendering were not reached |
| E01-SHELL | `bash -n` passed for `sim/build_sim.sh`, `sim/run_f04_check.sh`, and `tests/syntax_check/run_scene_check.sh` | Syntax verification of changed entrypoints, not a passing allocator/render scenario |

Initial dependency failures above are historical evidence, not current build failures. User `.bak2` files are preserved; obsolete active octal/I2C-slave/MRAM/tier/persistence/duplicate-diagnostic paths were removed by protocol-9 cutover.

### 2.1 Exercised software evidence and current contract

| Evidence ID | Observed result / artifact | Scope limit |
|---|---|---|
| E02-SW | Native geometry76 checks, command/render pipeline51 consumer checks, full redraw/resource-only/scissor scenarios, 2D clipping/text/blend suite, scheduler63 dispatches955 tiles, transport75 boundary checks, typed cache74051 checks, clock304 checks covering nine exact profiles/VSEL ordering/publication, device222 checks and all four backend encoder/state suites pass | Actual shared parser/frame engine/native two-worker scheduler and platform-seam clock logic; no physical RP execution |
| E02-HOST | Protocol9 link/session/READY/ownership/recovery suite and ROM-loader checksum/readback/partial-read/deadline/reset-retry suite pass; exact MHz mapping through 336/ID8 needs Info9 mask negotiation in-session; actual Arduino-ESP32 3.3.6/IDF5.5.2 Xtensa hello_triangle/layered_display objects compile | Real host adapter objects and bounded native HAL, not flashing/electrical tests |
| E02-BUILD | FLASH_LOCAL and RAM_HOST ELF/bin/map/IMAGE_DEF/vector/identity/budget manifests generated; optional selected-CS QMI variants and FLASH_LOCAL hostless diagnostic also link/package; actual target reports under build/runtime-*/ | Compiler/static memory evidence; ≥32KiB reserve enforced, no runtime stack/current guarantee |
| E02-PACK | Twelve real-image packaging tests pass; adjustable-clock pair identity/source hash changed after this cutover and requires a fresh pair bundle after every final image build | Fresh images/reproduction evidence should be generated when a release is requested; no hardware/OTP/authentication proof |
| E02-CORPUS | All9 current scene consumer oracles pass. Cube/2D/textures/full-frame PSB/empty/nearest-bilinear references remain byte-exact. Historical teapot differs5376 pixels, alpha1 pixel and multicamera456 pixels after lossless candidate/depth/edge corrections | Comparator still exits1 for those3 unchanged historical references; no repinning or all-goldens-pass claim. Geometry/edge/depth invariants are checked independently; RP image/quality gate remains open. |

Published limits: 8192 framebuffer/depth pixels,64 tile cells,1024 transformed vertices,1280 source/projected triangles,64 meshes/materials,16 textures,4 cameras,64 draws,8 layers,4 shader programs,128 queued2D operations,64KiB scene arena,2×16KiB ingress,2Mi weighted post-FX operations/frame. Limits are aggregate-bounded, not independently fillable.

The full-frame three-program PSB corpus needs163 weighted operations/pixel ×8192 =1,335,296 operations. The initial1Mi work proposal was raised to finite2Mi; the benchmark was not narrowed. Source-triangle admission1280 preserves the existing1166-triangle teapot. Clip/view controls also work on primary target0. Camera projection remains panel-space; target stride/scissor/extents are actual.

SRAM cutover: float80-byte triangles plus4-byte conservative tile bounds; parser/view preparation share bounded13KiB target storage; typed16KiB depth/world union starts explicit C++17 lifetimes. Worker/output readers are drained before phase reuse. Obsolete unused12KiB frame bump pool is removed; overrides are transaction-owned in the scene arena.

Runtime SPI: control32 bytes (prefix0x80 included), read64 RX-only bytes after0x00 command, bulk0x81 prefix excluded from exact logical payload, one/four-lane data, 1MHz initialSCK and64µs minimumCS gap. READY is mandatory before every CS assertion. Physical ingress reserves at least31 body bytes even for a shorter resource batch; logical length is never inflated. Status payload44 distinguishes Control from BulkTerminal,45 reservedzero. Terminal bulk result stays sticky until the next control. QueryClock exposes requested/actual/override/transition/result. Accepted frame IDs and transfer sequences increase within a session; no wrapping, no arbitrary mixed-resource replacement.

Attached services are fixed-address I2C0(0x3D,GP20/21), raw temperature ADC and explicit-drive GP26; errors and replies are correlated and drained. Runtime reset is Busy with an armed reservation. Clock maintenance quiesces transport first, restores earlier gates on Busy and blocks new reservations while draining. Watchdog feeds only after a full main-loop turn, so a stuck worker wait cannot feed itself.

Final solidity corrections have failing-before/passing-after consumers: reserved generation255 cannot be created through raw firmware records; rejected paired shader dispatch returns FrameFailed; normal two-band effects resume from immutable neighbour data. Pass auto-uniforms use a small standalone bank rather than cloning verified instructions/constants onto4KiB stacks. Multicamera metrics count the cumulative source-preparation work once, not cumulative-prefix sums. A throwaway real-allocator diagnostics run observed capacity65536/free61248/used4096/peak8192/largest57152 bytes under fragmentation, then complete coalescing; it was removed.

Adjustable clock software: exact IDs 150/100/75/125/240/288/250/300/336MHz. New 240/288/336 are 48MHz multiples; 300 carries `UserBoardReference` from the user's own board observation; 336 is requested-only. Required VSEL is1100mV≤150MHz and1200mV>150MHz, always below the SDK1.3V limit and never disabling it. Firmware raises voltage before upward transitions and lowers it only after a slower clock/client retime. Fixed clock tree is ref12MHz XOSC/ticks and PLL_USB48MHz for UART/SPI/USB/ADC/HSTX. Old Capabilities mask0xff covers IDs0..7; Info9 reports mask0x01ff. SRAM-only admits all nine. Active PSRAM remains≤150MHz because its SDK3-bit RXDELAY cannot preserve32MHz timing at240+; no unqualified clamp. Native clock checks304 and host clock boundary checks pass; physical current/stability at300/336 remains OPEN.


Qualified rates/current/power, exact panel/LED electrical captures, actual GPIO reset isolation and physical cold-asset timing are not inferred from these software results. P10-03/04 contain implemented DSP/quad software but physical benefit/qualification remains BLOCKED; dirty regions and a second-stage loader remain unselected OPTIONAL.

### 2.2 Reference BOM and physical work-order prerequisites

SRAM-only RAM_HOST needs RP2350, reference clock/crystal/load network, local regulator/decoupling, RUN control, runtime link/output wiring and the actual display's buffers/drivers/power supply. Host asset storage and boot-UART isolation/strap circuit are part of this BOM; removing GPU NOR does not remove reset/level/isolation requirements. FLASH_LOCAL adds ordinary NOR boot storage, not nonvolatile graphics RAM. Optional PSRAM adds one allowed APS6404-class part, selected-CS8 routing/isolation and32KiB active staging carved from the existing arena; HUB/CUSTOM require a different approved routing to coexist.

The complete parts/passive values/current/power-gating BOM cannot be approved without B02/B04. Exact HIL work orders are retained in task acceptances: both boot-storage reset/corrupt-image cycles, scope-checked control/read/bulk/abort/READY, whole-scan HUB75 swap/OE, SSD1331 DC/endian/final-clock release, LED reset/pulse bounds/failed-black limitations, custom latch/OE, device disconnect timeouts, clock retiming, SRAM-stack high-water and sustained workloads. Supply schematic/part models and capture references before dispatch; no synthetic measurement closes a gate.


## 3. Shared contracts and write ownership

Before fan-out, the integration owner publishes a contract revision containing: dependency pins, board/profile identity, supported ProtoGL ABI/capabilities, surface/stride/format descriptors, resource-generation/epoch rules, ingress/frame credits, completion meanings, job/event descriptors, resource leases, and clock quiescence hooks. A proposed C/C++ symbol name is not a frozen ABI until that revision is published.

| Owner role | Writable slice | Shared-boundary rule |
|---|---|---|
| Integration owner | `CMakeLists.txt`, `src/gpu_config.h`, `src/gpu_core.*`, `src/main.cpp`, `src/scene_state.h`, `src/command_parser.*`, shared status/capability integration, these documents | One writer; applies coordinated caller/protocol changes and owns whole-profile builds/smoke/gates |
| ProtoGL owner | Actual ProtoGL shared headers, encoder/device/boot/transport adapter, related consumer docs | One cross-repo contract owner; all firmware changes use the same revision; host application callsites migrate with it |
| Scheduler owner | `src/scheduler/` | No independent FIFO protocol, scene mutation, display lifecycle, or clock implementation |
| Renderer owner | `src/render/`, `src/math/`, targeted existing render scenarios | Proposes scene/parser/capability changes to integration owner; no concurrent edits to those shared files |
| Transport owner | `src/transport/` runtime link internals | Management status integration stays with integration owner; publish exact timing/abort and descriptor contracts first |
| Display owner(s) | Assigned nonoverlapping `src/display/` backend files; existing SSD1331 implementation as agreed | Common DisplayDriver/DisplayManager interface has one designated owner; backend agents cannot independently alter it |
| Memory owner | `src/memory/`, proposed optional asset-backend implementation | Scene pointers/handles and linker budget changes go through integration owner; no autonomous relocation of live resources |
| Device/clock owner | Assigned device-service files (new) or `src/gpu_clock.*` | Common registry/maintenance hooks are integration-owned; do not edit shared config/core opportunistically |

New file locations in task descriptions are proposed work, not existing components. Do not reorganize the repository into the old plan's fictional `rp2350/`/`host/` directories. Host code belongs in ProtoGL or the real application, not this firmware repo.

Every agent must read current repo instructions and its assigned architecture sections, inspect actual definitions/callers, and use LSP references before an exported-symbol change when available. Shared interfaces change once with every caller migrated; no compatibility aliases/no-op fallbacks. Only code made obsolete by the agreed cutover is removed. User files/backups outside that scope are preserved.

During a concurrent implementation wave, agents skip builds, tests, linters, and formatters. They implement the assigned consumer-visible regression cases where needed and return commands/scenarios to the integration owner. After integration, run the selected checks once for that wave and exercise the real changed path. A hardware-blocked package may deliver software evidence, but its HIL gate remains open.

## 4. Package ledger

### P00 — Baseline, dependencies, and configuration

Lead: integration owner. Existing targets: root build/config/docs and existing `sim/`/`tests/syntax_check/`; external inputs: actual ProtoGL/ProtoGC, application, schematics, and vendor documents.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P00-01 | SOFTWARE_DONE | — | Inventory active sources and preserve/revise decisions; distinguish old README, v1 proposal, code, and public ProtoGL | Acceptance: Architecture §2/2.1 and E00-SOURCE/E00-PROTOGL; no implementation gate credited<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P00-02 | SOFTWARE_DONE | P00-01 | Locate/restore ProtoGL and ProtoGC, inspect dependency instructions, pin SDK/toolchain/IDF and a compatible host/firmware pair; make dependency acquisition reproducible | Acceptance: Manifest with exact revisions/paths and no silent moving-branch fetch; B01 cleared by real checkouts<br>Observed: dependencies.lock.json, exact source-hashed protocol9 pair and two snapshot rebuilds E02-PACK; no physical acceptance inferred. |
| P00-03 | SOFTWARE_DONE | P00-01 | Obtain full current RP datasheet/errata/hardware guide; inspect SDK no-flash startup, image metadata, PIO/DMA/GPIO windows and ROM UART boot | Acceptance: Source/version checklist covering boot selection, §5.8, timing, cache/errata and board constraints; B03 cleared<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P00-04 | SOFTWARE_DONE | P00-02 | Run existing native simulator/reference checks and current Pico baseline build without changing implementation or repinning goldens | Acceptance: Actual command logs, ELF/map, rendered images and failing cases recorded; blocked now by B01<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P00-05 | BLOCKED | P00-01 | Extract RP/S3 board pins, flash population, RUN/boot/isolation, output/LED/controller/custom protocol and attached-device bus roles | Acceptance: Approved configuration sheet with reset states/voltage/pin/peripheral constraints; currently B02, source defaults are not approval<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P00-06 | DEFERRED | P00-01 | Capture real representative/worst 3D scene, source quality and S3 per-stage/overlap trace under networking; identify host raster callsites | Acceptance: Replayable inputs, authoritative scene limits, unchanged-quality oracle and baseline S3 occupancy; currently B02<br>Observed: User deferred application adaptation; ProtoTracer-ESP32S3-Port.git future target only. |
| P00-07 | SOFTWARE_DONE | P00-04, P00-06 | Map actual protocol/capabilities and rendering conventions; classify working behavior versus deliberate correctness changes | Acceptance: Delta matrix: transforms, projection, clipping, depth, textures, alpha, camera/layers, PSB and unsupported/inert features<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P00-08 | BLOCKED | P00-03, P00-05, P00-06, P00-07 | Freeze minimum scene contract, output/reference devices, frame/latency/power targets, supported profiles and reserve floor | Acceptance: G00 signed by integration owner; 60 FPS remains a development objective unless explicitly supported by the frozen contract<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |

### P01 — One ProtoGL contract and cross-repo migration

Lead: ProtoGL owner with integration owner. Targets: actual ProtoGL `PglTypes.h`, `PglOpcodes.h`, `PglParser.h`, `PglEncoder.h`, `PglDevice.h`; firmware parser/status integration. Start with current v8 command records, not a new TGPU protocol.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P01-01 | SOFTWARE_DONE | P00-02, P00-07 | Freeze serialized command/transport-control/capability records, byte order, CRC, version-bump rules, unsupported-command behavior and frame limits | Acceptance: One shared specification and executable boundary/compatibility vectors; no duplicated wire header or unversioned reinterpretation<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P01-02 | SOFTWARE_DONE | P01-01 | Define fresh session, transfer identity/retry window/wrap, accepted/rendered/transferred/displayed/failed/replaced states and buffer-reuse times | Acceptance: Same-session duplicate returns same result; conflicting content/stale session/wrap rejected deterministically<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P01-03 | SOFTWARE_DONE | P01-01 | Define typed resource generations, nested material/texture references, upload/commit/update/destroy and generation-exhaustion policy | Acceptance: Stale and nested handles cannot alias a reused slot; index255 remains reserved unless jointly migrated<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P01-04 | SOFTWARE_DONE | P01-01, P00-08 | Specify immutable frame/pass inputs, target formats/strides/extents, clear/load behavior, opaque/transparent state and order, shader-stage limits | Acceptance: Consumer-facing semantics with no accepted inert camera/output features; changes mapped to actual encoder calls<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P01-05 | SOFTWARE_DONE | P01-02, P01-03, P01-04 | Migrate the local ProtoGL device/encoder and firmware callers together; retain the Arduino/ESP32 host adapter while separating shared command/resource semantics from platform transport; remove superseded aliases/assumptions/persistence claims at cutover | Acceptance: Complete reference/caller matrix and compatible host/firmware pair; Arduino application framework preserved, no Arduino dependency in bare-metal firmware, incompatible pairs rejected before drawing<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P01-06 | SOFTWARE_DONE | P01-04 | Evolve common display descriptor/present/release/capability and clock-quiescence interfaces through existing DisplayDriver/DisplayManager | Acceptance: Every existing caller/driver adapted; bounded busy/error/result behavior, no pointer-swap shim or ignored reconfiguration<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P01-07 | SOFTWARE_DONE | P01-02, P01-06 | Define board-allowlisted attached-device requests/results/events with finite lengths, ownership, deadlines and cancellation | Acceptance: Same completion/capacity semantics shared across host and firmware; no host pointers/register-write escape hatch<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P01-08 | SOFTWARE_DONE | P01-05, P01-06, P01-07 | Publish contract revision and migrate host examples/specs; capability query includes actual profile, capacities and supported completion levels | Acceptance: G01 evidence; API semantics come from compiled/qualified behavior rather than historical metadata version labels<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |

### P02 — Build profiles, SRAM accounting, and hardware leases

Lead: integration owner plus memory/resource slice. Targets: CMake/config/startup, allocator use, driver claims; new registry/board-profile files only where needed.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P02-01 | SOFTWARE_DONE | P00-02, P00-03, P01-01 | Add explicit Arm RAM_HOST/FLASH_LOCAL selection and independent output/diagnostic/optional-memory features, retaining one runtime | Acceptance: Clean profile builds produce correct SDK binary types; unknown/incompatible selections fail early; options documented as real only after implementation<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P02-02 | SOFTWARE_DONE | P02-01, P00-08 | Measure map/target sizeof/live intervals for code, both stacks, slots, triangles/tree, transform/morph/effects scratch, ingress, layers/pools and scan data | Acceptance: One non-double-counted byte/address ledger per profile; actual peak/reserve and linker alignment costs included<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P02-03 | SOFTWARE_DONE | P02-02 | Enforce aggregate reservations for capped scene heap, layers, raw/pool objects, upload rollback and frame/output arenas | Acceptance: Worst supported allocations fit; allocation failure leaves previous objects valid; double free/invalid pool block rejected<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P02-04 | SOFTWARE_DONE | P00-05, P02-01 | Centralize GPIO, PIO window/program words/SMs, DMA/DREQ/IRQ, bus role, timers and clock-domain claims | Acceptance: Conflicting profile rejected before driving pins; failed init rolls back all leases; disabled driver consumes/reports zero resources<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P02-05 | SOFTWARE_DONE | P02-01 | Remove unconditional asset-flash/XIP reads and persistence service from no-persistence/RAM_HOST startup; separate NOR boot storage | Acceptance: SRAM-only/no-flash image performs no local asset-flash access; unsupported persistence gives explicit error; no assumed 4 MiB asset partition<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P02-06 | SOFTWARE_DONE | P02-02, P02-03 | Replace synthetic free-memory/status calculations with authoritative capacity/usage/high-water snapshots, including independent ingress credits | Acceptance: Allocation pressure changes the correct arena counter; transport-ring free bytes are not reported as total SRAM<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P02-07 | SOFTWARE_DONE | P02-03, P02-04 | Define/reset full resource ownership: drain workers and CPU/DMA readers, clear raw pools/handles, queues and cache leases | Acceptance: Reset/cancel/OOM sequences reclaim exactly owned allocations with no stale readers or new-session leakage<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P02-08 | SOFTWARE_DONE | P02-02, P02-05, P02-06, P02-07 | Qualify viable minimum profile budgets; use scratch reuse/packing/bounded geometry design if needed, not silent quality/capacity reduction | Acceptance: G02 map and runtime high-water record; unresolved footprint/scene mismatch is an explicit blocker, not a passing small-scene build<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |

### P03 — Both boot paths, image packaging, and recovery

Lead: boot/ProtoGL host owner; integration owner owns target CMake/main/startup. New host loader/manifest code belongs in ProtoGL. No OTP provisioning.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P03-01 | SOFTWARE_DONE | P02-01, P02-05, P00-05 | Bring up FLASH_LOCAL safe startup/discovery with actual flash geometry and no mandatory host upload or external RAM | Acceptance: Installed GPU boots at 150 MHz with safe pins; host absence does not trigger an unbounded self-test or require asset persistence<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P03-02 | SOFTWARE_DONE | P02-01, P02-08 | Produce RAM_HOST ELF/map/flat image; verify vectors, IMAGE_DEF, load/run addresses, initialized sections, zero padding and all live SRAM exclusions | Acceptance: Packaging rejects wrong type/profile/load address/size; valid image plus worst supported runtime fits SRAM<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P03-03 | SOFTWARE_DONE | P03-02, P00-03 | Implement manifest/bundle validation and image embedding/storage with exact bytes, hash, identity, ABI/profile and padded bounds | Acceptance: Missing, mismatched and oversized images fail packaging; no full duplicate host-RAM image copy required<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P03-04 | SOFTWARE_DONE | P03-03, P00-05 | Implement S3 RUN/boot-UART state machine with absolute deadlines, partial reads, echo-paced chunks, whole-upload restart on uncertain pointer | Acceptance: Split reads/echo delays/truncation/corruption resolve finitely; launch echo is not application-ready<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P03-05 | SOFTWARE_DONE | P03-04, P06-02 | Implement runtime HELLO/identity and explicit boot-driver release; no auto QMI initialization before release | Acceptance: Expected firmware/profile/ABI observed over ordinary-GPIO link; host UART TX/selection truly high impedance before RAM handover<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P03-06 | BLOCKED | P03-01, P03-05, P00-05 | Qualify independent host/RP resets and flash-populated versus flashless electrical boot variants | Acceptance: Scope/capture proves flash/PSRAM not selected/driven incorrectly; unsafe same-board dual boot rejected or isolated explicitly<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P03-07 | SOFTWARE_DONE | P03-05, P04-07, P06-07 | Add recovery: stop submissions, preserve/cancel DMA ownership, reset selected profile, new session, invalidate handles/fences and reconstruct latest scene | Acceptance: RP stall and S3 restart recover without double-executed mutation, stale handles or lost host networking responsiveness<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P03-08 | BLOCKED | P03-06, P03-07 | Run controlled cold/warm/corrupt/truncated boot sequences on both profiles and record startup/readback/fault behavior | Acceptance: G03 captures and actual counts; initial engineering target 100 controlled boots/profile, with sample/conditions disclosed<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P03-09 | OPTIONAL | P03-08, P06-08 | If measured startup requires it, add a relocated SRAM second-stage loader with reserved code/stack/vectors, bounded destinations, verification and correct RP launch | Acceptance: Faster measured startup plus full-image ROM recovery; loader/application overlap or generic startup-skipping jumps rejected<br>Observed: Not selected: direct ROM upload/full redraw baseline; no measured requirement for this optional path. |

### P04 — Bare-metal event scheduling and resource epochs

Lead: scheduler owner; integration owner handles scene/parser publication. Do not add RP FreeRTOS or a second OS scheduler.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P04-01 | SOFTWARE_DONE | P01-02, P02-04 | Unify tagged multicore dispatch, immutable context lifetime, acquire/release ordering, completion epoch and one FIFO/doorbell owner | Acceptance: Exactly-once tile claims and correct DONE epoch; stale completion cannot release next job; SDK lockout compatibility explicit<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P04-02 | SOFTWARE_DONE | P04-01, P02-03 | Add fixed event/job descriptors and bounded queues/dependencies with defined full/cancel/fault results | Acceptance: Capacity exhaustion preserves active output; no allocation on job dispatch/ISR/hot inner loops<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P04-03 | SOFTWARE_DONE | P04-02, P00-08 | Add core-0 service points between bounded transform/tile/span/row work, prioritizing urgent completions/faults over background upload/diagnostics | Acceptance: Maximum main-loop/host/device service gap measured under accepted worst-case raster/effects work; tiles split if their WCET is too high<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P04-04 | SOFTWARE_DONE | P04-01, P01-03, P02-03 | Make resource replacement and immutable frame publication transactional; defer destruction/version retirement until last CPU/DMA read | Acceptance: OOM/malformed late record leaves old scene/resources valid; no dangling mesh/texture or partial upload exposure<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P04-05 | SOFTWARE_DONE | P04-04, P01-01 | Separate full-batch validation/capacity reservation from mutation; validate internal counts, dependencies and safe arithmetic without duplicate full SceneState storage | Acceptance: Invalid final command cannot mutate earlier committed resources; allocations reserved before ACCEPTED and released on rejection<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P04-06 | SOFTWARE_DONE | P04-02, P01-04 | Run raster, verified postprocess bands and backend conversion jobs with explicit dependencies; retain both-core tiles unless measured split favors conversion | Acceptance: Disjoint writes and immutable input proven; no parse into active mutable scene, simultaneous static-scratch reuse or unsupported concurrent raster frame<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P04-07 | SOFTWARE_DONE | P04-03, P04-04, P02-07 | Integrate coherent status snapshots, bounded fault/maintenance entry, idle wake race closure and progress-based watchdog | Acceptance: Job/device stall leads to finite failed result/reset; sleeping does not miss work; no lock spans render/DMA/dwell<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P04-08 | SOFTWARE_DONE | P04-05, P04-06, P04-07 | Remove unused legacy RP scheduler/obsolete RasterizeRange protocol and qualify scheduler service/stack bounds | Acceptance: G04: one worker protocol and caller set; physical max-load trace shows declared response bounds, not only sequential sim correctness<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |

### P05 — 3D correctness, semantics, and reusable-frame behavior

Lead: renderer owner. Preserve existing transforms/material code as the starting point; intentional correctness changes need independent expected output, not repinned broken goldens.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P05-01 | SOFTWARE_DONE | P00-07, P01-04 | Encode reference conventions for full object transforms, +Z perspective, orthographic policy, projection/scissor, source formats and winding | Acceptance: Independent transform/projection fixtures and existing scene comparisons distinguish preserved behavior from planned corrections<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P05-02 | SOFTWARE_DONE | P05-01, P04-05 | Validate indices/UV counts/nonfinite transforms and preserve near-plane clipping; fix conservative AABB/morph/near-crossing admission | Acceptance: Visible morphed/near-crossing geometry is not culled from base bounds; invalid indices/input rejected before workers; supported-plane behavior explicit<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P05-03 | SOFTWARE_DONE | P05-02, P02-03 | Make triangle pool, quadtree nodes/leaves and tile-candidate overflow lossless or explicitly fail the frame; include clipping expansion | Acceptance: Dense overlap, ninth deep-leaf entity, node exhaustion and >128 tile candidates never silently remove accepted geometry<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P05-04 | SOFTWARE_DONE | P05-02, P05-03 | Implement declared shared-edge/top-left coverage and perspective-correct UV/depth reconstruction before 16-bit encoding | Acceptance: Foreshortened/intersecting/shared-edge oracle images; bounded quantization/ties, no cracks/double-owned alpha edge or incorrect depth interpolation<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P05-05 | SOFTWARE_DONE | P05-04, P04-04 | Validate complete texture bytes/format/stride; retain declared nearest/bilinear/clamp behavior and direct SRAM fast path | Acceptance: Short upload/format rejection, texel-edge and RGB565/RGB888 reference cases; no out-of-range reads or per-sample allocation<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P05-06 | SOFTWARE_DONE | P05-04, P05-05 | Correct transparent submission order/opaque-depth testing/no transparent depth writes, zero-alpha/discard-mask behavior and ARM/scalar blend parity | Acceptance: Stacked transparent draws, mask occlusion, alpha0/1, saturating channels and target DSP endpoints match declared results; no hidden front-to-back opaque ordering on alpha<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P05-07 | SOFTWARE_DONE | P05-06, P01-06, P02-03 | Reconcile camera/target extents, clear/load/scissor, pass and layer order; effects bind intended target/scissor | Acceptance: Multiple cameras/layers, resize/destroy and same-frame UI/clear cases produce declared output; unsupported larger target/ignored field rejected<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P05-08 | SOFTWARE_DONE | P04-05, P01-04, P02-03 | Verify PSB blob subranges, opcodes, slots, vector bases and instruction limits; derive TEX2D snapshot needs; cap builtin kernels/passes | Acceptance: Malformed programs rejected before execution; deterministic one/two-worker output; bounded registers/uniforms/instructions/kernel work; no software vertex/compute claim<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P05-09 | SOFTWARE_DONE | P05-07, P05-08 | Repair frame-signature/time/layer invalidation and complete-frame reuse, including no-camera/empty frames and alternating-buffer scissors | Acceptance: Multi-frame pixel/fence tests cover time-only effects, 2D-only changes, target/layer lifecycle and unchanged frames; no stale two-frame-old regions or frozen animations<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P05-10 | SOFTWARE_DONE | P05-09, P04-06 | Share/align real firmware and simulator frame orchestration; remove sim-only no-camera behavior that masks firmware faults | Acceptance: Same complete frame sequence under native reference and RP path; skip/reuse still retires resources and resolves each new frame<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P05-11 | BLOCKED | P05-10, P04-08, P03-01 | Run renderer regressions and physical RP scene replay at supported baseline with actual allocation/service accounting | Acceptance: G05: equivalent declared quality, no silent geometry loss/ownership fault; target DSP differences explained, not hidden by desktop-only goldens<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |

### P06 — Runtime PIO link, host DMA ownership, and reliability

Lead: transport owner plus ProtoGL host owner. Boot depends on minimal HELLO, not on an already completed full 3D transport gate; avoid that dependency cycle.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P06-01 | SOFTWARE_DONE | P01-01, P01-02, P00-03 | Write exact single-lane prefix/control/bulk/status waveform including armed-state disambiguation, dummy clocks, direction and abort recovery | Acceptance: Cycle-level/PIO-store design and ESP-IDF phase mapping agree; control remains identifiable while armed; no CPU reaction assumed inside a fast edge<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P06-02 | SOFTWARE_DONE | P06-01, P02-01, P02-04 | Implement low-clock PIO control/status plus host driver and minimal runtime HELLO/identity | Acceptance: Actual round-trip over ordinary GPIO, bounded preparation/error; sufficient for P03-05 without requiring full boot/recovery gate<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P06-03 | SOFTWARE_DONE | P06-02, P02-03, P04-02 | Implement pre-reserved whole-transfer RX DMA ownership, exact count/CS completion, header/payload bounds and immutable parser input | Acceptance: DMA cannot overwrite borrowed data; no modulo-lap ambiguity, partial sync loss, partial-word reuse or hidden ring+slot double allocation<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P06-04 | SOFTWARE_DONE | P06-03, P01-05 | Correct host per-buffer DMA completion, descriptor/source lifetime and bounded one-owner bus service | Acceptance: Delay/reorder completions and attempt buffer reuse; owner never releases/reuses bytes before that buffer's actual completion; networking remains responsive<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P06-05 | SOFTWARE_DONE | P06-03, P06-04, P04-05 | Implement credits/session/retry dedup/status and command-frame assembly; chunks for large uploads, separate resource/frame pacing | Acceptance: Full queues/errors/skipped/no-frame branches keep credits truthful; duplicate mutation executes once; conflicting/stale/oversize batches rejected<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P06-06 | SOFTWARE_DONE | P06-05 | Inject early-CS, partial bits/words, extra clocks, long idle, bad CRC/length and resets in control and bulk states | Acceptance: Return to known control with freed reservation/released egress; last complete output unchanged; no silent corruption or stuck clock wait<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P06-07 | SOFTWARE_DONE | P06-06, P04-07 | Connect real terminal frame/resource results and coherent status, not reception-DMA or pointer-swap timestamps | Acceptance: ACCEPTED/RENDERED/TRANSFERRED/DISPLAYED are distinct and supported levels truthful; reset resolves/invalidate old session safely<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P06-08 | BLOCKED | P06-07, P07-07 | Sweep qualified SCK/packet/gap under scan/render/device load and record data volume/error/goodput | Acceptance: G06: exact bytes and captured direction/CS timing; actual stress volume reported; no inherited 80 MHz promise<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |

### P07 — Display lifecycle, HUB75, and PIO SPI output

Lead: common display owner; separate backend files may fan out after interface freeze. Existing SSD1331 is a hardware-SPI diagnostic, not the required PIO SPI backend.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P07-01 | SOFTWARE_DONE | P01-06, P02-04 | Make DisplayManager lifecycle/routing authoritative; idempotent shutdown, complete init rollback, configure-after-disable and brightness0 | Acceptance: No core bypass uses released driver handles; unsupported formats/regions/geometry rejected; disabled/capability state matches actual leases<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P07-02 | SOFTWARE_DONE | P07-01, P02-03, P04-02 | Implement RGB/encoded-scan ownership, pending/active/retiring publication and backend frame/result identity | Acceptance: Slow conversion/output cannot release/rewrite a CPU/DMA-owned buffer; only complete frame activated; supported terminal event observable<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P07-03 | SOFTWARE_DONE | P07-02, P00-05, P00-03 | Repair HUB75 data-complete/latch/OE handshake and actual mapping/IC initialization | Acceptance: Nonrepeating colors/rows/planes correct; latch waits for all shifted data; no clear-after-event lost-IRQ hang; OE safely blanks<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P07-04 | SOFTWARE_DONE | P07-03, P02-02 | Implement packed unique-plane encoding and bounded conversion/DMA schedules, counting dwell/control records and padding | Acceptance: Byte/FIFO order validated, actual assembled PIO fits shared store; scan footprint agrees with map; no hidden exponential BCM duplication<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P07-05 | SOFTWARE_DONE | P07-04, P04-06 | Make HUB75 scan continuously repeat completed encoded data with bounded boundary CPU work and whole-scan swap | Acceptance: Scan continues during long render/host idle; old descriptors/FIFO drained before reuse; alternating frames show no mixed rows/planes<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P07-06 | SOFTWARE_DONE | P07-02, P00-05, P02-04 | Adapt chosen SPI controller/SSD1331 init into a real normal-mode PIO+DMA backend with bounded direct/staged transfers | Acceptance: Color order/endian/CS/DC/reset/stride correct; release after final shifter clock, not DMA depletion; no self-test scene coupling<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P07-07 | SOFTWARE_DONE | P07-05, P07-06 | Define/observe output completion, repeated-frame counts, refresh versus content rate and available TE/scan boundaries | Acceptance: DISPLAYED only for measured activation; SPI-without-TE reports honest TRANSFERRED; frame/source buffers retire at actual last read<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P07-08 | BLOCKED | P07-07, P00-08 | Inject starvation/reset/cancel/brightness changes on HUB75 and SPI, with real panel captures and fixed source precision | Acceptance: OE/CS safe, no stuck lit HUB75 row, no half-frame publication; no claim of zero CPU conversion overhead<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P07-09 | BLOCKED | P07-08 | Complete both reference-backend qualification and record exact resource/clock/current/output limits | Acceptance: G07: two real backend reports, compiled words/SM/DMA/encoded sizes, panel/controller models and waveform/image evidence<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |

### P08 — LED arrays, custom mapping, and RP-attached devices

Lead: separate backend/device owners after the shared interfaces and registry are frozen. These are required capability families, not empty extension slots; their exact reference hardware comes from P00.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P08-01 | SOFTWARE_DONE | P07-01, P07-02, P00-05 | Implement selected addressable LED PIO program, bounded encoded buffer, channel order, reset/latch gap and finite pixel limit | Acceptance: Real chain shows independent nonrepeating channel/pixel patterns; captured high/low/reset timing meets selected part, even under concurrent load<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P08-02 | SOFTWARE_DONE | P08-01, P02-03 | Apply brightness/current/mapping and black-frame or hardware fail-dark policy; publish complete immutable chain updates | Acceptance: Clock/cancel/reset never truncates a live waveform; explicit latch completion/release; retained LEDs' fault behavior honestly documented<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P08-03 | SOFTWARE_DONE | P01-04, P07-02, P04-04 | Make pixel-layout coordinates actually consumed: rectangular/serpentine/scattered mapping, bounds/orientation/sampling and atomic layout versions | Acceptance: Sparse/nonrectangular reference mapping correct; out-of-range/duplicate destinations follow declared policy; no converter reads mutated layout<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P08-04 | SOFTWARE_DONE | P08-03, P00-05, P02-04 | Implement the selected custom-array reference electrical backend and encode/commit/fault contract; document how another backend plugs in | Acceptance: Actual custom protocol exercised with waveform/data comparison; mapping support is not misreported as universal electrical compatibility<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P08-05 | SOFTWARE_DONE | P01-07, P02-04, P04-03, P00-05 | Implement actual allowlisted attached-device bus service and timestamped bounded results/events; separate master/slave controller roles | Acceptance: Real configured device read/write/event works; HUD/config honored and bus initialized before use; no concurrent unarbitrated I2C master/slave traffic<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P08-06 | SOFTWARE_DONE | P08-05, P04-07 | Implement device timeout/cancel/reset and per-slice byte/deadline budget; ensure requests cannot starve host/output | Acceptance: Hung/disconnected device resolves finitely and releases resources; measured graphics/link service bounds remain satisfied<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P08-07 | SOFTWARE_DONE | P08-02, P08-04, P08-06 | Validate admitted display/device combinations, claims/failure rollback and full profile SRAM under simultaneous traffic | Acceptance: Supported combinations run; overbudget/conflicting pins/SM/store/DMA/bus combinations reject safely before enabling pins<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P08-08 | BLOCKED | P08-07 | Record LED/custom/device reference evidence and capability flags from actual implemented profiles | Acceptance: G08: no no-op backend or fictional sensor driver; exact models/protocols/mappings, timing and fault records retained<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |

### P09 — Adjustable clocks, idle behavior, and power evidence

Lead: clock owner with host/display/memory integration. Frequency changes are transactions through a drained safe point, not a direct I2C/PLL write.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P09-01 | SOFTWARE_DONE | P00-03, P00-08, P02-04 | Define qualified profile table: 150 MHz, candidate lower-power operating point, clocks/dividers, allowed regulator settings, affected clients and rate ceilings | Acceptance: All active SMs/services registered; no four-SM tracking limit or “stable overclock” comment used as qualification<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P09-02 | SOFTWARE_DONE | P09-01, P04-07, P06-07 | Implement requested/actual/override/transition state and host request consumer/status; stop new reservations and safely drain/cancel armed work | Acceptance: Supported request has observable finite result; unsupported frequency rejected; host never clocks bulk at old unsafe ceiling<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P09-03 | SOFTWARE_DONE | P09-02, P07-07, P08-07 | Implement both-core/peripheral/output quiescence and ordered voltage/PLL transitions with verify/rollback/reset fault policy | Acceptance: Busy host/RAM/device/LED/HUB75 prevents unsafe switching; no active output bits/latch/CS corrupted<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P09-04 | SOFTWARE_DONE | P09-03 | Retime every active output/link/bus/RAM client and preserve real-time deadlines/timestamps across changes | Acceptance: Capture output/link rates before/after; divider limits and insufficient sampling rate cause safe rejection/renegotiation, not silent clamping<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P09-05 | SOFTWARE_DONE | P09-04, P04-07 | Implement safe idle event waits and thermal override/recovery without bypassing maintenance or losing requested profile | Acceptance: Sleep/wake race and thermal transitions resolve; continuous scans remain serviced; emergency safe blank/reset distinct from routine throttling<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P09-06 | BLOCKED | P09-05 | Measure lower-power profile under same scene/output quality, board current and energy per committed frame with S3/device activity | Acceptance: Demonstrated qualified lower-power point and power/latency/service tradeoff; if output/link constraints prevent it, record a blocker and do not close G09<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P09-07 | BLOCKED | P09-06 | Qualify supported profiles and disable/tag unqualified overclocks; separate component and board power | Acceptance: G09 records actual clocks/voltage/temperature/sample counts/errors; no baseline dependence on 360 MHz self-test or OC<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |

### P10 — Measured acceleration, bottlenecks, and optional wider link

Lead: renderer/performance slices with integration owner. A candidate that loses measured quality/cost may be rejected; rejection evidence finishes an evaluation, not an advertised feature.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P10-01 | BLOCKED | P05-11, P06-08, P07-09, P08-08, P09-07 | Profile real occupied stages/overlap, host offload CPU relief, contention, queue age and worst service gaps at supported clock | Acceptance: Reproducible bottleneck report with p50/p95/p99/max and unchanged quality; throughput/latency and submit/display metrics distinguished<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P10-02 | BLOCKED | P10-01 | Apply only justified transform/material/span/conversion/candidate optimizations; preserve ordered alpha and bounds | Acceptance: Before/after real-scene cycles, images, code/SRAM cost and service gaps; no duplicate unused tree/bin cache or hidden copying<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P10-03 | BLOCKED | P10-02, P04-08 | Evaluate SIO interpolator/DSP/FPU fast paths for declared sampling/LUT/edge operations; core-local state protected | Acceptance: Exact parity/declared rounding and target disassembly/cycles; enable only real benefit, no Helium/SIMD/PIO rasterization claim<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P10-04 | BLOCKED | P06-08, P10-01 | Add negotiated dual/quad payload data only when transport/startup/assets bottleneck; preserve single-lane control/abort | Acceptance: Same byte/CS/fault cases at each width; pinned ESP-IDF phases and PIO program fit, measured useful goodput under output load<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P10-05 | OPTIONAL | P05-09, P10-01 | Add generation-correct dirty conversion/regions or bounded frame replacement only when demonstrated useful | Acceptance: Alternating-buffer moving/disappearing/overlapping objects and all changes since each buffer's generation remain correct<br>Observed: Not selected: direct ROM upload/full redraw baseline; no measured requirement for this optional path. |
| P10-06 | BLOCKED | P10-02 | Report acceleration decisions and achieved baseline/frame/power envelope; qualify selected optional paths separately | Acceptance: G10 uses real rendered/activated frames and defined scene target; limiting stage/remedy recorded if 60 FPS objective is not met<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |

### P11 — Optional PSRAM texture/asset expansion

Lead: memory owner. Entire package is optional and does not gate the SRAM-only release. QMI preferred for new routing only after boot/XIP ownership is safe; PIO is the alternative, not a second required backend.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P11-01 | BLOCKED | P00-08, P02-08, P10-01 | Prove residency need; select exact part, QMI/PIO routing, CS/window, electrical/reset/isolation and SRAM staging budget | Acceptance: Dated capacity/BOM/benefit decision and full timing datasheet; no MRAM/nvPSRAM or second RAM interface added by default<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P11-02 | SOFTWARE_DONE | P11-01, P03-06, P02-04 | Implement reset/ID/mode/verified capacity and absence/init-failure behavior; initialize allocator only after capacity known | Acceptance: No broad CS probing; zero usable memory on absence with released leases; minimum no-RAM profile unchanged<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P11-03 | SOFTWARE_DONE | P11-02 | Implement backend transfers/mapping/cache policy with one transaction owner, safe DMA tails/address0, chip/page/select bounds and actual bus-complete release | Acceptance: Boundary/alignment/chip-crossing patterns correct; QMI CPU/DMA/alias/direct coherence or PIO CS/turnaround/IRQ completion explicitly verified<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P11-04 | SOFTWARE_DONE | P11-03, P04-04, P05-05 | Integrate typed class/generation asset identity, chunk upload, complete-range prefetch and pinned SRAM spans consumed by real mesh/texture renderer | Acceptance: Larger-than-one-cache-line resources actually render; no raw PIO pointer, first-line-only promotion, ID-class collision or stale scene pointer<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P11-05 | SOFTWARE_DONE | P11-04, P09-04 | Implement lease-aware eviction/migration/clock retiming and absence/capacity-pressure handling; keep active output/depth/code in SRAM | Acceptance: Live CPU/DMA span cannot be moved/evicted; failed promotion leaves original data valid; host receives bounded explicit capacity result<br>Observed: Source/native/Arm evidence E02-SW/E02-BUILD; physical acceptance remains unqualified/open. |
| P11-06 | BLOCKED | P11-05, P06-08, P08-08 | Stress exact RAM timing/cache/refresh boundaries with both cores, host uploads and real display/device load | Acceptance: Real mismatch/latency/prefetch/stall records; capacity benefit and performance costs versus SRAM disclosed<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P11-07 | BLOCKED | P11-06 | Qualify selected optional profile across both allowed boot-storage variants and publish honest capability limits | Acceptance: G11 hardware evidence; every untested RAM/boot combination excluded, SRAM-only delivery does not wait for expansion<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |

### P12 — End-to-end host offload and release gates

Lead: integration owner with ProtoGL/application owner. Local host simulation is not a substitute for executing the actual S3/RP/output system.

| ID | Status | Depends on | Fine-grained work | Done evidence |
|---|---|---|---|---|
| P12-01 | DEFERRED | P01-08, P06-07, P05-10 | Migrate actual application render boundary to resident indexed meshes/materials and changing camera/object batches, leaving animation/networking intact | Acceptance: Instrumented S3 path no longer calls heavy 3D rasterizer; equivalent scene trace/quality, command/resource traffic quantified<br>Observed: User deferred application adaptation; ProtoTracer-ESP32S3-Port.git future target only. |
| P12-02 | DEFERRED | P12-01, P03-07 | Integrate resource reupload, bounded one-active/one-waiting frame pacing and fresh-session restoration without stale DMA buffers | Acceptance: Latest complete scene restored after reset; mixed mutation batches never arbitrarily coalesced; uploads cannot create unbounded queue latency<br>Observed: User deferred application adaptation; ProtoTracer-ESP32S3-Port.git future target only. |
| P12-03 | DEFERRED | P12-02, P03-08, P07-09, P08-08 | Run required output/device references with each validated boot profile on SRAM-only firmware | Acceptance: Indexed mesh draws/real terminal fences, proper mapping and actual attached-device operation on each declared combination; no host-rendered-frame shortcut<br>Observed: User deferred application adaptation; ProtoTracer-ESP32S3-Port.git future target only. |
| P12-04 | DEFERRED | P12-03, P09-07 | Exercise live qualified lower-power switching and busy/reset/device fault cases under networking/uploads | Acceptance: No stale handles, half-frame output, LED pulse corruption or lost host responsiveness; actual power/latency/service evidence<br>Observed: User deferred application adaptation; ProtoTracer-ESP32S3-Port.git future target only. |
| P12-05 | DEFERRED | P12-04, P10-06, P03-08 | Run sustained agreed worst/representative workload; inject transport/display/device starvation and recovery faults | Acceptance: Initial engineering objective one-hour scene run and recorded large deterministic link volume (e.g. 1 GiB); disclose actual completed volume/sample/conditions, no statistical production guarantee<br>Observed: User deferred application adaptation; ProtoTracer-ESP32S3-Port.git future target only. |
| P12-06 | SOFTWARE_DONE | P12-05 | Package exact compatible host/RP bundle, release manifests, build/profile commands, pin/resource maps, benchmark/replay and recovery procedures | Acceptance: Fresh checkout rebuilds tested pair and reproduces declared scenario/results; incompatible images rejected; no bespoke host OTA takeover<br>Observed: build/release-protocol9-final with both images/ELF/maps/32-byte manifests, Arduino host, locked compile/tool/replay snapshot; both scripts and bundled native full-frame PSB scenario exercised, E02-PACK. Application/physical tests remain deferred/open. |
| P12-07 | BLOCKED | P12-06 | Review minimum/static/optional variants' complete BOM including passives, buffers, clock and isolation; remove obsolete cutover paths/claims only | Acceptance: No hidden nvRAM cost or required overclock; all affected callers/docs migrated; preserved user hardware/functions listed<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |
| P12-08 | BLOCKED | P12-07 | Close required gates and publish release evidence plus separately selected optional-profile qualification | Acceptance: G12 complete only with physical requirements met; unavailable hardware remains named open gate, never “done” from stubs<br>Observed: Reachable software delivered; remaining physical/workload evidence requires B02/B04. |

## 5. Gates and acceptance records

G01's software contract/caller migration is exercised by E02-SW/E02-HOST and image pairing. All physical/configuration/performance gates remain OPEN. Task software completion and physical acceptance are separate; SOFTWARE_DONE never closes missing HIL evidence.

| Gate | Exit task | Required scope |
|---|---|---|
| G00 | P00-08 | Reproducible baseline, real configuration/workload/quality and explicit targets |
| G01 | P01-08 | One tested ProtoGL contract and clean caller migration |
| G02 | P02-08 | Boot/output-specific measured SRAM fit and authoritative hardware/allocation leases |
| G03 | P03-08 | RAM_HOST and FLASH_LOCAL exercised, bounded recovery and electrically safe reset/handover |
| G04 | P04-08 | Single bare-metal scheduler, immutable inputs/epochs and measured response/stack bounds |
| G05 | P05-11 | Actual RP 3D image/capacity/order/lifetime correctness at supported baseline |
| G06 | P06-08 | Real low-clock/control/bulk link reliability, host DMA ownership and qualified envelope |
| G07 | P07-09 | Real HUB75 and PIO SPI-display reference backends, correct publication/fault boundaries |
| G08 | P08-08 | Real LED/custom-array/device reference support and admitted resource combinations |
| G09 | P09-07 | Qualified adjustable clocks/power with safe host/output/device timing |
| G10 | P10-06 | Measured acceleration/bottleneck/quality/latency/power decisions |
| G11 | P11-07 | Selected optional PSRAM profile only; not a minimum-release dependency |
| G12 | P12-08 | End-to-end required system behavior and reproducible compatible release |

Mandatory release: G00–G10 and G12 for the declared reference configurations. G11, second-stage boot, widened transport, dirty regions, extra accelerators and overclock are not mandatory. If selected, each must pass its own evidence before its capability/profile is shipped. Do not gate the affordable SRAM-only product on PSRAM or OC experiments.

Evidence record for every gate: exact source and dependency identities; build/tool versions; board and silicon revision; boot/output/device/RAM profile; CPU/peripheral/reference/link/output/RAM clocks; regulator/supply/temperature conditions; workload and quality tolerances; linked/reserved/runtime SRAM and stack high-water; assembled PIO words/SM/DMA claims; faults/CRC/retry/overrun/underrun/reset counts; p50/p95/p99/max/queue-age/service-gap/presentation measurements; commands, logs, images, and waveform locations; hardware versus simulated scope. Store real artifacts using the project's chosen evidence location; do not create empty evidence directories or pretend artifacts exist.

## 6. Verification cases to assign with the packages

| Area | Consumer-visible boundaries/invariants |
|---|---|
| Parser/admission | Invalid late record/internal count, arithmetic overflow, nonfinite input, CRC/length/version, allocation failure: reject without partial publication/resource mutation |
| Resources | Failed replacement, nested generation handle, destroy/update during render/conversion/DMA, handle exhaustion/wrap, incomplete upload, reset and pool double free |
| Geometry | Near crossing, morph outside old bounds, bad indices, degenerate/shared edge, dense candidates/tree exhaustion/clipping expansion: no silent omission |
| Depth/textures | Perspective foreshortening, intersecting planes, quantized ties, UV edges, short uploads, nearest/bilinear/RGB formats and target CPU numerical paths |
| Blend/pass | Alpha0/1, stacked source-over order, mask discard/depth, saturation, camera/layer target clear/load/scissor and effect scope |
| Repeated frames | Time-only shaders/noise, 2D-only/layer changes, no-camera/empty, unchanged new IDs, buffer generations and partial regions across alternating targets |
| Shader verification | Truncated tables, bad slots/opcodes/vector bases, excessive instructions/kernels, missing snapshot flags: validate actual derived reads and finite work |
| Scheduler | Exactly-once tiles, stale FIFO/event epoch, context lifetime, queue full/cancel, both-core fault, check-to-sleep race, maximum accepted service gap |
| Transport/boot | Split sync/echo, malformed length/CRC, duplicate/conflict/stale session, reserve-full, early CS/extra clocks, independent reset/corrupt image and release ownership |
| Output/device | DMA versus bus completion, whole-scan swap/retirement, LED reset/latch, no-TE SPI completion, custom mapping, shutdown/reinit, device timeout and bus role conflicts |
| Clocks/power | Requests during armed bulk/live LED/scan/device/RAM, lower sampling ceiling, divider limits, thermal override, timebase preservation and fault rollback |
| Optional RAM | No-chip boot, wrong ID/capacity, address0, tails/alignment/page/chip/select limits, CPU/DMA/cache aliases, multi-line asset access, lease eviction and both boot variants |

Use the existing render/scenario tooling where appropriate. New permanent cases cover plausible user-visible regressions above; throwaway diagnostics can prove simple integration without creating wiring/mock/source-text tests. Exact golden differences caused by a deliberate semantics fix require independent expected output and explanation; never regenerate references simply to make a failure disappear.

Current executable software entrypoints (B01 cleared):

```bash
bash tests/native/run_kernel_tests.sh
bash tests/native/run_pipeline_tests.sh
bash tests/syntax_check/run_scene_check.sh
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
bash sim/run_2d_primitives_check.sh
bash sim/run_f04_check.sh
bash sim/run_golden.sh # historical comparison; intentional differences remain failures
cmake -S . -B build/flash -DPICO_BOARD=pico2 -DBOOT_STORAGE=FLASH_LOCAL -DDEFAULT_DISPLAY=SPI -DPSRAM=OFF
cmake --build build/flash --parallel
cmake -S . -B build/ram -DPICO_BOARD=pico2 -DBOOT_STORAGE=RAM_HOST -DDEFAULT_DISPLAY=SPI -DPSRAM=OFF
cmake --build build/ram --parallel
PGL_RAM_BUILD=build/ram PGL_FLASH_BUILD=build/flash python3 -m unittest discover -s tests/packaging -v
```

Set `PICO_TOOLCHAIN_PATH` if Arm GCC is not on PATH. Profile selectors and exact bundle commands are in README. Golden references are read-only; no `--write-golden`. Native render scenarios execute shared parser/frame/two-worker logic, not physical PIO/display/boot/current. Arm and actual Xtensa example-object builds establish compiler/linker contracts only. Hardware work orders above remain OPEN.

## 7. Parallel dispatch waves

The integration owner selects READY tasks and assigns disjoint writes. Do not dispatch a whole phase to every agent, outsource top-level planning, or fan out before shared contracts are frozen.

| Wave | Independent work after prerequisites | Integration responsibility |
|---|---|---|
| W0 | P00 dependency/document retrieval and application/board evidence collection | Freeze baseline and record exact blockers; no implementation fan-out on guessed pins/workload |
| W1 | P01 shared contract; P02 binary/profile foundations once their rows unlock | ProtoGL/firmware ABI and budget approval before worker interfaces diverge |
| W2 | Scheduler descriptors/epochs; renderer admission/reference corrections; low-clock control/HELLO; common display lifecycle; boot images/host packaging | One writer for parser/scene/core/config; minimal control can bring up boot before the full transport gate |
| W3 | Full receive/host DMA reliability; HUB75 engine; PIO SPI backend; renderer pass/cache/VM correctness; host ROM loader finishing after HELLO | Integrate resource/output completion before declaring end-to-end success |
| W4 | LED backend; mapping/custom reference; attached-device service; clock transaction work when quiescence contracts exist | Admit only combinations that fit real GPIO/PIO/DMA/SRAM/time budgets |
| W5 | Actual S3 offload/scene replay; measured bottleneck changes; optional width/accelerator/PSRAM slices only when selected | Run real baseline/power/quality comparisons; keep optional expansions out of minimum gating |
| W6 | Profile-specific HIL/recovery/stress, release packaging, documentation and BOM review | Close gates from artifacts; list remaining physical qualification gaps explicitly |

Wave labels are scheduling hints; each row's dependency IDs remain authoritative. For example P03-05 requires P06-02, while P06-02 does not require the completed boot gate. Do not introduce a boot/HELLO dependency cycle.

## 8. Copyable agent work order and completion template

```text
Goal: Implement [task IDs] for RP2350 ProtoGPU 3D offload through ProtoGL.
Architecture authority: docs/TinyGPU_Implementation_and_Agent_Handoff_Plan.md v2.
Progress authority: docs/Implementation_Tracker.md.
Contract revision / dependency pins / board profile: [actual selected values].
Prerequisites satisfied: [IDs + evidence]. Open hardware inputs: [specific blocker IDs].
Writable files: [exact nonoverlapping paths]. Shared integration files: parent-owned.
Preserve: existing 3D/material/layer semantics except listed correctness changes;
SRAM-only baseline; both boot profiles; bounded bare-metal scheduling.
Implement: [the task row's concrete steps and consumer-visible acceptance cases].
Do not: add RP FreeRTOS, independent graphics protocol, required PSRAM/OC,
compatibility shims, no-op backends, arbitrary hardware access, unrelated cleanup,
or claim unsupported physical timing/performance. No OTP programming.
During the wave: skip builds/tests/lint/formatters; add necessary deterministic
behavior regressions and return exact commands/scenarios to integration owner.
Return: changed paths/symbols, contract/caller impacts, buffer/resource ownership,
SRAM/PIO/DMA deltas, acceptance scenarios, actual evidence if assigned HIL,
remaining precise blockers. Do not mark HIL gates done from simulation.
```

The bracketed fields are deliberately a copyable dispatch template, not missing firmware implementation. Fill them from this ledger and actual selected configuration before spawning an agent.

Completion report fields: task IDs and scope; changed files/callers; accepted/rejected design decision; ownership/lifetime invariants; capacity/clock/capability deltas; verification command and observed output; waveform/image/build identity; hardware limits/blockers; tracker/gate status update. Integration owner reviews as a consumer, runs the real scenario after the wave, and only then records DONE/gate evidence.
