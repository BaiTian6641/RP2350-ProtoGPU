// AssetService native behavior tests (P11 memory slice).
//
// Exercises the REAL allocator/lease/span algorithm over a plain SRAM
// byte-buffer backing — the same AssetBacking interface the QMI backend
// (mem_qmi_psram.cpp) implements on target. There is deliberately no fake
// QMI device here; actual hardware qualification remains an open gate.
//
// Build and run via tests/memory/run_tests.sh (native g++, no Pico SDK).

#include "memory/mem_assets.h"
#include "memory/mem_qmi_psram.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>

using namespace gpumem;

namespace {

int gChecks = 0;
int gFailures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        ++gChecks;                                                             \
        if (!(cond)) {                                                         \
            ++gFailures;                                                       \
            std::printf("FAIL %s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); \
        }                                                                      \
    } while (0)

#define CHECK_STATUS(actual, expected) CHECK((actual) == (expected))

// ─── SRAM stand-in backing (real transfers, real bounds checks) ─────────────

struct RamBacking {
    uint8_t*  mem;
    uint32_t  capacity;
    uint32_t  readCalls   = 0;
    uint32_t  writeCalls  = 0;
    uint32_t  lastWriteOffset = 0;
    uint32_t  minWriteOffset  = 0xFFFFFFFFu;
    bool      failWrites  = false;
};

bool ramRead(void* ctx, uint32_t offset, void* dst, uint32_t bytes) {
    RamBacking* r = static_cast<RamBacking*>(ctx);
    if (offset > r->capacity || bytes > r->capacity - offset) return false;
    ++r->readCalls;
    std::memcpy(dst, r->mem + offset, bytes);
    return true;
}

bool ramWrite(void* ctx, uint32_t offset, const void* src, uint32_t bytes) {
    RamBacking* r = static_cast<RamBacking*>(ctx);
    if (r->failWrites) return false;
    if (offset > r->capacity || bytes > r->capacity - offset) return false;
    ++r->writeCalls;
    r->lastWriteOffset = offset;
    if (offset < r->minWriteOffset) r->minWriteOffset = offset;
    std::memcpy(r->mem + offset, src, bytes);
    return true;
}

AssetBacking makeBacking(RamBacking& r) {
    AssetBacking b;
    b.ctx = &r;
    b.capacityBytes = r.capacity;
    b.read = &ramRead;
    b.write = &ramWrite;
    return b;
}

void fillRamp(void* ptr, uint32_t count, uint8_t seed = 0) {
    uint8_t* p = static_cast<uint8_t*>(ptr);
    for (uint32_t i = 0; i < count; ++i) p[i] = static_cast<uint8_t>(i + seed);
}

bool rampMatches(const void* ptr, uint32_t count, uint8_t seed = 0) {
    const uint8_t* p = static_cast<const uint8_t*>(ptr);
    for (uint32_t i = 0; i < count; ++i) {
        if (p[i] != static_cast<uint8_t>(i + seed)) return false;
    }
    return true;
}

constexpr uint32_t kStoreBytes   = 32 * 1024;
constexpr uint32_t kStagingBytes = 4 * 1024;

alignas(16) uint8_t gStore[kStoreBytes];
alignas(16) uint8_t gStaging[kStagingBytes];

// ─── 1. Identity, address 0, ranges, content ────────────────────────────────

void testIdentityAndRanges() {
    RamBacking ram{gStore, kStoreBytes};
    std::memset(gStore, 0, sizeof(gStore));

    AssetService svc;
    CHECK_STATUS(svc.bind(makeBacking(ram), gStaging, kStagingBytes), AssetStatus::Ok);
    CHECK(svc.hasBacking());
    CHECK_STATUS(svc.capacityBytes(), kStoreBytes);
    CHECK_STATUS(svc.stats().storeFreeBytes, kStoreBytes);

    // First asset: offset 0 in the store is a VALID address (proven via the
    // backing's recorded write offset).
    AssetHandle mesh{};
    CHECK_STATUS(svc.create(AssetClass::Mesh, 4096, mesh), AssetStatus::Ok);
    CHECK(mesh.isValid());
    CHECK_STATUS(svc.stats().storeFreeBytes, kStoreBytes - 4096);

    uint8_t buf[8192];
    fillRamp(buf, 4096);
    CHECK_STATUS(svc.write(mesh, 0, buf, 4096), AssetStatus::Ok);
    CHECK_STATUS(ram.minWriteOffset, 0);  // store address 0 really used

    // Token form: slot0/Mesh/gen0 packs to token 0 — and it must be valid.
    const AssetToken tok = assetTokenFromHandle(mesh);
    if (mesh.index == 0) CHECK_STATUS(tok, 0u);
    const AssetHandle round = assetHandleFromToken(tok);
    CHECK(round.index == mesh.index && round.assetClass == mesh.assetClass &&
          round.generation == mesh.generation);
    AssetInfo info{};
    CHECK_STATUS(svc.info(round, info), AssetStatus::Ok);
    CHECK(info.assetClass == AssetClass::Mesh);
    CHECK_STATUS(info.bytes, 4096);
    CHECK_STATUS(svc.info(assetHandleFromToken(kAssetTokenInvalid), info),
                 AssetStatus::InvalidHandle);

    // Typed identity collision: same slot+generation, wrong class.
    AssetHandle wrongClass = mesh;
    wrongClass.assetClass = static_cast<uint8_t>(AssetClass::Texture);
    CHECK_STATUS(svc.info(wrongClass, info), AssetStatus::InvalidHandle);
    CHECK_STATUS(svc.write(wrongClass, 0, buf, 16), AssetStatus::InvalidHandle);

    // Range discipline: zero-length, over-end, wrapped offsets all rejected
    // without touching the backing.
    const uint32_t writesBefore = ram.writeCalls;
    CHECK_STATUS(svc.write(mesh, 0, buf, 0), AssetStatus::InvalidRange);
    CHECK_STATUS(svc.write(mesh, 4096, buf, 1), AssetStatus::InvalidRange);
    CHECK_STATUS(svc.write(mesh, 4000, buf, 97), AssetStatus::InvalidRange);
    CHECK_STATUS(svc.write(mesh, 0xFFFFFFF0u, buf, 16), AssetStatus::InvalidRange);
    CHECK_STATUS(svc.write(mesh, 0, nullptr, 16), AssetStatus::BadArgument);
    CHECK_STATUS(ram.writeCalls, writesBefore);

    // Unaligned tail: 3 bytes at asset offset 4093, then odd interior range.
    const uint8_t tail[3] = {0xA1, 0xB2, 0xC3};
    CHECK_STATUS(svc.write(mesh, 4093, tail, 3), AssetStatus::Ok);
    uint8_t back[3] = {};
    CHECK_STATUS(svc.read(mesh, 4093, back, 3), AssetStatus::Ok);
    CHECK(std::memcmp(back, tail, 3) == 0);
    uint8_t odd[37];
    fillRamp(odd, sizeof(odd), 0x55);
    CHECK_STATUS(svc.write(mesh, 1001, odd, sizeof(odd)), AssetStatus::Ok);
    uint8_t oddBack[37] = {};
    CHECK_STATUS(svc.read(mesh, 1001, oddBack, sizeof(oddBack)), AssetStatus::Ok);
    CHECK(std::memcmp(odd, oddBack, sizeof(odd)) == 0);

    // Full >4 KiB asset content, crossing 1024-byte page boundaries, plus a
    // second asset whose store offset itself crosses pages.
    AssetHandle tex{};
    CHECK_STATUS(svc.create(AssetClass::Texture, 8192, tex), AssetStatus::Ok);
    fillRamp(buf, 8192, 0x10);
    CHECK_STATUS(svc.write(tex, 0, buf, 8192), AssetStatus::Ok);
    uint8_t verify[8192];
    std::memset(verify, 0, sizeof(verify));
    CHECK_STATUS(svc.read(tex, 0, verify, 8192), AssetStatus::Ok);
    CHECK(rampMatches(verify, 8192, 0x10));

    // Stale-generation collision after destroy + slot reuse.
    AssetHandle stale = mesh;
    CHECK_STATUS(svc.destroy(mesh), AssetStatus::Ok);
    CHECK_STATUS(svc.info(stale, info), AssetStatus::InvalidHandle);
    AssetHandle reused{};
    CHECK_STATUS(svc.create(AssetClass::Mesh, 64, reused), AssetStatus::Ok);
    if (reused.index == stale.index) {
        CHECK(reused.generation != stale.generation);
        CHECK_STATUS(svc.info(stale, info), AssetStatus::InvalidHandle);
    }
    CHECK_STATUS(svc.destroy(reused), AssetStatus::Ok);
    CHECK_STATUS(svc.destroy(tex), AssetStatus::Ok);
}

// ─── 2. Finite capacity and coalescing ──────────────────────────────────────

void testCapacityAndCoalescing() {
    RamBacking ram{gStore, kStoreBytes};
    AssetService svc;
    CHECK_STATUS(svc.bind(makeBacking(ram), gStaging, kStagingBytes), AssetStatus::Ok);

    // Over-capacity and slot-table exhaustion are explicit, never silent.
    AssetHandle h{};
    CHECK_STATUS(svc.create(AssetClass::Mesh, kStoreBytes + 16, h),
                 AssetStatus::NoStoreCapacity);
    CHECK_STATUS(svc.create(AssetClass::Mesh, 0, h), AssetStatus::BadArgument);

    AssetHandle a{}, b{}, c{};
    CHECK_STATUS(svc.create(AssetClass::Mesh, 8192, a), AssetStatus::Ok);
    CHECK_STATUS(svc.create(AssetClass::Mesh, 8192, b), AssetStatus::Ok);
    CHECK_STATUS(svc.create(AssetClass::Mesh, 8192, c), AssetStatus::Ok);
    CHECK_STATUS(svc.stats().storeFreeBytes, kStoreBytes - 3 * 8192);

    // Free the middle; a too-large request still fails, a fitting one reuses.
    CHECK_STATUS(svc.destroy(b), AssetStatus::Ok);
    AssetHandle big{};
    CHECK_STATUS(svc.create(AssetClass::Mesh, 16384, big),
                 AssetStatus::NoStoreCapacity);
    AssetHandle d{};
    CHECK_STATUS(svc.create(AssetClass::Mesh, 8000, d), AssetStatus::Ok);

    // Free everything: coalescing restores the exact full capacity.
    CHECK_STATUS(svc.destroy(a), AssetStatus::Ok);
    CHECK_STATUS(svc.destroy(c), AssetStatus::Ok);
    CHECK_STATUS(svc.destroy(d), AssetStatus::Ok);
    const AssetServiceStats s = svc.stats();
    CHECK_STATUS(s.storeFreeBytes, kStoreBytes);
    CHECK_STATUS(s.storeLargestFreeBytes, kStoreBytes);
    CHECK_STATUS(s.assetCount, 0);

    // Slot table bound: GPU_MEM_MAX_ASSETS tiny assets, then NoSlots.
    AssetHandle many[GPU_MEM_MAX_ASSETS];
    for (size_t i = 0; i < GPU_MEM_MAX_ASSETS; ++i) {
        CHECK_STATUS(svc.create(AssetClass::LookupTable, 16, many[i]),
                     AssetStatus::Ok);
    }
    AssetHandle overflow{};
    CHECK_STATUS(svc.create(AssetClass::LookupTable, 16, overflow),
                 AssetStatus::NoSlots);
    for (size_t i = 0; i < GPU_MEM_MAX_ASSETS; ++i) {
        CHECK_STATUS(svc.destroy(many[i]), AssetStatus::Ok);
    }
    CHECK_STATUS(svc.stats().storeFreeBytes, kStoreBytes);
}

// ─── 3. Leases, eviction refusal, write-back, migration ─────────────────────

void testLeasesAndStaging() {
    RamBacking ram{gStore, kStoreBytes};
    std::memset(gStore, 0, sizeof(gStore));
    AssetService svc;
    CHECK_STATUS(svc.bind(makeBacking(ram), gStaging, kStagingBytes), AssetStatus::Ok);

    AssetHandle a{}, b{};
    CHECK_STATUS(svc.create(AssetClass::Texture, 3072, a), AssetStatus::Ok);
    CHECK_STATUS(svc.create(AssetClass::Texture, 2048, b), AssetStatus::Ok);
    uint8_t payload[3072];
    fillRamp(payload, 3072, 0x20);
    CHECK_STATUS(svc.write(a, 0, payload, 3072), AssetStatus::Ok);
    fillRamp(payload, 2048, 0x40);
    CHECK_STATUS(svc.write(b, 0, payload, 2048), AssetStatus::Ok);

    // Pin a 3 KiB span; the 4 KiB staging arena cannot fit another 2 KiB.
    AssetSpan spanA{};
    CHECK_STATUS(svc.prefetchSpan(a, 0, 3072, spanA), AssetStatus::Ok);
    CHECK(spanA.data != nullptr);
    CHECK(rampMatches(spanA.data, 3072, 0x20));

    AssetSpan spanB{};
    CHECK_STATUS(svc.prefetchSpan(b, 0, 2048, spanB), AssetStatus::StagingFull);
    CHECK(spanB.data == nullptr);

    // Live lease: destroy is refused, and the backing copy stays valid.
    CHECK_STATUS(svc.destroy(a), AssetStatus::LeaseActive);
    AssetInfo info{};
    CHECK_STATUS(svc.info(a, info), AssetStatus::Ok);
    CHECK_STATUS(info.bytes, 3072);

    // Shared prefetch of the identical range adds a lease on the same span.
    AssetSpan spanA2{};
    CHECK_STATUS(svc.prefetchSpan(a, 0, 3072, spanA2), AssetStatus::Ok);
    CHECK(spanA2.data == spanA.data);
    CHECK_STATUS(svc.stats().leasedSpanCount, 1);

    // Release one lease; the second keeps the span pinned.
    CHECK_STATUS(svc.releaseSpan(spanA, false), AssetStatus::Ok);
    CHECK(spanA.data == nullptr);
    CHECK_STATUS(svc.destroy(a), AssetStatus::LeaseActive);  // still pinned
    CHECK_STATUS(svc.releaseSpan(spanA2, false), AssetStatus::Ok);

    // Double release of an already-released span is rejected.
    CHECK_STATUS(svc.releaseSpan(spanA2, false), AssetStatus::BadArgument);

    // Now the span is cached (unleased): prefetch of B evicts it and succeeds.
    CHECK_STATUS(svc.prefetchSpan(b, 0, 2048, spanB), AssetStatus::Ok);
    CHECK(rampMatches(spanB.data, 2048, 0x40));
    CHECK_STATUS(svc.stats().leasedSpanCount, 1);

    // Dirty write-back: modify through the span, release dirty, evict, and
    // the backing must contain the modification.
    spanB.data[17] = 0xEE;
    CHECK_STATUS(svc.releaseSpan(spanB, true), AssetStatus::Ok);
    uint32_t freed = 0;
    CHECK_STATUS(svc.evictCached(4096, &freed), AssetStatus::Ok);
    CHECK(freed >= 2048);
    uint8_t check[2048];
    CHECK_STATUS(svc.read(b, 0, check, 2048), AssetStatus::Ok);
    CHECK(check[17] == 0xEE);
    CHECK(rampMatches(check, 17, 0x40));

    // flushSpan on a clean span is a no-op Ok.
    AssetSpan spanB2{};
    CHECK_STATUS(svc.prefetchSpan(b, 0, 2048, spanB2), AssetStatus::Ok);
    const uint32_t writesBefore = ram.writeCalls;
    CHECK_STATUS(svc.flushSpan(spanB2), AssetStatus::Ok);
    CHECK_STATUS(ram.writeCalls, writesBefore);

    // Write-back failure keeps the dirty span cached — no silent data loss.
    spanB2.data[3] = 0x77;
    CHECK_STATUS(svc.releaseSpan(spanB2, true), AssetStatus::Ok);
    ram.failWrites = true;
    CHECK_STATUS(svc.evictCached(4096, &freed), AssetStatus::BackingError);
    ram.failWrites = false;
    CHECK_STATUS(svc.evictCached(4096, &freed), AssetStatus::Ok);
    CHECK_STATUS(svc.read(b, 0, check, 2048), AssetStatus::Ok);
    CHECK(check[3] == 0x77);

    CHECK_STATUS(svc.destroy(a), AssetStatus::Ok);
    CHECK_STATUS(svc.destroy(b), AssetStatus::Ok);

    // reset() drain semantics: refuses with a live lease, flushes dirty
    // cached spans first, and never silently discards dirty data when the
    // backing write-back fails.
    AssetHandle d{};
    CHECK_STATUS(svc.create(AssetClass::Texture, 256, d), AssetStatus::Ok);
    uint8_t dpay[256];
    fillRamp(dpay, sizeof(dpay), 0x90);
    CHECK_STATUS(svc.write(d, 0, dpay, sizeof(dpay)), AssetStatus::Ok);
    AssetSpan sd{};
    CHECK_STATUS(svc.prefetchSpan(d, 0, 256, sd), AssetStatus::Ok);
    CHECK_STATUS(svc.reset(), AssetStatus::LeaseActive);  // live lease pins
    AssetInfo dinfo{};
    CHECK_STATUS(svc.info(d, dinfo), AssetStatus::Ok);    // state intact
    sd.data[5] = 0x66;
    CHECK_STATUS(svc.releaseSpan(sd, true), AssetStatus::Ok);  // dirty cached
    ram.failWrites = true;
    CHECK_STATUS(svc.reset(), AssetStatus::BackingError);      // nothing dropped
    CHECK_STATUS(svc.stats().spanCount, 1);                    // dirty span kept
    ram.failWrites = false;
    CHECK_STATUS(svc.reset(), AssetStatus::Ok);                // flush, then reset
    CHECK_STATUS(svc.info(d, dinfo), AssetStatus::InvalidHandle);
    CHECK_STATUS(svc.stats().assetCount, 0);
    CHECK_STATUS(svc.stats().storeFreeBytes, kStoreBytes);
}

void testCompactionMigration() {
    RamBacking ram{gStore, kStoreBytes};
    std::memset(gStore, 0, sizeof(gStore));
    AssetService svc;
    CHECK_STATUS(svc.bind(makeBacking(ram), gStaging, kStagingBytes), AssetStatus::Ok);

    AssetHandle a{}, b{}, c{};
    CHECK_STATUS(svc.create(AssetClass::Mesh, 512, a), AssetStatus::Ok);
    CHECK_STATUS(svc.create(AssetClass::Mesh, 512, b), AssetStatus::Ok);
    CHECK_STATUS(svc.create(AssetClass::Mesh, 512, c), AssetStatus::Ok);
    uint8_t payload[512];
    fillRamp(payload, 512, 0x01);
    CHECK_STATUS(svc.write(a, 0, payload, 512), AssetStatus::Ok);
    fillRamp(payload, 512, 0x02);
    CHECK_STATUS(svc.write(b, 0, payload, 512), AssetStatus::Ok);
    fillRamp(payload, 512, 0x03);
    CHECK_STATUS(svc.write(c, 0, payload, 512), AssetStatus::Ok);

    // Release A/B to cache them, but keep C's original lease. Destroying A
    // punches a hole at the front of the arena.
    AssetSpan sa{}, sb{}, sc{};
    CHECK_STATUS(svc.prefetchSpan(a, 0, 512, sa), AssetStatus::Ok);
    CHECK_STATUS(svc.prefetchSpan(b, 0, 512, sb), AssetStatus::Ok);
    CHECK_STATUS(svc.prefetchSpan(c, 0, 512, sc), AssetStatus::Ok);
    CHECK_STATUS(svc.releaseSpan(sa, false), AssetStatus::Ok);
    CHECK_STATUS(svc.releaseSpan(sb, false), AssetStatus::Ok);
    CHECK_STATUS(svc.destroy(a), AssetStatus::Ok);  // no leases: drops cached span
    CHECK_STATUS(svc.stats().spanCount, 2);

    // A second, separately owned lease shares C's span. Both leases must be
    // released before its cache record can be evicted.
    AssetSpan sc2{};
    CHECK_STATUS(svc.prefetchSpan(c, 0, 512, sc2), AssetStatus::Ok);
    CHECK(sc2.data == sc.data);
    uint8_t* pinnedC = sc.data;

    // Compact: unleased B migrates into the hole; leased C NEVER moves.
    const uint32_t moved = svc.compactStaging();
    CHECK(moved >= 1);
    CHECK(sc.data == pinnedC);  // caller's leased pointer untouched
    CHECK(sc2.data == pinnedC);

    // B's cached copy survived migration byte-exact; C reads back fine.
    AssetSpan sb2{};
    CHECK_STATUS(svc.prefetchSpan(b, 0, 512, sb2), AssetStatus::Ok);
    CHECK(rampMatches(sb2.data, 512, 0x02));
    CHECK(sb2.data != pinnedC);
    CHECK(rampMatches(sc.data, 512, 0x03));
    CHECK_STATUS(svc.releaseSpan(sb2, false), AssetStatus::Ok);
    CHECK_STATUS(svc.releaseSpan(sc2, false), AssetStatus::Ok);

    // Releasing one of C's leases does not release the other. Only B can be
    // evicted, and freed reports newly reclaimed bytes rather than all free
    // space already present in the arena.
    uint32_t freed = 0;
    CHECK_STATUS(svc.evictCached(kStagingBytes, &freed), AssetStatus::StagingFull);
    CHECK_STATUS(freed, 512u);
    CHECK_STATUS(svc.stats().spanCount, 1);
    CHECK_STATUS(svc.stats().leasedSpanCount, 1);
    CHECK_STATUS(svc.stats().stagingFreeBytes, kStagingBytes - 512u);
    CHECK(svc.stats().stagingLargestFreeBytes < kStagingBytes);
    CHECK_STATUS(svc.destroy(c), AssetStatus::LeaseActive);
    CHECK(sc.data == pinnedC);
    CHECK(rampMatches(sc.data, 512, 0x03));

    // The final release permits exact full-capacity restoration.
    CHECK_STATUS(svc.releaseSpan(sc, false), AssetStatus::Ok);
    CHECK_STATUS(svc.evictCached(kStagingBytes, &freed), AssetStatus::Ok);
    CHECK_STATUS(freed, 512u);
    CHECK_STATUS(svc.stats().spanCount, 0);
    CHECK_STATUS(svc.stats().leasedSpanCount, 0);
    CHECK_STATUS(svc.stats().stagingFreeBytes, kStagingBytes);
    CHECK_STATUS(svc.stats().stagingLargestFreeBytes, kStagingBytes);
    CHECK_STATUS(svc.evictCached(kStagingBytes, &freed), AssetStatus::Ok);
    CHECK_STATUS(freed, 0u);

    // A consumer can use the entire coalesced arena, not just observe stats.
    AssetHandle full{};
    uint8_t fullPayload[kStagingBytes];
    fillRamp(fullPayload, sizeof(fullPayload), 0x61);
    CHECK_STATUS(svc.create(AssetClass::Texture, kStagingBytes, full), AssetStatus::Ok);
    CHECK_STATUS(svc.write(full, 0, fullPayload, sizeof(fullPayload)), AssetStatus::Ok);
    AssetSpan fullSpan{};
    CHECK_STATUS(svc.prefetchSpan(full, 0, kStagingBytes, fullSpan), AssetStatus::Ok);
    CHECK(rampMatches(fullSpan.data, kStagingBytes, 0x61));
    CHECK_STATUS(svc.releaseSpan(fullSpan, false), AssetStatus::Ok);
    CHECK_STATUS(svc.destroy(full), AssetStatus::Ok);

    CHECK_STATUS(svc.destroy(b), AssetStatus::Ok);
    CHECK_STATUS(svc.destroy(c), AssetStatus::Ok);
}

void testCachedRecordAdmission() {
    constexpr uint32_t stagingBytes =
        (GPU_MEM_MAX_SPANS + 1u) * GPU_MEM_SPAN_ALIGN_BYTES;
    alignas(GPU_MEM_SPAN_ALIGN_BYTES) uint8_t staging[stagingBytes];
    uint8_t payload[GPU_MEM_MAX_SPANS + 1u];
    fillRamp(payload, sizeof(payload), 0x21);
    RamBacking ram{gStore, kStoreBytes};
    AssetService svc;
    CHECK_STATUS(svc.bind(makeBacking(ram), staging, sizeof(staging)), AssetStatus::Ok);
    AssetHandle asset{};
    CHECK_STATUS(svc.create(AssetClass::Texture, sizeof(payload), asset), AssetStatus::Ok);
    CHECK_STATUS(svc.write(asset, 0, payload, sizeof(payload)), AssetStatus::Ok);

    // Fill records, not SRAM: every one-byte range is cached and dirty.
    for (uint32_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
        AssetSpan span{};
        CHECK_STATUS(svc.prefetchSpan(asset, i, 1, span), AssetStatus::Ok);
        CHECK(span.data[0] == payload[i]);
        payload[i] ^= 0x80;
        span.data[0] = payload[i];
        CHECK_STATUS(svc.releaseSpan(span, true), AssetStatus::Ok);
    }
    const AssetServiceStats before = svc.stats();
    CHECK_STATUS(before.spanCount, GPU_MEM_MAX_SPANS);
    CHECK_STATUS(before.leasedSpanCount, 0);
    CHECK(before.stagingLargestFreeBytes >= 1);

    // Record reclamation must preserve dirty data and report failed write-back.
    AssetSpan extra{};
    const uint32_t readsBefore = ram.readCalls;
    ram.failWrites = true;
    CHECK_STATUS(svc.prefetchSpan(asset, GPU_MEM_MAX_SPANS, 1, extra),
                 AssetStatus::BackingError);
    CHECK(extra.data == nullptr);
    CHECK_STATUS(ram.readCalls, readsBefore);
    CHECK_STATUS(svc.stats().spanCount, before.spanCount);
    CHECK_STATUS(svc.stats().stagingFreeBytes, before.stagingFreeBytes);

    // Once write-back succeeds, a cached record is reusable even though SRAM
    // already had room. The retry cannot remain stuck at the metadata bound.
    ram.failWrites = false;
    CHECK_STATUS(svc.prefetchSpan(asset, GPU_MEM_MAX_SPANS, 1, extra), AssetStatus::Ok);
    CHECK(extra.data[0] == payload[GPU_MEM_MAX_SPANS]);
    CHECK_STATUS(svc.stats().spanCount, GPU_MEM_MAX_SPANS);
    CHECK_STATUS(svc.stats().leasedSpanCount, 1);
    CHECK_STATUS(svc.releaseSpan(extra, false), AssetStatus::Ok);

    uint32_t freed = 0;
    CHECK_STATUS(svc.evictCached(stagingBytes, &freed), AssetStatus::Ok);
    CHECK_STATUS(freed, GPU_MEM_MAX_SPANS);
    CHECK_STATUS(svc.stats().spanCount, 0);
    CHECK_STATUS(svc.stats().stagingFreeBytes, stagingBytes);
    CHECK_STATUS(svc.stats().stagingLargestFreeBytes, stagingBytes);
    uint8_t persisted[sizeof(payload)];
    CHECK_STATUS(svc.read(asset, 0, persisted, sizeof(persisted)), AssetStatus::Ok);
    CHECK(std::memcmp(persisted, payload, sizeof(payload)) == 0);

    AssetSpan whole{};
    CHECK_STATUS(svc.prefetchSpan(asset, 0, sizeof(payload), whole), AssetStatus::Ok);
    CHECK(std::memcmp(whole.data, payload, sizeof(payload)) == 0);
    CHECK_STATUS(svc.releaseSpan(whole, false), AssetStatus::Ok);
    CHECK_STATUS(svc.destroy(asset), AssetStatus::Ok);
    CHECK_STATUS(svc.stats().stagingLargestFreeBytes, stagingBytes);
}

void testAlignedCacheAdmission() {
    constexpr uint32_t alignment = GPU_MEM_SPAN_ALIGN_BYTES;
    if constexpr (alignment > 1) {
        constexpr uint32_t stagingBytes = 3u * alignment;
        alignas(GPU_MEM_SPAN_ALIGN_BYTES) uint8_t staging[stagingBytes];
        uint8_t payload[stagingBytes];
        fillRamp(payload, sizeof(payload), 0x52);
        RamBacking ram{gStore, kStoreBytes};
        AssetService svc;
        CHECK_STATUS(svc.bind(makeBacking(ram), staging, sizeof(staging)), AssetStatus::Ok);
        AssetHandle asset{};
        CHECK_STATUS(svc.create(AssetClass::Texture, sizeof(payload), asset), AssetStatus::Ok);
        CHECK_STATUS(svc.write(asset, 0, payload, sizeof(payload)), AssetStatus::Ok);

        AssetSpan pinned{}, cached{}, admitted{};
        CHECK_STATUS(svc.prefetchSpan(asset, 0, 1, pinned), AssetStatus::Ok);
        CHECK_STATUS(svc.prefetchSpan(asset, 1, alignment + 1u, cached), AssetStatus::Ok);
        uint8_t* pinnedData = pinned.data;
        cached.data[alignment] = 0xD7;
        payload[alignment + 1u] = 0xD7;
        CHECK_STATUS(svc.releaseSpan(cached, true), AssetStatus::Ok);
        CHECK_STATUS(svc.stats().stagingLargestFreeBytes, alignment - 1u);

        // Each free run is large enough in bytes, but neither has room after
        // alignment. Reclaim the dirty cache and retry the actual allocation.
        CHECK_STATUS(svc.prefetchSpan(asset, alignment + 2u, alignment - 1u, admitted),
                     AssetStatus::Ok);
        CHECK(std::memcmp(admitted.data, payload + alignment + 2u, alignment - 1u) == 0);
        CHECK(pinned.data == pinnedData);
        CHECK(pinned.data[0] == payload[0]);
        CHECK_STATUS(svc.stats().leasedSpanCount, 2);
        CHECK_STATUS(svc.stats().stagingFreeBytes, 2u * alignment);

        CHECK_STATUS(svc.releaseSpan(pinned, false), AssetStatus::Ok);
        CHECK_STATUS(svc.releaseSpan(admitted, false), AssetStatus::Ok);
        uint32_t freed = 0;
        CHECK_STATUS(svc.evictCached(stagingBytes, &freed), AssetStatus::Ok);
        CHECK_STATUS(freed, alignment);
        CHECK_STATUS(svc.stats().stagingFreeBytes, stagingBytes);
        CHECK_STATUS(svc.stats().stagingLargestFreeBytes, stagingBytes);

        // Dirty bytes survived reclamation; coalescing permits a full-arena lease.
        AssetSpan whole{};
        CHECK_STATUS(svc.prefetchSpan(asset, 0, stagingBytes, whole), AssetStatus::Ok);
        CHECK(std::memcmp(whole.data, payload, sizeof(payload)) == 0);
        CHECK_STATUS(svc.releaseSpan(whole, false), AssetStatus::Ok);
        CHECK_STATUS(svc.evictCached(stagingBytes, &freed), AssetStatus::Ok);
        CHECK_STATUS(freed, stagingBytes);
        CHECK_STATUS(svc.destroy(asset), AssetStatus::Ok);
    }
}

// ─── 4. Absent device / SRAM-only semantics ─────────────────────────────────

void testAbsentBacking() {
    AssetService svc;
    AssetBacking none{};  // null ops, zero capacity = absent device
    CHECK_STATUS(svc.bind(none, gStaging, kStagingBytes), AssetStatus::Ok);
    CHECK(!svc.hasBacking());
    CHECK_STATUS(svc.capacityBytes(), 0);

    AssetHandle h{};
    CHECK_STATUS(svc.create(AssetClass::Mesh, 256, h), AssetStatus::NoBacking);
    CHECK(!h.isValid());
    const AssetServiceStats s = svc.stats();
    CHECK_STATUS(s.storeCapacityBytes, 0);
    CHECK_STATUS(s.assetCount, 0);
    CHECK_STATUS(s.spanCount, 0);
    // The SRAM staging arena is untouched by the absent device.
    CHECK_STATUS(s.stagingFreeBytes, kStagingBytes);

    CHECK_STATUS(svc.reset(), AssetStatus::Ok);  // releases nothing, holds nothing
    CHECK(!svc.hasBacking());

    // Valid backing but no staging arena: transfers work, spans refuse.
    RamBacking ram{gStore, kStoreBytes};
    AssetService svc2;
    CHECK_STATUS(svc2.bind(makeBacking(ram), nullptr, 0), AssetStatus::Ok);
    AssetHandle m{};
    CHECK_STATUS(svc2.create(AssetClass::Mesh, 128, m), AssetStatus::Ok);
    uint8_t buf[128];
    fillRamp(buf, sizeof(buf), 0x77);
    CHECK_STATUS(svc2.write(m, 0, buf, sizeof(buf)), AssetStatus::Ok);
    uint8_t back[128];
    CHECK_STATUS(svc2.read(m, 0, back, sizeof(back)), AssetStatus::Ok);
    CHECK(rampMatches(back, sizeof(back), 0x77));
    AssetSpan span{};
    CHECK_STATUS(svc2.prefetchSpan(m, 0, 128, span), AssetStatus::StagingFull);
    CHECK_STATUS(svc2.destroy(m), AssetStatus::Ok);

    // Null arena with nonzero size is a bad argument, state unchanged.
    AssetService svc3;
    CHECK_STATUS(svc3.bind(makeBacking(ram), nullptr, 4096),
                 AssetStatus::BadArgument);
    CHECK(!svc3.hasBacking());
}

// ─── 5. Generation quarantine (no 8-bit wrap aliasing) ──────────────────────

void testGenerationQuarantine() {
    RamBacking ram{gStore, kStoreBytes};
    AssetService svc;
    CHECK_STATUS(svc.bind(makeBacking(ram), gStaging, kStagingBytes), AssetStatus::Ok);

    // Cycle one slot through all 256 generations: at the wrap point the slot
    // quarantines instead of handing out generation 0 again.
    AssetHandle first{};
    CHECK_STATUS(svc.create(AssetClass::Mesh, 64, first), AssetStatus::Ok);
    const uint16_t slotIndex = first.index;
    const AssetToken staleToken = assetTokenFromHandle(first);  // generation 0
    AssetHandle cur = first;
    bool sawQuarantine = false;
    for (int i = 0; i < 300; ++i) {
        CHECK_STATUS(svc.destroy(cur), AssetStatus::Ok);
        AssetHandle next{};
        CHECK_STATUS(svc.create(AssetClass::Mesh, 64, next), AssetStatus::Ok);
        if (next.index != slotIndex) {
            sawQuarantine = true;
            CHECK_STATUS(svc.destroy(next), AssetStatus::Ok);
            break;
        }
        cur = next;
    }
    CHECK(sawQuarantine);  // quarantined at the wrap, never wrapped around

    // The original generation-0 token still resolves to nothing — no alias.
    AssetInfo info{};
    CHECK_STATUS(svc.info(assetHandleFromToken(staleToken), info),
                 AssetStatus::InvalidHandle);

    // Exhaust every slot: creation eventually reports NoSlots, and 24k
    // create/destroy cycles leave the store coalesced byte-exact.
    AssetHandle t{};
    size_t ops = 0;
    for (;;) {
        const AssetStatus st = svc.create(AssetClass::Mesh, 64, t);
        if (st == AssetStatus::NoSlots) break;
        CHECK_STATUS(st, AssetStatus::Ok);
        CHECK_STATUS(svc.destroy(t), AssetStatus::Ok);
        CHECK(++ops <= GPU_MEM_MAX_ASSETS * 256u);  // must terminate
    }
    CHECK_STATUS(svc.stats().storeFreeBytes, kStoreBytes);
    CHECK_STATUS(svc.stats().storeLargestFreeBytes, kStoreBytes);
    CHECK_STATUS(svc.stats().assetCount, 0);

    // reset() is the epoch boundary: quarantined slots become usable again.
    CHECK_STATUS(svc.reset(), AssetStatus::Ok);
    CHECK_STATUS(svc.create(AssetClass::Mesh, 64, t), AssetStatus::Ok);
    CHECK_STATUS(svc.destroy(t), AssetStatus::Ok);
}

void testQmiTimingAdmission() {
    const QmiPsramTiming timing = QmiPsramConfig{}.timing;
    QmiPsramTimingFields fields{};
    for (uint32_t hz : {75000000u, 100000000u, 125000000u, 150000000u,
                        240000000u, 250000000u, 288000000u, 300000000u,
                        336000000u}) {
        const bool admitted = qmiPsramComputeTiming(hz, timing, fields);
        // SDK RXDELAY=divisor cannot fit its 3-bit field for high profiles.
        CHECK(admitted == (hz <= 224000000u));
        if (!admitted) continue;
        CHECK(uint64_t(hz) <= uint64_t(fields.divisor) * 32000000u);
        CHECK(fields.divisor > 1u);
        CHECK(fields.rxDelay == fields.divisor && fields.rxDelay <= 7u);
        CHECK(fields.maxSelect >= 1u && fields.maxSelect <= 63u);
        CHECK(fields.minDeselect <= 31u);
        // Decode actual register timings, including a complete 40-SCK
        // overrun, and assert the part's real CS-low and CS-high limits.
        const uint64_t selectedSysCycles =
            uint64_t(fields.maxSelect) * 64u + uint64_t(fields.divisor) * 40u;
        CHECK(selectedSysCycles * 1000000000ull <= uint64_t(hz) * 7000u);
        const uint64_t deselectedSysCycles =
            fields.minDeselect + (fields.divisor + 1u) / 2u;
        CHECK(deselectedSysCycles * 1000000000ull >= uint64_t(hz) * 50u);
    }
    CHECK(qmiPsramComputeTiming(224000000u, timing, fields));
    CHECK(fields.divisor == 7u && fields.rxDelay == 7u);
    const QmiPsramTimingFields last = fields;
    CHECK(!qmiPsramComputeTiming(224000001u, timing, fields));
    CHECK(fields.divisor == last.divisor && fields.maxSelect == last.maxSelect);
    CHECK(!qmiPsramComputeTiming(0, timing, fields));
    CHECK(!qmiPsramComputeTiming(UINT32_MAX, timing, fields));
    QmiPsramTiming bad = timing;
    bad.maxClockHz = 0;
    CHECK(!qmiPsramComputeTiming(150000000u, bad, fields));
    bad = timing;
    bad.maxClockHz = 32000001u;
    CHECK(!qmiPsramComputeTiming(300000000u, bad, fields));
    bad = timing;
    bad.maxSelectNs = 7001u;
    CHECK(!qmiPsramComputeTiming(150000000u, bad, fields));
    bad = timing;
    bad.minDeselectNs = 49u;
    CHECK(!qmiPsramComputeTiming(150000000u, bad, fields));
    bad = timing;
    bad.minDeselectNs = UINT32_MAX;
    CHECK(!qmiPsramComputeTiming(150000000u, bad, fields));
    bad = timing;
    bad.maxSelectNs = 1u;
    CHECK(!qmiPsramComputeTiming(150000000u, bad, fields));
}

void runGroup(const char* name, void (*fn)()) {
    const int before = gFailures;
    fn();
    std::printf("%-28s %s\n", name, gFailures == before ? "ok" : "FAIL");
}

} // namespace

int main() {
    runGroup("identity/ranges/content", testIdentityAndRanges);
    runGroup("capacity/coalescing", testCapacityAndCoalescing);
    runGroup("leases/staging/writeback", testLeasesAndStaging);
    runGroup("compaction/migration", testCompactionMigration);
    runGroup("cached record admission", testCachedRecordAdmission);
    runGroup("aligned cache admission", testAlignedCacheAdmission);
    runGroup("absent device/SRAM-only", testAbsentBacking);
    runGroup("generation quarantine", testGenerationQuarantine);
    runGroup("QMI timing admission", testQmiTimingAdmission);

    std::printf("\n%d checks, %d failures\n", gChecks, gFailures);
    if (gFailures == 0) {
        std::printf("RESULT: PASS\n");
        return 0;
    }
    std::printf("RESULT: FAIL\n");
    return 1;
}
