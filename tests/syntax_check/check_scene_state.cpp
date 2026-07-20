// SceneState scene-heap migration — native functional check.
//
// Compiles the firmware's scene_state.h with the ProtoGC desktop backend and
// RUNS it: alloc/free round trips, Reset() with live allocations (leak
// check), the SCENE_HEAP_MAX_BYTES cap, and the per-frame bump pool. This is
// the desktop gate for the bump-pool → HeapAllocator migration; it needs no
// Pico SDK or hardware.
//
// Build/run via tests/syntax_check/run_scene_check.sh.

#include "../src/scene_state.h"

#include <cstdio>
#include <cstring>

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_failures;
}

} // namespace

int main() {
    static SceneState scene;  // static: the object is large (slot arrays)

    scene.InitSceneHeap();

    // ─── Alloc/free round trips with exact accounting ───────────────────
    {
        const size_t used0 = scene.sceneHeap.stats().usedBytes;

        PglVec3* v = scene.AllocVertices(3);
        check(v != nullptr, "alloc: vertices from scene heap");
        check(scene.sceneHeap.stats().usedBytes >= used0 + 3 * sizeof(PglVec3),
              "alloc: used bytes grew by the vertex payload (+header)");

        const size_t usedAfterAlloc = scene.sceneHeap.stats().usedBytes;
        scene.FreeVertices(v);
        check(scene.sceneHeap.stats().usedBytes < usedAfterAlloc,
              "free: true random free returns bytes immediately");

        PglIndex3* idx = scene.AllocIndices(4);
        PglVec2*   uv  = scene.AllocUVVertices(2);
        PglIndex3* uvi = scene.AllocUVIndices(1);
        uint8_t*   tex = scene.AllocTexturePixels(1024);
        PglVec2*   lay = scene.AllocLayoutCoords(8);
        check(idx && uv && uvi && tex && lay, "alloc: all five resource kinds");
        std::memset(tex, 0xA5, 1024);
        check(tex[0] == 0xA5 && tex[1023] == 0xA5, "alloc: texture bytes writable");

        // Arbitrary destroy ORDER — the exact case the bump pools couldn't do.
        scene.FreeIndices(idx);
        scene.FreeTexturePixels(tex);
        scene.FreeUVVertices(uv);
        scene.FreeUVIndices(uvi);
        scene.FreeLayoutCoords(lay);
        const size_t usedEnd = scene.sceneHeap.stats().usedBytes;
        check(usedEnd <= used0 + 64, // small header slack, no payload left
              "free: arbitrary-order frees coalesce back (no fragmentation)");
    }

    // ─── Reset() with live allocations must not leak ────────────────────
    {
        // Simulate live resources via the slot arrays the Reset() walk uses.
        scene.meshes[0].active    = true;
        scene.meshes[0].vertices  = scene.AllocVertices(8);
        scene.meshes[0].indices   = scene.AllocIndices(4);
        scene.meshes[0].uvVertices = scene.AllocUVVertices(8);
        scene.meshes[0].uvIndices = scene.AllocUVIndices(4);
        scene.textures[1].active = true;
        scene.textures[1].pixels = scene.AllocTexturePixels(2048);
        scene.pixelLayouts[2].active = true;
        scene.pixelLayouts[2].coords = scene.AllocLayoutCoords(16);

        const size_t usedLive = scene.sceneHeap.stats().usedBytes;
        check(usedLive > 0, "reset: live allocations present before Reset()");
        check(scene.meshes[0].vertices && scene.textures[1].pixels &&
              scene.pixelLayouts[2].coords, "reset: slots wired");

        scene.Reset();
        check(scene.sceneHeap.stats().usedBytes == 0,
              "reset: heap fully drained after Reset() (no leaked slots)");
        check(!scene.meshes[0].active && scene.meshes[0].vertices == nullptr,
              "reset: slots zeroed");
    }

    // ─── Cap: SCENE_HEAP_MAX_BYTES is enforced ──────────────────────────
    {
        // Allocate until failure; must happen at or below the cap and must
        // fail cleanly (nullptr), never past it.
        size_t total = 0;
        void* ptrs[64] = {};
        size_t n = 0;
        while (n < 64) {
            void* p = scene.SceneHeapAlloc(16 * 1024);
            if (!p) break;
            ptrs[n++] = p;
            total += 16 * 1024;
        }
        check(n > 0 && scene.sceneHeap.stats().segmentBytes <= GpuConfig::SCENE_HEAP_MAX_BYTES,
              "cap: segment growth stops at SCENE_HEAP_MAX_BYTES");
        check(scene.SceneHeapAlloc(16 * 1024) == nullptr,
              "cap: over-cap allocation fails cleanly");
        for (size_t i = 0; i < n; ++i) scene.sceneHeap.deallocate(ptrs[i]);
        scene.Reset();  // drain remaining state; also exercises empty-heap Reset
    }

    // ─── Per-frame bump pool is still a static bump pool ────────────────
    {
        scene.BeginFrame(1);
        PglVec3* a = scene.AllocFrameVertices(4);
        PglVec3* b = scene.AllocFrameVertices(4);
        check(a && b && b == a + 4, "frame pool: bump layout preserved");
        scene.BeginFrame(2);
        PglVec3* c = scene.AllocFrameVertices(4);
        check(c == a, "frame pool: BeginFrame resets the bump pointer");
    }

    scene.PrintPoolUsage();  // eyeball check of the diagnostics path

    std::printf("\n");
    if (g_failures == 0) {
        std::printf("RESULT: PASS (scene heap migration)\n");
        return 0;
    }
    std::printf("RESULT: FAIL (%d failing check(s))\n", g_failures);
    return 1;
}
