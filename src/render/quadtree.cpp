/**
 * @file quadtree.cpp
 * @brief QuadTree spatial index — implementation.
 *
 * Static node pool (no malloc).  Subdivision uses GpuConfig limits for
 * max depth and max entities per leaf.
 */

#include "quadtree.h"
#include "../gpu_config.h"

#include <cstring>

// ─── Public API ─────────────────────────────────────────────────────────────

void QuadTree::Initialize(uint16_t screenWidth, uint16_t screenHeight) {
    Clear();
    rootIndex = AllocNode(0.0f, 0.0f,
                          static_cast<float>(screenWidth),
                          static_cast<float>(screenHeight));
}

void QuadTree::Clear() {
    nodeCount = 0;
    std::memset(nodes, 0, sizeof(nodes));
}

void QuadTree::Insert(TriHandle handle, const PglMath::AABB2D& bounds) {
    if (nodeCount == 0) return;
    InsertInto(rootIndex, handle, bounds, 0);
}

uint16_t QuadTree::Query(float minX, float minY, float maxX, float maxY,
                         TriHandle* outHandles, uint16_t maxHandles) const {
    uint16_t count = 0;
    if (nodeCount > 0) {
        QueryNode(rootIndex, minX, minY, maxX, maxY,
                  outHandles, maxHandles, count);
    }
    return count;
}

// ─── Internal ───────────────────────────────────────────────────────────────

uint16_t QuadTree::AllocNode(float minX, float minY, float maxX, float maxY) {
    if (nodeCount >= GpuConfig::QUADTREE_MAX_NODES) return 0;
    uint16_t idx = nodeCount++;
    QuadNode& n = nodes[idx];
    n.minX = minX;
    n.minY = minY;
    n.maxX = maxX;
    n.maxY = maxY;
    n.entityCount = 0;
    n.isLeaf = true;
    n.children[0] = n.children[1] = n.children[2] = n.children[3] = 0;
    return idx;
}

void QuadTree::Subdivide(uint16_t nodeIdx) {
    QuadNode& n = nodes[nodeIdx];
    float midX = (n.minX + n.maxX) * 0.5f;
    float midY = (n.minY + n.maxY) * 0.5f;

    n.children[0] = AllocNode(n.minX, n.minY, midX,   midY);    // top-left
    n.children[1] = AllocNode(midX,   n.minY, n.maxX,  midY);    // top-right
    n.children[2] = AllocNode(n.minX, midY,   midX,    n.maxY);  // bottom-left
    n.children[3] = AllocNode(midX,   midY,   n.maxX,  n.maxY);  // bottom-right

    n.isLeaf = false;

    // Re-insert existing entities into children
    for (uint8_t i = 0; i < n.entityCount; ++i) {
        // For simplicity, insert into ALL children whose bounds overlap.
        // A proper implementation would store per-entity AABBs — TODO(M4).
        for (int c = 0; c < 4; ++c) {
            if (n.children[c] != 0) {
                QuadNode& child = nodes[n.children[c]];
                child.entities[child.entityCount++] = n.entities[i];
            }
        }
    }
    n.entityCount = 0;
}

void QuadTree::InsertInto(uint16_t nodeIdx, TriHandle handle,
                          const PglMath::AABB2D& bounds, uint8_t depth) {
    QuadNode& n = nodes[nodeIdx];

    // Check overlap
    if (!Overlaps(n.minX, n.minY, n.maxX, n.maxY,
                  bounds.minX, bounds.minY, bounds.maxX, bounds.maxY)) {
        return;
    }

    if (n.isLeaf) {
        if (n.entityCount < GpuConfig::QUADTREE_MAX_ENTITIES ||
            depth >= GpuConfig::QUADTREE_MAX_DEPTH) {
            // Still room, or max depth reached — store here
            if (n.entityCount < GpuConfig::QUADTREE_MAX_ENTITIES) {
                n.entities[n.entityCount++] = handle;
            }
            return;
        }
        // Leaf full — subdivide
        Subdivide(nodeIdx);
    }

    // Insert into overlapping children
    for (int c = 0; c < 4; ++c) {
        if (n.children[c] != 0) {
            InsertInto(n.children[c], handle, bounds, depth + 1);
        }
    }
}

void QuadTree::QueryNode(uint16_t nodeIdx,
                         float qMinX, float qMinY, float qMaxX, float qMaxY,
                         TriHandle* outHandles, uint16_t maxHandles,
                         uint16_t& count) const {
    const QuadNode& n = nodes[nodeIdx];

    if (!Overlaps(n.minX, n.minY, n.maxX, n.maxY,
                  qMinX, qMinY, qMaxX, qMaxY)) {
        return;
    }

    // Collect entities stored in this node
    for (uint8_t i = 0; i < n.entityCount && count < maxHandles; ++i) {
        outHandles[count++] = n.entities[i];
    }

    // Recurse into children
    if (!n.isLeaf) {
        for (int c = 0; c < 4; ++c) {
            if (n.children[c] != 0 && count < maxHandles) {
                QueryNode(n.children[c], qMinX, qMinY, qMaxX, qMaxY,
                          outHandles, maxHandles, count);
            }
        }
    }
}

bool QuadTree::Overlaps(float aMinX, float aMinY, float aMaxX, float aMaxY,
                        float bMinX, float bMinY, float bMaxX, float bMaxY) {
    return aMinX <= bMaxX && aMaxX >= bMinX &&
           aMinY <= bMaxY && aMaxY >= bMinY;
}
