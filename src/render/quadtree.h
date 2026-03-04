/**
 * @file quadtree.h
 * @brief Spatial index for projected 2D triangles.
 *
 * Accelerates rasterization by allowing each core to quickly find which
 * triangles overlap a given Y band (or individual tile).
 *
 * The QuadTree is rebuilt every frame by Rasterizer::PrepareFrame() on Core 0,
 * then read concurrently by both cores during RasterizeRange().
 *
 * Node pool is statically allocated (GpuConfig::QUADTREE_MAX_NODES).
 */

#pragma once

#include <cstdint>
#include "../math/pgl_math.h"

/// Handle into the Triangle2D pool (set by Rasterizer during PrepareFrame).
using TriHandle = uint16_t;

/// A QuadTree node covering a rectangular region of screen space.
struct QuadNode {
    float minX, minY, maxX, maxY;       // bounds
    uint16_t children[4] = {};           // child node indices (0 = none)
    TriHandle entities[16] = {};         // triangle handles in this node
    uint8_t  entityCount = 0;
    bool     isLeaf      = true;
};

class QuadTree {
public:
    /// Initialize the tree to cover the full screen.
    void Initialize(uint16_t screenWidth, uint16_t screenHeight);

    /// Clear all nodes.  Call once per frame before inserting triangles.
    void Clear();

    /// Insert a projected triangle (by handle) with the given 2D AABB.
    void Insert(TriHandle handle, const PglMath::AABB2D& bounds);

    /// Query: collect all triangle handles whose AABB overlaps the given rect.
    /// Returns number of handles written to `outHandles` (up to maxHandles).
    uint16_t Query(float minX, float minY, float maxX, float maxY,
                   TriHandle* outHandles, uint16_t maxHandles) const;

    /// Current node count (diagnostic).
    uint16_t GetNodeCount() const { return nodeCount; }

private:
    QuadNode nodes[4096];   // GpuConfig::QUADTREE_MAX_NODES — static alloc
    uint16_t nodeCount = 0;
    uint16_t rootIndex = 0;

    uint16_t AllocNode(float minX, float minY, float maxX, float maxY);
    void     Subdivide(uint16_t nodeIdx);
    void     InsertInto(uint16_t nodeIdx, TriHandle handle,
                        const PglMath::AABB2D& bounds, uint8_t depth);
    void     QueryNode(uint16_t nodeIdx,
                       float qMinX, float qMinY, float qMaxX, float qMaxY,
                       TriHandle* outHandles, uint16_t maxHandles,
                       uint16_t& count) const;

    static bool Overlaps(float aMinX, float aMinY, float aMaxX, float aMaxY,
                         float bMinX, float bMinY, float bMaxX, float bMaxY);
};
