// vnav - moving obstacles, deliberately kept OUT of the voxel grid.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include <vnav/Types.hpp>

namespace vnav
{
// A frozen set of moving obstacles, spatially hashed.
//
// WHY THIS IS NOT IN THE VOXEL GRID
//
// The obvious implementation - drop moving things into the chunks and invalidate - is wrong here
// for two independent reasons:
//
//   1. Chunks are shared_ptr<const Chunk>. That immutability is the entire concurrency model: a
//      worker walking a chunk cannot have it changed underneath it. Making chunks mutable to
//      track a car would trade a cheap correct design for a hard one.
//   2. InvalidateBox re-samples at CHUNK granularity, which is 16 voxels on a side. Re-voxelising
//      a whole chunk because a vehicle moved a metre is absurdly coarse, and it would have to
//      happen every frame, per vehicle.
//
// Worse, both would poison the bake: baked chunks describe static geometry, and mixing transient
// occupancy into them means the cached static answer is no longer reusable.
//
// So dynamic obstacles live here instead, consulted as a SECOND predicate after the clearance
// test. The expensive static answer stays cached forever and the cheap dynamic answer stays fresh,
// which is the correct split.
//
// THREADING: this is immutable once built and handed round by shared_ptr - the "publish an
// immutable snapshot behind one handle" shape. A writer builds a whole new set and swaps the
// handle; readers on worker threads keep whatever they pinned. That is also why this is a sibling
// of IWorldProbe rather than an extension of it: a probe may only be called on the game thread,
// whereas a snapshot can be read from anywhere.
class DynamicObstacleSet
{
public:
    DynamicObstacleSet() = default;

    // `aInflate` is folded into the stored boxes, so queries are a plain point/segment test rather
    // than paying for the agent radius on every lookup.
    DynamicObstacleSet(const std::vector<AABB>& aBoxes, float aInflate, float aCellSize)
        : m_cell(aCellSize > 0.1f ? aCellSize : 8.0f)
    {
        m_boxes.reserve(aBoxes.size());
        for (const AABB& b : aBoxes)
        {
            const Vec3 inf{aInflate, aInflate, aInflate};
            m_boxes.push_back(AABB{b.min - inf, b.max + inf});
        }

        for (size_t i = 0; i < m_boxes.size(); ++i)
        {
            const AABB& b = m_boxes[i];
            const IVec3 lo = Cell(b.min);
            const IVec3 hi = Cell(b.max);

            // A single huge obstacle would otherwise carpet the grid; cap the span it registers
            // in and let it fall back to the linear list.
            const int64_t span = static_cast<int64_t>(hi.x - lo.x + 1) * (hi.y - lo.y + 1) * (hi.z - lo.z + 1);
            if (span > 4096)
            {
                m_oversized.push_back(static_cast<int32_t>(i));
                continue;
            }

            for (int32_t z = lo.z; z <= hi.z; ++z)
            {
                for (int32_t y = lo.y; y <= hi.y; ++y)
                {
                    for (int32_t x = lo.x; x <= hi.x; ++x)
                    {
                        m_grid[PackChunkKey({x, y, z})].push_back(static_cast<int32_t>(i));
                    }
                }
            }
        }
    }

    bool Empty() const { return m_boxes.empty(); }
    size_t Count() const { return m_boxes.size(); }

    bool Blocked(const Vec3& aPoint) const
    {
        if (m_boxes.empty())
        {
            return false;
        }
        for (int32_t i : m_oversized)
        {
            if (m_boxes[static_cast<size_t>(i)].Contains(aPoint))
            {
                return true;
            }
        }
        auto it = m_grid.find(PackChunkKey(Cell(aPoint)));
        if (it == m_grid.end())
        {
            return false;
        }
        for (int32_t i : it->second)
        {
            if (m_boxes[static_cast<size_t>(i)].Contains(aPoint))
            {
                return true;
            }
        }
        return false;
    }

    // Sampled rather than analytic. The planner already walks segments voxel by voxel, so matching
    // that granularity keeps the two predicates consistent; an exact slab test would be able to
    // reject a segment the voxel walk accepted, which reads as nondeterminism at the call site.
    bool SegmentBlocked(const Vec3& aFrom, const Vec3& aTo, float aStep) const
    {
        if (m_boxes.empty())
        {
            return false;
        }
        const Vec3 d = aTo - aFrom;
        const float len = d.Length();
        const auto steps = std::max(1, static_cast<int32_t>(len / std::max(aStep, 0.1f)));
        for (int32_t i = 0; i <= steps; ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(steps);
            if (Blocked(aFrom + d * t))
            {
                return true;
            }
        }
        return false;
    }

private:
    IVec3 Cell(const Vec3& aP) const
    {
        return IVec3{static_cast<int32_t>(std::floor(aP.x / m_cell)), static_cast<int32_t>(std::floor(aP.y / m_cell)),
                     static_cast<int32_t>(std::floor(aP.z / m_cell))};
    }

    std::vector<AABB> m_boxes;
    std::unordered_map<uint64_t, std::vector<int32_t>> m_grid;
    std::vector<int32_t> m_oversized;
    float m_cell = 8.0f;
};

using DynamicObstacleSetPtr = std::shared_ptr<const DynamicObstacleSet>;
} // namespace vnav
