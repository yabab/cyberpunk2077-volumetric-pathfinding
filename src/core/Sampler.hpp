// vnav - turns physics probes into voxel occupancy. GAME THREAD ONLY.
#pragma once

#include <memory>

#include <vnav/Types.hpp>
#include <vnav/WorldProbe.hpp>

#include "core/Chunk.hpp"
#include "core/Volume.hpp"

namespace vnav
{
struct SampleStats
{
    int32_t probes = 0;
    bool budgetExhausted = false; // hit maxProbesPerChunk; unresolved space was marked solid
};

// Voxelises one chunk by adaptive octree probing.
//
// Only emptiness is hereditary: an overlap test that reports "clear" proves every voxel inside
// the box is clear, but a test that reports "hit" proves nothing about the interior. So the
// recursion terminates on empty, and on hitting a depth/budget limit it marks the node SOLID.
// Every failure mode therefore over-blocks rather than under-blocks: an agent may fail to find a
// tight path, but is never routed through geometry.
//
// MUST be called from the game thread - it calls IWorldProbe.
//
// Runs to completion in one call, so a worst-case chunk is a worst-case single-frame cost. That is
// correct for the synchronous query paths, which are already blocking their caller by definition.
// The asynchronous pump uses ChunkSampler below instead, so one expensive chunk cannot stall a
// frame.
std::shared_ptr<const Chunk> SampleChunk(const Volume& aVolume, const IVec3& aChunkCoord, IWorldProbe& aProbe,
                                         SampleStats& aStats);

// The same voxelisation, suspended and resumed across frames.
//
// Why this exists: sampling one chunk was atomic, so maxProbesPerChunk doubled as the worst-case
// game-thread cost of a single chunk - and that coupling is what kept the budget pinned at 2048,
// which is BELOW the ~4681 probes a fully-refined chunk needs. The result was that narrow interior
// space silently resolved to solid (see the warning on VolumeConfig::maxProbesPerChunk).
// Splitting the work across pumps decouples the two: the per-chunk budget can now be raised for
// correctness without turning into a frame spike.
//
// A partially-sampled chunk is NEVER published - Take() only yields once Step() reports done - so
// a search can never observe half a map. It simply keeps parking on residency, which it already
// knows how to do.
class ChunkSampler
{
public:
    ChunkSampler(const Volume& aVolume, const IVec3& aChunkCoord);

    // Advance by at most aProbeSlice probes. Returns true when the chunk is complete.
    bool Step(IWorldProbe& aProbe, int32_t aProbeSlice);

    bool Done() const { return m_done; }

    // Valid only once Step has returned true. Collapses uniform results onto the shared
    // Empty/Solid singletons, exactly as the atomic path does.
    std::shared_ptr<const Chunk> Take();

    int32_t Probes() const { return m_probes; }
    bool BudgetExhausted() const { return m_exhausted; }

private:
    struct Node
    {
        int32_t lx, ly, lz, size;
    };

    void BeginLevel();
    bool Overlaps(IWorldProbe& aProbe, const Node& aNode, float aInflation) const;
    void MarkOccupied(const Node& aNode, bool aHardBlocked);

    std::shared_ptr<Chunk> m_chunk;
    std::vector<Node> m_current;
    std::vector<Node> m_next;
    size_t m_cursor = 0;

    IVec3 m_originVoxel{};
    float m_voxelSize = 1.0f;
    float m_hardInflate = 0.0f;
    float m_softInflate = 0.0f;
    CollisionMask m_mask = CollisionMask::World;
    int32_t m_maxDepth = 4;
    int32_t m_probeBudget = 2048;
    uint8_t m_tightValue = 0;

    int32_t m_depth = 0;
    int32_t m_probes = 0;
    bool m_leafLevel = false;
    bool m_done = false;
    bool m_exhausted = false;
};
} // namespace vnav
