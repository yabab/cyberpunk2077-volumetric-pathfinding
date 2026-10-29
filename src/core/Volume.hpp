// vnav - a NavVolume: one agent profile, one voxel grid, one chunk store.
#pragma once

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

#include <vnav/BakedNav.hpp>
#include <vnav/Config.hpp>
#include <vnav/Types.hpp>

#include "core/ChunkStore.hpp"
#include "core/DynamicObstacles.hpp"

namespace vnav
{
class Volume
{
public:
    Volume(std::string aName, const VolumeConfig& aConfig)
        : m_name(std::move(aName))
        , m_config(aConfig)
        , m_blockThreshold(ClearanceThreshold(aConfig.agentRadius, aConfig.voxelSize))
        , m_tightThreshold(ClearanceThreshold(aConfig.agentRadius + aConfig.margin, aConfig.voxelSize))
    {
    }

    const std::string& Name() const { return m_name; }
    const VolumeConfig& Config() const { return m_config; }

    // Clearance below this many voxels means the agent does not fit at all.
    uint8_t BlockThreshold() const { return m_blockThreshold; }

    // Clearance below this means the agent fits but is squeezing, which the planner penalises.
    uint8_t TightThreshold() const { return m_tightThreshold; }

    // Voxels of clearance an agent of the given radius needs.
    //
    // Clearance is the distance to the nearest solid voxel's CENTRE, but that voxel's surface
    // reaches half a voxel closer, so the half-voxel is added back here. Without it a
    // radius-2m agent at 1m voxels would be cleared to fly 1.5m from a wall.
    //
    // Rounded up, so a non-integral radius errs toward blocking. Clamped to at least 1 so that a
    // solid voxel (clearance 0) is always blocked, even for a zero-radius agent.
    static uint8_t ClearanceThreshold(float aRadius, float aVoxelSize)
    {
        if (!(aVoxelSize > 0.0f))
        {
            return 1;
        }
        const float voxels = std::ceil(aRadius / aVoxelSize + 0.5f);
        if (voxels < 1.0f)
        {
            return 1;
        }
        if (voxels >= static_cast<float>(kMaxClearance))
        {
            return kMaxClearance;
        }
        return static_cast<uint8_t>(voxels);
    }

    // Inverse of ClearanceThreshold: the free radius, in metres, that a stored clearance value
    // represents. Solving `voxels = ceil(r/s + 0.5)` for r gives `(voxels - 0.5) * s`, dropping
    // the ceil - so this is the LARGEST radius guaranteed to fit, which is the conservative
    // direction for a caller sizing a manoeuvre against it.
    float ClearanceMetres(uint8_t aClearance) const
    {
        const float r = (static_cast<float>(aClearance) - 0.5f) * m_config.voxelSize;
        return r > 0.0f ? r : 0.0f;
    }

    ChunkStore& Store() { return m_store; }
    const ChunkStore& Store() const { return m_store; }

    // --- coordinate transforms -----------------------------------------------------------
    // Voxel v spans world [v * voxelSize, (v+1) * voxelSize) on every axis.

    IVec3 WorldToVoxel(const Vec3& aWorld) const
    {
        const float s = m_config.voxelSize;
        return {static_cast<int32_t>(std::floor(aWorld.x / s)), static_cast<int32_t>(std::floor(aWorld.y / s)),
                static_cast<int32_t>(std::floor(aWorld.z / s))};
    }

    Vec3 VoxelCenter(const IVec3& aVoxel) const
    {
        const float s = m_config.voxelSize;
        return {(static_cast<float>(aVoxel.x) + 0.5f) * s, (static_cast<float>(aVoxel.y) + 0.5f) * s,
                (static_cast<float>(aVoxel.z) + 0.5f) * s};
    }

    static IVec3 VoxelToChunk(const IVec3& aVoxel)
    {
        return {FloorDiv(aVoxel.x, kChunkDim), FloorDiv(aVoxel.y, kChunkDim), FloorDiv(aVoxel.z, kChunkDim)};
    }

    static IVec3 VoxelLocal(const IVec3& aVoxel)
    {
        return {FloorMod(aVoxel.x, kChunkDim), FloorMod(aVoxel.y, kChunkDim), FloorMod(aVoxel.z, kChunkDim)};
    }

    // World-space AABB of a chunk, used by the sampler and by box invalidation.
    AABB ChunkBounds(const IVec3& aChunk) const
    {
        const float s = m_config.voxelSize * static_cast<float>(kChunkDim);
        const Vec3 lo{static_cast<float>(aChunk.x) * s, static_cast<float>(aChunk.y) * s,
                      static_cast<float>(aChunk.z) * s};
        return {lo, lo + Vec3{s, s, s}};
    }

    bool InBounds(const Vec3& aWorld) const { return m_config.bounds.Contains(aWorld); }
    bool InBounds(const IVec3& aVoxel) const { return m_config.bounds.Contains(VoxelCenter(aVoxel)); }

    // --- baked static geometry ---------------------------------------------------------------
    //
    // A bake, when present, is the authority for the static world: it sees geometry the physics
    // scene has not streamed in, so it is both faster and more correct at range. The probe stays
    // on as the layer that knows about doors, vehicles and destruction.

    void AttachBake(std::shared_ptr<BakedNav> aBaked) { m_baked = std::move(aBaked); }
    const std::shared_ptr<BakedNav>& Baked() const { return m_baked; }

    // Chunks the world has changed under. Once marked, a chunk is served by probing only - the
    // bake describes the world as shipped, and would otherwise keep undoing the change.
    void MarkProbeOnly(uint64_t aKey)
    {
        std::lock_guard<std::mutex> lock(m_probeOnlyMutex);
        m_probeOnly.insert(aKey);
    }

    bool IsProbeOnly(uint64_t aKey) const
    {
        std::lock_guard<std::mutex> lock(m_probeOnlyMutex);
        return m_probeOnly.find(aKey) != m_probeOnly.end();
    }

    void ClearProbeOnly()
    {
        std::lock_guard<std::mutex> lock(m_probeOnlyMutex);
        m_probeOnly.clear();
    }

    size_t ProbeOnlyCount() const
    {
        std::lock_guard<std::mutex> lock(m_probeOnlyMutex);
        return m_probeOnly.size();
    }

    // --- dynamic obstacles -----------------------------------------------------------------
    // Publish a whole new snapshot rather than mutating one in place, so a search already holding
    // the previous handle keeps a coherent view for its entire run. Game thread writes; workers
    // only ever read through a handle they pinned.
    void SetDynamicObstacles(DynamicObstacleSetPtr aSet)
    {
        std::lock_guard lock(m_dynamicMutex);
        m_dynamic = std::move(aSet);
    }

    DynamicObstacleSetPtr DynamicObstacles() const
    {
        std::lock_guard lock(m_dynamicMutex);
        return m_dynamic;
    }

    // --- telemetry -------------------------------------------------------------------------
    void AddProbes(int64_t n) { m_totalProbes.fetch_add(n, std::memory_order_relaxed); }
    void AddChunksSampled(int64_t n) { m_totalChunksSampled.fetch_add(n, std::memory_order_relaxed); }
    int64_t TotalProbes() const { return m_totalProbes.load(std::memory_order_relaxed); }
    int64_t TotalChunksSampled() const { return m_totalChunksSampled.load(std::memory_order_relaxed); }

private:
    std::string m_name;
    VolumeConfig m_config;
    uint8_t m_blockThreshold;
    uint8_t m_tightThreshold;
    ChunkStore m_store;

    std::shared_ptr<BakedNav> m_baked;

    mutable std::mutex m_probeOnlyMutex;
    std::unordered_set<uint64_t> m_probeOnly;

    mutable std::mutex m_dynamicMutex;
    DynamicObstacleSetPtr m_dynamic;

    std::atomic<int64_t> m_totalProbes{0};
    std::atomic<int64_t> m_totalChunksSampled{0};
};

// A job's private, pinned window onto a volume.
//
// Every chunk this view touches is held by shared_ptr for the view's whole lifetime. That gives
// the job an immutable, self-consistent picture of the world: invalidation and LRU eviction can
// run freely on the game thread while a search is mid-flight, and the search will neither see a
// torn update nor read freed memory. It is the same discipline the redscript side of this
// workspace arrived at the hard way - never walk shared mutable state, hold one handle to
// something immutable.
class PinnedView
{
public:
    PinnedView(Volume& aVolume, uint64_t aTick)
        : m_volume(aVolume)
        , m_tick(aTick)
        , m_blockThreshold(aVolume.BlockThreshold())
        , m_tightThreshold(aVolume.TightThreshold())
        , m_dynamic(aVolume.DynamicObstacles())
    {
        // Pinned once, for the same reason chunks are: a search must not see obstacles appear or
        // vanish halfway through, or it can produce a path that was never valid at any instant.
    }

    // nullptr means "not resident" - the caller must request it and suspend.
    const Chunk* GetChunk(const IVec3& aChunkCoord)
    {
        const uint64_t key = PackChunkKey(aChunkCoord);
        if (key == m_lastKey && m_lastChunk)
        {
            return m_lastChunk; // A* has very high spatial locality; this hits most of the time.
        }

        auto it = m_pinned.find(key);
        if (it == m_pinned.end())
        {
            auto chunk = m_volume.Store().Get(key, m_tick);
            if (!chunk)
            {
                return nullptr;
            }
            it = m_pinned.emplace(key, std::move(chunk)).first;
        }

        m_lastKey = key;
        m_lastChunk = it->second.get();
        return m_lastChunk;
    }

    bool IsChunkResident(const IVec3& aChunkCoord) { return GetChunk(aChunkCoord) != nullptr; }

    // Occupancy lookups. `aMissing` is set when the voxel's chunk is not resident, in which case
    // the return value is the conservative answer (blocked).
    bool IsBlocked(const IVec3& aVoxel, bool& aMissing)
    {
        const Chunk* c = GetChunk(Volume::VoxelToChunk(aVoxel));
        if (!c)
        {
            aMissing = true;
            return true;
        }
        aMissing = false;
        const IVec3 l = Volume::VoxelLocal(aVoxel);
        return c->ClearanceLocal(Chunk::LocalIndex(l.x, l.y, l.z)) < m_blockThreshold;
    }

    bool IsTight(const IVec3& aVoxel, bool& aMissing)
    {
        const Chunk* c = GetChunk(Volume::VoxelToChunk(aVoxel));
        if (!c)
        {
            aMissing = true;
            return true;
        }
        aMissing = false;
        const IVec3 l = Volume::VoxelLocal(aVoxel);
        return c->ClearanceLocal(Chunk::LocalIndex(l.x, l.y, l.z)) < m_tightThreshold;
    }

    // Raw clearance in voxels, rather than the two threshold comparisons above. The planner
    // already pays for this lookup on every node it tests; exposing the value is what lets a
    // result carry "how much room is there here" instead of the caller re-querying for it.
    // 0 on a non-resident chunk, matching the conservative answer IsBlocked gives.
    uint8_t Clearance(const IVec3& aVoxel, bool& aMissing)
    {
        const Chunk* c = GetChunk(Volume::VoxelToChunk(aVoxel));
        if (!c)
        {
            aMissing = true;
            return 0;
        }
        aMissing = false;
        const IVec3 l = Volume::VoxelLocal(aVoxel);
        return c->ClearanceLocal(Chunk::LocalIndex(l.x, l.y, l.z));
    }

    // The obstacle snapshot this view is pinned to. Null when none was ever published.
    const DynamicObstacleSet* Dynamic() const { return m_dynamic.get(); }

    size_t PinnedCount() const { return m_pinned.size(); }

private:
    Volume& m_volume;
    uint64_t m_tick;
    uint8_t m_blockThreshold;
    uint8_t m_tightThreshold;
    DynamicObstacleSetPtr m_dynamic;

    std::unordered_map<uint64_t, std::shared_ptr<const Chunk>> m_pinned;

    uint64_t m_lastKey = ~0ull;
    const Chunk* m_lastChunk = nullptr;
};
} // namespace vnav
