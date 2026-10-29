#include "core/Planner.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>

namespace vnav
{
namespace
{
constexpr std::array<IVec3, 26> MakeNeighbours()
{
    std::array<IVec3, 26> r{};
    int32_t i = 0;
    for (int32_t dz = -1; dz <= 1; ++dz)
    {
        for (int32_t dy = -1; dy <= 1; ++dy)
        {
            for (int32_t dx = -1; dx <= 1; ++dx)
            {
                if (dx != 0 || dy != 0 || dz != 0)
                {
                    r[static_cast<size_t>(i++)] = IVec3{dx, dy, dz};
                }
            }
        }
    }
    return r;
}

constexpr auto kNeighbours = MakeNeighbours();

// Cap on how far ahead the string-pull looks. Without it the pass is O(n^2) line-of-sight checks
// on long paths, which dwarfs the search itself.
constexpr size_t kStringPullWindow = 48;

constexpr size_t kMaxPrefetchChunks = 512;

Vec3 CatmullRom(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t)
{
    const float t2 = t * t;
    const float t3 = t2 * t;
    return (p1 * 2.0f + (p2 - p0) * t + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2 +
            (p1 * 3.0f - p0 - p2 * 3.0f + p3) * t3) *
           0.5f;
}
} // namespace

Planner::Planner(Volume& aVolume, const PathQuery& aQuery)
    : m_volume(aVolume)
    , m_query(aQuery)
{
    const float s = m_volume.Config().voxelSize;
    m_snapVoxelRadius = static_cast<int32_t>(std::ceil(m_query.snapRadius / s));
    m_snapVoxelRadius = std::clamp(m_snapVoxelRadius, 0, 32);

    // 4D is meaningless for a sphere: every yaw would test identically, so it would just multiply
    // the state space by yawBuckets for nothing. Silently fall back to 3D rather than burn it.
    if (m_query.yawBuckets >= 4 && m_query.agentLength > 0.0f)
    {
        m_yawBuckets = std::min(m_query.yawBuckets, kMaxYawBuckets);
    }
}

// ---------------------------------------------------------------------------------------------
// Hierarchical guidance
// ---------------------------------------------------------------------------------------------

void Planner::BuildCorridor()
{
    if (!m_query.coarseGuidance)
    {
        return;
    }
    const auto& baked = m_volume.Baked();
    if (!baked)
    {
        return; // nothing to build a coarse graph from
    }

    const IVec3 startChunk = Volume::VoxelToChunk(m_startVoxel);
    const IVec3 goalChunk = Volume::VoxelToChunk(m_goalVoxel);
    if (startChunk == goalChunk)
    {
        return; // same chunk: the fine search is already local, guidance would only cost time
    }

    struct CoarseRec
    {
        float g;
        uint64_t parent;
        bool closed;
    };
    struct CoarseItem
    {
        float f;
        uint64_t key;
    };

    const auto heuristic = [&](const IVec3& c)
    {
        const auto dx = static_cast<float>(c.x - goalChunk.x);
        const auto dy = static_cast<float>(c.y - goalChunk.y);
        const auto dz = static_cast<float>(c.z - goalChunk.z);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    std::unordered_map<uint64_t, CoarseRec> nodes;
    std::vector<CoarseItem> open;
    const auto cmp = [](const CoarseItem& a, const CoarseItem& b) { return a.f > b.f; };

    const uint64_t startKey = PackChunkKey(startChunk);
    const uint64_t goalKey = PackChunkKey(goalChunk);

    nodes[startKey] = CoarseRec{0.0f, startKey, false};
    open.push_back(CoarseItem{heuristic(startChunk), startKey});
    std::push_heap(open.begin(), open.end(), cmp);

    bool reached = false;
    while (!open.empty() && m_coarseExpansions < m_query.maxCoarseExpansions)
    {
        std::pop_heap(open.begin(), open.end(), cmp);
        const CoarseItem top = open.back();
        open.pop_back();

        auto it = nodes.find(top.key);
        if (it == nodes.end() || it->second.closed)
        {
            continue;
        }
        it->second.closed = true;
        ++m_coarseExpansions;

        if (top.key == goalKey)
        {
            reached = true;
            break;
        }

        const IVec3 c = UnpackChunkKey(top.key);
        const float g = it->second.g;

        for (const IVec3& off : kNeighbours)
        {
            const IVec3 n{c.x + off.x, c.y + off.y, c.z + off.z};

            // Solid is the only hard block. Mixed chunks stay open at this resolution precisely
            // because 16 m is too coarse to tell - resolving them is the fine search's job.
            if (baked->CoarseState(n) == ChunkState::Solid)
            {
                continue;
            }
            // Do not route the coarse plan outside the volume it is guiding.
            if (!m_volume.InBounds(IVec3{n.x * kChunkDim + kChunkDim / 2, n.y * kChunkDim + kChunkDim / 2,
                                         n.z * kChunkDim + kChunkDim / 2}))
            {
                continue;
            }

            const float step = std::sqrt(static_cast<float>(off.x * off.x + off.y * off.y + off.z * off.z));
            const float ng = g + step;
            const uint64_t nk = PackChunkKey(n);

            auto [nit, inserted] = nodes.try_emplace(nk, CoarseRec{FLT_MAX, nk, false});
            if (nit->second.closed || ng >= nit->second.g)
            {
                continue;
            }
            nit->second.g = ng;
            nit->second.parent = top.key;

            open.push_back(CoarseItem{ng + heuristic(n), nk});
            std::push_heap(open.begin(), open.end(), cmp);
        }
    }

    if (!reached)
    {
        return; // no coarse route; run unguided rather than guided by a wrong answer
    }

    // Walk the chain back, dilating as we go. A corridor exactly one chunk wide would pin the fine
    // path to a line of chunk centres; one chunk of slack on each side gives it room to actually
    // route around what it finds inside.
    uint64_t cur = goalKey;
    for (size_t guard = 0; guard <= nodes.size(); ++guard)
    {
        const IVec3 c = UnpackChunkKey(cur);
        for (int32_t dz = -1; dz <= 1; ++dz)
        {
            for (int32_t dy = -1; dy <= 1; ++dy)
            {
                for (int32_t dx = -1; dx <= 1; ++dx)
                {
                    m_corridor.insert(PackChunkKey({c.x + dx, c.y + dy, c.z + dz}));
                }
            }
        }

        auto it = nodes.find(cur);
        if (it == nodes.end() || it->second.parent == cur)
        {
            break;
        }
        cur = it->second.parent;
    }
}

bool Planner::OutsideCorridor(const IVec3& aVoxel) const
{
    if (m_corridor.empty())
    {
        return false; // no guidance in effect
    }
    return m_corridor.find(PackChunkKey(Volume::VoxelToChunk(aVoxel))) == m_corridor.end();
}

// ---------------------------------------------------------------------------------------------
// 4D state helpers
// ---------------------------------------------------------------------------------------------

uint64_t Planner::MakeKey(const IVec3& aVoxel, int32_t aYaw) const
{
    return Is4D() ? PackStateKey(aVoxel, aYaw) : PackVoxelKey(aVoxel);
}

IVec3 Planner::KeyVoxel(uint64_t aKey) const
{
    return Is4D() ? UnpackStateVoxel(aKey) : UnpackVoxelKey(aKey);
}

int32_t Planner::KeyYaw(uint64_t aKey) const
{
    return Is4D() ? UnpackStateYaw(aKey) : 0;
}

Vec3 Planner::YawHeading(int32_t aYaw) const
{
    if (!Is4D())
    {
        return Vec3{1.0f, 0.0f, 0.0f};
    }
    const float a = 6.283185307f * static_cast<float>(aYaw) / static_cast<float>(m_yawBuckets);
    return Vec3{std::cos(a), std::sin(a), 0.0f};
}

bool Planner::BodyFitsAtYaw(PinnedView& aView, const IVec3& aVoxel, int32_t aYaw) const
{
    return BodyFits(aView, m_volume.VoxelCenter(aVoxel), YawHeading(aYaw));
}

// ---------------------------------------------------------------------------------------------
// Residency
// ---------------------------------------------------------------------------------------------

void Planner::RequestChunkWithHalo(const IVec3& aChunkCoord)
{
    // Pull in the 3x3x3 neighbourhood, not just the one chunk. Parking costs a round trip to the
    // game thread, so it is much cheaper to over-fetch than to park again one voxel later.
    for (int32_t dz = -1; dz <= 1; ++dz)
    {
        for (int32_t dy = -1; dy <= 1; ++dy)
        {
            for (int32_t dx = -1; dx <= 1; ++dx)
            {
                const uint64_t key = PackChunkKey({aChunkCoord.x + dx, aChunkCoord.y + dy, aChunkCoord.z + dz});
                if (std::find(m_needed.begin(), m_needed.end(), key) == m_needed.end())
                {
                    m_needed.push_back(key);
                }
            }
        }
    }
}

bool Planner::EnsureBoxResident(PinnedView& aView, const IVec3& aMinVoxel, const IVec3& aMaxVoxel)
{
    const IVec3 lo = Volume::VoxelToChunk(aMinVoxel);
    const IVec3 hi = Volume::VoxelToChunk(aMaxVoxel);

    bool allResident = true;
    for (int32_t cz = lo.z; cz <= hi.z; ++cz)
    {
        for (int32_t cy = lo.y; cy <= hi.y; ++cy)
        {
            for (int32_t cx = lo.x; cx <= hi.x; ++cx)
            {
                const IVec3 c{cx, cy, cz};
                if (!aView.IsChunkResident(c))
                {
                    allResident = false;
                    const uint64_t key = PackChunkKey(c);
                    if (std::find(m_needed.begin(), m_needed.end(), key) == m_needed.end())
                    {
                        m_needed.push_back(key);
                    }
                }
            }
        }
    }
    return allResident;
}

bool Planner::EnsureNeighbourhoodResident(PinnedView& aView, const IVec3& aVoxel)
{
    // The 26 neighbours of a voxel span at most 2x2x2 chunks.
    if (EnsureBoxResident(aView, {aVoxel.x - 1, aVoxel.y - 1, aVoxel.z - 1},
                          {aVoxel.x + 1, aVoxel.y + 1, aVoxel.z + 1}))
    {
        return true;
    }

    // Parking already costs a round trip to the game thread, so widen the request while we are
    // paying for it - the frontier is about to move into these chunks anyway.
    RequestChunkWithHalo(Volume::VoxelToChunk(aVoxel));
    return false;
}

void Planner::CollectPrefetch(std::vector<uint64_t>& aOut) const
{
    const float chunkWorld = m_volume.Config().voxelSize * static_cast<float>(kChunkDim);
    const Vec3 delta = m_query.goal - m_query.start;
    const float dist = delta.Length();
    const int32_t steps = std::max(1, static_cast<int32_t>(dist / (chunkWorld * 0.5f)) + 1);
    const int32_t r = m_query.prefetchRadiusChunks;

    for (int32_t i = 0; i <= steps && aOut.size() < kMaxPrefetchChunks; ++i)
    {
        const float t = static_cast<float>(i) / static_cast<float>(steps);
        const Vec3 p = m_query.start + delta * t;
        const IVec3 c = Volume::VoxelToChunk(m_volume.WorldToVoxel(p));

        for (int32_t dz = -r; dz <= r; ++dz)
        {
            for (int32_t dy = -r; dy <= r; ++dy)
            {
                for (int32_t dx = -r; dx <= r; ++dx)
                {
                    const uint64_t key = PackChunkKey({c.x + dx, c.y + dy, c.z + dz});
                    if (std::find(aOut.begin(), aOut.end(), key) == aOut.end())
                    {
                        aOut.push_back(key);
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Occupancy queries
// ---------------------------------------------------------------------------------------------

bool Planner::IsTraversable(PinnedView& aView, const IVec3& aVoxel, bool& aMissing) const
{
    if (!m_volume.InBounds(aVoxel))
    {
        aMissing = false;
        return false;
    }
    if (aView.IsBlocked(aVoxel, aMissing))
    {
        return false;
    }

    // Dynamic obstacles are a SECOND predicate, applied only after the static answer says the
    // agent fits. Keeping them out of the clearance field is what lets the static answer stay
    // cached (and baked) forever while this one is rebuilt every frame.
    const DynamicObstacleSet* dyn = aView.Dynamic();
    if (dyn && !dyn->Empty() && dyn->Blocked(m_volume.VoxelCenter(aVoxel)))
    {
        return false;
    }
    return true;
}

bool Planner::CanStep(PinnedView& aView, const IVec3& aFrom, const IVec3& aTo, bool& aMissing) const
{
    if (!IsTraversable(aView, aTo, aMissing))
    {
        return false;
    }

    // Corner-cut guard. A 26-connected diagonal must not slip through a solid edge or corner, so
    // every axis-aligned component of the move has to be clear on its own.
    const IVec3 d = aTo - aFrom;
    bool miss = false;
    if (d.x != 0 && !IsTraversable(aView, {aFrom.x + d.x, aFrom.y, aFrom.z}, miss))
    {
        return false;
    }
    if (d.y != 0 && !IsTraversable(aView, {aFrom.x, aFrom.y + d.y, aFrom.z}, miss))
    {
        return false;
    }
    if (d.z != 0 && !IsTraversable(aView, {aFrom.x, aFrom.y, aFrom.z + d.z}, miss))
    {
        return false;
    }
    return true;
}

// Amanatides-Woo voxel traversal between two voxel centres.
// A chunk that is not resident counts as a blocker: LOS fails conservatively and the caller
// falls back to the grid parent. That keeps the search from having to park inside a LOS test.
bool Planner::LineOfSight(PinnedView& aView, const IVec3& aFrom, const IVec3& aTo) const
{
    if (aFrom == aTo)
    {
        return true;
    }

    // Any-angle shortcutting is exactly where a dynamic obstacle would otherwise be missed: the
    // voxel walk below only consults the static field, so a shortcut could be drawn straight
    // through a parked car that every node test had already rejected.
    const DynamicObstacleSet* dyn = aView.Dynamic();
    if (dyn && !dyn->Empty() &&
        dyn->SegmentBlocked(m_volume.VoxelCenter(aFrom), m_volume.VoxelCenter(aTo),
                            m_volume.Config().voxelSize * 0.5f))
    {
        return false;
    }

    const float s = m_volume.Config().voxelSize;
    const Vec3 pa = m_volume.VoxelCenter(aFrom);
    const Vec3 pb = m_volume.VoxelCenter(aTo);
    const Vec3 d = pb - pa;

    const int32_t step[3] = {d.x > 0.0f ? 1 : (d.x < 0.0f ? -1 : 0), d.y > 0.0f ? 1 : (d.y < 0.0f ? -1 : 0),
                             d.z > 0.0f ? 1 : (d.z < 0.0f ? -1 : 0)};
    const float dc[3] = {d.x, d.y, d.z};
    const float pac[3] = {pa.x, pa.y, pa.z};
    int32_t cur[3] = {aFrom.x, aFrom.y, aFrom.z};
    const int32_t dst[3] = {aTo.x, aTo.y, aTo.z};

    float tMax[3];
    float tDelta[3];
    for (int32_t a = 0; a < 3; ++a)
    {
        if (step[a] == 0)
        {
            tMax[a] = FLT_MAX;
            tDelta[a] = FLT_MAX;
        }
        else
        {
            const float boundary = static_cast<float>(cur[a] + (step[a] > 0 ? 1 : 0)) * s;
            tMax[a] = (boundary - pac[a]) / dc[a];
            tDelta[a] = s / std::fabs(dc[a]);
        }
    }

    const int32_t manhattan = std::abs(dst[0] - cur[0]) + std::abs(dst[1] - cur[1]) + std::abs(dst[2] - cur[2]);
    const int32_t maxSteps = manhattan + 8;

    for (int32_t i = 0; i < maxSteps; ++i)
    {
        const int32_t axis = (tMax[0] <= tMax[1] && tMax[0] <= tMax[2]) ? 0 : (tMax[1] <= tMax[2] ? 1 : 2);
        cur[axis] += step[axis];
        tMax[axis] += tDelta[axis];

        const IVec3 v{cur[0], cur[1], cur[2]};
        bool missing = false;
        if (!IsTraversable(aView, v, missing))
        {
            return false;
        }

        if (v == aTo)
        {
            return true;
        }
    }
    return false;
}

bool Planner::SnapToFree(PinnedView& aView, const IVec3& aVoxel, IVec3& aOut, bool& aMissing) const
{
    aMissing = false;

    bool missing = false;
    if (IsTraversable(aView, aVoxel, missing))
    {
        aOut = aVoxel;
        return true;
    }
    if (missing)
    {
        aMissing = true;
        return false;
    }

    // Expanding cube shells. Within a shell, take the Euclidean-nearest hit rather than whatever
    // the scan order reaches first - a corner of the shell is ~1.7x further away than a face, and
    // snapping the caller's start position further than necessary is visible in the resulting path.
    for (int32_t r = 1; r <= m_snapVoxelRadius; ++r)
    {
        int32_t bestDistSq = 0;
        bool found = false;

        for (int32_t dz = -r; dz <= r; ++dz)
        {
            for (int32_t dy = -r; dy <= r; ++dy)
            {
                for (int32_t dx = -r; dx <= r; ++dx)
                {
                    // Shell only: the interior was covered by smaller radii.
                    if (std::abs(dx) != r && std::abs(dy) != r && std::abs(dz) != r)
                    {
                        continue;
                    }
                    const int32_t distSq = dx * dx + dy * dy + dz * dz;
                    if (found && distSq >= bestDistSq)
                    {
                        continue;
                    }
                    const IVec3 v{aVoxel.x + dx, aVoxel.y + dy, aVoxel.z + dz};
                    if (IsTraversable(aView, v, missing))
                    {
                        aOut = v;
                        bestDistSq = distSq;
                        found = true;
                    }
                }
            }
        }

        if (found)
        {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------------------------
// Costs
// ---------------------------------------------------------------------------------------------

float Planner::SegmentCost(PinnedView& aView, const IVec3& aFrom, const IVec3& aTo) const
{
    const float s = m_volume.Config().voxelSize;
    const float dx = static_cast<float>(aTo.x - aFrom.x) * s;
    const float dy = static_cast<float>(aTo.y - aFrom.y) * s;
    const float dz = static_cast<float>(aTo.z - aFrom.z) * s * m_query.verticalCostMultiplier;

    float d = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (m_query.tightnessPenalty > 1.0f)
    {
        bool missing = false;
        if (aView.IsTight(aTo, missing))
        {
            d *= m_query.tightnessPenalty;
        }
    }

    // Hierarchical guidance, applied as a cost bias rather than a restriction so the search stays
    // complete - it will still leave the corridor when that is the only way through.
    if (m_query.coarsePenalty > 1.0f && OutsideCorridor(aTo))
    {
        d *= m_query.coarsePenalty;
    }
    return d;
}

// Plain Euclidean distance. Both cost multipliers are >= 1, so this never overestimates and the
// heuristic stays admissible (before heuristicWeight is applied).
float Planner::Heuristic(const IVec3& aVoxel, const IVec3& aTarget) const
{
    const float s = m_volume.Config().voxelSize;
    const float dx = static_cast<float>(aTarget.x - aVoxel.x) * s;
    const float dy = static_cast<float>(aTarget.y - aVoxel.y) * s;
    const float dz = static_cast<float>(aTarget.z - aVoxel.z) * s;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// The backward frontier grows from the goal, so an edge it is relaxing (voxel -> n in expansion
// order) is actually traversed n -> voxel in the finished path. Both CanStep and SegmentCost are
// direction-sensitive, so the arguments are swapped for that side. Getting this wrong does not
// fail loudly - it silently accepts diagonal moves that clip a corner, and mis-prices tight
// space - which is why it is funnelled through these two functions rather than open-coded.
bool Planner::StepAllowed(const Frontier& aF, PinnedView& aView, const IVec3& aFrom, const IVec3& aTo,
                          bool& aMissing) const
{
    return aF.forward ? CanStep(aView, aFrom, aTo, aMissing) : CanStep(aView, aTo, aFrom, aMissing);
}

float Planner::StepCost(const Frontier& aF, PinnedView& aView, const IVec3& aFrom, const IVec3& aTo) const
{
    return aF.forward ? SegmentCost(aView, aFrom, aTo) : SegmentCost(aView, aTo, aFrom);
}

void Planner::PushOpen(Frontier& aF, uint64_t aKey, float aPriority)
{
    const auto cmp = [](const HeapItem& a, const HeapItem& b) { return a.f > b.f; };
    aF.open.push_back(HeapItem{aPriority, aKey});
    std::push_heap(aF.open.begin(), aF.open.end(), cmp);
}

bool Planner::PopOpen(Frontier& aF, HeapItem& aOut)
{
    if (aF.open.empty())
    {
        return false;
    }
    const auto cmp = [](const HeapItem& a, const HeapItem& b) { return a.f > b.f; };
    std::pop_heap(aF.open.begin(), aF.open.end(), cmp);
    aOut = aF.open.back();
    aF.open.pop_back();
    return true;
}

// ---------------------------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------------------------

SliceResult Planner::Step(PinnedView& aView, const std::atomic<bool>& aCancelled)
{
    if (m_phase == Phase::Done)
    {
        return SliceResult::Finished;
    }

    if (aCancelled.load(std::memory_order_relaxed))
    {
        m_result.status = Status::Cancelled;
        m_phase = Phase::Done;
        return SliceResult::Cancelled;
    }

    // -------- Init: validate, make endpoints resident, snap out of geometry -------------------
    if (m_phase == Phase::Init)
    {
        if (!m_query.IsValid() || !m_volume.InBounds(m_query.start) || !m_volume.InBounds(m_query.goal))
        {
            m_result.status = Status::BadInput;
            m_phase = Phase::Done;
            return SliceResult::Finished;
        }

        const IVec3 rawStart = m_volume.WorldToVoxel(m_query.start);
        const IVec3 rawGoal = m_volume.WorldToVoxel(m_query.goal);
        const int32_t r = m_snapVoxelRadius;

        // Both calls must run: each accumulates its own missing chunks into m_needed, so a
        // short-circuit here would cost an extra park to discover the second box.
        const bool startResident = EnsureBoxResident(aView, {rawStart.x - r, rawStart.y - r, rawStart.z - r},
                                                     {rawStart.x + r, rawStart.y + r, rawStart.z + r});
        const bool goalResident = EnsureBoxResident(aView, {rawGoal.x - r, rawGoal.y - r, rawGoal.z - r},
                                                    {rawGoal.x + r, rawGoal.y + r, rawGoal.z + r});
        if (!startResident || !goalResident)
        {
            return SliceResult::NeedsWorld;
        }

        bool missing = false;
        if (!SnapToFree(aView, rawStart, m_startVoxel, missing))
        {
            if (missing)
            {
                return SliceResult::NeedsWorld;
            }
            m_result.status = Status::Unreachable; // start is buried and snapRadius did not help
            m_phase = Phase::Done;
            return SliceResult::Finished;
        }
        if (!SnapToFree(aView, rawGoal, m_goalVoxel, missing))
        {
            if (missing)
            {
                return SliceResult::NeedsWorld;
            }
            m_result.status = Status::Unreachable;
            m_phase = Phase::Done;
            return SliceResult::Finished;
        }

        m_result.startWasSnapped = !(m_startVoxel == rawStart);
        m_result.goalWasSnapped = !(m_goalVoxel == rawGoal);
        m_result.snappedStart = m_result.startWasSnapped ? m_volume.VoxelCenter(m_startVoxel) : m_query.start;
        m_result.snappedGoal = m_result.goalWasSnapped ? m_volume.VoxelCenter(m_goalVoxel) : m_query.goal;

        // In 4D the start carries the agent's initial heading; the goal is matched on position
        // alone (see the goal test in ExpandOne), so its yaw here is only a placeholder.
        m_startKey = MakeKey(m_startVoxel, 0);
        m_goalKey = MakeKey(m_goalVoxel, 0);

        // Coarse route first, so the fine search is biased from its very first expansion. Cheap:
        // it reads the bake index only, and is skipped entirely when there is no bake.
        BuildCorridor();

        if (m_startKey == m_goalKey)
        {
            m_result.points = {m_result.snappedStart, m_result.snappedGoal};
            m_result.length = (m_result.snappedGoal - m_result.snappedStart).Length();
            m_result.status = Status::Success;
            m_phase = Phase::Done;
            return SliceResult::Finished;
        }

        m_fwd.rootKey = m_startKey;
        m_fwd.target = m_goalVoxel;
        m_fwd.forward = true;
        m_fwd.active = true;

        NodeRec& start = m_fwd.nodes[m_startKey];
        start.g = 0.0f;
        start.parent = m_startKey;
        start.closed = false;

        m_fwd.bestKey = m_startKey;
        m_fwd.bestH = Heuristic(m_startVoxel, m_fwd.target);
        PushOpen(m_fwd, m_startKey, m_fwd.bestH * m_query.heuristicWeight);

        if (m_query.bidirectional)
        {
            m_bwd.rootKey = m_goalKey;
            m_bwd.target = m_startVoxel;
            m_bwd.forward = false;
            m_bwd.active = true;

            NodeRec& goal = m_bwd.nodes[m_goalKey];
            goal.g = 0.0f;
            goal.parent = m_goalKey;
            goal.closed = false;

            m_bwd.bestKey = m_goalKey;
            m_bwd.bestH = Heuristic(m_goalVoxel, m_bwd.target);
            PushOpen(m_bwd, m_goalKey, m_bwd.bestH * m_query.heuristicWeight);
        }

        m_phase = Phase::Search;
    }

    // -------- Search --------------------------------------------------------------------------
    int32_t sinceCancelCheck = 0;
    while (true)
    {
        if (++sinceCancelCheck >= 256)
        {
            sinceCancelCheck = 0;
            if (aCancelled.load(std::memory_order_relaxed))
            {
                m_result.status = Status::Cancelled;
                m_phase = Phase::Done;
                return SliceResult::Cancelled;
            }
        }

        if (m_expansions >= m_query.maxExpansions)
        {
            Finish(Status::BudgetExhausted, aView, m_fwd.bestKey);
            return SliceResult::Finished;
        }

        // Expand whichever side is currently cheaper to push forward. Balancing on the top f
        // rather than strictly alternating keeps a frontier that is stuck in a tight pocket from
        // dragging the whole search down, and it is what makes a sealed goal cheap to detect: the
        // small backward frontier drains and reports Exhausted almost immediately.
        float fFwd = 0.0f;
        float fBwd = 0.0f;
        const bool haveFwd = TopF(m_fwd, fFwd);
        const bool haveBwd = TopF(m_bwd, fBwd);

        // Either side running dry is decisive. Forward dry means nothing more is reachable from
        // the agent; backward dry means nothing reaches the goal. Both are genuine proofs of
        // unreachability, not budget failures - which is exactly the fast negative answer that
        // unidirectional search cannot give.
        if (!haveFwd || (m_bwd.active && !haveBwd))
        {
            if (m_haveMeet)
            {
                Finish(Status::Success, aView, m_meetKey);
            }
            else
            {
                Finish(Status::Unreachable, aView, m_fwd.bestKey);
            }
            return SliceResult::Finished;
        }

        // A meeting only bounds the answer from above. Once neither side can possibly beat it,
        // the best meeting found is the answer.
        if (m_haveMeet && (!m_bwd.active || fFwd + fBwd >= m_meetCost))
        {
            Finish(Status::Success, aView, m_meetKey);
            return SliceResult::Finished;
        }

        const bool useFwd = !m_bwd.active || fFwd <= fBwd;
        Frontier& side = useFwd ? m_fwd : m_bwd;
        Frontier& other = useFwd ? m_bwd : m_fwd;

        switch (ExpandOne(side, other, aView))
        {
        case ExpandResult::NeedsWorld:
            return SliceResult::NeedsWorld;

        case ExpandResult::Exhausted:
            if (m_haveMeet)
            {
                Finish(Status::Success, aView, m_meetKey);
            }
            else
            {
                Finish(Status::Unreachable, aView, m_fwd.bestKey);
            }
            return SliceResult::Finished;

        case ExpandResult::Continued:
            break;
        }

        // Unidirectional termination: reaching the goal node itself is the answer.
        if (!m_bwd.active && m_haveMeet)
        {
            Finish(Status::Success, aView, m_meetKey);
            return SliceResult::Finished;
        }
    }
}

bool Planner::TopF(const Frontier& aF, float& aOut) const
{
    if (!aF.active || aF.open.empty())
    {
        return false;
    }
    aOut = aF.open.front().f;
    return true;
}

// Expands the single cheapest node of one frontier. Identical logic for both sides; everything
// direction-dependent goes through the frontier's target (for the heuristic) and through
// StepAllowed/StepCost (for edge order).
Planner::ExpandResult Planner::ExpandOne(Frontier& aF, Frontier& aOther, PinnedView& aView)
{
    HeapItem top{};
    if (!PopOpen(aF, top))
    {
        return ExpandResult::Exhausted;
    }

    auto it = aF.nodes.find(top.key);
    if (it == aF.nodes.end() || it->second.closed)
    {
        return ExpandResult::Continued; // stale heap entry, superseded by a cheaper push
    }

    const IVec3 voxel = KeyVoxel(top.key);
    const int32_t yaw = KeyYaw(top.key);

    if (!EnsureNeighbourhoodResident(aView, voxel))
    {
        PushOpen(aF, top.key, top.f); // put it back untouched; resume re-pops it
        return ExpandResult::NeedsWorld;
    }

    NodeRec& rec = it->second;

    // Lazy Theta*: the parent was assigned optimistically. Verify line of sight now, and if it
    // does not hold, reattach to the best visible closed neighbour.
    // Lazy Theta*'s shortcut assumes it can jump straight from the grandparent, which does not
    // compose with an orientation constraint along that line - so 4D runs as plain A*.
    if (!Is4D() && rec.parent != top.key && !LineOfSight(aView, KeyVoxel(rec.parent), voxel))
    {
        float bestG = FLT_MAX;
        uint64_t bestParent = 0;
        bool found = false;

        for (const IVec3& off : kNeighbours)
        {
            const IVec3 n = voxel + off;
            auto nit = aF.nodes.find(MakeKey(n, yaw));
            if (nit == aF.nodes.end() || !nit->second.closed)
            {
                continue;
            }
            bool missing = false;
            // Expansion order is (voxel -> n); n is the candidate parent, so the traversal being
            // tested is the reverse of a normal relaxation. Passing (n, voxel) here lets
            // StepAllowed apply the same flip it applies everywhere else.
            if (!StepAllowed(aF, aView, n, voxel, missing))
            {
                continue;
            }
            const float cand = nit->second.g + StepCost(aF, aView, n, voxel);
            if (cand < bestG)
            {
                bestG = cand;
                bestParent = MakeKey(n, yaw);
                found = true;
            }
        }

        if (!found)
        {
            return ExpandResult::Continued; // orphaned - no visible closed neighbour, drop it
        }
        rec.g = bestG;
        rec.parent = bestParent;
    }

    rec.closed = true;
    ++m_expansions;

    const float h = Heuristic(voxel, aF.target);
    if (h < aF.bestH)
    {
        aF.bestH = h;
        aF.bestKey = top.key;
    }

    // Have the frontiers touched here? For a unidirectional search `aOther` is inactive and this
    // degenerates to the plain "did we reach the goal node" test.
    if (aOther.active)
    {
        auto oit = aOther.nodes.find(top.key);
        if (oit != aOther.nodes.end() && oit->second.closed)
        {
            const float total = rec.g + oit->second.g;
            if (total < m_meetCost)
            {
                m_meetCost = total;
                m_meetKey = top.key;
                m_haveMeet = true;
            }
        }
    }
    else if (Is4D() ? (voxel == m_goalVoxel) : (top.key == m_goalKey))
    {
        // Unidirectional: only the forward frontier is ever active, so reaching the goal node is
        // the terminal condition and "the meeting point" is simply the goal.
        //
        // In 4D the goal is matched on POSITION only - arriving pointing any direction counts.
        // Demanding a particular final heading would make most goals unreachable for no benefit;
        // a caller that needs one can plan to a pose-specific goal itself.
        m_meetKey = top.key;
        m_meetCost = rec.g;
        m_haveMeet = true;
    }

    // Copy out of the map before any insertion below. References into unordered_map survive
    // rehashing, but locals make the lifetime obvious rather than subtle.
    // 3D attaches neighbours to the GRANDPARENT (Theta* path 2) and repairs at pop time. 4D
    // cannot: that shortcut is a straight line the orientation constraint was never checked
    // along, so it attaches to this node instead and runs as plain A*.
    const uint64_t pathParent = Is4D() ? top.key : rec.parent;
    const IVec3 parentVoxel = KeyVoxel(pathParent);

    // Guard the lookup, not the result. Every parent is written from a key that exists, so this
    // should never miss - but `find(...)->second` on a miss is an access violation, and a
    // self-parent fallback keeps a logic slip from becoming a crash.
    const auto parentIt = aF.nodes.find(pathParent);
    const float parentG = (parentIt != aF.nodes.end()) ? parentIt->second.g : rec.g;

    for (const IVec3& off : kNeighbours)
    {
        const IVec3 n = voxel + off;
        bool missing = false;
        if (!StepAllowed(aF, aView, voxel, n, missing))
        {
            continue;
        }

        // In 4D the agent occupies a segment, not a point, so a voxel being free is not enough -
        // the whole body has to fit there at its current heading. This is what lets the search
        // discover that a long body must be aligned a particular way to pass at all.
        if (Is4D())
        {
            if (!BodyFitsAtYaw(aView, n, yaw))
            {
                continue;
            }

            // Keep the body pointing roughly where it is going. The direction tested is the one
            // the finished path is TRAVERSED in, which is the reverse of expansion order on the
            // backward frontier - same asymmetry StepAllowed handles for the edge tests.
            if (m_query.yawFollowsTravel)
            {
                const IVec3 d = aF.forward ? (n - voxel) : (voxel - n);
                const float hx = static_cast<float>(d.x);
                const float hy = static_cast<float>(d.y);
                const float hlen = std::sqrt(hx * hx + hy * hy);
                if (hlen > 1e-4f)
                {
                    const Vec3 head = YawHeading(yaw);
                    const float alignment = (head.x * hx + head.y * hy) / hlen;
                    if (alignment < std::cos(m_query.yawTravelCone))
                    {
                        continue; // would be flying sideways; rotate first
                    }
                }
            }
        }

        const uint64_t nk = MakeKey(n, yaw);
        auto [nit, inserted] = aF.nodes.try_emplace(nk);
        if (inserted)
        {
            nit->second.g = FLT_MAX;
            nit->second.parent = nk;
            nit->second.closed = false;
        }
        else if (nit->second.closed)
        {
            continue;
        }

        // Path 2 of Theta*: attach straight to the grandparent and let the pop-time LOS check
        // repair it if the shortcut turns out to be blocked. Disabled in 4D (see the pop-time
        // check), where `pathParent` is the node itself so this reduces to plain A*.
        const float newG = parentG + StepCost(aF, aView, parentVoxel, n);
        if (newG < nit->second.g)
        {
            nit->second.g = newG;
            nit->second.parent = pathParent;
            PushOpen(aF, nk, newG + Heuristic(n, aF.target) * m_query.heuristicWeight);
        }
    }

    // Rotation in place: the fourth dimension. Without this the agent could only ever hold the
    // heading it started with, which would make 4D a strictly worse 3D.
    if (Is4D())
    {
        for (int32_t d = -1; d <= 1; d += 2)
        {
            const int32_t ny = (yaw + d + m_yawBuckets) % m_yawBuckets;
            if (!BodyFitsAtYaw(aView, voxel, ny))
            {
                continue;
            }

            const uint64_t nk = MakeKey(voxel, ny);
            auto [nit, inserted] = aF.nodes.try_emplace(nk);
            if (inserted)
            {
                nit->second.g = FLT_MAX;
                nit->second.parent = nk;
                nit->second.closed = false;
            }
            else if (nit->second.closed)
            {
                continue;
            }

            const float newG = rec.g + m_query.yawCost;
            if (newG < nit->second.g)
            {
                nit->second.g = newG;
                nit->second.parent = top.key;
                PushOpen(aF, nk, newG + Heuristic(voxel, aF.target) * m_query.heuristicWeight);
            }
        }
    }

    return ExpandResult::Continued;
}

// ---------------------------------------------------------------------------------------------
// Result construction
// ---------------------------------------------------------------------------------------------

void Planner::Abort(PinnedView& aView, Status aReason)
{
    if (m_phase == Phase::Done)
    {
        return;
    }
    // Init never got far enough to have a frontier, so there is nothing to salvage.
    if (m_phase == Phase::Init)
    {
        m_phase = Phase::Done;
        m_result.status = aReason;
        return;
    }
    // An aborted bidirectional search may still have touched. That is a complete path, not a
    // partial, so prefer it over the forward frontier's best-effort stub.
    if (m_haveMeet)
    {
        Finish(Status::Success, aView, m_meetKey);
        return;
    }
    Finish(aReason, aView, m_fwd.bestKey);
}

void Planner::Finish(Status aStatus, PinnedView& aView, uint64_t aEndKey)
{
    m_phase = Phase::Done;
    m_result.expansions = m_expansions;

    if (aStatus == Status::Success)
    {
        // A meeting point has half its chain in each frontier, so it cannot be walked by
        // BuildPath, which only knows about the forward one.
        if (m_bwd.active && m_haveMeet && aEndKey == m_meetKey)
        {
            BuildMeetingPath(aView);
        }
        else
        {
            BuildPath(aView, aEndKey);
        }
        m_result.status = Status::Success;
        return;
    }

    // Failure. A best-effort path toward the goal beats nothing for a flying agent, but only if
    // it actually made progress - a "path" consisting of the start point is noise.
    if (m_query.allowPartial && aEndKey != m_startKey)
    {
        BuildPath(aView, aEndKey);
        if (m_result.points.size() >= 2)
        {
            m_result.status = Status::PartialSuccess;
            return;
        }
    }

    m_result.points.clear();
    m_result.clearances.clear(); // must not outlive the points it describes
    m_result.yaws.clear();
    m_result.length = 0.0f;
    m_result.status = aStatus;
}

void Planner::BuildPath(PinnedView& aView, uint64_t aEndKey)
{
    std::vector<IVec3> path;
    std::vector<int32_t> yaws;
    path.reserve(64);

    // A parent chain is at most one entry per visited node; anything longer means a cycle, which
    // would otherwise spin until the process runs out of memory.
    const auto maxSteps = static_cast<int64_t>(m_fwd.nodes.size()) + 2;

    uint64_t cur = aEndKey;
    for (int64_t guard = 0; guard < maxSteps; ++guard)
    {
        path.push_back(KeyVoxel(cur));
        yaws.push_back(KeyYaw(cur));
        auto it = m_fwd.nodes.find(cur);
        if (it == m_fwd.nodes.end() || it->second.parent == cur)
        {
            break;
        }
        cur = it->second.parent;
    }
    std::reverse(path.begin(), path.end());
    std::reverse(yaws.begin(), yaws.end());

    // Smoothing drops waypoints, which would silently desynchronise the yaw sequence from the
    // points it describes. In 4D the orientation plan IS the answer - including any decision to
    // rotate in place before entering a gap - so it is kept verbatim instead.
    if (!Is4D() && m_query.smoothing != Smoothing::None)
    {
        StringPull(aView, path);
    }

    EmitWorldPoints(path);

    m_result.yaws.clear();
    if (Is4D() && yaws.size() == m_result.points.size())
    {
        m_result.yaws.reserve(yaws.size());
        for (int32_t y : yaws)
        {
            m_result.yaws.push_back(6.283185307f * static_cast<float>(y) /
                                    static_cast<float>(m_yawBuckets));
        }
    }

    // The first voxel contains the snapped start, so use the exact position. The last voxel only
    // corresponds to the goal when the goal was actually reached - a partial path must keep its
    // own endpoint.
    if (!m_result.points.empty())
    {
        m_result.points.front() = m_result.snappedStart;
        // In 4D the goal is matched on position, so the end key carries whatever yaw the agent
        // arrived with and will not equal m_goalKey by value.
        const bool reachedGoal = Is4D() ? (KeyVoxel(aEndKey) == m_goalVoxel) : (aEndKey == m_goalKey);
        if (reachedGoal)
        {
            m_result.points.back() = m_result.snappedGoal;
        }
    }

    FinalisePath(aView);
}

// Stitch the two halves of a bidirectional search at the meeting node.
//
// The frontiers store parents pointing back at their own root, so the forward chain walks
// meet -> start and needs reversing, while the backward chain already walks meet -> goal and does
// not. The join is a hard vertex that neither half's any-angle smoothing has seen, which is why
// the concatenated path is string-pulled as a whole rather than per half.
void Planner::BuildMeetingPath(PinnedView& aView)
{
    std::vector<IVec3> path;
    std::vector<int32_t> yaws;
    path.reserve(64);

    const auto fwdGuard = static_cast<int64_t>(m_fwd.nodes.size()) + 2;
    uint64_t cur = m_meetKey;
    for (int64_t guard = 0; guard < fwdGuard; ++guard)
    {
        path.push_back(KeyVoxel(cur));
        yaws.push_back(KeyYaw(cur));
        auto it = m_fwd.nodes.find(cur);
        if (it == m_fwd.nodes.end() || it->second.parent == cur)
        {
            break;
        }
        cur = it->second.parent;
    }
    std::reverse(path.begin(), path.end());
    std::reverse(yaws.begin(), yaws.end());

    // Skip the meeting node itself: it is already the last entry of the forward half.
    const auto bwdGuard = static_cast<int64_t>(m_bwd.nodes.size()) + 2;
    auto meetIt = m_bwd.nodes.find(m_meetKey);
    if (meetIt != m_bwd.nodes.end())
    {
        cur = meetIt->second.parent;
        for (int64_t guard = 0; guard < bwdGuard; ++guard)
        {
            if (cur == m_meetKey)
            {
                break; // meeting node was the backward root; nothing to append
            }
            path.push_back(KeyVoxel(cur));
            yaws.push_back(KeyYaw(cur));

            auto it = m_bwd.nodes.find(cur);
            if (it == m_bwd.nodes.end() || it->second.parent == cur)
            {
                break;
            }
            cur = it->second.parent;
        }
    }

    // Same reason as BuildPath: dropping waypoints would desynchronise the yaw sequence.
    if (!Is4D() && m_query.smoothing != Smoothing::None)
    {
        StringPull(aView, path);
    }

    EmitWorldPoints(path);

    m_result.yaws.clear();
    if (Is4D() && yaws.size() == m_result.points.size())
    {
        m_result.yaws.reserve(yaws.size());
        for (int32_t y : yaws)
        {
            m_result.yaws.push_back(6.283185307f * static_cast<float>(y) /
                                    static_cast<float>(m_yawBuckets));
        }
    }

    FinalisePath(aView);
}

bool Planner::BodyFits(PinnedView& aView, const Vec3& aPos, const Vec3& aHeading) const
{
    const float half = m_query.agentLength * 0.5f;
    if (!(half > 0.0f))
    {
        return true;
    }

    const float step = m_volume.Config().voxelSize;
    const auto n = std::max(1, static_cast<int32_t>(std::ceil(half / std::max(step, 1e-3f))));

    // Both ends plus the middle. The centre is already known good (it is a path vertex), but
    // walking it costs nothing and keeps the loop uniform.
    for (int32_t i = -n; i <= n; ++i)
    {
        const float t = (static_cast<float>(i) / static_cast<float>(n)) * half;
        const Vec3 p = aPos + aHeading * t;
        bool missing = false;
        if (!IsTraversable(aView, m_volume.WorldToVoxel(p), missing))
        {
            return false;
        }
    }
    return true;
}

bool Planner::TurnFits(PinnedView& aView, const Vec3& aPos, const Vec3& aFrom, const Vec3& aTo) const
{
    if (!TurnClearanceEnabled())
    {
        return true;
    }

    const int32_t steps = std::max(1, m_query.turnSamples);
    for (int32_t k = 0; k <= steps; ++k)
    {
        const float t = static_cast<float>(k) / static_cast<float>(steps);
        // Normalised lerp rather than a true slerp: the intermediate directions do not need to be
        // evenly spaced in angle, they only need to cover the sweep.
        const Vec3 mixed = aFrom * (1.0f - t) + aTo * t;
        if (mixed.LengthSq() < 1e-8f)
        {
            // A near-exact reversal has no meaningful interpolation - the body has to sweep all
            // the way round, so require it to fit across the full turn instead of through it.
            continue;
        }
        if (!BodyFits(aView, aPos, mixed.Normalized()))
        {
            return false;
        }
    }
    return true;
}

void Planner::StringPull(PinnedView& aView, std::vector<IVec3>& aPath) const
{
    if (aPath.size() < 3)
    {
        return;
    }

    const bool turnAware = TurnClearanceEnabled();

    std::vector<IVec3> out;
    out.reserve(aPath.size());
    out.push_back(aPath.front());

    size_t i = 0;
    while (i + 1 < aPath.size())
    {
        const size_t limit = std::min(aPath.size() - 1, i + kStringPullWindow);
        size_t best = i + 1;
        for (size_t j = limit; j > i + 1; --j)
        {
            if (!LineOfSight(aView, aPath[i], aPath[j]))
            {
                continue;
            }

            // Shortcutting makes corners SHARPER, and a sharper corner is exactly what an
            // elongated body cannot rotate through. So a shortcut has to be paid for at the
            // vertex it creates, not just checked for line of sight.
            if (turnAware && out.size() >= 2)
            {
                const Vec3 prev = m_volume.VoxelCenter(out[out.size() - 2]);
                const Vec3 here = m_volume.VoxelCenter(aPath[i]);
                const Vec3 next = m_volume.VoxelCenter(aPath[j]);
                const Vec3 in = (here - prev).Normalized();
                const Vec3 outDir = (next - here).Normalized();
                if (!TurnFits(aView, here, in, outDir))
                {
                    continue; // try a nearer j, which bends less
                }
            }

            best = j;
            break;
        }
        out.push_back(aPath[best]);
        i = best;
    }

    aPath.swap(out);
}

// Shared tail of every path build. Smoothing can replace the point list wholesale (the spline
// resamples it), so clearance and length are both derived from the FINAL points rather than from
// the voxel chain - otherwise they would describe a path that is no longer the one being returned.
void Planner::FinalisePath(PinnedView& aView)
{
    if (m_query.smoothing == Smoothing::Spline)
    {
        ApplySpline(aView);
    }
    else if (m_query.smoothing == Smoothing::Fillet)
    {
        ApplyFillet(aView);
    }

    m_result.coarseExpansions = m_coarseExpansions;
    m_result.corridorChunks = static_cast<int32_t>(m_corridor.size());

    m_result.clearances.clear();
    m_result.clearances.reserve(m_result.points.size());
    for (const Vec3& p : m_result.points)
    {
        bool missing = false;
        const uint8_t c = aView.Clearance(m_volume.WorldToVoxel(p), missing);
        m_result.clearances.push_back(missing ? 0.0f : m_volume.ClearanceMetres(c));
    }

    // Report corners an elongated body still cannot rotate through. The smoother already avoids
    // creating these, so anything left is the underlying route being too tight for the body -
    // worth telling the caller about rather than handing back a path it cannot fly.
    m_result.infeasibleTurns = 0;
    if (TurnClearanceEnabled())
    {
        for (size_t i = 1; i + 1 < m_result.points.size(); ++i)
        {
            const Vec3 in = (m_result.points[i] - m_result.points[i - 1]).Normalized();
            const Vec3 outDir = (m_result.points[i + 1] - m_result.points[i]).Normalized();
            if (!TurnFits(aView, m_result.points[i], in, outDir))
            {
                ++m_result.infeasibleTurns;
            }
        }
    }

    float len = 0.0f;
    for (size_t i = 1; i < m_result.points.size(); ++i)
    {
        len += (m_result.points[i] - m_result.points[i - 1]).Length();
    }
    m_result.length = len;
}

// Round each corner to a bounded turn radius.
//
// Each corner is replaced by a quadratic Bezier through its two tangent points, which by the
// convex-hull property stays inside the triangle (tangentIn, vertex, tangentOut) - so unlike the
// Catmull-Rom pass it cannot bulge outward into geometry the polyline had cleared. Every sample is
// still validated, and a corner that will not validate is retried at half the radius before being
// left as a hard vertex. Degrading per corner rather than discarding the whole curve is the point:
// one tight bend should not cost the smoothing on every other bend.
void Planner::ApplyFillet(PinnedView& aView)
{
    const auto& pts = m_result.points;
    if (pts.size() < 3)
    {
        return;
    }

    const int32_t samples = std::max(2, m_query.filletSamples);

    std::vector<Vec3> out;
    out.reserve(pts.size() * static_cast<size_t>(samples));
    out.push_back(pts.front());

    for (size_t i = 1; i + 1 < pts.size(); ++i)
    {
        const Vec3 prev = pts[i - 1];
        const Vec3 here = pts[i];
        const Vec3 next = pts[i + 1];

        const Vec3 inSeg = here - prev;
        const Vec3 outSeg = next - here;
        const float inLen = inSeg.Length();
        const float outLen = outSeg.Length();

        if (inLen < 1e-4f || outLen < 1e-4f)
        {
            out.push_back(here); // degenerate (a 4D rotation in place, for instance)
            continue;
        }

        const Vec3 inDir = inSeg * (1.0f / inLen);
        const Vec3 outDir = outSeg * (1.0f / outLen);

        const float cosTurn = std::clamp(inDir.Dot(outDir), -1.0f, 1.0f);
        if (cosTurn > 0.9999f)
        {
            out.push_back(here); // effectively straight; nothing to round
            continue;
        }

        const float turn = std::acos(cosTurn);

        // Never eat more than half of either neighbouring segment, or adjacent fillets would
        // overlap and the path would start cutting through its own corners.
        float d = std::min(inLen, outLen) * 0.5f;
        if (m_query.maxTurnRadius > 0.0f)
        {
            d = std::min(d, m_query.maxTurnRadius * std::tan(turn * 0.5f));
        }

        bool placed = false;
        for (int32_t attempt = 0; attempt < 3 && !placed; ++attempt, d *= 0.5f)
        {
            if (d < m_volume.Config().voxelSize * 0.25f)
            {
                break; // too small to be worth the extra points
            }

            const Vec3 p0 = here - inDir * d;
            const Vec3 p2 = here + outDir * d;

            std::vector<Vec3> arc;
            arc.reserve(static_cast<size_t>(samples) + 1);
            bool ok = true;

            for (int32_t k = 0; k <= samples; ++k)
            {
                const float t = static_cast<float>(k) / static_cast<float>(samples);
                const float u = 1.0f - t;
                const Vec3 b = p0 * (u * u) + here * (2.0f * u * t) + p2 * (t * t);

                bool missing = false;
                if (!IsTraversable(aView, m_volume.WorldToVoxel(b), missing))
                {
                    ok = false;
                    break;
                }
                // An elongated body has to fit along the curve too, tangent to it.
                if (TurnClearanceEnabled() && k > 0 && !BodyFits(aView, b, (b - arc.back()).Normalized()))
                {
                    ok = false;
                    break;
                }
                arc.push_back(b);
            }

            if (ok)
            {
                for (const Vec3& b : arc)
                {
                    out.push_back(b);
                }
                placed = true;
            }
        }

        if (!placed)
        {
            out.push_back(here); // no room to round it; keep the hard corner
        }
    }

    out.push_back(pts.back());
    m_result.points.swap(out);
}

void Planner::EmitWorldPoints(const std::vector<IVec3>& aPath)
{
    m_result.points.clear();
    m_result.points.reserve(aPath.size());
    for (const IVec3& v : aPath)
    {
        m_result.points.push_back(m_volume.VoxelCenter(v));
    }
}

void Planner::ApplySpline(PinnedView& aView)
{
    const auto& pts = m_result.points;
    if (pts.size() < 3)
    {
        return;
    }

    const int32_t samples = m_query.splineSamplesPerSegment;
    const auto n = static_cast<int32_t>(pts.size());
    const auto at = [&](int32_t i) { return pts[static_cast<size_t>(std::clamp(i, 0, n - 1))]; };

    std::vector<Vec3> curve;
    curve.reserve(pts.size() * static_cast<size_t>(samples) + 1);

    for (int32_t i = 0; i + 1 < n; ++i)
    {
        for (int32_t s = 0; s < samples; ++s)
        {
            const float t = static_cast<float>(s) / static_cast<float>(samples);
            curve.push_back(CatmullRom(at(i - 1), at(i), at(i + 1), at(i + 2), t));
        }
    }
    curve.push_back(pts.back());

    // A Catmull-Rom curve overshoots on tight turns and can clip a corner the polyline cleared.
    // Validate every sample; on any violation keep the polyline, which is known good.
    for (const Vec3& p : curve)
    {
        bool missing = false;
        if (!IsTraversable(aView, m_volume.WorldToVoxel(p), missing))
        {
            return;
        }
    }

    m_result.points.swap(curve);
}
} // namespace vnav
