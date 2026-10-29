// vnav - Lazy Theta* over the voxel occupancy, with suspend/resume.
#pragma once

#include <atomic>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <vnav/Config.hpp>
#include <vnav/Types.hpp>

#include "core/Volume.hpp"

namespace vnav
{
enum class SliceResult
{
    Finished,   // m_result is terminal; nothing more to do
    NeedsWorld, // NeededChunks() must be sampled before calling Step again
    Cancelled,
};

// A resumable search.
//
// All state lives in the object, so suspending is just "return NeedsWorld" and resuming is just
// "call Step again". Nothing is torn down or rebuilt across a park, which is what makes the
// demand-driven sampling loop affordable.
//
// Runs entirely on a worker thread. It reads the world only through PinnedView and never touches
// IWorldProbe.
class Planner
{
public:
    Planner(Volume& aVolume, const PathQuery& aQuery);

    // Advance the search as far as it can go. Safe to call repeatedly after NeedsWorld.
    SliceResult Step(PinnedView& aView, const std::atomic<bool>& aCancelled);

    // Stop early and keep whatever the search already found. Used when an external budget runs
    // out (probe budget, wall-clock timeout) rather than an internal one: without this the
    // caller would have to throw away a perfectly good partial path just because it could not
    // afford to keep going. No-op once the planner is finished.
    void Abort(PinnedView& aView, Status aReason);

    bool IsDone() const { return m_phase == Phase::Done; }

    // Straight-line distance from the closest node the search has reached to the goal. Decreases
    // as the search closes in, so the caller can tell a search that is working from one that is
    // stuck without knowing anything about the search itself. Used to bound async searches by
    // progress rather than by effort.
    //
    // Reports the forward side only. The backward side closing on the start is progress too, but
    // mixing them would let one side mask the other stalling, and the forward number is the one
    // that corresponds to "how close is the agent to its goal".
    float BestHeuristic() const { return m_fwd.bestH; }

    // Chunks that must become resident before the next Step. Cleared by the caller once queued.
    const std::vector<uint64_t>& NeededChunks() const { return m_needed; }
    void ClearNeeded() { m_needed.clear(); }

    // Chunks along the straight start->goal corridor, queued at submission so the search rarely
    // has to park. Best-effort: the planner does not wait on these.
    void CollectPrefetch(std::vector<uint64_t>& aOut) const;

    const PathResult& Result() const { return m_result; }
    PathResult& MutableResult() { return m_result; }

private:
    enum class Phase
    {
        Init,   // resolve + snap the endpoints
        Search, // A*
        Done,
    };

    struct NodeRec
    {
        float g = 0.0f;
        uint64_t parent = 0;
        bool closed = false;
    };

    struct HeapItem
    {
        float f;
        uint64_t key;
    };

    // One side of the search. A unidirectional query runs `m_fwd` alone and simply never seeds
    // `m_bwd`, so there is exactly one expansion code path rather than two implementations to
    // keep in agreement.
    struct Frontier
    {
        std::unordered_map<uint64_t, NodeRec> nodes;
        std::vector<HeapItem> open;

        uint64_t rootKey = 0; // where this side started
        IVec3 target{};       // what its heuristic measures toward

        // Closest approach to `target`, for partial results.
        uint64_t bestKey = 0;
        float bestH = 0.0f;

        // False for the backward side, and load-bearing. The edge tests are NOT symmetric:
        // CanStep checks IsTraversable on its DESTINATION and derives its corner-cut probes from
        // its SOURCE, and SegmentCost applies tightnessPenalty to its DESTINATION. A backward
        // frontier that presented edges in expansion order would therefore accept diagonal steps
        // the forward search rejects - i.e. paths that clip geometry. Every edge is instead
        // presented in FORWARD-TRAVERSAL order via StepAllowed/StepCost.
        bool forward = true;

        bool active = false;
    };

    // --- helpers ---------------------------------------------------------------------------
    bool EnsureBoxResident(PinnedView& aView, const IVec3& aMinVoxel, const IVec3& aMaxVoxel);
    bool EnsureNeighbourhoodResident(PinnedView& aView, const IVec3& aVoxel);
    void RequestChunkWithHalo(const IVec3& aChunkCoord);

    // Nearest free voxel to `aVoxel` within snapRadius. Returns false if none / not resident.
    bool SnapToFree(PinnedView& aView, const IVec3& aVoxel, IVec3& aOut, bool& aMissing) const;

    bool IsTraversable(PinnedView& aView, const IVec3& aVoxel, bool& aMissing) const;
    bool CanStep(PinnedView& aView, const IVec3& aFrom, const IVec3& aTo, bool& aMissing) const;
    bool LineOfSight(PinnedView& aView, const IVec3& aFrom, const IVec3& aTo) const;

    float SegmentCost(PinnedView& aView, const IVec3& aFrom, const IVec3& aTo) const;
    float Heuristic(const IVec3& aVoxel, const IVec3& aTarget) const;

    // Present an edge in forward-traversal order whichever way the frontier is growing. `aFrom`
    // and `aTo` are always given in EXPANSION order (from the node being expanded, to the
    // neighbour); these flip them for the backward side, where the real traversal runs the other
    // way. See Frontier::forward for why this is not cosmetic.
    bool StepAllowed(const Frontier& aF, PinnedView& aView, const IVec3& aFrom, const IVec3& aTo,
                     bool& aMissing) const;
    float StepCost(const Frontier& aF, PinnedView& aView, const IVec3& aFrom, const IVec3& aTo) const;

    enum class ExpandResult
    {
        Continued,  // expanded (or skipped a stale entry); keep going
        NeedsWorld, // parked on chunk residency
        Exhausted,  // open list empty: nothing more is reachable from this side
    };

    ExpandResult ExpandOne(Frontier& aF, Frontier& aOther, PinnedView& aView);
    bool TopF(const Frontier& aF, float& aOut) const;

    void PushOpen(Frontier& aF, uint64_t aKey, float aPriority);
    bool PopOpen(Frontier& aF, HeapItem& aOut);

    void Finish(Status aStatus, PinnedView& aView, uint64_t aEndKey);
    void BuildPath(PinnedView& aView, uint64_t aEndKey);
    void BuildMeetingPath(PinnedView& aView);
    // --- elongated-agent turn clearance ----------------------------------------------------
    // Does the body fit, centred at aPos and pointing along aHeading? Walks the body axis and
    // asks the blocked/free test at each sample.
    //
    // Deliberately NOT a comparison against the clearance field: that field is only computed out
    // to the volume's tight threshold and saturates a couple of voxels from geometry, so it
    // cannot answer "does a 10 m body fit" at all. The traversability bit stays exact at any
    // distance, which is why the check is a walk rather than a lookup.
    bool BodyFits(PinnedView& aView, const Vec3& aPos, const Vec3& aHeading) const;

    // Can the body rotate from aFrom to aTo while centred at aPos? Samples intermediate headings.
    bool TurnFits(PinnedView& aView, const Vec3& aPos, const Vec3& aFrom, const Vec3& aTo) const;

    bool TurnClearanceEnabled() const { return m_query.agentLength > 0.0f; }

    // --- hierarchical guidance ---------------------------------------------------------------
    // A* over whole chunks, using only the bake index's per-chunk state byte. Fills m_corridor
    // with the chunks along the coarse route, dilated so the fine search has room to manoeuvre.
    // No-op without a bake. Runs once, during Init.
    void BuildCorridor();

    // True when the voxel is outside the coarse corridor and should therefore cost more.
    bool OutsideCorridor(const IVec3& aVoxel) const;

    // --- 4D (x,y,z,yaw) search -------------------------------------------------------------
    // Node keys carry a yaw bucket in 4D and do not in 3D, so every key round-trip inside the
    // planner goes through these three rather than Pack/UnpackVoxelKey directly.
    bool Is4D() const { return m_yawBuckets > 0; }
    uint64_t MakeKey(const IVec3& aVoxel, int32_t aYaw) const;
    IVec3 KeyVoxel(uint64_t aKey) const;
    int32_t KeyYaw(uint64_t aKey) const;

    // Heading of a yaw bucket, as a unit vector. Yaw is rotation about the vertical (Z) axis.
    Vec3 YawHeading(int32_t aYaw) const;

    // Does the body fit centred on this voxel at this orientation?
    bool BodyFitsAtYaw(PinnedView& aView, const IVec3& aVoxel, int32_t aYaw) const;

    void StringPull(PinnedView& aView, std::vector<IVec3>& aPath) const;
    void EmitWorldPoints(const std::vector<IVec3>& aPath);
    void ApplySpline(PinnedView& aView);
    void ApplyFillet(PinnedView& aView);
    void FinalisePath(PinnedView& aView);

    Volume& m_volume;
    PathQuery m_query;
    PathResult m_result;

    Phase m_phase = Phase::Init;

    IVec3 m_startVoxel{};
    IVec3 m_goalVoxel{};
    uint64_t m_startKey = 0;
    uint64_t m_goalKey = 0;

    Frontier m_fwd; // seeded at the start, aims at the goal
    Frontier m_bwd; // seeded at the goal, aims at the start. Inactive unless bidirectional.

    // Best node where the two frontiers have touched, and the cost of going through it. The
    // search does not stop at the first touch: a meeting only bounds the answer from above, so
    // it keeps going until neither side can beat it (see the f-sum test in Step).
    uint64_t m_meetKey = 0;
    float m_meetCost = FLT_MAX;
    bool m_haveMeet = false;

    int32_t m_expansions = 0;
    int32_t m_snapVoxelRadius = 0;

    // 0 == plain 3D. Clamped from PathQuery::yawBuckets at construction.
    int32_t m_yawBuckets = 0;

    // Chunks along the coarse route. Empty means "no guidance" - either it was switched off, there
    // is no bake, or the coarse search found nothing - and every lookup then reports "inside", so
    // the fine search runs exactly as it did before.
    std::unordered_set<uint64_t> m_corridor;
    int32_t m_coarseExpansions = 0;

    std::vector<uint64_t> m_needed;
};
} // namespace vnav
