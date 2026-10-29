// vnav - configuration, query and result types.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <vnav/Types.hpp>

namespace vnav
{
// Collision groups the sampler asks the world about. Values are a private bitmask; the game
// layer maps them onto CName collision groups. Defaults cover "solid world geometry".
enum class CollisionMask : uint32_t
{
    None = 0,
    Static = 1u << 0,
    Terrain = 1u << 1,
    Collider = 1u << 2,
    Destructible = 1u << 3,
    Dynamic = 1u << 4,
    Vehicle = 1u << 5,
    PlayerBlocker = 1u << 6,

    World = Static | Terrain | Collider | Destructible,
    WorldAndVehicles = World | Vehicle,
};

constexpr CollisionMask operator|(CollisionMask a, CollisionMask b)
{
    return static_cast<CollisionMask>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

constexpr bool HasFlag(CollisionMask v, CollisionMask f)
{
    return (static_cast<uint32_t>(v) & static_cast<uint32_t>(f)) != 0;
}

// One NavVolume = one independently configured region + agent profile. Agent radius is baked
// into the sampled data (probe boxes are inflated by it), so a volume serves exactly one agent
// size. Make a second volume for a second agent class - that is the intended usage.
struct VolumeConfig
{
    // Edge length of a voxel, in metres. Pick roughly the agent's radius: it bounds both the
    // memory and the granularity at which the sampler can resolve geometry.
    float voxelSize = 1.0f;

    // Agent's collision radius. Probe boxes are inflated by this, turning "does the agent fit"
    // into a single bit test at search time.
    float agentRadius = 0.5f;

    // Extra slack beyond agentRadius. Voxels that are free at agentRadius but blocked at
    // agentRadius+margin are marked `tight` and cost `tightnessPenalty` more to traverse,
    // so paths prefer open air over scraping along walls. 0 disables the second bitset.
    float margin = 1.0f;

    // Hard limit on where this volume will plan. Queries outside it fail with BadInput.
    AABB bounds{};

    CollisionMask collision = CollisionMask::World;

    // --- sampling budgets -----------------------------------------------------------------
    // Octree depth cap. 4 == subdivide to a single voxel (16 -> 8 -> 4 -> 2 -> 1).
    // Lower values make sampling cheaper and the map more conservative (over-blocked).
    int32_t maxProbeDepth = 4;

    // Hard cap on physics probes spent resolving one chunk. Refinement is breadth-first, so on
    // exhaustion the whole chunk degrades to a coarser, uniformly conservative map rather than
    // one arbitrary octant being sealed.
    //
    // ⚠️ EXHAUSTION IS NOT HARMLESS. An earlier version of this comment claimed dense geometry
    // "correctly resolves to solid"; it does not - it resolves to solid whether or not it IS
    // solid. This was 2048, and 2048 is below what full refinement costs, so any chunk needing
    // leaf-level detail everywhere silently over-blocked.
    //
    // The arithmetic: refining a 16^3 chunk to leaves costs 1+8+64+512+4096 = 4681 probes, plus up
    // to 4096 more for the second (hard-inflation) probe at leaf level - 8777 worst case, which is
    // exactly what a 4 m corridor was measured to need. Measured, same world, nothing else
    // changed: that corridor read as completely filled in at 2048 (every query Unreachable) and is
    // passable once the budget covers refinement. Narrow interior space is precisely where a low
    // budget lies, and it lies in the direction of "there is no way through".
    //
    // 9216 therefore covers the worst case rather than truncating it. This used to be capped low
    // because a chunk was sampled atomically inside one Pump, making it the worst-case single-frame
    // cost too; ServiceConfig::maxProbesPerChunkPerPump now bounds the frame cost separately, so
    // this one is free to be sized for correctness. Ordinary surfaces are unaffected - open sky
    // still costs 1 probe and a single plane ~940, because refinement stops as soon as a box tests
    // clear.
    //
    // Prefer attaching a bake regardless: it carries exact per-voxel occupancy and never approximates.
    int32_t maxProbesPerChunk = 9216;

    // --- residency ------------------------------------------------------------------------
    // LRU cap. At 16^3 voxels a Mixed chunk costs ~1KB; Empty/Solid chunks cost nothing.
    int32_t maxResidentChunks = 32768;

    // Chunks older than this are dropped during Pump. 0 disables age-based eviction.
    float chunkTtlSeconds = 0.0f;

    bool IsValid() const;
};

// "Get me out of here" - a search for open space, with no goal at all.
//
// Exists because a failed path query is not actionable. When every route out of a spot fails, the
// agent is told Unreachable and has nowhere to go, so it stops and never moves again - measured as
// 0 m of movement over 32 s, terminal. What breaks that deadlock is somewhere to fly TO, chosen
// purely on room rather than on progress toward a goal that is currently unreachable.
struct EscapeQuery
{
    Vec3 from{};

    // How far to look, in metres. Bounded work: this decides the voxel box that has to be sampled.
    float searchRadius = 16.0f;

    // Room required at the destination, in metres. 0 means "anywhere the agent merely fits", which
    // is the right bar when wedged in geometry. Raise it to insist on genuinely open air.
    float minClearance = 0.0f;

    // Refuse answers closer than this, in metres.
    //
    // Load-bearing for the second kind of stuck. If the agent is wedged, 0 is right and the nearest
    // free voxel is the answer. But if it is sitting in open space where every route to the goal
    // fails, the nearest qualifying position is where it already is - a correct answer that changes
    // nothing. Demanding real distance turns this into "go somewhere else and replan from there",
    // which is what actually escapes a local minimum.
    float minDistance = 0.0f;

    int32_t maxProbesPerQuery = 60000;

    bool IsValid() const;
};

struct EscapeResult
{
    Status status = Status::Invalid;

    Vec3 position{};      // where to go; only meaningful on Success
    float clearance = 0.0f; // room there, in metres (saturates - see PathResult::clearances)
    float distance = 0.0f;  // how far that is from `from`

    int32_t probes = 0;
    int32_t chunksSampled = 0;
    float elapsedMs = 0.0f;
};

enum class Smoothing : int32_t
{
    None = 0,       // raw voxel-centre waypoints
    StringPull = 1, // drop waypoints joined by clear line of sight (default)
    Spline = 2,     // string-pull, then resample as a Catmull-Rom curve

    // String-pull, then round each corner to a bounded turn radius.
    //
    // Differs from Spline in the way that matters for a vehicle: a Catmull-Rom curve interpolates
    // the waypoints and OVERSHOOTS on tight turns (which is why ApplySpline throws the whole curve
    // away if any sample is blocked), whereas a fillet cuts the corner and stays inside the
    // triangle formed by its two tangent points and the original vertex. So it can bound turn
    // radius, and it degrades per corner instead of all-or-nothing.
    //
    // Purely geometric - it needs the clearance field and nothing else, no mass, thrust or time -
    // which is what keeps it inside this library rather than in a vehicle controller.
    Fillet = 3,
};

struct PathQuery
{
    Vec3 start{};
    Vec3 goal{};

    // If start/goal land inside geometry, search outward this far (metres) for a free voxel.
    // 0 makes an occupied endpoint a hard failure.
    float snapRadius = 4.0f;

    // Return the best path found toward the goal when the goal itself cannot be reached.
    bool allowPartial = true;

    // A* node expansion cap. The dominant safety valve on search cost.
    int32_t maxExpansions = 60000;

    // Physics probes one query may cause. Only bounds the *synchronous* path; async queries are
    // bounded per-frame by the service budget instead.
    //
    // This is a safety valve, not a target. A chunk can cost up to maxProbesPerChunk, so this
    // buys roughly 30 fresh chunks - fine for a short hop through already-mapped space, nowhere
    // near enough to explore a city block. Long-range planning belongs on the async path; a sync
    // query that exhausts this returns whatever partial path it had.
    //
    // KEEP THIS ~30x maxProbesPerChunk. The two are coupled through that sentence, and raising the
    // per-chunk budget alone silently shrinks what a query can reach: at 60000 against a 9216
    // per-chunk ceiling a query buys SIX chunks, and previously-passing queries start returning
    // partial paths for no visible reason. Caught exactly that way when the per-chunk budget was
    // raised to stop narrow space over-blocking.
    //
    // Worst case, not typical: open sky costs 1 probe and an ordinary surface ~940, so real
    // queries spend a small fraction of this. It only has to be large enough that a pathological
    // chunk cannot consume the whole allowance.
    int32_t maxProbesPerQuery = 276480;

    // --- hierarchical guidance ----------------------------------------------------------------
    // Plan a coarse route over whole CHUNKS first, then bias the voxel search toward it.
    //
    // Cheap because it needs no new data: a bake's index already records one state byte per chunk
    // and treats absent-but-covered as open sky, so a whole-map connectivity graph is available for
    // the cost of hash lookups - no probing, no decoding, no file reads. Requires an attached bake;
    // without one there is nothing to build the coarse graph from and this does nothing.
    //
    // A* over a 16 m grid reaches a distant goal in a few hundred expansions where the voxel search
    // needs tens of thousands, which is what lifts the range ceiling.
    bool coarseGuidance = true;

    // Cost multiplier for voxel steps that leave the coarse corridor.
    //
    // A BIAS, deliberately not a restriction. Hard-restricting the fine search to the corridor
    // would make it incomplete: the coarse graph works in 16 m blocks, so a chunk it thinks is
    // passable can be impassable at voxel resolution, and the query would fail even though a route
    // exists. A penalty keeps the search complete - it will leave the corridor if it must - while
    // still strongly preferring it. Costs only ever increase, so the Euclidean heuristic stays
    // admissible and the result stays optimal for the modified cost.
    //
    // 1.0 disables the bias without disabling the corridor computation.
    float coarsePenalty = 4.0f;

    // Cap on coarse-graph expansions. Chunks are 16 m, so this reaches a long way.
    int32_t maxCoarseExpansions = 20000;

    // > 1.0 weights the heuristic, trading optimality for speed (greedier, fewer expansions).
    float heuristicWeight = 1.0f;

    // Multiplier applied when either end of an edge is in `tight` space.
    float tightnessPenalty = 1.35f;

    // Multiplier on the vertical component of edge cost. > 1 discourages climbing/diving,
    // which keeps flight paths from bobbing. 1.0 makes movement isotropic.
    float verticalCostMultiplier = 1.2f;

    // Search from both ends and meet in the middle.
    //
    // Two distinct wins, and the second is the one that motivated it: a bidirectional search
    // explores far fewer nodes on long routes (two shallow frontiers instead of one deep one),
    // AND it detects an unreachable goal quickly - a goal sealed inside geometry exhausts its own
    // small backward frontier almost immediately, instead of the forward search having to explore
    // everything reachable before it can conclude anything.
    //
    // The meeting point is a hard vertex where the two halves join, so the path is string-pulled
    // afterwards to smooth the seam (Smoothing::None therefore leaves a visible corner there).
    bool bidirectional = true;

    // --- elongated agents -------------------------------------------------------------------
    // Length of the agent along its direction of travel, in metres. 0 (default) treats it as a
    // sphere, which is the historical behaviour.
    //
    // Why this exists: LineOfSight already sweeps a sphere along each segment, and a swept sphere
    // IS a capsule - so on a STRAIGHT run an elongated body is already covered, and `agentRadius`
    // should be its CROSS-SECTION half-extent (the larger of half-width and half-height), never
    // half its length. Sizing the radius to the length just makes it refuse gaps it could fly.
    //
    // The real gap is at BENDS: a long body rotating through a corner sweeps a volume strictly
    // larger than the capsule, so nose and tail swing outside the validated corridor. Setting this
    // makes the smoother verify the body can actually rotate through each corner it creates, and
    // report any it could not fix via PathResult::infeasibleTurns.
    //
    // Lives on the query rather than the volume because, unlike agentRadius, it is not baked into
    // the sampled data - it is consulted only while smoothing. That also means it can vary per
    // query against a shared volume, and costs nothing when left at 0.
    //
    // Scope: this constrains SMOOTHING, not the search - the search still plans for the capsule.
    // For orientation as a genuine search dimension, use yawBuckets below instead.
    float agentLength = 0.0f;

    // Intermediate orientations tested when checking whether an elongated agent can rotate
    // through a corner. Only used when agentLength > 0. More samples catch tighter failures; each
    // costs a short walk along the body axis.
    int32_t turnSamples = 5;

    // Orientation as a genuine SEARCH DIMENSION: 0 (default) plans in 3D, anything >= 4 plans in
    // 4D over (x, y, z, yaw) with this many yaw buckets. Capped at kMaxYawBuckets.
    //
    // This is the expensive tier, and it buys something agentLength alone cannot: agentLength only
    // constrains smoothing, so it can reject a corner but never DISCOVER that a long body has to
    // be aligned a particular way to fit through a gap at all. 4D searches orientation, so it can
    // route a long body lengthwise through a slot too narrow to cross sideways, and can decide to
    // rotate in place before entering one.
    //
    // Costs: the state space is multiplied by yawBuckets, and any-angle shortcutting is disabled
    // (Lazy Theta* assumes it can jump straight to a grandparent, which does not compose with an
    // orientation constraint along that line), so paths come back as grid moves before smoothing.
    // Reach for it when agents must thread genuinely tight interior spaces; leave it off for
    // mostly-open airspace, where agentLength gets most of the benefit for a fraction of the cost.
    //
    // Requires agentLength > 0 to mean anything - with a sphere agent every yaw is identical.
    int32_t yawBuckets = 0;

    // Cost of rotating one yaw bucket, in metres-equivalent. Trades turning against travelling:
    // raise it and the search prefers to fly around rather than pivot.
    float yawCost = 0.5f;

    // Require the body to point roughly where it is going, instead of letting it crab sideways.
    //
    // Without this, 4D is correct but useless in open space: yaw is unconstrained wherever the
    // body fits at every angle, rotating costs yawCost and buys nothing, so the search leaves the
    // heading at its initial bucket and the returned plan is all zeros. Measured exactly that
    // before adding it.
    //
    // Still purely geometric - no mass, thrust or time - so it stays inside the library's scope.
    // It only decides which (position, orientation) transitions are legal.
    bool yawFollowsTravel = true;

    // Half-angle of the cone, in radians, between the body's heading and its horizontal direction
    // of travel. Must be at least one bucket wide or the search cannot move at all; pi/4 suits the
    // common 8-bucket case. Purely vertical moves are exempt - they have no horizontal direction
    // to align with.
    float yawTravelCone = 0.785398f;

    Smoothing smoothing = Smoothing::StringPull;

    // Points emitted per segment when smoothing == Spline.
    int32_t splineSamplesPerSegment = 4;

    // --- Smoothing::Fillet ------------------------------------------------------------------
    // Largest turn radius a fillet will use, in metres. 0 means "as round as will fit", bounded
    // only by the adjacent segment lengths and by what the geometry actually allows.
    //
    // A fillet is always shrunk until it validates, so this is a preference rather than a
    // guarantee: a corner in tight space simply gets a smaller radius, and one with no room at
    // all is left as a hard vertex.
    float maxTurnRadius = 0.0f;

    // Points emitted per rounded corner. More is smoother and costs more validation.
    int32_t filletSamples = 6;

    // Chunk radius prefetched along the straight start->goal corridor before searching.
    int32_t prefetchRadiusChunks = 1;

    // Wall-clock cap for an async job, counted from submission. 0 disables.
    //
    // Generous on purpose. An async search is invisible to the player - nothing is waiting on it
    // - so it is worth spending real time to return a correct path instead of a fast wrong one.
    // This is a backstop, not the primary bound: noProgressParks below is what actually stops a
    // search that is going nowhere.
    float timeoutSeconds = 30.0f;

    // --- async persistence ----------------------------------------------------------------
    // Both are ASYNC ONLY. The sync path samples chunks inline and never parks, so it is bounded
    // by maxExpansions / maxProbesPerQuery and fails fast - that is the whole point of asking
    // for a path synchronously.

    // Parks (game-thread round trips to make chunks resident) before an async search gives up.
    // 0 == unlimited, which is the default: the search continues until it finds a path, proves
    // the goal unreachable, stalls, is cancelled, or times out.
    //
    // This used to be a hard-coded 512 and it was the real reason long searches in dense geometry
    // returned nothing: at roughly one park per frame it killed the search after ~8.5 s at 60 fps,
    // starved on chunk delivery rather than short of compute. Letting it run is safe because the
    // game-thread cost is already capped per frame by ServiceConfig::defaultPumpBudgetMicros - a
    // search that runs for a minute costs the same per frame as one that runs for a second.
    int32_t maxParks = 0;

    // Livelock guard, and the primary bound in unlimited mode: give up after this many
    // CONSECUTIVE parks without the search getting any closer to the goal.
    //
    // Bounding by PROGRESS rather than by effort is the important part. A long, legitimate search
    // through complex geometry keeps improving and runs as long as it needs; one that is trapped
    // in a sealed region (an agent under a floor slab, a goal inside geometry) never improves and
    // terminates instead of grinding out the full timeout. 0 disables.
    //
    // Deliberately the same number as the old hard park cap, which makes this change a pure
    // relaxation: a search that used to die after 512 parks TOTAL now only dies after 512
    // CONSECUTIVE parks with no improvement, and any progress at all resets the counter. Nothing
    // that used to succeed can start failing.
    //
    // Do not tighten this casually. `m_bestH` only improves when a node beats the closest
    // distance-to-goal seen so far, and a search rounding a large building legitimately moves
    // sideways or away for a long stretch without improving - which is exactly the hard case this
    // is meant to let succeed. Lower it only if you want trapped agents to give up sooner and are
    // willing to lose long detours to get that.
    int32_t noProgressParks = 512;

    bool IsValid() const;
};

struct PathResult
{
    Status status = Status::Invalid;

    // World-space waypoints, start-inclusive and goal-inclusive. Empty unless IsUsable(status).
    std::vector<Vec3> points;

    // Body heading at each waypoint, in radians about the vertical (Z) axis. Same length as
    // `points` when a 4D (yawBuckets > 0) search produced this path; EMPTY otherwise.
    //
    // For a 4D plan this is part of the answer, not a derived convenience: the search may have
    // decided to rotate in place before entering a gap, and that intent is only visible here.
    std::vector<float> yaws;

    // Free radius at each waypoint, in metres. Always the same length as `points`.
    //
    // The search already tests this value at every node it touches, so returning it is nearly
    // free - and without it a consumer that wants to cut a corner, bank into a turn, or slow down
    // for a gap has to re-enter the library and re-derive room it was just told about. This is
    // the difference between handing back a polyline and handing back a polyline plus how much
    // space there is at each vertex.
    //
    // ⚠️ THIS SATURATES, and usually quite close in. It is a "how tight is it here" signal, NOT a
    // measurement of open space. Two independent reasons, both deliberate:
    //
    //   1. The clearance field is only computed out to the volume's TIGHT THRESHOLD
    //      (agentRadius + margin). Beyond that every value compares identically for traversability,
    //      so the loader stops and writes kMaxClearance - see the halo cut-off in BakedNav. With
    //      agentRadius 0.5 and margin 0.5 that is ~2 voxels, so a waypoint 4 voxels from anything
    //      already reads saturated. Raise `margin` to buy more range; it costs a wider halo, and
    //      therefore more neighbouring chunks per chunk load.
    //   2. Probe-sampled chunks are coarser still. A physics probe answers one bit - "is this
    //      inflated box clear?" - so the sampler can only write three levels: 0 (blocked),
    //      roughly agentRadius+margin (squeezing), or saturated. Baked chunks do carry a real EDT
    //      and so grade properly, but only below the cut-off in (1).
    //
    // Read it as "at least this much room", never as "exactly this much". That is the right
    // direction for sizing a manoeuvre, and it is exactly the information needed to slow down for
    // a gap or refuse to cut a corner - which is what it is for. It is not a rangefinder.
    std::vector<float> clearances;

    // Total length of `points`, in metres.
    float length = 0.0f;

    // Where the query actually started/ended after snapping out of geometry.
    Vec3 snappedStart{};
    Vec3 snappedGoal{};
    bool startWasSnapped = false;
    bool goalWasSnapped = false;

    // --- diagnostics ----------------------------------------------------------------------
    int32_t expansions = 0;    // A* nodes popped

    // Chunk-level nodes popped by the hierarchical pass, and how many chunks its corridor covers.
    // corridorChunks == 0 means guidance did not apply: switched off, no bake, endpoints in the
    // same chunk, or no coarse route found.
    int32_t coarseExpansions = 0;
    int32_t corridorChunks = 0;
    int32_t probes = 0;        // physics probes attributed to this query
    int32_t chunksSampled = 0; // chunks this query caused to be voxelised
    int32_t parkCount = 0;     // times the job suspended waiting for the game thread
    float elapsedMs = 0.0f;

    // Corners an elongated agent (VolumeConfig::agentLength > 0) cannot rotate through. The
    // smoother avoids creating these, so a non-zero count means the underlying route itself is
    // too tight for the body - a genuine "it does not fit", not a smoothing artefact. Always 0
    // for a sphere agent.
    int32_t infeasibleTurns = 0;

    void Clear();
};

struct ServiceConfig
{
    // 0 == auto: clamp(hardware_concurrency / 4, 1, 4). Kept small on purpose - the game
    // already saturates the machine with its own job system.
    int32_t workerCount = 0;

    // Default per-frame sampling budget, in microseconds of game-thread time.
    int32_t defaultPumpBudgetMicros = 1500;

    // Probes one chunk may spend within a single pump before being suspended and resumed next
    // frame. Bounds the cost of ONE chunk, where defaultPumpBudgetMicros bounds the whole pump.
    //
    // Both are needed. The time budget alone could not stop a single expensive chunk overrunning a
    // frame, because sampling was atomic - the pump had to finish whatever chunk it started. That
    // is what forced maxProbesPerChunk to stay at a value too low to resolve narrow space (see the
    // warning there). This is what decouples them.
    //
    // 0 disables suspension, restoring the old atomic behaviour.
    int32_t maxProbesPerChunkPerPump = 512;

    // Completed jobs are kept this long so a caller can still poll their result, then reaped.
    float resultRetentionSeconds = 30.0f;

    // Upper bound on live jobs; submissions beyond it fail fast rather than queueing forever.
    int32_t maxConcurrentJobs = 256;
};

struct Stats
{
    int64_t totalQueries = 0;
    int64_t succeeded = 0;
    int64_t partial = 0;
    int64_t failed = 0;

    int64_t totalProbes = 0;
    int64_t totalChunksSampled = 0;

    // Chunks served from a bake rather than probed. The ratio against totalChunksSampled is the
    // clearest signal that a bake is actually attached and covering where the agents are.
    int64_t chunksFromBake = 0;

    int32_t residentChunks = 0;
    int64_t residentBytes = 0;

    int32_t jobsPending = 0;
    int32_t jobsParked = 0;
    int32_t jobsRunning = 0;

    int32_t pendingChunkRequests = 0;
};
} // namespace vnav
