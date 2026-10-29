// vnav - the public facade. One service owns every volume, the worker pool and the job registry.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <vnav/Config.hpp>
#include <vnav/Types.hpp>
#include <vnav/WorldProbe.hpp>

namespace vnav
{
// Invoked on the GAME THREAD from inside Pump(), never on a worker. That restriction exists so a
// callback can safely touch RTTI / script state.
using PathCallback = std::function<void(uint32_t aJobId, const PathResult&)>;

class NavService
{
public:
    explicit NavService(const ServiceConfig& aConfig = {});
    ~NavService();

    NavService(const NavService&) = delete;
    NavService& operator=(const NavService&) = delete;

    // ---------------------------------------------------------------------------------------
    // Lifecycle. Call from the game thread.
    // ---------------------------------------------------------------------------------------

    // Marks the calling thread as the game thread. FindPathSync uses this to decide whether it
    // may sample inline or must block waiting for a pump. Call once, early.
    void SetGameThread();

    // The probe is borrowed, not owned; it must outlive the service or be cleared first.
    // Passing nullptr installs a probe that reports empty space everywhere.
    void SetProbe(IWorldProbe* aProbe);

    // Cancels every job, joins the workers. Idempotent; also run by the destructor.
    void Shutdown();

    // ---------------------------------------------------------------------------------------
    // Volumes. Safe from any thread.
    // ---------------------------------------------------------------------------------------

    bool CreateVolume(const std::string& aName, const VolumeConfig& aConfig);

    // Attaches a baked static world to a volume. The bake is the authority for geometry the
    // physics scene cannot see - anything outside the streaming radius - and the probe stays on
    // as the layer that knows about doors, vehicles and destruction.
    //
    // Fails if the volume does not exist, the file cannot be read, or the volume's agent is too
    // large for the transform the loader will run; aError says which.
    bool AttachBake(const std::string& aVolume, const std::string& aPath, std::string& aError);

    // Attach, but refuse a bake that was built from a different game build.
    //
    // A bake describes the world as it shipped. Patch the game so geometry moves, keep the old
    // bake, and the planner routes confidently through walls that are no longer there - silently,
    // because nothing about the data looks wrong. This turns that into a load-time error.
    //
    // aExpectedGameBuild is supplied by the caller rather than detected here, because vnav_core
    // deliberately knows nothing about the game. Pass the running build string.
    //
    // An UNSTAMPED bake (written before stamping existed) cannot be checked, so it is accepted and
    // reported through aError as a warning with the attach still succeeding - refusing those would
    // break every bake already shipped.
    bool AttachBakeChecked(const std::string& aVolume, const std::string& aPath,
                           const std::string& aExpectedGameBuild, std::string& aError);

    // The game build the attached bake was made from; empty if unstamped or no bake.
    std::string BakeGameBuild(const std::string& aVolume) const;

    bool HasBake(const std::string& aVolume) const;
    void DetachBake(const std::string& aVolume);
    bool DestroyVolume(const std::string& aName);
    bool HasVolume(const std::string& aName) const;
    bool GetVolumeConfig(const std::string& aName, VolumeConfig& aOut) const;
    std::vector<std::string> GetVolumeNames() const;

    // ---------------------------------------------------------------------------------------
    // Queries.
    // ---------------------------------------------------------------------------------------

    // Queue a search. Returns a job id, or 0 if the volume is unknown, the query is malformed or
    // the service is at maxConcurrentJobs. Safe from any thread.
    //
    // `aCallback` (optional) fires once, on the game thread, during the Pump that observes the
    // job finish. Polling and callbacks can be mixed freely.
    uint32_t SubmitAsync(const std::string& aVolume, const PathQuery& aQuery, PathCallback aCallback = {});

    // Read a job's current state. Non-consuming: poll as often as you like. `aOut` is only
    // populated once the status is terminal. Returns Status::Invalid for unknown/reaped ids.
    // Safe from any thread.
    Status Poll(uint32_t aJobId, PathResult& aOut);

    // Cooperative cancel. Returns false if the job already finished or never existed.
    bool Cancel(uint32_t aJobId);

    // Blocking search.
    //   - On the game thread: samples inline as needed, bounded by the query's own budgets.
    //     Never deadlocks, but does spend game-thread time proportional to the work.
    //   - Off the game thread: submits and waits for the game thread to pump. Requires that
    //     something is still calling Pump(), and honours PathQuery::timeoutSeconds.
    Status FindPathSync(const std::string& aVolume, const PathQuery& aQuery, PathResult& aOut);

    // ---------------------------------------------------------------------------------------
    // World maintenance. Call from the game thread.
    // ---------------------------------------------------------------------------------------

    // Drop cached chunks overlapping `aBox` so they are re-sampled on next use. Use after
    // anything that changes collision: doors, destruction, streaming, moved vehicles.
    // Jobs already in flight are unaffected - they pinned the chunks they are walking.
    // Where to resume a freshly-arrived path, given where the agent actually is.
    //
    // SHIP THIS RATHER THAN LETTING EVERY CONSUMER REDISCOVER IT. Continuous replanning has a
    // trap that looks like a physics or steering bug: a query takes 0.2-1.7 s to come back, the
    // agent has moved on in the meantime, and a follower that restarts the new path at waypoint 0
    // turns around and flies BACK to where it was when it asked. Measured in the test harness:
    // 122 path adoptions, 72% followed by a ~180 degree reversal (heading dot ~= -1.00), peak
    // deviation 7.9 m. Adding this dropped reversals from 206 to 0.
    //
    // Returns the index of the waypoint to steer at: the far end of the segment nearest the agent,
    // so progress already made is not thrown away. Returns 0 for a path with fewer than 2 points.
    //
    // Pure geometry over the returned points - no service state - so it is safe to call from
    // anywhere, at any time, on any thread.
    static int32_t ResumeIndex(const std::vector<Vec3>& aPoints, const Vec3& aAgentPosition);

    // Can this volume's agent occupy this exact point? Samples the containing chunk if it is not
    // already resident, so this is a real answer rather than "not loaded, therefore no".
    //
    // Exists mostly to stop callers manufacturing unreachable goals. Offsetting a goal - hovering
    // above a target, standing off from a wall - is unremarkable outdoors and puts the goal inside
    // the ceiling indoors, and the resulting query failure looks like a pathfinding bug rather than
    // a bad goal. Check the point first and fall back if it is solid.
    //
    // Game thread only. Consults the dynamic obstacle layer as well as the static map.
    bool IsFree(const std::string& aVolume, const Vec3& aPoint);

    // Find open space near a point, with no goal - the recovery for an agent that has nowhere to
    // go. See EscapeQuery for why a failed path query alone leaves an agent frozen.
    //
    // Game thread only, and synchronous: the work is bounded by searchRadius, and an agent that is
    // stuck is not doing anything else anyway. Returns Success, Unreachable (nothing qualifying in
    // range), or BudgetExhausted (ran out of probes before it could finish looking).
    Status FindEscape(const std::string& aVolume, const EscapeQuery& aQuery, EscapeResult& aOut);

    void InvalidateBox(const std::string& aVolume, const AABB& aBox);

    // Replace the volume's moving obstacles wholesale. Cheap enough to call every frame: it
    // builds a fresh spatial hash and swaps one handle.
    //
    // Use this for anything that MOVES - other vehicles, NPCs, opened doors, moving platforms.
    // Do NOT use InvalidateBox for those: that re-samples at chunk granularity (16 voxels a side)
    // and writes the result into the cached static map, so a car moving a metre would re-voxelise
    // whole chunks every frame and corrupt the bake's static answer while doing it. These boxes
    // instead sit in a separate layer consulted after the static test, leaving chunks immutable.
    //
    // Boxes are inflated by the volume's agentRadius on the way in, so pass the obstacle's own
    // extents. Passing an empty list clears the layer.
    void SetDynamicObstacles(const std::string& aVolume, const std::vector<AABB>& aBoxes);

    size_t DynamicObstacleCount(const std::string& aVolume) const;
    void InvalidateAll(const std::string& aVolume);
    void InvalidateAllVolumes();

    // ---------------------------------------------------------------------------------------
    // The pump. GAME THREAD ONLY.
    // ---------------------------------------------------------------------------------------

    // Services one frame's worth of work:
    //   1. drain the chunk-sample queue, spending at most `aBudgetMicros` on physics probes
    //   2. wake jobs whose chunks have become resident
    //   3. run completion callbacks
    //   4. evict stale chunks and reap old results
    //
    // Pass -1 for ServiceConfig::defaultPumpBudgetMicros. Safe to call with no probe attached
    // (steps 2-4 still run). Cheap when idle.
    void Pump(int32_t aBudgetMicros = -1);

    Stats GetStats() const;
    void ResetStats();

private:
    struct Impl;
    std::unique_ptr<Impl> m;
};
} // namespace vnav
