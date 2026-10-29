// vnav - redscript / CET facing API, registered as RTTI globals.
//
// Deliberately scalar-only. Nothing here marshals a struct or an array across the RTTI
// boundary: volume setup takes plain parameters, search tuning lives on the volume, and paths
// are read back by handle + index. That keeps the binding surface small enough to be obviously
// correct, and it tolerates the library gaining options without breaking existing scripts.

#include <string>

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Scripting/Natives/Generated/Vector4.hpp>

#include <vnav/Config.hpp>
#include <vnav/NavService.hpp>

#include "game/Log.hpp"
#include "game/Runtime.hpp"

namespace
{
using namespace vnav;
using vnav::game::kSyncHandleBit;
using vnav::game::Runtime;

std::string NameToString(RED4ext::CName aName)
{
    const char* s = aName.ToString();
    return (s && *s) ? std::string(s) : std::to_string(aName.hash);
}

Vec3 ToVec3(const RED4ext::Vector4& v)
{
    return Vec3{v.X, v.Y, v.Z};
}

RED4ext::Vector4 ToVector4(const Vec3& v)
{
    return RED4ext::Vector4{v.x, v.y, v.z, 1.0f};
}

bool ResolveResult(uint32_t aHandle, PathResult& aOut)
{
    return Runtime::Get().ResolveResult(aHandle, aOut);
}

// -------------------------------------------------------------------------------------------
// Volumes
// -------------------------------------------------------------------------------------------

void VNavCreateVolume(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName name;
    float voxelSize = 1.0f;
    float agentRadius = 0.5f;
    float margin = 1.0f;
    RED4ext::Vector4 boundsMin{};
    RED4ext::Vector4 boundsMax{};
    uint32_t collisionMask = 0;

    RED4ext::GetParameter(aFrame, &name);
    RED4ext::GetParameter(aFrame, &voxelSize);
    RED4ext::GetParameter(aFrame, &agentRadius);
    RED4ext::GetParameter(aFrame, &margin);
    RED4ext::GetParameter(aFrame, &boundsMin);
    RED4ext::GetParameter(aFrame, &boundsMax);
    RED4ext::GetParameter(aFrame, &collisionMask);
    aFrame->code++; // ParamEnd

    Runtime& rt = Runtime::Get();
    bool ok = false;

    if (rt.IsRunning())
    {
        VolumeConfig cfg;
        cfg.voxelSize = voxelSize;
        cfg.agentRadius = agentRadius;
        cfg.margin = margin;
        cfg.bounds = AABB{ToVec3(boundsMin), ToVec3(boundsMax)};
        cfg.collision = collisionMask == 0 ? CollisionMask::World : static_cast<CollisionMask>(collisionMask);
        ok = rt.Nav().CreateVolume(NameToString(name), cfg);
    }

    if (aOut)
    {
        *aOut = ok;
    }
}

void VNavDestroyVolume(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName name;
    RED4ext::GetParameter(aFrame, &name);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    const bool ok = rt.IsRunning() && rt.Nav().DestroyVolume(NameToString(name));
    if (aOut)
    {
        *aOut = ok;
    }
}

void VNavHasVolume(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName name;
    RED4ext::GetParameter(aFrame, &name);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    const bool ok = rt.IsRunning() && rt.Nav().HasVolume(NameToString(name));
    if (aOut)
    {
        *aOut = ok;
    }
}

// Async persistence knobs, kept separate from VNavSetVolumeTuning so that adding them does not
// change an existing signature. Read-modify-write, so calling either function leaves the other's
// settings alone.
//
// These affect ASYNC queries only. A sync query blocks its caller, so it stays bounded and fails
// fast no matter what is set here.
void VNavSetSearchPersistence(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName name;
    int32_t maxParks = 0;
    int32_t noProgressParks = 512;
    float timeoutSeconds = 30.0f;

    RED4ext::GetParameter(aFrame, &name);
    RED4ext::GetParameter(aFrame, &maxParks);
    RED4ext::GetParameter(aFrame, &noProgressParks);
    RED4ext::GetParameter(aFrame, &timeoutSeconds);
    aFrame->code++;

    const std::string volume = NameToString(name);

    PathQuery q = Runtime::Get().QueryFor(volume);
    q.maxParks = maxParks;
    q.noProgressParks = noProgressParks;
    q.timeoutSeconds = timeoutSeconds;

    const bool ok = q.IsValid();
    if (ok)
    {
        Runtime::Get().SetQueryDefaults(volume, q);
    }
    if (aOut)
    {
        *aOut = ok;
    }
}

// Agent body length, for turn clearance. See VolumeConfig::agentLength - agentRadius stays the
// CROSS-SECTION half-extent, because LineOfSight already sweeps a capsule along every straight
// segment. This only adds the bend case.
void VNavSetAgentLength(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName name;
    float length = 0.0f;
    int32_t turnSamples = 5;

    RED4ext::GetParameter(aFrame, &name);
    RED4ext::GetParameter(aFrame, &length);
    RED4ext::GetParameter(aFrame, &turnSamples);
    aFrame->code++;

    const std::string volume = NameToString(name);

    // Both live on PathQuery, not VolumeConfig: agentLength is not baked into the sampled data
    // (it is consulted only while smoothing), so it needs no volume rebuild and cannot race a
    // running query.
    PathQuery q = Runtime::Get().QueryFor(volume);
    q.agentLength = length;
    q.turnSamples = turnSamples;

    const bool ok = q.IsValid();
    if (ok)
    {
        Runtime::Get().SetQueryDefaults(volume, q);
    }
    if (aOut)
    {
        *aOut = ok;
    }
}

// Hierarchical guidance: plan over whole chunks first, then bias the voxel search toward that
// route. Needs an attached bake - the coarse graph is read from the bake index.
void VNavSetCoarseGuidance(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName name;
    bool enabled = true;
    float penalty = 4.0f;

    RED4ext::GetParameter(aFrame, &name);
    RED4ext::GetParameter(aFrame, &enabled);
    RED4ext::GetParameter(aFrame, &penalty);
    aFrame->code++;

    const std::string volume = NameToString(name);

    PathQuery q = Runtime::Get().QueryFor(volume);
    q.coarseGuidance = enabled;
    q.coarsePenalty = penalty;

    const bool ok = q.IsValid();
    if (ok)
    {
        Runtime::Get().SetQueryDefaults(volume, q);
    }
    if (aOut)
    {
        *aOut = ok;
    }
}

// Is this point occupiable? See NavService::IsFree - mainly for checking an OFFSET goal before
// committing to it, since "hover N metres above the target" is fine outdoors and inside the
// ceiling indoors.
void VNavIsFree(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName volume;
    RED4ext::Vector4 point{};
    RED4ext::GetParameter(aFrame, &volume);
    RED4ext::GetParameter(aFrame, &point);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    const bool free = rt.IsRunning() && rt.Nav().IsFree(NameToString(volume), ToVec3(point));
    if (aOut)
    {
        *aOut = free;
    }
}

// --- escape / unstick ------------------------------------------------------------------------
//
// Scalar-out like everything else here: the position is returned via VNavEscapePosition rather
// than through an out-parameter, so the binding surface stays free of struct marshalling.
EscapeResult& LastEscape()
{
    static EscapeResult s;
    return s;
}

void VNavFindEscape(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    RED4ext::CName name;
    RED4ext::Vector4 from{};
    float searchRadius = 16.0f;
    float minClearance = 0.0f;
    float minDistance = 0.0f;

    RED4ext::GetParameter(aFrame, &name);
    RED4ext::GetParameter(aFrame, &from);
    RED4ext::GetParameter(aFrame, &searchRadius);
    RED4ext::GetParameter(aFrame, &minClearance);
    RED4ext::GetParameter(aFrame, &minDistance);
    aFrame->code++;

    EscapeQuery q;
    q.from = ToVec3(from);
    q.searchRadius = searchRadius;
    q.minClearance = minClearance;
    q.minDistance = minDistance;

    Runtime& rt = Runtime::Get();
    LastEscape() = EscapeResult{};
    if (rt.IsRunning())
    {
        rt.Nav().FindEscape(NameToString(name), q, LastEscape());
    }

    if (aOut)
    {
        *aOut = static_cast<int32_t>(LastEscape().status);
    }
}

void VNavEscapePosition(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::Vector4* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
    {
        *aOut = ToVector4(LastEscape().position);
    }
}

void VNavEscapeDistance(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
    {
        *aOut = LastEscape().distance;
    }
}

// Where to resume a freshly-arrived path, given where the agent is now. See
// NavService::ResumeIndex - this is the fix for the replan reversal, and it belongs in the library
// rather than in every consumer.
void VNavPathResumeIndex(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    uint32_t handle = 0;
    RED4ext::Vector4 agent{};
    RED4ext::GetParameter(aFrame, &handle);
    RED4ext::GetParameter(aFrame, &agent);
    aFrame->code++;

    int32_t index = 0;
    PathResult r;
    if (ResolveResult(handle, r))
    {
        index = NavService::ResumeIndex(r.points, ToVec3(agent));
    }

    if (aOut)
    {
        *aOut = index;
    }
}

// --- dynamic obstacles ----------------------------------------------------------------------
//
// Batched begin/add/commit because the RTTI boundary here is deliberately scalar-only - there is
// no array marshalling anywhere in this file. The staging list lives on the game thread and is
// only published to the volume by Commit, so a search never observes a half-built set.
std::vector<AABB>& DynamicStaging()
{
    static std::vector<AABB> s;
    return s;
}

void VNavDynamicBegin(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    DynamicStaging().clear();
}

void VNavDynamicAdd(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    RED4ext::Vector4 center{};
    RED4ext::Vector4 halfExtents{};
    RED4ext::GetParameter(aFrame, &center);
    RED4ext::GetParameter(aFrame, &halfExtents);
    aFrame->code++;

    const Vec3 c = ToVec3(center);
    const Vec3 h = ToVec3(halfExtents);
    DynamicStaging().push_back(AABB{c - h, c + h});
}

void VNavDynamicCommit(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    RED4ext::CName name;
    RED4ext::GetParameter(aFrame, &name);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    int32_t count = 0;
    if (rt.IsRunning())
    {
        const std::string volume = NameToString(name);
        rt.Nav().SetDynamicObstacles(volume, DynamicStaging());
        count = static_cast<int32_t>(rt.Nav().DynamicObstacleCount(volume));
    }
    DynamicStaging().clear();

    if (aOut)
    {
        *aOut = count;
    }
}

// Orientation as a real search dimension - the expensive non-sphere tier. Needs agentLength set
// (every yaw is identical for a sphere), and falls back to 3D silently if it is not.
void VNavSetYawSearch(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName name;
    int32_t yawBuckets = 0;
    float yawCost = 0.5f;
    bool followsTravel = true;
    float travelCone = 0.785398f;

    RED4ext::GetParameter(aFrame, &name);
    RED4ext::GetParameter(aFrame, &yawBuckets);
    RED4ext::GetParameter(aFrame, &yawCost);
    RED4ext::GetParameter(aFrame, &followsTravel);
    RED4ext::GetParameter(aFrame, &travelCone);
    aFrame->code++;

    const std::string volume = NameToString(name);

    PathQuery q = Runtime::Get().QueryFor(volume);
    q.yawBuckets = yawBuckets;
    q.yawCost = yawCost;
    q.yawFollowsTravel = followsTravel;
    q.yawTravelCone = travelCone;

    const bool ok = q.IsValid();
    if (ok)
    {
        Runtime::Get().SetQueryDefaults(volume, q);
    }
    if (aOut)
    {
        *aOut = ok;
    }
}

// Exposed mainly so the two search modes can be A/B'd against each other in-game. Measured
// offline: bidirectional costs ~1.9x the expansions on a 50 m hop, but on a 640 m route through
// obstacles unidirectional does not finish at all - it exhausts its budget and returns a partial
// - while bidirectional completes for ~35% less work. Hence the default is on.
void VNavSetBidirectional(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName name;
    bool enabled = true;

    RED4ext::GetParameter(aFrame, &name);
    RED4ext::GetParameter(aFrame, &enabled);
    aFrame->code++;

    const std::string volume = NameToString(name);

    PathQuery q = Runtime::Get().QueryFor(volume);
    q.bidirectional = enabled;

    const bool ok = q.IsValid();
    if (ok)
    {
        Runtime::Get().SetQueryDefaults(volume, q);
    }
    if (aOut)
    {
        *aOut = ok;
    }
}

void VNavSetVolumeTuning(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName name;
    float heuristicWeight = 1.0f;
    float tightnessPenalty = 1.35f;
    float verticalCostMultiplier = 1.2f;
    float snapRadius = 4.0f;
    int32_t maxExpansions = 60000;
    int32_t smoothing = 1;

    RED4ext::GetParameter(aFrame, &name);
    RED4ext::GetParameter(aFrame, &heuristicWeight);
    RED4ext::GetParameter(aFrame, &tightnessPenalty);
    RED4ext::GetParameter(aFrame, &verticalCostMultiplier);
    RED4ext::GetParameter(aFrame, &snapRadius);
    RED4ext::GetParameter(aFrame, &maxExpansions);
    RED4ext::GetParameter(aFrame, &smoothing);
    aFrame->code++;

    // Read-modify-write rather than a fresh PathQuery: constructing a new one here would silently
    // reset whatever VNavSetSearchPersistence had configured for this volume.
    const std::string volume = NameToString(name);

    PathQuery q = Runtime::Get().QueryFor(volume);
    q.heuristicWeight = heuristicWeight;
    q.tightnessPenalty = tightnessPenalty;
    q.verticalCostMultiplier = verticalCostMultiplier;
    q.snapRadius = snapRadius;
    q.maxExpansions = maxExpansions;
    // 3 == Fillet; anything out of range falls back to the StringPull default.
    q.smoothing = static_cast<Smoothing>(smoothing < 0 || smoothing > 3 ? 1 : smoothing);

    const bool ok = q.IsValid();
    if (ok)
    {
        Runtime::Get().SetQueryDefaults(volume, q);
    }
    if (aOut)
    {
        *aOut = ok;
    }
}

// Attaches a baked static world. The path is resolved relative to the game root, so a mod ships
// its bake alongside its other files and passes a relative path.
void VNavAttachBakeChecked(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName volume;
    RED4ext::CString path;
    RED4ext::CString gameBuild;

    RED4ext::GetParameter(aFrame, &volume);
    RED4ext::GetParameter(aFrame, &path);
    RED4ext::GetParameter(aFrame, &gameBuild);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    bool ok = false;

    if (rt.IsRunning() && path.Length() > 0)
    {
        std::string err;
        ok = rt.Nav().AttachBakeChecked(NameToString(volume), rt.ResolveGamePath(path.c_str()),
                                        gameBuild.Length() > 0 ? gameBuild.c_str() : "", err);
        // A successful attach can still carry a message - an unstamped bake cannot be checked and
        // says so rather than being accepted silently.
        if (!err.empty())
        {
            vnav::Log("AttachBakeChecked: %s", err.c_str());
        }
    }

    if (aOut)
    {
        *aOut = ok;
    }
}

void VNavAttachBake(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName volume;
    RED4ext::CString path;

    RED4ext::GetParameter(aFrame, &volume);
    RED4ext::GetParameter(aFrame, &path);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    bool ok = false;

    if (rt.IsRunning() && path.Length() > 0)
    {
        std::string err;
        ok = rt.Nav().AttachBake(NameToString(volume), rt.ResolveGamePath(path.c_str()), err);
        if (!ok)
        {
            vnav::Log("AttachBake failed: %s", err.c_str());
        }
        else
        {
            // Gated at the call site too: NameToString allocates, and call-site arguments are
            // evaluated whether or not Log()'s internal constexpr guard is on. Everything else
            // passes values that already exist; this one is built FOR the log line.
            if constexpr (kLoggingEnabled)
            {
                vnav::Log("bake attached to volume '%s'", NameToString(volume).c_str());
            }
        }
    }

    if (aOut)
    {
        *aOut = ok;
    }
}

void VNavHasBake(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CName volume;
    RED4ext::GetParameter(aFrame, &volume);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    if (aOut)
    {
        *aOut = rt.IsRunning() && rt.Nav().HasBake(NameToString(volume));
    }
}

// -------------------------------------------------------------------------------------------
// Queries
// -------------------------------------------------------------------------------------------

void VNavFindPathAsync(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, uint32_t* aOut, int64_t)
{
    RED4ext::CName volume;
    RED4ext::Vector4 start{};
    RED4ext::Vector4 goal{};

    RED4ext::GetParameter(aFrame, &volume);
    RED4ext::GetParameter(aFrame, &start);
    RED4ext::GetParameter(aFrame, &goal);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    uint32_t handle = 0;

    if (rt.IsRunning())
    {
        const std::string name = NameToString(volume);
        PathQuery q = rt.QueryFor(name);
        q.start = ToVec3(start);
        q.goal = ToVec3(goal);
        handle = rt.Nav().SubmitAsync(name, q);
    }

    if (aOut)
    {
        *aOut = handle;
    }
}

void VNavFindPathSync(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, uint32_t* aOut, int64_t)
{
    RED4ext::CName volume;
    RED4ext::Vector4 start{};
    RED4ext::Vector4 goal{};

    RED4ext::GetParameter(aFrame, &volume);
    RED4ext::GetParameter(aFrame, &start);
    RED4ext::GetParameter(aFrame, &goal);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    uint32_t handle = 0;

    if (rt.IsRunning())
    {
        const std::string name = NameToString(volume);
        PathQuery q = rt.QueryFor(name);
        q.start = ToVec3(start);
        q.goal = ToVec3(goal);

        PathResult result;
        rt.Nav().FindPathSync(name, q, result);
        handle = rt.StoreSyncResult(result);
    }

    if (aOut)
    {
        *aOut = handle;
    }
}

void VNavPathStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    uint32_t handle = 0;
    RED4ext::GetParameter(aFrame, &handle);
    aFrame->code++;

    PathResult r;
    ResolveResult(handle, r);
    if (aOut)
    {
        *aOut = static_cast<int32_t>(r.status);
    }
}

void VNavPathPointCount(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    uint32_t handle = 0;
    RED4ext::GetParameter(aFrame, &handle);
    aFrame->code++;

    PathResult r;
    const int32_t count = ResolveResult(handle, r) ? static_cast<int32_t>(r.points.size()) : 0;
    if (aOut)
    {
        *aOut = count;
    }
}

void VNavPathPoint(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::Vector4* aOut, int64_t)
{
    uint32_t handle = 0;
    int32_t index = 0;
    RED4ext::GetParameter(aFrame, &handle);
    RED4ext::GetParameter(aFrame, &index);
    aFrame->code++;

    RED4ext::Vector4 point{0.0f, 0.0f, 0.0f, 1.0f};
    PathResult r;
    if (ResolveResult(handle, r) && index >= 0 && static_cast<size_t>(index) < r.points.size())
    {
        point = ToVector4(r.points[static_cast<size_t>(index)]);
    }

    if (aOut)
    {
        *aOut = point;
    }
}

// Free radius at a waypoint, in metres. The search tested this value anyway, so reading it back
// costs nothing - and it is what lets a caller cut a corner or pick a speed against real headroom
// instead of guessing or re-querying. Returns 0 for an out-of-range index.
void VNavPathClearance(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    uint32_t handle = 0;
    int32_t index = 0;
    RED4ext::GetParameter(aFrame, &handle);
    RED4ext::GetParameter(aFrame, &index);
    aFrame->code++;

    float clearance = 0.0f;
    PathResult r;
    if (ResolveResult(handle, r) && index >= 0 && static_cast<size_t>(index) < r.clearances.size())
    {
        clearance = r.clearances[static_cast<size_t>(index)];
    }

    if (aOut)
    {
        *aOut = clearance;
    }
}

// Corners an elongated agent cannot rotate through. Non-zero means the route itself is too tight
// for the configured agentLength - the smoother already avoided every corner it could.
void VNavPathInfeasibleTurns(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    uint32_t handle = 0;
    RED4ext::GetParameter(aFrame, &handle);
    aFrame->code++;

    int32_t turns = 0;
    PathResult r;
    if (ResolveResult(handle, r))
    {
        turns = r.infeasibleTurns;
    }

    if (aOut)
    {
        *aOut = turns;
    }
}

// Body heading at a waypoint, radians about the vertical axis. Only meaningful for a 4D plan;
// returns 0 for a 3D one, where orientation was never part of the answer.
void VNavPathYaw(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    uint32_t handle = 0;
    int32_t index = 0;
    RED4ext::GetParameter(aFrame, &handle);
    RED4ext::GetParameter(aFrame, &index);
    aFrame->code++;

    float yaw = 0.0f;
    PathResult r;
    if (ResolveResult(handle, r) && index >= 0 && static_cast<size_t>(index) < r.yaws.size())
    {
        yaw = r.yaws[static_cast<size_t>(index)];
    }

    if (aOut)
    {
        *aOut = yaw;
    }
}

void VNavPathLength(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    uint32_t handle = 0;
    RED4ext::GetParameter(aFrame, &handle);
    aFrame->code++;

    PathResult r;
    const float len = ResolveResult(handle, r) ? r.length : 0.0f;
    if (aOut)
    {
        *aOut = len;
    }
}

void VNavPathRelease(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    uint32_t handle = 0;
    RED4ext::GetParameter(aFrame, &handle);
    aFrame->code++;

    Runtime::Get().ReleaseResult(handle);
}

void VNavCancelPath(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    uint32_t handle = 0;
    RED4ext::GetParameter(aFrame, &handle);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    const bool ok = rt.IsRunning() && (handle & kSyncHandleBit) == 0 && rt.Nav().Cancel(handle);
    if (aOut)
    {
        *aOut = ok;
    }
}

// -------------------------------------------------------------------------------------------
// World maintenance + pump
// -------------------------------------------------------------------------------------------

void VNavInvalidateBox(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    RED4ext::CName volume;
    RED4ext::Vector4 center{};
    RED4ext::Vector4 halfExtents{};

    RED4ext::GetParameter(aFrame, &volume);
    RED4ext::GetParameter(aFrame, &center);
    RED4ext::GetParameter(aFrame, &halfExtents);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    if (rt.IsRunning())
    {
        rt.Nav().InvalidateBox(NameToString(volume), AABB::FromCenterExtents(ToVec3(center), ToVec3(halfExtents)));
    }
}

void VNavInvalidateAll(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    RED4ext::CName volume;
    RED4ext::GetParameter(aFrame, &volume);
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    if (rt.IsRunning())
    {
        rt.Nav().InvalidateAll(NameToString(volume));
    }
}

void VNavPump(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    int32_t budgetMicros = -1;
    RED4ext::GetParameter(aFrame, &budgetMicros);
    aFrame->code++;

    Runtime::Get().PumpFromScript(budgetMicros);
}

void VNavIsReady(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;

    Runtime& rt = Runtime::Get();
    if (aOut)
    {
        *aOut = rt.IsRunning() && rt.Probe().IsReady();
    }
}

// Single scalar accessor instead of a registered stats struct - one function to keep working
// as the stat set grows.
void VNavStat(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    RED4ext::CName key;
    RED4ext::GetParameter(aFrame, &key);
    aFrame->code++;

    int32_t value = 0;
    Runtime& rt = Runtime::Get();
    if (rt.IsRunning())
    {
        const Stats s = rt.Nav().GetStats();
        const std::string k = NameToString(key);

        if (k == "queries") { value = static_cast<int32_t>(s.totalQueries); }
        else if (k == "succeeded") { value = static_cast<int32_t>(s.succeeded); }
        else if (k == "partial") { value = static_cast<int32_t>(s.partial); }
        else if (k == "failed") { value = static_cast<int32_t>(s.failed); }
        else if (k == "probes") { value = static_cast<int32_t>(s.totalProbes); }
        else if (k == "chunksSampled") { value = static_cast<int32_t>(s.totalChunksSampled); }
        else if (k == "residentChunks") { value = s.residentChunks; }
        else if (k == "residentKB") { value = static_cast<int32_t>(s.residentBytes / 1024); }
        else if (k == "jobsPending") { value = s.jobsPending; }
        else if (k == "jobsParked") { value = s.jobsParked; }
        else if (k == "jobsRunning") { value = s.jobsRunning; }
        else if (k == "pendingRequests") { value = s.pendingChunkRequests; }
        else if (k == "usingFilterPath") { value = rt.Probe().UsingFilterPath() ? 1 : 0; }
        else if (k == "nativePump") { value = rt.NativePumpActive() ? 1 : 0; }
        else if (k == "chunksFromBake") { value = static_cast<int32_t>(s.chunksFromBake); }
    }

    if (aOut)
    {
        *aOut = value;
    }
}

RED4ext::CBaseFunction::Flags GlobalFlags()
{
    RED4ext::CBaseFunction::Flags flags{};
    flags.isNative = true;
    flags.isStatic = true;
    return flags;
}
} // namespace

namespace vnav::game
{
void RegisterScriptBindings()
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    if (!rtti)
    {
        return;
    }

    const auto flags = GlobalFlags();

    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavCreateVolume", "VNavCreateVolume", &VNavCreateVolume);
        f->AddParam("CName", "name");
        f->AddParam("Float", "voxelSize");
        f->AddParam("Float", "agentRadius");
        f->AddParam("Float", "margin");
        f->AddParam("Vector4", "boundsMin");
        f->AddParam("Vector4", "boundsMax");
        f->AddParam("Uint32", "collisionMask");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavDestroyVolume", "VNavDestroyVolume", &VNavDestroyVolume);
        f->AddParam("CName", "name");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavHasVolume", "VNavHasVolume", &VNavHasVolume);
        f->AddParam("CName", "name");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavSetVolumeTuning", "VNavSetVolumeTuning", &VNavSetVolumeTuning);
        f->AddParam("CName", "name");
        f->AddParam("Float", "heuristicWeight");
        f->AddParam("Float", "tightnessPenalty");
        f->AddParam("Float", "verticalCostMultiplier");
        f->AddParam("Float", "snapRadius");
        f->AddParam("Int32", "maxExpansions");
        f->AddParam("Int32", "smoothing");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavSetCoarseGuidance", "VNavSetCoarseGuidance",
                                                   &VNavSetCoarseGuidance);
        f->AddParam("CName", "name");
        f->AddParam("Bool", "enabled");
        f->AddParam("Float", "penalty");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavIsFree", "VNavIsFree", &VNavIsFree);
        f->AddParam("CName", "volume");
        f->AddParam("Vector4", "point");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavFindEscape", "VNavFindEscape", &VNavFindEscape);
        f->AddParam("CName", "volume");
        f->AddParam("Vector4", "from");
        f->AddParam("Float", "searchRadius");
        f->AddParam("Float", "minClearance");
        f->AddParam("Float", "minDistance");
        f->SetReturnType("Int32");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f =
            RED4ext::CGlobalFunction::Create("VNavEscapePosition", "VNavEscapePosition", &VNavEscapePosition);
        f->SetReturnType("Vector4");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f =
            RED4ext::CGlobalFunction::Create("VNavEscapeDistance", "VNavEscapeDistance", &VNavEscapeDistance);
        f->SetReturnType("Float");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavPathResumeIndex", "VNavPathResumeIndex",
                                                   &VNavPathResumeIndex);
        f->AddParam("Uint32", "handle");
        f->AddParam("Vector4", "agentPosition");
        f->SetReturnType("Int32");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f =
            RED4ext::CGlobalFunction::Create("VNavDynamicBegin", "VNavDynamicBegin", &VNavDynamicBegin);
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavDynamicAdd", "VNavDynamicAdd", &VNavDynamicAdd);
        f->AddParam("Vector4", "center");
        f->AddParam("Vector4", "halfExtents");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f =
            RED4ext::CGlobalFunction::Create("VNavDynamicCommit", "VNavDynamicCommit", &VNavDynamicCommit);
        f->AddParam("CName", "volume");
        f->SetReturnType("Int32");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f =
            RED4ext::CGlobalFunction::Create("VNavSetYawSearch", "VNavSetYawSearch", &VNavSetYawSearch);
        f->AddParam("CName", "name");
        f->AddParam("Int32", "yawBuckets");
        f->AddParam("Float", "yawCost");
        f->AddParam("Bool", "yawFollowsTravel");
        f->AddParam("Float", "yawTravelCone");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f =
            RED4ext::CGlobalFunction::Create("VNavSetAgentLength", "VNavSetAgentLength", &VNavSetAgentLength);
        f->AddParam("CName", "name");
        f->AddParam("Float", "agentLength");
        f->AddParam("Int32", "turnSamples");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavSetBidirectional", "VNavSetBidirectional",
                                                   &VNavSetBidirectional);
        f->AddParam("CName", "name");
        f->AddParam("Bool", "enabled");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavSetSearchPersistence", "VNavSetSearchPersistence",
                                                   &VNavSetSearchPersistence);
        f->AddParam("CName", "name");
        f->AddParam("Int32", "maxParks");
        f->AddParam("Int32", "noProgressParks");
        f->AddParam("Float", "timeoutSeconds");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavAttachBakeChecked", "VNavAttachBakeChecked",
                                                   &VNavAttachBakeChecked);
        f->AddParam("CName", "volume");
        f->AddParam("String", "path");
        f->AddParam("String", "gameBuild");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavAttachBake", "VNavAttachBake", &VNavAttachBake);
        f->AddParam("CName", "volume");
        f->AddParam("String", "path");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavHasBake", "VNavHasBake", &VNavHasBake);
        f->AddParam("CName", "volume");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavFindPathAsync", "VNavFindPathAsync", &VNavFindPathAsync);
        f->AddParam("CName", "volume");
        f->AddParam("Vector4", "start");
        f->AddParam("Vector4", "goal");
        f->SetReturnType("Uint32");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavFindPathSync", "VNavFindPathSync", &VNavFindPathSync);
        f->AddParam("CName", "volume");
        f->AddParam("Vector4", "start");
        f->AddParam("Vector4", "goal");
        f->SetReturnType("Uint32");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavPathStatus", "VNavPathStatus", &VNavPathStatus);
        f->AddParam("Uint32", "handle");
        f->SetReturnType("Int32");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavPathPointCount", "VNavPathPointCount", &VNavPathPointCount);
        f->AddParam("Uint32", "handle");
        f->SetReturnType("Int32");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavPathPoint", "VNavPathPoint", &VNavPathPoint);
        f->AddParam("Uint32", "handle");
        f->AddParam("Int32", "index");
        f->SetReturnType("Vector4");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f =
            RED4ext::CGlobalFunction::Create("VNavPathClearance", "VNavPathClearance", &VNavPathClearance);
        f->AddParam("Uint32", "handle");
        f->AddParam("Int32", "index");
        f->SetReturnType("Float");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavPathInfeasibleTurns", "VNavPathInfeasibleTurns",
                                                   &VNavPathInfeasibleTurns);
        f->AddParam("Uint32", "handle");
        f->SetReturnType("Int32");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavPathYaw", "VNavPathYaw", &VNavPathYaw);
        f->AddParam("Uint32", "handle");
        f->AddParam("Int32", "index");
        f->SetReturnType("Float");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavPathLength", "VNavPathLength", &VNavPathLength);
        f->AddParam("Uint32", "handle");
        f->SetReturnType("Float");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavPathRelease", "VNavPathRelease", &VNavPathRelease);
        f->AddParam("Uint32", "handle");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavCancelPath", "VNavCancelPath", &VNavCancelPath);
        f->AddParam("Uint32", "handle");
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavInvalidateBox", "VNavInvalidateBox", &VNavInvalidateBox);
        f->AddParam("CName", "volume");
        f->AddParam("Vector4", "center");
        f->AddParam("Vector4", "halfExtents");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavInvalidateAll", "VNavInvalidateAll", &VNavInvalidateAll);
        f->AddParam("CName", "volume");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavPump", "VNavPump", &VNavPump);
        f->AddParam("Int32", "budgetMicros");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavIsReady", "VNavIsReady", &VNavIsReady);
        f->SetReturnType("Bool");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
    {
        auto* f = RED4ext::CGlobalFunction::Create("VNavStat", "VNavStat", &VNavStat);
        f->AddParam("CName", "key");
        f->SetReturnType("Int32");
        f->flags = flags;
        rtti->RegisterFunction(f);
    }
}
} // namespace vnav::game
