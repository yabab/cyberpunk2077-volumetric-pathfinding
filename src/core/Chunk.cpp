#include "core/Chunk.hpp"

#include <vnav/Config.hpp>

namespace vnav
{
namespace
{
std::shared_ptr<const Chunk> MakeUniform(ChunkState aState)
{
    auto c = std::make_shared<Chunk>();
    c->state = aState;

    // ClearanceLocal answers from `state` alone for the uniform cases, so this array is never
    // consulted. Filled anyway so the object is not internally inconsistent for anyone who
    // reads it directly - a baked writer, a test, a debugger.
    c->clearance.fill(aState == ChunkState::Solid ? 0 : kMaxClearance);
    return c;
}
} // namespace

const std::shared_ptr<const Chunk>& Chunk::EmptyChunk()
{
    static const std::shared_ptr<const Chunk> s = MakeUniform(ChunkState::Empty);
    return s;
}

const std::shared_ptr<const Chunk>& Chunk::SolidChunk()
{
    static const std::shared_ptr<const Chunk> s = MakeUniform(ChunkState::Solid);
    return s;
}

const char* ToString(Status s)
{
    switch (s)
    {
    case Status::Invalid: return "Invalid";
    case Status::Pending: return "Pending";
    case Status::WaitingForWorld: return "WaitingForWorld";
    case Status::Running: return "Running";
    case Status::Success: return "Success";
    case Status::PartialSuccess: return "PartialSuccess";
    case Status::Unreachable: return "Unreachable";
    case Status::Cancelled: return "Cancelled";
    case Status::BudgetExhausted: return "BudgetExhausted";
    case Status::BadInput: return "BadInput";
    case Status::NoVolume: return "NoVolume";
    case Status::TimedOut: return "TimedOut";
    }
    return "?";
}

void PathResult::Clear()
{
    *this = PathResult{};
}

bool EscapeQuery::IsValid() const
{
    if (!from.IsFinite())
    {
        return false;
    }
    if (!(searchRadius > 0.0f) || searchRadius > 512.0f)
    {
        return false;
    }
    if (minClearance < 0.0f || minDistance < 0.0f)
    {
        return false;
    }
    // A minimum further out than the search itself can never be satisfied.
    if (minDistance > searchRadius)
    {
        return false;
    }
    return maxProbesPerQuery >= 0;
}

bool VolumeConfig::IsValid() const
{
    if (!(voxelSize > 0.01f) || voxelSize > 64.0f)
    {
        return false;
    }
    if (agentRadius < 0.0f || margin < 0.0f)
    {
        return false;
    }
    if (maxProbeDepth < 0 || maxProbeDepth > 4)
    {
        return false;
    }
    if (maxProbesPerChunk < 1 || maxResidentChunks < 1)
    {
        return false;
    }
    if (chunkTtlSeconds < 0.0f)
    {
        return false;
    }
    if (!(bounds.min.x <= bounds.max.x && bounds.min.y <= bounds.max.y && bounds.min.z <= bounds.max.z))
    {
        return false;
    }
    return true;
}

bool PathQuery::IsValid() const
{
    if (!start.IsFinite() || !goal.IsFinite())
    {
        return false;
    }
    if (snapRadius < 0.0f || maxExpansions < 1 || maxProbesPerQuery < 0)
    {
        return false;
    }
    if (!(heuristicWeight >= 1.0f) || !(tightnessPenalty >= 1.0f) || !(verticalCostMultiplier > 0.0f))
    {
        return false;
    }
    if (splineSamplesPerSegment < 1 || prefetchRadiusChunks < 0 || prefetchRadiusChunks > 8)
    {
        return false;
    }
    // 0 is meaningful for both (unlimited / guard disabled), so only negatives are rejected.
    if (maxParks < 0 || noProgressParks < 0)
    {
        return false;
    }
    if (turnSamples < 1 || agentLength < 0.0f)
    {
        return false;
    }
    if (filletSamples < 2 || maxTurnRadius < 0.0f)
    {
        return false;
    }
    // A travel cone narrower than one bucket would leave no legal heading to move in.
    if (yawBuckets < 0 || yawBuckets > kMaxYawBuckets || yawCost < 0.0f || yawTravelCone <= 0.0f)
    {
        return false;
    }
    if (timeoutSeconds < 0.0f)
    {
        return false;
    }
    return true;
}
} // namespace vnav
