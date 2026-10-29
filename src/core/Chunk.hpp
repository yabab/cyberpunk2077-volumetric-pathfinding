// vnav - the voxel chunk: 16^3 clearance values, immutable once published.
#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include <vnav/Types.hpp>

namespace vnav
{
enum class ChunkState : uint8_t
{
    Empty = 0, // every voxel wide open
    Solid = 1, // every voxel blocked
    Mixed = 2, // consult the clearance field
};

// A sampled 16^3 block of space.
//
// IMMUTABILITY IS THE WHOLE CONCURRENCY STORY. A chunk is filled in privately by whatever
// produced it - the runtime sampler on the game thread, or the baked loader - and then published
// into a ChunkStore behind a shared_ptr<const Chunk>. After that point nothing ever writes to it,
// so any number of worker threads can read it with no synchronisation at all. Invalidation
// replaces the pointer; it never mutates the payload, so a job already walking the old chunk
// keeps a coherent view until it drops its pin.
struct Chunk
{
    ChunkState state = ChunkState::Empty;

    // Distance from each voxel to the nearest solid voxel, measured in VOXELS and saturating at
    // kMaxClearance. Zero means the voxel is itself solid.
    //
    // Storing distance rather than per-agent occupancy is what lets one baked volume serve every
    // agent size: "does a radius-r agent fit here" becomes a comparison against a threshold the
    // volume computes once, instead of a fact frozen into the data when it was sampled.
    //
    // Only meaningful when state == Mixed, and a Mixed chunk MUST have this filled - the
    // default-constructed all-zero array would read as solid everywhere.
    std::array<uint8_t, kChunkVoxels> clearance{};

    uint32_t probeCount = 0; // physics probes this chunk cost, for telemetry

    static constexpr int32_t LocalIndex(int32_t lx, int32_t ly, int32_t lz)
    {
        return (lz * kChunkDim + ly) * kChunkDim + lx;
    }

    uint8_t ClearanceLocal(int32_t aIdx) const
    {
        if (state == ChunkState::Empty)
        {
            return kMaxClearance;
        }
        if (state == ChunkState::Solid)
        {
            return 0;
        }
        return clearance[static_cast<size_t>(aIdx)];
    }

    void SetClearanceLocal(int32_t aIdx, uint8_t aValue)
    {
        clearance[static_cast<size_t>(aIdx)] = aValue;
    }

    // Bytes this chunk actually costs. Uniform chunks are shared singletons, so they cost
    // nothing beyond the map entry that points at them.
    size_t ByteSize() const { return state == ChunkState::Mixed ? sizeof(Chunk) : 0; }

    // Process-wide singletons. Open sky and solid rock are by far the most common outcomes,
    // and neither needs a payload.
    static const std::shared_ptr<const Chunk>& EmptyChunk();
    static const std::shared_ptr<const Chunk>& SolidChunk();
};
} // namespace vnav
