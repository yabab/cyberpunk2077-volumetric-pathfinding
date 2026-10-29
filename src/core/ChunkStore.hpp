// vnav - sharded, thread-safe map of chunk key -> immutable chunk.
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include "core/Chunk.hpp"

namespace vnav
{
// Chunk residency cache.
//
// Sharded so that the game thread publishing a chunk contends with at most 1/kShards of the
// readers. Values are shared_ptr<const Chunk>: a reader takes a *copy of the handle*, which is
// an atomic refcount bump, and then reads a payload that is guaranteed never to change. Writers
// only ever replace or erase the handle. That is what makes concurrent read/publish safe here,
// and it is why nothing in this class hands out a raw Chunk* that outlives its handle.
class ChunkStore
{
public:
    static constexpr size_t kShards = 64;

    // Fetch a chunk, or nullptr if it is not resident. Bumps its LRU stamp.
    std::shared_ptr<const Chunk> Get(uint64_t aKey, uint64_t aTick) const;

    // Residency test that does not pin the chunk or disturb the LRU order.
    bool Contains(uint64_t aKey) const;

    // Install (or replace) a chunk. Game thread.
    void Publish(uint64_t aKey, std::shared_ptr<const Chunk> aChunk, uint64_t aTick);

    void Erase(uint64_t aKey);

    // Drop every chunk whose key satisfies the predicate. Used by box invalidation.
    void EraseIf(const std::function<bool(uint64_t)>& aPred);

    void Clear();

    size_t Size() const;
    size_t ByteSize() const;

    // Approximate LRU: collects (stamp, key) pairs and drops the oldest until Size() fits.
    // Called from the game thread during Pump, so the scan cost is paid on a schedule.
    void EvictTo(size_t aMaxChunks);

    // Drop chunks not touched since `aOldestAllowedTick`.
    void EvictOlderThan(uint64_t aOldestAllowedTick);

private:
    struct Entry
    {
        std::shared_ptr<const Chunk> chunk;
        mutable std::atomic<uint64_t> lastUsed;

        Entry(std::shared_ptr<const Chunk> aChunk, uint64_t aTick)
            : chunk(std::move(aChunk))
            , lastUsed(aTick)
        {
        }
    };

    struct Shard
    {
        mutable std::shared_mutex mutex;
        std::unordered_map<uint64_t, Entry> map;
    };

    static size_t ShardOf(uint64_t aKey)
    {
        // Fibonacci hashing - the low bits of a packed coord are far from uniform.
        return static_cast<size_t>((aKey * 0x9E3779B97F4A7C15ull) >> 58) & (kShards - 1);
    }

    mutable Shard m_shards[kShards];
};
} // namespace vnav
