#include "core/ChunkStore.hpp"

#include <algorithm>

namespace vnav
{
std::shared_ptr<const Chunk> ChunkStore::Get(uint64_t aKey, uint64_t aTick) const
{
    Shard& s = m_shards[ShardOf(aKey)];
    std::shared_lock lock(s.mutex);

    auto it = s.map.find(aKey);
    if (it == s.map.end())
    {
        return nullptr;
    }

    // Safe under a shared lock precisely because it is atomic: several readers may bump this
    // concurrently and the worst outcome is a slightly stale LRU order.
    it->second.lastUsed.store(aTick, std::memory_order_relaxed);
    return it->second.chunk;
}

bool ChunkStore::Contains(uint64_t aKey) const
{
    Shard& s = m_shards[ShardOf(aKey)];
    std::shared_lock lock(s.mutex);
    return s.map.find(aKey) != s.map.end();
}

void ChunkStore::Publish(uint64_t aKey, std::shared_ptr<const Chunk> aChunk, uint64_t aTick)
{
    if (!aChunk)
    {
        return;
    }

    Shard& s = m_shards[ShardOf(aKey)];
    std::unique_lock lock(s.mutex);

    auto it = s.map.find(aKey);
    if (it == s.map.end())
    {
        s.map.try_emplace(aKey, std::move(aChunk), aTick);
    }
    else
    {
        // Replacing the handle, never mutating the payload. Readers holding the old chunk keep
        // reading the old chunk, which stays alive until they drop it.
        it->second.chunk = std::move(aChunk);
        it->second.lastUsed.store(aTick, std::memory_order_relaxed);
    }
}

void ChunkStore::Erase(uint64_t aKey)
{
    Shard& s = m_shards[ShardOf(aKey)];
    std::unique_lock lock(s.mutex);
    s.map.erase(aKey);
}

void ChunkStore::EraseIf(const std::function<bool(uint64_t)>& aPred)
{
    for (auto& s : m_shards)
    {
        std::unique_lock lock(s.mutex);
        for (auto it = s.map.begin(); it != s.map.end();)
        {
            it = aPred(it->first) ? s.map.erase(it) : std::next(it);
        }
    }
}

void ChunkStore::Clear()
{
    for (auto& s : m_shards)
    {
        std::unique_lock lock(s.mutex);
        s.map.clear();
    }
}

size_t ChunkStore::Size() const
{
    size_t n = 0;
    for (auto& s : m_shards)
    {
        std::shared_lock lock(s.mutex);
        n += s.map.size();
    }
    return n;
}

size_t ChunkStore::ByteSize() const
{
    size_t n = 0;
    for (auto& s : m_shards)
    {
        std::shared_lock lock(s.mutex);
        for (const auto& [key, entry] : s.map)
        {
            n += entry.chunk->ByteSize() + sizeof(Entry) + sizeof(uint64_t);
        }
    }
    return n;
}

void ChunkStore::EvictTo(size_t aMaxChunks)
{
    const size_t total = Size();
    if (total <= aMaxChunks)
    {
        return;
    }

    // Gather stamps across all shards, then take the cut point. One pass, no global lock held
    // while erasing.
    std::vector<std::pair<uint64_t, uint64_t>> stamps; // (lastUsed, key)
    stamps.reserve(total);
    for (auto& s : m_shards)
    {
        std::shared_lock lock(s.mutex);
        for (const auto& [key, entry] : s.map)
        {
            stamps.emplace_back(entry.lastUsed.load(std::memory_order_relaxed), key);
        }
    }

    const size_t toDrop = stamps.size() > aMaxChunks ? stamps.size() - aMaxChunks : 0;
    if (toDrop == 0)
    {
        return;
    }

    std::nth_element(stamps.begin(), stamps.begin() + static_cast<ptrdiff_t>(toDrop), stamps.end());
    for (size_t i = 0; i < toDrop; ++i)
    {
        Erase(stamps[i].second);
    }
}

void ChunkStore::EvictOlderThan(uint64_t aOldestAllowedTick)
{
    for (auto& s : m_shards)
    {
        std::unique_lock lock(s.mutex);
        for (auto it = s.map.begin(); it != s.map.end();)
        {
            const bool stale = it->second.lastUsed.load(std::memory_order_relaxed) < aOldestAllowedTick;
            it = stale ? s.map.erase(it) : std::next(it);
        }
    }
}
} // namespace vnav
