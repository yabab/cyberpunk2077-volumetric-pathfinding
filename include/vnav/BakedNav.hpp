// vnav - the ".vnavbake" baked navigation volume: static world occupancy for the whole map.
//
// WHY THIS EXISTS
//
// The runtime sampler can only probe collision that is currently streamed in, so anything past
// the streaming radius reads back as empty air. That is not a slow answer, it is a wrong one: a
// path planned at range flies straight through buildings the engine has not loaded yet. Reading
// the archives offline sees the whole world at once and settles it permanently.
//
// WHY IT STORES OCCUPANCY AND NOT CLEARANCE
//
// The planner wants clearance - distance to the nearest solid - because that is what makes the
// data agent-agnostic: "does a radius-r agent fit here" becomes one threshold compare, so a
// single bake serves a drone and an AV alike.
//
// Storing that clearance directly was measured and rejected. A clearance field is a gradient,
// and in a city nearly every voxel is on one, so it barely compresses: at 1 m voxels the whole
// map came to several GB. Occupancy is one bit per voxel, has enormous flat runs, and the
// clearance is recovered exactly by a distance transform at load time.
//
// The transform only has to reach as far as the asking volume's own threshold, so the halo is
// sized by the agent rather than by a global cap - which also makes this strictly more accurate
// than a stored field would have been, since there is no saturation to lose information to.
//
// FORMAT
//
//     Header  (fixed size)
//     Index   ChunkCount records, sorted by key, binary-searchable
//     Data    variable-length occupancy payloads for mixed chunks only
//
// A chunk absent from the index is open sky. That is the right default for a flying agent and it
// keeps the index to the surfaces that actually matter rather than the whole volume of the map.
// Solid chunks ARE stored, because "absent means open" would otherwise make bedrock flyable.
#pragma once

#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <vnav/Types.hpp>

namespace vnav
{
struct Chunk;
enum class ChunkState : uint8_t;

inline constexpr uint32_t kBakeVersion = 2;
inline constexpr int32_t kBakeHeaderSize = 96;
inline constexpr int32_t kBakeIndexRecordSize = 24;

// One bit per voxel.
inline constexpr int32_t kOccupancyBytes = kChunkVoxels / 8; // 512

// Largest halo the loader will run a transform over. Bounds both the work per chunk and the
// biggest agent that can be asked about.
inline constexpr uint8_t kMaxHalo = 16;

// Longest game-build stamp a bake can carry, including the terminator.
inline constexpr int32_t kBakeGameBuildBytes = 24;

struct BakeHeader
{
    uint32_t version = kBakeVersion;
    uint32_t chunkDim = static_cast<uint32_t>(kChunkDim);
    float voxelSize = 1.0f;
    uint32_t chunkCount = 0;
    uint64_t indexOffset = 0;
    uint64_t dataOffset = 0;
    AABB bounds;

    // Which game build this bake was made from. Empty means "unstamped".
    //
    // A bake describes the world as it shipped. Patch the game so geometry moves, keep the old
    // bake, and the planner routes confidently through walls that are no longer where it thinks -
    // silently, because nothing about the data looks wrong. The stamp is what turns that into a
    // detectable mismatch.
    //
    // Deliberately written into the header's existing spare bytes rather than behind a version
    // bump: a bump would make every bake already shipped unloadable and force a re-bake. Old files
    // have zeros here and simply read as unstamped, which is a warning rather than a refusal.
    char gameBuild[kBakeGameBuildBytes] = {};
};

struct BakeIndexRecord
{
    uint64_t key = 0;           // PackChunkKey of the chunk coordinate
    uint64_t payloadOffset = 0; // relative to dataOffset; meaningless unless Mixed
    uint32_t payloadBytes = 0;
    uint8_t state = 0; // ChunkState
};

// Run-length codec for a chunk's 512 occupancy bytes.
//
// Occupancy is mostly long stretches of all-solid or all-open, so runs are long. If the encoding
// does not pay for itself the raw block is stored instead, which the decoder detects from the
// payload length - so the worst case is "no compression", never expansion.
namespace bakecodec
{
size_t Encode(const uint8_t* aOccupancy, std::vector<uint8_t>& aOut);
bool Decode(const uint8_t* aPayload, size_t aBytes, uint8_t* aOccupancy);
} // namespace bakecodec

// Read-only view of a baked volume.
//
// The index is held in memory; occupancy payloads are read from the file on demand and cached
// decoded, so a city-sized bake costs tens of megabytes resident rather than gigabytes.
// Operations are serialised on an internal mutex because they move the shared file cursor and
// share the cache - callers are expected to fetch on the game thread during Pump, which is
// where sampling already happens.
class BakedNav
{
public:
    BakedNav() = default;
    ~BakedNav();

    BakedNav(const BakedNav&) = delete;
    BakedNav& operator=(const BakedNav&) = delete;

    bool Open(const std::string& aPath, std::string& aError);
    void Close();

    bool IsOpen() const { return m_file != nullptr; }

    const BakeHeader& Header() const { return m_header; }

    // The game build this bake was made from, or empty if it predates stamping.
    std::string GameBuild() const { return std::string(m_header.gameBuild); }
    size_t ChunkCount() const { return m_index.size(); }

    // True if a transform out to this many voxels is supported.
    static bool SupportsThreshold(uint8_t aThreshold) { return aThreshold <= kMaxHalo; }

    // Coarse occupancy for one chunk, WITHOUT loading or decoding it.
    //
    // This is the whole basis of the hierarchical search: the index already records a state byte
    // per chunk, and absent-but-covered means open sky, so a whole-map connectivity graph is
    // available for the cost of hash lookups - no probing, no distance transform, no file reads.
    //
    // Returns Empty for chunks outside the baked region: unknown space is left for the fine search
    // (and the probe) to resolve rather than being written off as blocked.
    ChunkState CoarseState(const IVec3& aChunkCoord) const;

    // Does the bake cover this chunk at all? False means "outside the baked region", which is
    // different from "baked and known to be open".
    bool Covers(const IVec3& aChunkCoord) const;

    // Builds the chunk for a coordinate, running the distance transform out to aHalo voxels.
    // aHalo should be the asking volume's tight threshold: clearance saturates there, which is
    // exactly far enough for every comparison that volume will make.
    //
    // Returns nullptr for coordinates outside the baked region.
    std::shared_ptr<const Chunk> Load(const IVec3& aChunkCoord, uint8_t aHalo);

    // Raw index, for tooling that needs to enumerate what a bake actually stores (the LOD
    // comparison in the baker, for one). Not needed for planning.
    const std::unordered_map<uint64_t, BakeIndexRecord>& Index() const { return m_index; }

    void ClearCache();

private:
    // Occupancy for one chunk, expanded. Absent-from-index chunks resolve to all-open without
    // touching the file.
    const uint8_t* FetchOccupancy(const IVec3& aChunkCoord);

    std::FILE* m_file = nullptr;
    BakeHeader m_header;
    std::unordered_map<uint64_t, BakeIndexRecord> m_index;

    // Decoded occupancy cache. Chunk loads walk a 3x3x3 neighbourhood, and consecutive loads
    // overlap heavily, so even a modest cache turns 27 reads into ~1.
    std::unordered_map<uint64_t, std::vector<uint8_t>> m_cache;
    size_t m_cacheLimit = 8192;

    // Chunk-coordinate span of the baked region, so Covers() is a range test rather than a
    // failed hash lookup (absent-but-covered has to mean open sky, not "not baked").
    IVec3 m_chunkMin{0, 0, 0};
    IVec3 m_chunkMax{0, 0, 0};

    std::mutex m_mutex; // guards the file cursor and the cache
};

// Writes a .vnavbake. Chunks are appended in any order; the index is sorted and written on Close.
class BakedNavWriter
{
public:
    ~BakedNavWriter();

    // aRegion is the world volume this bake CLAIMS to describe, which is not the same as the
    // volume it stores chunks for. Open sky is not stored, so deriving coverage from the stored
    // chunks would report the empty air above a city as "not baked" and send the runtime back to
    // probing exactly where the bake is most useful.
    bool Open(const std::string& aPath, float aVoxelSize, const AABB& aRegion, std::string& aError);

    // aOccupancy must be kOccupancyBytes. Uniform chunks are detected here: all-solid is stored
    // without a payload, and all-open is not stored at all.
    bool AddChunk(const IVec3& aChunkCoord, const uint8_t* aOccupancy);

    // Stamp the bake with the game build it was made from. Optional, but strongly recommended -
    // see BakeHeader::gameBuild for what an unstamped bake costs you after a patch. Truncated to
    // kBakeGameBuildBytes-1 characters. Call any time before Close.
    void SetGameBuild(const std::string& aBuild);

    bool Close(std::string& aError);

    uint64_t MixedChunks() const { return m_mixed; }
    uint64_t SolidChunks() const { return m_solid; }
    uint64_t EmptyChunksSkipped() const { return m_emptySkipped; }
    uint64_t PayloadBytes() const { return m_payloadBytes; }

private:
    std::FILE* m_file = nullptr;
    float m_voxelSize = 1.0f;
    AABB m_region;
    char m_gameBuild[kBakeGameBuildBytes] = {};

    std::vector<BakeIndexRecord> m_index;
    std::vector<uint8_t> m_scratch;

    uint64_t m_payloadBytes = 0;
    uint64_t m_mixed = 0;
    uint64_t m_solid = 0;
    uint64_t m_emptySkipped = 0;
};
} // namespace vnav
