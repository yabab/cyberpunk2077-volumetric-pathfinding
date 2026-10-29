#include <vnav/BakedNav.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/Chunk.hpp"

namespace vnav
{
namespace
{
constexpr char kMagic[8] = {'V', 'N', 'A', 'V', 'B', 'A', 'K', '\0'};

bool ReadAt(std::FILE* aFile, int64_t aOffset, void* aDst, size_t aBytes)
{
    if (_fseeki64(aFile, aOffset, SEEK_SET) != 0)
    {
        return false;
    }
    return std::fread(aDst, 1, aBytes, aFile) == aBytes;
}

bool BitAt(const uint8_t* aOccupancy, int32_t aIdx)
{
    return (aOccupancy[aIdx >> 3] >> (aIdx & 7)) & 1u;
}

// Felzenszwalb & Huttenlocher's exact squared-distance transform, one dimension at a time.
// Linear in the number of samples and separable, so the 3-D transform is three passes.
void Transform1D(const float* aIn, float* aOut, int32_t aN, int32_t* aV, float* aZ)
{
    constexpr float kInf = 1e20f;

    int32_t k = 0;
    aV[0] = 0;
    aZ[0] = -kInf;
    aZ[1] = kInf;

    for (int32_t q = 1; q < aN; ++q)
    {
        float s = 0.0f;
        while (true)
        {
            const int32_t vk = aV[k];
            s = ((aIn[q] + static_cast<float>(q) * q) - (aIn[vk] + static_cast<float>(vk) * vk)) /
                (2.0f * static_cast<float>(q - vk));
            if (s > aZ[k])
            {
                break;
            }
            --k;
        }

        ++k;
        aV[k] = q;
        aZ[k] = s;
        aZ[k + 1] = kInf;
    }

    k = 0;
    for (int32_t q = 0; q < aN; ++q)
    {
        while (aZ[k + 1] < static_cast<float>(q))
        {
            ++k;
        }
        const int32_t vk = aV[k];
        const float d = static_cast<float>(q - vk);
        aOut[q] = d * d + aIn[vk];
    }
}
} // namespace

// -----------------------------------------------------------------------------------------------
// codec
// -----------------------------------------------------------------------------------------------

namespace bakecodec
{
size_t Encode(const uint8_t* aOccupancy, std::vector<uint8_t>& aOut)
{
    aOut.clear();
    aOut.reserve(128);

    int32_t i = 0;
    while (i < kOccupancyBytes)
    {
        const uint8_t value = aOccupancy[i];

        int32_t run = 1;
        while (i + run < kOccupancyBytes && run < 255 && aOccupancy[i + run] == value)
        {
            ++run;
        }

        aOut.push_back(static_cast<uint8_t>(run));
        aOut.push_back(value);
        i += run;

        // Once the encoding has grown past the raw block there is nothing left to win.
        if (aOut.size() >= static_cast<size_t>(kOccupancyBytes))
        {
            aOut.assign(aOccupancy, aOccupancy + kOccupancyBytes);
            return aOut.size();
        }
    }

    return aOut.size();
}

bool Decode(const uint8_t* aPayload, size_t aBytes, uint8_t* aOccupancy)
{
    // A payload exactly the size of the raw block is the uncompressed fallback.
    if (aBytes == static_cast<size_t>(kOccupancyBytes))
    {
        std::memcpy(aOccupancy, aPayload, static_cast<size_t>(kOccupancyBytes));
        return true;
    }

    if ((aBytes & 1u) != 0)
    {
        return false;
    }

    int32_t written = 0;
    for (size_t p = 0; p + 1 < aBytes; p += 2)
    {
        const int32_t run = aPayload[p];
        const uint8_t value = aPayload[p + 1];

        if (run == 0 || written + run > kOccupancyBytes)
        {
            return false;
        }

        std::memset(aOccupancy + written, value, static_cast<size_t>(run));
        written += run;
    }

    return written == kOccupancyBytes;
}
} // namespace bakecodec

// -----------------------------------------------------------------------------------------------
// reader
// -----------------------------------------------------------------------------------------------

BakedNav::~BakedNav()
{
    Close();
}

void BakedNav::Close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_file)
    {
        std::fclose(m_file);
        m_file = nullptr;
    }
    m_index.clear();
    m_cache.clear();
}

void BakedNav::ClearCache()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_cache.clear();
}

bool BakedNav::Open(const std::string& aPath, std::string& aError)
{
    Close();

    std::FILE* file = nullptr;
    if (fopen_s(&file, aPath.c_str(), "rb") != 0 || !file)
    {
        aError = "cannot open " + aPath;
        return false;
    }

    char magic[8] = {};
    float lo[3] = {};
    float hi[3] = {};
    uint8_t pad[4] = {};
    BakeHeader header;

    bool ok = ReadAt(file, 0, magic, sizeof(magic));
    ok = ok && std::fread(&header.version, 1, 4, file) == 4;
    ok = ok && std::fread(&header.chunkDim, 1, 4, file) == 4;
    ok = ok && std::fread(&header.voxelSize, 1, 4, file) == 4;
    ok = ok && std::fread(pad, 1, 4, file) == 4;
    ok = ok && std::fread(&header.chunkCount, 1, 4, file) == 4;
    ok = ok && std::fread(&header.indexOffset, 1, 8, file) == 8;
    ok = ok && std::fread(&header.dataOffset, 1, 8, file) == 8;
    ok = ok && std::fread(lo, 1, 12, file) == 12;
    ok = ok && std::fread(hi, 1, 12, file) == 12;
    // Spare-region stamp. Bakes written before stamping have zeros here and read as unstamped,
    // which is why this needs no version bump - see BakeHeader::gameBuild.
    ok = ok && std::fread(header.gameBuild, 1, kBakeGameBuildBytes, file) == kBakeGameBuildBytes;
    header.gameBuild[kBakeGameBuildBytes - 1] = '\0'; // never trust the file to terminate it

    if (!ok)
    {
        std::fclose(file);
        aError = "truncated .vnavbake header";
        return false;
    }
    if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0)
    {
        std::fclose(file);
        aError = "not a .vnavbake file (bad magic)";
        return false;
    }
    if (header.version != kBakeVersion)
    {
        std::fclose(file);
        aError = "unsupported .vnavbake version " + std::to_string(header.version);
        return false;
    }
    if (header.chunkDim != static_cast<uint32_t>(kChunkDim))
    {
        std::fclose(file);
        aError = "bake chunk dimension " + std::to_string(header.chunkDim) + " does not match this build";
        return false;
    }

    header.bounds = AABB{Vec3{lo[0], lo[1], lo[2]}, Vec3{hi[0], hi[1], hi[2]}};

    std::vector<uint8_t> raw(static_cast<size_t>(header.chunkCount) * kBakeIndexRecordSize);
    if (!raw.empty() && !ReadAt(file, static_cast<int64_t>(header.indexOffset), raw.data(), raw.size()))
    {
        std::fclose(file);
        aError = "truncated .vnavbake index";
        return false;
    }

    std::unordered_map<uint64_t, BakeIndexRecord> index;
    index.reserve(header.chunkCount * 2);

    for (uint32_t i = 0; i < header.chunkCount; ++i)
    {
        const uint8_t* p = raw.data() + static_cast<size_t>(i) * kBakeIndexRecordSize;

        BakeIndexRecord rec;
        std::memcpy(&rec.key, p + 0, 8);
        std::memcpy(&rec.payloadOffset, p + 8, 8);
        std::memcpy(&rec.payloadBytes, p + 16, 4);
        rec.state = p[20];

        index.emplace(rec.key, rec);
    }

    // Coverage comes from the region the baker processed, NOT from the chunks it stored. Open
    // sky is deliberately absent from the index, so deriving the range from stored chunks would
    // declare the air above a city un-baked.
    const float span = header.voxelSize * static_cast<float>(kChunkDim);
    const auto toChunk = [span](float aWorld) { return static_cast<int32_t>(std::floor(aWorld / span)); };

    IVec3 chunkMin{toChunk(header.bounds.min.x), toChunk(header.bounds.min.y), toChunk(header.bounds.min.z)};
    IVec3 chunkMax{toChunk(header.bounds.max.x), toChunk(header.bounds.max.y), toChunk(header.bounds.max.z)};

    if (header.bounds.max.x <= header.bounds.min.x)
    {
        chunkMin = IVec3{0, 0, 0};
        chunkMax = IVec3{-1, -1, -1}; // empty region: Covers() is always false
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_file = file;
        m_header = header;
        m_index = std::move(index);
        m_chunkMin = chunkMin;
        m_chunkMax = chunkMax;
        m_cache.clear();
    }

    return true;
}

ChunkState BakedNav::CoarseState(const IVec3& aChunkCoord) const
{
    if (!Covers(aChunkCoord))
    {
        return ChunkState::Empty; // outside the bake: unknown, not blocked
    }
    auto it = m_index.find(PackChunkKey(aChunkCoord));
    if (it == m_index.end())
    {
        return ChunkState::Empty; // covered but unstored == open sky
    }
    return static_cast<ChunkState>(it->second.state);
}

bool BakedNav::Covers(const IVec3& aChunkCoord) const
{
    return aChunkCoord.x >= m_chunkMin.x && aChunkCoord.x <= m_chunkMax.x && aChunkCoord.y >= m_chunkMin.y &&
           aChunkCoord.y <= m_chunkMax.y && aChunkCoord.z >= m_chunkMin.z && aChunkCoord.z <= m_chunkMax.z;
}

const uint8_t* BakedNav::FetchOccupancy(const IVec3& aChunkCoord)
{
    static const uint8_t kAllOpen[kOccupancyBytes] = {};
    static const std::vector<uint8_t> kAllSolidStorage(static_cast<size_t>(kOccupancyBytes), 0xFF);

    const uint64_t key = PackChunkKey(aChunkCoord);

    auto cached = m_cache.find(key);
    if (cached != m_cache.end())
    {
        return cached->second.data();
    }

    auto it = m_index.find(key);
    if (it == m_index.end())
    {
        // Absent means open sky, whether or not it is inside the baked region. Outside-the-region
        // reads only happen as halo lookups, where treating unknown space as open is the same
        // assumption the region boundary already makes.
        return kAllOpen;
    }

    if (it->second.state == static_cast<uint8_t>(ChunkState::Solid))
    {
        return kAllSolidStorage.data();
    }
    if (it->second.state == static_cast<uint8_t>(ChunkState::Empty))
    {
        return kAllOpen;
    }

    std::vector<uint8_t> payload(it->second.payloadBytes);
    const int64_t offset = static_cast<int64_t>(m_header.dataOffset + it->second.payloadOffset);
    if (payload.empty() || !ReadAt(m_file, offset, payload.data(), payload.size()))
    {
        return kAllSolidStorage.data(); // unreadable geometry is impassable, never open
    }

    std::vector<uint8_t> occupancy(static_cast<size_t>(kOccupancyBytes));
    if (!bakecodec::Decode(payload.data(), payload.size(), occupancy.data()))
    {
        return kAllSolidStorage.data();
    }

    // Crude but adequate: loads are spatially local, so the cache is warm for a working set and
    // a wholesale drop between working sets costs one reload rather than thrashing.
    if (m_cache.size() >= m_cacheLimit)
    {
        m_cache.clear();
    }

    return m_cache.emplace(key, std::move(occupancy)).first->second.data();
}

std::shared_ptr<const Chunk> BakedNav::Load(const IVec3& aChunkCoord, uint8_t aHalo)
{
    if (!Covers(aChunkCoord))
    {
        return nullptr;
    }

    const int32_t halo = std::clamp<int32_t>(aHalo, 1, kMaxHalo);

    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_file)
    {
        return nullptr;
    }

    // Fast paths that need no transform at all: a solid chunk is solid whatever the halo, and a
    // chunk is only open sky if nothing within the halo is solid - which the neighbourhood scan
    // below establishes.
    auto self = m_index.find(PackChunkKey(aChunkCoord));
    if (self != m_index.end() && self->second.state == static_cast<uint8_t>(ChunkState::Solid))
    {
        return Chunk::SolidChunk();
    }

    // Gather occupancy for the chunk and every neighbour the halo can reach.
    const int32_t dim = kChunkDim + 2 * halo;
    const int32_t reach = (halo + kChunkDim - 1) / kChunkDim; // neighbour chunks per side

    std::vector<float> field(static_cast<size_t>(dim) * dim * dim, 1e20f);
    bool anySolid = false;

    for (int32_t cz = -reach; cz <= reach; ++cz)
    {
        for (int32_t cy = -reach; cy <= reach; ++cy)
        {
            for (int32_t cx = -reach; cx <= reach; ++cx)
            {
                const IVec3 neighbour{aChunkCoord.x + cx, aChunkCoord.y + cy, aChunkCoord.z + cz};
                const uint8_t* occ = FetchOccupancy(neighbour);

                // Where this neighbour's voxels land in the padded field.
                const int32_t baseX = cx * kChunkDim + halo;
                const int32_t baseY = cy * kChunkDim + halo;
                const int32_t baseZ = cz * kChunkDim + halo;

                // Skip entirely-open neighbours cheaply.
                bool empty = true;
                for (int32_t b = 0; b < kOccupancyBytes; ++b)
                {
                    if (occ[b] != 0)
                    {
                        empty = false;
                        break;
                    }
                }
                if (empty)
                {
                    continue;
                }

                for (int32_t lz = 0; lz < kChunkDim; ++lz)
                {
                    const int32_t fz = baseZ + lz;
                    if (fz < 0 || fz >= dim)
                    {
                        continue;
                    }
                    for (int32_t ly = 0; ly < kChunkDim; ++ly)
                    {
                        const int32_t fy = baseY + ly;
                        if (fy < 0 || fy >= dim)
                        {
                            continue;
                        }
                        for (int32_t lx = 0; lx < kChunkDim; ++lx)
                        {
                            const int32_t fx = baseX + lx;
                            if (fx < 0 || fx >= dim)
                            {
                                continue;
                            }
                            if (BitAt(occ, Chunk::LocalIndex(lx, ly, lz)))
                            {
                                field[(static_cast<size_t>(fz) * dim + fy) * dim + fx] = 0.0f;
                                anySolid = true;
                            }
                        }
                    }
                }
            }
        }
    }

    if (!anySolid)
    {
        return Chunk::EmptyChunk();
    }

    // Separable exact EDT over the padded block.
    std::vector<float> line(static_cast<size_t>(dim));
    std::vector<float> out(static_cast<size_t>(dim));
    std::vector<int32_t> v(static_cast<size_t>(dim));
    std::vector<float> z(static_cast<size_t>(dim) + 1);

    const auto index = [dim](int32_t x, int32_t y, int32_t zz)
    { return (static_cast<size_t>(zz) * dim + y) * dim + x; };

    for (int32_t zz = 0; zz < dim; ++zz)
    {
        for (int32_t y = 0; y < dim; ++y)
        {
            float* row = field.data() + index(0, y, zz);
            Transform1D(row, out.data(), dim, v.data(), z.data());
            std::copy(out.begin(), out.end(), row);
        }
    }
    for (int32_t zz = 0; zz < dim; ++zz)
    {
        for (int32_t x = 0; x < dim; ++x)
        {
            for (int32_t y = 0; y < dim; ++y)
            {
                line[static_cast<size_t>(y)] = field[index(x, y, zz)];
            }
            Transform1D(line.data(), out.data(), dim, v.data(), z.data());
            for (int32_t y = 0; y < dim; ++y)
            {
                field[index(x, y, zz)] = out[static_cast<size_t>(y)];
            }
        }
    }
    for (int32_t y = 0; y < dim; ++y)
    {
        for (int32_t x = 0; x < dim; ++x)
        {
            for (int32_t zz = 0; zz < dim; ++zz)
            {
                line[static_cast<size_t>(zz)] = field[index(x, y, zz)];
            }
            Transform1D(line.data(), out.data(), dim, v.data(), z.data());
            for (int32_t zz = 0; zz < dim; ++zz)
            {
                field[index(x, y, zz)] = out[static_cast<size_t>(zz)];
            }
        }
    }

    auto chunk = std::make_shared<Chunk>();
    chunk->state = ChunkState::Mixed;

    // Clearance saturates at the halo. Since the halo is the asking volume's own tight
    // threshold, everything at or beyond it compares identically to kMaxClearance.
    const float haloF = static_cast<float>(halo);
    bool allOpen = true;
    bool allSolid = true;

    for (int32_t lz = 0; lz < kChunkDim; ++lz)
    {
        for (int32_t ly = 0; ly < kChunkDim; ++ly)
        {
            for (int32_t lx = 0; lx < kChunkDim; ++lx)
            {
                const float d = std::sqrt(field[index(lx + halo, ly + halo, lz + halo)]);
                const uint8_t value =
                    d >= haloF ? kMaxClearance : static_cast<uint8_t>(static_cast<int32_t>(d));

                chunk->SetClearanceLocal(Chunk::LocalIndex(lx, ly, lz), value);

                if (value != kMaxClearance)
                {
                    allOpen = false;
                }
                if (value != 0)
                {
                    allSolid = false;
                }
            }
        }
    }

    if (allOpen)
    {
        return Chunk::EmptyChunk();
    }
    if (allSolid)
    {
        return Chunk::SolidChunk();
    }

    return chunk;
}

// -----------------------------------------------------------------------------------------------
// writer
// -----------------------------------------------------------------------------------------------

BakedNavWriter::~BakedNavWriter()
{
    if (m_file)
    {
        std::fclose(m_file);
        m_file = nullptr;
    }
}

bool BakedNavWriter::Open(const std::string& aPath, float aVoxelSize, const AABB& aRegion, std::string& aError)
{
    if (fopen_s(&m_file, aPath.c_str(), "wb") != 0 || !m_file)
    {
        aError = "cannot create " + aPath;
        return false;
    }

    m_voxelSize = aVoxelSize;
    m_region = aRegion;

    // Reserve the header; payloads follow immediately and the index is appended last, once its
    // size is known.
    const std::vector<uint8_t> zeros(static_cast<size_t>(kBakeHeaderSize), 0);
    if (std::fwrite(zeros.data(), 1, zeros.size(), m_file) != zeros.size())
    {
        aError = "cannot write header placeholder";
        return false;
    }

    return true;
}

bool BakedNavWriter::AddChunk(const IVec3& aChunkCoord, const uint8_t* aOccupancy)
{
    bool allOpen = true;
    bool allSolid = true;
    for (int32_t i = 0; i < kOccupancyBytes; ++i)
    {
        if (aOccupancy[i] != 0x00)
        {
            allOpen = false;
        }
        if (aOccupancy[i] != 0xFF)
        {
            allSolid = false;
        }
        if (!allOpen && !allSolid)
        {
            break;
        }
    }

    // Open sky is not stored at all - an absent chunk already means open sky, which is most of
    // the map.
    if (allOpen)
    {
        ++m_emptySkipped;
        return true;
    }

    BakeIndexRecord rec;
    rec.key = PackChunkKey(aChunkCoord);

    if (allSolid)
    {
        rec.state = static_cast<uint8_t>(ChunkState::Solid);
        m_index.push_back(rec);
        ++m_solid;
        return true;
    }

    const size_t bytes = bakecodec::Encode(aOccupancy, m_scratch);
    if (std::fwrite(m_scratch.data(), 1, bytes, m_file) != bytes)
    {
        return false;
    }

    rec.payloadOffset = m_payloadBytes;
    rec.payloadBytes = static_cast<uint32_t>(bytes);
    rec.state = static_cast<uint8_t>(ChunkState::Mixed);
    m_index.push_back(rec);

    m_payloadBytes += bytes;
    ++m_mixed;
    return true;
}

void BakedNavWriter::SetGameBuild(const std::string& aBuild)
{
    const size_t n = std::min(aBuild.size(), static_cast<size_t>(kBakeGameBuildBytes - 1));
    std::memset(m_gameBuild, 0, sizeof(m_gameBuild));
    std::memcpy(m_gameBuild, aBuild.data(), n);
}

bool BakedNavWriter::Close(std::string& aError)
{
    if (!m_file)
    {
        return true;
    }

    // Sorted so a consumer that prefers binary search over a hash map can use the index directly.
    std::sort(m_index.begin(), m_index.end(),
              [](const BakeIndexRecord& a, const BakeIndexRecord& b) { return a.key < b.key; });

    const uint64_t dataOffset = static_cast<uint64_t>(kBakeHeaderSize);
    const uint64_t indexOffset = dataOffset + m_payloadBytes;

    std::vector<uint8_t> raw(m_index.size() * kBakeIndexRecordSize, 0);
    for (size_t i = 0; i < m_index.size(); ++i)
    {
        uint8_t* p = raw.data() + i * kBakeIndexRecordSize;
        std::memcpy(p + 0, &m_index[i].key, 8);
        std::memcpy(p + 8, &m_index[i].payloadOffset, 8);
        std::memcpy(p + 16, &m_index[i].payloadBytes, 4);
        p[20] = m_index[i].state;
    }

    if (!raw.empty() && std::fwrite(raw.data(), 1, raw.size(), m_file) != raw.size())
    {
        aError = "cannot write index";
        return false;
    }

    const Vec3 lo = m_region.min;
    const Vec3 hi = m_region.max;

    if (_fseeki64(m_file, 0, SEEK_SET) != 0)
    {
        aError = "cannot rewind to patch header";
        return false;
    }

    const uint32_t version = kBakeVersion;
    const uint32_t chunkDim = static_cast<uint32_t>(kChunkDim);
    const uint32_t chunkCount = static_cast<uint32_t>(m_index.size());
    const uint8_t pad[4] = {0, 0, 0, 0};

    bool ok = std::fwrite(kMagic, 1, 8, m_file) == 8;
    ok = ok && std::fwrite(&version, 1, 4, m_file) == 4;
    ok = ok && std::fwrite(&chunkDim, 1, 4, m_file) == 4;
    ok = ok && std::fwrite(&m_voxelSize, 1, 4, m_file) == 4;
    ok = ok && std::fwrite(pad, 1, 4, m_file) == 4;
    ok = ok && std::fwrite(&chunkCount, 1, 4, m_file) == 4;
    ok = ok && std::fwrite(&indexOffset, 1, 8, m_file) == 8;
    ok = ok && std::fwrite(&dataOffset, 1, 8, m_file) == 8;
    ok = ok && std::fwrite(&lo, 1, 12, m_file) == 12;
    ok = ok && std::fwrite(&hi, 1, 12, m_file) == 12;
    ok = ok && std::fwrite(m_gameBuild, 1, kBakeGameBuildBytes, m_file) == kBakeGameBuildBytes;

    std::fclose(m_file);
    m_file = nullptr;

    if (!ok)
    {
        aError = "cannot write header";
        return false;
    }

    return true;
}
} // namespace vnav
