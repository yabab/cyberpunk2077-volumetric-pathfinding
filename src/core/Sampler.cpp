#include "core/Sampler.hpp"

#include <vector>

namespace vnav
{
namespace
{
struct Node
{
    int32_t lx, ly, lz, size;
};

struct SampleCtx
{
    IWorldProbe& probe;
    Chunk& chunk;
    IVec3 originVoxel; // world voxel coord of this chunk's (0,0,0) corner
    float voxelSize;
    float hardInflate; // agentRadius
    float softInflate; // agentRadius + margin
    CollisionMask mask;
    int32_t maxDepth;
    int32_t probeBudget;

    // Clearance value meaning "fits at agentRadius, but not at agentRadius + margin".
    uint8_t tightValue;

    int32_t probes = 0;
    bool exhausted = false;
    bool anyFree = false;
    bool anyBlocked = false;
};

// Record a node the agent cannot pass freely.
//
// A probe answers one bit - "is this inflated box clear?" - so probing yields a three-level
// approximation of the clearance field rather than a true distance: wide open, squeezing, or
// blocked. That is exactly the information the old pair of bitsets carried, so the probe path is
// unchanged in behaviour; it just writes it in the encoding the baked path can express fully.
void MarkOccupied(SampleCtx& c, const Node& n, bool aHardBlocked)
{
    const uint8_t value = aHardBlocked ? 0 : c.tightValue;

    for (int32_t z = n.lz; z < n.lz + n.size; ++z)
    {
        for (int32_t y = n.ly; y < n.ly + n.size; ++y)
        {
            for (int32_t x = n.lx; x < n.lx + n.size; ++x)
            {
                c.chunk.SetClearanceLocal(Chunk::LocalIndex(x, y, z), value);
            }
        }
    }
}

bool Overlaps(SampleCtx& c, const Node& n, float aInflation)
{
    const float s = c.voxelSize;
    const float half = static_cast<float>(n.size) * s * 0.5f;
    const Vec3 lo{static_cast<float>(c.originVoxel.x + n.lx) * s, static_cast<float>(c.originVoxel.y + n.ly) * s,
                  static_cast<float>(c.originVoxel.z + n.lz) * s};
    const Vec3 center{lo.x + half, lo.y + half, lo.z + half};
    const float h = half + aInflation;
    return c.probe.OverlapBox(center, Vec3{h, h, h}, c.mask);
}

bool AllClearance(const std::array<uint8_t, kChunkVoxels>& a, uint8_t aValue)
{
    for (uint8_t v : a)
    {
        if (v != aValue)
        {
            return false;
        }
    }
    return true;
}
} // namespace

std::shared_ptr<const Chunk> SampleChunk(const Volume& aVolume, const IVec3& aChunkCoord, IWorldProbe& aProbe,
                                         SampleStats& aStats)
{
    const VolumeConfig& cfg = aVolume.Config();

    auto chunk = std::make_shared<Chunk>();
    chunk->state = ChunkState::Mixed;

    // Start wide open; the traversal only ever writes the voxels it finds occupied.
    chunk->clearance.fill(kMaxClearance);

    // One below the tight threshold reads as "fits, but squeezing". If margin is too small to
    // span a voxel the two thresholds coincide, the band vanishes, and tight space simply stops
    // being penalised - a lost hint, never an unsafe answer.
    const uint8_t block = aVolume.BlockThreshold();
    const uint8_t tight = aVolume.TightThreshold();
    const uint8_t tightValue = tight > block ? static_cast<uint8_t>(tight - 1) : block;

    SampleCtx ctx{aProbe,
                  *chunk,
                  IVec3{aChunkCoord.x * kChunkDim, aChunkCoord.y * kChunkDim, aChunkCoord.z * kChunkDim},
                  cfg.voxelSize,
                  cfg.agentRadius,
                  cfg.agentRadius + cfg.margin,
                  cfg.collision,
                  cfg.maxProbeDepth,
                  cfg.maxProbesPerChunk,
                  tightValue};

    // Level-order (breadth-first) octree refinement.
    //
    // Depth-first would spend the whole probe budget resolving one octant in perfect detail and
    // then declare the rest of the chunk solid, which silently seals off whatever happened to be
    // last in the traversal - a doorway, a window, a shaft. Refining level by level means running
    // out of budget degrades the WHOLE chunk to a coarser, uniformly conservative map instead of
    // corrupting an arbitrary part of it.
    std::vector<Node> current;
    std::vector<Node> next;
    current.push_back(Node{0, 0, 0, kChunkDim});

    for (int32_t depth = 0;; ++depth)
    {
        if (current.empty())
        {
            break;
        }

        // One soft probe per node in this level. If we cannot afford the level at all, everything
        // still queued becomes solid at the parent's resolution.
        if (ctx.probes + static_cast<int32_t>(current.size()) > ctx.probeBudget)
        {
            for (const Node& n : current)
            {
                MarkOccupied(ctx, n, true);
            }
            ctx.exhausted = true;
            ctx.anyBlocked = true;
            break;
        }

        const bool leafLevel = depth >= ctx.maxDepth || current.front().size == 1;

        next.clear();
        for (const Node& n : current)
        {
            // Probe with the LARGER (margin) inflation. Emptiness at r+m implies emptiness at r,
            // and an inflated child box is always contained in the inflated parent box, so the
            // early-out stays sound.
            ++ctx.probes;
            if (!Overlaps(ctx, n, ctx.softInflate))
            {
                ctx.anyFree = true;
                continue; // free at both radii; bits are already clear
            }

            if (leafLevel)
            {
                // Tight for certain. One more probe decides whether the agent fits here at all.
                bool hardBlocked = true;
                if (ctx.softInflate > ctx.hardInflate && ctx.probes < ctx.probeBudget)
                {
                    ++ctx.probes;
                    hardBlocked = Overlaps(ctx, n, ctx.hardInflate);
                }

                MarkOccupied(ctx, n, hardBlocked);
                (hardBlocked ? ctx.anyBlocked : ctx.anyFree) = true;
            }
            else
            {
                const int32_t h = n.size / 2;
                for (int32_t dz = 0; dz < 2; ++dz)
                {
                    for (int32_t dy = 0; dy < 2; ++dy)
                    {
                        for (int32_t dx = 0; dx < 2; ++dx)
                        {
                            next.push_back(Node{n.lx + dx * h, n.ly + dy * h, n.lz + dz * h, h});
                        }
                    }
                }
            }
        }

        if (leafLevel)
        {
            break;
        }
        current.swap(next);
    }

    aStats.probes = ctx.probes;
    aStats.budgetExhausted = ctx.exhausted;

    // Collapse uniform results onto the shared singletons. Open sky is overwhelmingly the common
    // case and costs a map entry rather than a kilobyte.
    if (AllClearance(chunk->clearance, kMaxClearance))
    {
        return Chunk::EmptyChunk();
    }
    if (AllClearance(chunk->clearance, 0))
    {
        return Chunk::SolidChunk();
    }

    chunk->probeCount = static_cast<uint32_t>(ctx.probes);
    return chunk;
}


// ---------------------------------------------------------------------------------------------
// ChunkSampler - the same octree walk, resumable
// ---------------------------------------------------------------------------------------------

ChunkSampler::ChunkSampler(const Volume& aVolume, const IVec3& aChunkCoord)
{
    const VolumeConfig& cfg = aVolume.Config();

    m_chunk = std::make_shared<Chunk>();
    m_chunk->state = ChunkState::Mixed;
    m_chunk->clearance.fill(kMaxClearance);

    m_originVoxel = IVec3{aChunkCoord.x * kChunkDim, aChunkCoord.y * kChunkDim, aChunkCoord.z * kChunkDim};
    m_voxelSize = cfg.voxelSize;
    m_hardInflate = cfg.agentRadius;
    m_softInflate = cfg.agentRadius + cfg.margin;
    m_mask = cfg.collision;
    m_maxDepth = cfg.maxProbeDepth;
    m_probeBudget = cfg.maxProbesPerChunk;

    const uint8_t block = aVolume.BlockThreshold();
    const uint8_t tight = aVolume.TightThreshold();
    m_tightValue = tight > block ? static_cast<uint8_t>(tight - 1) : block;

    m_current.push_back(Node{0, 0, 0, kChunkDim});
    BeginLevel();
}

bool ChunkSampler::Overlaps(IWorldProbe& aProbe, const Node& aNode, float aInflation) const
{
    const float s = m_voxelSize;
    const float half = static_cast<float>(aNode.size) * s * 0.5f;
    const Vec3 lo{static_cast<float>(m_originVoxel.x + aNode.lx) * s,
                  static_cast<float>(m_originVoxel.y + aNode.ly) * s,
                  static_cast<float>(m_originVoxel.z + aNode.lz) * s};
    const Vec3 center{lo.x + half, lo.y + half, lo.z + half};
    const float h = half + aInflation;
    return aProbe.OverlapBox(center, Vec3{h, h, h}, m_mask);
}

void ChunkSampler::MarkOccupied(const Node& aNode, bool aHardBlocked)
{
    const uint8_t value = aHardBlocked ? 0 : m_tightValue;
    for (int32_t z = aNode.lz; z < aNode.lz + aNode.size; ++z)
    {
        for (int32_t y = aNode.ly; y < aNode.ly + aNode.size; ++y)
        {
            for (int32_t x = aNode.lx; x < aNode.lx + aNode.size; ++x)
            {
                m_chunk->SetClearanceLocal(Chunk::LocalIndex(x, y, z), value);
            }
        }
    }
}

// Entering a level is where the per-chunk budget is enforced, and it stays a WHOLE-LEVEL decision.
// Refinement is breadth-first precisely so that running out degrades the whole chunk to a coarser
// uniform map rather than sealing off one arbitrary octant - splitting the work across frames must
// not quietly turn that into a depth-first failure.
void ChunkSampler::BeginLevel()
{
    m_cursor = 0;

    if (m_current.empty())
    {
        m_done = true;
        return;
    }

    if (m_probes + static_cast<int32_t>(m_current.size()) > m_probeBudget)
    {
        for (const Node& n : m_current)
        {
            MarkOccupied(n, true);
        }
        m_exhausted = true;
        m_done = true;
        return;
    }

    m_leafLevel = m_depth >= m_maxDepth || m_current.front().size == 1;
}

bool ChunkSampler::Step(IWorldProbe& aProbe, int32_t aProbeSlice)
{
    if (m_done)
    {
        return true;
    }

    int32_t spent = 0;
    while (!m_done)
    {
        if (m_cursor >= m_current.size())
        {
            if (m_leafLevel)
            {
                m_done = true;
                break;
            }
            m_current.swap(m_next);
            m_next.clear();
            ++m_depth;
            BeginLevel();
            continue;
        }

        // Always make at least one probe of progress, so a tiny slice cannot livelock.
        if (spent >= aProbeSlice && spent > 0)
        {
            return false;
        }

        const Node n = m_current[m_cursor++];

        // Probe with the LARGER (margin) inflation. Emptiness at r+m implies emptiness at r, and an
        // inflated child box is always contained in the inflated parent box, so the early-out is sound.
        ++m_probes;
        ++spent;
        if (!Overlaps(aProbe, n, m_softInflate))
        {
            continue; // free at both radii; clearance is already kMaxClearance here
        }

        if (m_leafLevel)
        {
            bool hardBlocked = true;
            if (m_softInflate > m_hardInflate && m_probes < m_probeBudget)
            {
                ++m_probes;
                ++spent;
                hardBlocked = Overlaps(aProbe, n, m_hardInflate);
            }
            MarkOccupied(n, hardBlocked);
        }
        else
        {
            const int32_t h = n.size / 2;
            for (int32_t dz = 0; dz < 2; ++dz)
            {
                for (int32_t dy = 0; dy < 2; ++dy)
                {
                    for (int32_t dx = 0; dx < 2; ++dx)
                    {
                        m_next.push_back(Node{n.lx + dx * h, n.ly + dy * h, n.lz + dz * h, h});
                    }
                }
            }
        }
    }

    return m_done;
}

std::shared_ptr<const Chunk> ChunkSampler::Take()
{
    if (AllClearance(m_chunk->clearance, kMaxClearance))
    {
        return Chunk::EmptyChunk();
    }
    if (AllClearance(m_chunk->clearance, 0))
    {
        return Chunk::SolidChunk();
    }
    m_chunk->probeCount = static_cast<uint32_t>(m_probes);
    return m_chunk;
}
} // namespace vnav
