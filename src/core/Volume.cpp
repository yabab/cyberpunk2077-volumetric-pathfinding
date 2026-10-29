// Volume is header-only; this TU exists so the header is compiled standalone and cannot
// silently acquire a dependency on include order elsewhere.
#include "core/Volume.hpp"

namespace vnav
{
static_assert(kChunkVoxels == 4096, "chunk geometry changed - revisit ByteSize and the clearance array");

// One byte of clearance per voxel, so a mixed chunk is ~4 KB - four times what the old pair of
// bitsets cost. That is the price of storing distance instead of per-agent occupancy, and it is
// what lets a single baked volume serve every agent size. At the default 32768 resident chunks
// this caps the runtime at roughly 135 MB.
static_assert(sizeof(Chunk) <= 4160, "Chunk grew unexpectedly - LRU budgets assume ~4KB");
} // namespace vnav
