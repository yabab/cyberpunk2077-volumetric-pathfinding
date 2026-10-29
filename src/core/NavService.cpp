#include <vnav/NavService.hpp>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <limits>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "core/Planner.hpp"
#include "core/Sampler.hpp"
#include "core/Volume.hpp"

namespace vnav
{
namespace
{
using Clock = std::chrono::steady_clock;

// Async park bounds now live in PathQuery (maxParks / noProgressParks) so a caller can let a
// search run as long as it keeps making progress. The ping-pong case this constant used to guard
// - a job needing chunks that keep getting evicted - is caught by the stall guard instead, which
// stops a search that is not improving rather than one that has merely worked hard.

// Inline (game-thread) sync searches keep a hard round cap: a caller is blocked on this one.
constexpr int32_t kMaxInlineRounds = 512;

float MillisSince(Clock::time_point a, Clock::time_point b)
{
    return std::chrono::duration<float, std::milli>(b - a).count();
}
} // namespace

// -------------------------------------------------------------------------------------------
// Job
// -------------------------------------------------------------------------------------------

struct PathJob
{
    uint32_t id = 0;

    // Held by shared_ptr so DestroyVolume during an in-flight search cannot dangle: the volume
    // simply stays alive until the last job using it lets go.
    std::shared_ptr<Volume> volume;

    PathQuery query;
    PathCallback callback;

    std::atomic<Status> status{Status::Pending};
    std::atomic<bool> cancelled{false};

    std::unique_ptr<Planner> planner;
    std::unique_ptr<PinnedView> view;

    // Only written before `status` becomes terminal, only read after it has. The release/acquire
    // pair on `status` is what publishes it.
    PathResult result;

    std::vector<uint64_t> pending; // chunks this job is parked on
    int32_t parkCount = 0;
    int32_t chunksSampled = 0;
    int32_t probes = 0;

    // Progress tracking for the persistence guard. `bestH` is the closest the search has ever
    // got to the goal; `stalledParks` counts consecutive parks that failed to beat it. Only
    // touched from the worker in ParkJob, so no synchronisation is needed.
    float bestH = std::numeric_limits<float>::max();
    int32_t stalledParks = 0;

    Clock::time_point submitted;
    Clock::time_point finished;

    std::mutex doneMutex;
    std::condition_variable doneCv;
};

// -------------------------------------------------------------------------------------------
// Impl
// -------------------------------------------------------------------------------------------

struct NavService::Impl
{
    struct SampleRequest
    {
        std::shared_ptr<Volume> volume;
        uint64_t key;
        std::weak_ptr<PathJob> requester;
    };

    struct ReqKey
    {
        Volume* volume;
        uint64_t key;
        bool operator==(const ReqKey& o) const { return volume == o.volume && key == o.key; }
    };

    struct ReqKeyHash
    {
        size_t operator()(const ReqKey& k) const
        {
            const auto a = reinterpret_cast<uintptr_t>(k.volume);
            return static_cast<size_t>((a * 0x9E3779B97F4A7C15ull) ^ (k.key * 0xC2B2AE3D27D4EB4Full));
        }
    };

    explicit Impl(const ServiceConfig& aConfig)
        : config(aConfig)
    {
        if (config.workerCount <= 0)
        {
            const auto hw = static_cast<int32_t>(std::thread::hardware_concurrency());
            config.workerCount = std::clamp(hw / 4, 1, 4);
        }
        config.workerCount = std::clamp(config.workerCount, 1, 16);
    }

    // --- volumes -----------------------------------------------------------------------------
    mutable std::shared_mutex volumeMutex;
    std::unordered_map<std::string, std::shared_ptr<Volume>> volumes;

    std::shared_ptr<Volume> FindVolume(const std::string& aName) const
    {
        std::shared_lock lock(volumeMutex);
        auto it = volumes.find(aName);
        return it == volumes.end() ? nullptr : it->second;
    }

    // --- jobs --------------------------------------------------------------------------------
    mutable std::mutex jobMutex;
    std::unordered_map<uint32_t, std::shared_ptr<PathJob>> jobs;
    std::atomic<uint32_t> nextJobId{1};

    std::mutex readyMutex;
    std::condition_variable readyCv;
    std::deque<std::shared_ptr<PathJob>> ready;

    std::mutex parkedMutex;
    std::vector<std::shared_ptr<PathJob>> parked;

    std::mutex callbackMutex;
    std::vector<std::shared_ptr<PathJob>> completed;

    // --- sampling ----------------------------------------------------------------------------
    std::mutex sampleMutex;
    std::deque<SampleRequest> sampleQueue;
    std::unordered_set<ReqKey, ReqKeyHash> sampleDedup;

    // Chunks part-way through sampling, suspended between pumps. Game thread only (DrainSamples is
    // the sole toucher), so no guard - unlike the queue above, which producers on worker threads
    // also push to.
    std::unordered_map<ReqKey, std::unique_ptr<ChunkSampler>, ReqKeyHash> partialSamples;

    // --- runtime -----------------------------------------------------------------------------
    ServiceConfig config;
    std::atomic<IWorldProbe*> probe{nullptr};
    NullWorldProbe nullProbe;

    std::vector<std::thread> workers;
    std::atomic<bool> running{false};

    std::atomic<uint64_t> tick{1};
    std::thread::id gameThread{};
    std::atomic<bool> gameThreadSet{false};

    // --- stats -------------------------------------------------------------------------------
    std::atomic<int64_t> statQueries{0};
    std::atomic<int64_t> statSucceeded{0};
    std::atomic<int64_t> statPartial{0};
    std::atomic<int64_t> statFailed{0};
    std::atomic<int64_t> statProbes{0};
    std::atomic<int64_t> statChunks{0};
    std::atomic<int64_t> statBaked{0};

    // ---------------------------------------------------------------------------------------

    bool IsGameThread() const
    {
        return gameThreadSet.load(std::memory_order_acquire) && std::this_thread::get_id() == gameThread;
    }

    IWorldProbe& Probe()
    {
        IWorldProbe* p = probe.load(std::memory_order_acquire);
        return p ? *p : nullProbe;
    }

    void Start()
    {
        running.store(true, std::memory_order_release);
        workers.reserve(static_cast<size_t>(config.workerCount));
        for (int32_t i = 0; i < config.workerCount; ++i)
        {
            workers.emplace_back([this] { WorkerMain(); });
        }
    }

    void Stop()
    {
        if (!running.exchange(false))
        {
            return;
        }

        {
            std::lock_guard lock(jobMutex);
            for (auto& [id, job] : jobs)
            {
                job->cancelled.store(true, std::memory_order_relaxed);
            }
        }

        readyCv.notify_all();
        for (auto& t : workers)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
        workers.clear();

        // Wake anything blocked in the off-thread sync path so it exits with a terminal status
        // rather than waiting on workers that no longer exist.
        std::vector<std::shared_ptr<PathJob>> stragglers;
        {
            std::lock_guard lock(jobMutex);
            for (auto& [id, job] : jobs)
            {
                if (!IsTerminal(job->status.load(std::memory_order_acquire)))
                {
                    stragglers.push_back(job);
                }
            }
        }
        for (auto& job : stragglers)
        {
            FinishJob(job, Status::Cancelled);
        }

        {
            std::lock_guard lock(jobMutex);
            jobs.clear();
        }
        {
            std::lock_guard lock(readyMutex);
            ready.clear();
        }
        {
            std::lock_guard lock(parkedMutex);
            parked.clear();
        }
        {
            std::lock_guard lock(callbackMutex);
            completed.clear();
        }
        {
            std::lock_guard lock(sampleMutex);
            sampleQueue.clear();
            sampleDedup.clear();
        }
    }

    void Enqueue(std::shared_ptr<PathJob> aJob)
    {
        {
            std::lock_guard lock(readyMutex);
            ready.push_back(std::move(aJob));
        }
        readyCv.notify_one();
    }

    void WorkerMain()
    {
        while (true)
        {
            std::shared_ptr<PathJob> job;
            {
                std::unique_lock lock(readyMutex);
                readyCv.wait(lock, [this] { return !running.load(std::memory_order_acquire) || !ready.empty(); });
                if (!running.load(std::memory_order_acquire))
                {
                    return;
                }
                job = std::move(ready.front());
                ready.pop_front();
            }
            RunSlice(job);
        }
    }

    void RunSlice(const std::shared_ptr<PathJob>& aJob)
    {
        if (aJob->cancelled.load(std::memory_order_relaxed))
        {
            FinishJob(aJob, Status::Cancelled);
            return;
        }

        // External budgets (wall clock, park thrashing) stop the search but must not throw away
        // what it already found - a partial path is the whole point of allowPartial.
        if (aJob->query.timeoutSeconds > 0.0f &&
            MillisSince(aJob->submitted, Clock::now()) > aJob->query.timeoutSeconds * 1000.0f)
        {
            aJob->planner->Abort(*aJob->view, Status::TimedOut);
            FinishJob(aJob, aJob->planner->Result().status, &aJob->planner->MutableResult());
            return;
        }

        // Two ways an async search can be stopped short, and they mean different things.
        //
        // An explicit park ceiling (opt-in, 0 == unlimited and is the default) bounds raw effort.
        // The stall guard bounds LACK OF PROGRESS, and it is the one that matters: a search
        // grinding through complex geometry keeps closing on the goal and is allowed to continue
        // for as long as it takes, while one that cannot improve - trapped under a slab, or aimed
        // at a goal sealed inside geometry - is cut off quickly instead of burning the full
        // timeout to discover the same thing.
        if (aJob->query.maxParks > 0 && aJob->parkCount >= aJob->query.maxParks)
        {
            aJob->planner->Abort(*aJob->view, Status::BudgetExhausted);
            FinishJob(aJob, aJob->planner->Result().status, &aJob->planner->MutableResult());
            return;
        }

        if (aJob->query.noProgressParks > 0 && aJob->stalledParks >= aJob->query.noProgressParks)
        {
            aJob->planner->Abort(*aJob->view, Status::BudgetExhausted);
            FinishJob(aJob, aJob->planner->Result().status, &aJob->planner->MutableResult());
            return;
        }

        aJob->status.store(Status::Running, std::memory_order_release);

        const SliceResult r = aJob->planner->Step(*aJob->view, aJob->cancelled);

        switch (r)
        {
        case SliceResult::Finished:
            FinishJob(aJob, aJob->planner->Result().status, &aJob->planner->MutableResult());
            break;

        case SliceResult::Cancelled:
            FinishJob(aJob, Status::Cancelled);
            break;

        case SliceResult::NeedsWorld:
            ParkJob(aJob);
            break;
        }
    }

    void ParkJob(const std::shared_ptr<PathJob>& aJob)
    {
        // Order matters. `pending` is written first, then the sample requests are queued, and
        // only then does the job become visible to Pump via the parked list. The parked-list
        // mutex is the release/acquire edge that publishes `pending`.
        aJob->pending = aJob->planner->NeededChunks();
        aJob->planner->ClearNeeded();
        ++aJob->parkCount;

        // Did this slice get us any closer? The epsilon keeps floating-point noise from reading
        // as progress and defeating the guard.
        constexpr float kProgressEpsilon = 0.01f;
        const float h = aJob->planner->BestHeuristic();
        if (h < aJob->bestH - kProgressEpsilon)
        {
            aJob->bestH = h;
            aJob->stalledParks = 0;
        }
        else
        {
            ++aJob->stalledParks;
        }

        QueueSamples(aJob->volume, aJob->pending, aJob);

        aJob->status.store(Status::WaitingForWorld, std::memory_order_release);
        {
            std::lock_guard lock(parkedMutex);
            parked.push_back(aJob);
        }
    }

    void QueueSamples(const std::shared_ptr<Volume>& aVolume, const std::vector<uint64_t>& aKeys,
                      const std::shared_ptr<PathJob>& aRequester)
    {
        std::lock_guard lock(sampleMutex);
        for (uint64_t key : aKeys)
        {
            const ReqKey rk{aVolume.get(), key};
            if (sampleDedup.insert(rk).second)
            {
                sampleQueue.push_back(SampleRequest{aVolume, key, aRequester});
            }
        }
    }

    void FinishJob(const std::shared_ptr<PathJob>& aJob, Status aStatus, PathResult* aFrom = nullptr)
    {
        // Guard against a job being finished twice (e.g. cancel racing completion).
        Status expected = aJob->status.load(std::memory_order_acquire);
        if (IsTerminal(expected))
        {
            return;
        }

        if (aFrom)
        {
            aJob->result = std::move(*aFrom);
        }

        aJob->result.parkCount = aJob->parkCount;
        aJob->result.chunksSampled = aJob->chunksSampled;
        aJob->result.probes = aJob->probes;
        aJob->finished = Clock::now();
        aJob->result.elapsedMs = MillisSince(aJob->submitted, aJob->finished);

        // Drop the planner and the pinned view now, not at reap time: the view is holding
        // shared_ptrs to chunks and would otherwise pin them for resultRetentionSeconds.
        aJob->planner.reset();
        aJob->view.reset();

        switch (aStatus)
        {
        case Status::Success: statSucceeded.fetch_add(1, std::memory_order_relaxed); break;
        case Status::PartialSuccess: statPartial.fetch_add(1, std::memory_order_relaxed); break;
        default: statFailed.fetch_add(1, std::memory_order_relaxed); break;
        }

        aJob->result.status = aStatus;

        // Everything above is published by this store; Poll() acquires on it before reading.
        {
            std::lock_guard lock(aJob->doneMutex);
            aJob->status.store(aStatus, std::memory_order_release);
        }
        aJob->doneCv.notify_all();

        if (aJob->callback)
        {
            std::lock_guard lock(callbackMutex);
            completed.push_back(aJob);
        }
    }

    // --- the pump ----------------------------------------------------------------------------

    // Serve a chunk from the volume's bake, if it has one and the chunk has not been
    // invalidated out from under it. Costs a small file read plus a local distance transform -
    // no physics, no streaming, and correct at any range.
    bool TryBaked(Volume& aVolume, uint64_t aKey)
    {
        const std::shared_ptr<BakedNav>& baked = aVolume.Baked();
        if (!baked || !baked->IsOpen() || aVolume.IsProbeOnly(aKey))
        {
            return false;
        }

        auto chunk = baked->Load(UnpackChunkKey(aKey), aVolume.TightThreshold());
        if (!chunk)
        {
            return false; // outside the baked region - fall through to probing
        }

        aVolume.Store().Publish(aKey, std::move(chunk), tick.load(std::memory_order_relaxed));
        aVolume.AddChunksSampled(1);
        statChunks.fetch_add(1, std::memory_order_relaxed);
        statBaked.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    void DrainSamples(int32_t aBudgetMicros)
    {
        IWorldProbe* p = probe.load(std::memory_order_acquire);

        const auto start = Clock::now();
        const auto budget = std::chrono::microseconds(std::max(0, aBudgetMicros));
        bool first = true;

        while (true)
        {
            // Always sample at least one chunk per pump so a tiny budget still makes progress.
            if (!first && Clock::now() - start >= budget)
            {
                break;
            }
            first = false;

            SampleRequest req;
            {
                std::lock_guard lock(sampleMutex);
                if (sampleQueue.empty())
                {
                    break;
                }
                req = std::move(sampleQueue.front());
                sampleQueue.pop_front();
                sampleDedup.erase(ReqKey{req.volume.get(), req.key});
            }

            if (req.volume->Store().Contains(req.key))
            {
                continue; // sampled by someone else in the meantime
            }

            if (TryBaked(*req.volume, req.key))
            {
                if (auto owner = req.requester.lock())
                {
                    owner->chunksSampled += 1;
                }
                continue;
            }

            if (!p)
            {
                // No world to probe and no bake covering this chunk. Put it back so the parked
                // job still has a request outstanding when a probe does arrive - dropping it
                // would leave that job waiting on a chunk nobody is going to produce.
                std::lock_guard lock(sampleMutex);
                sampleDedup.insert(ReqKey{req.volume.get(), req.key});
                sampleQueue.push_front(std::move(req));
                break;
            }

            // Resumable: a chunk too expensive to finish this frame is suspended and picked up
            // next pump, so one pathological chunk cannot overrun a frame. It is NOT published
            // until complete - a half-sampled chunk would read as a map with holes in it - so the
            // requesting job simply keeps parking on residency, which it already handles.
            const ReqKey rk{req.volume.get(), req.key};
            auto it = partialSamples.find(rk);
            if (it == partialSamples.end())
            {
                it = partialSamples.emplace(rk, std::make_unique<ChunkSampler>(*req.volume, UnpackChunkKey(req.key)))
                         .first;
            }

            const int32_t slice = config.maxProbesPerChunkPerPump > 0 ? config.maxProbesPerChunkPerPump
                                                                      : std::numeric_limits<int32_t>::max();
            const int32_t before = it->second->Probes();
            const bool complete = it->second->Step(*p, slice);
            const int32_t spent = it->second->Probes() - before;

            req.volume->AddProbes(spent);
            statProbes.fetch_add(spent, std::memory_order_relaxed);
            if (auto owner = req.requester.lock())
            {
                owner->probes += spent;
            }

            if (!complete)
            {
                // Requeue at the FRONT: this chunk is already half paid for, and a job is parked
                // waiting on it. Sending it to the back would let newer requests starve it.
                std::lock_guard lock(sampleMutex);
                sampleDedup.insert(rk);
                sampleQueue.push_front(std::move(req));
                continue;
            }

            auto chunk = it->second->Take();
            partialSamples.erase(it);
            req.volume->Store().Publish(req.key, std::move(chunk), tick.load(std::memory_order_relaxed));

            req.volume->AddChunksSampled(1);
            statChunks.fetch_add(1, std::memory_order_relaxed);

            // Approximate attribution: a chunk requested by several jobs is credited to whichever
            // asked first, since the later requests were deduplicated away.
            if (auto owner = req.requester.lock())
            {
                owner->chunksSampled += 1;
            }
        }
    }

    void WakeParked()
    {
        std::vector<std::shared_ptr<PathJob>> wake;
        {
            std::lock_guard lock(parkedMutex);
            for (auto it = parked.begin(); it != parked.end();)
            {
                const auto& job = *it;

                bool ready_ = job->cancelled.load(std::memory_order_relaxed);
                if (!ready_)
                {
                    ready_ = std::all_of(job->pending.begin(), job->pending.end(),
                                         [&](uint64_t k) { return job->volume->Store().Contains(k); });
                }

                if (ready_)
                {
                    wake.push_back(job);
                    it = parked.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        for (auto& job : wake)
        {
            Enqueue(std::move(job));
        }
    }

    void RunCallbacks()
    {
        std::vector<std::shared_ptr<PathJob>> batch;
        {
            std::lock_guard lock(callbackMutex);
            batch.swap(completed);
        }
        for (auto& job : batch)
        {
            if (job->callback)
            {
                job->callback(job->id, job->result);
                job->callback = {}; // fire exactly once
            }
        }
    }

    void MaintainVolumes()
    {
        std::vector<std::shared_ptr<Volume>> snapshot;
        {
            std::shared_lock lock(volumeMutex);
            snapshot.reserve(volumes.size());
            for (auto& [name, vol] : volumes)
            {
                snapshot.push_back(vol);
            }
        }

        const uint64_t now = tick.load(std::memory_order_relaxed);
        for (auto& vol : snapshot)
        {
            const auto& cfg = vol->Config();
            if (cfg.chunkTtlSeconds > 0.0f)
            {
                // Pump is nominally per-frame; 60 fps is close enough for a coarse TTL.
                const auto ttlTicks = static_cast<uint64_t>(cfg.chunkTtlSeconds * 60.0f);
                if (now > ttlTicks)
                {
                    vol->Store().EvictOlderThan(now - ttlTicks);
                }
            }
            vol->Store().EvictTo(static_cast<size_t>(cfg.maxResidentChunks));
        }
    }

    void ReapJobs()
    {
        const auto now = Clock::now();
        std::lock_guard lock(jobMutex);
        for (auto it = jobs.begin(); it != jobs.end();)
        {
            const auto& job = it->second;
            const bool terminal = IsTerminal(job->status.load(std::memory_order_acquire));
            const bool expired =
                terminal && MillisSince(job->finished, now) > config.resultRetentionSeconds * 1000.0f;
            it = expired ? jobs.erase(it) : std::next(it);
        }
    }
};

// -------------------------------------------------------------------------------------------
// NavService
// -------------------------------------------------------------------------------------------

NavService::NavService(const ServiceConfig& aConfig)
    : m(std::make_unique<Impl>(aConfig))
{
    m->Start();
}

NavService::~NavService()
{
    m->Stop();
}

void NavService::SetGameThread()
{
    m->gameThread = std::this_thread::get_id();
    m->gameThreadSet.store(true, std::memory_order_release);
}

void NavService::SetProbe(IWorldProbe* aProbe)
{
    m->probe.store(aProbe, std::memory_order_release);
}

void NavService::Shutdown()
{
    m->Stop();
}

bool NavService::CreateVolume(const std::string& aName, const VolumeConfig& aConfig)
{
    if (aName.empty() || !aConfig.IsValid())
    {
        return false;
    }

    std::unique_lock lock(m->volumeMutex);
    if (m->volumes.find(aName) != m->volumes.end())
    {
        return false;
    }
    m->volumes.emplace(aName, std::make_shared<Volume>(aName, aConfig));
    return true;
}

bool NavService::DestroyVolume(const std::string& aName)
{
    std::unique_lock lock(m->volumeMutex);
    // In-flight jobs hold their own shared_ptr, so this only removes the name binding.
    return m->volumes.erase(aName) > 0;
}

bool NavService::HasVolume(const std::string& aName) const
{
    return m->FindVolume(aName) != nullptr;
}

bool NavService::GetVolumeConfig(const std::string& aName, VolumeConfig& aOut) const
{
    auto vol = m->FindVolume(aName);
    if (!vol)
    {
        return false;
    }
    aOut = vol->Config();
    return true;
}

std::vector<std::string> NavService::GetVolumeNames() const
{
    std::vector<std::string> names;
    std::shared_lock lock(m->volumeMutex);
    names.reserve(m->volumes.size());
    for (const auto& [name, vol] : m->volumes)
    {
        names.push_back(name);
    }
    return names;
}

uint32_t NavService::SubmitAsync(const std::string& aVolume, const PathQuery& aQuery, PathCallback aCallback)
{
    auto vol = m->FindVolume(aVolume);
    if (!vol || !aQuery.IsValid() || !m->running.load(std::memory_order_acquire))
    {
        return 0;
    }

    {
        std::lock_guard lock(m->jobMutex);
        if (static_cast<int32_t>(m->jobs.size()) >= m->config.maxConcurrentJobs)
        {
            return 0;
        }
    }

    auto job = std::make_shared<PathJob>();
    job->id = m->nextJobId.fetch_add(1, std::memory_order_relaxed);
    job->volume = vol;
    job->query = aQuery;
    job->callback = std::move(aCallback);
    job->submitted = Clock::now();
    job->planner = std::make_unique<Planner>(*vol, aQuery);
    job->view = std::make_unique<PinnedView>(*vol, m->tick.load(std::memory_order_relaxed));

    {
        std::lock_guard lock(m->jobMutex);
        m->jobs.emplace(job->id, job);
    }
    m->statQueries.fetch_add(1, std::memory_order_relaxed);

    // Warm the straight-line corridor so the first slices rarely have to park.
    std::vector<uint64_t> prefetch;
    job->planner->CollectPrefetch(prefetch);
    m->QueueSamples(vol, prefetch, job);

    m->Enqueue(job);
    return job->id;
}

Status NavService::Poll(uint32_t aJobId, PathResult& aOut)
{
    std::shared_ptr<PathJob> job;
    {
        std::lock_guard lock(m->jobMutex);
        auto it = m->jobs.find(aJobId);
        if (it == m->jobs.end())
        {
            return Status::Invalid;
        }
        job = it->second;
    }

    // Acquire pairs with the release in FinishJob; everything written to `result` before that
    // store is visible here.
    const Status s = job->status.load(std::memory_order_acquire);
    if (IsTerminal(s))
    {
        aOut = job->result;
    }
    return s;
}

bool NavService::Cancel(uint32_t aJobId)
{
    std::shared_ptr<PathJob> job;
    {
        std::lock_guard lock(m->jobMutex);
        auto it = m->jobs.find(aJobId);
        if (it == m->jobs.end())
        {
            return false;
        }
        job = it->second;
    }

    if (IsTerminal(job->status.load(std::memory_order_acquire)))
    {
        return false;
    }

    job->cancelled.store(true, std::memory_order_release);
    return true;
}

Status NavService::FindPathSync(const std::string& aVolume, const PathQuery& aQuery, PathResult& aOut)
{
    aOut.Clear();

    auto vol = m->FindVolume(aVolume);
    if (!vol)
    {
        aOut.status = Status::NoVolume;
        return aOut.status;
    }
    if (!aQuery.IsValid())
    {
        aOut.status = Status::BadInput;
        return aOut.status;
    }

    // ---- off the game thread: submit and wait for someone else to pump --------------------
    if (!m->IsGameThread())
    {
        const uint32_t id = SubmitAsync(aVolume, aQuery);
        if (id == 0)
        {
            aOut.status = Status::BadInput;
            return aOut.status;
        }

        std::shared_ptr<PathJob> job;
        {
            std::lock_guard lock(m->jobMutex);
            auto it = m->jobs.find(id);
            job = it == m->jobs.end() ? nullptr : it->second;
        }
        if (!job)
        {
            aOut.status = Status::Invalid;
            return aOut.status;
        }

        const auto timeout = std::chrono::duration<float>(aQuery.timeoutSeconds > 0.0f ? aQuery.timeoutSeconds : 10.0f);
        std::unique_lock lock(job->doneMutex);
        const bool done = job->doneCv.wait_for(lock, timeout, [&] {
            return IsTerminal(job->status.load(std::memory_order_acquire));
        });
        lock.unlock();

        if (!done)
        {
            job->cancelled.store(true, std::memory_order_release);
            aOut.status = Status::TimedOut;
            return aOut.status;
        }

        aOut = job->result;
        return aOut.status;
    }

    // ---- on the game thread: sample inline, no parking, no deadlock ------------------------
    const auto started = Clock::now();
    const uint64_t stamp = m->tick.load(std::memory_order_relaxed);

    Planner planner(*vol, aQuery);
    PinnedView view(*vol, stamp);
    const std::atomic<bool> neverCancelled{false};

    IWorldProbe& probe = m->Probe();
    int32_t probeBudget = aQuery.maxProbesPerQuery;
    int32_t chunksSampled = 0;
    int32_t probesUsed = 0;
    bool finished = false;
    bool outOfProbes = false;

    for (int32_t round = 0; round < kMaxInlineRounds; ++round)
    {
        const SliceResult r = planner.Step(view, neverCancelled);
        if (r != SliceResult::NeedsWorld)
        {
            finished = true;
            break;
        }

        if (outOfProbes)
        {
            break; // it still wants world we cannot afford to sample
        }

        for (uint64_t key : planner.NeededChunks())
        {
            if (probeBudget <= 0)
            {
                outOfProbes = true;
                break;
            }
            if (vol->Store().Contains(key))
            {
                continue;
            }

            // Baked chunks cost no probes, so they do not draw on the probe budget at all.
            if (m->TryBaked(*vol, key))
            {
                ++chunksSampled;
                continue;
            }

            SampleStats stats;
            auto chunk = SampleChunk(*vol, UnpackChunkKey(key), probe, stats);
            vol->Store().Publish(key, std::move(chunk), stamp);

            probeBudget -= stats.probes;
            probesUsed += stats.probes;
            ++chunksSampled;

            vol->AddProbes(stats.probes);
            vol->AddChunksSampled(1);
            m->statProbes.fetch_add(stats.probes, std::memory_order_relaxed);
            m->statChunks.fetch_add(1, std::memory_order_relaxed);
        }
        planner.ClearNeeded();
    }

    m->statQueries.fetch_add(1, std::memory_order_relaxed);

    // Running out of probe budget is an external limit, not a verdict on the search. Let the
    // planner finalise so a partial path survives instead of being discarded.
    if (!finished)
    {
        planner.Abort(view, Status::BudgetExhausted);
    }

    aOut = planner.Result();
    aOut.probes = probesUsed;
    aOut.chunksSampled = chunksSampled;
    aOut.elapsedMs = MillisSince(started, Clock::now());

    switch (aOut.status)
    {
    case Status::Success: m->statSucceeded.fetch_add(1, std::memory_order_relaxed); break;
    case Status::PartialSuccess: m->statPartial.fetch_add(1, std::memory_order_relaxed); break;
    default: m->statFailed.fetch_add(1, std::memory_order_relaxed); break;
    }

    return aOut.status;
}

bool NavService::AttachBakeChecked(const std::string& aVolume, const std::string& aPath,
                                   const std::string& aExpectedGameBuild, std::string& aError)
{
    // Read the stamp BEFORE attaching, so a mismatched bake is never briefly live.
    if (!aExpectedGameBuild.empty())
    {
        BakedNav probe;
        std::string openError;
        if (!probe.Open(aPath, openError))
        {
            aError = openError;
            return false;
        }

        const std::string stamp = probe.GameBuild();
        probe.Close();

        if (stamp.empty())
        {
            // Unstamped: cannot be checked, and refusing it would break every bake already
            // shipped. Attach, but say so - a silent accept is how the mismatch stays invisible.
            aError = "bake is unstamped, so it cannot be checked against game build '" + aExpectedGameBuild +
                     "'; attaching anyway";
        }
        else if (stamp != aExpectedGameBuild)
        {
            aError = "bake was built from game build '" + stamp + "' but this is '" + aExpectedGameBuild +
                     "' - geometry may have moved; re-bake rather than planning against it";
            return false;
        }
    }

    std::string attachError;
    if (!AttachBake(aVolume, aPath, attachError))
    {
        aError = attachError;
        return false;
    }
    return true;
}

std::string NavService::BakeGameBuild(const std::string& aVolume) const
{
    auto vol = m->FindVolume(aVolume);
    if (!vol || !vol->Baked())
    {
        return {};
    }
    return vol->Baked()->GameBuild();
}

bool NavService::AttachBake(const std::string& aVolume, const std::string& aPath, std::string& aError)
{
    auto vol = m->FindVolume(aVolume);
    if (!vol)
    {
        aError = "no such volume: " + aVolume;
        return false;
    }

    if (!BakedNav::SupportsThreshold(vol->TightThreshold()))
    {
        aError = "volume agent is too large for a baked lookup (needs " +
                 std::to_string(static_cast<int>(vol->TightThreshold())) + " voxels of clearance, limit " +
                 std::to_string(static_cast<int>(kMaxHalo)) + ")";
        return false;
    }

    auto baked = std::make_shared<BakedNav>();
    if (!baked->Open(aPath, aError))
    {
        return false;
    }

    // A bake voxelised at a different resolution would put its occupancy in the wrong places
    // entirely, so this is a hard mismatch rather than something to scale around.
    const float ratio = baked->Header().voxelSize / vol->Config().voxelSize;
    if (ratio < 0.999f || ratio > 1.001f)
    {
        aError = "bake voxel size " + std::to_string(baked->Header().voxelSize) + " does not match volume " +
                 std::to_string(vol->Config().voxelSize);
        return false;
    }

    vol->AttachBake(std::move(baked));

    // Anything already sampled came from probing a partially streamed world; drop it so the
    // bake gets to answer instead.
    vol->ClearProbeOnly();
    vol->Store().Clear();
    return true;
}

bool NavService::HasBake(const std::string& aVolume) const
{
    auto vol = m->FindVolume(aVolume);
    return vol && vol->Baked() && vol->Baked()->IsOpen();
}

void NavService::DetachBake(const std::string& aVolume)
{
    if (auto vol = m->FindVolume(aVolume))
    {
        vol->AttachBake(nullptr);
        vol->ClearProbeOnly();
        vol->Store().Clear();
    }
}

int32_t NavService::ResumeIndex(const std::vector<Vec3>& aPoints, const Vec3& aAgentPosition)
{
    if (aPoints.size() < 2)
    {
        return 0;
    }

    // Nearest SEGMENT, not nearest waypoint. Waypoints can be far apart on an any-angle path, so
    // the nearest waypoint may sit behind the agent while it is already most of the way along the
    // segment leaving it - which is the reversal this exists to prevent.
    int32_t best = 1;
    float bestDistSq = std::numeric_limits<float>::max();

    for (size_t i = 0; i + 1 < aPoints.size(); ++i)
    {
        const Vec3 a = aPoints[i];
        const Vec3 b = aPoints[i + 1];
        const Vec3 ab = b - a;

        const float lenSq = ab.LengthSq();
        float t = 0.0f;
        if (lenSq > 1e-8f)
        {
            t = std::clamp((aAgentPosition - a).Dot(ab) / lenSq, 0.0f, 1.0f);
        }

        const Vec3 closest = a + ab * t;
        const float d = (aAgentPosition - closest).LengthSq();
        if (d < bestDistSq)
        {
            bestDistSq = d;
            best = static_cast<int32_t>(i) + 1; // steer at the FAR end of that segment
        }
    }

    return best;
}

bool NavService::IsFree(const std::string& aVolume, const Vec3& aPoint)
{
    auto vol = m->FindVolume(aVolume);
    if (!vol || !m->IsGameThread() || !aPoint.IsFinite())
    {
        return false;
    }

    const IVec3 voxel = vol->WorldToVoxel(aPoint);
    if (!vol->InBounds(voxel))
    {
        return false;
    }

    const uint64_t key = PackChunkKey(Volume::VoxelToChunk(voxel));
    const uint64_t stamp = m->tick.load(std::memory_order_relaxed);

    if (!vol->Store().Contains(key) && !m->TryBaked(*vol, key))
    {
        IWorldProbe& probe = m->Probe();
        SampleStats stats;
        auto chunk = SampleChunk(*vol, UnpackChunkKey(key), probe, stats);
        vol->Store().Publish(key, std::move(chunk), stamp);

        vol->AddProbes(stats.probes);
        vol->AddChunksSampled(1);
        m->statProbes.fetch_add(stats.probes, std::memory_order_relaxed);
        m->statChunks.fetch_add(1, std::memory_order_relaxed);
    }

    PinnedView view(*vol, stamp);

    bool missing = false;
    if (view.IsBlocked(voxel, missing) || missing)
    {
        return false;
    }

    const DynamicObstacleSet* dyn = view.Dynamic();
    return !(dyn && !dyn->Empty() && dyn->Blocked(aPoint));
}

Status NavService::FindEscape(const std::string& aVolume, const EscapeQuery& aQuery, EscapeResult& aOut)
{
    aOut = EscapeResult{};

    auto vol = m->FindVolume(aVolume);
    if (!vol)
    {
        aOut.status = Status::NoVolume;
        return aOut.status;
    }
    if (!aQuery.IsValid() || !m->IsGameThread())
    {
        aOut.status = Status::BadInput;
        return aOut.status;
    }

    const auto started = Clock::now();
    const uint64_t stamp = m->tick.load(std::memory_order_relaxed);
    const float voxelSize = vol->Config().voxelSize;

    PinnedView view(*vol, stamp);
    IWorldProbe& probe = m->Probe();

    const IVec3 origin = vol->WorldToVoxel(aQuery.from);
    const auto radiusVoxels = std::max(1, static_cast<int32_t>(std::ceil(aQuery.searchRadius / voxelSize)));
    const auto minDistVoxels = static_cast<float>(aQuery.minDistance / voxelSize);
    const uint8_t needed = std::max<uint8_t>(vol->BlockThreshold(),
                                             Volume::ClearanceThreshold(aQuery.minClearance, voxelSize));

    int32_t probeBudget = aQuery.maxProbesPerQuery;
    bool outOfProbes = false;

    // Pull in every chunk the search box touches up front. Bounded by searchRadius, and cheaper
    // than discovering residency shell by shell - the shells revisit the same chunks repeatedly.
    const IVec3 lo = Volume::VoxelToChunk({origin.x - radiusVoxels, origin.y - radiusVoxels, origin.z - radiusVoxels});
    const IVec3 hi = Volume::VoxelToChunk({origin.x + radiusVoxels, origin.y + radiusVoxels, origin.z + radiusVoxels});

    for (int32_t cz = lo.z; cz <= hi.z && !outOfProbes; ++cz)
    {
        for (int32_t cy = lo.y; cy <= hi.y && !outOfProbes; ++cy)
        {
            for (int32_t cx = lo.x; cx <= hi.x && !outOfProbes; ++cx)
            {
                const uint64_t key = PackChunkKey({cx, cy, cz});
                if (vol->Store().Contains(key))
                {
                    continue;
                }
                if (m->TryBaked(*vol, key))
                {
                    ++aOut.chunksSampled;
                    continue;
                }
                if (probeBudget <= 0)
                {
                    outOfProbes = true;
                    break;
                }

                SampleStats stats;
                auto chunk = SampleChunk(*vol, IVec3{cx, cy, cz}, probe, stats);
                vol->Store().Publish(key, std::move(chunk), stamp);

                probeBudget -= stats.probes;
                aOut.probes += stats.probes;
                ++aOut.chunksSampled;

                vol->AddProbes(stats.probes);
                vol->AddChunksSampled(1);
                m->statProbes.fetch_add(stats.probes, std::memory_order_relaxed);
                m->statChunks.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }

    // Expanding shells, nearest-first within each - same shape as SnapToFree, but selecting on
    // room rather than on mere traversability, and with no goal to bias toward.
    const DynamicObstacleSet* dyn = view.Dynamic();

    for (int32_t r = 0; r <= radiusVoxels; ++r)
    {
        int32_t bestDistSq = 0;
        bool found = false;

        for (int32_t dz = -r; dz <= r; ++dz)
        {
            for (int32_t dy = -r; dy <= r; ++dy)
            {
                for (int32_t dx = -r; dx <= r; ++dx)
                {
                    if (r > 0 && std::abs(dx) != r && std::abs(dy) != r && std::abs(dz) != r)
                    {
                        continue; // shell only; the interior was covered by smaller radii
                    }
                    const int32_t distSq = dx * dx + dy * dy + dz * dz;
                    if (static_cast<float>(distSq) < minDistVoxels * minDistVoxels)
                    {
                        continue;
                    }
                    if (found && distSq >= bestDistSq)
                    {
                        continue;
                    }

                    const IVec3 v{origin.x + dx, origin.y + dy, origin.z + dz};
                    if (!vol->InBounds(v))
                    {
                        continue;
                    }

                    bool missing = false;
                    const uint8_t c = view.Clearance(v, missing);
                    if (missing || c < needed)
                    {
                        continue;
                    }

                    const Vec3 world = vol->VoxelCenter(v);
                    if (dyn && !dyn->Empty() && dyn->Blocked(world))
                    {
                        continue;
                    }

                    aOut.position = world;
                    aOut.clearance = vol->ClearanceMetres(c);
                    bestDistSq = distSq;
                    found = true;
                }
            }
        }

        if (found)
        {
            aOut.distance = (aOut.position - aQuery.from).Length();
            aOut.status = Status::Success;
            aOut.elapsedMs = MillisSince(started, Clock::now());
            return aOut.status;
        }
    }

    // Nothing qualifying. Distinguish "looked everywhere and there is none" from "could not afford
    // to finish looking" - the second is worth retrying, the first is not.
    aOut.status = outOfProbes ? Status::BudgetExhausted : Status::Unreachable;
    aOut.elapsedMs = MillisSince(started, Clock::now());
    return aOut.status;
}

void NavService::SetDynamicObstacles(const std::string& aVolume, const std::vector<AABB>& aBoxes)
{
    auto vol = m->FindVolume(aVolume);
    if (!vol)
    {
        return;
    }

    if (aBoxes.empty())
    {
        vol->SetDynamicObstacles(nullptr);
        return;
    }

    // Cell sized to a chunk so the hash's granularity matches everything else here; obstacles far
    // larger than that fall back to a short linear list inside the set.
    const float cell = vol->Config().voxelSize * static_cast<float>(kChunkDim);
    vol->SetDynamicObstacles(
        std::make_shared<const DynamicObstacleSet>(aBoxes, vol->Config().agentRadius, cell));
}

size_t NavService::DynamicObstacleCount(const std::string& aVolume) const
{
    auto vol = m->FindVolume(aVolume);
    if (!vol)
    {
        return 0;
    }
    const auto set = vol->DynamicObstacles();
    return set ? set->Count() : 0;
}

void NavService::InvalidateBox(const std::string& aVolume, const AABB& aBox)
{
    auto vol = m->FindVolume(aVolume);
    if (!vol)
    {
        return;
    }

    // Anything the box touches is now the probe's business, not the bake's - otherwise the next
    // fetch would reload the shipped geometry and quietly undo the door that just opened. Marked
    // by chunk coordinate rather than by what happens to be resident, so a chunk evicted before
    // it is next needed still comes back from the probe.
    const AABB grown{aBox.min - Vec3{1.0f, 1.0f, 1.0f}, aBox.max + Vec3{1.0f, 1.0f, 1.0f}};
    if (vol->Baked())
    {
        const IVec3 lo = Volume::VoxelToChunk(vol->WorldToVoxel(grown.min));
        const IVec3 hi = Volume::VoxelToChunk(vol->WorldToVoxel(grown.max));
        for (int32_t z = lo.z; z <= hi.z; ++z)
        {
            for (int32_t y = lo.y; y <= hi.y; ++y)
            {
                for (int32_t x = lo.x; x <= hi.x; ++x)
                {
                    vol->MarkProbeOnly(PackChunkKey(IVec3{x, y, z}));
                }
            }
        }
    }

    vol->Store().EraseIf([&](uint64_t key) { return vol->ChunkBounds(UnpackChunkKey(key)).Overlaps(aBox); });
}

void NavService::InvalidateAll(const std::string& aVolume)
{
    if (auto vol = m->FindVolume(aVolume))
    {
        vol->Store().Clear();
    }
}

void NavService::InvalidateAllVolumes()
{
    std::shared_lock lock(m->volumeMutex);
    for (auto& [name, vol] : m->volumes)
    {
        vol->Store().Clear();
    }
}

void NavService::Pump(int32_t aBudgetMicros)
{
    if (!m->running.load(std::memory_order_acquire))
    {
        return;
    }

    m->tick.fetch_add(1, std::memory_order_relaxed);

    const int32_t budget = aBudgetMicros < 0 ? m->config.defaultPumpBudgetMicros : aBudgetMicros;

    m->DrainSamples(budget);
    m->WakeParked();
    m->RunCallbacks();
    m->MaintainVolumes();
    m->ReapJobs();
}

Stats NavService::GetStats() const
{
    Stats s;
    s.totalQueries = m->statQueries.load(std::memory_order_relaxed);
    s.succeeded = m->statSucceeded.load(std::memory_order_relaxed);
    s.partial = m->statPartial.load(std::memory_order_relaxed);
    s.failed = m->statFailed.load(std::memory_order_relaxed);
    s.chunksFromBake = m->statBaked.load(std::memory_order_relaxed);
    s.totalProbes = m->statProbes.load(std::memory_order_relaxed);
    s.totalChunksSampled = m->statChunks.load(std::memory_order_relaxed);

    {
        std::shared_lock lock(m->volumeMutex);
        for (const auto& [name, vol] : m->volumes)
        {
            s.residentChunks += static_cast<int32_t>(vol->Store().Size());
            s.residentBytes += static_cast<int64_t>(vol->Store().ByteSize());
        }
    }

    {
        std::lock_guard lock(m->jobMutex);
        for (const auto& [id, job] : m->jobs)
        {
            switch (job->status.load(std::memory_order_acquire))
            {
            case Status::Pending: ++s.jobsPending; break;
            case Status::WaitingForWorld: ++s.jobsParked; break;
            case Status::Running: ++s.jobsRunning; break;
            default: break;
            }
        }
    }

    {
        std::lock_guard lock(m->sampleMutex);
        s.pendingChunkRequests = static_cast<int32_t>(m->sampleQueue.size());
    }

    return s;
}

void NavService::ResetStats()
{
    m->statQueries.store(0, std::memory_order_relaxed);
    m->statSucceeded.store(0, std::memory_order_relaxed);
    m->statPartial.store(0, std::memory_order_relaxed);
    m->statFailed.store(0, std::memory_order_relaxed);
    m->statProbes.store(0, std::memory_order_relaxed);
    m->statChunks.store(0, std::memory_order_relaxed);
}
} // namespace vnav
