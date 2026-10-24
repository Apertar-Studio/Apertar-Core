#include "ProresEncoder.hpp"
#include "ProresColor.hpp"
#include "ProresRepair.hpp"
#include "../audio/RecordingClock.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <cctype>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include <condition_variable>
#include <deque>
#include <mutex>

#include <chrono>
#include <pthread.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

extern "C" {
#include <libavutil/display.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}

namespace apertar {
namespace {

const char *avErr(int err)
{
    static thread_local char buf[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(err, buf, sizeof buf);
    return buf;
}

/* The same clock the zero-copy borrow stamps use (CameraController's
 * `borrowedAt`), so the legs measured here subtract cleanly against it. */
int64_t nowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
}

void bumpPeak(std::atomic<uint64_t> &peak, uint64_t v)
{
    uint64_t cur = peak.load(std::memory_order_relaxed);
    while (v > cur && !peak.compare_exchange_weak(cur, v, std::memory_order_relaxed))
        ;
}

/* ---- NAME THE FRAME THREADS ----------------------------------------------
 *
 * libavcodec creates its frame threads itself, so they inherit the process
 * comm and every one of them shows up as "apertar-core". That is not cosmetic:
 * the per-clip CPU table filters encode threads on the "apertar-enc" prefix, so
 * the ~270% of a core doing the actual ProRes encoding -- and HOLDING EVERY
 * BORROWED CAMERA BUFFER -- appeared in no `enc_*` number and could not be
 * told apart from the main thread in a /proc sample (ZEROCOPY-HOLD.md §5.3).
 *
 * This hook is the one place in this process that provably runs ON a frame
 * thread: av_buffer_create's free callback fires when the encoder drops its
 * last reference, which is after the thread that encoded the frame is done
 * with it. So each frame thread renames itself the first time it finishes a
 * frame.
 *
 * TWO GUARDS, both load-bearing. The rename applies only to a thread still
 * carrying the inherited process name -- so the encode worker, already named
 * apertar-encN, keeps its name and `enc_cpu_pct` keeps its meaning -- and never
 * to the main thread, whose tid equals the pid and which is legitimately
 * called apertar-core. The new prefix is deliberately NOT "apertar-enc": that would
 * silently fold four more threads into a figure every previous take's
 * enc_run_delay_ms was computed without. */
void nameFrameThreadOnce()
{
    if (static_cast<pid_t>(::syscall(SYS_gettid)) == ::getpid())
        return;
    char cur[32] = {0};
    if (::pthread_getname_np(::pthread_self(), cur, sizeof(cur)) != 0)
        return;
    if (std::strcmp(cur, "apertar-core") != 0)
        return;
    static std::atomic<unsigned> counter{0};
    char name[16];
    std::snprintf(name, sizeof(name), "apertar-fth%u",
                  counter.fetch_add(1, std::memory_order_relaxed) % 100u);
    ::pthread_setname_np(::pthread_self(), name);
}

int avProfileFor(ProresEncoder::Profile p)
{
    switch (p) {
    case ProresEncoder::Profile::Proxy:    return AV_PROFILE_PRORES_PROXY;
    case ProresEncoder::Profile::Lt:       return AV_PROFILE_PRORES_LT;
    case ProresEncoder::Profile::Standard: return AV_PROFILE_PRORES_STANDARD;
    case ProresEncoder::Profile::Hq:       return AV_PROFILE_PRORES_HQ;
    }
    return AV_PROFILE_PRORES_LT;
}

}  // namespace

struct Borrow {
    void (*release)(void *) = nullptr;
    void *token = nullptr;
};

struct ProresEncoder::Impl {
    /* av_buffer_create's free hook: fires once, when the encoder's last
     * reference to the frame goes away. */
    static void releaseBorrow(void *opaque, uint8_t *)
    {
        /* Cheap after the first call on each thread: a getname plus a strcmp.
         * See nameFrameThreadOnce() -- this is the only callback in the
         * process that is guaranteed to run on a libavcodec frame thread. */
        nameFrameThreadOnce();
        Borrow *b = static_cast<Borrow *>(opaque);
        if (b) {
            if (b->release) b->release(b->token);
            delete b;
        }
    }

    Config cfg;
    AVFormatContext *fmt = nullptr;
    AVStream *stream = nullptr;
    AVCodecContext *codec = nullptr;
    /* A RING of frames, not one.
     *
     * With FF_THREAD_FRAME the encoder keeps a reference to every frame still
     * in its pipeline, so a single reused AVFrame is never writable when the
     * next frame arrives: av_frame_make_writable then allocates a fresh 33 MB
     * buffer and COPIES into it, once per frame, purely to satisfy the
     * refcount. Holding threads+2 frames and cycling through them means the
     * one we pick up has already been released by the encoder, make_writable
     * is a no-op, and the copy disappears. */
    std::vector<AVFrame *> ring;
    size_t ringNext = 0;
    AVPacket *pkt = nullptr;
    prores_color::TransferLut lut{};
    std::unique_ptr<prores_color::CscPool> csc;
    double logK = 0.0;          /* fused mode: passed to the encoder */
    bool useLut = false;
    bool open = false;
    bool headerWritten = false;
    RecordingClock recordingClock;

    /* ---- BOUNDING WHAT A POWER CUT COSTS -------------------------------
     *
     * avio buffers writes, so at the instant the lights go out the last few
     * megabytes of picture are in userspace and gone. Flushing on a cadence
     * pushes them to the card, which turns "lose whatever was buffered" into
     * "lose at most the last flush interval".
     *
     * ONE SECOND OF FRAMES, not every frame: avio_flush() is a write(2), and
     * at 30 fps and ~3 MB a frame the buffer turns over constantly anyway --
     * flushing per frame would add a syscall per frame for no more safety than
     * this. It does NOT fsync: that would stall the encode thread on the card,
     * and the kernel's own writeback is what actually gets the data down. The
     * sidecar, which is the small thing that must survive, IS fsync'd. */
    unsigned flushEvery = 30;
    unsigned sinceFlush = 0;
    void maybeFlush()
    {
        if (!fmt || !fmt->pb) return;
        if (++sinceFlush < flushEvery) return;
        sinceFlush = 0;
        avio_flush(fmt->pb);
    }

    /* ---- WHO OWNS THE AVIOContext ---------------------------------------
     *
     * These two touch `fmt->pb` -- avio_flush() writes through it and
     * avio_tell() reads its position -- so they belong to WHICHEVER THREAD IS
     * DOING THE WRITING, and never to both. In the synchronous arm that is the
     * encode worker, as it always was. In the writer arm the AVIOContext
     * belongs exclusively to the muxer thread between avformat_write_header()
     * and the join in closeClip(), so the worker must not call them at all:
     * an avio_flush() racing an av_interleaved_write_frame() on the same
     * context is a corrupt file, and it would not look like a race -- it would
     * look like a bad card.
     *
     * workerPostFrame() is therefore the worker's hook and no-ops in the
     * writer arm; flushAndHygiene() is what the owning thread calls. */
    void flushAndHygiene()
    {
        maybeFlush();
        maybeWriteback();
    }
    void workerPostFrame()
    {
        if (writerArm)
            return;
        flushAndHygiene();
    }

    /* ---- WRITEBACK HYGIENE -- AN ARM, NOT YET A FIX ----------------------
     *
     * APERTAR_PRORES_WRITEBACK=<MiB> turns this on. DEFAULT OFF: without it the
     * behaviour is byte-for-byte what shipped before.
     *
     * THE HYPOTHESIS IT EXISTS TO TEST. avio's DIRECT flag removed the
     * userspace staging copy but the note beside it says plainly what it did
     * NOT do: "It does NOT set O_DIRECT on the fd -- libavformat's file
     * protocol never does -- so the page-cache writeback backlog is unchanged
     * by this." So a ProRes take pours 83-191 MB/s of dirty pages into the
     * page cache and nothing pushes them out but the kernel's own thresholds.
     *
     * MEASURED, 2026-09-12, over 905 complete ProRes takes in the core logs,
     * at 3840x2160 and 60 fps sensor rate -- identical frame count and take
     * length, only the output byte rate differing:
     *
     *     profile  MB/s   takes  stalled  median t of 1st stall  MB written
     *     HQ       191.4      9      9      2.37 s               458.7
     *     422      123.5    165     25      2.98 s               376.8
     *     LT        83.3    161     19      5.17 s               431.3
     *
     * The TIME to the first queue-full stall scales as 1/byte-rate; the BYTES
     * WRITTEN before it is constant at 377-459 MB across a 2.3x range of rate.
     * ~400 MB is what vm.dirty_background_ratio (10% here) comes to against
     * the ~4 GB of dirtyable memory this board has during a take. The ProRes
     * write runs on the encode worker, inside av_interleaved_write_frame
     * inside drain(), and balance_dirty_pages() throttles the writing task in
     * D state -- a total-output stall of exactly the observed shape.
     *
     * It also supersedes the earlier "ext4 commit=5 at ~5.5 s" reading: across
     * 78 first-stall events the time runs from 1.4 s to 940 s and only 12 land
     * anywhere near 5.5 s. It is not a once-per-take timer.
     *
     * WHAT THIS DOES. The standard streaming-write recipe, and the same pair
     * QrawDiskWriter keeps for its buffered fallback: start asynchronous
     * writeback on each completed window as soon as it is whole, and drop the
     * window BEFORE it -- which by then has had a full window's time to reach
     * the card -- out of the page cache. Dirty never accumulates, so the
     * threshold is never reached and the throttle never engages.
     *
     * WHY A SECOND fd. libavformat's file protocol owns the real one and
     * exposes it nowhere public. Both calls act on the inode's page cache, not
     * on the file description, so any fd on the same file will do.
     *
     * WHY NOT JUST O_DIRECT. ProRes packets are not block-aligned and the mov
     * muxer seeks backwards to patch the mdat size at close, so O_DIRECT would
     * need an aligned bounce buffer and a re-open for the trailer. This pair
     * has the same effect on the dirty backlog with none of that, and it is
     * what QRAW already does on its buffered path.
     *
     * RISK. sync_file_range with WAIT_AFTER can block, which is why it is
     * applied only to the window ONE BEHIND the one just started -- that one
     * has had `window` bytes of writing time to complete. If the card really
     * is too slow the wait is real, and it lands on the encode worker, which
     * is exactly where the current stall already lands. Watch
     * prores_flush_peak_ms: this work is inside that stamp. */
    int wbFd = -1;
    int64_t wbWindow = 0;          /* bytes; 0 = disarmed */
    int64_t wbSyncedTo = 0;        /* everything below this has had WRITE started */
    int64_t wbDroppedTo = 0;       /* everything below this has been DONTNEEDed */
    uint64_t wbSyncCalls = 0;
    uint64_t wbDropCalls = 0;

    void maybeWriteback()
    {
        if (wbFd < 0 || wbWindow <= 0 || !fmt || !fmt->pb)
            return;
        const int64_t pos = avio_tell(fmt->pb);
        if (pos < 0 || pos - wbSyncedTo < wbWindow)
            return;
#if defined(__linux__)
        /* 1. Start writeback on everything newly complete. Asynchronous: this
         *    call only queues the pages, it does not wait for the card. */
        const int64_t syncFrom = wbSyncedTo;
        const int64_t syncLen = pos - wbSyncedTo;
        ::sync_file_range(wbFd, syncFrom, syncLen, SYNC_FILE_RANGE_WRITE);
        ++wbSyncCalls;
        wbSyncedTo = pos;

        /* 2. Drop the window BEFORE the one just started. DONTNEED is a no-op
         *    on dirty pages, so the WAIT_AFTER is what makes it actually free
         *    anything -- and by now those pages have had a whole window's
         *    worth of writing time, so it should find them already clean. */
        if (syncFrom > wbDroppedTo) {
            const int64_t dropFrom = wbDroppedTo;
            const int64_t dropLen = syncFrom - wbDroppedTo;
            ::sync_file_range(wbFd, dropFrom, dropLen,
                              SYNC_FILE_RANGE_WAIT_BEFORE | SYNC_FILE_RANGE_WRITE
                                  | SYNC_FILE_RANGE_WAIT_AFTER);
            ::posix_fadvise(wbFd, dropFrom, dropLen, POSIX_FADV_DONTNEED);
            ++wbDropCalls;
            wbDroppedTo = syncFrom;
        }
#endif
    }

    /* ---- THE MUXER WRITER THREAD ----------------------------------------
     *
     * THE DEFECT THIS EXISTS TO REMOVE, stated as the numbers that found it.
     *
     * At 4K72 RAW10 ProRes LT the pipeline reaches a 72.005 fps mean and two
     * takes in four are clean. The other two lose frames, and every lost frame
     * is downstream of ONE rare event: a writeback stall inside
     * av_interleaved_write_frame(). From the zc-hold instrument, the three
     * holds that are almost pure write with an empty queue behind them:
     *
     *     frame=7963  total=533.3ms  queue=2.3ms   WRITE=530.5ms
     *     frame=7609  total=413.8ms  queue=0.1ms   WRITE=413.2ms
     *     frame=2544  total=279.9ms  queue=0.1ms   WRITE=278.9ms
     *
     * Four such events in ~35,000 frames -- rare, and enormous. The encode
     * queue holds 23 frames, which at 13.889 ms/frame is 319 ms of slack, so
     * the overflow and the drop count are the same number:
     *
     *     530.5 ms stall -> 211 ms overflow -> 15.2 frames  (take r1 dropped 15)
     *     413.2 ms stall ->  94 ms overflow ->  6.8 frames  (take r4 dropped 7)
     *
     * and the takes with no stall dropped nothing. That is the whole failure.
     *
     * THE STALL IS NOT THE THING TO FIX, AND THAT IS THE POINT. It is the page
     * cache and the card, it is bounded system-wide already by
     * /etc/sysctl.d/60-apertar-writeback.conf, and four events in 35,000 frames is
     * not a rate anything here can drive to zero. What is fixable is that the
     * stall lands ON THE ENCODE WORKER: because the write is called from
     * drain(), from writeFrame(), on the one thread that also pops the encode
     * queue, a stalled write stops frame drain and the queue overflows BEHIND
     * something that has nothing to do with encoding.
     *
     * So the write moves to a thread of its own. The worker hands the packet
     * over and goes straight back to draining; a stall now costs the writer's
     * time and nothing else, and the encode queue keeps emptying through it.
     * The queue depth below is the budget for how long a stall may last before
     * the worker feels it at all.
     *
     * -- ORDERING. A FIFO consumed by exactly ONE thread. ProRes is intra-only
     *    so the pictures do not depend on each other, but the mov muxer's
     *    sample table is built in call order and interleaving is stateful, so
     *    packets must reach av_interleaved_write_frame() in the order drain()
     *    produced them. One producer (the single ProRes encode worker -- see
     *    the note beside plannedWorkerCount() in CameraController.cpp), one
     *    consumer, a std::deque, and push order IS write order.
     *
     * -- OWNERSHIP. The worker's `pkt` is a single reusable AVPacket reused
     *    for the whole clip, so it cannot itself be queued. Each queued packet
     *    is a fresh AVPacket struct that takes the reference by
     *    av_packet_move_ref(), which leaves the worker's `pkt` blank and ready
     *    for the next avcodec_receive_packet() and copies NO PIXEL DATA -- it
     *    moves the AVBufferRef. From that moment the queue owns the packet
     *    outright: av_interleaved_write_frame() consumes the reference and
     *    blanks the struct, and the writer frees the struct afterwards. There
     *    is exactly one owner at every instant and no path frees twice.
     *
     *    (This file has already been bitten once by getting an ownership
     *    lifetime wrong across a thread boundary -- the stamp-after-writeFrame
     *    use-after-free that ProresRelease::refs exists to close. The rule
     *    applied here is the same one: nothing reads or frees an object after
     *    handing it to another thread.)
     *
     * -- THE QUEUE HOLDS NO ZERO-COPY BORROW. VERIFIED, not assumed, because
     *    the borrow pool is covered EXACTLY with zero margin -- "queue 23 +
     *    1 workers + 4 libavcodec frame threads = 28 frames max, against 28
     *    borrow slots" -- so one extra holder would push every surplus frame
     *    into the 16.59 MB staging memcpy and make things worse, not better.
     *
     *    A queued packet cannot hold one. Two facts, both read out of the
     *    libavcodec this build links (apertar-y216/ffmpeg-apertar):
     *      1. proresenc_anatoliy.c:1573 takes its output buffer from
     *         ff_get_encode_buffer(), a fresh allocation with no relationship
     *         to the input frame. The packet never points into the ISP buffer.
     *      2. encode.c, ff_encode_encode_cb(): `av_frame_unref(frame)` runs at
     *         the END of the callback, and frame_thread_encoder.c only sets
     *         task->finished AFTER that call returns. So the frame thread has
     *         already dropped its reference -- and therefore fired
     *         releaseBorrow() -- before the packet is even visible to
     *         avcodec_receive_packet(). By the time drain() has a packet to
     *         queue, that frame's borrow is gone.
     *    The borrow accounting line in CameraController.cpp is unchanged and
     *    stays true. What the writer queue costs is ordinary heap, bounded
     *    below, and nothing from the camera's buffer pool.
     *
     * -- THE BOUND. Unbounded would turn a pathological stall into an OOM, so
     *    the queue is capped on BOTH packets and bytes and the worker blocks
     *    when it is full. Blocking is the correct full-queue behaviour and not
     *    a compromise: it degenerates exactly to today's synchronous arm, one
     *    frame at a time, instead of losing picture. Dropping a packet is
     *    never an option -- it is a hole in the sample table.
     *
     *    48 packets and 96 MiB. At 4K72 LT (~1.4 MB/frame, 83.3 MB/s measured
     *    over 161 takes) 48 packets is 667 ms of absorption on top of the 319
     *    ms the encode queue already has -- ~986 ms against the worst stall
     *    ever observed here, 533 ms, and it never reaches the byte cap. At 4K
     *    HQ (~3.2 MB/frame) the byte cap binds first at ~30 packets, ~500 ms,
     *    which is the right trade: HQ is the profile that stalls most often
     *    and is also the one whose queue would cost the most RAM. Peak cost is
     *    96 MiB of anonymous memory against 3.2 GB free, and it is independent
     *    of vm.dirty_bytes (128 MB), which bounds page cache, not this.
     *
     * -- ERRORS STILL SURFACE. A failed write used to be the return value of
     *    writeFrame(), which CameraController counts into writeFailures_ and
     *    logs. Asynchronously there is no return value to fail, so the writer
     *    latches the FIRST error and every subsequent enqueue reports it: the
     *    very next writeFrame() returns false with that message, through the
     *    same call site, into the same counter. The delay is one frame. After
     *    a failure the writer keeps draining and freeing its queue without
     *    writing, so the worker can never block for ever behind a dead card.
     *
     * -- CLOSE DRAINS BEFORE THE MOOV. closeClip() flushes the encoder, which
     *    enqueues the tail, then stops and JOINS the writer, and only then
     *    calls av_write_trailer(). Writing the trailer while packets were
     *    still queued would index samples that are not in the file yet -- a
     *    truncated moov, which is an unplayable clip rather than a short one.
     *    The join is what makes that impossible.
     *
     * APERTAR_PRORES_WRITER_THREAD=0 restores the synchronous arm byte for byte,
     * for an A/B. Default is ON: this is the fix. */
    bool writerArm = false;
    std::thread writerThread;
    std::mutex writerMutex;
    std::condition_variable writerWork;   /* writer waits: work, or stop */
    std::condition_variable writerRoom;   /* worker waits: a free slot */
    std::deque<AVPacket *> writerQueue;   /* guarded by writerMutex */
    size_t writerBytes = 0;               /* bytes currently queued */
    size_t writerMaxPackets = 48;
    size_t writerMaxBytes = (size_t)96 << 20;
    bool writerStop = false;              /* guarded by writerMutex */
    bool writerFailed = false;            /* guarded by writerMutex */
    std::string writerError;              /* guarded by writerMutex */
    size_t writerQueuePeakPkts = 0;       /* guarded by writerMutex */
    uint64_t writerWrites = 0;            /* writer thread only */
    std::atomic<uint64_t> writerSlowWrites{0};
    std::atomic<uint64_t> writeBlockPeakNs{0};
    /* One log line per write over this, the same 250 ms the zc-hold event
     * threshold uses, so the two logs line up frame for frame. These are the
     * events the whole fix exists to absorb; they must stay visible, because
     * "the drops stopped" and "the stalls stopped" are completely different
     * outcomes and only this line tells them apart. */
    int64_t writerSlowNs = 250'000'000;

    void writerLoop()
    {
        ::pthread_setname_np(::pthread_self(), "apertar-mux");
        for (;;) {
            AVPacket *out = nullptr;
            bool failed = false;
            {
                std::unique_lock<std::mutex> lock(writerMutex);
                writerWork.wait(lock, [this] {
                    return writerStop || !writerQueue.empty();
                });
                /* STOP DOES NOT MEAN STOP NOW. The queue is drained first and
                 * the thread exits only when it is empty, which is what makes
                 * the join in closeClip() a guarantee that every packet is in
                 * the file before the trailer is written. */
                if (writerQueue.empty())
                    return;
                out = writerQueue.front();
                writerQueue.pop_front();
                const size_t sz = out->size > 0 ? (size_t)out->size : 0u;
                writerBytes = writerBytes > sz ? writerBytes - sz : 0u;
                failed = writerFailed;
            }
            /* Notified with the lock released: the worker wakes to find the
             * slot already free rather than waking to block again. */
            writerRoom.notify_one();

            if (!failed) {
                const int64_t pts = out->pts;
                const int64_t w0 = nowNs();
                const int w = av_interleaved_write_frame(fmt, out);
                const uint64_t dt = (uint64_t)(nowNs() - w0);
                bumpPeak(writePeakNs, dt);
                ++writerWrites;
                if (w < 0) {
                    std::lock_guard<std::mutex> lock(writerMutex);
                    if (!writerFailed) {
                        writerFailed = true;
                        writerError = std::string("ProRes write_frame: ") + avErr(w);
                    }
                    /* Wake a worker that is blocked on a full queue: from here
                     * on the queue only drains, it never writes. */
                    writerRoom.notify_all();
                } else {
                    /* The flush and the writeback hygiene belong to whoever
                     * owns the AVIOContext, and in this arm that is this
                     * thread. Timed into the same flushPeakNs the synchronous
                     * arm feeds, so prores_flush_peak_ms keeps its meaning.
                     *
                     * NOTE FOR THE WRITEBACK ARM (APERTAR_PRORES_WRITEBACK, still
                     * default 0): the reason that arm cost 1.2 fps is that its
                     * sync_file_range(WAIT_AFTER) blocked the ENCODE WORKER.
                     * Here it blocks this thread instead, which is exactly the
                     * thread that is allowed to block. It is worth re-testing
                     * on top of this change; it is not enabled by it. */
                    const int64_t f0 = nowNs();
                    flushAndHygiene();
                    bumpPeak(flushPeakNs, (uint64_t)(nowNs() - f0));
                }
                if ((int64_t)dt >= writerSlowNs) {
                    const uint64_t n = writerSlowWrites.fetch_add(
                            1, std::memory_order_relaxed) + 1;
                    /* Rate-limited the same way and at the same points as the
                     * zc-hold event line, so the two logs stay readable
                     * together. `n` is always the TRUE count, so a suppressed
                     * line never costs a number -- and a card that has gone
                     * slow for every frame cannot drown the journal. At the
                     * production 250 ms threshold this never engages: four
                     * events in ~35,000 frames. */
                    const bool suppress = (n > 200 && (n % 16) != 0);
                    size_t depth = 0;
                    if (!suppress) {
                        {
                            std::lock_guard<std::mutex> lock(writerMutex);
                            depth = writerQueue.size();
                        }
                        std::cerr << "ApertarCore ProRes writer: slow write pts=" << pts
                                  << " write=" << (double(dt) * 1.0e-6)
                                  << "ms queued_after=" << depth << "/" << writerMaxPackets
                                  << " n=" << n
                                  << (n > 200 ? " (RATE-LIMITED to 1-in-16 above 200"
                                                " printed; n is the TRUE count)"
                                              : "")
                                  << " (ABSORBED -- the encode worker did not wait"
                                     " unless write_block_peak_ms is also non-zero)\n";
                    }
                }
            }
            /* Consumed or not, the struct is ours to free. On the success path
             * av_interleaved_write_frame() has already taken the reference and
             * left `out` blank, so this is a free of an empty packet; on the
             * failure path it unrefs the data we never wrote. Either way this
             * is the single point where a queued packet dies. */
            av_packet_free(&out);
        }
    }

    /* Hand one packet to the writer. Returns false only when the writer has
     * already failed, in which case *error carries ITS error -- the delayed
     * surfacing described above. Blocks while the queue is full. */
    bool enqueueForWriter(AVPacket *out, std::string *error)
    {
        const size_t sz = out->size > 0 ? (size_t)out->size : 0u;
        {
            std::unique_lock<std::mutex> lock(writerMutex);
            /* `writerQueue.empty()` in the predicate is a forward-progress
             * guard, not an optimisation: a single packet larger than the byte
             * cap would otherwise never satisfy it and the worker would wait
             * for ever. An empty queue always accepts, whatever the size. */
            writerRoom.wait(lock, [&] {
                return writerFailed
                    || writerQueue.empty()
                    || (writerQueue.size() < writerMaxPackets
                        && writerBytes + sz <= writerMaxBytes);
            });
            if (writerFailed) {
                if (error) *error = writerError;
                lock.unlock();
                av_packet_free(&out);
                return false;
            }
            writerQueue.push_back(out);
            writerBytes += sz;
            if (writerQueue.size() > writerQueuePeakPkts)
                writerQueuePeakPkts = writerQueue.size();
        }
        writerWork.notify_one();
        return true;
    }

    /* Drain the queue, join the thread, and report the first error it saw.
     * Idempotent, so the destructor can call it after closeClip() already has.
     * MUST NOT be called with writerMutex held. */
    bool stopWriter(std::string *error)
    {
        if (writerThread.joinable()) {
            {
                std::lock_guard<std::mutex> lock(writerMutex);
                writerStop = true;
            }
            writerWork.notify_all();
            writerThread.join();
        }
        std::lock_guard<std::mutex> lock(writerMutex);
        /* Nothing should be left -- the loop exits only on an empty queue --
         * but a writer that never started (an openClip that failed after the
         * header) can leave packets here, and leaking them would leak whole
         * frames of picture. */
        for (AVPacket *p : writerQueue)
            av_packet_free(&p);
        writerQueue.clear();
        writerBytes = 0;
        if (writerFailed) {
            if (error) *error = writerError;
            return false;
        }
        return true;
    }

    int64_t pts = 0;
    int64_t firstTimestampUs = -1;
    /* THE PIPELINE IS WARMED WITH REAL FRAMES THAT NEVER REACH THE FILE.
     *
     * Every ProRes take was losing sensor frames in its opening ~200 ms, at the
     * interval where libavcodec's frame threads all engage at once -- four of
     * them, each streaming a 47.5 MB RGB48 frame out of DRAM. It survives the
     * CPU clock being pinned at the top and it is identical in Proxy, which
     * reads the same input bytes for a fraction of the output, so it is a
     * bandwidth step rather than a shortage of cycles.
     *
     * A step cannot be removed by making it cheaper, only by moving it. So the
     * first `warmUpFrames` frames of a take ARE encoded -- they have to be, or
     * the threads never spin up -- and their packets are dropped on the floor
     * instead of being muxed. Whatever the front end loses while that happens
     * is lost BEFORE the clip's first frame, which is the point: the recorded
     * clip is gapless from frame 1.
     *
     * Packets are matched by pts because frame threading returns them late: at
     * the moment the last warm-up frame is submitted, up to `threads` of its
     * predecessors are still inside the encoder. Everything with pts below the
     * mark is discarded, and everything above it is rebased so the movie still
     * starts at zero. */
    int64_t discardBeforePts = 0;
    std::atomic<uint64_t> warmUpDropped{0};
    std::atomic<uint64_t> frames{0};
    std::atomic<uint64_t> bytes{0};

    /* ---- THE WORKER'S LEG SPLIT (see ProresEncoder::FrameTiming) ---------
     * `lastWriteNs` is scratch, written by drain() and read by the timing
     * helper immediately after: one worker thread runs both, back to back,
     * with nothing in between, so it needs no synchronisation. The peaks are
     * atomics only because the accessors are read from the worker's exit path
     * while nothing else is running -- they cost nothing on the frame path. */
    uint64_t lastWriteNs = 0;
    std::atomic<uint64_t> sendPeakNs{0};
    std::atomic<uint64_t> drainPeakNs{0};
    std::atomic<uint64_t> writePeakNs{0};
    std::atomic<uint64_t> flushPeakNs{0};

    void noteFrameTiming(int64_t send0, int64_t send1, int64_t drain1, int64_t flush1,
                         ProresEncoder::FrameTiming *out)
    {
        const uint64_t sendNs  = send1 > send0 ? uint64_t(send1 - send0) : 0u;
        const uint64_t drainNs = drain1 > send1 ? uint64_t(drain1 - send1) : 0u;
        const uint64_t flushNs = flush1 > drain1 ? uint64_t(flush1 - drain1) : 0u;
        bumpPeak(sendPeakNs, sendNs);
        bumpPeak(drainPeakNs, drainNs);
        bumpPeak(writePeakNs, lastWriteNs);
        bumpPeak(flushPeakNs, flushNs);
        if (out) {
            out->sendDoneAtNs = send1;
            out->doneAtNs = flush1;
            out->sendNs = sendNs;
            out->drainNs = drainNs;
            /* In the writer arm this is the handoff, not the write. See the
             * long note on FrameTiming in the header -- the quantity a hold is
             * made of is the worker's cost, not the disk's. */
            out->writeNs = lastWriteNs;
            out->flushNs = flushNs;
            out->asyncWrite = writerArm;
        }
    }

    bool drain(std::string *error)
    {
        for (;;) {
            const int ret = avcodec_receive_packet(codec, pkt);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                return true;
            if (ret < 0) {
                if (error) *error = std::string("ProRes receive_packet: ") + avErr(ret);
                return false;
            }
            /* WARM-UP PACKETS GO NOWHERE. Not counted, not written, not
             * charged to the clip's byte total -- they exist only to have made
             * the encoder run. See discardBeforePts. */
            if (pkt->pts != AV_NOPTS_VALUE && pkt->pts < discardBeforePts) {
                warmUpDropped.fetch_add(1, std::memory_order_relaxed);
                av_packet_unref(pkt);
                continue;
            }
            // prores_aw writes unspecified colour IDs in its frame header.
            // Set the three metadata bytes, not the compressed image payload.
            if (pkt->size < 28 || std::memcmp(pkt->data + 4, "icpf", 4) != 0 ||
                av_packet_make_writable(pkt) < 0) {
                if (error) *error = "ProRes: cannot set frame colour metadata";
                av_packet_unref(pkt);
                return false;
            }
            pkt->data[22] = cfg.colorProfile == ProresColorProfile::Standard ? AVCOL_PRI_BT709 : AVCOL_PRI_UNSPECIFIED;
            pkt->data[23] = cfg.colorProfile == ProresColorProfile::P3D60Linear ? AVCOL_TRC_LINEAR : AVCOL_TRC_UNSPECIFIED;
            pkt->data[24] = AVCOL_SPC_BT709;
            /* REBASED, so the movie starts at zero however long the warm-up
             * was. Both stamps move together; dts is not always set by every
             * encoder, so it is checked on its own. */
            if (discardBeforePts > 0) {
                if (pkt->pts != AV_NOPTS_VALUE) pkt->pts -= discardBeforePts;
                if (pkt->dts != AV_NOPTS_VALUE) pkt->dts -= discardBeforePts;
            }
            frames.fetch_add(1, std::memory_order_relaxed);
            bytes.fetch_add((uint64_t)pkt->size, std::memory_order_relaxed);
            pkt->stream_index = 0;
            /* ONE FRAME, EXPLICITLY -- this is why every clip was a frame short.
             *
             * prores_aw leaves pkt->duration at 0, and the mov muxer then
             * derives each sample's duration from the NEXT sample's dts. That
             * works for every sample but the last one, which has no next: it
             * was written with duration 0, so stts came out
             * [(240, 1000), (1, 0)] and a 241-frame clip presented as 240.
             * The picture was never missing -- stsz had all 241 entries and
             * they summed to the whole mdat -- but a zero-duration last sample
             * is invisible to ffprobe, to Resolve and to every player.
             *
             * pts advances by exactly 1 per frame in codec->time_base, so one
             * frame IS a duration of 1 there, and rescale below carries it into
             * the stream's units along with pts/dts. */
            if (pkt->duration <= 0)
                pkt->duration = 1;
            av_packet_rescale_ts(pkt, codec->time_base, stream->time_base);
            /* TIMED SEPARATELY FROM THE REST OF drain(). This call is the
             * whole write path -- interleave, mov muxer, avio, write(2) -- and
             * it is the only thing on the ProRes encode worker that can block
             * on the filesystem. Splitting it out is what lets a 400 ms worker
             * leg be attributed to the card rather than to libavcodec. */
            if (writerArm) {
                /* THE HANDOFF ARM. av_packet_move_ref() transfers the
                 * AVBufferRef and blanks `pkt`, so no pixel data is copied and
                 * the worker's reusable packet is immediately ready for the
                 * next avcodec_receive_packet(). Everything the muxer needs --
                 * pts, dts, duration, stream_index, side data -- moves with
                 * it, which is why the rescale above happens BEFORE this and
                 * not on the writer thread.
                 *
                 * Timed the same way the write was, because in this arm the
                 * handoff is what can block the worker, and the whole point of
                 * the stamp is to measure what blocks the worker. */
                const int64_t w0 = nowNs();
                AVPacket *out = av_packet_alloc();
                if (!out) {
                    if (error) *error = "ProRes writer: packet alloc";
                    av_packet_unref(pkt);
                    return false;
                }
                av_packet_move_ref(out, pkt);
                /* enqueueForWriter() owns `out` from here on EVERY path,
                 * including its own failure path -- so there is nothing left
                 * to free here and nothing that could be freed twice. */
                const bool queued = enqueueForWriter(out, error);
                const uint64_t blockNs = uint64_t(nowNs() - w0);
                lastWriteNs += blockNs;
                bumpPeak(writeBlockPeakNs, blockNs);
                if (!queued)
                    return false;
                continue;
            }
            const int64_t w0 = nowNs();
            const int w = av_interleaved_write_frame(fmt, pkt);
            lastWriteNs += uint64_t(nowNs() - w0);
            if (w < 0) {
                if (error) *error = std::string("ProRes write_frame: ") + avErr(w);
                return false;
            }
        }
    }
};

/* ---- BENCH TUNING OVERRIDE ------------------------------------------------
 *
 * The ProRes knobs below are read from the environment, which is fixed when
 * apertar-core starts -- so sweeping one costs a service restart per arm, and a
 * restart between arms is exactly what makes an A/B untrustworthy on this
 * board (a fresh session loses no frames at the start of a take; an aged one
 * loses 2-6, and the clock ladder rests differently).
 *
 * So a knob may also come from an optional file, re-read at CLIP OPEN, which
 * lets every arm of a sweep run in one process against one warm state:
 *
 *     /home/pi/.apertar-tuning        NAME=VALUE per line, # comments allowed
 *
 * The file is absent in production and then this costs one failed open() per
 * clip and changes nothing. The environment still wins, so anything already
 * pinned by a systemd drop-in keeps its meaning. */
static std::string tuningOverride(const char *name)
{
    if (const char *env = std::getenv(name))
        return std::string(env);
    std::ifstream in("/home/pi/.apertar-tuning");
    if (!in)
        return std::string();
    std::string line;
    const std::string key(name);
    while (std::getline(in, line)) {
        const std::size_t hash = line.find('#');
        if (hash != std::string::npos)
            line.resize(hash);
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos)
            continue;
        std::string k = line.substr(0, eq);
        std::string v = line.substr(eq + 1);
        auto trim = [](std::string &t) {
            while (!t.empty() && std::isspace(static_cast<unsigned char>(t.front()))) t.erase(t.begin());
            while (!t.empty() && std::isspace(static_cast<unsigned char>(t.back()))) t.pop_back();
        };
        trim(k); trim(v);
        if (k == key)
            return v;
    }
    return std::string();
}

std::string ProresEncoder::tuningValue(const char *name)
{
    return tuningOverride(name);
}

unsigned ProresEncoder::inFlightFrames()
{
    /* Config::threads, which is the frame-thread count handed to libavcodec.
     * Kept as a function rather than exposing the default so the two cannot
     * drift; Config's own default is the same 4.
     *
     * APERTAR_PRORES_THREADS moves both together. The caller that sizes the borrow
     * pool has no Config to consult -- it runs before the clip is opened -- so
     * the knob is read here as well rather than passed, and the two readings
     * cannot disagree because both clamp to 1..4 the same way. */
    const std::string v = tuningOverride("APERTAR_PRORES_THREADS");
    if (!v.empty()) {
        const int n = std::atoi(v.c_str());
        if (n >= 1 && n <= 4)
            return static_cast<unsigned>(n);
    }
    return 4u;
}

ProresEncoder::ProresEncoder(const Config &config) : impl_(new Impl)
{
    impl_->cfg = config;
    switch (config.transfer) {
    case ProresTransfer::Display:
        impl_->useLut = false;
        break;
    case ProresTransfer::GpLog2:
        prores_color::buildGpLog2Lut(&impl_->lut);
        impl_->useLut = true;
        impl_->logK = 599.0;
        break;
    case ProresTransfer::BmdFilmGen5:
        prores_color::buildBmdFilmGen5Lut(&impl_->lut);
        impl_->useLut = true;
        impl_->logK = 137.36;
        break;
    }

    /* THE FUSED ENCODER CANNOT EXPRESS A TOE, so a transfer that has one must
     * not use it.
     *
     * The fused path hands libavcodec a single `log_k` and the patched encoder
     * applies y = ln(1+kx)/ln(1+k) inside its slice fetch. That is the whole
     * family it implements -- there is nowhere to put a join, a slope or a
     * pedestal. Both log transfers now carry a linear toe (GP-Log2 gained one
     * on 2026-09-07), so both must take the separate LUT pass.
     *
     * The comment beside BmdFilmGen5 has claimed this fallback since the toe
     * was added there, but nothing implemented it: fused BMD silently encoded
     * the toe-less curve while the code said otherwise. It is real now, and it
     * matters more than it did, because the toe is what carries the sub-black
     * pedestal -- encoding without it clips the shadow noise the pedestal
     * exists to keep.
     *
     * The production path is unaffected: APERTAR_PRORES_TRANSFER=display applies
     * no curve here at all, because the ISP has already applied it, so display
     * keeps the fused fast path. */
    if (config.transfer == ProresTransfer::GpLog2
        || config.transfer == ProresTransfer::BmdFilmGen5) {
        if (impl_->cfg.fusedRgb) {
            std::cerr << "ApertarCore ProRes: "
                      << (config.transfer == ProresTransfer::GpLog2
                              ? "GP-Log2" : "Blackmagic Film Gen5")
                      << " has a linear toe, which the fused RGB48 encoder path"
                         " cannot express -- using the separate colour pass\n";
            impl_->cfg.fusedRgb = false;
        }
    }
}

ProresEncoder::~ProresEncoder()
{
    std::string ignored;
    if (impl_->open) closeClip(&ignored);
    /* BELT AND BRACES, and not optional. closeClip() joins the writer on every
     * path it takes, but a std::thread that is still joinable when its owner
     * is destroyed calls std::terminate -- so the one case closeClip() cannot
     * have covered (an encoder destroyed without ever being closed) must be
     * covered here. stopWriter() is idempotent. */
    impl_->stopWriter(nullptr);
    /* closeClip() closes it on the normal path; this covers a clip that never
     * opened or that failed before the trailer. */
    if (impl_->wbFd >= 0) { ::close(impl_->wbFd); impl_->wbFd = -1; }
    for (auto &f : impl_->ring) if (f) av_frame_free(&f);
    if (impl_->pkt) av_packet_free(&impl_->pkt);
    if (impl_->codec) avcodec_free_context(&impl_->codec);
    if (impl_->fmt) {
        if (impl_->fmt->pb) avio_closep(&impl_->fmt->pb);
        avformat_free_context(impl_->fmt);
    }
}

bool ProresEncoder::openClip(const std::string &path, std::string *error)
{
    Impl &s = *impl_;

    /* prores_aw, NOT prores_ks. Measured on the reference UHD frame: ks is
     * 6.8x slower at LT *and* 1.6 dB worse, because it estimates every
     * quantiser in the profile range per slice and then runs a trellis. */
    const AVCodec *codec = avcodec_find_encoder_by_name("prores_aw");
    if (!codec) {
        if (error) *error = "ProRes encoder prores_aw not available in this libavcodec";
        return false;
    }

    int ret = avformat_alloc_output_context2(&s.fmt, nullptr, "mov", path.c_str());
    if (ret < 0) { if (error) *error = std::string("ProRes mov muxer: ") + avErr(ret); return false; }

    s.stream = avformat_new_stream(s.fmt, nullptr);
    if (!s.stream) { if (error) *error = "ProRes: could not create stream"; return false; }

    /* A rational timebase, so 23.976/29.97 are exact rather than rounded. */
    const auto standardRate = recordingFrameRate(s.cfg.fps);
    const int num = standardRate.first;
    const int den = standardRate.second;
    s.stream->time_base = AVRational{den, num};

    s.codec = avcodec_alloc_context3(codec);
    if (!s.codec) { if (error) *error = "ProRes: codec allocation failed"; return false; }
    if (s.cfg.fusedRgb && !av_opt_find(s.codec->priv_data, "rgb_input", nullptr, 0, 0)) {
        s.cfg.fusedRgb = false;
        std::cerr << "ApertarCore ProRes: standard FFmpeg, using NEON RGB48 conversion\n";
    }
    s.codec->width = (int)s.cfg.width;
    s.codec->height = (int)s.cfg.height;
    if (s.cfg.ispYuv422p10)
        s.cfg.fusedRgb = false;   /* nothing to fuse: the ISP did the colour */
    s.codec->pix_fmt = s.cfg.fusedRgb ? AV_PIX_FMT_RGB48LE : AV_PIX_FMT_YUV422P10LE;
    s.codec->time_base = AVRational{den, num};
    s.codec->framerate = AVRational{num, den};
    s.stream->avg_frame_rate = s.codec->framerate;
    s.stream->r_frame_rate = s.codec->framerate;
    s.codec->profile = avProfileFor(s.cfg.profile);
    s.codec->thread_count = (int)s.cfg.threads;
    /* FF_THREAD_FRAME is not a preference. prores_aw implements only frame
     * threading; asking for slice threading is accepted and does nothing
     * (measured 89.5 ms/frame at 1 and at 4 slice threads). */
    s.codec->thread_type = FF_THREAD_FRAME;
    /* Set on the CODEC context and copied to codecpar by
     * avcodec_parameters_from_context() below; the mov muxer writes it as a
     * pasp atom. The stream's own copy is set as well because the muxer reads
     * st->sample_aspect_ratio when the two disagree. */
    if (s.cfg.pixelAspectNum > 0 && s.cfg.pixelAspectDen > 0) {
        s.codec->sample_aspect_ratio =
            AVRational{ s.cfg.pixelAspectNum, s.cfg.pixelAspectDen };
        s.stream->sample_aspect_ratio = s.codec->sample_aspect_ratio;
    }
    s.codec->color_primaries = s.cfg.colorProfile == ProresColorProfile::Standard
        ? AVCOL_PRI_BT709 : AVCOL_PRI_UNSPECIFIED;
    s.codec->colorspace = AVCOL_SPC_BT709;
    s.codec->color_range = AVCOL_RANGE_MPEG;
    /* Under a log curve the transfer is NOT Rec.709; tagging it as such would
     * make a player apply a display transform to already-log data. */
    /* NEVER CLAIM Rec.709 ON THE PASS-THROUGH PATH.
     *
     * This used to tag Display as AVCOL_TRC_BT709 on the reasoning that
     * "pass-through" means the ISP's ordinary display rendering. That stopped
     * being true when the ISP's own gamma became GP-Log2: the file then carries
     * log and says Rec.709, and a player that HONOURS the tag applies a
     * transform to it -- which looks exactly like someone left a LUT on.
     *
     * Unspecified is the honest answer, because from here the ISP's curve is
     * genuinely unknown: it is whatever the tuning file's gamma_curve says, and
     * that is chosen outside this process. Both log transfers already tag
     * unspecified for the same reason. */
    // Linear is explicitly known; vendor log curves and tuning rendering remain unspecified.
    s.codec->color_trc = s.cfg.colorProfile == ProresColorProfile::P3D60Linear ? AVCOL_TRC_LINEAR : AVCOL_TRC_UNSPECIFIED;
    if (s.fmt->oformat->flags & AVFMT_GLOBALHEADER)
        s.codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    /* Apple vendor tag: Resolve and FCP are happier with a clip that claims
     * Apple provenance, and it costs nothing. */
    av_opt_set(s.codec->priv_data, "vendor", "FFMP", 0);
    if (s.cfg.quickRc)
        av_opt_set(s.codec->priv_data, "quick_rc", "1", 0);
    /* qp_pred seeds each slice's qp from max(left, above) rather than from the
     * left neighbour alone. It ships in 0003-prores_aw-a76-... as one of the
     * seven optimisations and was then never switched on -- the AVOption
     * defaults to 0, and nothing here set it. A better seed means fewer slices
     * that come out over budget and have to be encoded a second time, which is
     * the same cost quick_rc attacks from the other end.
     *
     * It is marked "bitstream change" because the qp actually chosen differs;
     * the output is still conformant ProRes. APERTAR_PRORES_QP_PRED=0 disables. */
    if (s.cfg.qpPred)
        av_opt_set(s.codec->priv_data, "qp_pred", "1", 0);
    if (s.cfg.fusedRgb) {
        if (av_opt_set(s.codec->priv_data, "rgb_input", "1", 0) < 0) {
            if (error) *error = "ProRes: libavcodec lacks the fused RGB48 input "
                                "(apply 0002-prores_aw-fused-rgb48-input.patch)";
            return false;
        }
        /* The curve moves INTO the encoder in fused mode -- one LUT gather per
         * component inside the slice fetch, instead of a whole extra pass. */
        av_opt_set_double(s.codec->priv_data, "log_k", s.logK, 0);
    }

    if ((ret = avcodec_open2(s.codec, codec, nullptr)) < 0) {
        if (error) *error = std::string("ProRes avcodec_open2: ") + avErr(ret);
        return false;
    }
    if ((ret = avcodec_parameters_from_context(s.stream->codecpar, s.codec)) < 0) {
        if (error) *error = std::string("ProRes codecpar: ") + avErr(ret);
        return false;
    }
    s.stream->codecpar->color_primaries = s.cfg.colorProfile == ProresColorProfile::Standard ? AVCOL_PRI_BT709 : AVCOL_PRI_UNSPECIFIED;
    s.stream->codecpar->color_trc = s.cfg.colorProfile == ProresColorProfile::P3D60Linear ? AVCOL_TRC_LINEAR : AVCOL_TRC_UNSPECIFIED;
    s.stream->codecpar->color_space = AVCOL_SPC_BT709;
    s.stream->codecpar->color_range = AVCOL_RANGE_MPEG;
    /* AFTER avcodec_parameters_from_context, which overwrites codecpar
     * wholesale and would discard side data attached before it, and BEFORE
     * write_header, which is what serialises it into the moov. */
    if (s.cfg.rotationDegrees % 360 != 0) {
        AVPacketSideData *sd = av_packet_side_data_new(
            &s.stream->codecpar->coded_side_data,
            &s.stream->codecpar->nb_coded_side_data,
            AV_PKT_DATA_DISPLAYMATRIX, sizeof(int32_t) * 9, 0);
        if (sd) {
            /* NEGATED: av_display_rotation_set() takes the angle the DECODER
             * must apply anticlockwise, which is the inverse of the rotation
             * the picture needs. Getting this backwards is invisible at 180
             * and exactly wrong at 90. */
            av_display_rotation_set(reinterpret_cast<int32_t *>(sd->data),
                                    -(double)s.cfg.rotationDegrees);
        } else if (error) {
            *error = "ProRes: could not attach display matrix";
            return false;
        }
    }

    /* AVIO_FLAG_DIRECT -- retried 2026-09-12 with the ACTUAL cause fixed.
     *
     * WHY IT IS WORTH DOING. Without it avio stages every packet through its
     * own buffer, which ffio_fdopen() sizes to the file protocol's
     * max_packet_size (262144). A ~2.0 MB ProRes 422 frame is therefore
     * memcpy'd out of the packet and into that buffer, eight chunks at a time,
     * before a single byte reaches write(). At 60 fps that is ~124 MB/s read
     * plus ~124 MB/s written -- ~246 MB/s of DRAM traffic on a board whose
     * capture chain loses frames for every GB/s taken away from it. QRAW's own
     * writer has had the equivalent (real O_DIRECT) since day one; this was
     * the last staged writer in the process.
     *
     * THE 2026-09-11 FAILURE WAS NOT THE MOV MUXER'S BACKWARD SEEK.
     *
     * The previous note here blamed the seek av_write_trailer() does to patch
     * the mdat size. That diagnosis is wrong. Every clip came out 36 bytes
     * with no moov, and the takes appeared to "gain" 32 frames, because
     * NOTHING WAS EVER WRITTEN. libavformat/avio.c, ffurl_write2():
     *
     *     if (h->max_packet_size && size > h->max_packet_size)
     *         return AVERROR(EIO);
     *
     * and libavformat/file.c sets max_packet_size = 262144 on every writable
     * regular file. In buffered mode nothing can ever hand it more than that,
     * because the staging buffer IS that size. In direct mode avio_write()
     * hands it the whole 1.98 MB packet, so EVERY FRAME failed EIO. The seek
     * was never involved and works fine in direct mode.
     *
     * file.c takes a `pkt_size` option that replaces max_packet_size, so the
     * whole fix is one entry in the avio_open2() options dict. It is sized to
     * the frame: ProRes does not exceed ~1 B/px even at 4444 XQ, and the same
     * number also becomes the avio staging buffer, which direct mode barely
     * touches (small header/moov writes only).
     *
     * NOTE ON SCOPE: this removes the userspace staging copy. It does NOT set
     * O_DIRECT on the fd -- libavformat's file protocol never does -- so the
     * page-cache writeback backlog is unchanged by this. That is a separate
     * problem with a separate fix.
     *
     * If the option is not consumed (an older libavformat with no pkt_size),
     * direct mode would hit the EIO above, so we REFUSE it and fall back
     * rather than write another 36-byte clip.
     *
     * ARM SELECT: APERTAR_PRORES_ODIRECT=0 selects the old buffered arm.
     * Default is direct. */
    bool wantDirect = true;
    {
        const std::string v = tuningOverride("APERTAR_PRORES_ODIRECT");
        if (!v.empty())
            wantDirect = !(v == "0" || v == "no" || v == "off" || v == "false");
    }
    const size_t kBufferedPkt = 262144;
    /* 2 B/px plus a 4 MiB floor. The bound has to be a CEILING, not a typical
     * size: a packet over it is an EIO, which av_interleaved_write_frame()
     * does surface as a failed take (drain(), above) rather than silently, but
     * a failed take is still a lost take. Reference points at 3840x2160 422:
     * real ISP content 1.98 MB/frame (0.24 B/px); pure 16-bit noise, which is
     * the incompressible worst case, 5.41 MB/frame (0.65 B/px) -- measured
     * with prores-avio-validate. 2 B/px is above ProRes 4444 XQ's ceiling, so
     * nothing this encoder can emit reaches it. The same number becomes avio's
     * staging buffer (ffio_fdopen sizes it to max_packet_size), which direct
     * mode leaves almost entirely untouched -- one ~21 MB allocation per clip
     * at 4K, and nothing per frame. */
    size_t pktSize = (size_t)s.codec->width * (size_t)s.codec->height * 2u + (size_t)(4u << 20);
    if (pktSize < (size_t)(4u << 20))  pktSize = (size_t)(4u << 20);
    if (pktSize > (size_t)(64u << 20)) pktSize = (size_t)(64u << 20);

    bool gotDirect = false;
    std::string directWhyNot;
    if (wantDirect) {
        AVDictionary *opts = nullptr;
        av_dict_set_int(&opts, "pkt_size", (int64_t)pktSize, 0);
        ret = avio_open2(&s.fmt->pb, path.c_str(),
                         AVIO_FLAG_WRITE | AVIO_FLAG_DIRECT, nullptr, &opts);
        /* av_opt_set_dict() ERASES what it consumed, so anything left is an
         * option this libavformat's file protocol does not have. */
        const bool consumed = (av_dict_count(opts) == 0);
        av_dict_free(&opts);
        if (ret >= 0 && consumed) {
            gotDirect = true;
        } else {
            if (ret >= 0) {
                avio_closep(&s.fmt->pb);
                directWhyNot = "this libavformat's file protocol has no pkt_size option";
            } else {
                directWhyNot = avErr(ret);
            }
            std::cerr << "ApertarCore ProRes: direct avio refused ("
                      << directWhyNot << "); using buffered writes\n";
        }
    }
    if (!gotDirect) {
        if ((ret = avio_open(&s.fmt->pb, path.c_str(), AVIO_FLAG_WRITE)) < 0) {
            if (error) *error = std::string("ProRes open ") + path + ": " + avErr(ret);
            return false;
        }
    }
    /* WRITEBACK HYGIENE ARM. See maybeWriteback(). Opened here, after
     * avio_open2() has created the file, and closed in closeClip(). A window
     * below 4 MiB is rejected rather than clamped: the whole mechanism rests
     * on the dropped window having had a full window's writing time, and a
     * window smaller than a few frames does not give it any. */
    {
        const std::string v = tuningOverride("APERTAR_PRORES_WRITEBACK");
        const long mib = v.empty() ? 0 : std::atol(v.c_str());
        if (mib >= 4 && mib <= 512) {
            s.wbFd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
            if (s.wbFd < 0) {
                std::cerr << "ApertarCore ProRes: writeback hygiene wanted " << mib
                          << " MiB but the file could not be reopened ("
                          << std::strerror(errno) << "); disabled for this clip\n";
            } else {
                s.wbWindow = (int64_t)mib << 20;
            }
        } else if (!v.empty() && mib != 0) {
            std::cerr << "ApertarCore ProRes: APERTAR_PRORES_WRITEBACK=" << v
                      << " is outside 4..512 MiB; writeback hygiene stays OFF\n";
        }
    }

    /* ---- WRITER ARM SELECT ----------------------------------------------
     *
     * Default ON: this is the fix for the 4K72 drop cliff, not an experiment
     * (see the long note beside writerLoop()). APERTAR_PRORES_WRITER_THREAD=0
     * restores the synchronous write on the encode worker, which is the arm
     * every measurement before 2026-09-13 was taken on, so an A/B needs no
     * rebuild and -- because the knob is read through tuningOverride() -- no
     * service restart either.
     *
     * The depth knobs are here for the same reason: if a card is ever slow
     * enough that write_block_peak_ms stops being zero, the answer is a deeper
     * queue, and finding out how much deeper should not cost a rebuild. */
    {
        const std::string v = tuningOverride("APERTAR_PRORES_WRITER_THREAD");
        s.writerArm = v.empty()
            || !(v == "0" || v == "no" || v == "off" || v == "false");
        const std::string q = tuningOverride("APERTAR_PRORES_WRITER_QUEUE");
        if (!q.empty()) {
            const long n = std::atol(q.c_str());
            if (n >= 1 && n <= 512) s.writerMaxPackets = (size_t)n;
        }
        const std::string m = tuningOverride("APERTAR_PRORES_WRITER_QUEUE_MB");
        if (!m.empty()) {
            const long mb = std::atol(m.c_str());
            if (mb >= 8 && mb <= 1024) s.writerMaxBytes = (size_t)mb << 20;
        }
        /* Lowering this is how the slow-write log path gets exercised without
         * waiting for a real writeback stall -- 0 prints every write, which is
         * only ever useful for a few seconds. */
        const std::string sl = tuningOverride("APERTAR_PRORES_WRITER_SLOW_MS");
        if (!sl.empty()) {
            const long ms = std::atol(sl.c_str());
            if (ms >= 0 && ms <= 60000) s.writerSlowNs = (int64_t)ms * 1000000;
        }
    }

    /* One line per take, so an A/B sweep can prove which arm each clip ran. */
    std::cerr << "ApertarCore ProRes: disk arm="
              << (gotDirect ? "direct" : "buffered")
              << " APERTAR_PRORES_ODIRECT=" << (wantDirect ? 1 : 0)
              << " pkt_size=" << (gotDirect ? pktSize : kBufferedPkt)
              << " writeback="
              << (s.wbWindow > 0 ? std::to_string(s.wbWindow >> 20) + "MiB"
                                 : std::string("off"))
              << " write arm="
              << (s.writerArm ? "WRITER THREAD (av_interleaved_write_frame off"
                                " the encode worker)"
                              : "synchronous (write on the encode worker --"
                                " a writeback stall overflows the encode queue)")
              << (s.writerArm
                      ? " writer_queue=" + std::to_string(s.writerMaxPackets)
                        + "pkt/" + std::to_string(s.writerMaxBytes >> 20) + "MiB"
                      : std::string())
              << " clip=" << path << "\n";
    s.headerWritten = false;

    /* ---- THE RECOVERY SIDECAR -------------------------------------------
     *
     * A .mov is ONE FILE and its sample table -- the moov atom -- is written
     * by av_write_trailer() at the very END. Until that runs the file has
     * every frame of picture in its mdat and no index at all, so a power cut,
     * a kernel panic or a crash mid-take loses the WHOLE clip rather than the
     * last few frames. That is not hypothetical: three takes were lost to it
     * on 2026-09-06 before the record_stop SEGV was found, each one "moov atom
     * not found" over hundreds of megabytes of perfectly good frames.
     *
     * A frame sequence has no such failure mode -- every .gpr is closed as it
     * is written -- so this is the one thing ProRes gives up by being a movie,
     * and it is worth buying back.
     *
     * The frames themselves are recoverable WITHOUT this file: ProRes samples
     * are self-delimiting, [4-byte big-endian size]['icpf'], laid end to end
     * from the start of the mdat. What the sidecar adds is everything the
     * bitstream does NOT carry -- the frame rate, the profile fourcc, and
     * where the mdat payload begins -- so a repair does not have to guess.
     *
     * Written ONCE, at open, and fsync'd. It is ~200 bytes and it is the thing
     * that must survive the event that killed the take. */
    {
        const char *fourcc = "apcn";
        switch (s.cfg.profile) {
        case Profile::Proxy:    fourcc = "apco"; break;
        case Profile::Lt:       fourcc = "apcs"; break;
        case Profile::Standard: fourcc = "apcn"; break;
        case Profile::Hq:       fourcc = "apch"; break;
        }
        const std::string side = prores_repair::recoverySidecarPath(path);
        FILE *f = std::fopen(side.c_str(), "we");
        if (f) {
            std::fprintf(f,
                "{\n"
                "  \"clip\": \"%s\",\n"
                "  \"width\": %u,\n"
                "  \"height\": %u,\n"
                "  \"fps\": %.6f,\n"
                "  \"profile\": \"%s\",\n"
                "  \"pix_fmt\": \"yuv422p10le\",\n"
                "  \"color_profile\": \"%s\",\n"
                "  \"input_color_space\": \"%s\",\n"
                "  \"input_gamma\": \"%s\",\n"
                "  \"data_levels\": \"Video\",\n"
                "  \"yuv_matrix\": \"BT.709\",\n"
                "  \"curve_implementation\": \"ISP piecewise-linear approximation; not added dynamic range\",\n"
                "  \"logc4_warning\": \"ARRI recommends at least 12-bit; this ProRes is 10-bit\",\n"
                "  \"note\": \"Written at clip open. If the matching .mov has no moov atom it was interrupted; apertar-core repairs it on the next start, or run prores-repair by hand.\"\n"
                "}\n",
                path.c_str(), s.cfg.width, s.cfg.height,
                s.cfg.fps > 0.0 ? s.cfg.fps : 30.0, fourcc,
                toString(s.cfg.colorProfile), proresColorSpace(s.cfg.colorProfile), proresGamma(s.cfg.colorProfile));
            std::fflush(f);
            ::fsync(::fileno(f));
            std::fclose(f);
        } else {
            std::cerr << "ApertarCore ProRes: could not write the recovery sidecar "
                      << side << " (" << std::strerror(errno) << ") -- an "
                      "interrupted take will still be repairable, but the repair "
                      "will have to assume the frame rate\n";
        }
    }

    /* The ring and the CSC pool exist only for the separate-pass path; in
     * fused mode nothing is staged, so neither is allocated. */
    if (!s.cfg.fusedRgb)
        s.ring.resize((size_t)s.cfg.threads + 2, nullptr);
    for (auto &f : s.ring) {
        f = av_frame_alloc();
        if (!f) { if (error) *error = "ProRes: frame allocation failed"; return false; }
        f->format = s.codec->pix_fmt;
        f->width = s.codec->width;
        f->height = s.codec->height;
        if ((ret = av_frame_get_buffer(f, 64)) < 0) {
            if (error) *error = std::string("ProRes frame buffer: ") + avErr(ret);
            return false;
        }
    }
    s.ringNext = 0;
    /* The CSC pool is sized to the machine, not to the encoder's frame
     * threads: the two do not run at the same instant for a given frame (the
     * CSC finishes before send_frame is called on it), so they are not
     * competing for cores so much as taking turns. */
    /* CSC CONCURRENCY IS A PERMIT COUNT, NOT A THREAD COUNT.
     *
     * Straight from the QRAW wavelet-gate result: the colour conversion is the
     * bandwidth-bound phase here exactly as the wavelet is there, and running
     * it on every core does NOT go faster -- it steals DRAM from the encoder
     * threads, which are the ones actually on the critical path. QRAW measured
     * 4 encoders pulling 37.7 MiB/frame against 24.9 for 2, with IPC falling
     * 1.98 -> 1.66. Same shape, same fix: cap it.
     * APERTAR_PRORES_CSC_THREADS overrides for an A/B. */
    unsigned cscThreads = s.cfg.fusedRgb
        ? 1u
        : std::max(1u, std::min(2u, (unsigned)std::thread::hardware_concurrency()));
    if (const char *e = getenv("APERTAR_PRORES_CSC_THREADS")) {
        const int v = atoi(e);
        if (v >= 1 && v <= 8) cscThreads = (unsigned)v;
    }
    if (!s.cfg.fusedRgb && !s.cfg.ispYuv422p10)
        s.csc.reset(new prores_color::CscPool(cscThreads));

    s.pkt = av_packet_alloc();
    if (!s.pkt) { if (error) *error = "ProRes: packet allocation failed"; return false; }
    s.pts = 0;
    s.firstTimestampUs = -1;
    s.discardBeforePts = (int64_t)s.cfg.warmUpFrames;
    s.warmUpDropped.store(0, std::memory_order_relaxed);
    s.sinceFlush = 0;
    s.flushEvery = (unsigned)std::max(1.0, s.cfg.fps > 0.0 ? s.cfg.fps : 30.0);

    // The muxer header/writer starts with the first frame so its timecode
    // shares the actual capture origin rather than encoder initialization.
    s.open = true;
    return true;
}

bool ProresEncoder::writeFrame(const IspFrameView &view, std::string *error,
                               FrameTiming *timing)
{
    Impl &s = *impl_;
    /* The caller hands us the frame's storage. Every exit from this function
     * must account for it exactly once, including the early error returns --
     * otherwise a rejected frame leaks a camera buffer and the pipeline stalls
     * a pool slot at a time. Disarmed only where an AVBufferRef takes over. */
    struct ReleaseGuard {
        void (*fn)(void *);
        void *token;
        ~ReleaseGuard() { if (fn) fn(token); }
        void disarm() { fn = nullptr; }
    } guard{ view.release, view.token };

    if (!s.open) { if (error) *error = "ProRes: clip not open"; return false; }
    if (view.width != s.cfg.width || view.height != s.cfg.height) {
        if (error) *error = "ProRes: frame geometry does not match the clip";
        return false;
    }
    const size_t need = (size_t)view.stride * view.height;
    if (!view.data || view.size < need) {
        if (error) *error = "ProRes: short ISP frame";
        return false;
    }

    /* THE ROW MUST HOLD RGB48, NOT MERELY BE self-consistent.
     *
     * The check above only proves size == stride * height, which is true of
     * ANY format the ISP might hand back. If the recording stream is not
     * 6 bytes per pixel -- APERTAR_PRORES_ISP_FORMAT selects narrower ones for
     * bandwidth diagnostics -- ffmpeg still reads width*6 bytes per row from
     * a shorter row and walks off the end of the buffer. Rejecting the frame
     * costs a log line; the overread is a crash. */
    if (!s.cfg.ispYuv422p10 && view.stride < (size_t)view.width * 6) {
        if (error)
            *error = "ProRes: ISP row is " + std::to_string(view.stride)
                   + " bytes, RGB48 needs " + std::to_string((size_t)view.width * 6)
                   + " -- wrong APERTAR_PRORES_ISP_FORMAT for this encoder path";
        return false;
    }

    if (!s.headerWritten) {
        // The first accepted sensor frame, not encoder startup, defines timecode.
        const auto tc = s.cfg.timecode == "@capture"
            ? recordingTimecode(true, s.recordingClock.wallTime(view.timestampUs), s.cfg.fps) : s.cfg.timecode;
        if (!tc.empty()) av_dict_set(&s.stream->metadata, "timecode", tc.c_str(), 0);
        av_dict_set(&s.fmt->metadata, "apertar_color_profile", toString(s.cfg.colorProfile), 0);
        av_dict_set(&s.fmt->metadata, "apertar_input_color_space", proresColorSpace(s.cfg.colorProfile), 0);
        av_dict_set(&s.fmt->metadata, "apertar_input_gamma", proresGamma(s.cfg.colorProfile), 0);
        av_dict_set(&s.fmt->metadata, "apertar_data_levels", "Video", 0);
        AVDictionary *options = nullptr;
        av_dict_set(&options, "movflags", "+use_metadata_tags+write_colr", 0);
        const int result = avformat_write_header(s.fmt, &options);
        av_dict_free(&options);
        if (result < 0) { if (error) *error = std::string("ProRes write_header: ") + avErr(result); return false; }
        s.headerWritten = true;
        if (s.writerArm) {
            s.writerStop = false;s.writerFailed = false;s.writerError.clear();
            s.writerQueuePeakPkts = 0;s.writerWrites = 0;
            s.writerSlowWrites.store(0, std::memory_order_relaxed);
            s.writeBlockPeakNs.store(0, std::memory_order_relaxed);
            s.writerThread = std::thread(&Impl::writerLoop, &s);
        }
    }

    // Preserve capture gaps rather than speeding the video up against audio.
    int64_t framePts = s.pts;
    if (view.timestampUs > 0) {
        if (s.firstTimestampUs < 0) s.firstTimestampUs = view.timestampUs;
        framePts = std::max(framePts, av_rescale_q(view.timestampUs - s.firstTimestampUs,
            AVRational{1, 1000000}, s.codec->time_base));
    }
    s.pts = framePts + 1;

    if (s.cfg.ispYuv422p10) {
        /* Q216 is ONE dmabuf holding three contiguous planes, laid out by
         * pisp_be.c:354-362 from plane_factor {1, 0.5, 0.5}:
         *     Y  at 0                  stride  bytes/row  (= width * 2)
         *     Cb at ySize              stride/2
         *     Cr at ySize + ySize/2    stride/2
         * so the encoder can address all three without a second mapping. */
        const size_t ySize = (size_t)view.stride * view.height;
        const size_t cSize = ySize / 2;
        if (view.size < ySize + 2 * cSize) {
            if (error) *error = "ProRes: Q216 frame is " + std::to_string(view.size)
                              + " bytes, need " + std::to_string(ySize + 2 * cSize);
            return false;
        }
        AVFrame *rf = av_frame_alloc();
        if (!rf) { if (error) *error = "ProRes: frame alloc"; return false; }
        rf->format = AV_PIX_FMT_YUV422P10LE;
        rf->width = (int)view.width;
        rf->height = (int)view.height;
        rf->data[0] = const_cast<uint8_t *>(view.data);
        rf->data[1] = const_cast<uint8_t *>(view.data) + ySize;
        rf->data[2] = const_cast<uint8_t *>(view.data) + ySize + cSize;
        rf->linesize[0] = (int)view.stride;
        rf->linesize[1] = (int)(view.stride / 2);
        rf->linesize[2] = (int)(view.stride / 2);
        /* ONE refcounted buffer shared by all three planes: the release must
         * fire exactly once, when the encoder drops its last reference. Three
         * independent av_buffer_create()s would need two no-op free hooks and
         * are easy to get wrong. */
        Borrow *b = new Borrow{ view.release, view.token };
        rf->buf[0] = av_buffer_create(const_cast<uint8_t *>(view.data), view.size,
                                      &Impl::releaseBorrow, b, AV_BUFFER_FLAG_READONLY);
        if (!rf->buf[0]) {
            delete b;
            av_frame_free(&rf);
            if (error) *error = "ProRes: buffer wrap";
            return false;
        }
        rf->buf[1] = av_buffer_ref(rf->buf[0]);
        rf->buf[2] = av_buffer_ref(rf->buf[0]);
        guard.disarm();
        rf->pts = framePts;
        const int64_t tSend0 = nowNs();
        const int sret = avcodec_send_frame(s.codec, rf);
        const int64_t tSend1 = nowNs();
        av_frame_free(&rf);
        if (sret < 0) { if (error) *error = std::string("ProRes send_frame: ") + avErr(sret); return false; }
        s.lastWriteNs = 0;
        const bool ok = s.drain(error);
        const int64_t tDrain1 = nowNs();
        s.workerPostFrame();
        s.noteFrameTiming(tSend0, tSend1, tDrain1, nowNs(), timing);
        return ok;
    }

    if (s.cfg.fusedRgb) {
        /* Wrap the ISP buffer; do not copy it. The buffer's own release runs
         * when the encoder drops its last reference (see IspFrameView). */
        AVFrame *rf = av_frame_alloc();
        if (!rf) { if (error) *error = "ProRes: frame alloc"; return false; }
        rf->format = AV_PIX_FMT_RGB48LE;
        rf->width = (int)view.width;
        rf->height = (int)view.height;
        rf->data[0] = const_cast<uint8_t *>(view.data);
        rf->linesize[0] = (int)view.stride;
        Borrow *b = new Borrow{ view.release, view.token };
        rf->buf[0] = av_buffer_create(const_cast<uint8_t *>(view.data), view.size,
                                      &Impl::releaseBorrow, b, AV_BUFFER_FLAG_READONLY);
        if (!rf->buf[0]) {
            delete b;
            av_frame_free(&rf);
            if (error) *error = "ProRes: buffer wrap";
            return false;
        }
        guard.disarm();                /* buf[0] now carries the release */
        rf->pts = framePts;
        const int64_t tSend0 = nowNs();
        const int sret = avcodec_send_frame(s.codec, rf);
        const int64_t tSend1 = nowNs();
        av_frame_free(&rf);            /* encoder holds its own reference */
        if (sret < 0) { if (error) *error = std::string("ProRes send_frame: ") + avErr(sret); return false; }
        /* Counted in drain() now, on the packet actually written, so a warm-up
         * frame cannot inflate the clip's length. */
        s.lastWriteNs = 0;
        const bool ok = s.drain(error);
        const int64_t tDrain1 = nowNs();
        s.workerPostFrame();
        s.noteFrameTiming(tSend0, tSend1, tDrain1, nowNs(), timing);
        return ok;
    }

    AVFrame *f = s.ring[s.ringNext];
    s.ringNext = (s.ringNext + 1) % s.ring.size();

    int ret = av_frame_make_writable(f);
    if (ret < 0) { if (error) *error = std::string("ProRes make_writable: ") + avErr(ret); return false; }

    s.csc->convert(
        view.data, view.stride,
        reinterpret_cast<uint16_t *>(f->data[0]), (size_t)f->linesize[0] / 2,
        reinterpret_cast<uint16_t *>(f->data[1]),
        reinterpret_cast<uint16_t *>(f->data[2]), (size_t)f->linesize[1] / 2,
        view.width, view.height, s.useLut ? &s.lut : nullptr);

    f->pts = framePts;
    const int64_t tSend0 = nowNs();
    ret = avcodec_send_frame(s.codec, f);
    const int64_t tSend1 = nowNs();
    if (ret < 0) {
        if (error) *error = std::string("ProRes send_frame: ") + avErr(ret);
        return false;
    }
    /* Counted in drain(), same as the fused path. */
    s.lastWriteNs = 0;
    const bool ok = s.drain(error);
    const int64_t tDrain1 = nowNs();
    s.workerPostFrame();
    s.noteFrameTiming(tSend0, tSend1, tDrain1, nowNs(), timing);
    return ok;
}

bool ProresEncoder::closeClip(std::string *error)
{
    Impl &s = *impl_;
    if (!s.open) return true;
    s.open = false;
    if (!s.headerWritten) {
        avio_closep(&s.fmt->pb);
        if (s.wbFd >= 0) { ::close(s.wbFd);s.wbFd = -1; }
        return true;
    }

    /* Frame threading holds `threads` frames in flight; a clip closed without
     * this loses the tail. */
    avcodec_send_frame(s.codec, nullptr);
    const bool drained = s.drain(error);

    /* ---- THE MOOV MUST NOT OVERTAKE THE PICTURE ------------------------
     *
     * In the writer arm the drain above did not write the tail, it QUEUED it.
     * av_write_trailer() builds the sample table from what the muxer has been
     * given, and it writes through the same AVIOContext the writer thread is
     * using -- so calling it now would both race the writer and index samples
     * that are not in the mdat yet. That is a truncated moov, and a truncated
     * moov is an UNPLAYABLE clip, not a short one: the failure mode this
     * codebase already lost three takes to on 2026-09-06.
     *
     * stopWriter() drains the queue, joins the thread, and only then is the
     * AVIOContext this thread's again. Unconditional, and before the `drained`
     * check, because a thread that is never joined is a crash in the
     * destructor however the encode went. No-op in the synchronous arm. */
    std::string writerErr;
    const bool writerOk = s.stopWriter(&writerErr);

    if (!drained) return false;
    /* A LATCHED WRITER ERROR IS TREATED EXACTLY AS A SYNCHRONOUS ONE WAS.
     *
     * In the synchronous arm a failed write inside this final drain returned
     * false right here and the trailer was never written. Same outcome, same
     * place, so the two arms fail identically and the recovery sidecar --
     * which exists precisely for a .mov with no moov -- remains the designed
     * way back. Writing a trailer over a short mdat anyway would be a
     * behaviour change in the highest-risk path in this file and it is
     * deliberately not made here; it is noted in WRITER-THREAD.md as a
     * separate question. */
    if (!writerOk) {
        if (error) *error = writerErr;
        return false;
    }

    const int ret = av_write_trailer(s.fmt);
    if (ret < 0) { if (error) *error = std::string("ProRes write_trailer: ") + avErr(ret); return false; }
    avio_closep(&s.fmt->pb);
    if (s.wbFd >= 0) {
        /* Deliberately NOT a final DONTNEED over the whole file. The trailer
         * has just been written and the moov is the one part of a .mov that
         * must be readable immediately; dropping it here would only force the
         * clip browser to read it back off the card. */
        std::cerr << "ApertarCore ProRes writeback hygiene: " << s.wbSyncCalls
                  << " windows started, " << s.wbDropCalls << " dropped, "
                  << (s.wbSyncedTo >> 20) << " MiB covered\n";
        ::close(s.wbFd);
        s.wbFd = -1;
    }
    return true;
}

/* MEASURED ceilings on this board (Pi 5, 4x A76 @ 2.8 GHz, 4 frame threads,
 * quick_rc, real ISP content), encode only -- the CSC and the rest of the
 * pipeline are NOT in these numbers. PRORES-FINDINGS.md has the method. */
double ProresEncoder::sustainableFps(Profile profile, uint32_t width, uint32_t height)
{
    const uint64_t px = (uint64_t)width * height;
    const uint64_t uhd = 3840ull * 2160ull;
    double uhdFps = 0.0;
    switch (profile) {
    case Profile::Proxy:    uhdFps = 57.4; break;
    case Profile::Lt:       uhdFps = 35.8; break;
    case Profile::Standard: uhdFps = 24.8; break;
    case Profile::Hq:       uhdFps = 14.9; break;
    }
    if (px == 0) return 0.0;
    /* Cost is very close to linear in pixel count across the four sizes
     * measured (1920x1080, 2560x1440, 3840x1600, 3840x2160). */
    return uhdFps * (double)uhd / (double)px;
}

const char *ProresEncoder::profileName(Profile profile)
{
    switch (profile) {
    case Profile::Proxy:    return "proxy";
    case Profile::Lt:       return "lt";
    case Profile::Standard: return "422";
    case Profile::Hq:       return "hq";
    }
    return "lt";
}

uint64_t ProresEncoder::framesEncoded() const { return impl_->frames.load(std::memory_order_relaxed); }
uint64_t ProresEncoder::warmUpFramesDiscarded() const
{
    return impl_->warmUpDropped.load(std::memory_order_relaxed);
}
uint64_t ProresEncoder::bytesWritten() const { return impl_->bytes.load(std::memory_order_relaxed); }
uint64_t ProresEncoder::sendPeakNanos() const { return impl_->sendPeakNs.load(std::memory_order_relaxed); }
uint64_t ProresEncoder::drainPeakNanos() const { return impl_->drainPeakNs.load(std::memory_order_relaxed); }
uint64_t ProresEncoder::writePeakNanos() const { return impl_->writePeakNs.load(std::memory_order_relaxed); }
uint64_t ProresEncoder::flushPeakNanos() const { return impl_->flushPeakNs.load(std::memory_order_relaxed); }
uint64_t ProresEncoder::writeBlockPeakNanos() const
{ return impl_->writeBlockPeakNs.load(std::memory_order_relaxed); }
uint64_t ProresEncoder::writerSlowWrites() const
{ return impl_->writerSlowWrites.load(std::memory_order_relaxed); }
uint64_t ProresEncoder::writerQueuePeak() const
{
    /* Under the lock: the writer thread is normally already joined when this
     * is read (the recording queue reads it after closeClip()), but the live
     * telemetry may not be, and a torn size_t read would be a lie in a log. */
    std::lock_guard<std::mutex> lock(impl_->writerMutex);
    return (uint64_t)impl_->writerQueuePeakPkts;
}

}  // namespace apertar
