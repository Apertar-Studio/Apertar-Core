#pragma once

/* ProRes 422 family encoder for QRAW.
 *
 * WHY THIS IS SHAPED DIFFERENTLY FROM QrawEncoder.
 *
 * QrawEncoder writes ONE FILE PER FRAME and eats MSB-justified Bayer straight
 * off the CFE -- that is the whole reason the zero-copy borrow path exists
 * (QrawEncoder.cpp feedInPlace_). ProRes can do neither:
 *
 *   1. It is a YUV 4:2:2 codec. There is no Bayer entry point. The frame has
 *      to be demosaiced and colour-converted before the encoder sees it.
 *   2. It lives in a QuickTime container. A clip is ONE file with a sample
 *      table, not a directory of frames, so the per-frame `FrameJob::path`
 *      model does not apply -- the muxer holds an open fd for the clip.
 *
 * WHERE THE PIXELS COME FROM, AND WHY NOT A SOFTWARE DEMOSAIC.
 * Measured on this board (scratchpad/prores/devbench.c, 3840x2160, one A76 at
 * 2.8 GHz): black-subtract+gains 5.6 ms, bilinear demosaic 26.3 ms, RGB->YUV
 * 42.0 ms = 73.9 ms/frame. That is MORE than the entire ProRes LT encode
 * (28.7 ms/frame wall on four threads), so demosaicing on the encode workers
 * roughly triples the cost of the pipeline. The PiSP back end already does
 * demosaic + colour matrix in hardware for the preview stream, so ProRes takes
 * ISP output and only does the colour conversion the ISP cannot.
 *
 * WHY RGB48 AND NOT THE ISP'S OWN 4:2:2.
 * The PiSP back end offers '422P' / 'YM16' planar YUV 4:2:2 -- but every YUV
 * output format it has is EIGHT BIT (enumerated from /dev/video20 on this
 * machine; the only >8-bit non-Bayer outputs are RGB6/BGR6, 48-bit RGB).
 * Feeding 8-bit YUV into a 10-bit codec is bad enough for a display-gamma
 * picture and unacceptable under a LOG curve, where the whole point is to
 * spend code values on shadow detail: log into 8 bits bands visibly. So the
 * back end is asked for RGB48 and this file does RGB48 -> yuv422p10le in NEON
 * (15.0 ms/frame at 4K, against 29.2 scalar -- ProresColor.cpp).
 *
 * THE COST MODEL THAT FOLLOWS FROM THAT, at 3840x2160 ProRes LT:
 *     encode  28.7 ms/frame wall x 4 threads = 114.7 ms CPU
 *     CSC                                    =  15.0 ms CPU
 *     total 129.7 ms CPU / 4 cores           =  32.4 ms  ->  30.8 fps
 * 4K60 ProRes is therefore NOT reachable on this board's CPUs; 4K30 LT is,
 * with headroom. Numbers and method: PRORES-FINDINGS.md.
 */

#include <cstdint>
#include <memory>
#include <string>
#include <apertar/core/ProresColorProfile.hpp>

namespace apertar {

/* Planar or packed ISP output handed to the encoder. Deliberately NOT
 * RawFrameView: that struct is single-plane and Bayer-typed (Types.hpp:674)
 * and has no vocabulary for a three-component picture. */
struct IspFrameView {
    const uint8_t *data = nullptr;   /* RGB48 little-endian, R,G,B interleaved */
    size_t size = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;             /* bytes per row, >= width*6 */
    int64_t timestampUs = 0;

    /* ZERO-COPY BORROW, same contract as the QRAW recording queue's.
     *
     * In fused mode the encoder reads THIS buffer directly -- it is never
     * staged -- and frame threading means it keeps reading it for up to
     * `threads` more frames. So the ISP buffer cannot be requeued when
     * writeFrame() returns, the way it could if we had copied.
     *
     * If `release` is set it is called with `token` exactly once, when the
     * encoder drops its last reference to this frame. That is the moment the
     * dmabuf may be handed back. Leave it null and the caller is promising the
     * buffer outlives the clip. */
    void (*release)(void *token) = nullptr;
    void *token = nullptr;
};

/* Legacy encoder-side curves. Production color profiles use Display here:
 * their gamut and gamma are applied in the ISP using internal derived tuning,
 * reloaded when the profile changes, not a second curve over rendered RGB. */
enum class ProresTransfer {
    Display,      /* pass through the ISP's own rendering (Rec.709-ish) */
    GpLog2,       /* the GPLOG2 curve already used elsewhere in this camera */
    BmdFilmGen5,  /* Blackmagic Film Gen 5 */
};

class ProresEncoder {
public:
    /* Apple's four 4:2:2 profiles. HQ and Standard are included because they
     * are cheap to expose, not because they are usable at 4K on this board. */
    enum class Profile { Proxy, Lt, Standard, Hq };

    /* How many frames the encoder can be holding on top of the one the caller
     * is submitting. libavcodec frame-threads prores_aw, so send_frame()
     * returns before the frame is encoded and its buffer stays borrowed until
     * that thread finishes. The recorder needs this to size the borrow pool --
     * see the queue clamp in DngRecordingQueue::startAtFolder(). */
    static unsigned inFlightFrames();

    /* Bench tuning override: environment first, then /home/pi/.apertar-tuning
     * re-read at clip open. Empty string when the knob is unset. See the
     * comment on tuningOverride() in the .cpp. */
    static std::string tuningValue(const char *name);


    struct Config {
        Profile profile = Profile::Lt;
        ProresTransfer transfer = ProresTransfer::Display;
        ProresColorProfile colorProfile = ProresColorProfile::Standard;
        uint32_t width = 3840;
        uint32_t height = 2160;
        double fps = 30.0;
        std::string timecode;
        /* Frame threads. prores_aw implements ONLY frame threading -- slice
         * threading is accepted and silently does nothing (measured: 89.5
         * ms/frame at 1..4 slice threads, unchanged). Frame threading costs
         * `threads` frames of latency. */
        unsigned threads = 4;
        /* Rate control. quick_rc skips the exhaustive qp search; measured
         * -0.19 dB PSNR at LT for 9% fewer bits and ~10% less CPU. */
        bool quickRc = true;
        /* See the note beside the av_opt_set in openClip(). */
        bool qpPred = true;
        /* WARM-UP FRAMES: encoded and THROWN AWAY before the clip's first
         * recorded frame. See beginWarmUp() and the note in drain(). Zero keeps
         * the old behaviour, where frame 1 of the take is also frame 1 through
         * a cold encoder. */
        unsigned warmUpFrames = 0;
        /* Convert inside the encoder's slice fetch instead of in a separate
         * full-frame pass. Needs the 0002-prores_aw-fused-rgb48-input patch.
         * Measured at 3840x2160 ProRes 422 on ClearHDR, GP-Log2:
         *     separate pass  59.92 ms  16.7 fps
         *     fused          45.31 ms  22.1 fps
         *     fused + NEON   41.74 ms  24.0 fps
         * Output is byte-identical across all three. Off only for an A/B. */
        bool fusedRgb = true;
        /* THE ISP HANDS US FINISHED 10-BIT 4:2:2 -- no CPU colour pass.
         *
         * With the patched pispbe driver and libcamera, the PiSP back end can
         * write Q216: three contiguous planes of 16-bit YUV 4:2:2, shifted
         * right 6 in hardware so the samples are 10-bit. The limited-range
         * output CSC already puts black at 16<<8 and white at 235<<8, so after
         * the shift they land on 64..940 -- ProRes legal range. That layout IS
         * AV_PIX_FMT_YUV422P10LE, so the encoder points straight at the ISP
         * buffer: no matrix, no LUT, no staging copy, and no patched
         * libavcodec (whose source no longer exists -- see PRORES-FINDINGS).
         *
         * WHY IT IS WORTH IT. The BE's recording write is what caps 4K, not
         * the encoder: measured 2026-09-09, RGB48 at 6 B/px delivers 42 fps
         * with ~178 sensor frames lost upstream and the encode queue EMPTY,
         * while 4 B/px delivers 60.0 with none lost.
         *
         * Mutually exclusive with fusedRgb. */
        bool ispYuv422p10 = false;
        /* Clockwise rotation a player should apply, in degrees (0/90/180/270).
         * Written as the container's DISPLAY MATRIX rather than baked into the
         * pixels: the camera can be mounted inverted, and the operator's fix
         * for that is a UI orientation setting, not a re-shoot.
         *
         * Metadata and not a real rotation because the alternative -- flipping
         * the sensor -- turns the Bayer order from RGGB into BGGR, and the QRAW
         * encoder accepts only RGGB and GBRG (QrawEncoder.cpp). A sensor flip
         * would therefore fix ProRes by breaking QRAW. The display matrix costs
         * nothing, is what every NLE reads, and leaves the pixels untouched. */
        int rotationDegrees = 0;
        /* PIXEL aspect ratio, as a fraction, for the anamorphic sensor modes.
         *
         * The IMX585's 1x2 modes subsample vertically: "3840x2080" reads a
         * 2080-row window and delivers 1040 rows, and "3840x1600" delivers 800.
         * The picture is therefore 3840x1040 of storage describing a 3840x2080
         * scene, and without this the clip is a square-pixel 3840x1040 file
         * that every NLE draws squashed to half height.
         *
         * QRAW and cDNG record the same fact as `vertical_squeeze` in their
         * sidecars. A MOV has a place for it in the container -- pasp -- so it
         * goes there instead, which is what a player and an NLE actually read.
         *
         * DAR = (width / height) * (num / den), so a 2:1 vertical squeeze is
         * num/den = 1/2: each stored pixel is half as wide as it is tall. */
        int pixelAspectNum = 1;
        int pixelAspectDen = 1;
    };

    explicit ProresEncoder(const Config &config);
    ~ProresEncoder();

    ProresEncoder(const ProresEncoder &) = delete;
    ProresEncoder &operator=(const ProresEncoder &) = delete;

    /* ---- WHERE THE ENCODE WORKER'S TIME WENT, PER FRAME ------------------
     *
     * Added 2026-09-12 for the 640-910 ms zero-copy hold. The hold was ONE
     * interval -- camera thread to AVBuffer free hook -- spanning queue
     * residency, the worker, and the frame's whole life inside libavcodec,
     * with no intermediate stamp, so a 900 ms hold could not be attributed to
     * any leg (ZEROCOPY-HOLD.md §1). These are the stamps inside writeFrame(),
     * and they split the worker's part of it into the three things that can
     * actually block there:
     *
     *   sendNs   avcodec_send_frame(). Frame threading means this returns
     *            immediately UNLESS every frame thread is busy, in which case
     *            the worker blocks here. A long sendNs is ENCODER BACKPRESSURE
     *            -- the head-of-line block §5.1 argues is structurally forced.
     *   drainNs  the avcodec_receive_packet loop, which is also where the
     *            picture is MUXED AND WRITTEN.
     *   writeNs  the av_interleaved_write_frame() part of drainNs on its own.
     *            This is the disk. A stall here and nowhere else is the ext4
     *            journal / writeback hypothesis (§4.1) confirmed; a stall in
     *            drainNs with writeNs flat is the encoder, not the card.
     *   flushNs  the periodic avio_flush().
     *
     * WHAT writeNs MEANS ONCE THE WRITER THREAD IS ARMED (2026-09-13).
     *
     * These stamps exist to explain a ZERO-COPY HOLD, and a hold is made of
     * time THE WORKER SPENT NOT DRAINING. So writeNs keeps that meaning rather
     * than its literal one: it is the worker's own cost of getting this packet
     * onto the disk path.
     *
     *   APERTAR_PRORES_WRITER_THREAD=0  the worker does the write itself, so
     *                                writeNs IS av_interleaved_write_frame,
     *                                exactly as it has always been.
     *   APERTAR_PRORES_WRITER_THREAD=1  the worker hands the packet to the muxer
     *                                thread, so writeNs is the HANDOFF --
     *                                near zero normally, and equal to the
     *                                blocking time when the writer has fallen
     *                                far enough behind to fill its queue.
     *                                That blocking time is the only part of a
     *                                writeback stall that can still reach the
     *                                encode queue, which is precisely the
     *                                quantity worth carrying on the hold line.
     *
     * `asyncWrite` says which of the two a given sample is, so a log line
     * never has to be guessed at. The LITERAL write duration is not lost: the
     * writer thread times its own av_interleaved_write_frame and feeds the
     * same writePeakNanos() the synchronous arm does, so prores_write_peak_ms
     * keeps meaning "the worst real write this clip saw" in both arms -- it is
     * the instrument that proved the diagnosis, and it had to survive the fix.
     *
     * `sendDoneAtNs` and `doneAtNs` are steady_clock nanoseconds, the same
     * clock the borrow stamps use, so the caller can subtract them against
     * borrowedAtNs directly. */
    struct FrameTiming {
        int64_t sendDoneAtNs = 0;
        int64_t doneAtNs = 0;
        uint64_t sendNs = 0;
        uint64_t drainNs = 0;
        uint64_t writeNs = 0;
        uint64_t flushNs = 0;
        /* True when writeNs is a handoff rather than a write. See above. */
        bool asyncWrite = false;
    };

    /* Prepares the clip; the first frame supplies the container timecode. */
    bool openClip(const std::string &path, std::string *error);
    /* Converts, encodes and muxes one ISP frame. `timing`, when given,
     * receives the split above; passing null is exactly the old behaviour. */
    bool writeFrame(const IspFrameView &frame, std::string *error,
                    FrameTiming *timing = nullptr);
    /* Flushes the encoder's frame-thread pipeline and writes the sample table.
     * MUST be called before the caller is told recording stopped, for the same
     * reason QrawEncoder::flushPendingWrites() is (CameraController.cpp:5791):
     * "stopped" has to mean "on the card". */
    bool closeClip(std::string *error);

    /* Measured ceilings, not nominal ones -- see PRORES-FINDINGS.md. Used by
     * the fps ladder the way QrawEncoder::sustainableFps() is. */
    static double sustainableFps(Profile profile, uint32_t width, uint32_t height);
    static const char *clipExtension() { return ".mov"; }
    static const char *profileName(Profile profile);

    /* Frames WRITTEN to the clip -- warm-up frames are not among them, so this
     * is the movie's true length and not the count of frames submitted. */
    uint64_t framesEncoded() const;
    /* How many frames were encoded and discarded to warm the pipeline. Reported
     * so a clip that is N frames shorter than the take is explainable. */
    uint64_t warmUpFramesDiscarded() const;
    uint64_t bytesWritten() const;

    /* Per-clip worst case of each leg above, in nanoseconds. Read by the
     * recording queue just before it destroys the encoder, for the same reason
     * warmUpFramesDiscarded() is: the perf file is written after the worker
     * exits, by which point the only object that knew these numbers is gone. */
    uint64_t sendPeakNanos() const;
    uint64_t drainPeakNanos() const;
    /* The worst REAL av_interleaved_write_frame of the clip, whichever thread
     * ran it -- the worker in the synchronous arm, the muxer thread in the
     * writer arm. Deliberately unchanged in meaning by the writer thread. */
    uint64_t writePeakNanos() const;
    uint64_t flushPeakNanos() const;
    /* WRITER ARM ONLY, and zero in the synchronous arm. The worst time the
     * encode worker spent BLOCKED handing a packet over because the writer
     * queue was full. This is the writer arm's own failure mode made visible:
     * it stays zero for as long as the queue absorbs the stall, and the moment
     * it is not zero the stall has reached the encode queue again and the
     * queue wants to be deeper. Read it beside writePeakNanos() -- a large
     * write peak with a zero block peak is the fix working. */
    uint64_t writeBlockPeakNanos() const;
    /* Deepest the writer queue ever got, in packets, and how many single
     * writes ran over the slow-write log threshold. Zero in the sync arm. */
    uint64_t writerQueuePeak() const;
    uint64_t writerSlowWrites() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace apertar
