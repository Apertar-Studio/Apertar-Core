#include "ProresColor.hpp"

#include <cmath>
#include <cstring>
#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace apertar {
namespace prores_color {
namespace {

inline uint16_t clamp10(int v) { return v < 0 ? 0 : (v > 1023 ? 1023 : (uint16_t)v); }

/* Scene-linear normalised 0..1 -> coded 0..1, then to 10-bit limited range. */
void fillFromCurve(TransferLut *lut, double (*curve)(double))
{
    for (int i = 0; i < 65536; ++i) {
        const double x = i / 65535.0;
        double y = curve(x);
        if (y < 0.0) y = 0.0;
        if (y > 1.0) y = 1.0;
        lut->table[i] = clamp10((int)lround(y * 876.0 + 64.0));
    }
}

/* The ISP has already applied its own rendering, so "display" is a straight
 * range map, not a curve. */
double displayCurve(double x) { return x; }

/* THE TWO LOG CURVES ARE THE SAME FUNCTION WITH ONE DIFFERENT CONSTANT.
 *
 * Both are the Protune log1p family
 *
 *     y = ln(1 + k*x) / ln(1 + k)          x, y in 0..1
 *
 * which is not an assumption: the LinearizationTable (tag 50712, 4096 entries)
 * was extracted from the Blackmagic CinemaDNG in /media/RAW and fitted, and
 * this one-parameter model reproduces the file's own table to 0.10 % worst
 * case over codes 600-3600. Method and residuals: dng-analysis.md.
 *
 *     GP-Log2         k = 599      (what this camera already ships)
 *     Blackmagic Film k = 137.36   (fitted from the DNG)
 *
 * WHY GP-LOG2 IS THE ONE TO KEEP, given the brief was maximum dynamic range.
 * The two are within a whisker of each other over the top four stops (453.2
 * against 443.6 codes per stop). They part company in the shadows, and they
 * part company the wrong way for a wide-DR sensor: below -8 stops GP-Log2
 * still spends 772 codes where Blackmagic Film spends 274, it quantises
 * shadows 2x to 6.5x more finely, and it stays log-like 2.12 stops deeper.
 * Adopting k = 137 would buy about 2 % in the highlights and pay 2.12 stops
 * for it. Blackmagic Film is offered here because it is a deliverable a
 * colourist may ASK for by name, not because it holds more range.
 *
 * Blackmagic's real curve also carries a linear toe below code ~470 and a 9.3 %
 * sub-black pedestal. The toe is reproduced; the pedestal is a container
 * convention rather than part of the transfer and is left to the DNG path. */
double log1pCurve(double x, double k)
{
    if (x <= 0.0) return 0.0;
    return std::log(1.0 + k * x) / std::log(1.0 + k);
}

/* THE LINEAR TOE'S JOIN, as a fraction of full scale, matching the ISP's
 * gamma_curve knot. Keep the two in step: the ISP renders the recorded picture
 * on the display transfer, so a mismatch here is a curve that disagrees with
 * the one on the card. */
constexpr double kGpLog2ToeJoin = 0.002289;

/* GP-LOG2 WITH A LINEAR TOE.
 *
 * Above the join it is exactly the curve it always was. Below it, a straight
 * line with the log's own slope at the join -- so the splice is C1 and, because
 * a log is concave, the line reaches x=0 ABOVE y=0. That lifted black is a
 * SUB-BLACK PEDESTAL of 4.46%, and it is the whole point:
 *
 *   - the encode slope near black is bounded at 39.5 instead of running to
 *     k/ln(1+k) = 93.6, so one code of residual sensor pedestal no longer
 *     becomes ninety-three codes of lifted, blue-tinted shadow;
 *   - the negative half of the shadow noise distribution survives instead of
 *     being rectified against zero. The previous curve clamped at 0, which
 *     raises the mean of the noisier channel and is why the shadows went blue
 *     with gain -- see the black-level work in the 2026-09-06 handoff.
 *
 * The join is EIGHT SIGMA above the measured ISO 100 noise floor (sigma 1.12
 * codes of 3895 full scale, temporal difference of consecutive cDNG frames).
 * Below four sigma there is no picture to describe, only noise; above eight the
 * toe would start eating real shadow detail. The resulting pedestal is four
 * sigma deep, which is what makes the rectification go away rather than merely
 * shrink.
 *
 * Blackmagic's own DNG carries a 9.3% sub-black pedestal, measured from the
 * reference file in dng-analysis.md -- the same structure, arrived at
 * independently. */
double gpLog2Curve(double x)
{
    constexpr double kK = 599.0;
    const double denom = std::log(1.0 + kK);
    const double yJoin = std::log(1.0 + kK * kGpLog2ToeJoin) / denom;
    const double slope = kK / ((1.0 + kK * kGpLog2ToeJoin) * denom);
    const double y0 = std::max(0.0, yJoin - slope * kGpLog2ToeJoin);
    if (x <= 0.0)
        return y0;
    if (x < kGpLog2ToeJoin)
        return y0 + slope * x;
    return log1pCurve(x, kK);
}

/* The fitted Blackmagic toe: linear below the join, log above, meeting
 * tangentially the way Cineon-derived curves do. Join taken from the fit at
 * code ~470 of 4095. */
double bmdFilmCurve(double x)
{
    constexpr double kK = 137.36;
    constexpr double kJoin = 0.0115;
    if (x <= 0.0) return 0.0;
    if (x < kJoin) {
        /* slope of the log branch at the join, so the splice is C1 */
        const double slope = kK / ((1.0 + kK * kJoin) * std::log(1.0 + kK));
        return log1pCurve(kJoin, kK) - slope * (kJoin - x);
    }
    return log1pCurve(x, kK);
}

}  // namespace

void buildDisplayLut(TransferLut *lut) { fillFromCurve(lut, displayCurve); }
void buildGpLog2Lut(TransferLut *lut) { fillFromCurve(lut, gpLog2Curve); }
void buildBmdFilmGen5Lut(TransferLut *lut) { fillFromCurve(lut, bmdFilmCurve); }

/* ---- BT.709, FULL-RANGE RGB IN, LIMITED-RANGE 10-BIT YCbCr OUT ---------
 *
 * Q10 fixed point. Derived, not guessed, and pinned by prores_csc_test:
 *
 *   Y  = 64  + (186r + 628g +  63b) >> 10      white -> 940
 *   Cb = 512 + (-103r - 346g + 449b) >> 10     blue  -> 960
 *   Cr = 512 + ( 449r - 407g -  42b) >> 10     red   -> 960
 *
 * The chroma rows sum to zero, which is what makes a neutral pixel land
 * exactly on 512 at every brightness.
 *
 * The previous values (183/614/62, -101/-338/439, 439/-399/-40) were the same
 * matrix scaled by 0.976: hue-correct, but white came out at 922 instead of 940
 * and every colour was 2.4% desaturated. Small, and it cost 2% of the code
 * range at the top for nothing. */
constexpr int kYr = 186, kYg = 628, kYb = 63;
constexpr int kCbr = -103, kCbg = -346, kCbb = 449;
constexpr int kCrr = 449, kCrg = -407, kCrb = -42;

/* ---- THE SAME MATRIX IN THE CODED DOMAIN (the LUT path) ----------------
 *
 * There the inputs are ALREADY 64..940, so luma is a plain weighted mean that
 * sums to 1024, and chroma scales the difference from luma:
 *
 *   Cb = 512 + ((b - y) * 565) >> 10       565/1024 = 448/876/0.9278
 *   Cr = 512 + ((r - y) * 665) >> 10       665/1024 = 448/876/0.7874
 *
 * THE OLD Cr WAS 519, AND THAT IS A REAL COLOUR ERROR, not a rounding one:
 * 519/665 = 0.78, so red was 22% short of where it belonged on every clip
 * recorded with a log or Blackmagic transfer. Cb was 590 against 565, 4.5% too
 * far the other way -- and far enough that pure blue produced Cb 980, outside
 * the legal 64..960 window, so a conforming decoder clipped it. Together they
 * tilt the whole image off the red axis and towards cyan, which is what "looks
 * like a LUT has been applied to it" actually was. */
constexpr int kCodedCb = 565, kCodedCr = 665;

/* BT.709 luma/chroma coefficients in Q10, limited range. */
/* Row-range core. Both the single-threaded entry point and CscPool use this;
 * rowBegin/rowEnd are whole rows so 4:2:2 pairing cannot straddle a band. */
void convertRows(const uint8_t *rgb, size_t rgbStride,
                 uint16_t *y, size_t yStride,
                 uint16_t *u, uint16_t *v, size_t cStride,
                 uint32_t width, uint32_t rowBegin, uint32_t rowEnd,
                 const TransferLut *lut)
{
    for (uint32_t j = rowBegin; j < rowEnd; ++j) {
        const uint16_t *p = reinterpret_cast<const uint16_t *>(rgb + (size_t)j * rgbStride);
        uint16_t *yr = y + (size_t)j * yStride;
        uint16_t *ur = u + (size_t)j * cStride;
        uint16_t *vr = v + (size_t)j * cStride;

        uint32_t i = 0;
#if defined(__aarch64__)
        /* The LUT path cannot be gathered in NEON (no 64K gather), so it stays
         * scalar; the pass-through path is the one that vectorises. This is
         * the right split: a graded deliverable wants the curve and can afford
         * the scalar walk, while the display path is the one used for the
         * long, fast, uncurved takes. */
        if (!lut) {
            const int16x8_t kyr = vdupq_n_s16(kYr), kyg = vdupq_n_s16(kYg), kyb = vdupq_n_s16(kYb);
            for (; i + 8 <= width; i += 8) {
                const uint16x8x3_t t = vld3q_u16(p + 3 * i);
                const int16x8_t r = vreinterpretq_s16_u16(vshrq_n_u16(t.val[0], 6));
                const int16x8_t g = vreinterpretq_s16_u16(vshrq_n_u16(t.val[1], 6));
                const int16x8_t b = vreinterpretq_s16_u16(vshrq_n_u16(t.val[2], 6));

                int32x4_t ylo = vmull_s16(vget_low_s16(r), vget_low_s16(kyr));
                ylo = vmlal_s16(ylo, vget_low_s16(g), vget_low_s16(kyg));
                ylo = vmlal_s16(ylo, vget_low_s16(b), vget_low_s16(kyb));
                int32x4_t yhi = vmull_s16(vget_high_s16(r), vget_high_s16(kyr));
                yhi = vmlal_s16(yhi, vget_high_s16(g), vget_high_s16(kyg));
                yhi = vmlal_s16(yhi, vget_high_s16(b), vget_high_s16(kyb));
                const uint16x8_t yv = vcombine_u16(vqmovun_s32(vshrq_n_s32(ylo, 10)),
                                                   vqmovun_s32(vshrq_n_s32(yhi, 10)));
                vst1q_u16(yr + i, vminq_u16(vaddq_u16(yv, vdupq_n_u16(64)),
                                            vdupq_n_u16(1023)));

                /* Co-sited pair average, 8 luma -> 4 chroma. */
                const int16x4_t rm = vmovn_s32(vshrq_n_s32(vpaddlq_s16(r), 1));
                const int16x4_t gm = vmovn_s32(vshrq_n_s32(vpaddlq_s16(g), 1));
                const int16x4_t bm = vmovn_s32(vshrq_n_s32(vpaddlq_s16(b), 1));
                int32x4_t uu = vmull_n_s16(rm, kCbr);
                uu = vmlal_n_s16(uu, gm, kCbg); uu = vmlal_n_s16(uu, bm, kCbb);
                int32x4_t vv = vmull_n_s16(rm, kCrr);
                vv = vmlal_n_s16(vv, gm, kCrg); vv = vmlal_n_s16(vv, bm, kCrb);
                /* THE +512 GOES ON WHILE THE VALUE IS STILL SIGNED.
                 *
                 * This used to narrow first -- vqmovun_s32(uu >> 10) -- and
                 * vqmovun is a SIGNED-to-UNSIGNED saturating narrow, so every
                 * negative result became 0 and then +512, i.e. exactly neutral.
                 * Chroma below neutral could not be represented AT ALL.
                 *
                 * Half of colour space is below neutral on one axis or the
                 * other, so this was not subtle. A warm scene wants Cb BELOW
                 * 512 (yellow) while Cr sits above it (red); the Cb half was
                 * clamped flat and the Cr half was not, which leaves exactly
                 * one colour: magenta. Tungsten-lit walls came out pink.
                 *
                 * Only the vector body was affected -- the scalar tail below
                 * does the arithmetic in int and clamps properly -- so the last
                 * few pixels of every row were right and the other 3832 were
                 * not. */
                uu = vaddq_s32(vshrq_n_s32(uu, 10), vdupq_n_s32(512));
                vv = vaddq_s32(vshrq_n_s32(vv, 10), vdupq_n_s32(512));
                const int32x4_t lo = vdupq_n_s32(0), hi = vdupq_n_s32(1023);
                uu = vminq_s32(vmaxq_s32(uu, lo), hi);
                vv = vminq_s32(vmaxq_s32(vv, lo), hi);
                vst1_u16(ur + (i >> 1), vmovn_u32(vreinterpretq_u32_s32(uu)));
                vst1_u16(vr + (i >> 1), vmovn_u32(vreinterpretq_u32_s32(vv)));
            }
        }
#endif
        for (; i < width; i += 2) {
            int r0, g0, b0, r1, g1, b1;
            if (lut) {
                r0 = lut->table[p[3 * i + 0]];     g0 = lut->table[p[3 * i + 1]];     b0 = lut->table[p[3 * i + 2]];
                r1 = lut->table[p[3 * i + 3]];     g1 = lut->table[p[3 * i + 4]];     b1 = lut->table[p[3 * i + 5]];
                /* Already 10-bit coded by the LUT: matrix in the coded domain,
                 * which is what every log workflow expects. */
                const int y0 = (218 * r0 + 732 * g0 + 74 * b0) >> 10;
                const int y1 = (218 * r1 + 732 * g1 + 74 * b1) >> 10;
                yr[i] = clamp10(y0); if (i + 1 < width) yr[i + 1] = clamp10(y1);
                const int rm = (r0 + r1) >> 1, gm = (g0 + g1) >> 1, bm = (b0 + b1) >> 1;
                const int ym = (218 * rm + 732 * gm + 74 * bm) >> 10;
                ur[i >> 1] = clamp10(512 + (((bm - ym) * kCodedCb) >> 10));
                vr[i >> 1] = clamp10(512 + (((rm - ym) * kCodedCr) >> 10));
            } else {
                r0 = p[3 * i + 0] >> 6; g0 = p[3 * i + 1] >> 6; b0 = p[3 * i + 2] >> 6;
                r1 = p[3 * i + 3] >> 6; g1 = p[3 * i + 4] >> 6; b1 = p[3 * i + 5] >> 6;
                yr[i] = clamp10(64 + ((kYr * r0 + kYg * g0 + kYb * b0) >> 10));
                if (i + 1 < width) yr[i + 1] = clamp10(64 + ((kYr * r1 + kYg * g1 + kYb * b1) >> 10));
                const int rm = (r0 + r1) >> 1, gm = (g0 + g1) >> 1, bm = (b0 + b1) >> 1;
                ur[i >> 1] = clamp10(512 + ((kCbr * rm + kCbg * gm + kCbb * bm) >> 10));
                vr[i >> 1] = clamp10(512 + ((kCrr * rm + kCrg * gm + kCrb * bm) >> 10));
            }
        }
    }
}


void rgb48ToYuv422p10(const uint8_t *rgb, size_t rgbStride,
                      uint16_t *y, size_t yStride,
                      uint16_t *u, uint16_t *v, size_t cStride,
                      uint32_t width, uint32_t height,
                      const TransferLut *lut)
{
    convertRows(rgb, rgbStride, y, yStride, u, v, cStride, width, 0, height, lut);
}


/* ---- CscPool ---------------------------------------------------------- */

struct CscPool::Pool {
    std::vector<std::thread> workers;
    std::mutex m;
    std::condition_variable cvStart, cvDone;
    bool stop = false;
    unsigned generation = 0;
    unsigned outstanding = 0;

    /* The job, published under `m` and read after cvStart. */
    const uint8_t *rgb = nullptr; size_t rgbStride = 0;
    uint16_t *y = nullptr; size_t yStride = 0;
    uint16_t *u = nullptr; uint16_t *v = nullptr; size_t cStride = 0;
    uint32_t width = 0, height = 0;
    const TransferLut *lut = nullptr;
    unsigned bands = 0;
};

CscPool::CscPool(unsigned threads) : pool_(new Pool), threads_(threads < 1 ? 1 : threads)
{
    /* One fewer worker than bands: the calling thread takes the last band
     * itself rather than blocking, which is both one less wakeup and one more
     * core doing useful work. */
    for (unsigned i = 0; i + 1 < threads_; ++i) {
        pool_->workers.emplace_back([this, i]() {
            Pool &p = *pool_;
            unsigned seen = 0;
            for (;;) {
                std::unique_lock<std::mutex> lk(p.m);
                p.cvStart.wait(lk, [&] { return p.stop || p.generation != seen; });
                if (p.stop) return;
                seen = p.generation;
                const uint32_t h = p.height, bands = p.bands;
                const uint8_t *rgb = p.rgb; const size_t rs = p.rgbStride;
                uint16_t *y = p.y; const size_t ys = p.yStride;
                uint16_t *u = p.u; uint16_t *v = p.v; const size_t cs = p.cStride;
                const uint32_t w = p.width; const TransferLut *lut = p.lut;
                lk.unlock();

                /* Even row boundaries: 4:2:2 pairs are horizontal, but the
                 * chroma plane is indexed by row, so a band must still start
                 * on a whole row. */
                const uint32_t per = ((h / bands) + 1u) & ~1u;
                const uint32_t b0 = std::min(h, i * per);
                const uint32_t b1 = std::min(h, b0 + per);
                if (b0 < b1)
                    convertRows(rgb, rs, y, ys, u, v, cs, w, b0, b1, lut);

                lk.lock();
                if (--p.outstanding == 0) p.cvDone.notify_one();
            }
        });
    }
}

CscPool::~CscPool()
{
    {
        std::lock_guard<std::mutex> lk(pool_->m);
        pool_->stop = true;
    }
    pool_->cvStart.notify_all();
    for (auto &t : pool_->workers) t.join();
    delete pool_;
}

void CscPool::convert(const uint8_t *rgb, size_t rgbStride,
                      uint16_t *y, size_t yStride,
                      uint16_t *u, uint16_t *v, size_t cStride,
                      uint32_t width, uint32_t height,
                      const TransferLut *lut)
{
    Pool &p = *pool_;
    const unsigned bands = threads_;
    if (bands <= 1 || height < 4) {
        convertRows(rgb, rgbStride, y, yStride, u, v, cStride, width, 0, height, lut);
        return;
    }
    {
        std::lock_guard<std::mutex> lk(p.m);
        p.rgb = rgb; p.rgbStride = rgbStride;
        p.y = y; p.yStride = yStride;
        p.u = u; p.v = v; p.cStride = cStride;
        p.width = width; p.height = height; p.lut = lut; p.bands = bands;
        p.outstanding = bands - 1;   /* the caller does the last band */
        ++p.generation;
    }
    p.cvStart.notify_all();

    const uint32_t per = ((height / bands) + 1u) & ~1u;
    const uint32_t b0 = std::min(height, (bands - 1) * per);
    if (b0 < height)
        convertRows(rgb, rgbStride, y, yStride, u, v, cStride, width, b0, height, lut);

    std::unique_lock<std::mutex> lk(p.m);
    p.cvDone.wait(lk, [&] { return p.outstanding == 0; });
}

}  // namespace prores_color
}  // namespace apertar

