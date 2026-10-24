#pragma once

/* Colour conversion for the ProRes path: PiSP RGB48 -> yuv422p10le, plus the
 * transfer curves. Split out of ProresEncoder.cpp because this is the part
 * that is hot, hand-vectorised and worth benchmarking on its own. */

#include <cstdint>
#include <cstddef>

namespace apertar {
namespace prores_color {

/* 16-bit-in / 10-bit-out transfer LUT. Built once per clip: the curve is a
 * per-clip property, and a 65536-entry gather is one L2-resident load per
 * component against a transcendental per pixel. */
struct TransferLut {
    uint16_t table[65536];
};

void buildDisplayLut(TransferLut *lut);
void buildGpLog2Lut(TransferLut *lut);
void buildBmdFilmGen5Lut(TransferLut *lut);

/* RGB48LE -> planar yuv422p10le, BT.709 matrix, limited range (64..940 luma,
 * 64..960 chroma), horizontal chroma averaged over the co-sited pair.
 *
 * `lut` may be null, in which case the ISP's own rendering is passed through
 * with a plain 16->10 bit shift. Destination planes must be 16-byte aligned;
 * yStride/cStride are in SAMPLES, not bytes. Width must be even. */
void rgb48ToYuv422p10(const uint8_t *rgb, size_t rgbStride,
                      uint16_t *y, size_t yStride,
                      uint16_t *u, uint16_t *v, size_t cStride,
                      uint32_t width, uint32_t height,
                      const TransferLut *lut);

}  // namespace prores_color
}  // namespace apertar

namespace apertar {
namespace prores_color {

/* Band-parallel colour conversion.
 *
 * WHY THIS EXISTS. The single-threaded CSC is the pipeline's serial section
 * and it dominates: measured end to end at 3840x2160 ProRes LT, 53.1 ms/frame
 * against an encode-only cost of 28.7, i.e. ~24 ms of the frame is one core
 * doing colour while the other three wait on it. Under a log curve it is worse
 * (73.9 ms) because the LUT gather has no NEON form and the whole row walk
 * goes scalar. Amdahl, not bandwidth: splitting the frame into row bands and
 * running them on a persistent pool removes almost all of it.
 *
 * The pool is persistent because at 60 fps a per-frame thread create/join pair
 * is ~40-80 us of pure overhead, and because the encoder's own frame threads
 * already exist -- churning more of them fights the scheduler for no reason.
 *
 * Bands are whole rows and always an EVEN number of rows apart, so 4:2:2
 * chroma pairing never straddles a band edge. */
class CscPool {
public:
    explicit CscPool(unsigned threads);
    ~CscPool();
    CscPool(const CscPool &) = delete;
    CscPool &operator=(const CscPool &) = delete;

    void convert(const uint8_t *rgb, size_t rgbStride,
                 uint16_t *y, size_t yStride,
                 uint16_t *u, uint16_t *v, size_t cStride,
                 uint32_t width, uint32_t height,
                 const TransferLut *lut);

    unsigned threads() const { return threads_; }

private:
    struct Pool;
    Pool *pool_;
    unsigned threads_;
};

}  // namespace prores_color
}  // namespace apertar

