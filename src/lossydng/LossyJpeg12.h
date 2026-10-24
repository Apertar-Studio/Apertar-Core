/* 12-bit baseline (SOF1, extended sequential Huffman) JPEG encoder for lossy DNG tiles.
 * Matches the Blackmagic 3:1 CinemaDNG tile format measured from /media/RAW/BMD Example.dng:
 *   SOI, DQT(16-bit), DHT x4, SOF1 precision=12 ncomp=2 H=V=1, SOS x2 (non-interleaved), EOI
 * Huffman tables are optimised per tile (two-pass), as Blackmagic's are. */
#ifndef LOSSY_JPEG12_H
#define LOSSY_JPEG12_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Rate-control statistics, gathered for free in the entropy loop.
 *
 * mag[c] counts AC coefficients whose JPEG magnitude category is c (1..15), i.e.
 * |value| in [2^(c-1), 2^c). Because the emit loop already computes that category
 * for every nonzero, collecting this costs one increment per symbol.
 *
 * It is a distribution, not just a byte count, which is what makes it usable as
 * FEED-FORWARD: scaling the quantiser by r shifts every magnitude by log2(r), so
 * the whole histogram can be re-evaluated for any other table without re-encoding.
 * A bytes-only feedback loop cannot do that -- the plant gain d(ln bytes)/d(ln
 * scale) spans 14x across the operating range, so no fixed loop gain is both
 * stable and fast. */
typedef struct {
    uint32_t mag[16];      /* AC magnitude-category counts, all frequencies */
    /* HIGH-FREQUENCY ONLY (zigzag rank >= 32). On raw sensor content these
     * coefficients are overwhelmingly photon noise, so their median magnitude
     * is a usable estimate of the frame's own noise floor -- which is what the
     * quality floor needs. The all-frequency histogram cannot serve: it carries
     * low-frequency STRUCTURE too, which inflates the median and would make a
     * detailed frame look noisy. */
    uint32_t mag_hf[16];
    uint32_t nz;           /* total AC nonzeros */
    uint32_t nz_hf;        /* nonzeros with zigzag rank >= 32 */
    uint32_t blocks;       /* blocks contributing */
    uint64_t scan_bits;    /* entropy-coded bits actually emitted */
} lj12_rate_stats;

/* Reusable per-worker context. Holds the coefficient and output scratch so a
 * recording loop does not malloc (and page-fault) ~17.5 MB per 4K frame.
 * Create one per encode worker; it is NOT thread-safe. */
typedef struct lj12_ctx lj12_ctx;
lj12_ctx *lj12_ctx_create(void);
void      lj12_ctx_destroy(lj12_ctx *ctx);

/* Huffman tables that can be carried across frames. Building them needs a statistics
 * pass over every coefficient, which costs ~27% of the whole encode; reusing the
 * previous frame's tables for the SAME TILE INDEX deletes that pass for a measured
 * +0.5-1.2% in file size. Tables built for reuse are seeded so that every legal
 * 12-bit symbol is codable, otherwise a symbol unseen in the training frame would
 * emit a zero-length code and corrupt the stream. */
typedef struct lj12_tables lj12_tables;
lj12_tables *lj12_tables_create(void);
void         lj12_tables_destroy(lj12_tables *t);
void         lj12_tables_invalidate(lj12_tables *t);   /* force a rebuild, e.g. scene cut */
int          lj12_tables_valid(const lj12_tables *t);

/* tabs == NULL  -> build tables from this tile (two-pass, bit-exact with lj12_encode_tile_ctx)
 * tabs invalid  -> build, and store them into tabs for subsequent frames
 * tabs valid    -> reuse, skipping the statistics pass entirely */
int lj12_encode_tile_reuse(lj12_ctx *ctx,
                           const uint16_t *comp0, const uint16_t *comp1,
                           int w, int h, int stride0, int stride1,
                           const uint16_t qnat[64], lj12_tables *tabs,
                           const uint8_t **out, size_t *out_len);

/* Encode into ctx-owned scratch. *out points INTO the context and is valid until
 * the next call on that context -- copy it out before reusing. */
int lj12_encode_tile_ctx(lj12_ctx *ctx,
                         const uint16_t *comp0, const uint16_t *comp1,
                         int w, int h, int stride0, int stride1,
                         const uint16_t qnat[64],
                         const uint8_t **out, size_t *out_len);

/* Encode a tile DIRECTLY from the Bayer frame, with no materialised component
 * planes. The BMD layout means an 8-sample run in a component is 8 samples
 * strided by 2 in the Bayer raster (vld2q_u16 territory), and because the
 * phase seam sits at component column w/2 -- a multiple of 8 for any legal
 * tile -- no 8x8 block ever straddles it. `fwd` is an optional 4096-entry
 * compand LUT applied during the gather (NULL = identity).
 * `bayer` points at the tile's top-left CFA sample; `stride` is in samples. */
int lj12_encode_tile_cfa(lj12_ctx *ctx, const uint16_t *bayer, long stride,
                         int tile_w, int tile_h, int sample_shift,
                         const uint16_t *fwd, const uint16_t qnat[64],
                         lj12_tables *tabs, const uint8_t **out, size_t *out_len);

/* As above, but also accumulates rate statistics into *rs (pass NULL to skip).
 * *rs is ADDED TO, not reset, so a caller can sum a whole frame's tiles. */
int lj12_encode_tile_cfa_rs(lj12_ctx *ctx, const uint16_t *bayer, long stride,
                            int tile_w, int tile_h, int sample_shift,
                            const uint16_t *fwd, const uint16_t qnat[64],
                            lj12_tables *tabs, const uint8_t **out, size_t *out_len,
                            lj12_rate_stats *rs);

/* Blackmagic lossy CinemaDNG stores each W x H Bayer tile as one 12-bit SOF1
 * component with JPEG geometry 2W x H/2. Even source rows occupy the left
 * half and odd source rows the right half. */
int lj12_encode_tile_bmd_rs(lj12_ctx *ctx, const uint16_t *bayer, long stride,
                            int tile_w, int tile_h, int sample_shift,
                            const uint16_t *fwd, const uint16_t qnat[64],
                            lj12_tables *tabs, const uint8_t **out, size_t *out_len,
                            lj12_rate_stats *rs);

/* Solve the rate model for the quantiser scale that lands on `budget_bytes`.
 * `cur_scale` is the scale `rs` was gathered at. Returns a scale in [1,20000].
 * Self-calibrating: bits-per-symbol overhead is derived from rs->scan_bits, so
 * it tracks content and needs no offline fit. */
int lj12_solve_scale(const lj12_rate_stats *rs, int cur_scale, double budget_bytes);

/* The COARSEST scale whose quantisation error still sits under `alpha` times the
 * frame's own noise floor. Returns 0 when it cannot be estimated (too few
 * high-frequency nonzeros -- e.g. an almost-black frame), meaning "no opinion".
 *
 * Use it as a FLOOR on coarseness, not a target:
 *     scale = max(rate_solved, quality_solved)
 * so it can only ever make a frame smaller, never finer. A named ratio is a
 * CEILING -- on clean content 3:1 is not reachable at all (the finest table
 * yields ~3.8:1), and without this a clean frame is handed bits it cannot use. */
int lj12_solve_quality_scale(const lj12_rate_stats *rs, int cur_scale, double alpha);

/* Encode one tile from two component planes, each w x h, 12-bit samples (0..4095).
 * qnat = 64-entry quantisation table in NATURAL (row-major) order.
 * Returns 0 on success; *out is malloc'd, caller frees. */
int lj12_encode_tile(const uint16_t *comp0, const uint16_t *comp1,
                     int w, int h, int stride0, int stride1,
                     const uint16_t qnat[64],
                     uint8_t **out, size_t *out_len);

/* Scale a base quant table by a percentage (100 = unchanged). Clamps to 1..65535. */
void lj12_scale_qtable(const uint16_t base[64], int scale_pct, uint16_t out[64]);

/* The measured Blackmagic 3:1 table, natural order. */
extern const uint16_t LJ12_QTBL_BMD_3TO1[64];
/* The table the encoder uses: BMD's shape with the HF ramp square-rooted.
 * -26% edge chroma error at matched bytes -- see the measurements in the .cpp. */
extern const uint16_t LJ12_QTBL_CFA_G05[64];

#ifdef __cplusplus
}
#endif
#endif
