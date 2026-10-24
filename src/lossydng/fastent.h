/* fastent.h — optimised entropy stage for the 12-bit SOF1 lossy-DNG encoder.
 *
 * Drop-in replacements for gather_stats()/emit_scan()/bw_* in src/lossy_jpeg12.c.
 * Everything here is bit-exact with the validated implementation given the same
 * quantised coefficients and the same Huffman tables.
 *
 * Three layers, usable independently:
 *   1. fbw_*        — 64-bit-accumulator bit writer, 4 bytes/store, branchless
 *                     0xFF detection, bounds check per block instead of per byte.
 *   2. *_ctz        — coefficient scan driven by a 64-bit nonzero bitmap + ctz,
 *                     so the loop runs once per NONZERO instead of 63 times.
 *   3. lj12_rec_*   — "record stream": the block->symbol decomposition is done
 *                     ONCE (fusable into the DCT/quantise pass), the statistics
 *                     histogram falls out for free, and the emit pass is a flat
 *                     walk with one merged 512-entry code table and no branches.
 */
#ifndef FASTENT_H
#define FASTENT_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

/* ---------------------------------------------------------------- magcat */
/* Baseline: a shift loop, 1..16 iterations, unpredictable trip count.
 * aarch64: clz is 1 instruction, the whole thing is clz+sub+csel (3 ops). */
static inline int magcat_fast(int v) {
    unsigned a = (unsigned)(v < 0 ? -v : v);
    return a ? 32 - __builtin_clz(a) : 0;
}
/* The "value bits" JPEG wants for a coefficient: v-1 if negative, masked to s.
 * Baseline uses a branch; (v + (v>>31)) is the same thing with no branch. */
static inline uint32_t magbits(int v, int s) {
    return (uint32_t)(v + (v >> 31)) & (uint32_t)((1u << s) - 1u);
}

/* ------------------------------------------------------------- bit writer */
typedef struct {
    uint8_t *p, *base, *limit;   /* limit = base + cap - FBW_SLACK */
    size_t   cap;
    uint64_t acc;                /* bits right-aligned in the low nbits */
    int      nbits;
    int      oom;
} fbw_t;

/* worst case for one 8x8 block: 64 symbols * (16 code + 16 value) bits
 * = 256 bytes, doubled by 0xFF stuffing, + 8 bytes of accumulator slop. */
/* + 8 for fbw_putw, which always stores 8 bytes and advances by however many
 * became whole. */
#define FBW_SLACK 576

static inline int fbw_init(fbw_t *b, size_t cap) {
    if (cap < 4096) cap = 4096;
    b->base = (uint8_t*)malloc(cap);
    if (!b->base) { memset(b, 0, sizeof *b); b->oom = 1; return -1; }
    b->p = b->base; b->cap = cap; b->limit = b->base + cap - FBW_SLACK;
    b->acc = 0; b->nbits = 0; b->oom = 0;
    return 0;
}
/* Call once per block (or per N records) — NOT per byte. */
static inline int fbw_reserve(fbw_t *b) {
    if (__builtin_expect(b->p <= b->limit, 1)) return 0;
    size_t used = (size_t)(b->p - b->base), nc = b->cap * 2;
    uint8_t *q = (uint8_t*)realloc(b->base, nc);
    if (!q) { b->oom = 1; return -1; }
    b->base = q; b->cap = nc; b->p = q + used; b->limit = q + nc - FBW_SLACK;
    return 0;
}

static uint8_t *fbw_stuff4(uint8_t *p, uint32_t w) {
    for (int i = 24; i >= 0; i -= 8) {
        uint8_t v = (uint8_t)(w >> i);
        *p++ = v;
        if (v == 0xFF) *p++ = 0x00;
    }
    return p;
}

/* Append n (0..32) bits. code must already be masked to n bits.
 * Invariant: nbits < 32 on entry, so one 4-byte drain at most. */
static inline void fbw_put(fbw_t *b, uint32_t code, int n) {
    b->acc = (b->acc << n) | (uint64_t)code;
    b->nbits += n;
    if (b->nbits >= 32) {
        b->nbits -= 32;
        uint32_t w  = (uint32_t)(b->acc >> b->nbits);
        uint32_t be = __builtin_bswap32(w);
        memcpy(b->p, &be, 4);
        /* exact SWAR "does any byte equal 0xFF" — haszero(~w) */
        uint32_t ff = ((~w) - 0x01010101u) & w & 0x80808080u;
        if (__builtin_expect(ff == 0, 1)) b->p += 4;
        else                              b->p  = fbw_stuff4(b->p, w);
    }
}

/* End of scan: pad to a byte boundary with 1 bits, drain, reset. */
static inline void fbw_finish(fbw_t *b) {
    int r = b->nbits & 7;
    if (r) { int pad = 8 - r; fbw_put(b, (1u << pad) - 1u, pad); }
    while (b->nbits >= 8) {
        b->nbits -= 8;
        uint8_t v = (uint8_t)(b->acc >> b->nbits);
        *b->p++ = v;
        if (v == 0xFF) *b->p++ = 0x00;
    }
    b->acc = 0; b->nbits = 0;
}
static inline void fbw_byte(fbw_t *b, uint8_t v) { *b->p++ = v; }   /* markers only */

/* --------------------------------------------------------- nonzero bitmap */
/* bit i = (c[i] != 0), c[] in zigzag order. */
static inline uint64_t nzmask_scalar(const int16_t *c) {
    uint64_t m = 0;
    for (int i = 0; i < 64; i++) m |= (uint64_t)(c[i] != 0) << i;
    return m;
}
#ifdef __ARM_NEON
static inline uint64_t nzmask_neon(const int16_t *c) {
    static const uint16_t BITW[8] = {1,2,4,8,16,32,64,128};
    const uint16x8_t bw = vld1q_u16(BITW);
    const int16x8_t  z  = vdupq_n_s16(0);
    uint64_t m = 0;
    for (int k = 0; k < 8; k++) {
        int16x8_t v   = vld1q_s16(c + k*8);
        uint16x8_t nz = vmvnq_u16(vceqq_s16(v, z));       /* 0xFFFF where nonzero */
        m |= (uint64_t)vaddvq_u16(vandq_u16(nz, bw)) << (k*8);
    }
    return m;
}
#define nzmask nzmask_neon
#else
#define nzmask nzmask_scalar
#endif

/* ------------------------------------------------- record stream (layer 3) */
/* One uint32 per emitted Huffman symbol:
 *   bits  0..15  value bits (already in JPEG's v-1-if-negative form, masked)
 *   bits 16..24  merged code-table index: AC symbol 0..255, DC uses 256+s
 *   bits 25..29  number of value bits (0..16)
 * The merged index means the emit loop does ONE table load and no DC/AC branch. */
#define REC_DC_BASE 256u
#define REC_MAX_PER_BLOCK 66

typedef struct { uint32_t code_size[512]; } rectab_t;   /* (size<<16)|code */

/* Decompose one block (zigzag-order quantised coeffs) into records and count
 * the symbols. This is the ONLY pass that touches coefficients; fuse it into
 * the quantise loop and the separate statistics pass disappears entirely.
 * `mask` is the nonzero bitmap (pass nzmask(c) if you do not have it already). */
static inline uint32_t *lj12_rec_block(uint32_t *r, const int16_t *c, uint64_t mask,
                                       int *prevdc, uint32_t *hist512)
{
    int dc = c[0], diff = dc - *prevdc;
    *prevdc = dc;
    int s = magcat_fast(diff);
    uint32_t idx = REC_DC_BASE + (unsigned)s;
    hist512[idx]++;
    *r++ = ((uint32_t)s << 25) | (idx << 16) | magbits(diff, s);

    uint64_t m = mask & ~1ULL;
    int prev = 0;
    while (m) {
        int i = __builtin_ctzll(m);
        m &= m - 1;
        int run = i - prev - 1;
        prev = i;
        while (run >= 16) { hist512[0xF0]++; *r++ = (0xF0u << 16); run -= 16; }
        int v = c[i];
        int sz = 32 - __builtin_clz((unsigned)(v < 0 ? -v : v));   /* v != 0 here */
        uint32_t sym = ((uint32_t)run << 4) | (uint32_t)sz;
        hist512[sym]++;
        *r++ = ((uint32_t)sz << 25) | (sym << 16) | magbits(v, sz);
    }
    if (prev < 63) { hist512[0]++; *r++ = 0; }   /* EOB */
    return r;
}

/* Flat emit over the record stream. bounds-checked once per chunk. */
static inline void lj12_emit_records(fbw_t *b, const uint32_t *rec, size_t n,
                                     const rectab_t *t)
{
    const size_t CHUNK = 512;      /* 512 recs * 4B max = 2KB < FBW_SLACK*4 */
    size_t k = 0;
    while (k < n) {
        size_t e = k + CHUNK; if (e > n) e = n;
        if (b->p > b->base + b->cap - (CHUNK*8 + 64)) {
            size_t used = (size_t)(b->p - b->base), nc = b->cap * 2;
            uint8_t *q = (uint8_t*)realloc(b->base, nc);
            if (!q) { b->oom = 1; return; }
            b->base = q; b->cap = nc; b->p = q + used; b->limit = q + nc - FBW_SLACK;
        }
        for (; k < e; k++) {
            uint32_t r  = rec[k];
            uint32_t cs = t->code_size[(r >> 16) & 0x1FF];
            int      nv = (int)(r >> 25);
            fbw_put(b, ((cs & 0xFFFFu) << nv) | (r & 0xFFFFu), (int)(cs >> 16) + nv);
        }
    }
}


/* ------------------------------------------------- writer variant 2: no branch
 * Accumulator is LEFT-aligned; every put stores 8 bytes unconditionally and
 * advances the pointer by however many whole bytes became available. The only
 * branch is the rare 0xFF stuffing case (~1.6% of stores at 4 bytes/put).
 * Needs 8 bytes of tail slack in the buffer (FBW_SLACK covers it). */
typedef struct {
    uint8_t *p, *base, *limit;
    size_t   cap;
    uint64_t acc;    /* valid bits packed downward from bit 63 */
    int      nbits;  /* 0..7 between puts */
    int      oom;
} fbw2_t;

static inline int fbw2_init(fbw2_t *b, size_t cap) {
    if (cap < 4096) cap = 4096;
    b->base = (uint8_t*)malloc(cap);
    if (!b->base) { memset(b,0,sizeof *b); b->oom = 1; return -1; }
    b->p = b->base; b->cap = cap; b->limit = b->base + cap - FBW_SLACK;
    b->acc = 0; b->nbits = 0; b->oom = 0;
    return 0;
}
static inline int fbw2_reserve(fbw2_t *b) {
    if (__builtin_expect(b->p <= b->limit, 1)) return 0;
    size_t used = (size_t)(b->p - b->base), nc = b->cap * 2;
    uint8_t *q = (uint8_t*)realloc(b->base, nc);
    if (!q) { b->oom = 1; return -1; }
    b->base = q; b->cap = nc; b->p = q + used; b->limit = q + nc - FBW_SLACK;
    return 0;
}
/* n must be >= 1 and <= 32; code must be masked to n bits. */
static inline void fbw2_put(fbw2_t *b, uint32_t code, int n) {
    b->acc |= (uint64_t)code << (64 - b->nbits - n);
    b->nbits += n;
    uint64_t be = __builtin_bswap64(b->acc);
    memcpy(b->p, &be, 8);
    int whole = b->nbits >> 3;
    uint64_t ff = ((~b->acc) - 0x0101010101010101ULL) & b->acc & 0x8080808080808080ULL;
    ff &= (uint64_t)0 - (uint64_t)(whole != 0);
    ff &= ~0ULL << ((64 - (whole << 3)) & 63);
    if (__builtin_expect(ff == 0, 1)) {
        b->p     += whole;
        b->acc  <<= (whole << 3);
        b->nbits -= whole << 3;
    } else {
        uint8_t *q = b->p;
        for (int i = 0; i < whole; i++) {
            uint8_t v = (uint8_t)(b->acc >> 56);
            b->acc <<= 8;
            *q++ = v;
            if (v == 0xFF) *q++ = 0x00;
        }
        b->p = q; b->nbits -= whole << 3;
    }
}
static inline void fbw2_finish(fbw2_t *b) {
    if (b->nbits) fbw2_put(b, (1u << (8 - b->nbits)) - 1u, 8 - b->nbits);
    b->acc = 0; b->nbits = 0;
}
static inline void fbw2_byte(fbw2_t *b, uint8_t v) { *b->p++ = v; }

static inline void lj12_emit_records2(fbw2_t *b, const uint32_t *rec, size_t n,
                                      const rectab_t *t)
{
    const size_t CHUNK = 512;
    size_t k = 0;
    while (k < n) {
        size_t e = k + CHUNK; if (e > n) e = n;
        if (b->p > b->base + b->cap - (CHUNK*8 + 64)) {
            size_t used = (size_t)(b->p - b->base), nc = b->cap * 2;
            uint8_t *q = (uint8_t*)realloc(b->base, nc);
            if (!q) { b->oom = 1; return; }
            b->base = q; b->cap = nc; b->p = q + used; b->limit = q + nc - FBW_SLACK;
        }
        for (; k < e; k++) {
            uint32_t r  = rec[k];
            uint32_t cs = t->code_size[(r >> 16) & 0x1FF];
            int      nv = (int)(r >> 25);
            fbw2_put(b, ((cs & 0xFFFFu) << nv) | (r & 0xFFFFu), (int)(cs >> 16) + nv);
        }
    }
}


/* -------------------------------------------------- wide bit writer (opt-emit2)
 * fbw_put drains 4 bytes and so caps one call at 32 bits; that is one Huffman
 * symbol plus its value bits, i.e. one accumulator round trip per COEFFICIENT.
 * fbw_putw drains every whole byte instead, which lifts the cap to 57 bits and
 * lets several symbols ride in one call. It is the same left-aligned store the
 * opt-entropy work measured 60% SLOWER -- but that measurement was at ONE symbol
 * per call, where the 8-byte store and the 64-bit SWAR are pure overhead. At 5
 * symbols per call (~28 bits) it wins. Invariant: nbits < 8 on entry, n <= 57.
 * Writes 8 bytes and advances by however many are whole, so the buffer needs 8
 * bytes of tail slack -- FBW_SLACK covers it. */
static inline void fbw_putw(fbw_t *b, uint64_t code, int n)
{
    b->acc = (b->acc << n) | code;
    b->nbits += n;
    int full = b->nbits >> 3;
    b->nbits &= 7;
    uint64_t v  = b->acc >> b->nbits;            /* `full` whole bytes, right-aligned */
    unsigned sh = (unsigned)(64 - (full << 3)) & 63;
    uint64_t w  = v << sh;                       /* stale high bits shifted out */
    uint64_t be = __builtin_bswap64(w);
    memcpy(b->p, &be, 8);                        /* always 8; p advances by `full` */
    uint64_t ff = ((~w) - 0x0101010101010101ULL) & w & 0x8080808080808080ULL;
    ff &= (uint64_t)0 - (uint64_t)(full != 0);   /* full==0: nothing was emitted */
    ff &= ~0ULL << sh;
    if (__builtin_expect(ff == 0, 1)) { b->p += full; return; }
    uint8_t *p = b->p;
    for (int i = 0; i < full; i++) {
        uint8_t byte = (uint8_t)(w >> 56); w <<= 8;
        *p++ = byte; if (byte == 0xFF) *p++ = 0x00;
    }
    b->p = p;
}

#ifdef __ARM_NEON
/* ------------------------------------------- per-coefficient symbol precompute
 * The emit loop's critical chain used to be  load coef -> clz -> size -> symbol
 * -> Huffman table load.  Both the size and the value bits depend only on the
 * coefficient, so NEON computes all 64 of each before the scan starts and the
 * chain loses two links.  Fused with the nonzero bitmap, which reads the same
 * registers, so it replaces nzmask() rather than adding to it.
 *   szv[i] = magcat_fast(c[i]),  vbv[i] = magbits(c[i], szv[i])
 * `danger` flags the one int16 value (-32768) whose 16-bit |v| would disagree
 * with the scalar 32-bit magcat; the caller then redoes the block scalar. */
static const uint16_t LJ12_BITW8[8] = {1,2,4,8,16,32,64,128};
static inline uint64_t lj12_mask_split(const int16x8_t z[8], uint16_t *szv,
                                       uint16_t *vbv, int *danger)
{
    const uint16x8_t bw = vld1q_u16(LJ12_BITW8);
    const int16x8_t  zero = vdupq_n_s16(0);
    const uint16x8_t k16 = vdupq_n_u16(16), ones = vdupq_n_u16(0xFFFF);
    int16x8_t mn = vdupq_n_s16(0);
    uint64_t m = 0;
    for (int k = 0; k < 8; k++) {
        int16x8_t v = z[k];
        mn = vminq_s16(mn, v);
        uint16x8_t nz = vmvnq_u16(vceqq_s16(v, zero));
        m |= (uint64_t)vaddvq_u16(vandq_u16(nz, bw)) << (k*8);
        uint16x8_t a   = vreinterpretq_u16_s16(vqabsq_s16(v));
        uint16x8_t sz  = vsubq_u16(k16, vclzq_u16(a));       /* 0 when v == 0 */
        int16x8_t  adj = vaddq_s16(v, vshrq_n_s16(v, 15));   /* v-1 if negative */
        uint16x8_t mk  = vmvnq_u16(vshlq_u16(ones, vreinterpretq_s16_u16(sz)));
        vst1q_u16(szv + k*8, sz);
        vst1q_u16(vbv + k*8, vandq_u16(vreinterpretq_u16_s16(adj), mk));
    }
    *danger = (vminvq_s16(mn) == (int16_t)-32768);
    return m;
}

#endif /* __ARM_NEON */

#endif /* FASTENT_H */
