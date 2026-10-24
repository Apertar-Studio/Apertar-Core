/* Compiled as C++: LANGUAGES CXX only, a .c file is silently dropped.
 *
 * 12-bit SOF1 JPEG encoder for lossy DNG tiles.
 * Format target measured from /media/RAW/BMD Example.dng (Blackmagic 3:1 CinemaDNG):
 *   SOI, DQT(Pq=1,16-bit), DHT x4, SOF1 prec=12 ncomp=2 H=V=1, SOS(comp0), SOS(comp1), EOI
 * Two non-interleaved scans, optimised Huffman tables per tile.
 *
 * Correctness first: the FDCT here is the textbook separable float transform.
 * The NEON integer path is a drop-in replacement for fdct_8x8() once this validates
 * (measured 784 Mpx/s/core in bench/dct_bench2.c). */
#include "LossyJpeg12.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "fastent.h"

/* MEASURED FROM THE BMD FILE, and kept because it is the reference the encoder
 * is validated against -- re-encoding BMD's own tiles with THIS table is what
 * reproduces their bytes to within one. It is no longer what we ENCODE with;
 * see LJ12_QTBL_CFA_G05 below. */
const uint16_t LJ12_QTBL_BMD_3TO1[64] = {   /* natural order, measured from the BMD file */
     6,  6,  7,  9, 12, 16, 21, 27,
     6,  8,  7,  8, 10, 13, 16, 21,
     7,  7, 10, 12, 13, 16, 20, 24,
     9,  8, 12, 15, 18, 21, 25, 29,
    12, 10, 13, 18, 23, 27, 32, 37,
    16, 13, 16, 21, 27, 34, 40, 48,
    21, 16, 20, 25, 32, 40, 50, 59,
    27, 21, 24, 29, 37, 48, 59, 63,
};

/* THE TABLE WE ENCODE WITH. BMD's shape with its high-frequency ramp SQUARE-
 * ROOTED: q[i] = round(q[0] * (BMD[i]/q[0])^0.5), so q[63] falls 63 -> 19 while
 * DC is unchanged.
 *
 * WHY BMD'S SHAPE WAS THE WRONG ONE TO INHERIT. It was adopted because it is
 * what the reference file uses, never because it suits this content. BMD's
 * table is shaped for a normal image; our components are CFA PHASE PLANES,
 * whose Nyquist is half the image's, so the table's steep HF ramp is coarsening
 * what are really the image's MID frequencies -- and each of the four phases is
 * quantised independently, so that error lands differently on R, G1, G2 and B
 * and shows up after demosaic as COLOUR at edges.
 *
 * MEASURED at MATCHED BYTES on a real 4K ClearHDR frame, each shape solved to
 * the same 4.147 MB 3:1 budget (chroma error is the (R-G)/(B-G) residual on the
 * top 1% of luma-gradient pixels; shadow blockDC is the 8x8 block-mean error in
 * the darkest quartile):
 *
 *     shape       edge chroma   RMSE all   shadow blockDC luma / chroma
 *     BMD            147.6        46.6           0.06 / 0.13
 *     flat            95.6        31.1           0.12 / 0.28
 *     gamma0.5       108.6        36.1           0.07 / 0.17   <- this
 *     gamma1.5       205.2        56.7           0.06 / 0.10
 *
 * -26% edge chroma error and -23% whole-frame RMSE for the SAME file size.
 *
 * WHY NOT FLAT, which is better still on both. Flat is MSE-optimal (as theory
 * requires for an orthonormal transform) and it is the shape this project has
 * already documented as the one that visibly BLOCKS IN SHADOWS -- the table
 * above shows why, at 2.0x the shadow block-DC luma error and 2.2x the chroma.
 * gamma0.5 takes most of the edge win for 1.2x/1.3x instead. The absolute
 * shadow penalty is small next to what it buys (0.04 against 39 on the same
 * 0..62335 scale), but block-DC artefacts are spatially correlated and visible
 * at low amplitude -- which is exactly the trap that metric exists to catch --
 * so the conservative rung is the right one to ship.
 *
 * EDGE HF RETENTION IS UNCHANGED at ~100% for every shape, which is why the
 * gradient metrics could not choose between them. The difference is not how
 * much edge energy survives; it is whether the four phases agree about it. */
const uint16_t LJ12_QTBL_CFA_G05[64] = {
     6,  6,  6,  7,  8, 10, 11, 13,
     6,  7,  6,  7,  8,  9, 10, 11,
     6,  6,  8,  8,  9, 10, 11, 12,
     7,  7,  8,  9, 10, 11, 12, 13,
     8,  8,  9, 10, 12, 13, 14, 15,
    10,  9, 10, 11, 13, 14, 15, 17,
    11, 10, 11, 12, 14, 15, 17, 19,
    13, 11, 12, 13, 15, 17, 19, 19,
};

/* Byte-level zigzag for vqtbl4q: output int16 i takes source bytes 2*ZZ[i], +1.
 * vqtbl4q_u8 returns 0 for indices >= 64, so subtracting 64 for the second half
 * makes the two lookups disjoint and they can simply be OR'd. */
static uint8_t BZZ[128];
static uint8_t BZZ2[128];   /* BZZ - 64, so the second vqtbl4q needs no subtract */
static int bzz_ready = 0;
static const uint8_t ZZ[64] = {
     0, 1, 8,16, 9, 2, 3,10, 17,24,32,25,18,11, 4, 5,
    12,19,26,33,40,48,41,34, 27,20,13, 6, 7,14,21,28,
    35,42,49,56,57,50,43,36, 29,22,15,23,30,37,44,51,
    58,59,52,45,38,31,39,46, 53,60,61,54,47,55,62,63
};

struct lj12_ctx {
    int16_t  *coef;  size_t coef_cap;    /* two components' quantised coefficients */
    uint64_t *mask;  size_t mask_cap;    /* per-block nonzero bitmap, built in quantise */
    uint8_t  *out;   size_t out_cap;     /* entropy output */
};
lj12_ctx *lj12_ctx_create(void) {
    lj12_ctx *c = static_cast<lj12_ctx*>(calloc(1, sizeof(lj12_ctx)));
    return c;
}
void lj12_ctx_destroy(lj12_ctx *c) { if (!c) return; free(c->coef); free(c->mask); free(c->out); free(c); }
static int ctx_reserve(lj12_ctx *c, size_t coef_bytes, size_t mask_bytes, size_t out_bytes) {
    if (mask_bytes > c->mask_cap) {
        void *p = realloc(c->mask, mask_bytes);
        if (!p) return -1;
        c->mask = (uint64_t*)p; c->mask_cap = mask_bytes; memset(c->mask, 0, mask_bytes);
    }
    if (coef_bytes > c->coef_cap) {
        void *p = realloc(c->coef, coef_bytes);
        if (!p) return -1;
        c->coef = (int16_t*)p; c->coef_cap = coef_bytes;
        memset(c->coef, 0, coef_bytes);        /* fault the pages in once, not per frame */
    }
    if (out_bytes > c->out_cap) {
        void *p = realloc(c->out, out_bytes);
        if (!p) return -1;
        c->out = (uint8_t*)p; c->out_cap = out_bytes;
        memset(c->out, 0, out_bytes);
    }
    return 0;
}

void lj12_scale_qtable(const uint16_t base[64], int scale_pct, uint16_t out[64]) {
    for (int i = 0; i < 64; i++) {
        long v = ((long)base[i] * scale_pct + 50) / 100;
        if (v < 1) v = 1;
        if (v > 65535) v = 65535;
        out[i] = (uint16_t)v;
    }
}

/* ---- FDCT: separable, float, textbook. Correct by construction. ---- */
static double COSTAB[8][8];
static int costab_ready = 0;
static void init_bzz(void) {
    if (bzz_ready) return;
    for (int i = 0; i < 64; i++) { BZZ[2*i] = (uint8_t)(2*ZZ[i]); BZZ[2*i+1] = (uint8_t)(2*ZZ[i]+1); }
    for (int i = 0; i < 128; i++) BZZ2[i] = (uint8_t)(BZZ[i] - 64);
    bzz_ready = 1;
}
static void init_costab(void) {
    if (costab_ready) return;
    for (int u = 0; u < 8; u++)
        for (int x = 0; x < 8; x++)
            COSTAB[u][x] = cos((2.0*x + 1.0) * u * M_PI / 16.0) * (u == 0 ? 0.353553390593273762 : 0.5);
    costab_ready = 1;
}
static void fdct_8x8(const int *in, double *out) {
    double tmp[64];
    for (int y = 0; y < 8; y++)
        for (int u = 0; u < 8; u++) {
            double s = 0;
            for (int x = 0; x < 8; x++) s += in[y*8+x] * COSTAB[u][x];
            tmp[y*8+u] = s;
        }
    for (int u = 0; u < 8; u++)
        for (int v = 0; v < 8; v++) {
            double s = 0;
            for (int y = 0; y < 8; y++) s += tmp[y*8+u] * COSTAB[v][y];
            out[v*8+u] = s;
        }
}


/* ---- Integer FDCT: libjpeg's "islow" (Loeffler-Ligtenberg-Moschytz).
 * Output is the TRUE DCT at 1:1 scale (verified: ratio 0.999993, RMS 0.305 vs
 * the float reference). libjpeg's (qval << 3) divisor belongs to its own
 * differently-scaled pipeline and must NOT be used here.
 * PASS1_BITS=1 for 12-bit input (the 8-bit build uses 2; the extra headroom
 * is spent on the 4 extra sample bits). ---- */
#define CONST_BITS 13
#define PASS1_BITS 1
#define FIX_0_298631336  2446
#define FIX_0_390180644  3196
#define FIX_0_541196100  4433
#define FIX_0_765366865  6270
#define FIX_0_899976223  7373
#define FIX_1_175875602  9633
#define FIX_1_501321110 12299
#define FIX_1_847759065 15137
#define FIX_1_961570560 16069
#define FIX_2_053119869 16819
#define FIX_2_562915447 20995
#define FIX_3_072711026 25172
#define DESCALE(x,n) (((x) + ((int32_t)1 << ((n)-1))) >> (n))

static void fdct_islow(int32_t *data) {
    int32_t tmp0,tmp1,tmp2,tmp3,tmp4,tmp5,tmp6,tmp7;
    int32_t tmp10,tmp11,tmp12,tmp13, z1,z2,z3,z4,z5;
    int32_t *dp = data;
    for (int c = 0; c < 8; c++) {            /* pass 1: rows */
        tmp0 = dp[0]+dp[7]; tmp7 = dp[0]-dp[7];
        tmp1 = dp[1]+dp[6]; tmp6 = dp[1]-dp[6];
        tmp2 = dp[2]+dp[5]; tmp5 = dp[2]-dp[5];
        tmp3 = dp[3]+dp[4]; tmp4 = dp[3]-dp[4];
        tmp10 = tmp0+tmp3; tmp13 = tmp0-tmp3;
        tmp11 = tmp1+tmp2; tmp12 = tmp1-tmp2;
        dp[0] = (tmp10+tmp11) << PASS1_BITS;
        dp[4] = (tmp10-tmp11) << PASS1_BITS;
        z1 = (tmp12+tmp13) * FIX_0_541196100;
        dp[2] = DESCALE(z1 + tmp13*FIX_0_765366865,  CONST_BITS-PASS1_BITS);
        dp[6] = DESCALE(z1 - tmp12*FIX_1_847759065,  CONST_BITS-PASS1_BITS);
        z1 = tmp4+tmp7; z2 = tmp5+tmp6; z3 = tmp4+tmp6; z4 = tmp5+tmp7;
        z5 = (z3+z4) * FIX_1_175875602;
        tmp4 *= FIX_0_298631336; tmp5 *= FIX_2_053119869;
        tmp6 *= FIX_3_072711026; tmp7 *= FIX_1_501321110;
        z1 *= -FIX_0_899976223;  z2 *= -FIX_2_562915447;
        z3 *= -FIX_1_961570560;  z4 *= -FIX_0_390180644;
        z3 += z5; z4 += z5;
        dp[7] = DESCALE(tmp4+z1+z3, CONST_BITS-PASS1_BITS);
        dp[5] = DESCALE(tmp5+z2+z4, CONST_BITS-PASS1_BITS);
        dp[3] = DESCALE(tmp6+z2+z3, CONST_BITS-PASS1_BITS);
        dp[1] = DESCALE(tmp7+z1+z4, CONST_BITS-PASS1_BITS);
        dp += 8;
    }
    dp = data;
    for (int c = 0; c < 8; c++) {            /* pass 2: columns */
        tmp0 = dp[8*0]+dp[8*7]; tmp7 = dp[8*0]-dp[8*7];
        tmp1 = dp[8*1]+dp[8*6]; tmp6 = dp[8*1]-dp[8*6];
        tmp2 = dp[8*2]+dp[8*5]; tmp5 = dp[8*2]-dp[8*5];
        tmp3 = dp[8*3]+dp[8*4]; tmp4 = dp[8*3]-dp[8*4];
        tmp10 = tmp0+tmp3; tmp13 = tmp0-tmp3;
        tmp11 = tmp1+tmp2; tmp12 = tmp1-tmp2;
        dp[8*0] = DESCALE(tmp10+tmp11, PASS1_BITS+3);
        dp[8*4] = DESCALE(tmp10-tmp11, PASS1_BITS+3);
        z1 = (tmp12+tmp13) * FIX_0_541196100;
        dp[8*2] = DESCALE(z1 + tmp13*FIX_0_765366865, CONST_BITS+PASS1_BITS+3);
        dp[8*6] = DESCALE(z1 - tmp12*FIX_1_847759065, CONST_BITS+PASS1_BITS+3);
        z1 = tmp4+tmp7; z2 = tmp5+tmp6; z3 = tmp4+tmp6; z4 = tmp5+tmp7;
        z5 = (z3+z4) * FIX_1_175875602;
        tmp4 *= FIX_0_298631336; tmp5 *= FIX_2_053119869;
        tmp6 *= FIX_3_072711026; tmp7 *= FIX_1_501321110;
        z1 *= -FIX_0_899976223;  z2 *= -FIX_2_562915447;
        z3 *= -FIX_1_961570560;  z4 *= -FIX_0_390180644;
        z3 += z5; z4 += z5;
        dp[8*7] = DESCALE(tmp4+z1+z3, CONST_BITS+PASS1_BITS+3);
        dp[8*5] = DESCALE(tmp5+z2+z4, CONST_BITS+PASS1_BITS+3);
        dp[8*3] = DESCALE(tmp6+z2+z3, CONST_BITS+PASS1_BITS+3);
        dp[8*1] = DESCALE(tmp7+z1+z4, CONST_BITS+PASS1_BITS+3);
        dp++;
    }
}


#if defined(__ARM_NEON)
#include <arm_neon.h>
/* ---------------------------------------------------------------------------
 * FUSED forward DCT + quantise + zigzag, register-resident end to end.
 *
 * Three memory round trips per block disappear:
 *   - the transpose no longer goes through the stack (4 x 4x4 TRN1/TRN2);
 *   - the DCT does not write 64 int32 for the quantiser to read back;
 *   - the quantiser does not write 64 int16 for the zigzag to read back.
 * The block enters as 16 vectors and leaves as 8 stores of the final
 * zigzag-ordered int16 coefficients.
 *
 * The quantiser collapses from seven instructions per four coefficients
 * (vabs, load qhalf, vadd, load recip, vmull_u32 x2, vshrn x2, vcombine) to
 * three (vabs, load recip, vqrdmulh), and one of the two tables disappears:
 *   floor((|d| + floor(q/2)) / q)  IS  round-half-up(|d|/q)  for all integers,
 *   and vqrdmulhq_s32(n, ceil(2^31/q))  IS  (n*ceil(2^31/q) + 2^30) >> 31,
 * which is that same round-half-up.  Exact for every q in [2,65535] over the
 * coefficient range -- |DCT coefficient| <= 16384 because the DC basis weights
 * sum to exactly 8 and 8*2048 = 16384.  Brute-forced in verify_recip2.c.  Note
 * this is a WIDER exact range than the 32-bit form it replaces, whose first
 * failure was q=1039; only q=1 now needs the scalar fallback.
 *
 * Arithmetic is otherwise unchanged: same FIX_ constants, same vrshrq DESCALE
 * points, same +q/2 magnitude rounding, same saturating narrow.  Output is
 * bit-identical to fdct_islow_neon() followed by the old quantise+zigzag.
 * --------------------------------------------------------------------------- */

#define T4_S32(a,b,c,d, A,B,C,D) do {                                          \
    int32x4_t x0_=vtrn1q_s32(a,b), x1_=vtrn2q_s32(a,b);                        \
    int32x4_t x2_=vtrn1q_s32(c,d), x3_=vtrn2q_s32(c,d);                        \
    int64x2_t y0_=vreinterpretq_s64_s32(x0_), y1_=vreinterpretq_s64_s32(x1_);  \
    int64x2_t y2_=vreinterpretq_s64_s32(x2_), y3_=vreinterpretq_s64_s32(x3_);  \
    A=vreinterpretq_s32_s64(vtrn1q_s64(y0_,y2_));                              \
    B=vreinterpretq_s32_s64(vtrn1q_s64(y1_,y3_));                              \
    C=vreinterpretq_s32_s64(vtrn2q_s64(y0_,y2_));                              \
    D=vreinterpretq_s32_s64(vtrn2q_s64(y1_,y3_));                              \
} while (0)

#define T8_S32(a0,a1,a2,a3,a4,a5,a6,a7, c0,c1,c2,c3,c4,c5,c6,c7) do {          \
    int32x4_t A0,A1,A2,A3,B0,B1,B2,B3,C0,C1,C2,C3,D0,D1,D2,D3;                 \
    T4_S32(a0,a1,a2,a3, A0,A1,A2,A3);                                          \
    T4_S32(a4,a5,a6,a7, B0,B1,B2,B3);                                          \
    T4_S32(c0,c1,c2,c3, C0,C1,C2,C3);                                          \
    T4_S32(c4,c5,c6,c7, D0,D1,D2,D3);                                          \
    a0=A0; a1=A1; a2=A2; a3=A3;   c0=B0; c1=B1; c2=B2; c3=B3;                  \
    a4=C0; a5=C1; a6=C2; a7=C3;   c4=D0; c5=D1; c6=D2; c7=D3;                  \
} while (0)

/* (m4 + y1) + y3  ==  (y1 + y3) + t4*FIX : int32 addition is associative modulo
 * 2^32, so folding the four odd-part multiplies into MLAs is bit-identical and
 * drops four instructions per pass -- and four live registers with them. */
#define ISLOW_ODD(t4,t5,t6,t7, o1,o3,o5,o7, SH) do {                           \
    int32x4_t y1=vaddq_s32(t4,t7), y2=vaddq_s32(t5,t6);                        \
    int32x4_t y3=vaddq_s32(t4,t6), y4=vaddq_s32(t5,t7);                        \
    int32x4_t y5=vmulq_n_s32(vaddq_s32(y3,y4), FIX_1_175875602);               \
    y1=vmulq_n_s32(y1,-FIX_0_899976223); y2=vmulq_n_s32(y2,-FIX_2_562915447);  \
    y3=vmulq_n_s32(y3,-FIX_1_961570560); y4=vmulq_n_s32(y4,-FIX_0_390180644);  \
    y3=vaddq_s32(y3,y5); y4=vaddq_s32(y4,y5);                                  \
    o7=vrshrq_n_s32(vmlaq_n_s32(vaddq_s32(y1,y3), t4, FIX_0_298631336), SH);   \
    o5=vrshrq_n_s32(vmlaq_n_s32(vaddq_s32(y2,y4), t5, FIX_2_053119869), SH);   \
    o3=vrshrq_n_s32(vmlaq_n_s32(vaddq_s32(y2,y3), t6, FIX_3_072711026), SH);   \
    o1=vrshrq_n_s32(vmlaq_n_s32(vaddq_s32(y1,y4), t7, FIX_1_501321110), SH);   \
} while (0)

#define ISLOW_PASS1(d0,d1,d2,d3,d4,d5,d6,d7) do {                              \
    int32x4_t t0=vaddq_s32(d0,d7), t7=vsubq_s32(d0,d7);                        \
    int32x4_t t1=vaddq_s32(d1,d6), t6=vsubq_s32(d1,d6);                        \
    int32x4_t t2=vaddq_s32(d2,d5), t5=vsubq_s32(d2,d5);                        \
    int32x4_t t3=vaddq_s32(d3,d4), t4=vsubq_s32(d3,d4);                        \
    int32x4_t ta=vaddq_s32(t0,t3), td=vsubq_s32(t0,t3);                        \
    int32x4_t tb=vaddq_s32(t1,t2), tc=vsubq_s32(t1,t2);                        \
    int32x4_t z1=vmulq_n_s32(vaddq_s32(tc,td), FIX_0_541196100);               \
    d0=vshlq_n_s32(vaddq_s32(ta,tb), PASS1_BITS);                              \
    d4=vshlq_n_s32(vsubq_s32(ta,tb), PASS1_BITS);                              \
    d2=vrshrq_n_s32(vmlaq_n_s32(z1,td,FIX_0_765366865), CONST_BITS-PASS1_BITS);\
    d6=vrshrq_n_s32(vmlsq_n_s32(z1,tc,FIX_1_847759065), CONST_BITS-PASS1_BITS);\
    ISLOW_ODD(t4,t5,t6,t7, d1,d3,d5,d7, CONST_BITS-PASS1_BITS);                \
} while (0)

#define ISLOW_PASS2(d0,d1,d2,d3,d4,d5,d6,d7) do {                              \
    int32x4_t t0=vaddq_s32(d0,d7), t7=vsubq_s32(d0,d7);                        \
    int32x4_t t1=vaddq_s32(d1,d6), t6=vsubq_s32(d1,d6);                        \
    int32x4_t t2=vaddq_s32(d2,d5), t5=vsubq_s32(d2,d5);                        \
    int32x4_t t3=vaddq_s32(d3,d4), t4=vsubq_s32(d3,d4);                        \
    int32x4_t ta=vaddq_s32(t0,t3), td=vsubq_s32(t0,t3);                        \
    int32x4_t tb=vaddq_s32(t1,t2), tc=vsubq_s32(t1,t2);                        \
    int32x4_t z1=vmulq_n_s32(vaddq_s32(tc,td), FIX_0_541196100);               \
    d0=vrshrq_n_s32(vaddq_s32(ta,tb), PASS1_BITS+3);                           \
    d4=vrshrq_n_s32(vsubq_s32(ta,tb), PASS1_BITS+3);                           \
    d2=vrshrq_n_s32(vmlaq_n_s32(z1,td,FIX_0_765366865), CONST_BITS+PASS1_BITS+3);\
    d6=vrshrq_n_s32(vmlsq_n_s32(z1,tc,FIX_1_847759065), CONST_BITS+PASS1_BITS+3);\
    ISLOW_ODD(t4,t5,t6,t7, d1,d3,d5,d7, CONST_BITS+PASS1_BITS+3);              \
} while (0)

/* One quantised int16x8 row: lo = natural coefficients 8v..8v+3, hi = 8v+4..8v+7.
 *
 * The reference computes floor((|d| + floor(q/2)) / q).  For every integer n and
 * q that is exactly round-half-up(n/q), and vqrdmulhq_s32(n,R) with
 * R = ceil(2^31/q) IS (n*R + 2^30) >> 31, i.e. round-half-up(n/q).  So the whole
 * qhalf table -- 16 loads and 16 adds per block -- is not needed at all.  Exact
 * for q in [2,65535] over the coefficient range, brute-forced in verify_recip2.c.
 * The five-instruction 64-bit form (vmull_u32 x2, vshrn x2, vcombine) it replaces
 * was itself only exact to q=1038. */
#define QROW(lo,hi,idx,r31) ({                                              \
    int32x4_t dl_=lo, dh_=hi;                                                  \
    int32x4_t ml_=vqrdmulhq_s32(vabsq_s32(dl_), vld1q_s32((r31)+(idx)));       \
    int32x4_t mh_=vqrdmulhq_s32(vabsq_s32(dh_), vld1q_s32((r31)+(idx)+4));     \
    int32x4_t zl_=vdupq_n_s32(0);                                              \
    ml_=vbslq_s32(vcltq_s32(dl_,zl_), vnegq_s32(ml_), ml_);                    \
    mh_=vbslq_s32(vcltq_s32(dh_,zl_), vnegq_s32(mh_), mh_);                    \
    vcombine_s16(vqmovn_s32(ml_), vqmovn_s32(mh_));                            \
})

/* pre_t != 0: b[] arrives transposed (b[x*8+y] = sample(y,x)) */
__attribute__((always_inline))
static inline void fdct_qz_core(const int32_t *b, const int32_t *r31,
                                int16x8_t z[8], int pre_t)
{
    int32x4_t a0=vld1q_s32(b+0*8), c0=vld1q_s32(b+0*8+4);
    int32x4_t a1=vld1q_s32(b+1*8), c1=vld1q_s32(b+1*8+4);
    int32x4_t a2=vld1q_s32(b+2*8), c2=vld1q_s32(b+2*8+4);
    int32x4_t a3=vld1q_s32(b+3*8), c3=vld1q_s32(b+3*8+4);
    int32x4_t a4=vld1q_s32(b+4*8), c4=vld1q_s32(b+4*8+4);
    int32x4_t a5=vld1q_s32(b+5*8), c5=vld1q_s32(b+5*8+4);
    int32x4_t a6=vld1q_s32(b+6*8), c6=vld1q_s32(b+6*8+4);
    int32x4_t a7=vld1q_s32(b+7*8), c7=vld1q_s32(b+7*8+4);
    if (!pre_t) T8_S32(a0,a1,a2,a3,a4,a5,a6,a7, c0,c1,c2,c3,c4,c5,c6,c7);
    ISLOW_PASS1(a0,a1,a2,a3,a4,a5,a6,a7);
    ISLOW_PASS1(c0,c1,c2,c3,c4,c5,c6,c7);
    T8_S32(a0,a1,a2,a3,a4,a5,a6,a7, c0,c1,c2,c3,c4,c5,c6,c7);
    ISLOW_PASS2(a0,a1,a2,a3,a4,a5,a6,a7);
    ISLOW_PASS2(c0,c1,c2,c3,c4,c5,c6,c7);
    /* GCC re-materialised both TBL4 source quads on every one of the eight
     * zigzag steps -- eight MOVs each, 64 per block, 17% of the kernel.  TBL4
     * needs four CONSECUTIVE registers; going through a 128-byte stack buffer
     * and two LD1 {v..-v..} loads establishes that once, in two instructions. */
    int16_t zbuf_[64];
    vst1q_s16(zbuf_+0, QROW(a0,c0,0,r31));
    vst1q_s16(zbuf_+8, QROW(a1,c1,8,r31));
    vst1q_s16(zbuf_+16, QROW(a2,c2,16,r31));
    vst1q_s16(zbuf_+24, QROW(a3,c3,24,r31));
    vst1q_s16(zbuf_+32, QROW(a4,c4,32,r31));
    vst1q_s16(zbuf_+40, QROW(a5,c5,40,r31));
    vst1q_s16(zbuf_+48, QROW(a6,c6,48,r31));
    vst1q_s16(zbuf_+56, QROW(a7,c7,56,r31));
    uint8x16x4_t lo_ = vld1q_u8_x4((const uint8_t*)zbuf_);
    uint8x16x4_t hi_ = vld1q_u8_x4((const uint8_t*)zbuf_ + 64);    for (int k = 0; k < 8; k++) {                    /* leave them in registers */
        z[k] = vreinterpretq_s16_u8(vorrq_u8(vqtbl4q_u8(lo_, vld1q_u8(BZZ  + k*16)),
                                             vqtbl4q_u8(hi_, vld1q_u8(BZZ2 + k*16))));
    }
}
static void fdct_qz_z (const int32_t *b, const int32_t *r31, int16x8_t z[8])
{ fdct_qz_core(b,r31,z,0); }
static void fdct_qz_zT(const int32_t *b, const int32_t *r31, int16x8_t z[8])
{ fdct_qz_core(b,r31,z,1); }
/* buffered path (collect_blocks) still wants them in the coefficient array */
static void fdct_qz(const int32_t *b, const int32_t *r31, int16_t *out)
{ int16x8_t z[8]; fdct_qz_core(b,r31,z,0); for (int k=0;k<8;k++) vst1q_s16(out+k*8, z[k]); }

/* ---------------------------------------------------------------------------
 * fp32 AAN forward DCT, sharing the fused structure above: same one transpose,
 * same register-resident path, same eight zigzag registers handed to
 * lj12_emit_block.  NOT bit-exact against Blackmagic -- but MORE accurate than
 * islow, not less: rms error against a double-precision DCT is 0.0001 where
 * islow's is 0.3004.
 *
 * Why it is faster than the integer kernel it replaces:
 *   - AAN needs 5 multiplies per 8-point transform where islow needs 12, and
 *     two of those five are FMLA;
 *   - fp32 FMLA is 2/cycle on Cortex-A76 where a Q-form integer multiply is
 *     1/cycle;
 *   - quantisation becomes vmulq_f32 + vcvtaq_s32_f32 -- FCVTAS rounds to
 *     nearest with ties away from zero, which is exactly what the integer
 *     path's +floor(q/2) was computing -- so the abs/negate/select sequence
 *     disappears along with its two instructions per four coefficients.
 *   - the AAN scale factors 1/(8*S_u*S_v) fold into that one reciprocal table.
 *     The DQT written into the file is still the true quantiser; a decoder
 *     sees nothing unusual.
 * --------------------------------------------------------------------------- */
#define T4_F32(a,b,c,d, A,B,C,D) do {                                          \
    float32x4_t x0_=vtrn1q_f32(a,b), x1_=vtrn2q_f32(a,b);                      \
    float32x4_t x2_=vtrn1q_f32(c,d), x3_=vtrn2q_f32(c,d);                      \
    float64x2_t y0_=vreinterpretq_f64_f32(x0_), y1_=vreinterpretq_f64_f32(x1_);\
    float64x2_t y2_=vreinterpretq_f64_f32(x2_), y3_=vreinterpretq_f64_f32(x3_);\
    A=vreinterpretq_f32_f64(vtrn1q_f64(y0_,y2_));                              \
    B=vreinterpretq_f32_f64(vtrn1q_f64(y1_,y3_));                              \
    C=vreinterpretq_f32_f64(vtrn2q_f64(y0_,y2_));                              \
    D=vreinterpretq_f32_f64(vtrn2q_f64(y1_,y3_));                              \
} while (0)
#define T8_F32(a0,a1,a2,a3,a4,a5,a6,a7, c0,c1,c2,c3,c4,c5,c6,c7) do {          \
    float32x4_t A0,A1,A2,A3,B0,B1,B2,B3,C0,C1,C2,C3,D0,D1,D2,D3;               \
    T4_F32(a0,a1,a2,a3, A0,A1,A2,A3); T4_F32(a4,a5,a6,a7, B0,B1,B2,B3);        \
    T4_F32(c0,c1,c2,c3, C0,C1,C2,C3); T4_F32(c4,c5,c6,c7, D0,D1,D2,D3);        \
    a0=A0;a1=A1;a2=A2;a3=A3; c0=B0;c1=B1;c2=B2;c3=B3;                          \
    a4=C0;a5=C1;a6=C2;a7=C3; c4=D0;c5=D1;c6=D2;c7=D3;                          \
} while (0)
#define AAN_PASS(d0,d1,d2,d3,d4,d5,d6,d7) do {                                 \
    float32x4_t t0=vaddq_f32(d0,d7), t7=vsubq_f32(d0,d7);                      \
    float32x4_t t1=vaddq_f32(d1,d6), t6=vsubq_f32(d1,d6);                      \
    float32x4_t t2=vaddq_f32(d2,d5), t5=vsubq_f32(d2,d5);                      \
    float32x4_t t3=vaddq_f32(d3,d4), t4=vsubq_f32(d3,d4);                      \
    float32x4_t ta=vaddq_f32(t0,t3), td=vsubq_f32(t0,t3);                      \
    float32x4_t tb=vaddq_f32(t1,t2), tc=vsubq_f32(t1,t2);                      \
    d0=vaddq_f32(ta,tb); d4=vsubq_f32(ta,tb);                                  \
    float32x4_t z1=vmulq_n_f32(vaddq_f32(tc,td), 0.707106781186547524f);       \
    d2=vaddq_f32(td,z1); d6=vsubq_f32(td,z1);                                  \
    float32x4_t u0=vaddq_f32(t4,t5), u1=vaddq_f32(t5,t6), u2=vaddq_f32(t6,t7); \
    float32x4_t z5=vmulq_n_f32(vsubq_f32(u0,u2), 0.382683432365089772f);       \
    float32x4_t z2=vfmaq_n_f32(z5,u0, 0.541196100146196984f);                  \
    float32x4_t z4=vfmaq_n_f32(z5,u2, 1.306562964876376527f);                  \
    float32x4_t z3=vmulq_n_f32(u1, 0.707106781186547524f);                     \
    float32x4_t z11=vaddq_f32(t7,z3), z13=vsubq_f32(t7,z3);                    \
    d5=vaddq_f32(z13,z2); d3=vsubq_f32(z13,z2);                                \
    d1=vaddq_f32(z11,z4); d7=vsubq_f32(z11,z4);                                \
} while (0)

/* inv[i] = 1 / (8 * S_u * S_v * q[i]) in NATURAL order, S_k = cos(k*pi/16)*sqrt2 */
/* Zero threshold, in quantiser steps: the magnitude below which a coefficient
 * is forced to zero.  Anything <= 0.5 is INERT -- it reproduces today's
 * round-half-away-from-zero bit for bit, which is how the change was verified.
 *
 * Measured, 4K RAW12 ISO800, 4 workers, CPU time, against the same frame and
 * the same quant table (scale 169):
 *     0.4999 (off)  4139937 B   block-DC err 2.8914   LF(zz 1-5) 3.3722
 *     0.90          3870468 B   block-DC err 2.8914   LF(zz 1-5) 3.3722
 * i.e. -6.5% bytes and +5% encode speed with the block mean and the low
 * frequencies bit-exactly unchanged; the added error is all in zigzag rank 6
 * and above, which on raw sensor content at 3:1 is overwhelmingly photon
 * noise.  The baseline cannot buy those bytes any other way: reaching
 * 3.87 MB by coarsening the table instead costs block-DC error 2.89 -> 3.46.
 *
 * LJ12_ZTHR_FROM is why DC and LF are untouched -- below that zigzag rank the
 * threshold is left inert, which is the same "protect the low frequencies
 * harder" principle the quant-table shaping already uses.  Going from rank 6
 * down to rank 1 buys only 0.17% more bytes and costs LF 3.3722 -> 3.7606.
 *
 * ====================================================================
 * TURNED OFF 2026-09-15. THE CASE ABOVE IS SOUND AND MEASURES THE WRONG THING.
 *
 * Every quality figure quoted above is block-DC error or LF (zigzag 1-5) error.
 * The threshold only touches rank 6 and above, so it CANNOT move either of them
 * -- the note even says they are bit-exactly unchanged, and treats that as
 * evidence of harmlessness. It is evidence of nothing: those two metrics are
 * blind to this lever by construction. It is the same trap the quant-table
 * shaping and the NR blotching both hit -- the metric being optimised was not
 * the one the eye reads.
 *
 * WHAT IT ACTUALLY COSTS, measured on the SAME frame at MATCHED BYTES (4K RAW12
 * ISO800, uncompressed reference re-encoded through this encoder, gradient
 * retention on the G1 phase by local-contrast class):
 *
 *                              MB    trueRMS   flat    mid   edge   EDGE
 *     0.90, scale 169        3.50     13.07   93.9%  67.7%  90.9%  99.5%
 *     off,  scale 205        3.53     11.96  103.4%  98.2%  99.0% 101.5%
 *
 * Same file size. Turning it off recovers THIRTY POINTS of mid-contrast detail
 * and LOWERS true error. It is not a rate/quality trade -- it is a pure loss.
 * For scale, QRAW Lossless scores 99.5% on that mid band, so 3:1 with the
 * threshold off sits at the Lossless level and with it on it does not.
 *
 * WHY IT LOSES. The band it kills is [0.5q, 0.9q] at rank >= 6: coefficients
 * too small to be edges and too large to be noise, which is exactly where
 * mid-contrast texture lives -- fabric, foliage, skin. Strong edges are far
 * above q and survive either way, which is why the failure reads as "soft" or
 * "mushy" rather than as visible artefacts, and why no error metric anchored to
 * block means or to the noise floor would ever flag it.
 *
 * "on raw sensor content at 3:1 [rank >= 6] is overwhelmingly photon noise" is
 * the load-bearing assumption and it is false at these rates. At 3:1 this codec
 * is spending 4.0 bits/pixel; there is real signal up there.
 *
 * KEPT AS A LEVER rather than deleted, because the byte and speed figures above
 * are real and a future rung far coarser than 5:1 might want them. Anything
 * <= 0.5 is inert. Re-enable only with a quality-at-matched-bytes measurement,
 * never with a bytes-at-matched-scale one.
 * ==================================================================== */
#ifndef LJ12_ZTHR
#define LJ12_ZTHR 0.4999f   /* inert: reproduces round-half-away-from-zero */
#endif
#ifndef LJ12_ZTHR_FROM          /* lowest zigzag rank the threshold may touch */
#define LJ12_ZTHR_FROM 6
#endif
static void aan_inv_table(const uint16_t qnat[64], float inv[128]) {
    static double S[8]; static int ready = 0;
    if (!ready) { for (int k = 0; k < 8; k++) S[k] = (k==0)?1.0:(cos(k*M_PI/16.0)*sqrt(2.0)); ready = 1; }
    for (int v = 0; v < 8; v++) for (int u = 0; u < 8; u++)
        inv[v*8+u] = (float)(1.0 / (8.0 * S[u] * S[v] * (double)qnat[v*8+u]));
    /* Per-position zero threshold, in the same natural order as the
     * reciprocals so QROW_F loads it with the same index. */
    for (int i = 0; i < 64; i++) inv[64+i] = 0.4999f;
    for (int r = LJ12_ZTHR_FROM; r < 64; r++) inv[64+ZZ[r]] = (float)(LJ12_ZTHR);
}

/* vcvtaq_s32_f32 is round-half-away-from-zero, so every coefficient that
 * reaches 0.5 quantiser steps becomes a non-zero symbol.  On raw sensor
 * content at 3:1 that leaves ~48 of 64 coefficients non-zero per block --
 * run-length coding has nothing to run over, and the entropy stage is 60% of
 * this encoder (measured by stubbing the emit: 49.1 -> 19.5 ms/frame/worker).
 * vcagtq_f32 (|a| > |b|) against the per-position threshold zeroes the
 * isolated +-1s and leaves the rounding of every coefficient it keeps exact.
 *
 * This is deliberately NOT a dead zone.  A dead zone shifts the rounding
 * offset, so it also biases the coefficients it keeps; measured across a
 * 0.1-0.5 sweep it was RD-worse than the baseline in EVERY band at matched
 * bytes, block DC included (at 3.88 MB: DC 4.13 against the baseline's 3.46).
 * Cost here: one FACGT and one AND per four lanes. */
#define QROW_F(lo,hi,idx,inv) ({                                               \
    float32x4_t tl_=vmulq_f32(lo, vld1q_f32((inv)+(idx)));                     \
    float32x4_t th_=vmulq_f32(hi, vld1q_f32((inv)+(idx)+4));                   \
    int32x4_t ql_=vcvtaq_s32_f32(tl_);                                         \
    int32x4_t qh_=vcvtaq_s32_f32(th_);                                         \
    ql_=vandq_s32(ql_,vreinterpretq_s32_u32(vcagtq_f32(tl_,vld1q_f32((inv)+64+(idx)))));  \
    qh_=vandq_s32(qh_,vreinterpretq_s32_u32(vcagtq_f32(th_,vld1q_f32((inv)+68+(idx)))));  \
    vcombine_s16(vqmovn_s32(ql_), vqmovn_s32(qh_));                            \
})

__attribute__((always_inline))
static inline void aan_qz_core(const int32_t *b, const float *inv, int16x8_t z[8], int pre_t)
{
    float32x4_t a0=vcvtq_f32_s32(vld1q_s32(b+0*8)), c0=vcvtq_f32_s32(vld1q_s32(b+0*8+4));
    float32x4_t a1=vcvtq_f32_s32(vld1q_s32(b+1*8)), c1=vcvtq_f32_s32(vld1q_s32(b+1*8+4));
    float32x4_t a2=vcvtq_f32_s32(vld1q_s32(b+2*8)), c2=vcvtq_f32_s32(vld1q_s32(b+2*8+4));
    float32x4_t a3=vcvtq_f32_s32(vld1q_s32(b+3*8)), c3=vcvtq_f32_s32(vld1q_s32(b+3*8+4));
    float32x4_t a4=vcvtq_f32_s32(vld1q_s32(b+4*8)), c4=vcvtq_f32_s32(vld1q_s32(b+4*8+4));
    float32x4_t a5=vcvtq_f32_s32(vld1q_s32(b+5*8)), c5=vcvtq_f32_s32(vld1q_s32(b+5*8+4));
    float32x4_t a6=vcvtq_f32_s32(vld1q_s32(b+6*8)), c6=vcvtq_f32_s32(vld1q_s32(b+6*8+4));
    float32x4_t a7=vcvtq_f32_s32(vld1q_s32(b+7*8)), c7=vcvtq_f32_s32(vld1q_s32(b+7*8+4));
    if (!pre_t) T8_F32(a0,a1,a2,a3,a4,a5,a6,a7, c0,c1,c2,c3,c4,c5,c6,c7);
    AAN_PASS(a0,a1,a2,a3,a4,a5,a6,a7);
    AAN_PASS(c0,c1,c2,c3,c4,c5,c6,c7);
    /* Transpose straight into the SECOND pass's variables.  T8_F32 computed its
     * result in sixteen temporaries and then assigned them back over a0..c7; the
     * register allocator cannot always coalesce that and it was costing sixteen
     * MOVs per block.  Renaming at the call site is free.
     * After the transpose rows 0..3 are (A|B) and rows 4..7 are (C|D), so the
     * low-lane column set is A0..A3,C0..C3 and the high-lane set is B,D. */
    float32x4_t A0,A1,A2,A3,B0,B1,B2,B3,C0,C1,C2,C3,D0,D1,D2,D3;
    T4_F32(a0,a1,a2,a3, A0,A1,A2,A3);
    T4_F32(a4,a5,a6,a7, B0,B1,B2,B3);
    T4_F32(c0,c1,c2,c3, C0,C1,C2,C3);
    T4_F32(c4,c5,c6,c7, D0,D1,D2,D3);
    AAN_PASS(A0,A1,A2,A3, C0,C1,C2,C3);
    AAN_PASS(B0,B1,B2,B3, D0,D1,D2,D3);
    /* GCC re-materialised both TBL4 source quads on every one of the eight
     * zigzag steps -- eight MOVs each, 64 per block, 17% of the kernel.  TBL4
     * needs four CONSECUTIVE registers; going through a 128-byte stack buffer
     * and two LD1 {v..-v..} loads establishes that once, in two instructions. */
    int16_t zbuf_[64];
    vst1q_s16(zbuf_+0, QROW_F(A0,B0, 0,inv));
    vst1q_s16(zbuf_+8, QROW_F(A1,B1, 8,inv));
    vst1q_s16(zbuf_+16, QROW_F(A2,B2,16,inv));
    vst1q_s16(zbuf_+24, QROW_F(A3,B3,24,inv));
    vst1q_s16(zbuf_+32, QROW_F(C0,D0,32,inv));
    vst1q_s16(zbuf_+40, QROW_F(C1,D1,40,inv));
    vst1q_s16(zbuf_+48, QROW_F(C2,D2,48,inv));
    vst1q_s16(zbuf_+56, QROW_F(C3,D3,56,inv));
    uint8x16x4_t lo_ = vld1q_u8_x4((const uint8_t*)zbuf_);
    uint8x16x4_t hi_ = vld1q_u8_x4((const uint8_t*)zbuf_ + 64);    for (int k = 0; k < 8; k++) {                    /* leave them in registers */
        z[k] = vreinterpretq_s16_u8(vorrq_u8(vqtbl4q_u8(lo_, vld1q_u8(BZZ  + k*16)),
                                             vqtbl4q_u8(hi_, vld1q_u8(BZZ2 + k*16))));
    }
}
static void aan_qz_z (const int32_t *b, const float *inv, int16x8_t z[8])
{ aan_qz_core(b,inv,z,0); }
static void aan_qz_zT(const int32_t *b, const float *inv, int16x8_t z[8])
{ aan_qz_core(b,inv,z,1); }
static void aan_qz   (const int32_t *b, const float *inv, int16_t *out)
{ int16x8_t z[8]; aan_qz_core(b,inv,z,0); for (int k=0;k<8;k++) vst1q_s16(out+k*8, z[k]); }


/* plain DCT (natural-order output) for the q=1 fallback path */
static void fdct_islow_T2(int32_t *b)
{
    int32x4_t a0=vld1q_s32(b+0*8), c0=vld1q_s32(b+0*8+4);
    int32x4_t a1=vld1q_s32(b+1*8), c1=vld1q_s32(b+1*8+4);
    int32x4_t a2=vld1q_s32(b+2*8), c2=vld1q_s32(b+2*8+4);
    int32x4_t a3=vld1q_s32(b+3*8), c3=vld1q_s32(b+3*8+4);
    int32x4_t a4=vld1q_s32(b+4*8), c4=vld1q_s32(b+4*8+4);
    int32x4_t a5=vld1q_s32(b+5*8), c5=vld1q_s32(b+5*8+4);
    int32x4_t a6=vld1q_s32(b+6*8), c6=vld1q_s32(b+6*8+4);
    int32x4_t a7=vld1q_s32(b+7*8), c7=vld1q_s32(b+7*8+4);
    ISLOW_PASS1(a0,a1,a2,a3,a4,a5,a6,a7);
    ISLOW_PASS1(c0,c1,c2,c3,c4,c5,c6,c7);
    T8_S32(a0,a1,a2,a3,a4,a5,a6,a7, c0,c1,c2,c3,c4,c5,c6,c7);
    ISLOW_PASS2(a0,a1,a2,a3,a4,a5,a6,a7);
    ISLOW_PASS2(c0,c1,c2,c3,c4,c5,c6,c7);
    vst1q_s32(b+0*8,a0); vst1q_s32(b+0*8+4,c0);
    vst1q_s32(b+1*8,a1); vst1q_s32(b+1*8+4,c1);
    vst1q_s32(b+2*8,a2); vst1q_s32(b+2*8+4,c2);
    vst1q_s32(b+3*8,a3); vst1q_s32(b+3*8+4,c3);
    vst1q_s32(b+4*8,a4); vst1q_s32(b+4*8+4,c4);
    vst1q_s32(b+5*8,a5); vst1q_s32(b+5*8+4,c5);
    vst1q_s32(b+6*8,a6); vst1q_s32(b+6*8+4,c6);
    vst1q_s32(b+7*8,a7); vst1q_s32(b+7*8+4,c7);
}
static void fdct_islow_v2(int32_t *b)
{
    int32x4_t a0=vld1q_s32(b+0*8), c0=vld1q_s32(b+0*8+4);
    int32x4_t a1=vld1q_s32(b+1*8), c1=vld1q_s32(b+1*8+4);
    int32x4_t a2=vld1q_s32(b+2*8), c2=vld1q_s32(b+2*8+4);
    int32x4_t a3=vld1q_s32(b+3*8), c3=vld1q_s32(b+3*8+4);
    int32x4_t a4=vld1q_s32(b+4*8), c4=vld1q_s32(b+4*8+4);
    int32x4_t a5=vld1q_s32(b+5*8), c5=vld1q_s32(b+5*8+4);
    int32x4_t a6=vld1q_s32(b+6*8), c6=vld1q_s32(b+6*8+4);
    int32x4_t a7=vld1q_s32(b+7*8), c7=vld1q_s32(b+7*8+4);
    T8_S32(a0,a1,a2,a3,a4,a5,a6,a7, c0,c1,c2,c3,c4,c5,c6,c7);
    ISLOW_PASS1(a0,a1,a2,a3,a4,a5,a6,a7);
    ISLOW_PASS1(c0,c1,c2,c3,c4,c5,c6,c7);
    T8_S32(a0,a1,a2,a3,a4,a5,a6,a7, c0,c1,c2,c3,c4,c5,c6,c7);
    ISLOW_PASS2(a0,a1,a2,a3,a4,a5,a6,a7);
    ISLOW_PASS2(c0,c1,c2,c3,c4,c5,c6,c7);
    vst1q_s32(b+0*8,a0); vst1q_s32(b+0*8+4,c0);
    vst1q_s32(b+1*8,a1); vst1q_s32(b+1*8+4,c1);
    vst1q_s32(b+2*8,a2); vst1q_s32(b+2*8+4,c2);
    vst1q_s32(b+3*8,a3); vst1q_s32(b+3*8+4,c3);
    vst1q_s32(b+4*8,a4); vst1q_s32(b+4*8+4,c4);
    vst1q_s32(b+5*8,a5); vst1q_s32(b+5*8+4,c5);
    vst1q_s32(b+6*8,a6); vst1q_s32(b+6*8+4,c6);
    vst1q_s32(b+7*8,a7); vst1q_s32(b+7*8+4,c7);
}

#endif

/* ---- Huffman table construction (JPEG Annex K.2) ---- */
typedef struct {
    uint8_t  bits[17];
    uint8_t  huffval[256];
    uint16_t code[256];
    uint8_t  size[256];
    /* Merged (size<<16)|code. The emit loop reads one uint32 per symbol instead
     * of touching code[] and size[] separately -- two arrays, two cache lines,
     * per coefficient. */
    uint32_t cs[256];
} huff_t;

struct lj12_tables { huff_t dc0, ac0, dc1, ac1; int valid; };
lj12_tables *lj12_tables_create(void){ return static_cast<lj12_tables*>(calloc(1,sizeof(lj12_tables))); }
void lj12_tables_destroy(lj12_tables *t){ free(t); }
void lj12_tables_invalidate(lj12_tables *t){ if(t) t->valid = 0; }
int  lj12_tables_valid(const lj12_tables *t){ return t && t->valid; }

static void huff_build(const long freq_in[257], huff_t *h) {
    long freq[257]; int codesize[257]; int others[257];
    for (int i = 0; i < 257; i++) { freq[i] = freq_in[i]; codesize[i] = 0; others[i] = -1; }
    freq[256] = 1;                               /* reserved slot, guarantees no all-ones code */
    for (;;) {
        long v1 = -1, v2 = -1; long f1 = 0, f2 = 0;
        for (int i = 0; i < 257; i++) if (freq[i]) {
            if (v1 < 0 || freq[i] < f1 || (freq[i] == f1 && i > v1)) { v2=v1; f2=f1; v1=i; f1=freq[i]; }
            else if (v2 < 0 || freq[i] < f2 || (freq[i] == f2 && i > v2)) { v2=i; f2=freq[i]; }
        }
        if (v2 < 0) break;
        freq[v1] += freq[v2]; freq[v2] = 0;
        codesize[v1]++;
        while (others[v1] >= 0) { v1 = others[v1]; codesize[v1]++; }
        others[v1] = v2;
        codesize[v2]++;
        while (others[v2] >= 0) { v2 = others[v2]; codesize[v2]++; }
    }
    /* bits[] must span the DEEPEST code the tree actually produced, not a presumed
     * 32. Highly skewed frequencies (a seeded symbol at freq 1 beside one at 1e6)
     * push package-merge past 32 bits; starting the limiting walk at 32 then leaves
     * those entries in place and libjpeg rejects the table as over-subscribed
     * ("Bogus Huffman table definition"). */
    int maxlen = 0;
    for (int i = 0; i < 257; i++) if (codesize[i] > maxlen) maxlen = codesize[i];
    if (maxlen > 256) maxlen = 256;
    int bits[258]; memset(bits, 0, sizeof bits);
    for (int i = 0; i < 257; i++) if (codesize[i]) bits[codesize[i]]++;
    for (int i = maxlen; i > 16; i--) {
        while (bits[i] > 0) {
            int j = i - 2;
            while (bits[j] == 0) j--;
            bits[i]   -= 2;
            bits[i-1] += 1;
            bits[j+1] += 2;
            bits[j]   -= 1;
        }
    }
    int k = 16; while (k > 0 && bits[k] == 0) k--;
    if (k > 0) bits[k]--;                        /* remove the reserved code */
    memset(h->bits, 0, sizeof h->bits);
    for (int i = 1; i <= 16; i++) h->bits[i] = (uint8_t)bits[i];
    /* Collect huffval in ORIGINAL codesize order, walking to 32 not 16: the
     * length-limiting pass above rewrites bits[] counts but leaves each symbol's
     * codesize untouched, so stopping at 16 silently drops every symbol that was
     * longer and leaves huffval shorter than sum(bits) -- garbage codes follow. */
    int p = 0;
    for (int len = 1; len <= 256; len++)
        for (int i = 0; i < 256; i++)
            if (codesize[i] == len) h->huffval[p++] = (uint8_t)i;
    /* assign codes */
    memset(h->size, 0, sizeof h->size);
    memset(h->code, 0, sizeof h->code);
    int idx = 0; uint16_t code = 0;
    for (int len = 1; len <= 16; len++) {
        for (int n = 0; n < h->bits[len]; n++) {
            h->code[h->huffval[idx]] = code++;
            h->size[h->huffval[idx]] = (uint8_t)len;
            idx++;
        }
        code <<= 1;
    }
    for (int i = 0; i < 256; i++)
        h->cs[i] = ((uint32_t)h->size[i] << 16) | (uint32_t)h->code[i];
}

#ifdef __ARM_NEON
/* --------------------------------------------------------------- block emit
 * One 8x8 block, zigzagged coefficients still in the eight NEON registers the
 * quantiser left them in.  AC symbols are emitted LJ12_EMIT_W at a time and
 * merged into a single fbw_putw; at ~5.5 bits per symbol five of them average
 * 28 bits, comfortably inside the 57-bit cap, and the rare group that overflows
 * (or contains a run of 16+ zeros) falls back to one call per symbol.
 * Bit-exact with the one-symbol-per-call loop it replaces. */
#define LJ12_EMIT_W 5
static inline void lj12_emit_block(fbw_t *b, const int16x8_t z[8], int *prevdc,
                                   const huff_t *dc, const huff_t *ac)
{
    uint16_t szv[64], vbv[64];
    int danger;
    uint64_t m = lj12_mask_split(z, szv, vbv, &danger);
    if (__builtin_expect(danger, 0)) {                 /* never seen in practice */
        int16_t o[64];
        for (int k = 0; k < 8; k++) vst1q_s16(o + k*8, z[k]);
        for (int i = 0; i < 64; i++) {
            int v = o[i], s = magcat_fast(v);
            szv[i] = (uint16_t)s; vbv[i] = (uint16_t)magbits(v, s);
        }
    }
    const uint32_t zrl = ac->cs[0xF0], eob = ac->cs[0];
    int dcv = vgetq_lane_s16(z[0], 0);
    int diff = dcv - *prevdc; *prevdc = dcv;
    int sdc = magcat_fast(diff); uint32_t dcs = dc->cs[sdc];
    fbw_putw(b, ((uint64_t)(dcs & 0xFFFF) << sdc) | magbits(diff, sdc), (int)(dcs >> 16) + sdc);

    uint64_t mm = m & ~1ULL;
    int prev = 0, cnt = __builtin_popcountll(mm);
    while (cnt >= LJ12_EMIT_W) {
        cnt -= LJ12_EMIT_W;
        int ii[LJ12_EMIT_W], rr[LJ12_EMIT_W];
        for (int t = 0; t < LJ12_EMIT_W; t++) { ii[t] = __builtin_ctzll(mm); mm &= mm - 1; }
        rr[0] = ii[0] - prev - 1;
        for (int t = 1; t < LJ12_EMIT_W; t++) rr[t] = ii[t] - ii[t-1] - 1;
        prev = ii[LJ12_EMIT_W-1];
        uint64_t code = 0; int tot = 0, bad = 0;
        for (int t = 0; t < LJ12_EMIT_W; t++) {
            unsigned s = szv[ii[t]]; bad |= rr[t];
            uint32_t c = ac->cs[((unsigned)rr[t] << 4) | s];
            int l = (int)(c >> 16) + (int)s;
            code = (code << l) | (((uint64_t)(c & 0xFFFF) << s) | vbv[ii[t]]);
            tot += l;
        }
        if (__builtin_expect(bad < 16 && tot <= 57, 1)) fbw_putw(b, code, tot);
        else for (int t = 0; t < LJ12_EMIT_W; t++) {
            int run = rr[t];
            while (run >= 16) { fbw_putw(b, zrl & 0xFFFF, (int)(zrl >> 16)); run -= 16; }
            unsigned s = szv[ii[t]];
            uint32_t c = ac->cs[((unsigned)run << 4) | s];
            fbw_putw(b, ((uint64_t)(c & 0xFFFF) << s) | vbv[ii[t]], (int)(c >> 16) + (int)s);
        }
    }
    while (cnt--) {
        int i = __builtin_ctzll(mm); mm &= mm - 1;
        int run = i - prev - 1; prev = i;
        while (run >= 16) { fbw_putw(b, zrl & 0xFFFF, (int)(zrl >> 16)); run -= 16; }
        unsigned s = szv[i];
        uint32_t c = ac->cs[((unsigned)run << 4) | s];
        fbw_putw(b, ((uint64_t)(c & 0xFFFF) << s) | vbv[i], (int)(c >> 16) + (int)s);
    }
    if (prev < 63) fbw_putw(b, eob & 0xFFFF, (int)(eob >> 16));
}
#endif /* __ARM_NEON */

static int magcat(int v) { int a = v < 0 ? -v : v, n = 0; while (a) { n++; a >>= 1; } return n; }

/* ---- per-component coefficient production ---- */
/* DCT + quantise + nonzero-mask + Huffman histogram, all in one pass over the
 * coefficients while they are still in registers. The separate statistics pass
 * cost 56 ms/frame standalone; fused it costs ~16 ms marginal. */
static void collect_blocks(const uint16_t *plane, int w, int h, int stride,
                           const uint16_t qnat[64], int16_t *coeffs,
                           uint64_t *masks, uint32_t *hist)
{
    init_costab(); init_bzz();
    /* Exact replacement for the per-coefficient integer divide: n/q == (n*recip)>>40
     * with recip = ceil(2^40/q). Brute-force verified over 5.7e9 (q,n) pairs.
     * 64 sdivs per block was costing more than the whole DCT butterfly. */
    uint64_t recip[64];
    for (int i = 0; i < 64; i++) recip[i] = ((1ULL << 40) + qnat[i] - 1) / qnat[i];
#if defined(__ARM_NEON)
    /* AAN scale factors folded into the reciprocal: inv = 1/(8*S_u*S_v*q). */
    float inv[128]; aan_inv_table(qnat, inv);
    const int fastq = 1;            /* the fp32 path has no q>=2 restriction at all */
#endif

    int bx = (w + 7) / 8, by = (h + 7) / 8;
    int prevdc = 0;
    for (int byi = 0; byi < by; byi++)
      for (int bxi = 0; bxi < bx; bxi++) {
        int32_t blk[64];
        const int x0 = bxi*8, y0 = byi*8;
        if (x0 + 8 <= w && y0 + 8 <= h) {
#if defined(__ARM_NEON)
            const int32x4_t bias = vdupq_n_s32(2048);
            for (int y = 0; y < 8; y++) {
                uint16x8_t v = vld1q_u16(plane + (size_t)(y0+y)*stride + x0);
                vst1q_s32(blk+y*8,   vsubq_s32(vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(v))),  bias));
                vst1q_s32(blk+y*8+4, vsubq_s32(vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(v))), bias));
            }
#else
            for (int y = 0; y < 8; y++) {
                const uint16_t *src = plane + (size_t)(y0+y)*stride + x0;
                for (int x = 0; x < 8; x++) blk[y*8+x] = (int32_t)src[x] - 2048;
            }
#endif
        } else {                                   /* edge blocks: replicate, not zero-pad */
            for (int y = 0; y < 8; y++) {
                int sy = y0 + y; if (sy >= h) sy = h - 1;
                for (int x = 0; x < 8; x++) {
                    int sx = x0 + x; if (sx >= w) sx = w - 1;
                    blk[y*8+x] = (int32_t)plane[(size_t)sy*stride + sx] - 2048;
                }
            }
        }
#if defined(__ARM_NEON)
        int16_t *out = coeffs + ((size_t)byi*bx + bxi) * 64;
        if (fastq) {
            aan_qz(blk, inv, out);   /* DCT + quantise + zigzag, no round trip */
        } else
#endif
        {
#if defined(__ARM_NEON)
        fdct_islow_v2(blk);
#else
        fdct_islow(blk);
        int16_t *out = coeffs + ((size_t)byi*bx + bxi) * 64;
#endif
        for (int i = 0; i < 64; i++) {
            int nat = ZZ[i];
            int32_t d = blk[nat];
            uint32_t n = (uint32_t)((d >= 0 ? d : -d) + (qnat[nat] >> 1));
            uint32_t mag = (uint32_t)(((uint64_t)n * recip[nat]) >> 40);
            int v = d >= 0 ? (int)mag : -(int)mag;
            if (v >  32767) v =  32767;
            if (v < -32768) v = -32768;
            out[i] = (int16_t)v;                   /* zigzag order */
        }
        }
        {   /* fused statistics. Four rotating histogram banks: hist[sym]++ on a
             * repeating index (EOB and ZRL dominate) serialises on store-forwarding,
             * and the increments are otherwise independent. Summed by the caller. */
            size_t bi = (size_t)byi*bx + bxi;
            uint64_t m = nzmask(out);
            masks[bi] = m;
            if (hist) {
                int diff = out[0] - prevdc; prevdc = out[0];
                hist[256 + magcat_fast(diff)]++;
                uint64_t mm = m & ~1ULL; int prev = 0; int bank = 0;
                while (mm) {
                    int i = __builtin_ctzll(mm); mm &= mm - 1;
                    int run = i - prev - 1; prev = i;
                    while (run >= 16) { hist[512*(bank&3) + 0xF0]++; bank++; run -= 16; }
                    hist[512*(bank&3) + (((unsigned)run << 4) | (unsigned)magcat_fast(out[i]))]++;
                    bank++;
                }
                if (prev < 63) hist[0]++;
            }
        }
      }
}

static void emit_scan(fbw_t *b, const int16_t *coeffs, const uint64_t *masks,
                      int nblk, const huff_t *dc, const huff_t *ac)
{
    int prevdc = 0;
    for (int blk = 0; blk < nblk; blk++) {
        if (fbw_reserve(b)) return;
        const int16_t *c = coeffs + (size_t)blk*64;
        int diff = c[0] - prevdc; prevdc = c[0];
        int s = magcat_fast(diff);
        uint32_t dcs = dc->cs[s];
        fbw_put(b, ((dcs & 0xFFFF) << s) | magbits(diff, s), (int)(dcs >> 16) + s);
        uint64_t m = masks[blk] & ~1ULL; int prev = 0;
        while (m) {                                  /* one iteration per NONZERO */
            int i = __builtin_ctzll(m); m &= m - 1;
            int run = i - prev - 1; prev = i;
            while (run >= 16) { fbw_put(b, ac->cs[0xF0] & 0xFFFF, (int)(ac->cs[0xF0] >> 16)); run -= 16; }
            int v = c[i], sz = magcat_fast(v);
            unsigned sym = ((unsigned)run << 4) | (unsigned)sz;
            uint32_t cs = ac->cs[sym];
            fbw_put(b, ((cs & 0xFFFF) << sz) | magbits(v, sz), (int)(cs >> 16) + sz);
        }
        if (prev < 63) fbw_put(b, ac->cs[0] & 0xFFFF, (int)(ac->cs[0] >> 16));
    }
}

static void emit_dht(fbw_t *b, int tclass, int tid, const huff_t *h) {
    int nsym = 0; for (int i = 1; i <= 16; i++) nsym += h->bits[i];
    int len = 2 + 1 + 16 + nsym;
    fbw_byte(b,0xFF); fbw_byte(b,0xC4);
    fbw_byte(b,(uint8_t)(len>>8)); fbw_byte(b,(uint8_t)len);
    fbw_byte(b,(uint8_t)((tclass<<4)|tid));
    for (int i = 1; i <= 16; i++) fbw_byte(b,h->bits[i]);
    for (int i = 0; i < nsym; i++) fbw_byte(b,h->huffval[i]);
}


/* FUSED quantise+emit. Usable only when the Huffman tables are already known
 * (table-reuse mode): coefficients go straight from the DCT registers into the
 * bitstream, so the int16 coefficient array and the nonzero-mask array vanish
 * along with the second pass over them. At 4K that is ~35 MB/frame of write +
 * read-back traffic removed. The two scans are emitted sequentially into the
 * same buffer, so no splice is needed either. */
static void collect_emit(const uint16_t *plane, int w, int h, int stride,
                         const uint16_t qnat[64], const huff_t *dc, const huff_t *ac,
                         fbw_t *b)
{
    init_costab(); init_bzz();
    /* AAN scale factors folded into the reciprocal: inv = 1/(8*S_u*S_v*q). */
    float inv[128]; aan_inv_table(qnat, inv);
    const int fastq = 1;            /* the fp32 path has no q>=2 restriction at all */
    uint64_t recip[64];
    for (int i = 0; i < 64; i++) recip[i] = ((1ULL << 40) + qnat[i] - 1) / qnat[i];

    const int bx = (w + 7) / 8, by = (h + 7) / 8;
    int prevdc = 0;
    int16_t out[64];
    int16x8_t z[8];
    for (int byi = 0; byi < by; byi++)
      for (int bxi = 0; bxi < bx; bxi++) {
        int32_t blk[64];
        const int x0 = bxi*8, y0 = byi*8;
        if (x0 + 8 <= w && y0 + 8 <= h) {
            const int32x4_t bias = vdupq_n_s32(2048);
            for (int y = 0; y < 8; y++) {
                uint16x8_t v = vld1q_u16(plane + (size_t)(y0+y)*stride + x0);
                vst1q_s32(blk+y*8,   vsubq_s32(vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(v))),  bias));
                vst1q_s32(blk+y*8+4, vsubq_s32(vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(v))), bias));
            }
        } else {
            for (int y = 0; y < 8; y++) {
                int sy = y0 + y; if (sy >= h) sy = h - 1;
                for (int x = 0; x < 8; x++) {
                    int sx = x0 + x; if (sx >= w) sx = w - 1;
                    blk[y*8+x] = (int32_t)plane[(size_t)sy*stride + sx] - 2048;
                }
            }
        }
        if (fastq) {
            aan_qz_z(blk, inv, z);   /* DCT + quantise + zigzag, straight into registers */
        } else {
            fdct_islow_v2(blk);
            for (int i = 0; i < 64; i++) {
                int nat = ZZ[i];
                int32_t d = blk[nat];
                uint32_t n = (uint32_t)((d >= 0 ? d : -d) + (qnat[nat] >> 1));
                uint32_t mag = (uint32_t)(((uint64_t)n * recip[nat]) >> 40);
                int v = d >= 0 ? (int)mag : -(int)mag;
                if (v >  32767) v =  32767;
                if (v < -32768) v = -32768;
                out[i] = (int16_t)v;
            }
            for (int k = 0; k < 8; k++) z[k] = vld1q_s16(out + k*8);
        }
        if (fbw_reserve(b)) return;
        lj12_emit_block(b, z, &prevdc, dc, ac);
      }
}


/* Gather one 8x8 block straight out of the Bayer raster in BMD component order.
 * comp: 0 = even CFA rows, 1 = odd. Column phase is constant within a block. */
static inline void cfa_gather_block(const uint16_t *bayer, long stride, int comp,
                                    int bx8, int by8, int hw, int shift,
                                    const uint16_t *fwd, int32_t blk[64])
{
    const int oddPhase = (bx8 >= hw);
    const int cbase = oddPhase ? (bx8 - hw) : bx8;
    for (int y = 0; y < 8; y++) {
        /* BMD/slimRAW packing, VERIFIED against a Blackmagic lossless frame
         * decoded by Adobe's DNG SDK: the JPEG COMPONENT selects the COLUMN
         * parity and the half-row selects the ROW parity -- the transpose of
         * the obvious reading. Getting it backwards leaves the two off-diagonal
         * CFA phases exchanged: on BMD's own GBRG that swaps R and B, but on
         * this sensor's RGGB it exchanges the two GREEN sub-lattices, so every
         * green sample lands one pixel off the diagonal. Flat fields hide it
         * (the greens are equal there) and our own reader hid it completely
         * (same convention both ends) -- it only showed up as destroyed edges
         * and colour fringing in Resolve. */
        const uint16_t *row = bayer + (long)(2 * (by8 + y) + (oddPhase ? 1 : 0)) * stride
                                    + (long)(2 * cbase) + comp;
        /* TRANSPOSED store: blk[x*8+y].  The DCT's first act was to transpose the
         * block, and this gather is a scalar strided one anyway -- swapping the
         * store index costs nothing and deletes a whole 8x8 transpose. */
        /* Samples arrive MSB-justified in 16 bits with shift = 16 - sampleBits,
         * so row[2*x] >> shift is already inside 0..4095 -- the mask was a
         * no-op -- and the compand LUT is never NULL on the production path
         * (LossyDngEncoder always passes the GP-Log2 table).  Both were
         * per-sample work on 8.3 M samples per frame; removing them is
         * byte-identical and measured +0.8 to +1.6%. */
        for (int x = 0; x < 8; x++)
            blk[x * 8 + y] = (int32_t)fwd[row[2 * x] >> shift] - 2048;
    }
}

/* Dual purpose: with `b` it emits, with `hist` it only gathers statistics.
 * Frame 1 of a take has no carried tables, so it runs this twice; every frame
 * after that runs it once. */
static void collect_emit_cfa(const uint16_t *bayer, long stride, int comp,
                             int w, int h, int shift, const uint16_t *fwd,
                             const uint16_t qnat[64],
                             const huff_t *dc, const huff_t *ac, fbw_t *b,
                             uint32_t *hist, lj12_rate_stats *rs)
{
    /* Rate statistics are SUBSAMPLED 1 block in 16. Measured by the rate study:
     * 1/16 costs nothing (size-error sd 2.25% vs 2.18% at full rate), and the
     * emit path now hands its coefficients to lj12_emit_block in registers, so
     * a per-symbol hook there would slow the hot loop for every block to serve
     * a statistic that only needs a sample. */
    unsigned rs_blk = 0;
    init_costab(); init_bzz();
    /* AAN scale factors folded into the reciprocal: inv = 1/(8*S_u*S_v*q). */
    float inv[128]; aan_inv_table(qnat, inv);
    const int fastq = 1;            /* the fp32 path has no q>=2 restriction at all */
    uint64_t recip[64];
    for (int i = 0; i < 64; i++) recip[i] = ((1ULL << 40) + qnat[i] - 1) / qnat[i];
    const int bx = (w + 7) / 8, by = (h + 7) / 8, hw = w / 2;
    int prevdc = 0;
    int16_t out[64];
    int16x8_t z[8];
    for (int byi = 0; byi < by; byi++)
      for (int bxi = 0; bxi < bx; bxi++) {
        int32_t blk[64];
        cfa_gather_block(bayer, stride, comp, bxi * 8, byi * 8, hw, shift, fwd, blk);
        if (fastq) {
            aan_qz_zT(blk, inv, z);   /* DCT + quantise + zigzag, straight into registers */
        } else {
            fdct_islow_T2(blk);
            for (int i = 0; i < 64; i++) {
                int nat = ZZ[i];
                int32_t d = blk[nat];
                uint32_t n = (uint32_t)((d >= 0 ? d : -d) + (qnat[nat] >> 1));
                uint32_t mag = (uint32_t)(((uint64_t)n * recip[nat]) >> 40);
                int v = d >= 0 ? (int)mag : -(int)mag;
                if (v >  32767) v =  32767;
                if (v < -32768) v = -32768;
                out[i] = (int16_t)v;
            }
            for (int k = 0; k < 8; k++) z[k] = vld1q_s16(out + k*8);
        }
        if (hist) {
            for (int k = 0; k < 8; k++) vst1q_s16(out + k*8, z[k]);
            uint64_t m = nzmask(out);
            int d0 = out[0] - prevdc; prevdc = out[0];
            hist[256 + magcat_fast(d0)]++;
            uint64_t hm = m & ~1ULL; int hp = 0; int bank = 0;
            while (hm) {
                int i = __builtin_ctzll(hm); hm &= hm - 1;
                int run = i - hp - 1; hp = i;
                while (run >= 16) { hist[512*(bank&3) + 0xF0]++; bank++; run -= 16; }
                hist[512*(bank&3) + (((unsigned)run << 4) | (unsigned)magcat_fast(out[i]))]++;
                bank++;
            }
            if (hp < 63) hist[0]++;
            continue;
        }
        if (fbw_reserve(b)) return;
        lj12_emit_block(b, z, &prevdc, dc, ac);
        if (rs && ((rs_blk++ & 15u) == 0u)) {
            int16_t smp[64];
            for (int k = 0; k < 8; k++) vst1q_s16(smp + k*8, z[k]);
            for (int i = 1; i < 64; i++) {           /* AC only; DC is differential */
                int v = smp[i];
                if (!v) continue;
                const unsigned c = (unsigned)magcat_fast(v) & 15u;
                rs->mag[c]++; rs->nz++;
                if (i >= 32) { rs->mag_hf[c]++; rs->nz_hf++; }   /* noise proxy */
            }
            rs->blocks++;
        }
      }
}

/* Gather the Blackmagic CinemaDNG folded raster. A TIFF tile remains W x H,
 * but its JPEG payload is a single 2W x H/2 plane: the even source row is the
 * left half and the following odd row is the right half. Decoders reverse this
 * row deinterleave when placing the tile into the Bayer image. */
static inline void bmd_gather_block(const uint16_t *bayer, long stride,
                                    int x0, int y0, int source_w, int shift,
                                    const uint16_t *fwd, int32_t blk[64])
{
    for (int y = 0; y < 8; ++y) {
        const int folded_y = y0 + y;
        for (int x = 0; x < 8; ++x) {
            const int folded_x = x0 + x;
            const bool odd_row = folded_x >= source_w;
            const int source_x = odd_row ? folded_x - source_w : folded_x;
            const int source_y = folded_y * 2 + (odd_row ? 1 : 0);
            const uint16_t sample = bayer[long(source_y) * stride + source_x];
            blk[x * 8 + y] = int32_t(fwd[sample >> shift]) - 2048;
        }
    }
}

static void collect_emit_bmd(const uint16_t *bayer, long stride,
                             int source_w, int source_h, int shift,
                             const uint16_t *fwd, const uint16_t qnat[64],
                             const huff_t *dc, const huff_t *ac, fbw_t *b,
                             uint32_t *hist, lj12_rate_stats *rs)
{
    unsigned rs_blk = 0;
    init_costab(); init_bzz();
    float inv[128]; aan_inv_table(qnat, inv);
    const int folded_w = source_w * 2;
    const int folded_h = source_h / 2;
    const int bx = folded_w / 8;
    const int by = folded_h / 8;
    int prevdc = 0;
    int16_t out[64];
    int16x8_t z[8];
    for (int byi = 0; byi < by; ++byi) {
        for (int bxi = 0; bxi < bx; ++bxi) {
            int32_t blk[64];
            bmd_gather_block(bayer, stride, bxi * 8, byi * 8,
                             source_w, shift, fwd, blk);
            aan_qz_zT(blk, inv, z);
            if (hist) {
                for (int k = 0; k < 8; ++k)
                    vst1q_s16(out + k * 8, z[k]);
                uint64_t mask = nzmask(out);
                const int diff = out[0] - prevdc;
                prevdc = out[0];
                hist[256 + magcat_fast(diff)]++;
                uint64_t acmask = mask & ~1ULL;
                int previous = 0;
                int bank = 0;
                while (acmask) {
                    const int i = __builtin_ctzll(acmask);
                    acmask &= acmask - 1;
                    int run = i - previous - 1;
                    previous = i;
                    while (run >= 16) {
                        hist[512 * (bank & 3) + 0xF0]++;
                        ++bank;
                        run -= 16;
                    }
                    hist[512 * (bank & 3) + ((unsigned(run) << 4) |
                                             unsigned(magcat_fast(out[i])))]++;
                    ++bank;
                }
                if (previous < 63)
                    hist[0]++;
                continue;
            }
            if (fbw_reserve(b))
                return;
            lj12_emit_block(b, z, &prevdc, dc, ac);
            if (rs && ((rs_blk++ & 15u) == 0u)) {
                int16_t sample[64];
                for (int k = 0; k < 8; ++k)
                    vst1q_s16(sample + k * 8, z[k]);
                for (int i = 1; i < 64; ++i) {
                    const int value = sample[i];
                    if (!value)
                        continue;
                    const unsigned category = unsigned(magcat_fast(value)) & 15u;
                    rs->mag[category]++;
                    rs->nz++;
                    if (i >= 32) {
                        rs->mag_hf[category]++;
                        rs->nz_hf++;
                    }
                }
                rs->blocks++;
            }
        }
    }
}



/* ---- rate control -----------------------------------------------------------
 *
 * Predicts the compressed size at ANY quantiser scale from a magnitude
 * histogram gathered at one scale, then solves for the scale that lands on a
 * byte budget.
 *
 * WHY A HISTOGRAM AND NOT A FEEDBACK LOOP ON BYTES. The plant gain
 * d(ln bytes)/d(ln scale) runs from -0.09 to -1.29 across the operating range --
 * a 14x span -- so no fixed loop gain is both stable at one end and fast at the
 * other, and a measured sweep took tens of frames to settle or oscillated. A
 * distribution can be re-evaluated for a different table directly, which turns
 * the problem from control into arithmetic.
 *
 * THE MODEL. Scaling the quantiser by r multiplies every coefficient by 1/r, so
 * each JPEG magnitude category c moves to c - log2(r); the histogram shifts
 * bodily and fractional shifts split between adjacent bins. Bits are then
 *     magnitude bits (literal, exact)  +  nonzeros * (bits per symbol)
 * and the bits-per-symbol term is MEASURED from the frame just encoded
 * (scan_bits minus the magnitude bits, over the nonzero count) rather than
 * fitted offline -- so it tracks content, the Huffman tables, and any change to
 * the encoder itself without recalibration.
 *
 * KNOWN BIAS: coefficients that quantised to zero are not in the histogram, so
 * going FINER (more of them surviving) is underestimated. The caller's trim
 * absorbs it, and the per-frame step clamp stops it running away.
 */
int lj12_solve_scale(const lj12_rate_stats *rs, int cur_scale, double budget_bytes)
{
    if (!rs || rs->nz == 0 || cur_scale <= 0 || budget_bytes <= 0.0) return cur_scale;

    /* Magnitude bits actually spent at the current scale, from the sample. */
    double smag = 0.0, nz = 0.0;
    for (int c = 1; c < 16; c++) { smag += (double)rs->mag[c] * c; nz += rs->mag[c]; }
    if (nz <= 0.0) return cur_scale;

    /* Sample covers 1 block in 16, so scale the sample up to the whole frame. */
    const double sampled_blocks = rs->blocks > 0 ? (double)rs->blocks : 1.0;
    const double frame_bits = (double)rs->scan_bits;
    const double sample_frac = sampled_blocks * 16.0;      /* blocks represented */
    (void)sample_frac;
    /* bits per symbol beyond its magnitude bits, measured on this frame */
    const double scaled_smag = smag * (frame_bits > 0.0 ? 1.0 : 1.0);
    double per_sym = 0.0;
    {
        /* frame_bits corresponds to ALL blocks; smag/nz to the sample. Put both
         * on the sample's scale using the same 1/16 factor. */
        const double frame_bits_sample = frame_bits / 16.0;
        per_sym = (frame_bits_sample - scaled_smag) / nz;
        if (!(per_sym > 0.0) || per_sym > 32.0) per_sym = 6.0;   /* sane fallback */
    }

    /* Bytes predicted at scale s: shift the histogram by -log2(s/cur). */
    const double bytes_at = 0.0; (void)bytes_at;
    double lo = 1.0, hi = 20000.0, best = (double)cur_scale;
    for (int it = 0; it < 40; it++) {
        const double s = 0.5 * (lo + hi);
        const double d = -log2(s / (double)cur_scale);   /* category shift */
        const int    di = (int)floor(d);
        const double fr = d - (double)di;                /* split weight */
        double pbits = 0.0, pnz = 0.0;
        for (int c = 1; c < 16; c++) {
            const double n = (double)rs->mag[c];
            if (n <= 0.0) continue;
            const int c0 = c + di, c1 = c0 + 1;
            const double w1 = fr, w0 = 1.0 - fr;
            if (c0 >= 1) { pbits += n * w0 * (double)c0; pnz += n * w0; }
            if (c1 >= 1) { pbits += n * w1 * (double)c1; pnz += n * w1; }
        }
        const double pred_bytes = ((pbits + pnz * per_sym) * 16.0) / 8.0;
        if (pred_bytes > budget_bytes) lo = s; else hi = s;
        best = s;
        if (hi - lo < 0.5) break;
    }
    /* ASYMMETRIC STEP CLAMP, because the model's error is asymmetric.
     *
     * Measured on a real 4K frame, solving from four starting scales to four
     * budgets: moving COARSER (scale up, fewer bytes) lands within +0.9% to
     * +2.5%; moving FINER (scale down, more bytes) overshoots by up to +242%,
     * because coefficients that quantised to zero are absent from the histogram
     * and the solver cannot see how many would come back. Near the operating
     * point -- the steady state -- it is accurate to about 2% in both
     * directions.
     *
     * So: let it coarsen quickly (it is right, and over-budget is the direction
     * that drops frames and overruns the card), and approach finer scales in
     * measured steps, converging over a few frames instead of overshooting into
     * a frame the disk cannot take. */
    const double cur = (double)cur_scale;
    if (best > cur * 2.00) best = cur * 2.00;    /* coarser: model is reliable */
    if (best < cur * 0.75) best = cur * 0.75;    /* finer: model underestimates */
    if (best < 1.0) best = 1.0;
    if (best > 20000.0) best = 20000.0;
    return (int)(best + 0.5);
}


/* The coarsest scale that is still transparent against this frame's own noise.
 *
 * THE NOISE FLOOR comes from the high-frequency coefficients, which on raw
 * sensor content are overwhelmingly photon noise. Their MEDIAN magnitude, not
 * their RMS: the rate study measured the RMS of those bands overestimating
 * sigma by 1.2x to 2.3x with a content-dependent bias, while the median
 * recovered it to 0.93 +/- 0.14x. For a half-normal, median = 0.6745 sigma.
 *
 * Everything here is in QUANTISER UNITS -- the histogram holds |coefficient|/q
 * as categories -- so sigma/q falls out directly and the answer is a ratio on
 * the current scale, needing no absolute calibration.
 *
 * Transparency: uniform quantisation with step Q has RMS error Q/sqrt(12), so
 * the coarsest transparent step is Q = alpha * sigma * sqrt(12).
 *
 * Returns 0 for "no opinion" when there is not enough high-frequency signal to
 * estimate from -- an almost-black frame, or one already through a lossy codec,
 * both of which read near zero here. That is correct, and must not be mistaken
 * for a clean sensor: with no opinion the rate solver is left to decide alone.
 */
int lj12_solve_quality_scale(const lj12_rate_stats *rs, int cur_scale, double alpha)
{
    if (!rs || cur_scale <= 0 || alpha <= 0.0) return 0;
    const double hf_total = (double)rs->blocks * 32.0;   /* AC coefficients, zigzag >= 32 */
    if (hf_total < 4096.0) return 0;
    const double frac = (double)rs->nz_hf / hf_total;    /* fraction that survived */

    /* SIGMA FROM THE SURVIVING FRACTION, not from a median.
     *
     * A median estimator fails exactly where this lives: at these ratios most
     * high-frequency coefficients quantise to zero, so the median of the full
     * population IS zero and carries no information. The SURVIVAL RATE still
     * does. High-frequency coefficients on raw sensor content are essentially
     * zero-mean Gaussian noise, and a coefficient survives when |c| >= q/2, so
     *     frac = erfc( (q/2) / (sigma*sqrt2) )
     * which inverts for sigma/q. It is well conditioned for frac in roughly
     * 0.02..0.9 and degrades gracefully outside, where we decline to answer.
     *
     * Everything is in quantiser units, so no absolute calibration is needed --
     * the result is a ratio to apply to the current scale.
     */
    /* The UPPER guard used to sit at 0.90 and fired on exactly the case this
     * function exists for: a clean frame, where the rate solver drives the scale
     * very fine, nearly every high-frequency coefficient survives, and frac goes
     * high. Measured: a 0.3 DN synthetic frame reached scale 17 and 11:1 -- far
     * under budget and far finer than transparency needs, costing encode time
     * for nothing -- while the floor declined to answer. erfc stays invertible
     * well past 0.9 (erfc(0.0089) = 0.99), so only the genuinely degenerate ends
     * are excluded now. */
    if (frac < 0.02 || frac > 0.995) return 0;

    /* Invert frac = erfc(x) for x by bisection; erfc is monotone. */
    double lo = 0.0, hi = 4.0, x = 1.0;
    for (int it = 0; it < 50; it++) {
        x = 0.5 * (lo + hi);
        if (erfc(x) > frac) lo = x; else hi = x;
    }
    if (x <= 1e-6) return 0;
    /* x = (q/2)/(sigma*sqrt2)  =>  sigma/q = 1/(2*sqrt2*x) */
    const double sigma_over_q = 1.0 / (2.0 * 1.41421356 * x);

    /* Uniform quantisation with step Q has RMS error Q/sqrt(12); transparency
     * at alpha means Q <= alpha*sigma*sqrt(12). */
    const double q_ratio = alpha * sigma_over_q * 3.4641016;
    if (q_ratio <= 1.0) return 0;    /* already at or finer than transparent: no opinion */

    double sc = (double)cur_scale * q_ratio;
    if (sc < 1.0) sc = 1.0;
    if (sc > 20000.0) sc = 20000.0;
    return (int)(sc + 0.5);
}

int lj12_encode_tile_cfa(lj12_ctx *ctx, const uint16_t *bayer, long stride,
                         int tile_w, int tile_h, int sample_shift,
                         const uint16_t *fwd, const uint16_t qnat[64],
                         lj12_tables *tabs, const uint8_t **out, size_t *out_len)
{
    return lj12_encode_tile_cfa_rs(ctx, bayer, stride, tile_w, tile_h, sample_shift,
                                   fwd, qnat, tabs, out, out_len, NULL);
}

int lj12_encode_tile_cfa_rs(lj12_ctx *ctx, const uint16_t *bayer, long stride,
                            int tile_w, int tile_h, int sample_shift,
                            const uint16_t *fwd, const uint16_t qnat[64],
                            lj12_tables *tabs, const uint8_t **out, size_t *out_len,
                            lj12_rate_stats *rs)
{
    if (!ctx || !bayer || tile_w <= 0 || tile_h <= 0) return -1;
    if ((tile_w % 16) || (tile_h % 16)) return -1;     /* TIFF: both multiples of 16 */
    const int w = tile_w, h = tile_h / 2;              /* component geometry */
    const int bx = (w + 7) / 8, by = (h + 7) / 8, nblk = bx * by;
    const size_t outb = (size_t)nblk * 64 * 2 * 2 + 65536;
    if (ctx_reserve(ctx, 0, 0, outb)) return -1;

    huff_t hdc0, hac0, hdc1, hac1;
    const int reuse = lj12_tables_valid(tabs);
    if (reuse) { hdc0 = tabs->dc0; hac0 = tabs->ac0; hdc1 = tabs->dc1; hac1 = tabs->ac1; }
    else {
        uint32_t h0[512*4] = {0}, h1[512*4] = {0};
        collect_emit_cfa(bayer, stride, 0, w, h, sample_shift, fwd, qnat, NULL, NULL, NULL, h0, NULL);
        collect_emit_cfa(bayer, stride, 1, w, h, sample_shift, fwd, qnat, NULL, NULL, NULL, h1, NULL);
        for (int b2 = 1; b2 < 4; b2++)
            for (int i = 0; i < 512; i++) { h0[i] += h0[512*b2+i]; h1[i] += h1[512*b2+i]; }
        long dcf0[257]={0}, acf0[257]={0}, dcf1[257]={0}, acf1[257]={0};
        for (int i = 0; i < 256; i++) { acf0[i] = h0[i]; acf1[i] = h1[i]; }
        for (int i = 0; i < 17;  i++) { dcf0[i] = h0[256+i]; dcf1[i] = h1[256+i]; }
        if (tabs) {
            for (int r = 0; r < 16; r++) for (int z = 1; z <= 14; z++) { acf0[(r<<4)|z]++; acf1[(r<<4)|z]++; }
            acf0[0x00]++; acf1[0x00]++; acf0[0xF0]++; acf1[0xF0]++;
            for (int i = 0; i <= 15; i++) { dcf0[i]++; dcf1[i]++; }
        }
        huff_build(dcf0,&hdc0); huff_build(acf0,&hac0);
        huff_build(dcf1,&hdc1); huff_build(acf1,&hac1);
        if (tabs) { tabs->dc0=hdc0; tabs->ac0=hac0; tabs->dc1=hdc1; tabs->ac1=hac1; tabs->valid=1; }
    }

    fbw_t b;
    b.base = b.p = ctx->out; b.cap = ctx->out_cap;
    b.limit = ctx->out + ctx->out_cap - FBW_SLACK;
    b.acc = 0; b.nbits = 0; b.oom = 0;
    fbw_byte(&b,0xFF); fbw_byte(&b,0xD8);
    fbw_byte(&b,0xFF); fbw_byte(&b,0xDB); fbw_byte(&b,0); fbw_byte(&b,67+64); fbw_byte(&b,0x10);
    for (int i = 0; i < 64; i++) { uint16_t q = qnat[ZZ[i]]; fbw_byte(&b,(uint8_t)(q>>8)); fbw_byte(&b,(uint8_t)q); }
    emit_dht(&b,0,0,&hdc0); emit_dht(&b,1,0,&hac0);
    emit_dht(&b,0,1,&hdc1); emit_dht(&b,1,1,&hac1);
    fbw_byte(&b,0xFF); fbw_byte(&b,0xC1);
    fbw_byte(&b,0); fbw_byte(&b,8+3*2); fbw_byte(&b,12);
    fbw_byte(&b,(uint8_t)(h>>8)); fbw_byte(&b,(uint8_t)h);
    fbw_byte(&b,(uint8_t)(w>>8)); fbw_byte(&b,(uint8_t)w);
    fbw_byte(&b,2);
    fbw_byte(&b,0); fbw_byte(&b,0x11); fbw_byte(&b,0);
    fbw_byte(&b,1); fbw_byte(&b,0x11); fbw_byte(&b,0);
    fbw_byte(&b,0xFF); fbw_byte(&b,0xDA); fbw_byte(&b,0); fbw_byte(&b,8);
    fbw_byte(&b,1); fbw_byte(&b,0); fbw_byte(&b,0x00); fbw_byte(&b,0); fbw_byte(&b,63); fbw_byte(&b,0);
    const uint8_t *scan0 = b.p;
    collect_emit_cfa(bayer, stride, 0, w, h, sample_shift, fwd, qnat, &hdc0, &hac0, &b, NULL, rs);
    fbw_finish(&b);
    fbw_byte(&b,0xFF); fbw_byte(&b,0xDA); fbw_byte(&b,0); fbw_byte(&b,8);
    fbw_byte(&b,1); fbw_byte(&b,1); fbw_byte(&b,0x11); fbw_byte(&b,0); fbw_byte(&b,63); fbw_byte(&b,0);
    collect_emit_cfa(bayer, stride, 1, w, h, sample_shift, fwd, qnat, &hdc1, &hac1, &b, NULL, rs);
    fbw_finish(&b);
    fbw_byte(&b,0xFF); fbw_byte(&b,0xD9);
    if (b.oom) return -1;
    if (rs) rs->scan_bits += (uint64_t)(b.p - scan0) * 8u;
    ctx->out = b.base; ctx->out_cap = b.cap;
    *out = b.base; *out_len = (size_t)(b.p - b.base);
    return 0;
}

int lj12_encode_tile_bmd_rs(lj12_ctx *ctx, const uint16_t *bayer, long stride,
                            int tile_w, int tile_h, int sample_shift,
                            const uint16_t *fwd, const uint16_t qnat[64],
                            lj12_tables *tabs, const uint8_t **out, size_t *out_len,
                            lj12_rate_stats *rs)
{
    if (!ctx || !bayer || !fwd || tile_w <= 0 || tile_h <= 0)
        return -1;
    if ((tile_w % 4) || (tile_h % 16))
        return -1;

    const int w = tile_w * 2;
    const int h = tile_h / 2;
    const int nblk = (w / 8) * (h / 8);
    const size_t outb = size_t(nblk) * 64u * 2u + 65536u;
    if (ctx_reserve(ctx, 0, 0, outb))
        return -1;

    huff_t hdc, hac;
    const int reuse = lj12_tables_valid(tabs);
    if (reuse) {
        hdc = tabs->dc0;
        hac = tabs->ac0;
    } else {
        uint32_t histogram[512 * 4] = {0};
        collect_emit_bmd(bayer, stride, tile_w, tile_h, sample_shift, fwd,
                         qnat, nullptr, nullptr, nullptr, histogram, nullptr);
        for (int bank = 1; bank < 4; ++bank)
            for (int i = 0; i < 512; ++i)
                histogram[i] += histogram[512 * bank + i];
        long dcf[257] = {0}, acf[257] = {0};
        for (int i = 0; i < 256; ++i)
            acf[i] = histogram[i];
        for (int i = 0; i < 17; ++i)
            dcf[i] = histogram[256 + i];
        if (tabs) {
            for (int run = 0; run < 16; ++run)
                for (int size = 1; size <= 14; ++size)
                    acf[(run << 4) | size]++;
            acf[0x00]++;
            acf[0xF0]++;
            for (int i = 0; i <= 15; ++i)
                dcf[i]++;
        }
        huff_build(dcf, &hdc);
        huff_build(acf, &hac);
        if (tabs) {
            tabs->dc0 = hdc;
            tabs->ac0 = hac;
            tabs->valid = 1;
        }
    }

    fbw_t b;
    b.base = b.p = ctx->out;
    b.cap = ctx->out_cap;
    b.limit = ctx->out + ctx->out_cap - FBW_SLACK;
    b.acc = 0;
    b.nbits = 0;
    b.oom = 0;

    fbw_byte(&b, 0xFF); fbw_byte(&b, 0xD8);
    fbw_byte(&b, 0xFF); fbw_byte(&b, 0xDB);
    fbw_byte(&b, 0); fbw_byte(&b, 67 + 64); fbw_byte(&b, 0x10);
    for (int i = 0; i < 64; ++i) {
        const uint16_t q = qnat[ZZ[i]];
        fbw_byte(&b, uint8_t(q >> 8));
        fbw_byte(&b, uint8_t(q));
    }
    emit_dht(&b, 0, 0, &hdc);
    emit_dht(&b, 1, 0, &hac);
    fbw_byte(&b, 0xFF); fbw_byte(&b, 0xC1);
    fbw_byte(&b, 0); fbw_byte(&b, 11); fbw_byte(&b, 12);
    fbw_byte(&b, uint8_t(h >> 8)); fbw_byte(&b, uint8_t(h));
    fbw_byte(&b, uint8_t(w >> 8)); fbw_byte(&b, uint8_t(w));
    fbw_byte(&b, 1);
    fbw_byte(&b, 1); fbw_byte(&b, 0x11); fbw_byte(&b, 0);
    fbw_byte(&b, 0xFF); fbw_byte(&b, 0xDA);
    fbw_byte(&b, 0); fbw_byte(&b, 8);
    fbw_byte(&b, 1); fbw_byte(&b, 1); fbw_byte(&b, 0x00);
    fbw_byte(&b, 0); fbw_byte(&b, 63); fbw_byte(&b, 0);
    const uint8_t *scan = b.p;
    collect_emit_bmd(bayer, stride, tile_w, tile_h, sample_shift, fwd,
                     qnat, &hdc, &hac, &b, nullptr, rs);
    fbw_finish(&b);
    fbw_byte(&b, 0xFF); fbw_byte(&b, 0xD9);
    if (b.oom)
        return -1;
    if (rs)
        rs->scan_bits += uint64_t(b.p - scan) * 8u;
    ctx->out = b.base;
    ctx->out_cap = b.cap;
    *out = b.base;
    *out_len = size_t(b.p - b.base);
    return 0;
}

int lj12_encode_tile_reuse(lj12_ctx *ctx,
                         const uint16_t *comp0, const uint16_t *comp1,
                         int w, int h, int stride0, int stride1,
                         const uint16_t qnat[64], lj12_tables *tabs,
                         const uint8_t **out, size_t *out_len)
{
    const int reuse = lj12_tables_valid(tabs);
    if (!ctx || !comp0 || !comp1 || w <= 0 || h <= 0) return -1;
    int bx = (w + 7) / 8, by = (h + 7) / 8, nblk = bx * by;
    /* In reuse mode the coefficient and mask arrays are never materialised --
     * collect_emit() writes straight to the bitstream -- so only the output
     * buffer is needed. */
    size_t coefb = reuse ? 0 : (size_t)nblk * 64 * sizeof(int16_t) * 2;
    size_t maskb = reuse ? 0 : (size_t)nblk * sizeof(uint64_t) * 2;
    size_t outb  = (size_t)nblk * 64 * 2 * 2 + 65536;
    if (ctx_reserve(ctx, coefb, maskb, outb)) return -1;
    int16_t  *c0 = ctx->coef, *c1 = c0 ? ctx->coef + (size_t)nblk * 64 : NULL;
    uint64_t *m0 = ctx->mask, *m1 = m0 ? ctx->mask + nblk : NULL;

    uint32_t h0[512*4] = {0}, h1[512*4] = {0};
    if (!reuse) {
        collect_blocks(comp0, w, h, stride0, qnat, c0, m0, h0);
        collect_blocks(comp1, w, h, stride1, qnat, c1, m1, h1);
    }

    huff_t hdc0, hac0, hdc1, hac1;
    if (reuse) {
        hdc0 = tabs->dc0; hac0 = tabs->ac0; hdc1 = tabs->dc1; hac1 = tabs->ac1;
    } else {
        for (int b = 1; b < 4; b++)                  /* fold the rotating banks */
            for (int i = 0; i < 512; i++) { h0[i] += h0[512*b+i]; h1[i] += h1[512*b+i]; }
        long dcf0[257]={0}, acf0[257]={0}, dcf1[257]={0}, acf1[257]={0};
        for (int i = 0; i < 256; i++) { acf0[i] = h0[i];       acf1[i] = h1[i]; }
        for (int i = 0; i < 17;  i++) { dcf0[i] = h0[256+i];   dcf1[i] = h1[256+i]; }
        if (tabs) {
            /* Seed every LEGAL symbol so nothing is uncodable next frame. AC size
             * tops out at 14 for 12-bit; DC at 16. An unseeded symbol that shows up
             * later would emit a zero-length code and corrupt the scan. */
            for (int r = 0; r < 16; r++) for (int z = 1; z <= 14; z++) {
                acf0[(r<<4)|z]++; acf1[(r<<4)|z]++;
            }
            acf0[0x00]++; acf1[0x00]++; acf0[0xF0]++; acf1[0xF0]++;
            /* DC categories are 0..15 for 12-bit JPEG. Seeding 16 produces a
             * 17-symbol DC table that libjpeg rejects outright. */
            for (int i = 0; i <= 15; i++) { dcf0[i]++; dcf1[i]++; }
        }
        huff_build(dcf0,&hdc0); huff_build(acf0,&hac0);
        huff_build(dcf1,&hdc1); huff_build(acf1,&hac1);
        if (tabs) { tabs->dc0=hdc0; tabs->ac0=hac0; tabs->dc1=hdc1; tabs->ac1=hac1; tabs->valid=1; }
    }

    fbw_t b;
    b.base = b.p = ctx->out; b.cap = ctx->out_cap;
    b.limit = ctx->out + ctx->out_cap - FBW_SLACK;
    b.acc = 0; b.nbits = 0; b.oom = 0;

    fbw_byte(&b,0xFF); fbw_byte(&b,0xD8);                               /* SOI */
    fbw_byte(&b,0xFF); fbw_byte(&b,0xDB);                               /* DQT, Pq=1 */
    fbw_byte(&b,0); fbw_byte(&b,67+64); fbw_byte(&b,0x10);
    for (int i = 0; i < 64; i++) { uint16_t q = qnat[ZZ[i]]; fbw_byte(&b,(uint8_t)(q>>8)); fbw_byte(&b,(uint8_t)q); }
    emit_dht(&b,0,0,&hdc0); emit_dht(&b,1,0,&hac0);
    emit_dht(&b,0,1,&hdc1); emit_dht(&b,1,1,&hac1);
    fbw_byte(&b,0xFF); fbw_byte(&b,0xC1);                               /* SOF1 */
    fbw_byte(&b,0); fbw_byte(&b,8+3*2); fbw_byte(&b,12);
    fbw_byte(&b,(uint8_t)(h>>8)); fbw_byte(&b,(uint8_t)h);
    fbw_byte(&b,(uint8_t)(w>>8)); fbw_byte(&b,(uint8_t)w);
    fbw_byte(&b,2);
    fbw_byte(&b,0); fbw_byte(&b,0x11); fbw_byte(&b,0);
    fbw_byte(&b,1); fbw_byte(&b,0x11); fbw_byte(&b,0);
    fbw_byte(&b,0xFF); fbw_byte(&b,0xDA); fbw_byte(&b,0); fbw_byte(&b,8); /* scan 0 */
    fbw_byte(&b,1); fbw_byte(&b,0); fbw_byte(&b,0x00); fbw_byte(&b,0); fbw_byte(&b,63); fbw_byte(&b,0);
    if (reuse) collect_emit(comp0, w, h, stride0, qnat, &hdc0, &hac0, &b);
    else       emit_scan(&b, c0, m0, nblk, &hdc0, &hac0);
    fbw_finish(&b);
    fbw_byte(&b,0xFF); fbw_byte(&b,0xDA); fbw_byte(&b,0); fbw_byte(&b,8); /* scan 1 */
    fbw_byte(&b,1); fbw_byte(&b,1); fbw_byte(&b,0x11); fbw_byte(&b,0); fbw_byte(&b,63); fbw_byte(&b,0);
    if (reuse) collect_emit(comp1, w, h, stride1, qnat, &hdc1, &hac1, &b);
    else       emit_scan(&b, c1, m1, nblk, &hdc1, &hac1);
    fbw_finish(&b);
    fbw_byte(&b,0xFF); fbw_byte(&b,0xD9);                               /* EOI */

    if (b.oom) return -1;
    ctx->out = b.base; ctx->out_cap = b.cap;     /* fbw_reserve may have grown it */
    *out = b.base; *out_len = (size_t)(b.p - b.base);
    return 0;
}

int lj12_encode_tile_ctx(lj12_ctx *ctx,
                         const uint16_t *comp0, const uint16_t *comp1,
                         int w, int h, int stride0, int stride1,
                         const uint16_t qnat[64], const uint8_t **out, size_t *out_len)
{
    return lj12_encode_tile_reuse(ctx, comp0, comp1, w, h, stride0, stride1,
                                  qnat, NULL, out, out_len);
}

/* Convenience wrapper: allocates a throwaway context. Tests only -- a recording
 * loop must hold a context per worker or it pays the page faults every frame. */
int lj12_encode_tile(const uint16_t *comp0, const uint16_t *comp1,
                     int w, int h, int stride0, int stride1,
                     const uint16_t qnat[64], uint8_t **out, size_t *out_len)
{
    lj12_ctx *c = lj12_ctx_create();
    if (!c) return -1;
    const uint8_t *p = NULL; size_t n = 0;
    int rc = lj12_encode_tile_ctx(c, comp0, comp1, w, h, stride0, stride1, qnat, &p, &n);
    if (rc == 0) {
        uint8_t *copy = (uint8_t*)malloc(n);
        if (!copy) rc = -1; else { memcpy(copy, p, n); *out = copy; *out_len = n; }
    }
    lj12_ctx_destroy(c);
    return rc;
}
