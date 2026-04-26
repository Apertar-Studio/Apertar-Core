#include "cdng/CdngEncoder.hpp"

#include "cdng/IfdBuilder.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <unistd.h>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define APERTARCORE_HAS_NEON 1
#else
#define APERTARCORE_HAS_NEON 0
#endif

namespace apertar {
namespace {

constexpr int32_t kDngBlackLevel = 206;

uint32_t packedRowBytes(uint32_t width, RawBitDepth bitDepth)
{
    return static_cast<uint32_t>((static_cast<uint64_t>(width) * static_cast<unsigned int>(bitDepth) + 7u) / 8u);
}

uint16_t bitsPerSample(RawBitDepth bitDepth)
{
    return static_cast<uint16_t>(bitDepth);
}

uint32_t whiteLevel(RawBitDepth bitDepth)
{
    const unsigned int bits = static_cast<unsigned int>(bitDepth);
    return bits >= 16 ? 65535u : ((1u << bits) - 1u);
}

const uint8_t *cfaPattern(BayerPattern pattern)
{
    static constexpr uint8_t rggb[4] = { 0, 1, 1, 2 };
    static constexpr uint8_t grbg[4] = { 1, 0, 2, 1 };
    static constexpr uint8_t gbrg[4] = { 1, 2, 0, 1 };
    static constexpr uint8_t bggr[4] = { 2, 1, 1, 0 };

    switch (pattern) {
    case BayerPattern::RGGB: return rggb;
    case BayerPattern::GRBG: return grbg;
    case BayerPattern::GBRG: return gbrg;
    case BayerPattern::BGGR: return bggr;
    }

    return rggb;
}

struct Matrix {
    float m[9]{};

    Matrix() = default;
    Matrix(float m0, float m1, float m2,
           float m3, float m4, float m5,
           float m6, float m7, float m8)
    {
        m[0] = m0; m[1] = m1; m[2] = m2;
        m[3] = m3; m[4] = m4; m[5] = m5;
        m[6] = m6; m[7] = m7; m[8] = m8;
    }

    Matrix transpose() const
    {
        return Matrix(m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]);
    }

    Matrix cofactors() const
    {
        return Matrix(
            m[4] * m[8] - m[5] * m[7],
           -(m[3] * m[8] - m[5] * m[6]),
            m[3] * m[7] - m[4] * m[6],
           -(m[1] * m[8] - m[2] * m[7]),
            m[0] * m[8] - m[2] * m[6],
           -(m[0] * m[7] - m[1] * m[6]),
            m[1] * m[5] - m[2] * m[4],
           -(m[0] * m[5] - m[2] * m[3]),
            m[0] * m[4] - m[1] * m[3]);
    }

    float determinant() const
    {
        return m[0] * (m[4] * m[8] - m[5] * m[7]) -
               m[1] * (m[3] * m[8] - m[5] * m[6]) +
               m[2] * (m[3] * m[7] - m[4] * m[6]);
    }

    Matrix inverse() const
    {
        const float det = determinant();
        if (std::fabs(det) < 1e-9f)
            return Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);

        Matrix adj = cofactors().transpose();
        for (float &value : adj.m)
            value /= det;
        return adj;
    }

    Matrix operator*(const Matrix &other) const
    {
        Matrix result;
        for (int y = 0; y < 3; ++y) {
            for (int x = 0; x < 3; ++x) {
                result.m[y * 3 + x] =
                    m[y * 3 + 0] * other.m[x + 0] +
                    m[y * 3 + 1] * other.m[x + 3] +
                    m[y * 3 + 2] * other.m[x + 6];
            }
        }
        return result;
    }
};

Matrix cameraToXyzMatrix(const CdngEncoder::Metadata &metadata)
{
    Matrix ccm(
        metadata.colourMatrix[0], metadata.colourMatrix[1], metadata.colourMatrix[2],
        metadata.colourMatrix[3], metadata.colourMatrix[4], metadata.colourMatrix[5],
        metadata.colourMatrix[6], metadata.colourMatrix[7], metadata.colourMatrix[8]);

    Matrix rgbToXyz(
        0.4124564f, 0.3575761f, 0.1804375f,
        0.2126729f, 0.7151522f, 0.0721750f,
        0.0193339f, 0.1191920f, 0.9503041f);

    return (rgbToXyz * ccm).inverse();
}

void encodeRationalArray(const float *src, int count, int32_t *dst, int32_t scale = 10000)
{
    for (int i = 0; i < count; ++i) {
        const float value = src[i];
        if (!std::isfinite(value)) {
            dst[i * 2 + 0] = 0;
            dst[i * 2 + 1] = 1;
        } else {
            dst[i * 2 + 0] = static_cast<int32_t>(std::round(value * scale));
            dst[i * 2 + 1] = scale;
        }
    }
}

void appendAscii(tiff::IfdBuilder &ifd, uint16_t tag, const std::string &value)
{
    std::string withTerminator = value;
    if (withTerminator.empty() || withTerminator.back() != '\0')
        withTerminator.push_back('\0');
    ifd.add(tag, tiff::Ascii, static_cast<uint32_t>(withTerminator.size()), withTerminator.data());
}

std::string dateTimeString(uint64_t timestampUs)
{
    const std::time_t seconds = static_cast<std::time_t>(timestampUs / 1000000ull);
    std::tm localTime{};
#if defined(_WIN32)
    localtime_s(&localTime, &seconds);
#else
    localtime_r(&seconds, &localTime);
#endif

    char buffer[20]{};
    std::strftime(buffer, sizeof(buffer), "%Y:%m:%d %H:%M:%S", &localTime);
    return std::string(buffer);
}

std::array<uint8_t, 8> timeCode(uint64_t timestampUs, double fps)
{
    const std::time_t seconds = static_cast<std::time_t>(timestampUs / 1000000ull);
    std::tm localTime{};
#if defined(_WIN32)
    localtime_s(&localTime, &seconds);
#else
    localtime_r(&seconds, &localTime);
#endif

    const double safeFps = fps > 0.0 ? fps : 24.0;
    const int frame = static_cast<int>(((timestampUs % 1000000ull) * safeFps) / 1000000.0);
    auto bcd = [](int value) {
        return static_cast<uint8_t>(((value / 10) << 4) | (value % 10));
    };

    return {
        bcd(frame),
        bcd(localTime.tm_sec),
        bcd(localTime.tm_min),
        bcd(localTime.tm_hour),
        0, 0, 0, 0,
    };
}

uint32_t checkedTiffOffset(uint64_t value)
{
    if (value > std::numeric_limits<uint32_t>::max())
        throw std::runtime_error("Classic TIFF offset overflow.");
    return static_cast<uint32_t>(value);
}

uint64_t align4(uint64_t value)
{
    return (value + 3u) & ~uint64_t{3u};
}

void putLe16(uint8_t *dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value & 0xffu);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xffu);
}

void putLe32(uint8_t *dst, uint32_t value)
{
    dst[0] = static_cast<uint8_t>(value & 0xffu);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xffu);
    dst[2] = static_cast<uint8_t>((value >> 16) & 0xffu);
    dst[3] = static_cast<uint8_t>((value >> 24) & 0xffu);
}

uint32_t readLe32(const uint8_t *src)
{
    return static_cast<uint32_t>(src[0]) |
           (static_cast<uint32_t>(src[1]) << 8) |
           (static_cast<uint32_t>(src[2]) << 16) |
           (static_cast<uint32_t>(src[3]) << 24);
}

iovec makeIov(const void *data, size_t size)
{
    iovec out{};
    out.iov_base = const_cast<void *>(data);
    out.iov_len = size;
    return out;
}

bool writeAllVectored(int fd, std::vector<iovec> iovecs)
{
    size_t index = 0;
    while (index < iovecs.size()) {
        const ssize_t written = ::writev(fd,
                                         iovecs.data() + index,
                                         static_cast<int>(iovecs.size() - index));
        if (written < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }
        if (written == 0) {
            errno = EIO;
            return false;
        }

        size_t remaining = static_cast<size_t>(written);
        while (index < iovecs.size() && remaining >= iovecs[index].iov_len) {
            remaining -= iovecs[index].iov_len;
            ++index;
        }

        if (remaining > 0 && index < iovecs.size()) {
            auto *base = static_cast<uint8_t *>(iovecs[index].iov_base);
            iovecs[index].iov_base = base + remaining;
            iovecs[index].iov_len -= remaining;
        }
    }

    return true;
}

void packCsi2Raw10ToDngRow(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    uint32_t x = 0;
    for (; x + 3 < width; x += 4, src += 5, dst += 5) {
        const uint8_t s0 = src[0];
        const uint8_t s1 = src[1];
        const uint8_t s2 = src[2];
        const uint8_t s3 = src[3];
        const uint8_t low = src[4];

        dst[0] = s0;
        dst[1] = static_cast<uint8_t>(((low & 0x03u) << 6) | (s1 >> 2));
        dst[2] = static_cast<uint8_t>(((s1 & 0x03u) << 6) | (((low >> 2) & 0x03u) << 4) | (s2 >> 4));
        dst[3] = static_cast<uint8_t>(((s2 & 0x0fu) << 4) | (((low >> 4) & 0x03u) << 2) | (s3 >> 6));
        dst[4] = static_cast<uint8_t>(((s3 & 0x3fu) << 2) | ((low >> 6) & 0x03u));
    }

    if (x < width) {
        std::array<uint16_t, 4> pixels{};
        std::array<uint8_t, 5> packed{};
        Csi2RawConverter::unpack10Row(src, pixels.data(), width - x);
        Csi2RawConverter::packDng10Row(pixels.data(), packed.data(), width - x);
        std::memcpy(dst, packed.data(), packedRowBytes(width - x, RawBitDepth::Bits10));
    }
}

void packCsi2Raw12ToDngRow(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    uint32_t x = 0;
    for (; x + 1 < width; x += 2, src += 3, dst += 3) {
        const uint8_t s0 = src[0];
        const uint8_t s1 = src[1];
        const uint8_t low = src[2];

        dst[0] = s0;
        dst[1] = static_cast<uint8_t>(((low & 0x0fu) << 4) | (s1 >> 4));
        dst[2] = static_cast<uint8_t>(((s1 & 0x0fu) << 4) | (low >> 4));
    }

    if (x < width) {
        std::array<uint16_t, 2> pixels{};
        std::array<uint8_t, 3> packed{};
        Csi2RawConverter::unpack12Row(src, pixels.data(), width - x);
        Csi2RawConverter::packDng12Row(pixels.data(), packed.data(), width - x);
        std::memcpy(dst, packed.data(), packedRowBytes(width - x, RawBitDepth::Bits12));
    }
}

void pack16ToDng12Row(const uint16_t *src, uint8_t *dst, uint32_t width)
{
#if APERTARCORE_HAS_NEON
    uint32_t x = 0;
    for (; x + 15 < width; x += 16) {
        const uint16_t *block = src + x;
        uint8_t *out = dst + (static_cast<size_t>(x) / 2) * 3;
        const uint16x8_t lo = vld1q_u16(block);
        const uint16x8_t hi = vld1q_u16(block + 8);
        const uint16x8x2_t evenOdd = vuzpq_u16(lo, hi);

        const uint16x8_t even12 = vshrq_n_u16(evenOdd.val[0], 4);
        const uint16x8_t odd12 = vshrq_n_u16(evenOdd.val[1], 4);

        uint8x8x3_t packed;
        packed.val[0] = vmovn_u16(vshrq_n_u16(even12, 4));
        packed.val[1] = vorr_u8(
            vshl_n_u8(vmovn_u16(vandq_u16(even12, vdupq_n_u16(0x000f))), 4),
            vmovn_u16(vshrq_n_u16(odd12, 8)));
        packed.val[2] = vmovn_u16(vandq_u16(odd12, vdupq_n_u16(0x00ff)));

        vst3_u8(out, packed);
    }

    for (; x < width; x += 2) {
#else
    for (uint32_t x = 0; x < width; x += 2) {
#endif
        const uint16_t p0 = static_cast<uint16_t>(src[x] >> 4);
        const uint16_t p1 = x + 1 < width ? static_cast<uint16_t>(src[x + 1] >> 4) : 0;
        uint8_t *out = dst + (static_cast<size_t>(x) / 2) * 3;

        out[0] = static_cast<uint8_t>(p0 >> 4);
        out[1] = static_cast<uint8_t>((p0 << 4) | (p1 >> 8));
        out[2] = static_cast<uint8_t>(p1);
    }
}

constexpr uint16_t kPispCompressOffset = 2048;
constexpr uint8_t kPispCompressMode = 1;

uint16_t pispPostprocess(uint16_t value)
{
    if (kPispCompressMode & 2) {
        if (kPispCompressMode == 3 && value < 0x4000)
            value = value >> 2;
        else if (value < 0x1000)
            value = value >> 4;
        else if (value < 0x1800)
            value = (value - 0x800) >> 3;
        else if (value < 0x3000)
            value = (value - 0x1000) >> 2;
        else if (value < 0x6000)
            value = (value - 0x2000) >> 1;
        else if (value < 0xC000)
            value = (value - 0x4000);
        else
            value = 2 * (value - 0x8000);
    }

    return static_cast<uint16_t>(std::min<int>(0xffff, value + kPispCompressOffset));
}

constexpr size_t kPispDequantLutSize = 1024;

uint16_t pispDequantizeScalar(uint16_t q, int qmode)
{
    switch (qmode) {
    case 0:
        return static_cast<uint16_t>((q < 320) ? (16 * q) : (32 * (q - 160)));
    case 1:
        return static_cast<uint16_t>(std::min<uint32_t>(65535u, 64u * q));
    case 2:
        return static_cast<uint16_t>(std::min<uint32_t>(65535u, 128u * q));
    default:
        return static_cast<uint16_t>((q < 94) ? (256 * q) : std::min<uint32_t>(65535u, 512u * (q - 47)));
    }
}

std::array<uint16_t, kPispDequantLutSize> buildPispDequantLut(int qmode)
{
    std::array<uint16_t, kPispDequantLutSize> lut{};
    for (size_t i = 0; i < lut.size(); ++i)
        lut[i] = pispDequantizeScalar(static_cast<uint16_t>(i), qmode);
    return lut;
}

const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode0 = buildPispDequantLut(0);
const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode1 = buildPispDequantLut(1);
const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode2 = buildPispDequantLut(2);
const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode3 = buildPispDequantLut(3);

inline uint16_t pispDequantizeFast(int q, int qmode)
{
    const size_t idx = static_cast<size_t>(std::clamp(q, 0, static_cast<int>(kPispDequantLutSize - 1)));
    switch (qmode) {
    case 0: return kPispDequantMode0[idx];
    case 1: return kPispDequantMode1[idx];
    case 2: return kPispDequantMode2[idx];
    default: return kPispDequantMode3[idx];
    }
}

inline void applyPispOffset(uint16_t *dst)
{
#if APERTARCORE_HAS_NEON
    uint16x8_t values = vld1q_u16(dst);
    values = vqaddq_u16(values, vdupq_n_u16(kPispCompressOffset));
    vst1q_u16(dst, values);
#else
    for (int i = 0; i < 8; ++i)
        dst[i] = pispPostprocess(dst[i]);
#endif
}

void pispSubBlock(uint16_t *dst, uint32_t word)
{
    int q[4]{};
    const int qmode = word & 3;
    if (qmode < 3) {
        const int field0 = (word >> 2) & 511;
        const int field1 = (word >> 11) & 127;
        const int field2 = (word >> 18) & 127;
        const int field3 = (word >> 25) & 127;
        if (qmode == 2 && field0 >= 384) {
            q[1] = field0;
            q[2] = field1 + 384;
        } else {
            q[1] = (field1 >= 64) ? field0 : field0 + 64 - field1;
            q[2] = (field1 >= 64) ? field0 + field1 - 64 : field0;
        }
        int p1 = std::max(0, q[1] - 64);
        if (qmode == 2)
            p1 = std::min(384, p1);
        int p2 = std::max(0, q[2] - 64);
        if (qmode == 2)
            p2 = std::min(384, p2);
        q[0] = p1 + field2;
        q[3] = p2 + field3;
    } else {
        const int pack0 = (word >> 2) & 32767;
        const int pack1 = (word >> 17) & 32767;
        q[0] = (pack0 & 15) + 16 * ((pack0 >> 8) / 11);
        q[1] = (pack0 >> 4) % 176;
        q[2] = (pack1 & 15) + 16 * ((pack1 >> 8) / 11);
        q[3] = (pack1 >> 4) % 176;
    }

    dst[0] = pispDequantizeFast(q[0], qmode);
    dst[2] = pispDequantizeFast(q[1], qmode);
    dst[4] = pispDequantizeFast(q[2], qmode);
    dst[6] = pispDequantizeFast(q[3], qmode);
}

void decodePispComp1BlockToWorking(uint32_t w0, uint32_t w1, uint16_t *dst)
{
    pispSubBlock(dst, w0);
    pispSubBlock(dst + 1, w1);
    applyPispOffset(dst);
}

void unpackPispCompressedRowToPacked12(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    const uint32_t fullBlocks = width / 8u;
    uint32_t x = 0;

    for (uint32_t block = 0; block < fullBlocks; ++block, x += 8u, src += 8u) {
        uint16_t working[8];
        if (kPispCompressMode & 1) {
            decodePispComp1BlockToWorking(readLe32(src), readLe32(src + 4), working);
        } else {
            for (int i = 0; i < 8; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
        }
        pack16ToDng12Row(working, dst + (static_cast<size_t>(x) / 2u) * 3u, 8u);
    }

    const uint32_t remaining = width - x;
    if (remaining > 0) {
        uint16_t working[8]{};
        uint8_t packed[12]{};

        if (kPispCompressMode & 1)
            decodePispComp1BlockToWorking(readLe32(src), readLe32(src + 4), working);
        else {
            for (uint32_t i = 0; i < remaining; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
        }

        pack16ToDng12Row(working, packed, remaining);
        std::memcpy(dst + (static_cast<size_t>(x) / 2u) * 3u,
                    packed,
                    packedRowBytes(remaining, RawBitDepth::Bits12));
    }
}

void requireFrame(const RawFrameView &frame)
{
    if (!frame.data)
        throw std::runtime_error("Raw frame has no data.");
    if (frame.width == 0 || frame.height == 0 || frame.stride == 0)
        throw std::runtime_error("Raw frame has invalid geometry.");
}

} // namespace

void Csi2RawConverter::unpack10Row(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    const uint32_t alignedWidth = width & ~3u;
    uint32_t x = 0;

    for (; x < alignedWidth; x += 4, src += 5) {
        dst[x + 0] = (static_cast<uint16_t>(src[0]) << 2) | ((src[4] >> 0) & 0x3);
        dst[x + 1] = (static_cast<uint16_t>(src[1]) << 2) | ((src[4] >> 2) & 0x3);
        dst[x + 2] = (static_cast<uint16_t>(src[2]) << 2) | ((src[4] >> 4) & 0x3);
        dst[x + 3] = (static_cast<uint16_t>(src[3]) << 2) | ((src[4] >> 6) & 0x3);
    }

    if (x < width) {
        const uint8_t tail = src[4];
        for (uint32_t i = 0; x < width; ++x, ++i)
            dst[x] = (static_cast<uint16_t>(src[i]) << 2) | ((tail >> (i * 2)) & 0x3);
    }
}

void Csi2RawConverter::unpack12Row(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    const uint32_t alignedWidth = width & ~1u;
    uint32_t x = 0;

    for (; x < alignedWidth; x += 2, src += 3) {
        dst[x + 0] = (static_cast<uint16_t>(src[0]) << 4) | ((src[2] >> 0) & 0xF);
        dst[x + 1] = (static_cast<uint16_t>(src[1]) << 4) | ((src[2] >> 4) & 0xF);
    }

    if (x < width)
        dst[x] = (static_cast<uint16_t>(src[0]) << 4) | ((src[2] >> 0) & 0xF);
}

void Csi2RawConverter::packDng10Row(const uint16_t *src, uint8_t *dst, uint32_t width)
{
    uint32_t x = 0;
    for (; x < width; x += 4) {
        const uint16_t p0 = src[x + 0] & 0x03FF;
        const uint16_t p1 = x + 1 < width ? (src[x + 1] & 0x03FF) : 0;
        const uint16_t p2 = x + 2 < width ? (src[x + 2] & 0x03FF) : 0;
        const uint16_t p3 = x + 3 < width ? (src[x + 3] & 0x03FF) : 0;

        dst[0] = static_cast<uint8_t>(p0 >> 2);
        dst[1] = static_cast<uint8_t>((p0 << 6) | (p1 >> 4));
        dst[2] = static_cast<uint8_t>((p1 << 4) | (p2 >> 6));
        dst[3] = static_cast<uint8_t>((p2 << 2) | (p3 >> 8));
        dst[4] = static_cast<uint8_t>(p3);
        dst += 5;
    }
}

void Csi2RawConverter::packDng12Row(const uint16_t *src, uint8_t *dst, uint32_t width)
{
    for (uint32_t x = 0; x < width; x += 2) {
        const uint16_t p0 = src[x] & 0x0FFF;
        const uint16_t p1 = x + 1 < width ? (src[x + 1] & 0x0FFF) : 0;

        dst[0] = static_cast<uint8_t>(p0 >> 4);
        dst[1] = static_cast<uint8_t>((p0 << 4) | (p1 >> 8));
        dst[2] = static_cast<uint8_t>(p1);
        dst += 3;
    }
}

CdngEncoder::EncodedRawPayload CdngEncoder::prepareRawPayload(const RawFrameView &frame) const
{
    requireFrame(frame);

    if (frame.packing == PixelPacking::Csi2Packed)
        return preparePackedCsi2Payload(frame);
    if (frame.packing == PixelPacking::PispCompressed1)
        return preparePispCompressedPayload(frame);

    return prepareUnpackedPayload(frame);
}

std::vector<uint8_t> CdngEncoder::takePayloadScratch(size_t size) const
{
    std::vector<uint8_t> bytes = std::move(payloadScratch_);
    bytes.resize(size);
    return bytes;
}

void CdngEncoder::recyclePayloadScratch(std::vector<uint8_t> bytes) const
{
    bytes.clear();
    payloadScratch_ = std::move(bytes);
}

void CdngEncoder::writeDngFile(const std::string &path,
                               const RawFrameView &frame,
                               const Metadata &metadata) const
{
    requireFrame(frame);

    EncodedRawPayload payload = prepareRawPayload(frame);
    if (payload.bytes.empty())
        throw std::runtime_error("Encoded raw payload is empty.");

    const uint16_t bits = bitsPerSample(payload.bitDepth);
    const uint16_t compression = 1;
    const uint16_t photometricCfa = 32803;
    const uint16_t samplesPerPixel = 1;
    const uint16_t planarConfig = 1;
    const uint16_t sampleFormatUint = 1;
    const uint16_t orientationTopLeft = 1;
    const uint32_t imageWidth = frame.width;
    const uint32_t imageHeight = frame.height;
    const uint32_t rowsPerStrip = frame.height;
    const uint32_t white = whiteLevel(payload.bitDepth);

    if (payload.bytes.size() > 0xffffffffu)
        throw std::runtime_error("DNG payload is too large for classic TIFF.");

    constexpr uint32_t tiffHeaderBytes = 8;
    const uint32_t stripOffset = tiffHeaderBytes;
    const uint32_t stripByteCount = checkedTiffOffset(payload.bytes.size());
    const uint64_t ifdOffset64 = align4(static_cast<uint64_t>(stripOffset) + payload.bytes.size());
    const uint32_t ifdOffset = checkedTiffOffset(ifdOffset64);
    const size_t paddingSize = static_cast<size_t>(ifdOffset64 - (static_cast<uint64_t>(stripOffset) + payload.bytes.size()));

    const uint64_t wallClock = metadata.wallClockTimestampUs
        ? metadata.wallClockTimestampUs
        : static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::system_clock::now().time_since_epoch()).count());
    const std::string originalDate = dateTimeString(wallClock);

    const std::array<uint8_t, 8> tc = timeCode(wallClock, metadata.fps);

    int32_t fpsRat[2] = {
        static_cast<int32_t>(std::round(std::max(1.0, metadata.fps) * 1000.0)),
        1000,
    };
    const uint32_t exifExposure[2] = {
        std::max<uint32_t>(1u, metadata.shutterUs),
        1000000u
    };
    const uint16_t exifIso = static_cast<uint16_t>(std::clamp<uint32_t>(metadata.iso, 1u, 65535u));

    auto buildMainIfdBuffer = [&](uint32_t exifIfdOffset) {
        tiff::IfdBuilder ifd;
        const uint32_t typeFullResolution = 0;
        ifd.add(254, tiff::Long, 1, &typeFullResolution);
        ifd.add(256, tiff::Long, 1, &imageWidth);
        ifd.add(257, tiff::Long, 1, &imageHeight);
        ifd.add(258, tiff::Short, 1, &bits);
        ifd.add(259, tiff::Short, 1, &compression);
        ifd.add(262, tiff::Short, 1, &photometricCfa);
        ifd.add(273, tiff::Long, 1, &stripOffset);
        ifd.add(274, tiff::Short, 1, &orientationTopLeft);
        ifd.add(277, tiff::Short, 1, &samplesPerPixel);
        ifd.add(278, tiff::Long, 1, &rowsPerStrip);
        ifd.add(279, tiff::Long, 1, &stripByteCount);
        ifd.add(284, tiff::Short, 1, &planarConfig);
        ifd.add(339, tiff::Short, 1, &sampleFormatUint);
        ifd.add(0x8769, tiff::Long, 1, &exifIfdOffset); // ExifIFD pointer

        appendAscii(ifd, 271, metadata.make);
        appendAscii(ifd, 272, metadata.model);
        appendAscii(ifd, 305, metadata.software);
        appendAscii(ifd, 306, dateTimeString(wallClock));

        static constexpr uint8_t dngVersion[4] = { 1, 4, 0, 0 };
        static constexpr uint8_t dngBackwardVersion[4] = { 1, 3, 0, 0 };
        ifd.add(0xC612, tiff::Byte, 4, dngVersion);
        ifd.add(0xC613, tiff::Byte, 4, dngBackwardVersion);
        appendAscii(ifd, 0xC614, metadata.uniqueCameraModel);
        if (!metadata.serial.empty())
            appendAscii(ifd, 0xC62F, metadata.serial);

        const uint16_t repeatDim[2] = { 2, 2 };
        ifd.add(0x828D, tiff::Short, 2, repeatDim);
        ifd.add(0x828E, tiff::Byte, 4, cfaPattern(frame.bayerPattern));
        ifd.add(0xC619, tiff::Short, 2, repeatDim);

        int32_t blackLevels[8]{};
        for (int i = 0; i < 4; ++i) {
            const int32_t blackLevel = metadata.hasBlackLevels
                    ? static_cast<int32_t>(std::lround(metadata.blackLevels[i]))
                    : kDngBlackLevel;
            blackLevels[i * 2 + 0] = blackLevel;
            blackLevels[i * 2 + 1] = 1;
        }
        ifd.add(0xC61A, tiff::Rational, 4, blackLevels);
        ifd.add(0xC61D, tiff::Long, 1, &white);

        Matrix cameraToXyz = cameraToXyzMatrix(metadata);
        int32_t colorMatrix[18]{};
        encodeRationalArray(cameraToXyz.m, 9, colorMatrix);
        ifd.add(0xC621, tiff::SRational, 9, colorMatrix);
        ifd.add(0xC622, tiff::SRational, 9, colorMatrix);

        const uint16_t illuminantD65 = 21;
        ifd.add(0xC65A, tiff::Short, 1, &illuminantD65);
        ifd.add(0xC65B, tiff::Short, 1, &illuminantD65);

        float neutralValues[3] = {
            1.0f / std::max(metadata.colourGains[0], 0.0001f),
            1.0f,
            1.0f / std::max(metadata.colourGains[1], 0.0001f),
        };
        int32_t neutral[6]{};
        encodeRationalArray(neutralValues, 3, neutral);
        ifd.add(0xC628, tiff::Rational, 3, neutral);

        appendAscii(ifd, 0x9003, originalDate);
        ifd.add(0xC763, tiff::Byte, static_cast<uint32_t>(tc.size()), tc.data());
        ifd.add(0xC764, tiff::SRational, 1, fpsRat);

        tiff::MemoryBuffer buffer;
        buffer.bytes.reserve(16 * 1024);
        const uint32_t builtIfdOffset = ifd.buildAt(buffer, ifdOffset);
        if (builtIfdOffset != ifdOffset)
            throw std::runtime_error("DNG IFD offset mismatch.");
        return buffer;
    };

    tiff::MemoryBuffer mainIfd = buildMainIfdBuffer(0);
    const uint32_t exifIfdOffset = checkedTiffOffset(align4(static_cast<uint64_t>(ifdOffset) + mainIfd.bytes.size()));
    mainIfd = buildMainIfdBuffer(exifIfdOffset);

    const uint32_t exifIfdOffsetRecomputed = checkedTiffOffset(align4(static_cast<uint64_t>(ifdOffset) + mainIfd.bytes.size()));
    if (exifIfdOffsetRecomputed != exifIfdOffset)
        throw std::runtime_error("ExifIFD offset changed after rebuild.");

    tiff::IfdBuilder exifIfd;
    exifIfd.add(0x829A, tiff::Rational, 1, exifExposure); // ExposureTime
    exifIfd.add(0x8827, tiff::Short, 1, &exifIso);        // PhotographicSensitivity
    appendAscii(exifIfd, 0x9003, originalDate);           // DateTimeOriginal
    exifIfd.add(0xA002, tiff::Long, 1, &imageWidth);      // PixelXDimension
    exifIfd.add(0xA003, tiff::Long, 1, &imageHeight);     // PixelYDimension

    tiff::MemoryBuffer exifBuffer;
    exifBuffer.bytes.reserve(512);
    const uint32_t builtExifOffset = exifIfd.buildAt(exifBuffer, exifIfdOffset);
    if (builtExifOffset != exifIfdOffset)
        throw std::runtime_error("ExifIFD offset mismatch.");

    tiff::MemoryBuffer ifdBuffer;
    ifdBuffer.bytes = std::move(mainIfd.bytes);
    const size_t exifRelativeOffset = static_cast<size_t>(exifIfdOffset - ifdOffset);
    if (ifdBuffer.bytes.size() < exifRelativeOffset)
        ifdBuffer.bytes.resize(exifRelativeOffset, 0);
    ifdBuffer.bytes.insert(ifdBuffer.bytes.end(), exifBuffer.bytes.begin(), exifBuffer.bytes.end());

    std::array<uint8_t, 8> header{};
    header[0] = 'I';
    header[1] = 'I';
    putLe16(header.data() + 2, 42);
    putLe32(header.data() + 4, ifdOffset);

    static constexpr std::array<uint8_t, 3> padding{ 0, 0, 0 };
    std::vector<iovec> writes;
    writes.reserve(4);
    writes.push_back(makeIov(header.data(), header.size()));
    writes.push_back(makeIov(payload.bytes.data(), payload.bytes.size()));
    if (paddingSize > 0)
        writes.push_back(makeIov(padding.data(), paddingSize));
    writes.push_back(makeIov(ifdBuffer.bytes.data(), ifdBuffer.bytes.size()));

    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    int openFlags = O_WRONLY | O_CREAT | O_TRUNC;
#ifdef O_CLOEXEC
    openFlags |= O_CLOEXEC;
#endif
    const int fd = ::open(path.c_str(), openFlags, 0644);
    if (fd < 0)
        throw std::system_error(errno, std::generic_category(), "Failed to open DNG for writing: " + path);

    ::posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);
    const bool ok = writeAllVectored(fd, std::move(writes));
    const int writeErrno = ok ? 0 : errno;
    ::posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
    ::close(fd);

    recyclePayloadScratch(std::move(payload.bytes));

    if (!ok)
        throw std::system_error(writeErrno, std::generic_category(), "Failed to write DNG: " + path);
}

CdngEncoder::EncodedRawPayload CdngEncoder::preparePackedCsi2Payload(const RawFrameView &frame) const
{
    if (frame.bitDepth != RawBitDepth::Bits10 && frame.bitDepth != RawBitDepth::Bits12)
        throw std::runtime_error("CSI2 packed conversion supports only 10-bit and 12-bit raw.");

    EncodedRawPayload payload;
    payload.bitDepth = frame.bitDepth;
    payload.packing = PixelPacking::Csi2Packed;
    payload.rowBytes = packedRowBytes(frame.width, frame.bitDepth);
    payload.bytes = takePayloadScratch(static_cast<size_t>(payload.rowBytes) * frame.height);

    for (uint32_t y = 0; y < frame.height; ++y) {
        const uint8_t *src = frame.data + static_cast<size_t>(y) * frame.stride;
        uint8_t *dst = payload.bytes.data() + static_cast<size_t>(y) * payload.rowBytes;

        if (frame.bitDepth == RawBitDepth::Bits10)
            packCsi2Raw10ToDngRow(src, dst, frame.width);
        else
            packCsi2Raw12ToDngRow(src, dst, frame.width);
    }

    return payload;
}

CdngEncoder::EncodedRawPayload CdngEncoder::preparePispCompressedPayload(const RawFrameView &frame) const
{
    if (frame.bitDepth != RawBitDepth::Bits16)
        throw std::runtime_error("PiSP COMP1 conversion expects a 16-bit output domain.");

    EncodedRawPayload payload;
    // PiSP COMP1 expands into a 16-bit working domain, but the IMX585 source
    // mode is 12-bit. Store packed 12-bit DNG frames to keep media bandwidth
    // and decoder overhead close to the camera's real signal depth.
    payload.bitDepth = RawBitDepth::Bits12;
    payload.packing = PixelPacking::Csi2Packed;
    payload.rowBytes = packedRowBytes(frame.width, payload.bitDepth);
    payload.bytes = takePayloadScratch(static_cast<size_t>(payload.rowBytes) * frame.height);

    for (uint32_t y = 0; y < frame.height; ++y) {
        const uint8_t *src = frame.data + static_cast<size_t>(y) * frame.stride;
        uint8_t *dst = payload.bytes.data() + static_cast<size_t>(y) * payload.rowBytes;

        unpackPispCompressedRowToPacked12(src, dst, frame.width);
    }

    return payload;
}

CdngEncoder::EncodedRawPayload CdngEncoder::prepareUnpackedPayload(const RawFrameView &frame) const
{
    EncodedRawPayload payload;
    payload.bitDepth = frame.bitDepth;
    payload.packing = PixelPacking::Unpacked;

    if (frame.bitDepth == RawBitDepth::Bits16) {
        payload.rowBytes = frame.width * 2;
        payload.bytes = takePayloadScratch(static_cast<size_t>(payload.rowBytes) * frame.height);

        for (uint32_t y = 0; y < frame.height; ++y) {
            const uint8_t *src = frame.data + static_cast<size_t>(y) * frame.stride;
            uint8_t *dst = payload.bytes.data() + static_cast<size_t>(y) * payload.rowBytes;
            std::memcpy(dst, src, payload.rowBytes);
        }

        return payload;
    }

    payload.rowBytes = packedRowBytes(frame.width, frame.bitDepth);
    payload.bytes = takePayloadScratch(static_cast<size_t>(payload.rowBytes) * frame.height);

    for (uint32_t y = 0; y < frame.height; ++y) {
        const auto *src = reinterpret_cast<const uint16_t *>(frame.data + static_cast<size_t>(y) * frame.stride);
        uint8_t *dst = payload.bytes.data() + static_cast<size_t>(y) * payload.rowBytes;

        if (frame.bitDepth == RawBitDepth::Bits10)
            Csi2RawConverter::packDng10Row(src, dst, frame.width);
        else if (frame.bitDepth == RawBitDepth::Bits12)
            Csi2RawConverter::packDng12Row(src, dst, frame.width);
        else
            throw std::runtime_error("Unsupported unpacked bit depth.");
    }

    return payload;
}

} // namespace apertar
