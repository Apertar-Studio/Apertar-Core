#include "mlv/MlvEncoder.hpp"
#include "audio/RecordingClock.hpp"
#include "color/ColorProfileConfig.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <system_error>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

// The RAW12 scalar packer is deliberately the default. The NEON table-shuffle
// version is optional because PiSP decode feeds this function small 8/16-pixel
// scratch blocks, where NEON setup/shuffle cost can be higher than scalar math.
#if defined(APERTAR_MLV_ENABLE_NEON_PACKER) && (defined(__ARM_NEON) || defined(__ARM_NEON__))
#include <arm_neon.h>
#define APERTAR_MLV_HAS_NEON 1
#else
#define APERTAR_MLV_HAS_NEON 0
#endif

namespace apertar {
namespace {

constexpr uint32_t kFileHeaderSize = 52;
constexpr uint32_t kFileHeaderVideoFrameCountOffset = 36;
constexpr uint16_t kVideoClassRaw = 1;
constexpr uint16_t kAudioClassNone = 0;
constexpr uint32_t kRawInfoStructSize = 160;
constexpr uint32_t kRawInfoBlockSize = 16 + 4 + kRawInfoStructSize;
constexpr uint32_t kVidfHeaderSize = 32;

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

void putLe64(uint8_t *dst, uint64_t value)
{
    for (unsigned int i = 0; i < 8; ++i)
        dst[i] = static_cast<uint8_t>((value >> (i * 8u)) & 0xffu);
}

bool writeAll(int fd, const void *data, size_t size)
{
    const uint8_t *cursor = static_cast<const uint8_t *>(data);
    while (size > 0) {
        const ssize_t written = ::write(fd, cursor, size);
        if (written < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }
        if (written == 0) {
            errno = EIO;
            return false;
        }
        cursor += written;
        size -= static_cast<size_t>(written);
    }
    return true;
}

template <size_t N>
void writeBytesOrThrow(int fd, const std::array<uint8_t, N> &bytes, const std::string &context)
{
    if (!writeAll(fd, bytes.data(), bytes.size()))
        throw std::system_error(errno, std::generic_category(), context);
}

template <size_t N>
class FixedBufferWriter {
public:
    explicit FixedBufferWriter(std::array<uint8_t, N> &bytes)
        : bytes_(bytes)
    {
        bytes_.fill(0);
    }

    void fourcc(const char *value)
    {
        write(value, 4);
    }

    void le16(uint16_t value)
    {
        ensure(2);
        putLe16(bytes_.data() + offset_, value);
        offset_ += 2;
    }

    void le32(uint32_t value)
    {
        ensure(4);
        putLe32(bytes_.data() + offset_, value);
        offset_ += 4;
    }

    void le64(uint64_t value)
    {
        ensure(8);
        putLe64(bytes_.data() + offset_, value);
        offset_ += 8;
    }

    void fixedAscii(const std::string &text, size_t width)
    {
        ensure(width);
        const size_t copied = std::min(width, text.size());
        if (copied > 0)
            std::memcpy(bytes_.data() + offset_, text.data(), copied);
        offset_ += width;
    }

    void write(const void *data, size_t size)
    {
        ensure(size);
        if (size > 0)
            std::memcpy(bytes_.data() + offset_, data, size);
        offset_ += size;
    }

    void skip(size_t size)
    {
        ensure(size);
        offset_ += size;
    }

    size_t offset() const { return offset_; }

private:
    void ensure(size_t size) const
    {
        if (offset_ + size > bytes_.size())
            throw std::runtime_error("Internal MLV fixed buffer overflow.");
    }

    std::array<uint8_t, N> &bytes_;
    size_t offset_ = 0;
};

uint32_t whiteLevel(RawBitDepth bitDepth)
{
    const unsigned int bits = static_cast<unsigned int>(bitDepth);
    return bits >= 16 ? 65535u : ((1u << bits) - 1u);
}

int32_t blackLevelForEncodedDepth(float sourceBlackLevel, RawBitDepth bitDepth)
{
    float black = sourceBlackLevel;
    if (bitDepth == RawBitDepth::Bits8)
        black *= 255.0f / 4095.0f;
    else if (bitDepth == RawBitDepth::Bits10)
        black *= 1023.0f / 4095.0f;

    const float maxBlack = static_cast<float>(std::max<uint32_t>(1u, whiteLevel(bitDepth)) - 1u);
    return static_cast<int32_t>(std::lround(std::clamp(black, 0.0f, maxBlack)));
}

uint32_t cfaPatternCode(BayerPattern pattern)
{
    std::array<uint8_t, 4> cfa{ 0, 1, 1, 2 };
    switch (pattern) {
    case BayerPattern::RGGB: cfa = { 0, 1, 1, 2 }; break;
    case BayerPattern::GRBG: cfa = { 1, 0, 2, 1 }; break;
    case BayerPattern::GBRG: cfa = { 1, 2, 0, 1 }; break;
    case BayerPattern::BGGR: cfa = { 2, 1, 1, 0 }; break;
    case BayerPattern::Monochrome: throw std::runtime_error("Monochrome MLV is not supported.");
    }

    return static_cast<uint32_t>(cfa[0]) |
           (static_cast<uint32_t>(cfa[1]) << 8u) |
           (static_cast<uint32_t>(cfa[2]) << 16u) |
           (static_cast<uint32_t>(cfa[3]) << 24u);
}

uint32_t fpsDenominator(double fps)
{
    if (std::fabs(fps - 23.976) < 0.01 ||
        std::fabs(fps - 29.97) < 0.01 ||
        std::fabs(fps - 59.94) < 0.01) {
        return 1001;
    }
    return 1000;
}

uint32_t fpsNumerator(double fps)
{
    const uint32_t denom = fpsDenominator(fps);
    return static_cast<uint32_t>(std::max<int64_t>(1, std::llround(fps * denom)));
}

uint64_t makeFileGuid(const std::string &path, const CdngEncoder::Metadata &metadata)
{
    uint64_t hash = 1469598103934665603ULL;
    auto mixByte = [&hash](uint8_t value) {
        hash ^= value;
        hash *= 1099511628211ULL;
    };
    auto mixString = [&](const std::string &value) {
        for (unsigned char ch : value)
            mixByte(ch);
    };
    auto mixU64 = [&](uint64_t value) {
        for (unsigned int i = 0; i < 8; ++i)
            mixByte(static_cast<uint8_t>((value >> (i * 8u)) & 0xffu));
    };

    mixString(path);
    mixString(metadata.clipName);
    mixString(metadata.serial);
    mixU64(metadata.wallClockTimestampUs);
    mixU64(static_cast<uint64_t>(std::max<int64_t>(0, metadata.sensorTimestampUs)));
    return hash == 0 ? 1 : hash;
}

uint32_t scaledGain(float value)
{
    return static_cast<uint32_t>(std::clamp<int64_t>(
        std::llround(std::max(0.0f, value) * 1024.0f), 0, 0x7fffffff));
}

int32_t rationalNumerator(float value)
{
    return static_cast<int32_t>(std::clamp<int64_t>(
        std::llround(static_cast<double>(value) * 100000.0), -0x7fffffffLL, 0x7fffffffLL));
}

uint32_t packedRowBytes(uint32_t width, RawBitDepth bitDepth)
{
    return static_cast<uint32_t>((static_cast<uint64_t>(width) * static_cast<unsigned int>(bitDepth) + 7u) / 8u);
}

uint16_t readLe16(const uint8_t *src)
{
    return static_cast<uint16_t>(src[0] | (static_cast<uint16_t>(src[1]) << 8));
}

uint32_t readLe32(const uint8_t *src)
{
    return static_cast<uint32_t>(src[0]) |
           (static_cast<uint32_t>(src[1]) << 8u) |
           (static_cast<uint32_t>(src[2]) << 16u) |
           (static_cast<uint32_t>(src[3]) << 24u);
}

uint8_t unpackedBytesPerSampleForFrame(const RawFrameView &frame)
{
    if (frame.unpackedBytesPerSample == 1u || frame.unpackedBytesPerSample == 2u)
        return frame.unpackedBytesPerSample;
    return frame.bitDepth == RawBitDepth::Bits8 ? 1u : 2u;
}

void copyUnpacked16ToRaw8Row(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    for (uint32_t x = 0; x < width; ++x)
        dst[x] = static_cast<uint8_t>(readLe16(src + (static_cast<size_t>(x) * 2u)) >> 8);
}

void unpackCsi2Raw10Row(const uint8_t *src, uint16_t *dst, uint32_t width)
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

void unpackCsi2Raw12Row(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    const uint32_t alignedWidth = width & ~1u;
    uint32_t x = 0;

    for (; x < alignedWidth; x += 2, src += 3) {
        dst[x + 0] = (static_cast<uint16_t>(src[0]) << 4) | ((src[2] >> 0) & 0xf);
        dst[x + 1] = (static_cast<uint16_t>(src[1]) << 4) | ((src[2] >> 4) & 0xf);
    }

    if (x < width)
        dst[x] = (static_cast<uint16_t>(src[0]) << 4) | ((src[2] >> 0) & 0xf);
}

void packMlvRawGenericRow(const uint16_t *src, uint8_t *dst, uint32_t width, RawBitDepth bitDepth);

#if APERTAR_MLV_HAS_NEON
void packMlvRaw12Block8Neon(const uint16_t *src, uint8_t *dst, uint8_t sourceShift)
{
    uint16x8_t pixels = vld1q_u16(src);
    if (sourceShift == 4)
        pixels = vshrq_n_u16(pixels, 4);
    pixels = vandq_u16(pixels, vdupq_n_u16(0x0fffu));

    const uint8x8_t high = vmovn_u16(vshrq_n_u16(pixels, 4));
    const uint8x8_t low = vmovn_u16(pixels);
    const uint8x8_t highNibble = vmovn_u16(
        vshlq_n_u16(vandq_u16(pixels, vdupq_n_u16(0x000fu)), 4));
    const uint8x8_t topNibble = vmovn_u16(vshrq_n_u16(pixels, 8));
    const uint8x8_t topNibbleNext = vext_u8(topNibble, topNibble, 1);
    const uint8x8_t combined = vorr_u8(highNibble, topNibbleNext);

    static constexpr uint8_t kIndex0[8] = { 0, 8, 10, 17, 19, 2, 4, 12 };
    static constexpr uint8_t kIndex1[8] = { 14, 21, 23, 6, 0xff, 0xff, 0xff, 0xff };
    const uint8x8x3_t table = {{ combined, high, low }};
    const uint8x8_t out0 = vtbl3_u8(table, vld1_u8(kIndex0));
    const uint8x8_t out1 = vtbl3_u8(table, vld1_u8(kIndex1));

    vst1_u8(dst, out0);
    uint8_t tail[8];
    vst1_u8(tail, out1);
    std::memcpy(dst + 8, tail, 4);
}
#endif

inline void packMlvRaw12Group4(uint8_t *dst,
                               uint16_t p0,
                               uint16_t p1,
                               uint16_t p2,
                               uint16_t p3,
                               uint8_t sourceShift = 0)
{
    p0 = static_cast<uint16_t>((p0 >> sourceShift) & 0x0fff);
    p1 = static_cast<uint16_t>((p1 >> sourceShift) & 0x0fff);
    p2 = static_cast<uint16_t>((p2 >> sourceShift) & 0x0fff);
    p3 = static_cast<uint16_t>((p3 >> sourceShift) & 0x0fff);

    // Magic Lantern MLV stores packed samples as a continuous bitstream in
    // little-endian 16-bit words. For RAW12, four pixels become three
    // words: p0|p1_hi, p1_lo|p2_hi, p2_lo|p3.
    dst[0] = static_cast<uint8_t>(((p0 & 0x000fu) << 4u) | (p1 >> 8u));
    dst[1] = static_cast<uint8_t>(p0 >> 4u);
    dst[2] = static_cast<uint8_t>(p2 >> 4u);
    dst[3] = static_cast<uint8_t>(p1 & 0x00ffu);
    dst[4] = static_cast<uint8_t>(p3 & 0x00ffu);
    dst[5] = static_cast<uint8_t>(((p2 & 0x000fu) << 4u) | (p3 >> 8u));
}

void packMlvRaw12Row(const uint16_t *src, uint8_t *dst, uint32_t width, uint8_t sourceShift = 0)
{
    uint32_t x = 0;
#if APERTAR_MLV_HAS_NEON
    if (sourceShift == 0 || sourceShift == 4) {
        for (; x + 7u < width; x += 8u, dst += 12)
            packMlvRaw12Block8Neon(src + x, dst, sourceShift);
    }
#endif

    for (; x + 3u < width; x += 4u, dst += 6) {
        packMlvRaw12Group4(dst,
                           src[x + 0],
                           src[x + 1],
                           src[x + 2],
                           src[x + 3],
                           sourceShift);
    }

    const uint32_t remaining = width - x;
    if (remaining > 0) {
        uint16_t tail[4]{};
        for (uint32_t i = 0; i < remaining; ++i)
            tail[i] = static_cast<uint16_t>(src[x + i] >> sourceShift);
        uint8_t packed[6]{};
        packMlvRawGenericRow(tail, packed, remaining, RawBitDepth::Bits12);
        std::memcpy(dst, packed, packedRowBytes(remaining, RawBitDepth::Bits12));
    }
}

void packMlvRawGenericRow(const uint16_t *src, uint8_t *dst, uint32_t width, RawBitDepth bitDepth)
{
    const unsigned int bits = static_cast<unsigned int>(bitDepth);
    const uint32_t rowBytes = packedRowBytes(width, bitDepth);
    const uint32_t mask = bits >= 16u ? 0xffffu : ((1u << bits) - 1u);
    std::fill(dst, dst + rowBytes, 0);

    for (uint32_t x = 0; x < width; ++x) {
        const uint32_t value = static_cast<uint32_t>(src[x]) & mask;
        uint32_t remaining = bits;
        uint64_t bitOffset = static_cast<uint64_t>(x) * bits;

        while (remaining > 0) {
            const uint64_t wordIndex = bitOffset / 16u;
            const uint32_t bitInWord = static_cast<uint32_t>(bitOffset % 16u);
            const uint32_t bitsHere = std::min<uint32_t>(remaining, 16u - bitInWord);
            const uint32_t sourceRightShift = remaining - bitsHere;
            const uint32_t destLeftShift = 16u - bitInWord - bitsHere;
            const uint32_t partMask = (1u << bitsHere) - 1u;
            const uint16_t part = static_cast<uint16_t>((value >> sourceRightShift) & partMask);
            const size_t byteOffset = static_cast<size_t>(wordIndex) * 2u;

            uint16_t word = 0;
            if (byteOffset < rowBytes)
                word |= dst[byteOffset];
            if (byteOffset + 1u < rowBytes)
                word |= static_cast<uint16_t>(dst[byteOffset + 1u]) << 8u;
            word |= static_cast<uint16_t>(part << destLeftShift);
            if (byteOffset < rowBytes)
                dst[byteOffset] = static_cast<uint8_t>(word & 0xffu);
            if (byteOffset + 1u < rowBytes)
                dst[byteOffset + 1u] = static_cast<uint8_t>((word >> 8u) & 0xffu);

            bitOffset += bitsHere;
            remaining -= bitsHere;
        }
    }
}

constexpr uint16_t kPispCompressOffset = 2048;
constexpr uint8_t kPispCompressMode = 1;
constexpr size_t kPispDequantLutSize = 1024;
constexpr int kPispMode2Field0Count = 512;
constexpr int kPispMode2Field1Count = 128;
constexpr int kPispMode3PackCount = 32768;

struct PispMode2PairDecodeEntry
{
    uint16_t q1Dequantized = 0;
    uint16_t q2Dequantized = 0;
    uint16_t p1Base = 0;
    uint16_t p2Base = 0;
};

struct PispMode3DecodeEntry
{
    uint16_t q0Dequantized = 0;
    uint16_t q1Dequantized = 0;
};

struct PispDecodedQuad
{
    uint16_t p0 = 0;
    uint16_t p1 = 0;
    uint16_t p2 = 0;
    uint16_t p3 = 0;
};

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

std::vector<PispMode2PairDecodeEntry> buildPispMode2PairDecodeLut()
{
    std::vector<PispMode2PairDecodeEntry> lut(static_cast<size_t>(kPispMode2Field0Count) * kPispMode2Field1Count);
    for (int field0 = 0; field0 < kPispMode2Field0Count; ++field0) {
        for (int field1 = 0; field1 < kPispMode2Field1Count; ++field1) {
            int q1 = 0;
            int q2 = 0;
            if (field0 >= 384) {
                q1 = field0;
                q2 = field1 + 384;
            } else {
                q1 = (field1 >= 64) ? field0 : field0 + 64 - field1;
                q2 = (field1 >= 64) ? field0 + field1 - 64 : field0;
            }

            const int p1 = std::min(384, std::max(0, q1 - 64));
            const int p2 = std::min(384, std::max(0, q2 - 64));

            PispMode2PairDecodeEntry entry;
            entry.q1Dequantized = pispDequantizeScalar(static_cast<uint16_t>(q1), 2);
            entry.q2Dequantized = pispDequantizeScalar(static_cast<uint16_t>(q2), 2);
            entry.p1Base = static_cast<uint16_t>(p1);
            entry.p2Base = static_cast<uint16_t>(p2);

            const size_t idx = (static_cast<size_t>(field0) * kPispMode2Field1Count)
                + static_cast<size_t>(field1);
            lut[idx] = entry;
        }
    }
    return lut;
}

std::vector<PispMode3DecodeEntry> buildPispMode3DecodeLut()
{
    std::vector<PispMode3DecodeEntry> lut(kPispMode3PackCount);
    for (int packed = 0; packed < kPispMode3PackCount; ++packed) {
        const int q0 = (packed & 15) + 16 * ((packed >> 8) / 11);
        const int q1 = (packed >> 4) % 176;

        PispMode3DecodeEntry entry;
        entry.q0Dequantized = pispDequantizeScalar(static_cast<uint16_t>(q0), 3);
        entry.q1Dequantized = pispDequantizeScalar(static_cast<uint16_t>(q1), 3);
        lut[static_cast<size_t>(packed)] = entry;
    }
    return lut;
}

const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode0 = buildPispDequantLut(0);
const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode1 = buildPispDequantLut(1);
const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode2 = buildPispDequantLut(2);
const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode3 = buildPispDequantLut(3);
const std::vector<PispMode2PairDecodeEntry> kPispMode2PairDecodeLut = buildPispMode2PairDecodeLut();
const std::vector<PispMode3DecodeEntry> kPispMode3DecodeLut = buildPispMode3DecodeLut();

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

inline PispDecodedQuad decodePispSubBlockMode01(uint32_t word, int qmode)
{
    const int field0 = (word >> 2) & 511;
    const int field1 = (word >> 11) & 127;
    const int field2 = (word >> 18) & 127;
    const int field3 = (word >> 25) & 127;
    const bool upper = field1 >= 64;

    const int q1 = upper ? field0 : field0 + 64 - field1;
    const int q2 = upper ? field0 + field1 - 64 : field0;
    const int q0 = std::max(0, q1 - 64) + field2;
    const int q3 = std::max(0, q2 - 64) + field3;

    return {
        pispDequantizeFast(q0, qmode),
        pispDequantizeFast(q1, qmode),
        pispDequantizeFast(q2, qmode),
        pispDequantizeFast(q3, qmode),
    };
}

inline PispDecodedQuad decodePispSubBlockMode2(uint32_t word)
{
    const int field0 = (word >> 2) & 511;
    const int field1 = (word >> 11) & 127;
    const int field2 = (word >> 18) & 127;
    const int field3 = (word >> 25) & 127;
    const size_t idx = (static_cast<size_t>(field0) * kPispMode2Field1Count)
        + static_cast<size_t>(field1);
    const PispMode2PairDecodeEntry &pair = kPispMode2PairDecodeLut[idx];

    return {
        pispDequantizeFast(static_cast<int>(pair.p1Base) + field2, 2),
        pair.q1Dequantized,
        pair.q2Dequantized,
        pispDequantizeFast(static_cast<int>(pair.p2Base) + field3, 2),
    };
}

inline PispDecodedQuad decodePispSubBlockMode3(uint32_t word)
{
    const int pack0 = (word >> 2) & 32767;
    const int pack1 = (word >> 17) & 32767;
    const PispMode3DecodeEntry &lo = kPispMode3DecodeLut[static_cast<size_t>(pack0)];
    const PispMode3DecodeEntry &hi = kPispMode3DecodeLut[static_cast<size_t>(pack1)];

    return {
        lo.q0Dequantized,
        lo.q1Dequantized,
        hi.q0Dequantized,
        hi.q1Dequantized,
    };
}

inline PispDecodedQuad pispSubBlock(uint32_t word)
{
    const int qmode = word & 3;
    if (qmode == 0 || qmode == 1)
        return decodePispSubBlockMode01(word, qmode);
    if (qmode == 2)
        return decodePispSubBlockMode2(word);
    return decodePispSubBlockMode3(word);
}

inline uint16_t pispPostprocess(uint16_t value)
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

inline void decodePispComp1Block(uint32_t w0, uint32_t w1, uint16_t *dst)
{
    const PispDecodedQuad even = pispSubBlock(w0);
    const PispDecodedQuad odd = pispSubBlock(w1);

    dst[0] = pispPostprocess(even.p0);
    dst[1] = pispPostprocess(odd.p0);
    dst[2] = pispPostprocess(even.p1);
    dst[3] = pispPostprocess(odd.p1);
    dst[4] = pispPostprocess(even.p2);
    dst[5] = pispPostprocess(odd.p2);
    dst[6] = pispPostprocess(even.p3);
    dst[7] = pispPostprocess(odd.p3);
}

inline void packPispComp1BlockToMlvRaw12(uint32_t w0, uint32_t w1, uint8_t *dst)
{
    const PispDecodedQuad even = pispSubBlock(w0);
    const PispDecodedQuad odd = pispSubBlock(w1);

    packMlvRaw12Group4(dst + 0,
                       pispPostprocess(even.p0),
                       pispPostprocess(odd.p0),
                       pispPostprocess(even.p1),
                       pispPostprocess(odd.p1),
                       4);
    packMlvRaw12Group4(dst + 6,
                       pispPostprocess(even.p2),
                       pispPostprocess(odd.p2),
                       pispPostprocess(even.p3),
                       pispPostprocess(odd.p3),
                       4);
}

inline void packPispComp1BlockToMlvRaw10(uint32_t w0, uint32_t w1, uint8_t *dst)
{
    uint16_t working[8];
    decodePispComp1Block(w0, w1, working);
    for (uint16_t &sample : working)
        sample = static_cast<uint16_t>(sample >> 6);
    packMlvRawGenericRow(working, dst, 8u, RawBitDepth::Bits10);
}

inline void packPispComp1BlockToMlvRaw8(uint32_t w0, uint32_t w1, uint8_t *dst)
{
    uint16_t working[8];
    decodePispComp1Block(w0, w1, working);
    for (uint32_t i = 0; i < 8u; ++i)
        dst[i] = static_cast<uint8_t>(working[i] >> 8);
}

void unpackPispCompressedRowToMlvRaw8(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    const uint32_t fullBlocks = width / 8u;
    uint32_t block = 0;
    uint8_t *out = dst;

    for (; block < fullBlocks; ++block, src += 8u, out += 8u) {
        if (kPispCompressMode & 1) {
            packPispComp1BlockToMlvRaw8(readLe32(src), readLe32(src + 4), out);
        } else {
            for (uint32_t i = 0; i < 8u; ++i)
                out[i] = static_cast<uint8_t>(pispPostprocess(static_cast<uint16_t>(src[i]) << 8) >> 8);
        }
    }

    const uint32_t remaining = width - (block * 8u);
    if (remaining > 0) {
        uint16_t working[8]{};
        if (kPispCompressMode & 1)
            decodePispComp1Block(readLe32(src), readLe32(src + 4), working);
        else {
            for (uint32_t i = 0; i < remaining; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
        }

        for (uint32_t i = 0; i < remaining; ++i)
            out[i] = static_cast<uint8_t>(working[i] >> 8);
    }
}

void unpackPispCompressedRowToMlvRaw12(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    const uint32_t fullBlocks = width / 8u;
    uint32_t block = 0;
    uint8_t *out = dst;

    for (; block + 1u < fullBlocks; block += 2u, src += 16u, out += 24u) {
        if (kPispCompressMode & 1) {
            packPispComp1BlockToMlvRaw12(readLe32(src), readLe32(src + 4), out);
            packPispComp1BlockToMlvRaw12(readLe32(src + 8), readLe32(src + 12), out + 12);
        } else {
            uint16_t working[16];
            for (int i = 0; i < 8; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
            for (int i = 0; i < 8; ++i)
                working[8 + i] = pispPostprocess(static_cast<uint16_t>(src[8 + i]) << 8);
            packMlvRaw12Row(working, out, 16u, 4);
        }
    }

    for (; block < fullBlocks; ++block, src += 8u, out += 12u) {
        if (kPispCompressMode & 1) {
            packPispComp1BlockToMlvRaw12(readLe32(src), readLe32(src + 4), out);
        } else {
            uint16_t working[8];
            for (int i = 0; i < 8; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
            packMlvRaw12Row(working, out, 8u, 4);
        }
    }

    const uint32_t remaining = width - (block * 8u);
    if (remaining > 0) {
        uint16_t working[8]{};
        uint8_t packed[24]{};

        if (kPispCompressMode & 1)
            decodePispComp1Block(readLe32(src), readLe32(src + 4), working);
        else {
            for (uint32_t i = 0; i < remaining; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
        }

        packMlvRaw12Row(working, packed, remaining, 4);
        std::memcpy(out, packed,
                    packedRowBytes(remaining, RawBitDepth::Bits12));
    }
}

void unpackPispCompressedRowToMlvRaw10(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    const uint32_t fullBlocks = width / 8u;
    uint32_t block = 0;
    uint8_t *out = dst;

    for (; block + 1u < fullBlocks; block += 2u, src += 16u, out += 20u) {
        if (kPispCompressMode & 1) {
            packPispComp1BlockToMlvRaw10(readLe32(src), readLe32(src + 4), out);
            packPispComp1BlockToMlvRaw10(readLe32(src + 8), readLe32(src + 12), out + 10);
        } else {
            uint16_t working[16];
            for (int i = 0; i < 8; ++i)
                working[i] = static_cast<uint16_t>(pispPostprocess(static_cast<uint16_t>(src[i]) << 8) >> 6);
            for (int i = 0; i < 8; ++i)
                working[8 + i] = static_cast<uint16_t>(pispPostprocess(static_cast<uint16_t>(src[8 + i]) << 8) >> 6);
            packMlvRawGenericRow(working, out, 16u, RawBitDepth::Bits10);
        }
    }

    for (; block < fullBlocks; ++block, src += 8u, out += 10u) {
        if (kPispCompressMode & 1) {
            packPispComp1BlockToMlvRaw10(readLe32(src), readLe32(src + 4), out);
        } else {
            uint16_t working[8];
            for (int i = 0; i < 8; ++i)
                working[i] = static_cast<uint16_t>(pispPostprocess(static_cast<uint16_t>(src[i]) << 8) >> 6);
            packMlvRawGenericRow(working, out, 8u, RawBitDepth::Bits10);
        }
    }

    const uint32_t remaining = width - (block * 8u);
    if (remaining > 0) {
        uint16_t working[8]{};
        uint8_t packed[10]{};

        if (kPispCompressMode & 1) {
            decodePispComp1Block(readLe32(src), readLe32(src + 4), working);
            for (uint32_t i = 0; i < remaining; ++i)
                working[i] = static_cast<uint16_t>(working[i] >> 6);
        } else {
            for (uint32_t i = 0; i < remaining; ++i)
                working[i] = static_cast<uint16_t>(pispPostprocess(static_cast<uint16_t>(src[i]) << 8) >> 6);
        }

        packMlvRawGenericRow(working, packed, remaining, RawBitDepth::Bits10);
        std::memcpy(out, packed,
                    packedRowBytes(remaining, RawBitDepth::Bits10));
    }
}

void requireRawFrame(const RawFrameView &frame)
{
    if (!frame.data)
        throw std::runtime_error("MLV raw frame has no data.");
    if (frame.width == 0 || frame.height == 0 || frame.stride == 0)
        throw std::runtime_error("MLV raw frame has invalid geometry.");
}

} // namespace

MlvEncoder::~MlvEncoder()
{
    close();
}

MlvEncoder::RawLayout MlvEncoder::prepareRawLayout(const RawFrameView &frame) const
{
    requireRawFrame(frame);

    RawLayout layout;
    layout.sourcePacking = frame.packing;

    if (frame.packing == PixelPacking::PispCompressed1) {
        if (frame.bitDepth == RawBitDepth::Bits8)
            layout.bitDepth = RawBitDepth::Bits8;
        else if (frame.bitDepth == RawBitDepth::Bits10)
            layout.bitDepth = RawBitDepth::Bits10;
        else
            layout.bitDepth = RawBitDepth::Bits12;
        layout.rowBytes = packedRowBytes(frame.width, layout.bitDepth);
    } else {
        layout.bitDepth = frame.bitDepth;
        layout.rowBytes = frame.bitDepth == RawBitDepth::Bits16
            ? frame.width * 2u
            : packedRowBytes(frame.width, frame.bitDepth);
    }

    layout.byteCount = static_cast<uint64_t>(layout.rowBytes) * frame.height;
    return layout;
}

bool MlvEncoder::writeRawPayload(const RawFrameView &frame)
{
    if (rawLayout_.rowBytes == 0 || rawLayout_.byteCount == 0) {
        errno = EINVAL;
        return false;
    }

    const size_t targetChunkSize = std::max<size_t>(
        static_cast<size_t>(rawLayout_.rowBytes),
        std::min<size_t>(4u * 1024u * 1024u,
                         std::max<size_t>(256u * 1024u,
                                          static_cast<size_t>(rawLayout_.rowBytes) * 64u)));
    if (payloadScratch_.size() < targetChunkSize)
        payloadScratch_.resize(targetChunkSize);
    if (frame.packing == PixelPacking::Csi2Packed && rawLayout_.bitDepth != RawBitDepth::Bits16)
        rowScratch_.resize(frame.width);

    size_t bufferedBytes = 0;
    auto flushScratch = [&]() {
        if (bufferedBytes == 0)
            return true;

        const bool ok = writeAll(fd_, payloadScratch_.data(), bufferedBytes);
        if (ok)
            bufferedBytes = 0;
        return ok;
    };

    for (uint32_t y = 0; y < frame.height; ++y) {
        if (bufferedBytes + rawLayout_.rowBytes > payloadScratch_.size()) {
            if (!flushScratch())
                return false;
        }

        const uint8_t *src = frame.data + static_cast<size_t>(y) * frame.stride;
        uint8_t *dst = payloadScratch_.data() + bufferedBytes;

        if (frame.packing == PixelPacking::PispCompressed1) {
            if (rawLayout_.bitDepth == RawBitDepth::Bits8)
                unpackPispCompressedRowToMlvRaw8(src, dst, frame.width);
            else if (rawLayout_.bitDepth == RawBitDepth::Bits10)
                unpackPispCompressedRowToMlvRaw10(src, dst, frame.width);
            else
                unpackPispCompressedRowToMlvRaw12(src, dst, frame.width);
        } else if (frame.packing == PixelPacking::Csi2Packed) {
            if (frame.bitDepth == RawBitDepth::Bits10) {
                unpackCsi2Raw10Row(src, rowScratch_.data(), frame.width);
                packMlvRawGenericRow(rowScratch_.data(), dst, frame.width, frame.bitDepth);
            } else if (frame.bitDepth == RawBitDepth::Bits12) {
                unpackCsi2Raw12Row(src, rowScratch_.data(), frame.width);
                packMlvRaw12Row(rowScratch_.data(), dst, frame.width);
            } else {
                errno = EINVAL;
                return false;
            }
        } else if (frame.bitDepth == RawBitDepth::Bits16) {
            std::memcpy(dst, src, rawLayout_.rowBytes);
        } else if (frame.bitDepth == RawBitDepth::Bits8) {
            const uint8_t sourceBytes = unpackedBytesPerSampleForFrame(frame);
            if (sourceBytes == 1u)
                std::memcpy(dst, src, rawLayout_.rowBytes);
            else if (sourceBytes == 2u)
                copyUnpacked16ToRaw8Row(src, dst, frame.width);
            else {
                errno = EINVAL;
                return false;
            }
        } else {
            const auto *src16 = reinterpret_cast<const uint16_t *>(src);
            if (frame.bitDepth == RawBitDepth::Bits12)
                packMlvRaw12Row(src16, dst, frame.width);
            else if (frame.bitDepth == RawBitDepth::Bits10)
                packMlvRawGenericRow(src16, dst, frame.width, frame.bitDepth);
            else {
                errno = EINVAL;
                return false;
            }
        }

        bufferedBytes += rawLayout_.rowBytes;
    }

    return flushScratch();
}

bool MlvEncoder::open(const std::string &path,
                      const CaptureSettings &settings,
                      const RawFrameView &firstFrame,
                      const CdngEncoder::Metadata &metadata,
                      std::string *error)
{
    close();

    try {
        if (firstFrame.bayerPattern == BayerPattern::Monochrome)
            throw std::runtime_error("Monochrome MLV is not supported. Use cDNG instead.");
        rawLayout_ = prepareRawLayout(firstFrame);
        if (rawLayout_.rowBytes == 0 || rawLayout_.byteCount == 0)
            throw std::runtime_error("MLV raw layout is empty.");

        int openFlags = O_WRONLY | O_CREAT | O_TRUNC;
#ifdef O_CLOEXEC
        openFlags |= O_CLOEXEC;
#endif
        fd_ = ::open(path.c_str(), openFlags, 0644);
        if (fd_ < 0)
            throw std::system_error(errno, std::generic_category(), "Failed to open MLV file");

        settings_ = settings;
        path_ = path;
        frameCount_ = 0;
        previewColorWritten_ = false;
        fileGuid_ = makeFileGuid(path, metadata);
        firstTimestampUs_ = firstFrame.timestampUs > 0 ? firstFrame.timestampUs : metadata.sensorTimestampUs;
        if (firstTimestampUs_ < 0)
            firstTimestampUs_ = 0;

        ::posix_fadvise(fd_, 0, 0, POSIX_FADV_SEQUENTIAL);
        writeFileHeader(0);
        writeVersionBlock();
        writeClockBlocks(metadata);
        writeIdentityBlock(metadata);
        writeRawInfoBlock(firstFrame, metadata);
        writePreviewColorBlock(metadata);
        writeExposureBlock(settings, metadata);
        writeWhiteBalanceBlock(metadata);
        return true;
    } catch (const std::exception &e) {
        if (error)
            *error = e.what();
        close();
        return false;
    }
}

bool MlvEncoder::writeFrame(const RawFrameView &frame,
                            const CdngEncoder::Metadata &metadata,
                            std::string *error)
{
    if (fd_ < 0) {
        if (error)
            *error = "MLV file is not open.";
        return false;
    }

    try {
        const uint64_t payloadBytes = rawLayout_.byteCount;
        writePreviewColorBlock(metadata, relativeTimestampUs(frame, metadata));
        if (metadata.whiteBalanceKelvin != lastWbKelvin_ ||
            scaledGain(metadata.colourGains[0]) != lastWbRed_ ||
            scaledGain(metadata.colourGains[1]) != lastWbBlue_)
            writeWhiteBalanceBlock(metadata, relativeTimestampUs(frame, metadata));
        const uint64_t blockSize64 = static_cast<uint64_t>(kVidfHeaderSize) + payloadBytes;
        if (blockSize64 > std::numeric_limits<uint32_t>::max())
            throw std::runtime_error("MLV frame is too large.");

        std::array<uint8_t, kVidfHeaderSize> header{};
        FixedBufferWriter writer(header);
        writer.fourcc("VIDF");
        writer.le32(static_cast<uint32_t>(blockSize64));
        writer.le64(relativeTimestampUs(frame, metadata));
        writer.le32(static_cast<uint32_t>(metadata.frameIndex));
        writer.le16(0); // cropPosX
        writer.le16(0); // cropPosY
        writer.le16(0); // panPosX
        writer.le16(0); // panPosY
        writer.le32(0); // frameSpace
        if (writer.offset() != header.size())
            throw std::runtime_error("Internal MLV VIDF size mismatch.");
        writeBytesOrThrow(fd_, header, "Failed to write MLV VIDF header");

        if (!writeRawPayload(frame))
            throw std::system_error(errno, std::generic_category(), "Failed to write MLV raw payload");

        ++frameCount_;
        return true;
    } catch (const std::exception &e) {
        if (error)
            *error = e.what();
        return false;
    }
}

void MlvEncoder::close()
{
    if (fd_ < 0)
        return;

    try {
        patchFrameCount();
    } catch (const std::exception &e) {
        std::cerr << "ApertarCore MLV finalize warning for "
                  << path_ << ": " << e.what() << "\n";
    }

    ::close(fd_);
    fd_ = -1;
    path_.clear();
}

void MlvEncoder::writeFileHeader(uint32_t frameCount)
{
    std::array<uint8_t, kFileHeaderSize> bytes{};
    FixedBufferWriter writer(bytes);
    writer.fourcc("MLVI");
    writer.le32(kFileHeaderSize);
    writer.fixedAscii("v2.0", 8);
    writer.le64(fileGuid_);
    writer.le16(0); // fileNum
    writer.le16(1); // fileCount
    writer.le32(0); // fileFlags
    writer.le16(kVideoClassRaw);
    writer.le16(kAudioClassNone);
    writer.le32(frameCount);
    writer.le32(0); // audioFrameCount
    writer.le32(fpsNumerator(settings_.fps));
    writer.le32(fpsDenominator(settings_.fps));
    if (writer.offset() != bytes.size())
        throw std::runtime_error("Internal MLV file header size mismatch.");
    writeBytesOrThrow(fd_, bytes, "Failed to write MLV file header");
}

void MlvEncoder::writeClockBlocks(const CdngEncoder::Metadata &metadata)
{
    const time_t seconds = time_t(metadata.wallClockTimestampUs / 1000000);
    tm local{};localtime_r(&seconds, &local);
    std::array<uint8_t,44> rtc{};
    FixedBufferWriter clock(rtc);
    clock.fourcc("RTCI");clock.le32(rtc.size());clock.le64(0);
    for (int value : {local.tm_sec, local.tm_min, local.tm_hour, local.tm_mday,
                      local.tm_mon, local.tm_year, local.tm_wday, local.tm_yday, local.tm_isdst})
        clock.le16(uint16_t(value));
    clock.le16(uint16_t(local.tm_gmtoff));
    clock.fixedAscii(local.tm_zone ? local.tm_zone : "", 8);
    writeBytesOrThrow(fd_, rtc, "Failed to write MLV RTCI block");

    // MLV has no MOV-style tmcd track. Keep a standard INFO block with the
    // precise start/timecode alongside RTCI and the synchronized BWF sidecar.
    const auto tc = recordingTimecode(metadata.timecodeMode == TimecodeMode::FreeRun,
                                      metadata.wallClockTimestampUs, metadata.fps);
    const std::string text = "Apertar Timecode=" + tc + " Mode=" +
        (metadata.timecodeMode == TimecodeMode::FreeRun ? "FreeRun" : "RecRun") +
        " FirstFrameEpochUs=" + std::to_string(metadata.wallClockTimestampUs);
    std::array<uint8_t,272> info{};
    FixedBufferWriter writer(info);
    writer.fourcc("INFO");writer.le32(info.size());writer.le64(0);writer.fixedAscii(text,256);
    writeBytesOrThrow(fd_, info, "Failed to write MLV INFO block");
}

void MlvEncoder::writeVersionBlock()
{
    constexpr char version[] = "ApertarCore MLV";
    constexpr uint32_t versionSize = sizeof(version) - 1u;
    constexpr uint32_t blockSize = 36;
    static_assert(blockSize == ((16u + 4u + versionSize + 3u) & ~3u), "Unexpected MLV VERS size.");

    std::array<uint8_t, blockSize> bytes{};
    FixedBufferWriter writer(bytes);
    writer.fourcc("VERS");
    writer.le32(blockSize);
    writer.le64(0);
    writer.le32(versionSize);
    writer.write(version, versionSize);
    writer.skip(bytes.size() - writer.offset());
    writeBytesOrThrow(fd_, bytes, "Failed to write MLV VERS block");
}

void MlvEncoder::writeIdentityBlock(const CdngEncoder::Metadata &metadata)
{
    constexpr uint32_t blockSize = 16 + 32 + 4 + 32;
    std::array<uint8_t, blockSize> bytes{};
    FixedBufferWriter writer(bytes);
    writer.fourcc("IDNT");
    writer.le32(blockSize);
    writer.le64(0);
    writer.fixedAscii(metadata.model.empty() ? std::string("Apertar D1") : metadata.model, 32);
    writer.le32(0xA9000001u);
    writer.fixedAscii(metadata.serial, 32);
    if (writer.offset() != bytes.size())
        throw std::runtime_error("Internal MLV IDNT size mismatch.");
    writeBytesOrThrow(fd_, bytes, "Failed to write MLV IDNT block");
}

void MlvEncoder::writeRawInfoBlock(const RawFrameView &frame,
                                   const CdngEncoder::Metadata &metadata)
{
    std::array<float, 9> ccm{};
    std::copy(std::begin(metadata.colourMatrix), std::end(metadata.colourMatrix), ccm.begin());
    auto colorMatrix = encoderDaylightColorMatrixForSensor(metadata.sensorModel);
    if (!colorMatrix)
        colorMatrix = dngColorMatrixFromCcm(ccm, {metadata.colourGains[0], metadata.colourGains[1]});

    const int32_t black = blackLevelForEncodedDepth(
        metadata.hasBlackLevels ? metadata.blackLevels[0] : 200.0f,
        rawLayout_.bitDepth);
    const int32_t white = static_cast<int32_t>(whiteLevel(rawLayout_.bitDepth));
    const int32_t bitDepth = static_cast<int32_t>(rawLayout_.bitDepth);
    const int32_t pitch = static_cast<int32_t>(rawLayout_.rowBytes);
    const int32_t frameSize = static_cast<int32_t>(std::min<uint64_t>(
        rawLayout_.byteCount, static_cast<uint64_t>(std::numeric_limits<int32_t>::max())));

    std::array<uint8_t, kRawInfoBlockSize> bytes{};
    FixedBufferWriter writer(bytes);
    writer.fourcc("RAWI");
    writer.le32(kRawInfoBlockSize);
    writer.le64(0);
    writer.le16(static_cast<uint16_t>(std::min<uint32_t>(frame.width, 0xffffu)));
    writer.le16(static_cast<uint16_t>(std::min<uint32_t>(frame.height, 0xffffu)));

    writer.le32(1); // raw_info api_version
    writer.le32(0); // 64-bit builds cannot store a pointer here
    writer.le32(frame.height);
    writer.le32(frame.width);
    writer.le32(static_cast<uint32_t>(pitch));
    writer.le32(static_cast<uint32_t>(frameSize));
    writer.le32(static_cast<uint32_t>(bitDepth));
    writer.le32(static_cast<uint32_t>(black));
    writer.le32(static_cast<uint32_t>(white));
    writer.le32(0); // crop origin x
    writer.le32(0); // crop origin y
    writer.le32(frame.width);
    writer.le32(frame.height);
    writer.le32(0); // active y1
    writer.le32(0); // active x1
    writer.le32(frame.height);
    writer.le32(frame.width);
    writer.le32(0); // exposure bias numerator
    writer.le32(1); // exposure bias denominator
    writer.le32(cfaPatternCode(frame.bayerPattern));
    writer.le32(21); // D65-style default illuminant
    for (float value : *colorMatrix) {
        writer.le32(static_cast<uint32_t>(rationalNumerator(value)));
        writer.le32(100000);
    }
    writer.le32(0); // dynamic range unknown

    if (writer.offset() != bytes.size())
        throw std::runtime_error("Internal MLV RAWI size mismatch.");
    writeBytesOrThrow(fd_, bytes, "Failed to write MLV RAWI block");
}

void MlvEncoder::writeExposureBlock(const CaptureSettings &settings,
                                    const CdngEncoder::Metadata &metadata)
{
    constexpr uint32_t blockSize = 16 + 4 + 4 + 4 + 4 + 8;
    const uint32_t iso = settings.isoAuto ? metadata.iso : settings.iso;
    const uint32_t digitalGain = static_cast<uint32_t>(std::clamp<int64_t>(
        std::llround(std::max(1.0f, metadata.digitalGain) * 1024.0f), 0, 0x7fffffff));

    std::array<uint8_t, blockSize> bytes{};
    FixedBufferWriter writer(bytes);
    writer.fourcc("EXPO");
    writer.le32(blockSize);
    writer.le64(0);
    writer.le32(settings.isoAuto ? 1u : 0u);
    writer.le32(iso);
    writer.le32(iso);
    writer.le32(digitalGain);
    writer.le64(metadata.shutterUs);
    if (writer.offset() != bytes.size())
        throw std::runtime_error("Internal MLV EXPO size mismatch.");
    writeBytesOrThrow(fd_, bytes, "Failed to write MLV EXPO block");
}

void MlvEncoder::writeWhiteBalanceBlock(const CdngEncoder::Metadata &metadata, uint64_t timestampUs)
{
    constexpr uint32_t blockSize = 16 + 28;
    std::array<uint8_t, blockSize> bytes{};
    FixedBufferWriter writer(bytes);
    writer.fourcc("WBAL");
    writer.le32(blockSize);
    writer.le64(timestampUs);
    writer.le32(9); // Kelvin
    writer.le32(metadata.whiteBalanceKelvin);
    writer.le32(scaledGain(metadata.colourGains[0]));
    writer.le32(1024);
    writer.le32(scaledGain(metadata.colourGains[1]));
    writer.le32(0);
    writer.le32(0);
    if (writer.offset() != bytes.size())
        throw std::runtime_error("Internal MLV WBAL size mismatch.");
    writeBytesOrThrow(fd_, bytes, "Failed to write MLV WBAL block");
    lastWbKelvin_ = metadata.whiteBalanceKelvin;
    lastWbRed_ = scaledGain(metadata.colourGains[0]);
    lastWbBlue_ = scaledGain(metadata.colourGains[1]);
}

void MlvEncoder::writePreviewColorBlock(const CdngEncoder::Metadata &metadata, uint64_t timestampUs)
{
    std::array<int32_t, 9> ccm{};
    for (size_t i = 0; i < ccm.size(); ++i) {
        const float value = metadata.colourMatrix[i];
        if (!std::isfinite(value) || std::abs(value) > 21474.0f)
            throw std::runtime_error("Invalid MLV preview colour matrix.");
        ccm[i] = rationalNumerator(value);
    }
    if (previewColorWritten_ && ccm == lastPreviewCcm_)
        return;

    // Use the standard DEBG container: MLVApp rejects unknown block names.
    // Its private APCM payload preserves the ISP transform for native playback.
    std::array<uint8_t, 64> bytes{};
    FixedBufferWriter writer(bytes);
    writer.fourcc("DEBG");
    writer.le32(bytes.size());
    writer.le64(timestampUs);
    writer.le32(0x4d435041); // private debug-data type: APCM
    writer.le32(40); // payload length, including version
    writer.le32(1); // version; nine signed matrix coefficients scaled by 100000
    for (int32_t value : ccm)
        writer.le32(static_cast<uint32_t>(value));
    writeBytesOrThrow(fd_, bytes, "Failed to write MLV preview colour block");
    lastPreviewCcm_ = ccm;
    previewColorWritten_ = true;
}

void MlvEncoder::patchFrameCount()
{
    if (::lseek(fd_, kFileHeaderVideoFrameCountOffset, SEEK_SET) < 0)
        throw std::system_error(errno, std::generic_category(), "Failed to seek MLV frame count");

    std::array<uint8_t, 4> countBytes{};
    putLe32(countBytes.data(), frameCount_);
    if (!writeAll(fd_, countBytes.data(), countBytes.size()))
        throw std::system_error(errno, std::generic_category(), "Failed to patch MLV frame count");

    if (::lseek(fd_, 0, SEEK_END) < 0)
        throw std::system_error(errno, std::generic_category(), "Failed to restore MLV file position");
}

uint64_t MlvEncoder::relativeTimestampUs(const RawFrameView &frame,
                                         const CdngEncoder::Metadata &metadata) const
{
    int64_t timestamp = frame.timestampUs > 0 ? frame.timestampUs : metadata.sensorTimestampUs;
    if (timestamp > firstTimestampUs_)
        return static_cast<uint64_t>(timestamp - firstTimestampUs_);

    const double safeFps = settings_.fps > 0.0 ? settings_.fps : 24.0;
    return static_cast<uint64_t>(std::llround(
        static_cast<double>(metadata.frameIndex) * 1000000.0 / safeFps));
}

} // namespace apertar
