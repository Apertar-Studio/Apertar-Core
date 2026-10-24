#include "cdng/CdngEncoder.hpp"
#include "cdng/BlackLevel.hpp"
#include "audio/RecordingClock.hpp"

#include "cdng/IfdBuilder.hpp"
#include "color/ColorProfileConfig.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <ctime>
#include <cstdlib>
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
#include <unistd.h>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define APERTARCORE_HAS_NEON 1
#else
#define APERTARCORE_HAS_NEON 0
#endif

namespace apertar {
namespace {

constexpr int32_t kDngBlackLevel = 200;
constexpr uint16_t kCinemaDngTagTimeCodes = 0xC763;
constexpr uint16_t kCinemaDngTagFrameRate = 0xC764;
constexpr uint16_t kCinemaDngTagReelName = 0xC789;
constexpr uint16_t kCinemaDngTagCameraLabel = 0xC7A1;

uint64_t elapsedMicroseconds(std::chrono::steady_clock::time_point started)
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - started).count());
}

uint32_t configuredLosslessTileRows()
{
    static const uint32_t value = []() {
        const char *text = std::getenv("APERTAR_CDNG_TILE_ROWS");
        if (!text || !*text)
            return 128u;

        char *end = nullptr;
        errno = 0;
        const unsigned long parsed = std::strtoul(text, &end, 10);
        if (errno != 0 || end == text || *end != '\0' || parsed < 16 || parsed > 1024)
            return 128u;
        return static_cast<uint32_t>(parsed);
    }();
    return value;
}

bool detailedLosslessProfilingEnabled()
{
    static const bool enabled = []() {
        const char *text = std::getenv("APERTAR_CDNG_PROFILE_DETAIL");
        return text && *text && std::strcmp(text, "0") != 0 &&
            std::strcmp(text, "false") != 0 && std::strcmp(text, "off") != 0;
    }();
    return enabled;
}

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

uint16_t readLe16(const uint8_t *src)
{
    return static_cast<uint16_t>(src[0] | (static_cast<uint16_t>(src[1]) << 8));
}

uint8_t unpackedBytesPerSampleForFrame(const RawFrameView &frame)
{
    if (frame.unpackedBytesPerSample == 1u || frame.unpackedBytesPerSample == 2u)
        return frame.unpackedBytesPerSample;
    return frame.bitDepth == RawBitDepth::Bits8 ? 1u : 2u;
}

void copyUnpacked16ToDng8Row(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    for (uint32_t x = 0; x < width; ++x)
        dst[x] = static_cast<uint8_t>(readLe16(src + (static_cast<size_t>(x) * 2u)) >> 8);
}

int32_t blackLevelForEncodedDepth(float sourceBlackLevel, RawBitDepth bitDepth,
                                 bool onContainerScale)
{
    return static_cast<int32_t>(std::lround(blackLevelForSampleBits(
        sourceBlackLevel, static_cast<unsigned>(bitDepth), onContainerScale)));
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
    case BayerPattern::Monochrome: return nullptr;
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

struct DngColorProfile {
    float colorMatrix1[9]{};
    float colorMatrix2[9]{};
    bool hasForwardMatrix1 = false;
    bool hasForwardMatrix2 = false;
    float forwardMatrix1[9]{};
    float forwardMatrix2[9]{};
    uint16_t illuminant1 = 21;
    uint16_t illuminant2 = 21;
    std::string cacheKey;
};

static constexpr int32_t kDngProfileRationalScale = 100000;

Matrix cameraToXyzMatrix(const CdngEncoder::Metadata &metadata)
{
    std::array<float, 9> ccm{};
    std::copy(std::begin(metadata.colourMatrix), std::end(metadata.colourMatrix), ccm.begin());
    const auto values = dngColorMatrixFromCcm(ccm, {metadata.colourGains[0], metadata.colourGains[1]});
    Matrix result;
    std::copy(values.begin(), values.end(), result.m);
    return result;
}

void appendProfileKeyMatrix(std::ostringstream &stream, const char *name, const float *matrix)
{
    stream << ';' << name << '=';
    for (int i = 0; i < 9; ++i)
        stream << std::setprecision(7) << matrix[i] << ',';
}

DngColorProfile colorProfileForMetadata(const CdngEncoder::Metadata &metadata)
{
    if (auto configured = encoderColorProfileForSensor(metadata.sensorModel, metadata.whiteBalanceKelvin)) {
        const bool hasConfiguredColor =
            configured->hasColorMatrix1 || configured->hasColorMatrix2;
        if (hasConfiguredColor) {
            DngColorProfile profile;
            if (configured->hasColorMatrix1)
                std::copy(configured->colorMatrix1.begin(), configured->colorMatrix1.end(), std::begin(profile.colorMatrix1));
            if (configured->hasColorMatrix2)
                std::copy(configured->colorMatrix2.begin(), configured->colorMatrix2.end(), std::begin(profile.colorMatrix2));
            if (configured->hasColorMatrix1 && !configured->hasColorMatrix2)
                std::copy(std::begin(profile.colorMatrix1), std::end(profile.colorMatrix1), std::begin(profile.colorMatrix2));
            if (!configured->hasColorMatrix1 && configured->hasColorMatrix2)
                std::copy(std::begin(profile.colorMatrix2), std::end(profile.colorMatrix2), std::begin(profile.colorMatrix1));

            profile.hasForwardMatrix1 = configured->hasForwardMatrix1;
            profile.hasForwardMatrix2 = configured->hasForwardMatrix2;
            if (profile.hasForwardMatrix1)
                std::copy(configured->forwardMatrix1.begin(), configured->forwardMatrix1.end(), std::begin(profile.forwardMatrix1));
            if (profile.hasForwardMatrix2)
                std::copy(configured->forwardMatrix2.begin(), configured->forwardMatrix2.end(), std::begin(profile.forwardMatrix2));
            profile.illuminant1 = configured->illuminant1;
            profile.illuminant2 = configured->illuminant2;
            profile.cacheKey = "config:" + configured->cacheKey;
            return profile;
        }
    }

    const Matrix cameraToXyz = cameraToXyzMatrix(metadata);

    DngColorProfile profile;
    std::copy(cameraToXyz.m, cameraToXyz.m + 9, std::begin(profile.colorMatrix1));
    std::copy(cameraToXyz.m, cameraToXyz.m + 9, std::begin(profile.colorMatrix2));
    profile.illuminant1 = 21;
    profile.illuminant2 = 21;
    std::array<float, 9> ccm{};
    std::copy(std::begin(metadata.colourMatrix), std::end(metadata.colourMatrix), ccm.begin());
    const auto forward = dngForwardMatrixFromCcm(ccm);
    std::copy(forward.begin(), forward.end(), profile.forwardMatrix1);
    std::copy(forward.begin(), forward.end(), profile.forwardMatrix2);
    profile.hasForwardMatrix1 = profile.hasForwardMatrix2 = true;
    std::ostringstream key;
    key << "runtime:" << metadata.sensorModel
        << ";i1=" << profile.illuminant1
        << ";i2=" << profile.illuminant2;
    appendProfileKeyMatrix(key, "cm", profile.colorMatrix1);
    appendProfileKeyMatrix(key, "fm", profile.forwardMatrix1);
    profile.cacheKey = key.str();
    return profile;
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

std::array<uint8_t, 8> timeCode(const CdngEncoder::Metadata &metadata, uint64_t timestampUs)
{
    auto bcd = [](int value) {
        return static_cast<uint8_t>(((value / 10) << 4) | (value % 10));
    };

    if (metadata.timecodeMode == TimecodeMode::RecRun) {
        const int nominalFps = std::max(1, static_cast<int>(std::llround(metadata.fps)));
        const uint64_t totalFrames = metadata.frameIndex;
        const uint64_t totalSeconds = totalFrames / static_cast<uint64_t>(nominalFps);
        const int frame = static_cast<int>(totalFrames % static_cast<uint64_t>(nominalFps));
        const int hours = static_cast<int>((totalSeconds / 3600ULL) % 24ULL);
        const int minutes = static_cast<int>((totalSeconds / 60ULL) % 60ULL);
        const int seconds = static_cast<int>(totalSeconds % 60ULL);

        return {
            bcd(frame),
            bcd(seconds),
            bcd(minutes),
            bcd(hours),
            0, 0, 0, 0,
        };
    }

    const std::time_t seconds = static_cast<std::time_t>(timestampUs / 1000000ull);
    std::tm localTime{};
#if defined(_WIN32)
    localtime_s(&localTime, &seconds);
#else
    localtime_r(&seconds, &localTime);
#endif

    const double safeFps = metadata.fps > 0.0 ? metadata.fps : 24.0;
    const int frame = static_cast<int>(((timestampUs % 1000000ull) * safeFps) / 1000000.0);

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

tiff::Entry readIfdEntry(const std::vector<uint8_t> &bytes, size_t offset)
{
    if (offset + sizeof(tiff::Entry) > bytes.size())
        throw std::runtime_error("TIFF entry offset is outside the buffer.");

    tiff::Entry entry{};
    std::memcpy(&entry, bytes.data() + offset, sizeof(entry));
    return entry;
}

uint32_t findIfdTagValueOffset(const std::vector<uint8_t> &bytes,
                               uint32_t mainIfdOffset,
                               uint32_t ifdRelativeOffset,
                               uint16_t tag,
                               bool inlineValue)
{
    if (ifdRelativeOffset + sizeof(uint16_t) > bytes.size())
        throw std::runtime_error("TIFF IFD offset is outside the buffer.");

    uint16_t entryCount = 0;
    std::memcpy(&entryCount, bytes.data() + ifdRelativeOffset, sizeof(entryCount));
    const size_t entriesBase = ifdRelativeOffset + sizeof(uint16_t);
    for (uint16_t i = 0; i < entryCount; ++i) {
        const size_t entryOffset = entriesBase + static_cast<size_t>(i) * sizeof(tiff::Entry);
        const tiff::Entry entry = readIfdEntry(bytes, entryOffset);
        if (entry.tag != tag)
            continue;

        const size_t payloadLength = tiff::typeSize(static_cast<tiff::Type>(entry.type)) * entry.count;
        if (inlineValue) {
            if (payloadLength > sizeof(entry.value))
                throw std::runtime_error("Requested inline TIFF tag is stored out-of-line.");
            return checkedTiffOffset(entryOffset + 8);
        }

        if (payloadLength <= sizeof(entry.value))
            throw std::runtime_error("Requested out-of-line TIFF tag is stored inline.");
        return checkedTiffOffset(readLe32(bytes.data() + entryOffset + 8) - mainIfdOffset);
    }

    throw std::runtime_error("Required TIFF tag is missing from cached template.");
}

void patchFixedAscii(std::vector<uint8_t> &bytes,
                     uint32_t offset,
                     uint32_t fieldSize,
                     const std::string &value)
{
    if (fieldSize == 0)
        return;
    if (offset == 0)
        throw std::runtime_error("ASCII patch target is missing from the DNG template.");
    if (offset + fieldSize > bytes.size())
        throw std::runtime_error("ASCII patch offset is outside the buffer.");

    std::fill(bytes.begin() + offset, bytes.begin() + offset + fieldSize, 0);
    const size_t bytesToCopy = std::min<size_t>(fieldSize > 0 ? fieldSize - 1 : 0, value.size());
    std::memcpy(bytes.data() + offset, value.data(), bytesToCopy);
}

bool writeAll(int fd, const void *data, size_t size)
{
    const uint8_t *cursor = static_cast<const uint8_t *>(data);
    size_t remaining = size;
    while (remaining > 0) {
        const ssize_t written = ::write(fd, cursor, remaining);
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
        remaining -= static_cast<size_t>(written);
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

void pack16ToDng10Row(const uint16_t *src, uint8_t *dst, uint32_t width, uint8_t sourceShift = 0)
{
    uint32_t x = 0;
    for (; x + 3 < width; x += 4, dst += 5) {
        const uint16_t p0 = static_cast<uint16_t>((src[x + 0] >> sourceShift) & 0x03ffu);
        const uint16_t p1 = static_cast<uint16_t>((src[x + 1] >> sourceShift) & 0x03ffu);
        const uint16_t p2 = static_cast<uint16_t>((src[x + 2] >> sourceShift) & 0x03ffu);
        const uint16_t p3 = static_cast<uint16_t>((src[x + 3] >> sourceShift) & 0x03ffu);

        dst[0] = static_cast<uint8_t>(p0 >> 2);
        dst[1] = static_cast<uint8_t>((p0 << 6) | (p1 >> 4));
        dst[2] = static_cast<uint8_t>((p1 << 4) | (p2 >> 6));
        dst[3] = static_cast<uint8_t>((p2 << 2) | (p3 >> 8));
        dst[4] = static_cast<uint8_t>(p3);
    }

    if (x < width) {
        std::array<uint16_t, 4> tail{};
        std::array<uint8_t, 5> packed{};
        const uint32_t remaining = width - x;
        for (uint32_t i = 0; i < remaining; ++i)
            tail[i] = static_cast<uint16_t>(src[x + i] >> sourceShift);
        Csi2RawConverter::packDng10Row(tail.data(), packed.data(), remaining);
        std::memcpy(dst, packed.data(), packedRowBytes(remaining, RawBitDepth::Bits10));
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
        lut[i] = pispPostprocess(pispDequantizeScalar(static_cast<uint16_t>(i), qmode));
    return lut;
}

const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode0 = buildPispDequantLut(0);
const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode1 = buildPispDequantLut(1);
const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode2 = buildPispDequantLut(2);
const std::array<uint16_t, kPispDequantLutSize> kPispDequantMode3 = buildPispDequantLut(3);

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
            entry.q1Dequantized = pispPostprocess(pispDequantizeScalar(static_cast<uint16_t>(q1), 2));
            entry.q2Dequantized = pispPostprocess(pispDequantizeScalar(static_cast<uint16_t>(q2), 2));
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
        entry.q0Dequantized = pispPostprocess(pispDequantizeScalar(static_cast<uint16_t>(q0), 3));
        entry.q1Dequantized = pispPostprocess(pispDequantizeScalar(static_cast<uint16_t>(q1), 3));
        lut[static_cast<size_t>(packed)] = entry;
    }
    return lut;
}

const std::vector<PispMode2PairDecodeEntry> kPispMode2PairDecodeLut = buildPispMode2PairDecodeLut();
const std::vector<PispMode3DecodeEntry> kPispMode3DecodeLut = buildPispMode3DecodeLut();

inline const PispMode2PairDecodeEntry &pispMode2PairDecodeFast(int field0, int field1)
{
    const size_t idx = (static_cast<size_t>(field0) * kPispMode2Field1Count)
        + static_cast<size_t>(field1);
    return kPispMode2PairDecodeLut[idx];
}

inline const PispMode3DecodeEntry &pispMode3DecodeFast(int packed)
{
    return kPispMode3DecodeLut[static_cast<size_t>(packed)];
}

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
    const PispMode2PairDecodeEntry &pair = pispMode2PairDecodeFast(field0, field1);

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
    const PispMode3DecodeEntry &lo = pispMode3DecodeFast(pack0);
    const PispMode3DecodeEntry &hi = pispMode3DecodeFast(pack1);

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

inline void packDng12Pair(uint8_t *dst, uint16_t p0, uint16_t p1)
{
    const uint16_t p0Dng = static_cast<uint16_t>(p0 >> 4);
    const uint16_t p1Dng = static_cast<uint16_t>(p1 >> 4);

    dst[0] = static_cast<uint8_t>(p0Dng >> 4);
    dst[1] = static_cast<uint8_t>((p0Dng << 4) | (p1Dng >> 8));
    dst[2] = static_cast<uint8_t>(p1Dng);
}

inline void packDng10Quad(uint8_t *dst, uint16_t p0, uint16_t p1, uint16_t p2, uint16_t p3)
{
    const uint16_t p0Dng = static_cast<uint16_t>((p0 >> 6) & 0x03ffu);
    const uint16_t p1Dng = static_cast<uint16_t>((p1 >> 6) & 0x03ffu);
    const uint16_t p2Dng = static_cast<uint16_t>((p2 >> 6) & 0x03ffu);
    const uint16_t p3Dng = static_cast<uint16_t>((p3 >> 6) & 0x03ffu);

    dst[0] = static_cast<uint8_t>(p0Dng >> 2);
    dst[1] = static_cast<uint8_t>((p0Dng << 6) | (p1Dng >> 4));
    dst[2] = static_cast<uint8_t>((p1Dng << 4) | (p2Dng >> 6));
    dst[3] = static_cast<uint8_t>((p2Dng << 2) | (p3Dng >> 8));
    dst[4] = static_cast<uint8_t>(p3Dng);
}

inline void packPispComp1BlockToDng12(uint32_t w0, uint32_t w1, uint8_t *dst)
{
    const PispDecodedQuad even = pispSubBlock(w0);
    const PispDecodedQuad odd = pispSubBlock(w1);

    packDng12Pair(dst + 0, even.p0, odd.p0);
    packDng12Pair(dst + 3, even.p1, odd.p1);
    packDng12Pair(dst + 6, even.p2, odd.p2);
    packDng12Pair(dst + 9, even.p3, odd.p3);
}

inline void packPispComp1BlockToDng10(uint32_t w0, uint32_t w1, uint8_t *dst)
{
    const PispDecodedQuad even = pispSubBlock(w0);
    const PispDecodedQuad odd = pispSubBlock(w1);

    packDng10Quad(dst + 0, even.p0, odd.p0, even.p1, odd.p1);
    packDng10Quad(dst + 5, even.p2, odd.p2, even.p3, odd.p3);
}

inline void packPispComp1BlockToDng8(uint32_t w0, uint32_t w1, uint8_t *dst)
{
    const PispDecodedQuad even = pispSubBlock(w0);
    const PispDecodedQuad odd = pispSubBlock(w1);

    dst[0] = static_cast<uint8_t>(even.p0 >> 8);
    dst[1] = static_cast<uint8_t>(odd.p0 >> 8);
    dst[2] = static_cast<uint8_t>(even.p1 >> 8);
    dst[3] = static_cast<uint8_t>(odd.p1 >> 8);
    dst[4] = static_cast<uint8_t>(even.p2 >> 8);
    dst[5] = static_cast<uint8_t>(odd.p2 >> 8);
    dst[6] = static_cast<uint8_t>(even.p3 >> 8);
    dst[7] = static_cast<uint8_t>(odd.p3 >> 8);
}

inline void decodePispComp1BlockToWorking(uint32_t w0, uint32_t w1, uint16_t *dst)
{
    const PispDecodedQuad even = pispSubBlock(w0);
    const PispDecodedQuad odd = pispSubBlock(w1);

    dst[0] = even.p0;
    dst[1] = odd.p0;
    dst[2] = even.p1;
    dst[3] = odd.p1;
    dst[4] = even.p2;
    dst[5] = odd.p2;
    dst[6] = even.p3;
    dst[7] = odd.p3;
}

void unpackPispCompressedRowToPacked8(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    const uint32_t fullBlocks = width / 8u;
    uint32_t x = 0;
    uint32_t block = 0;
    uint8_t *out = dst;

    for (; block < fullBlocks; ++block, x += 8u, src += 8u, out += 8u) {
        if (kPispCompressMode & 1) {
            packPispComp1BlockToDng8(readLe32(src), readLe32(src + 4), out);
        } else {
            for (uint32_t i = 0; i < 8u; ++i)
                out[i] = static_cast<uint8_t>(pispPostprocess(static_cast<uint16_t>(src[i]) << 8) >> 8);
        }
    }

    const uint32_t remaining = width - x;
    if (remaining > 0) {
        uint16_t working[8]{};

        if (kPispCompressMode & 1)
            decodePispComp1BlockToWorking(readLe32(src), readLe32(src + 4), working);
        else {
            for (uint32_t i = 0; i < remaining; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
        }

        for (uint32_t i = 0; i < remaining; ++i)
            out[i] = static_cast<uint8_t>(working[i] >> 8);
    }
}

void unpackPispCompressedRowToPacked12(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    const uint32_t fullBlocks = width / 8u;
    uint32_t x = 0;
    uint32_t block = 0;

    // Full PiSP COMP1 blocks can be written straight to packed DNG12 bytes.
    // The temporary 16-bit row path is kept only for partial tails/fallback modes.
    for (; block + 1u < fullBlocks; block += 2u, x += 16u, src += 16u) {
        if (kPispCompressMode & 1) {
            uint8_t *out = dst + (static_cast<size_t>(x) / 2u) * 3u;
            packPispComp1BlockToDng12(readLe32(src), readLe32(src + 4), out);
            packPispComp1BlockToDng12(readLe32(src + 8), readLe32(src + 12), out + 12);
        } else {
            uint16_t working[16];
            for (int i = 0; i < 8; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
            for (int i = 0; i < 8; ++i)
                working[8 + i] = pispPostprocess(static_cast<uint16_t>(src[8 + i]) << 8);
            pack16ToDng12Row(working, dst + (static_cast<size_t>(x) / 2u) * 3u, 16u);
        }
    }

    for (; block < fullBlocks; ++block, x += 8u, src += 8u) {
        if (kPispCompressMode & 1) {
            packPispComp1BlockToDng12(readLe32(src), readLe32(src + 4),
                                      dst + (static_cast<size_t>(x) / 2u) * 3u);
        } else {
            uint16_t working[8];
            for (int i = 0; i < 8; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
            pack16ToDng12Row(working, dst + (static_cast<size_t>(x) / 2u) * 3u, 8u);
        }
    }

    const uint32_t remaining = width - x;
    if (remaining > 0) {
        uint16_t working[8]{};
        uint8_t packed[24]{};

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

void unpackPispCompressedRowToPacked10(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    const uint32_t fullBlocks = width / 8u;
    uint32_t x = 0;
    uint32_t block = 0;
    uint8_t *out = dst;

    for (; block + 1u < fullBlocks; block += 2u, x += 16u, src += 16u, out += 20u) {
        if (kPispCompressMode & 1) {
            packPispComp1BlockToDng10(readLe32(src), readLe32(src + 4), out);
            packPispComp1BlockToDng10(readLe32(src + 8), readLe32(src + 12), out + 10);
        } else {
            uint16_t working[16];
            for (int i = 0; i < 8; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
            for (int i = 0; i < 8; ++i)
                working[8 + i] = pispPostprocess(static_cast<uint16_t>(src[8 + i]) << 8);
            pack16ToDng10Row(working, out, 16u, 6);
        }
    }

    for (; block < fullBlocks; ++block, x += 8u, src += 8u, out += 10u) {
        if (kPispCompressMode & 1) {
            packPispComp1BlockToDng10(readLe32(src), readLe32(src + 4), out);
        } else {
            uint16_t working[8];
            for (int i = 0; i < 8; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
            pack16ToDng10Row(working, out, 8u, 6);
        }
    }

    const uint32_t remaining = width - x;
    if (remaining > 0) {
        uint16_t working[8]{};
        uint8_t packed[10]{};

        if (kPispCompressMode & 1)
            decodePispComp1BlockToWorking(readLe32(src), readLe32(src + 4), working);
        else {
            for (uint32_t i = 0; i < remaining; ++i)
                working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
        }

        pack16ToDng10Row(working, packed, remaining, 6);
        std::memcpy(dst + packedRowBytes(x, RawBitDepth::Bits10),
                    packed,
                    packedRowBytes(remaining, RawBitDepth::Bits10));
    }
}

void writeDngRowBytes(const RawFrameView &frame,
                      const CdngEncoder::EncodedRawLayout &layout,
                      const uint8_t *src,
                      uint8_t *dst)
{
    if (frame.packing == PixelPacking::Csi2Packed) {
        if (frame.bitDepth == RawBitDepth::Bits10)
            packCsi2Raw10ToDngRow(src, dst, frame.width);
        else if (frame.bitDepth == RawBitDepth::Bits12)
            packCsi2Raw12ToDngRow(src, dst, frame.width);
        else
            throw std::runtime_error("Unsupported CSI2 packed bit depth.");
    } else if (frame.packing == PixelPacking::PispCompressed1) {
        if (layout.bitDepth == RawBitDepth::Bits8)
            unpackPispCompressedRowToPacked8(src, dst, frame.width);
        else if (layout.bitDepth == RawBitDepth::Bits10)
            unpackPispCompressedRowToPacked10(src, dst, frame.width);
        else
            unpackPispCompressedRowToPacked12(src, dst, frame.width);
    } else if (frame.bitDepth == RawBitDepth::Bits16) {
        std::memcpy(dst, src, layout.rowBytes);
    } else if (frame.bitDepth == RawBitDepth::Bits8) {
        const uint8_t sourceBytes = unpackedBytesPerSampleForFrame(frame);
        if (sourceBytes == 1u)
            std::memcpy(dst, src, layout.rowBytes);
        else if (sourceBytes == 2u)
            copyUnpacked16ToDng8Row(src, dst, frame.width);
        else
            throw std::runtime_error("Unsupported unpacked 8-bit container.");
    } else {
        const auto *src16 = reinterpret_cast<const uint16_t *>(src);
        if (frame.bitDepth == RawBitDepth::Bits10)
            Csi2RawConverter::packDng10Row(src16, dst, frame.width);
        else if (frame.bitDepth == RawBitDepth::Bits12)
            Csi2RawConverter::packDng12Row(src16, dst, frame.width);
        else
            throw std::runtime_error("Unsupported unpacked bit depth.");
    }
}

void readJpegSamples(const RawFrameView &frame, RawBitDepth depth, const uint8_t *src, uint16_t *dst)
{
    const unsigned bits = static_cast<unsigned>(depth);
    if (frame.packing == PixelPacking::PispCompressed1) {
        const unsigned shift = 16 - bits;
        for (uint32_t x = 0; x < frame.width; x += 8, src += 8) {
            uint16_t working[8];
            const unsigned n = std::min(8u, frame.width - x);
            if (kPispCompressMode & 1)
                decodePispComp1BlockToWorking(readLe32(src), readLe32(src + 4), working);
            else
                for (unsigned i = 0; i < n; ++i)
                    working[i] = pispPostprocess(static_cast<uint16_t>(src[i]) << 8);
            for (unsigned i = 0; i < n; ++i)
                dst[x + i] = static_cast<uint16_t>(working[i] >> shift);
        }
    } else if (frame.packing == PixelPacking::Csi2Packed) {
        if (depth == RawBitDepth::Bits10)
            Csi2RawConverter::unpack10Row(src, dst, frame.width);
        else
            Csi2RawConverter::unpack12Row(src, dst, frame.width);
    } else {
        const unsigned bytes = unpackedBytesPerSampleForFrame(frame);
        for (uint32_t x = 0; x < frame.width; ++x) {
            if (bytes == 1)
                dst[x] = src[x];
            else if (bits == 8)
                dst[x] = readLe16(src + x * 2u) >> 8;
            else
                dst[x] = readLe16(src + x * 2u);
        }
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

uint32_t CdngEncoder::losslessTileRows()
{
    return configuredLosslessTileRows();
}

void CdngEncoder::decodePispRowToMsb16(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    RawFrameView row;
    row.width = width;
    row.packing = PixelPacking::PispCompressed1;
    readJpegSamples(row, RawBitDepth::Bits16, src, dst);
}

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

CdngEncoder::EncodedRawLayout CdngEncoder::prepareRawLayout(const RawFrameView &frame) const
{
    requireFrame(frame);

    if (frame.packing == PixelPacking::Csi2Packed)
        return preparePackedCsi2Layout(frame);
    if (frame.packing == PixelPacking::PispCompressed1)
        return preparePispCompressedLayout(frame);

    return prepareUnpackedLayout(frame);
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
                               const Metadata &metadata, int destinationFd) const
{
    const auto writeStarted = std::chrono::steady_clock::now();
    lastPerformanceStats_ = PerformanceStats{};
    requireFrame(frame);
    const bool monochrome = frame.bayerPattern == BayerPattern::Monochrome;

    if (!metadata.linearizationTable.empty() &&
        (metadata.compression != CdngCompression::LosslessJpeg ||
         metadata.linearizationTable.size() != (size_t{1} << bitsPerSample(frame.bitDepth))))
        throw std::runtime_error("Invalid quantized RAW linearization table.");

    if (!metadata.jpegSampleLookup.empty() &&
        (metadata.compression != CdngCompression::LosslessJpeg ||
         frame.packing != PixelPacking::PispCompressed1 || metadata.jpegSampleShift > 8 ||
         metadata.jpegSampleLookup.size() != (size_t{1} << (16 - metadata.jpegSampleShift))))
        throw std::runtime_error("Invalid PiSP JPEG sample mapping.");

    const EncodedRawLayout layout = prepareRawLayout(frame);
    if (layout.rowBytes == 0 || layout.byteCount == 0)
        throw std::runtime_error("Encoded raw layout is empty.");

    const bool lossless = metadata.compression == CdngCompression::LosslessJpeg;
    const bool profileLosslessDetails = lossless && detailedLosslessProfilingEnabled();
    const uint32_t rowsPerStrip = lossless ? losslessTileRows() : frame.height;
    // A padded tile wider than the image is misread by LibRaw. Use two
    // aligned columns for non-aligned sensor widths, without changing ImageWidth.
    const uint32_t tileWidth = frame.width % 16 == 0 ? frame.width
        : ((frame.width / 2u + 15u) & ~15u);
    std::vector<uint32_t> stripOffsets;
    std::vector<uint32_t> stripByteCounts;
    struct OutputFile {
        int fd = -1;
        const std::string &path;
        bool complete = false;
        bool borrowed = false;
        ~OutputFile() {
            if (fd >= 0 && !borrowed) {
                ::close(fd);
                if (!complete)
                    ::unlink(path.c_str());
            }
        }
    } output{ -1, path, false, destinationFd >= 0 };
    if (destinationFd >= 0 && (::ftruncate(destinationFd, 0) != 0 ||
                              ::lseek(destinationFd, 0, SEEK_SET) < 0))
        throw std::system_error(errno, std::generic_category(), "Failed to reset DNG destination");
    uint64_t payloadByteCount = layout.byteCount;
    if (lossless) {
        if (frame.width % 2 || frame.width < 16)
            throw std::runtime_error("Lossless JPEG requires an even Bayer width of at least 16 pixels.");
        const uint64_t minRow = frame.packing == PixelPacking::PispCompressed1
            ? ((static_cast<uint64_t>(frame.width) + 7u) / 8u) * 8u
            : frame.packing == PixelPacking::Csi2Packed
                ? (frame.bitDepth == RawBitDepth::Bits10 ? ((frame.width + 3u) / 4u) * 5u
                                                        : ((frame.width + 1u) / 2u) * 3u)
                : static_cast<uint64_t>(frame.width) * unpackedBytesPerSampleForFrame(frame);
        if (frame.stride < minRow || frame.size < (static_cast<uint64_t>(frame.height) - 1) * frame.stride + minRow)
            throw std::runtime_error("Lossless JPEG source buffer is smaller than its RAW geometry.");
        output.fd = destinationFd >= 0 ? destinationFd
            : ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
        if (output.fd < 0)
            throw std::system_error(errno, std::generic_category(), "Failed to open DNG: " + path);
        const std::array<uint8_t, 8> placeholder{};
        if (!writeAll(output.fd, placeholder.data(), placeholder.size()))
            throw std::system_error(errno, std::generic_category(), "Failed to reserve DNG header");
        // Decode and encode one row at a time. The row remains in L1/L2 cache
        // instead of materializing and rereading an entire decoded tile.
        jpegSamples_.resize(tileWidth);
        const auto readSamples = [&](const RawFrameView &tile, const uint8_t *source, uint16_t *target) {
            if (metadata.jpegSampleLookup.empty()) {
                readJpegSamples(tile, layout.bitDepth, source, target);
            } else {
                readJpegSamples(tile, RawBitDepth::Bits16, source, target);
                for (uint32_t x = 0; x < tile.width; ++x)
                    target[x] = metadata.jpegSampleLookup[target[x] >> metadata.jpegSampleShift];
            }
        };
        payloadByteCount = 0;
        for (uint32_t y = 0; y < frame.height; y += rowsPerStrip) {
            const uint32_t rows = std::min(rowsPerStrip, frame.height - y);
            for (uint32_t tileX = 0; tileX < frame.width; tileX += tileWidth) {
                RawFrameView tile = frame;
                tile.width = std::min(tileWidth, frame.width - tileX);
                tile.unpackedBytesPerSample = unpackedBytesPerSampleForFrame(frame);
                const size_t byteOffset = frame.packing == PixelPacking::PispCompressed1 ? tileX
                    : frame.packing == PixelPacking::Csi2Packed
                        ? static_cast<size_t>(tileX) * bitsPerSample(frame.bitDepth) / 8u
                        : static_cast<size_t>(tileX) * tile.unpackedBytesPerSample;
                const auto encodeStarted = std::chrono::steady_clock::now();
                jpegEncoder_.begin(tileWidth, rowsPerStrip, bitsPerSample(layout.bitDepth),
                                   metadata.compactJpegCodes, monochrome);
                for (uint32_t row = 0; row < rows; ++row) {
                    uint16_t *target = jpegSamples_.data();
                    const uint8_t *source = frame.data +
                        static_cast<size_t>(y + row) * frame.stride + byteOffset;
                    std::chrono::steady_clock::time_point decodeFinished;
                    if (profileLosslessDetails) {
                        const auto decodeStarted = std::chrono::steady_clock::now();
                        readSamples(tile, source, target);
                        decodeFinished = std::chrono::steady_clock::now();
                        lastPerformanceStats_.losslessDecodeWallUs +=
                            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                decodeFinished - decodeStarted).count());
                        ++lastPerformanceStats_.losslessProfiledSourceRows;
                    } else {
                        readSamples(tile, source, target);
                    }
                    for (uint32_t x = tile.width; x < tileWidth; ++x)
                        target[x] = target[x - 2];
                    if (profileLosslessDetails) {
                        jpegEncoder_.appendRow(target);
                        lastPerformanceStats_.losslessEntropyWallUs +=
                            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now() - decodeFinished).count());
                    } else {
                        jpegEncoder_.appendRow(target);
                    }
                }
                if (profileLosslessDetails && rows < rowsPerStrip) {
                    const auto paddingStarted = std::chrono::steady_clock::now();
                    for (uint32_t row = rows; row < rowsPerStrip; ++row)
                        jpegEncoder_.appendRow(jpegSamples_.data());
                    lastPerformanceStats_.losslessEntropyWallUs += elapsedMicroseconds(paddingStarted);
                } else {
                    for (uint32_t row = rows; row < rowsPerStrip; ++row)
                        jpegEncoder_.appendRow(jpegSamples_.data());
                }
                const auto &jpeg = jpegEncoder_.finish();
                lastPerformanceStats_.losslessEncodeWallUs += elapsedMicroseconds(encodeStarted);
                stripOffsets.push_back(checkedTiffOffset(8u + payloadByteCount));
                stripByteCounts.push_back(checkedTiffOffset(jpeg.size()));
                const auto payloadWriteStarted = std::chrono::steady_clock::now();
                if (!writeAll(output.fd, jpeg.data(), jpeg.size()))
                    throw std::system_error(errno, std::generic_category(), "Failed to write JPEG tile");
                lastPerformanceStats_.losslessPayloadWriteWallUs += elapsedMicroseconds(payloadWriteStarted);
                lastPerformanceStats_.losslessPayloadBytes += jpeg.size();
                ++lastPerformanceStats_.losslessTileWrites;
                payloadByteCount += jpeg.size();
            }
        }
    }
    const uint16_t bits = bitsPerSample(layout.bitDepth);
    std::array<int32_t, 8> encodedBlackLevels{};
    const auto blackIndices = blackLevelCfaIndices(frame.bayerPattern);
    for (size_t i = 0; i < blackIndices.size(); ++i) {
        const float sourceBlackLevel = metadata.hasBlackLevels
            ? metadata.blackLevels[blackIndices[i]] : static_cast<float>(kDngBlackLevel);
        encodedBlackLevels[i * 2] = blackLevelForEncodedDepth(
            sourceBlackLevel, layout.bitDepth, metadata.blackLevelsOnContainerScale);
        encodedBlackLevels[i * 2 + 1] = 1;
    }
    const uint16_t compression = lossless ? 7 : 1;
    const uint16_t photometric = monochrome ? 34892 : 32803;
    const uint16_t samplesPerPixel = 1;
    const uint16_t planarConfig = 1;
    const uint16_t sampleFormatUint = 1;
    const uint16_t orientationTopLeft = 1;
    const uint32_t imageWidth = frame.width;
    const uint32_t imageHeight = frame.height;
    const uint32_t white = metadata.linearizationTable.empty()
        ? whiteLevel(layout.bitDepth) : metadata.linearizationTable.back();
    static constexpr uint8_t tiffEpStandardId[4] = { 0, 0, 0, 1 };
    const uint16_t defaultCropOrigin[2] = { 0u, 0u };
    const uint16_t defaultCropSize[2] = {
        static_cast<uint16_t>(std::min<uint32_t>(imageWidth, std::numeric_limits<uint16_t>::max())),
        static_cast<uint16_t>(std::min<uint32_t>(imageHeight, std::numeric_limits<uint16_t>::max())),
    };
    const int32_t baselineExposure[2] = { 0, 1 };
    const uint32_t analogBalance[6] = { 1u, 1u, 1u, 1u, 1u, 1u };

    if (payloadByteCount > 0xffffffffu)
        throw std::runtime_error("DNG payload is too large for classic TIFF.");

    constexpr uint32_t tiffHeaderBytes = 8;
    const uint32_t stripOffset = tiffHeaderBytes;
    const uint32_t stripByteCount = checkedTiffOffset(payloadByteCount);
    const uint64_t ifdOffset64 = align4(static_cast<uint64_t>(stripOffset) + payloadByteCount);
    const uint32_t ifdOffset = checkedTiffOffset(ifdOffset64);
    const size_t paddingSize = static_cast<size_t>(ifdOffset64 - (static_cast<uint64_t>(stripOffset) + payloadByteCount));

    const uint64_t wallClock = metadata.wallClockTimestampUs
        ? metadata.wallClockTimestampUs
        : static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::system_clock::now().time_since_epoch()).count());
    const std::string originalDate = dateTimeString(wallClock);

    const std::array<uint8_t, 8> tc = timeCode(metadata, wallClock);
    const DngColorProfile colorProfile = monochrome ? DngColorProfile{} : colorProfileForMetadata(metadata);
    const uint16_t exifSensitivityType = 3; // ISO speed

    auto sameDynamicMetadata = [](const DynamicMetadataCache &cache, const Metadata &dynamic) {
        return cache.valid &&
               std::fabs(cache.fps - dynamic.fps) < 0.0001 &&
               cache.iso == dynamic.iso &&
               cache.shutterUs == dynamic.shutterUs &&
               std::fabs(cache.colourGain0 - dynamic.colourGains[0]) < 0.0001f &&
               std::fabs(cache.colourGain1 - dynamic.colourGains[1]) < 0.0001f;
    };

    if (!sameDynamicMetadata(dynamicMetadataCache_, metadata)) {
        dynamicMetadataCache_.valid = true;
        dynamicMetadataCache_.fps = metadata.fps;
        dynamicMetadataCache_.iso = metadata.iso;
        dynamicMetadataCache_.shutterUs = metadata.shutterUs;
        dynamicMetadataCache_.colourGain0 = metadata.colourGains[0];
        dynamicMetadataCache_.colourGain1 = metadata.colourGains[1];
        const auto frameRate = recordingFrameRate(metadata.fps);
        dynamicMetadataCache_.fpsRat[0] = frameRate.first;
        dynamicMetadataCache_.fpsRat[1] = frameRate.second;
        dynamicMetadataCache_.exifExposure[0] = std::max<uint32_t>(1u, metadata.shutterUs);
        dynamicMetadataCache_.exifExposure[1] = 1000000u;
        dynamicMetadataCache_.exifIso = static_cast<uint16_t>(std::clamp<uint32_t>(metadata.iso, 1u, 65535u));
        dynamicMetadataCache_.exifIsoLong = std::max<uint32_t>(1u, metadata.iso);
        dynamicMetadataCache_.exifExposureIndex[0] = std::max<uint32_t>(1u, metadata.iso);
        dynamicMetadataCache_.exifExposureIndex[1] = 1u;

        float neutralValues[3] = {
            1.0f / std::max(metadata.colourGains[0], 0.0001f),
            1.0f,
            1.0f / std::max(metadata.colourGains[1], 0.0001f),
        };
        encodeRationalArray(neutralValues, 3, dynamicMetadataCache_.neutral);
    }

    auto takeReusableBytes = [](std::vector<uint8_t> &store, size_t reserveSize) {
        std::vector<uint8_t> bytes = std::move(store);
        bytes.clear();
        if (bytes.capacity() < reserveSize)
            bytes.reserve(reserveSize);
        return bytes;
    };

    auto recycleReusableBytes = [](std::vector<uint8_t> &store, std::vector<uint8_t> bytes) {
        bytes.clear();
        store = std::move(bytes);
    };

    auto makeMainIfdLayoutKey = [&]() {
        MainIfdLayoutCache key;
        key.valid = true;
        key.imageWidth = imageWidth;
        key.imageHeight = imageHeight;
        key.bits = bits;
        key.compression = compression;
        key.stripByteCount = stripByteCount;
        key.hasClipName = !metadata.clipName.empty();
        key.hasFrameFileName = !metadata.frameFileName.empty();
        key.hasMake = !metadata.make.empty();
        key.hasModel = !metadata.model.empty();
        key.hasUniqueCameraModel = !metadata.uniqueCameraModel.empty();
        key.hasSerial = !metadata.serial.empty();
        key.hasReelName = !metadata.reelName.empty();
        key.hasCameraLabel = !metadata.cameraLabel.empty();
        key.clipNameLength = static_cast<uint32_t>(metadata.clipName.size());
        key.frameFileNameLength = static_cast<uint32_t>(metadata.frameFileName.size());
        key.makeLength = static_cast<uint32_t>(metadata.make.size());
        key.modelLength = static_cast<uint32_t>(metadata.model.size());
        key.uniqueCameraModelLength = static_cast<uint32_t>(metadata.uniqueCameraModel.size());
        key.serialLength = static_cast<uint32_t>(metadata.serial.size());
        key.reelNameLength = static_cast<uint32_t>(metadata.reelName.size());
        key.cameraLabelLength = static_cast<uint32_t>(metadata.cameraLabel.size());
        key.softwareLength = static_cast<uint32_t>(metadata.software.size());
        return key;
    };

    auto sameMainIfdLayout = [](const MainIfdLayoutCache &left, const MainIfdLayoutCache &right) {
        return left.valid == right.valid &&
               left.imageWidth == right.imageWidth &&
               left.imageHeight == right.imageHeight &&
               left.bits == right.bits &&
               left.compression == right.compression &&
               left.stripByteCount == right.stripByteCount &&
               left.hasClipName == right.hasClipName &&
               left.hasFrameFileName == right.hasFrameFileName &&
               left.hasMake == right.hasMake &&
               left.hasModel == right.hasModel &&
               left.hasUniqueCameraModel == right.hasUniqueCameraModel &&
               left.hasSerial == right.hasSerial &&
               left.hasReelName == right.hasReelName &&
               left.hasCameraLabel == right.hasCameraLabel &&
               left.clipNameLength == right.clipNameLength &&
               left.frameFileNameLength == right.frameFileNameLength &&
               left.makeLength == right.makeLength &&
               left.modelLength == right.modelLength &&
               left.uniqueCameraModelLength == right.uniqueCameraModelLength &&
               left.serialLength == right.serialLength &&
               left.reelNameLength == right.reelNameLength &&
               left.cameraLabelLength == right.cameraLabelLength &&
               left.softwareLength == right.softwareLength;
    };

    const MainIfdLayoutCache layoutKey = makeMainIfdLayoutKey();

    auto templateMatches = [&]() {
        return !lossless && ifdTemplateCache_.valid &&
               ifdTemplateCache_.bayerPattern == frame.bayerPattern &&
               ifdTemplateCache_.colorProfileKey == colorProfile.cacheKey &&
               sameMainIfdLayout(ifdTemplateCache_.layout, layoutKey) &&
               ifdTemplateCache_.clipName == metadata.clipName &&
               ifdTemplateCache_.make == metadata.make &&
               ifdTemplateCache_.model == metadata.model &&
               ifdTemplateCache_.uniqueCameraModel == metadata.uniqueCameraModel &&
               ifdTemplateCache_.serial == metadata.serial &&
               ifdTemplateCache_.reelName == metadata.reelName &&
               ifdTemplateCache_.cameraLabel == metadata.cameraLabel &&
               ifdTemplateCache_.software == metadata.software;
    };

    auto buildMainIfdBuffer = [&](tiff::MemoryBuffer &buffer, uint32_t exifIfdOffset, size_t reserveHint) {
        buffer.bytes.clear();
        if (buffer.bytes.capacity() < reserveHint)
            buffer.bytes.reserve(reserveHint);

        tiff::IfdBuilder ifd;
        const uint32_t typeFullResolution = 0;
        ifd.add(254, tiff::Long, 1, &typeFullResolution);
        ifd.add(256, tiff::Long, 1, &imageWidth);
        ifd.add(257, tiff::Long, 1, &imageHeight);
        ifd.add(258, tiff::Short, 1, &bits);
        ifd.add(259, tiff::Short, 1, &compression);
        ifd.add(262, tiff::Short, 1, &photometric);
        if (!lossless)
            ifd.add(273, tiff::Long, 1, &stripOffset);
        ifd.add(274, tiff::Short, 1, &orientationTopLeft);
        ifd.add(277, tiff::Short, 1, &samplesPerPixel);
        if (lossless) {
            ifd.add(322, tiff::Long, 1, &tileWidth);
            ifd.add(323, tiff::Long, 1, &rowsPerStrip);
            ifd.add(324, tiff::Long, static_cast<uint32_t>(stripOffsets.size()), stripOffsets.data());
            ifd.add(325, tiff::Long, static_cast<uint32_t>(stripByteCounts.size()), stripByteCounts.data());
        } else {
            ifd.add(278, tiff::Long, 1, &rowsPerStrip);
            ifd.add(279, tiff::Long, 1, &stripByteCount);
        }
        ifd.add(284, tiff::Short, 1, &planarConfig);
        ifd.add(339, tiff::Short, 1, &sampleFormatUint);
        ifd.add(0x8769, tiff::Long, 1, &exifIfdOffset); // ExifIFD pointer
        ifd.add(0x9216, tiff::Byte, 4, tiffEpStandardId); // TIFF-EPStandardID

        if (!metadata.clipName.empty())
            appendAscii(ifd, 269, metadata.clipName); // DocumentName
        if (!metadata.clipName.empty())
            appendAscii(ifd, 270, metadata.clipName); // ImageDescription
        if (!metadata.frameFileName.empty())
            appendAscii(ifd, 285, metadata.frameFileName); // PageName
        if (!metadata.make.empty())
            appendAscii(ifd, 271, metadata.make);
        if (!metadata.model.empty())
            appendAscii(ifd, 272, metadata.model);
        appendAscii(ifd, 305, metadata.software);
        appendAscii(ifd, 306, originalDate);

        static constexpr uint8_t dngVersion[4] = { 1, 4, 0, 0 };
        static constexpr uint8_t dngBackwardVersion[4] = { 1, 3, 0, 0 };
        ifd.add(0xC612, tiff::Byte, 4, dngVersion);
        ifd.add(0xC613, tiff::Byte, 4, dngBackwardVersion);
        appendAscii(ifd, 0xC614, metadata.uniqueCameraModel);
        if (!metadata.serial.empty())
            appendAscii(ifd, 0xC62F, metadata.serial);
        if (!metadata.reelName.empty())
            appendAscii(ifd, kCinemaDngTagReelName, metadata.reelName);
        if (!metadata.cameraLabel.empty())
            appendAscii(ifd, kCinemaDngTagCameraLabel, metadata.cameraLabel);

        const uint16_t repeatDim[2] = { uint16_t(monochrome ? 1 : 2), uint16_t(monochrome ? 1 : 2) };
        if (!monochrome) {
            ifd.add(0x828D, tiff::Short, 2, repeatDim);
            ifd.add(0x828E, tiff::Byte, 4, cfaPattern(frame.bayerPattern));
        }
        ifd.add(0xC619, tiff::Short, 2, repeatDim);

        ifd.add(0xC61A, tiff::Rational, monochrome ? 1 : 4, encodedBlackLevels.data());
        if (!metadata.linearizationTable.empty())
            ifd.add(0xC618, tiff::Short, static_cast<uint32_t>(metadata.linearizationTable.size()),
                    metadata.linearizationTable.data());
        ifd.add(0xC61D, tiff::Long, 1, &white);

        if (!monochrome) {
            static constexpr int32_t identityCalibration[18] = {
                1, 1, 0, 1, 0, 1,
                0, 1, 1, 1, 0, 1,
                0, 1, 0, 1, 1, 1,
            };
            int32_t colorMatrix1[18]{};
            int32_t colorMatrix2[18]{};
            encodeRationalArray(colorProfile.colorMatrix1, 9, colorMatrix1, kDngProfileRationalScale);
            encodeRationalArray(colorProfile.colorMatrix2, 9, colorMatrix2, kDngProfileRationalScale);
            ifd.add(0xC621, tiff::SRational, 9, colorMatrix1);
            ifd.add(0xC622, tiff::SRational, 9, colorMatrix2);
            if (colorProfile.hasForwardMatrix1) {
                int32_t forwardMatrix1[18]{};
                encodeRationalArray(colorProfile.forwardMatrix1, 9, forwardMatrix1, kDngProfileRationalScale);
                ifd.add(0xC714, tiff::SRational, 9, forwardMatrix1);
            }
            if (colorProfile.hasForwardMatrix2) {
                int32_t forwardMatrix2[18]{};
                encodeRationalArray(colorProfile.forwardMatrix2, 9, forwardMatrix2, kDngProfileRationalScale);
                ifd.add(0xC715, tiff::SRational, 9, forwardMatrix2);
            }
            ifd.add(0xC65A, tiff::Short, 1, &colorProfile.illuminant1);
            ifd.add(0xC65B, tiff::Short, 1, &colorProfile.illuminant2);
            ifd.add(0xC623, tiff::SRational, 9, identityCalibration);
            ifd.add(0xC624, tiff::SRational, 9, identityCalibration);
            ifd.add(0xC627, tiff::Rational, 3, analogBalance);
            ifd.add(0xC628, tiff::Rational, 3, dynamicMetadataCache_.neutral);
        }

        ifd.add(0xC61F, tiff::Short, 2, defaultCropOrigin);
        ifd.add(0xC620, tiff::Short, 2, defaultCropSize);
        ifd.add(0xC62A, tiff::SRational, 1, baselineExposure);

        appendAscii(ifd, 0x9003, originalDate);
        ifd.add(kCinemaDngTagTimeCodes, tiff::Byte, static_cast<uint32_t>(tc.size()), tc.data());
        ifd.add(kCinemaDngTagFrameRate, tiff::SRational, 1, dynamicMetadataCache_.fpsRat);

        const uint32_t builtIfdOffset = ifd.buildAt(buffer, ifdOffset);
        if (builtIfdOffset != ifdOffset)
            throw std::runtime_error("DNG IFD offset mismatch.");
    };

    if (!templateMatches()) {
        tiff::MemoryBuffer mainIfd;
        mainIfd.bytes = takeReusableBytes(mainIfdScratch_, std::max<size_t>(16 * 1024, mainIfdLayoutCache_.mainIfdSize));

        buildMainIfdBuffer(mainIfd, 0, 16 * 1024);
        const uint32_t exifIfdOffset =
            checkedTiffOffset(align4(static_cast<uint64_t>(ifdOffset) + mainIfd.bytes.size()));
        buildMainIfdBuffer(mainIfd, exifIfdOffset, std::max<size_t>(16 * 1024, mainIfd.bytes.size()));

        tiff::IfdBuilder exifIfd;
        exifIfd.add(0x829A, tiff::Rational, 1, dynamicMetadataCache_.exifExposure); // ExposureTime
        exifIfd.add(0x8827, tiff::Short, 1, &dynamicMetadataCache_.exifIso);        // PhotographicSensitivity
        exifIfd.add(0x8830, tiff::Short, 1, &exifSensitivityType); // SensitivityType
        exifIfd.add(0x8831, tiff::Long, 1, &dynamicMetadataCache_.exifIsoLong);     // StandardOutputSensitivity
        exifIfd.add(0x8832, tiff::Long, 1, &dynamicMetadataCache_.exifIsoLong);     // RecommendedExposureIndex
        exifIfd.add(0x8833, tiff::Long, 1, &dynamicMetadataCache_.exifIsoLong);     // ISOSpeed
        appendAscii(exifIfd, 0x9003, originalDate);           // DateTimeOriginal
        exifIfd.add(0xA215, tiff::Rational, 1, dynamicMetadataCache_.exifExposureIndex); // ExposureIndex
        exifIfd.add(0xA002, tiff::Long, 1, &imageWidth);      // PixelXDimension
        exifIfd.add(0xA003, tiff::Long, 1, &imageHeight);     // PixelYDimension

        tiff::MemoryBuffer exifBuffer;
        exifBuffer.bytes = takeReusableBytes(exifScratch_, 512);
        const uint32_t builtExifOffset = exifIfd.buildAt(exifBuffer, exifIfdOffset);
        if (builtExifOffset != exifIfdOffset)
            throw std::runtime_error("ExifIFD offset mismatch.");
        const uint32_t exifRelativeOffset = static_cast<uint32_t>(exifIfdOffset - ifdOffset);
        if (mainIfd.bytes.size() < exifRelativeOffset)
            mainIfd.bytes.resize(exifRelativeOffset, 0);
        mainIfd.bytes.insert(mainIfd.bytes.end(), exifBuffer.bytes.begin(), exifBuffer.bytes.end());
        recycleReusableBytes(exifScratch_, std::move(exifBuffer.bytes));

        ifdTemplateCache_ = IfdTemplateCache{};
        ifdTemplateCache_.valid = true;
        ifdTemplateCache_.bayerPattern = frame.bayerPattern;
        ifdTemplateCache_.colorProfileKey = colorProfile.cacheKey;
        ifdTemplateCache_.layout = layoutKey;
        ifdTemplateCache_.layout.mainIfdSize = mainIfd.bytes.size();
        ifdTemplateCache_.clipName = metadata.clipName;
        ifdTemplateCache_.make = metadata.make;
        ifdTemplateCache_.model = metadata.model;
        ifdTemplateCache_.uniqueCameraModel = metadata.uniqueCameraModel;
        ifdTemplateCache_.serial = metadata.serial;
        ifdTemplateCache_.reelName = metadata.reelName;
        ifdTemplateCache_.cameraLabel = metadata.cameraLabel;
        ifdTemplateCache_.software = metadata.software;
        ifdTemplateCache_.bytes = std::move(mainIfd.bytes);
        ifdTemplateCache_.pageNameFieldSize = metadata.frameFileName.empty()
            ? 0u
            : static_cast<uint32_t>(metadata.frameFileName.size() + 1u);
        ifdTemplateCache_.pageNameOffset =
            metadata.frameFileName.empty() ? 0u
                                           : findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, 0, 285, false);
        ifdTemplateCache_.modifyDateOffset =
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, 0, 306, false);
        ifdTemplateCache_.dateTimeOriginalOffset =
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, exifRelativeOffset, 0x9003, false);
        ifdTemplateCache_.neutralOffset = monochrome ? 0 :
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, 0, 0xC628, false);
        ifdTemplateCache_.blackLevelsOffset =
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, 0, 0xC61A, false);
        ifdTemplateCache_.timeCodeOffset =
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, 0, kCinemaDngTagTimeCodes, false);
        ifdTemplateCache_.frameRateOffset =
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, 0, kCinemaDngTagFrameRate, false);
        ifdTemplateCache_.exposureTimeOffset =
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, exifRelativeOffset, 0x829A, false);
        ifdTemplateCache_.exifIsoShortOffset =
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, exifRelativeOffset, 0x8827, true);
        ifdTemplateCache_.exifIsoLongOffset1 =
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, exifRelativeOffset, 0x8831, true);
        ifdTemplateCache_.exifIsoLongOffset2 =
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, exifRelativeOffset, 0x8832, true);
        ifdTemplateCache_.exifIsoLongOffset3 =
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, exifRelativeOffset, 0x8833, true);
        ifdTemplateCache_.exposureIndexOffset =
            findIfdTagValueOffset(ifdTemplateCache_.bytes, ifdOffset, exifRelativeOffset, 0xA215, false);
        recycleReusableBytes(mainIfdScratch_, std::vector<uint8_t>{});
    }

    std::vector<uint8_t> ifdBytes = takeReusableBytes(mainIfdScratch_, ifdTemplateCache_.bytes.size());
    ifdBytes.assign(ifdTemplateCache_.bytes.begin(), ifdTemplateCache_.bytes.end());
    if (ifdTemplateCache_.pageNameFieldSize > 0)
        patchFixedAscii(ifdBytes, ifdTemplateCache_.pageNameOffset, ifdTemplateCache_.pageNameFieldSize, metadata.frameFileName);
    patchFixedAscii(ifdBytes, ifdTemplateCache_.modifyDateOffset, static_cast<uint32_t>(originalDate.size() + 1u), originalDate);
    patchFixedAscii(ifdBytes, ifdTemplateCache_.dateTimeOriginalOffset, static_cast<uint32_t>(originalDate.size() + 1u), originalDate);
    std::memcpy(ifdBytes.data() + ifdTemplateCache_.blackLevelsOffset,
                encodedBlackLevels.data(), monochrome ? 2 * sizeof(int32_t) : sizeof(encodedBlackLevels));
    if (!monochrome) std::memcpy(ifdBytes.data() + ifdTemplateCache_.neutralOffset,
                dynamicMetadataCache_.neutral,
                sizeof(dynamicMetadataCache_.neutral));
    std::memcpy(ifdBytes.data() + ifdTemplateCache_.timeCodeOffset, tc.data(), tc.size());
    std::memcpy(ifdBytes.data() + ifdTemplateCache_.frameRateOffset,
                dynamicMetadataCache_.fpsRat,
                sizeof(dynamicMetadataCache_.fpsRat));
    std::memcpy(ifdBytes.data() + ifdTemplateCache_.exposureTimeOffset,
                dynamicMetadataCache_.exifExposure,
                sizeof(dynamicMetadataCache_.exifExposure));
    putLe16(ifdBytes.data() + ifdTemplateCache_.exifIsoShortOffset, dynamicMetadataCache_.exifIso);
    putLe32(ifdBytes.data() + ifdTemplateCache_.exifIsoLongOffset1, dynamicMetadataCache_.exifIsoLong);
    putLe32(ifdBytes.data() + ifdTemplateCache_.exifIsoLongOffset2, dynamicMetadataCache_.exifIsoLong);
    putLe32(ifdBytes.data() + ifdTemplateCache_.exifIsoLongOffset3, dynamicMetadataCache_.exifIsoLong);
    std::memcpy(ifdBytes.data() + ifdTemplateCache_.exposureIndexOffset,
                dynamicMetadataCache_.exifExposureIndex,
                sizeof(dynamicMetadataCache_.exifExposureIndex));

    std::array<uint8_t, 8> header{};
    header[0] = 'I';
    header[1] = 'I';
    putLe16(header.data() + 2, 42);
    putLe32(header.data() + 4, ifdOffset);

    static constexpr std::array<uint8_t, 3> padding{ 0, 0, 0 };

    int openFlags = O_WRONLY | O_CREAT | O_TRUNC;
#ifdef O_CLOEXEC
    openFlags |= O_CLOEXEC;
#endif
    if (!lossless)
        output.fd = destinationFd >= 0 ? destinationFd : ::open(path.c_str(), openFlags, 0644);
    const int fd = output.fd;
    if (fd < 0)
        throw std::system_error(errno, std::generic_category(), "Failed to open DNG for writing: " + path);

    ::posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);
    bool ok = true;
    if (lossless) {
        size_t written = 0;
        while (written < header.size()) {
            const ssize_t n = ::pwrite(fd, header.data() + written, header.size() - written, written);
            if (n < 0 && errno == EINTR)
                continue;
            if (n <= 0) { ok = false; break; }
            written += static_cast<size_t>(n);
        }
    } else {
        ok = writeAll(fd, header.data(), header.size());
    }
    if (ok && !lossless)
        ok = writeRawPayload(fd, frame, layout);
    if (ok && paddingSize > 0)
        ok = writeAll(fd, padding.data(), paddingSize);
    if (ok)
        ok = writeAll(fd, ifdBytes.data(), ifdBytes.size());
    int writeErrno = ok ? 0 : errno;
    ::posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
    if (!output.borrowed && ::close(fd) != 0 && ok) {
        ok = false;
        writeErrno = errno;
    }
    output.fd = -1;
    output.complete = ok;
    if (!ok && !output.borrowed)
        ::unlink(path.c_str());

    recycleReusableBytes(mainIfdScratch_, std::move(ifdBytes));

    if (lossless) {
        const uint64_t totalWallUs = elapsedMicroseconds(writeStarted);
        const uint64_t accountedWallUs = lastPerformanceStats_.losslessEncodeWallUs
            + lastPerformanceStats_.losslessPayloadWriteWallUs;
        lastPerformanceStats_.losslessContainerWallUs = totalWallUs > accountedWallUs
            ? totalWallUs - accountedWallUs
            : 0;
    }

    if (!ok)
        throw std::system_error(writeErrno, std::generic_category(), "Failed to write DNG: " + path);
}

CdngEncoder::EncodedRawLayout CdngEncoder::preparePackedCsi2Layout(const RawFrameView &frame) const
{
    if (frame.bitDepth != RawBitDepth::Bits10 && frame.bitDepth != RawBitDepth::Bits12)
        throw std::runtime_error("CSI2 packed conversion supports only 10-bit and 12-bit raw.");

    EncodedRawLayout layout;
    layout.bitDepth = frame.bitDepth;
    layout.packing = PixelPacking::Csi2Packed;
    layout.rowBytes = packedRowBytes(frame.width, frame.bitDepth);
    layout.byteCount = static_cast<uint64_t>(layout.rowBytes) * frame.height;
    return layout;
}

CdngEncoder::EncodedRawLayout CdngEncoder::preparePispCompressedLayout(const RawFrameView &frame) const
{
    if (frame.bitDepth != RawBitDepth::Bits8 &&
        frame.bitDepth != RawBitDepth::Bits10 &&
        frame.bitDepth != RawBitDepth::Bits12 &&
        frame.bitDepth != RawBitDepth::Bits16) {
        throw std::runtime_error("PiSP COMP1 conversion supports only 8-bit, 10-bit, and 12-bit storage.");
    }

    EncodedRawLayout layout;
    // PiSP COMP1 expands through a 16-bit working domain. The frame view carries
    // the selected storage depth so the saved DNG matches the camera bit-depth mode.
    if (frame.bitDepth == RawBitDepth::Bits8)
        layout.bitDepth = RawBitDepth::Bits8;
    else if (frame.bitDepth == RawBitDepth::Bits10)
        layout.bitDepth = RawBitDepth::Bits10;
    else
        layout.bitDepth = RawBitDepth::Bits12;
    layout.packing = PixelPacking::Csi2Packed;
    layout.rowBytes = packedRowBytes(frame.width, layout.bitDepth);
    layout.byteCount = static_cast<uint64_t>(layout.rowBytes) * frame.height;
    return layout;
}

CdngEncoder::EncodedRawLayout CdngEncoder::prepareUnpackedLayout(const RawFrameView &frame) const
{
    EncodedRawLayout layout;
    layout.bitDepth = frame.bitDepth;
    layout.packing = frame.packing;
    layout.rowBytes = frame.bitDepth == RawBitDepth::Bits16
        ? frame.width * 2u
        : packedRowBytes(frame.width, frame.bitDepth);
    layout.byteCount = static_cast<uint64_t>(layout.rowBytes) * frame.height;
    return layout;
}

bool CdngEncoder::writeRawPayload(int fd, const RawFrameView &frame, const EncodedRawLayout &layout) const
{
    if (layout.rowBytes == 0 || layout.byteCount == 0) {
        errno = EINVAL;
        return false;
    }

    const size_t targetChunkSize = std::max<size_t>(
        static_cast<size_t>(layout.rowBytes),
        std::min<size_t>(4u * 1024u * 1024u,
                         std::max<size_t>(256u * 1024u,
                                          static_cast<size_t>(layout.rowBytes) * 64u)));
    std::vector<uint8_t> scratch = takePayloadScratch(targetChunkSize);
    size_t bufferedBytes = 0;

    auto flushScratch = [&]() {
        if (bufferedBytes == 0)
            return true;

        const bool ok = writeAll(fd, scratch.data(), bufferedBytes);
        if (ok)
            bufferedBytes = 0;
        return ok;
    };

    try {
        for (uint32_t y = 0; y < frame.height; ++y) {
            if (bufferedBytes + layout.rowBytes > scratch.size()) {
                if (!flushScratch()) {
                    recyclePayloadScratch(std::move(scratch));
                    return false;
                }
            }

            const uint8_t *src = frame.data + static_cast<size_t>(y) * frame.stride;
            uint8_t *dst = scratch.data() + bufferedBytes;
            writeDngRowBytes(frame, layout, src, dst);

            bufferedBytes += layout.rowBytes;
        }
    } catch (...) {
        recyclePayloadScratch(std::move(scratch));
        throw;
    }

    const bool ok = flushScratch();
    recyclePayloadScratch(std::move(scratch));
    return ok;
}

} // namespace apertar
