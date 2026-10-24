#include "LosslessJpegEncoder.hpp"

#include <array>
#include <cstddef>
#include <cstring>
#include <stdexcept>

namespace apertar {
namespace {

unsigned category(int value)
{
    const unsigned magnitude = static_cast<unsigned>(value < 0 ? -value : value);
    return magnitude ? 32u - static_cast<unsigned>(__builtin_clz(magnitude)) : 0u;
}

struct Huffman {
    std::array<unsigned, 17> counts{};
    std::array<unsigned, 17> codes{};
    std::array<unsigned, 17> lengths{};
    std::array<uint8_t, 17> symbols{};

    explicit Huffman(bool compact = false)
    {
        // A fixed lossless-DC table removes the full-tile histogram pass and
        // lets prediction feed the entropy writer directly. Categories 0-9
        // use the conventional JPEG DC progression; the sparse long tail
        // covers 10-16 without consuming the all-ones padding code.
        counts[2] = 1;
        counts[3] = 5;
        counts[4] = 1;
        counts[5] = 1;
        counts[6] = 1;
        counts[7] = 1;
        for (unsigned length = 9; length <= 15; ++length)
            counts[length] = 1;
        for (unsigned symbol = 0; symbol < symbols.size(); ++symbol)
            symbols[symbol] = static_cast<uint8_t>(symbol);

        if (compact) {
            // Quantized samples mostly predict to zero. Keep a complete
            // 12-bit category range without spending three bits on zero.
            counts.fill(0);
            for (unsigned length = 1; length <= 13; ++length)
                counts[length] = 1;
        }

        size_t index = 0;
        unsigned code = 0;
        for (unsigned length = 1; length <= 16; ++length) {
            for (unsigned i = 0; i < counts[length]; ++i) {
                const unsigned symbol = symbols[index++];
                codes[symbol] = code++;
                lengths[symbol] = length;
            }
            code <<= 1;
        }
    }
};

const Huffman &fixedHuffman()
{
    static const Huffman table;
    return table;
}

constexpr int kRaw12DifferenceLimit = 4095;
constexpr uint32_t kEncodedDifferenceBitsMask = (1u << 27) - 1u;

std::array<uint32_t, kRaw12DifferenceLimit * 2 + 1> buildRaw12DifferenceLut(bool compact = false)
{
    static const Huffman compactHuffman(true);
    const Huffman &huffman = compact ? compactHuffman : fixedHuffman();
    std::array<uint32_t, kRaw12DifferenceLimit * 2 + 1> table{};
    for (int difference = -kRaw12DifferenceLimit; difference <= kRaw12DifferenceLimit; ++difference) {
        const unsigned c = category(difference);
        const unsigned amplitudeBits = c;
        const uint32_t amplitudeMask = (1u << amplitudeBits) - 1u;
        const uint32_t amplitude = static_cast<uint32_t>(difference < 0 ? difference - 1 : difference)
            & amplitudeMask;
        const unsigned length = huffman.lengths[c] + amplitudeBits;
        const uint32_t encoded = (huffman.codes[c] << amplitudeBits) | amplitude;
        table[static_cast<size_t>(difference + kRaw12DifferenceLimit)] =
            (static_cast<uint32_t>(length) << 27) | encoded;
    }
    return table;
}

const std::array<uint32_t, kRaw12DifferenceLimit * 2 + 1> kRaw12DifferenceLut =
    buildRaw12DifferenceLut();

const auto kCompactDifferenceLut = buildRaw12DifferenceLut(true);

} // namespace

std::span<const uint8_t> LosslessJpegEncoder::encode(
    const uint16_t *samples, uint32_t rawWidth, uint32_t rows, unsigned bits)
{
    if (!samples)
        throw std::runtime_error("Lossless JPEG has no source samples.");

    begin(rawWidth, rows, bits);
    for (uint32_t y = 0; y < rows; ++y)
        appendRow(samples + static_cast<size_t>(y) * rawWidth);
    return finish();
}

void LosslessJpegEncoder::begin(uint32_t rawWidth, uint32_t rows, unsigned bits, bool compactCodes,
                                bool monochrome)
{
    const unsigned components = monochrome ? 1u : 2u;
    if (active_ || rawWidth < components || rawWidth % components || rawWidth / components > 65535 ||
        !rows || rows > 65535 || (bits != 8 && bits != 10 && bits != 12 && bits != 16))
        throw std::runtime_error("Invalid lossless JPEG strip geometry or precision.");

    if (compactCodes && bits > 12)
        throw std::runtime_error("Compact JPEG codes require at most 12-bit samples.");

    const size_t count = static_cast<size_t>(rawWidth) * rows;
    static const Huffman compactHuffman(true);
    const Huffman &huffman = compactCodes ? compactHuffman : fixedHuffman();
    differenceLut_ = compactCodes ? kCompactDifferenceLut.data() : kRaw12DifferenceLut.data();
    // Worst entropy length is 23 bits/sample through 12-bit and 29 bits at
    // 16-bit. Doubling for worst-case JPEG byte stuffing still fits these
    // bounds. Allocate without value-initializing bytes that will be replaced.
    const size_t requiredCapacity = count * (bits <= 12 ? 6u : 8u) + 256u;
    if (outputCapacity_ < requiredCapacity) {
        output_.reset(new uint8_t[requiredCapacity]);
        outputCapacity_ = requiredCapacity;
    }
    outputSize_ = 0;
    for (unsigned c = 0; c < 17; ++c) {
        const unsigned amplitudeBits = c == 16 ? 0 : c;
        amplitudeMasks_[c] = (1u << amplitudeBits) - 1u;
        encodedPrefixes_[c] = huffman.codes[c] << amplitudeBits;
        encodedLengths_[c] = huffman.lengths[c] + amplitudeBits;
    }
    appendWord(0xffd8); // SOI
    appendWord(0xffc3); // SOF3, lossless Huffman coding
    appendWord(8 + 3 * components);
    appendByte(bits);
    appendWord(rows);
    appendWord(rawWidth / components);
    appendByte(components);
    for (unsigned c = 0; c < components; ++c) {
        appendByte(c); appendByte(0x11); appendByte(0);
    }
    appendWord(0xffc4);
    const unsigned symbolCount = compactCodes ? 13 : 17;
    appendWord(19 + symbolCount);
    appendByte(0);
    for (unsigned i = 1; i <= 16; ++i)
        appendByte(huffman.counts[i]);
    for (unsigned symbol = 0; symbol < symbolCount; ++symbol)
        appendByte(huffman.symbols[symbol]);
    appendWord(0xffda);
    appendWord(6 + 2 * components);
    appendByte(components);
    for (unsigned c = 0; c < components; ++c) {
        appendByte(c); appendByte(0);
    }
    appendByte(1); appendByte(0); appendByte(0); // predictor 1, no point transform

    accumulator_ = 0;
    pendingBits_ = 0;
    rawWidth_ = rawWidth;
    rows_ = rows;
    nextRow_ = 0;
    bits_ = bits;
    components_ = components;
    previousFirst_.fill(0);
    active_ = true;
}

void LosslessJpegEncoder::appendRow(const uint16_t *row)
{
    if (!active_ || !row || nextRow_ >= rows_)
        throw std::runtime_error("Unexpected lossless JPEG row.");
#ifndef NDEBUG
    const unsigned mask = (1u << bits_) - 1u;
#endif
    if (components_ == 1) {
        // Mono predicts from the neighbouring pixel, not the same CFA colour.
#ifndef NDEBUG
        for (uint32_t x = 0; x < rawWidth_; ++x)
            if (row[x] > mask)
                throw std::runtime_error("RAW sample exceeds lossless JPEG precision.");
#endif
        const int prediction = nextRow_ ? previousFirst_[0] : (1 << (bits_ - 1));
        emitDifference(static_cast<int>(row[0]) - prediction);
        previousFirst_[0] = row[0];
        uint32_t x = 1;
        if (bits_ <= 12) {
            for (; x + 3 < rawWidth_; x += 4)
                emitDifferenceQuad(static_cast<int>(row[x]) - row[x - 1],
                                   static_cast<int>(row[x + 1]) - row[x],
                                   static_cast<int>(row[x + 2]) - row[x + 1],
                                   static_cast<int>(row[x + 3]) - row[x + 2]);
        }
        for (; x < rawWidth_; ++x)
            emitDifference(static_cast<int>(row[x]) - row[x - 1]);
        ++nextRow_;
        return;
    }
    int firstDifferences[2];
    for (unsigned c = 0; c < 2; ++c) {
#ifndef NDEBUG
        if (row[c] > mask)
            throw std::runtime_error("RAW sample exceeds lossless JPEG precision.");
#endif
        const int prediction = nextRow_ ? previousFirst_[c] : (1 << (bits_ - 1));
        firstDifferences[c] = static_cast<int>(row[c]) - prediction;
        previousFirst_[c] = row[c];
    }
    uint32_t x = 2;
    if (bits_ <= 12) {
        if (rawWidth_ >= 4) {
#ifndef NDEBUG
            if (row[2] > mask || row[3] > mask)
                throw std::runtime_error("RAW sample exceeds lossless JPEG precision.");
#endif
            emitDifferenceQuad(firstDifferences[0], firstDifferences[1],
                               static_cast<int>(row[2]) - row[0],
                               static_cast<int>(row[3]) - row[1]);
            x = 4;
        } else {
            emitDifferencePair(firstDifferences[0], firstDifferences[1]);
        }
    } else {
        emitDifference(firstDifferences[0]);
        emitDifference(firstDifferences[1]);
    }
    if (bits_ <= 12) {
        for (; x + 3 < rawWidth_; x += 4) {
#ifndef NDEBUG
            if (row[x] > mask || row[x + 1] > mask ||
                row[x + 2] > mask || row[x + 3] > mask) {
                throw std::runtime_error("RAW sample exceeds lossless JPEG precision.");
            }
#endif
            emitDifferenceQuad(static_cast<int>(row[x]) - row[x - 2],
                               static_cast<int>(row[x + 1]) - row[x - 1],
                               static_cast<int>(row[x + 2]) - row[x],
                               static_cast<int>(row[x + 3]) - row[x + 1]);
        }
        if (x < rawWidth_) {
#ifndef NDEBUG
            if (row[x] > mask || row[x + 1] > mask)
                throw std::runtime_error("RAW sample exceeds lossless JPEG precision.");
#endif
            emitDifferencePair(static_cast<int>(row[x]) - row[x - 2],
                               static_cast<int>(row[x + 1]) - row[x - 1]);
        }
    } else {
        for (; x < rawWidth_; x += 2) {
#ifndef NDEBUG
            if (row[x] > mask || row[x + 1] > mask)
                throw std::runtime_error("RAW sample exceeds lossless JPEG precision.");
#endif
            emitDifference(static_cast<int>(row[x]) - row[x - 2]);
            emitDifference(static_cast<int>(row[x + 1]) - row[x - 1]);
        }
    }
    ++nextRow_;
}

std::span<const uint8_t> LosslessJpegEncoder::finish()
{
    if (!active_ || nextRow_ != rows_)
        throw std::runtime_error("Incomplete lossless JPEG tile.");
    if (pendingBits_)
        emitBits((1u << (8 - pendingBits_)) - 1u, 8 - pendingBits_);
    appendWord(0xffd9);
    active_ = false;
    return { output_.get(), outputSize_ };
}

void LosslessJpegEncoder::appendByte(unsigned value)
{
#ifndef NDEBUG
    if (outputSize_ >= outputCapacity_)
        throw std::runtime_error("Lossless JPEG output exceeded its bound.");
#endif
    output_[outputSize_++] = static_cast<uint8_t>(value);
}

void LosslessJpegEncoder::appendWord(unsigned value)
{
    appendByte(value >> 8);
    appendByte(value);
}

void LosslessJpegEncoder::emitBits(uint64_t value, unsigned length)
{
    accumulator_ = (accumulator_ << length) | value;
    pendingBits_ += length;

    const unsigned byteCount = pendingBits_ / 8;
    if (!byteCount)
        return;

    const unsigned remainingBits = pendingBits_ % 8;
    const uint64_t bytes = accumulator_ >> remainingBits;
    const uint64_t relevantMask = (uint64_t{1} << (byteCount * 8)) - 1;
    const uint64_t ffMatches = (bytes ^ ~uint64_t{0}) | ~relevantMask;
    constexpr uint64_t lowBits = 0x0101010101010101ULL;
    constexpr uint64_t highBits = 0x8080808080808080ULL;

    // Most groups contain no 0xff byte. Store the complete big-endian group
    // at once and only take the byte-stuffing path when it is required.
    if (((ffMatches - lowBits) & ~ffMatches & highBits) == 0) {
#ifndef NDEBUG
        if (outputSize_ + byteCount > outputCapacity_)
            throw std::runtime_error("Lossless JPEG output exceeded its bound.");
#endif
        const uint64_t packed = __builtin_bswap64(bytes << ((8 - byteCount) * 8));
        std::memcpy(output_.get() + outputSize_, &packed, byteCount);
        outputSize_ += byteCount;
    } else {
        for (unsigned index = byteCount; index > 0; --index) {
            const uint8_t byte = static_cast<uint8_t>(bytes >> ((index - 1) * 8));
            appendByte(byte);
            if (byte == 0xff)
                appendByte(0); // JPEG entropy byte stuffing
        }
    }
    pendingBits_ = remainingBits;
}

void LosslessJpegEncoder::emitDifference(int difference)
{
    const int d = static_cast<int16_t>(difference);
    if (bits_ <= 12) {
#ifndef NDEBUG
        if (d < -kRaw12DifferenceLimit || d > kRaw12DifferenceLimit)
            throw std::runtime_error("RAW predictor difference exceeds the lookup range.");
#endif
        const uint32_t packed = differenceLut_[static_cast<size_t>(d + kRaw12DifferenceLimit)];
        emitBits(packed & kEncodedDifferenceBitsMask, packed >> 27);
        return;
    }

    const unsigned c = category(d);
    const uint32_t amplitude = static_cast<uint32_t>(d < 0 ? d - 1 : d) & amplitudeMasks_[c];
    emitBits(encodedPrefixes_[c] | amplitude, encodedLengths_[c]);
}

void LosslessJpegEncoder::emitDifferencePair(int first, int second)
{
    const int d0 = static_cast<int16_t>(first);
    const int d1 = static_cast<int16_t>(second);
#ifndef NDEBUG
    if (d0 < -kRaw12DifferenceLimit || d0 > kRaw12DifferenceLimit ||
        d1 < -kRaw12DifferenceLimit || d1 > kRaw12DifferenceLimit) {
        throw std::runtime_error("RAW predictor difference exceeds the lookup range.");
    }
#endif
    const uint32_t firstPacked =
        differenceLut_[static_cast<size_t>(d0 + kRaw12DifferenceLimit)];
    const uint32_t secondPacked =
        differenceLut_[static_cast<size_t>(d1 + kRaw12DifferenceLimit)];
    const unsigned firstLength = firstPacked >> 27;
    const unsigned secondLength = secondPacked >> 27;
    const uint64_t combined =
        (static_cast<uint64_t>(firstPacked & kEncodedDifferenceBitsMask) << secondLength) |
        (secondPacked & kEncodedDifferenceBitsMask);
    emitBits(combined, firstLength + secondLength);
}

void LosslessJpegEncoder::emitDifferenceQuad(int first, int second, int third, int fourth)
{
    const int d0 = static_cast<int16_t>(first);
    const int d1 = static_cast<int16_t>(second);
    const int d2 = static_cast<int16_t>(third);
    const int d3 = static_cast<int16_t>(fourth);
#ifndef NDEBUG
    if (d0 < -kRaw12DifferenceLimit || d0 > kRaw12DifferenceLimit ||
        d1 < -kRaw12DifferenceLimit || d1 > kRaw12DifferenceLimit ||
        d2 < -kRaw12DifferenceLimit || d2 > kRaw12DifferenceLimit ||
        d3 < -kRaw12DifferenceLimit || d3 > kRaw12DifferenceLimit) {
        throw std::runtime_error("RAW predictor difference exceeds the lookup range.");
    }
#endif
    const uint32_t p0 = differenceLut_[static_cast<size_t>(d0 + kRaw12DifferenceLimit)];
    const uint32_t p1 = differenceLut_[static_cast<size_t>(d1 + kRaw12DifferenceLimit)];
    const uint32_t p2 = differenceLut_[static_cast<size_t>(d2 + kRaw12DifferenceLimit)];
    const uint32_t p3 = differenceLut_[static_cast<size_t>(d3 + kRaw12DifferenceLimit)];
    const unsigned l0 = p0 >> 27;
    const unsigned l1 = p1 >> 27;
    const unsigned l2 = p2 >> 27;
    const unsigned l3 = p3 >> 27;
    const unsigned totalLength = l0 + l1 + l2 + l3;

    // Keep the combined code within the 64-bit accumulator, including the
    // at-most seven bits carried from the preceding call.
    if (totalLength <= 56) {
        const uint64_t combined =
            (static_cast<uint64_t>(p0 & kEncodedDifferenceBitsMask) << (l1 + l2 + l3)) |
            (static_cast<uint64_t>(p1 & kEncodedDifferenceBitsMask) << (l2 + l3)) |
            (static_cast<uint64_t>(p2 & kEncodedDifferenceBitsMask) << l3) |
            (p3 & kEncodedDifferenceBitsMask);
        emitBits(combined, totalLength);
        return;
    }

    emitDifferencePair(d0, d1);
    emitDifferencePair(d2, d3);
}

} // namespace apertar
