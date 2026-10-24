#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <span>

namespace apertar {

// One encoder per recording worker. Storage is reused between small strips.
class LosslessJpegEncoder {
public:
    std::span<const uint8_t> encode(const uint16_t *samples, uint32_t rawWidth,
                                    uint32_t rows, unsigned bits);
    void begin(uint32_t rawWidth, uint32_t rows, unsigned bits, bool compactCodes = false,
               bool monochrome = false);
    void appendRow(const uint16_t *samples);
    std::span<const uint8_t> finish();

private:
    void appendByte(unsigned value);
    void appendWord(unsigned value);
    void emitBits(uint64_t value, unsigned length);
    void emitDifference(int difference);
    void emitDifferencePair(int first, int second);
    void emitDifferenceQuad(int first, int second, int third, int fourth);

    std::unique_ptr<uint8_t[]> output_;
    size_t outputCapacity_ = 0;
    size_t outputSize_ = 0;
    std::array<uint32_t, 17> amplitudeMasks_{};
    std::array<uint32_t, 17> encodedPrefixes_{};
    std::array<unsigned, 17> encodedLengths_{};
    std::array<uint16_t, 2> previousFirst_{};
    uint64_t accumulator_ = 0;
    unsigned pendingBits_ = 0;
    uint32_t rawWidth_ = 0;
    uint32_t rows_ = 0;
    uint32_t nextRow_ = 0;
    unsigned bits_ = 0;
    unsigned components_ = 2;
    bool active_ = false;
    const uint32_t *differenceLut_ = nullptr;
};

} // namespace apertar
