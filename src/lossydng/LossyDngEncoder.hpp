#pragma once

#include "cdng/CdngEncoder.hpp"
#include <apertar/core/Types.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace apertar {

// Lossy quantization in a companded Bayer domain, followed by standard SOF3
// coding. Each file carries its inverse curve in the DNG LinearizationTable.
class LossyDngEncoder {
public:
    explicit LossyDngEncoder(LossyDngRatio ratio = LossyDngRatio::R3);
    ~LossyDngEncoder();
    LossyDngEncoder(const LossyDngEncoder &) = delete;
    LossyDngEncoder &operator=(const LossyDngEncoder &) = delete;

    void setRatio(LossyDngRatio ratio);
    void invalidateTables();
    void writeDngFile(const std::string &path, const RawFrameView &frame,
                      const CdngEncoder::Metadata &metadata);
    static void drainWrites();

private:
    void ensureBuf(size_t need, size_t used);

    LossyDngRatio ratio_;
    int scale_ = 0;
    CdngEncoder standardEncoder_;
    int memoryFd_ = -1;
    int cachedBlack_ = -1;
    int cachedLutBits_ = 0;
    bool cachedGradation_ = false;
    std::vector<uint16_t> rowScratch_;
    std::vector<uint16_t> unpacked_;
    std::vector<uint8_t> unpacked8_;
    std::vector<uint16_t> fwd_;
    std::vector<uint16_t> linTable_;
    std::vector<uint16_t> inverseQuantized_;
    std::vector<uint16_t> quantizedForward_;
    unsigned cachedQuantStep_ = 0;
    unsigned cachedOutputBits_ = 0;

    // The async writer owns submitted buffers; the next frame obtains another
    // pooled buffer instead of blocking an encoder worker on storage latency.
    uint8_t *buf_ = nullptr;
    size_t bufCap_ = 0;
    size_t lastBytes_ = 0;
    bool asyncWrite_ = true;
};

} // namespace apertar
