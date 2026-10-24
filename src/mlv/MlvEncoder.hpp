#pragma once

#include "cdng/CdngEncoder.hpp"

#include <apertar/core/Types.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace apertar {

class MlvEncoder {
public:
    ~MlvEncoder();

    bool open(const std::string &path,
              const CaptureSettings &settings,
              const RawFrameView &firstFrame,
              const CdngEncoder::Metadata &metadata,
              std::string *error);
    bool writeFrame(const RawFrameView &frame,
                    const CdngEncoder::Metadata &metadata,
                    std::string *error);
    void close();

    bool isOpen() const { return fd_ >= 0; }
    uint32_t frameCount() const { return frameCount_; }

private:
    struct RawLayout {
        RawBitDepth bitDepth = RawBitDepth::Bits12;
        PixelPacking sourcePacking = PixelPacking::Csi2Packed;
        uint32_t rowBytes = 0;
        uint64_t byteCount = 0;
    };

    void writeFileHeader(uint32_t frameCount);
    void writeVersionBlock();
    void writeClockBlocks(const CdngEncoder::Metadata &metadata);
    void writeIdentityBlock(const CdngEncoder::Metadata &metadata);
    void writeRawInfoBlock(const RawFrameView &frame,
                           const CdngEncoder::Metadata &metadata);
    void writeExposureBlock(const CaptureSettings &settings,
                            const CdngEncoder::Metadata &metadata);
    void writeWhiteBalanceBlock(const CdngEncoder::Metadata &metadata, uint64_t timestampUs = 0);
    void writePreviewColorBlock(const CdngEncoder::Metadata &metadata, uint64_t timestampUs = 0);
    void patchFrameCount();

    uint64_t relativeTimestampUs(const RawFrameView &frame,
                                 const CdngEncoder::Metadata &metadata) const;

    RawLayout prepareRawLayout(const RawFrameView &frame) const;
    bool writeRawPayload(const RawFrameView &frame);

    RawLayout rawLayout_{};
    CaptureSettings settings_{};
    std::string path_;
    std::vector<uint8_t> payloadScratch_;
    std::vector<uint16_t> rowScratch_;
    int fd_ = -1;
    uint64_t fileGuid_ = 0;
    int64_t firstTimestampUs_ = 0;
    uint32_t frameCount_ = 0;
    uint32_t lastWbKelvin_ = 0;
    uint32_t lastWbRed_ = 0;
    uint32_t lastWbBlue_ = 0;
    std::array<int32_t, 9> lastPreviewCcm_{};
    bool previewColorWritten_ = false;
};

} // namespace apertar
