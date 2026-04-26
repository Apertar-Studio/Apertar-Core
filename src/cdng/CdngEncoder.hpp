#pragma once

#include <apertar/core/Types.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace apertar {

class Csi2RawConverter {
public:
    static void unpack10Row(const uint8_t *src, uint16_t *dst, uint32_t width);
    static void unpack12Row(const uint8_t *src, uint16_t *dst, uint32_t width);

    static void packDng10Row(const uint16_t *src, uint8_t *dst, uint32_t width);
    static void packDng12Row(const uint16_t *src, uint8_t *dst, uint32_t width);
};

class CdngEncoder {
public:
    struct Metadata {
        uint64_t frameIndex = 0;
        int64_t sensorTimestampUs = 0;
        uint64_t wallClockTimestampUs = 0;
        double fps = 24.0;
        uint32_t iso = 800;
        uint32_t shutterUs = 20833;
        uint32_t whiteBalanceKelvin = 5600;
        float analogueGain = 1.0f;
        float digitalGain = 1.0f;
        float colourGains[2] = { 1.0f, 1.0f };
        bool hasBlackLevels = false;
        float blackLevels[4] = { 206.0f, 206.0f, 206.0f, 206.0f };
        bool hasColourMatrix = false;
        float colourMatrix[9] = {
            1.90255f, -0.77478f, -0.12777f,
           -0.31338f,  1.88197f, -0.56858f,
           -0.06001f, -0.61785f,  1.67786f,
        };
        std::string make = "Raspberry Pi";
        std::string model = "IMX585-AAQJ1";
        std::string uniqueCameraModel = "D1";
        std::string software = "ApertarCore";
        std::string serial;
    };

    struct EncodedRawPayload {
        std::vector<uint8_t> bytes;
        RawBitDepth bitDepth = RawBitDepth::Bits12;
        PixelPacking packing = PixelPacking::Csi2Packed;
        uint32_t rowBytes = 0;
    };

    EncodedRawPayload prepareRawPayload(const RawFrameView &frame) const;
    void writeDngFile(const std::string &path, const RawFrameView &frame, const Metadata &metadata) const;

private:
    std::vector<uint8_t> takePayloadScratch(size_t size) const;
    void recyclePayloadScratch(std::vector<uint8_t> bytes) const;

    EncodedRawPayload preparePackedCsi2Payload(const RawFrameView &frame) const;
    EncodedRawPayload preparePispCompressedPayload(const RawFrameView &frame) const;
    EncodedRawPayload prepareUnpackedPayload(const RawFrameView &frame) const;

    mutable std::vector<uint8_t> payloadScratch_;
};

} // namespace apertar
