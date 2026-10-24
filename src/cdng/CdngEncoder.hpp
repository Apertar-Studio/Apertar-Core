#pragma once

#include <apertar/core/Types.hpp>
#include "LosslessJpegEncoder.hpp"

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
        CdngCompression compression = CdngCompression::Uncompressed;
        // Optional inverse companding curve for quantized Bayer RAW.
        std::span<const uint16_t> linearizationTable{};
        // Optional encoding-only mapping from MSB16 PiSP samples to JPEG codes.
        std::span<const uint16_t> jpegSampleLookup{};
        unsigned jpegSampleShift = 0;
        bool compactJpegCodes = false;
        uint64_t frameIndex = 0;
        int64_t sensorTimestampUs = 0;
        uint64_t wallClockTimestampUs = 0;
        TimecodeMode timecodeMode = TimecodeMode::FreeRun;
        double fps = 24.0;
        uint32_t iso = 800;
        uint32_t shutterUs = 20833;
        uint32_t whiteBalanceKelvin = 5600;
        float analogueGain = 1.0f;
        float digitalGain = 1.0f;
        float colourGains[2] = { 1.0f, 1.0f };
        float asShotNeutralGains[2] = { 0.0f, 0.0f };
        std::string sensorModel;
        bool hasBlackLevels = false;
        // R, Gr, Gb, B on a 12-bit reference scale unless explicitly converted.
        float blackLevels[4] = { 206.0f, 206.0f, 206.0f, 206.0f };
        bool blackLevelsOnContainerScale = false;
        bool hasColourMatrix = false;
        float colourMatrix[9] = {
            1.90255f, -0.77478f, -0.12777f,
           -0.31338f,  1.88197f, -0.56858f,
           -0.06001f, -0.61785f,  1.67786f,
        };
        bool hasColourMatrix2 = false;
        float colourMatrix2[9] = {
            1.90255f, -0.77478f, -0.12777f,
           -0.31338f,  1.88197f, -0.56858f,
           -0.06001f, -0.61785f,  1.67786f,
        };
        bool hasForwardMatrix1 = false;
        float forwardMatrix1[9] = {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f,
        };
        bool hasForwardMatrix2 = false;
        float forwardMatrix2[9] = {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f,
        };
        uint16_t calibrationIlluminant1 = 17;
        uint16_t calibrationIlluminant2 = 21;
        std::string make = "Apertar";
        std::string model = "Apertar D1";
        std::string uniqueCameraModel = "Blackmagic Micro Cinema Camera";
        std::string software = "ApertarCore";
        std::string serial;
        std::string clipName;
        std::string frameFileName;
        std::string reelName;
        std::string cameraLabel;
        int rotationDegrees = 0;
    };

    struct EncodedRawLayout {
        RawBitDepth bitDepth = RawBitDepth::Bits12;
        PixelPacking packing = PixelPacking::Csi2Packed;
        uint32_t rowBytes = 0;
        uint64_t byteCount = 0;
    };

    struct PerformanceStats {
        uint64_t losslessEncodeWallUs = 0;
        uint64_t losslessDecodeWallUs = 0;
        uint64_t losslessEntropyWallUs = 0;
        uint64_t losslessPayloadWriteWallUs = 0;
        uint64_t losslessContainerWallUs = 0;
        uint64_t losslessPayloadBytes = 0;
        uint32_t losslessProfiledSourceRows = 0;
        uint32_t losslessTileWrites = 0;
    };

    EncodedRawLayout prepareRawLayout(const RawFrameView &frame) const;
    bool writeRawPayload(int fd, const RawFrameView &frame, const EncodedRawLayout &layout) const;
    void writeDngFile(const std::string &path, const RawFrameView &frame, const Metadata &metadata,
                      int destinationFd = -1) const;
    const PerformanceStats &lastPerformanceStats() const { return lastPerformanceStats_; }
    static uint32_t losslessTileRows();
    static void decodePispRowToMsb16(const uint8_t *src, uint16_t *dst, uint32_t width);

private:
    struct MainIfdLayoutCache {
        bool valid = false;
        uint32_t imageWidth = 0;
        uint32_t imageHeight = 0;
        uint16_t bits = 0;
        uint16_t compression = 1;
        uint32_t stripByteCount = 0;
        bool hasClipName = false;
        bool hasFrameFileName = false;
        bool hasMake = false;
        bool hasModel = false;
        bool hasUniqueCameraModel = false;
        bool hasSerial = false;
        bool hasReelName = false;
        bool hasCameraLabel = false;
        uint32_t clipNameLength = 0;
        uint32_t frameFileNameLength = 0;
        uint32_t makeLength = 0;
        uint32_t modelLength = 0;
        uint32_t uniqueCameraModelLength = 0;
        uint32_t serialLength = 0;
        uint32_t reelNameLength = 0;
        uint32_t cameraLabelLength = 0;
        uint32_t softwareLength = 0;
        size_t mainIfdSize = 0;
    };

    struct DynamicMetadataCache {
        bool valid = false;
        double fps = 0.0;
        uint32_t iso = 0;
        uint32_t shutterUs = 0;
        float colourGain0 = 0.0f;
        float colourGain1 = 0.0f;
        int32_t fpsRat[2] = { 24000, 1000 };
        uint32_t exifExposure[2] = { 20833, 1000000u };
        uint16_t exifIso = 800;
        uint32_t exifIsoLong = 800;
        uint32_t exifExposureIndex[2] = { 800u, 1u };
        int32_t neutral[6] = { 1, 1, 1, 1, 1, 1 };
    };

    struct IfdTemplateCache {
        bool valid = false;
        BayerPattern bayerPattern = BayerPattern::RGGB;
        std::string colorProfileKey;
        MainIfdLayoutCache layout{};
        std::string clipName;
        std::string make;
        std::string model;
        std::string uniqueCameraModel;
        std::string serial;
        std::string reelName;
        std::string cameraLabel;
        std::string software;
        std::vector<uint8_t> bytes;
        uint32_t pageNameOffset = 0;
        uint32_t pageNameFieldSize = 0;
        uint32_t modifyDateOffset = 0;
        uint32_t dateTimeOriginalOffset = 0;
        uint32_t neutralOffset = 0;
        uint32_t blackLevelsOffset = 0;
        uint32_t timeCodeOffset = 0;
        uint32_t frameRateOffset = 0;
        uint32_t exposureTimeOffset = 0;
        uint32_t exifIsoShortOffset = 0;
        uint32_t exifIsoLongOffset1 = 0;
        uint32_t exifIsoLongOffset2 = 0;
        uint32_t exifIsoLongOffset3 = 0;
        uint32_t exposureIndexOffset = 0;
    };

    std::vector<uint8_t> takePayloadScratch(size_t size) const;
    void recyclePayloadScratch(std::vector<uint8_t> bytes) const;

    EncodedRawLayout preparePackedCsi2Layout(const RawFrameView &frame) const;
    EncodedRawLayout preparePispCompressedLayout(const RawFrameView &frame) const;
    EncodedRawLayout prepareUnpackedLayout(const RawFrameView &frame) const;
    mutable std::vector<uint8_t> payloadScratch_;
    mutable std::vector<uint16_t> jpegSamples_;
    mutable LosslessJpegEncoder jpegEncoder_;
    mutable std::vector<uint8_t> mainIfdScratch_;
    mutable std::vector<uint8_t> exifScratch_;
    mutable MainIfdLayoutCache mainIfdLayoutCache_;
    mutable DynamicMetadataCache dynamicMetadataCache_;
    mutable IfdTemplateCache ifdTemplateCache_;
    mutable PerformanceStats lastPerformanceStats_{};
};

} // namespace apertar
