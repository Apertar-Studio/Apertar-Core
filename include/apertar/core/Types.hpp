#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

namespace apertar {

enum class CameraState {
    Offline,
    Starting,
    Ready,
    Recording,
    Error,
    Stopping
};

enum class BayerPattern {
    RGGB,
    GRBG,
    GBRG,
    BGGR
};

enum class RawBitDepth : unsigned int {
    Bits10 = 10,
    Bits12 = 12,
    Bits16 = 16
};

enum class PixelPacking {
    Unpacked,
    Csi2Packed,
    PispCompressed1
};

struct Resolution {
    uint32_t width = 3856;
    uint32_t height = 2180;

    std::string toString() const
    {
        return std::to_string(width) + "x" + std::to_string(height);
    }
};

struct CaptureSettings {
    Resolution resolution{};
    double fps = 24.0;
    uint32_t iso = 800;
    bool isoAuto = false;
    uint32_t shutterUs = 20833;
    bool shutterAuto = false;
    uint32_t whiteBalanceKelvin = 5600;
};

struct RawFrameView {
    const uint8_t *data = nullptr;
    size_t size = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    RawBitDepth bitDepth = RawBitDepth::Bits12;
    PixelPacking packing = PixelPacking::Csi2Packed;
    BayerPattern bayerPattern = BayerPattern::RGGB;
    int64_t timestampUs = 0;
};

struct PreviewFrameDescriptor {
    int fd = -1;
    int planeCount = 0;
    int planeFds[3] = { -1, -1, -1 };
    uint32_t planeOffsets[3] = { 0, 0, 0 };
    uint32_t planePitches[3] = { 0, 0, 0 };
    uint32_t planeLengths[3] = { 0, 0, 0 };
    uint64_t frameId = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    uint32_t captureWidth = 0;
    uint32_t captureHeight = 0;
    size_t size = 0;
    std::string fourcc = "YUV420";
    int64_t timestampUs = 0;
};

inline const char *toString(CameraState state)
{
    switch (state) {
    case CameraState::Offline: return "offline";
    case CameraState::Starting: return "starting";
    case CameraState::Ready: return "ready";
    case CameraState::Recording: return "recording";
    case CameraState::Error: return "error";
    case CameraState::Stopping: return "stopping";
    }
    return "unknown";
}

} // namespace apertar
