#pragma once

#include <cstddef>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <memory>
#include "ProresColorProfile.hpp"

namespace apertar {
struct PreviewExposureReference;

enum class CameraState {
    Offline,
    Starting,
    Ready,
    Standby,
    Recording,
    Error,
    Stopping
};

enum class BayerPattern {
    RGGB,
    GRBG,
    GBRG,
    BGGR,
    Monochrome
};

enum class RawBitDepth : unsigned int {
    Bits8 = 8,
    Bits10 = 10,
    Bits12 = 12,
    Bits16 = 16
};

enum class PixelPacking {
    Unpacked,
    Csi2Packed,
    PispCompressed1
};

enum class RecordingFormat {
    Cdng,
    Mlv,
    H264,
    LossyDng,
    Prores
};

enum class CdngCompression {
    Uncompressed,
    LosslessJpeg
};

enum class LossyDngRatio {
    R3,
    R5
};

enum class ProresProfile {
    HQ,
    Standard,
    LT,
    Proxy
};

enum class PhotoCaptureFormat {
    Dng,
    Jpeg,
    Png,
    Tiff,
    DngJpeg,
    DngPng,
    DngTiff
};

enum class TimecodeMode {
    FreeRun,
    RecRun
};

enum class AutofocusMode {
    Manual,
    Auto,
    Continuous
};

enum class AutofocusRange {
    Normal,
    Macro,
    Full
};

enum class AutofocusSpeed {
    Normal,
    Fast
};

struct Resolution {
    uint32_t width = 3840;
    uint32_t height = 2160;

    std::string toString() const
    {
        return std::to_string(width) + "x" + std::to_string(height);
    }
};

struct WhiteBalanceMeasurement {
    uint32_t kelvin = 0;
    float red = 0.0f;
    float blue = 0.0f;

    bool valid() const
    {
        return std::isfinite(red) && std::isfinite(blue) &&
               red > 0.0f && blue > 0.0f && red <= 32.0f && blue <= 32.0f;
    }
};

struct CaptureSettings {
    Resolution resolution{};
    double fps = 24.0;
    uint32_t iso = 800;
    bool isoAuto = false;
    uint32_t shutterUs = 20833;
    bool shutterAuto = false;
    double exposureCompensation = 0.0;
    bool exposureCompensationAvailable = false; // Runtime libcamera capability.
    std::string exposureMetering = "centre-weighted";
    uint8_t exposureMeteringModes = 0; // Runtime tuning/pipeline capability mask.
    bool photoModeEnabled = false;
    bool longExposureEnabled = false;
    bool bracketingEnabled = false;
    double bracketingEv = 1.0;
    uint32_t timelapseExposureLimitUs = 0; // Runtime constraint derived from the saved UI interval.
    uint32_t whiteBalanceKelvin = 5600;
    bool whiteBalanceAuto = false;
    bool whiteBalanceAvailable = false; // Runtime capability of the active tuning, not persisted.
    WhiteBalanceMeasurement whiteBalanceMeasured;
    RecordingFormat recordingFormat = RecordingFormat::Cdng;
    CdngCompression cdngCompression = CdngCompression::Uncompressed;
    LossyDngRatio lossyDngRatio = LossyDngRatio::R3;
    ProresProfile proresProfile = ProresProfile::Standard;
    ProresColorProfile proresColorProfile = ProresColorProfile::Standard;
    ProresColorProfile cdngPreviewColorProfile = ProresColorProfile::Standard;
    std::shared_ptr<const PreviewExposureReference> previewExposureReference; // Active ISP, not saved settings.
    std::string tuningFile;
    uint32_t rawBitDepth = 12;
    Resolution h264Resolution{0, 0};
    TimecodeMode timecodeMode = TimecodeMode::FreeRun;
    AutofocusMode autofocusMode = AutofocusMode::Continuous;
    AutofocusRange autofocusRange = AutofocusRange::Normal;
    AutofocusSpeed autofocusSpeed = AutofocusSpeed::Normal;
    double autofocusWindowX = 0.25;
    double autofocusWindowY = 0.25;
    double autofocusWindowWidth = 0.5;
    double autofocusWindowHeight = 0.5;
};

inline bool usesStillBracketing(const CaptureSettings &settings)
{
    return settings.photoModeEnabled && settings.bracketingEnabled && !settings.timelapseExposureLimitUs;
}

inline ProresColorProfile activeIspColorProfile(const CaptureSettings &settings)
{
    if (settings.photoModeEnabled) return ProresColorProfile::Standard;
    if (settings.recordingFormat == RecordingFormat::Prores) return settings.proresColorProfile;
    if (settings.recordingFormat == RecordingFormat::Cdng || settings.recordingFormat == RecordingFormat::LossyDng)
        return settings.cdngPreviewColorProfile;
    return ProresColorProfile::Standard;
}

inline uint32_t timelapseExposureLimitUs(double intervalSeconds)
{
    return std::isfinite(intervalSeconds) && intervalSeconds > 0
        ? static_cast<uint32_t>(std::fmin(30'000'000.0, std::round(intervalSeconds * 1'000'000))) : 0;
}

struct AudioRecordingOptions {
    bool enabled = false;
    bool liveMonitoringEnabled = true;
    std::string inputDevice = "default";
    std::string outputDevice;
    uint32_t sampleRate = 0; // Auto prefers native 48 kHz.
    uint16_t channels = 0; // Auto selects up to two native input channels.
    uint16_t firstChannel = 1;
    std::string sampleFormat = "auto";
    uint8_t inputVolume = 60;
    uint8_t outputVolume = 55;
    double fps = 24.0;
    TimecodeMode timecodeMode = TimecodeMode::FreeRun;
};

struct GyroRecordingOptions {
    bool enabled = false;
};

struct RawFrameView {
    const uint8_t *data = nullptr;
    size_t size = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    RawBitDepth bitDepth = RawBitDepth::Bits12;
    uint8_t unpackedBytesPerSample = 0;
    uint32_t cropX = 0;
    uint32_t cropY = 0;
    unsigned int sampleBits = 0;
    bool gradationCompanded = false;
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
    uint32_t slotIndex = 0;
    uint64_t frameId = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    uint32_t captureWidth = 0;
    uint32_t captureHeight = 0;
    size_t size = 0;
    std::string fourcc = "YUV420";
    int64_t timestampUs = 0;
    bool registration = false;
};

inline const char *toString(CameraState state)
{
    switch (state) {
    case CameraState::Offline: return "offline";
    case CameraState::Starting: return "starting";
    case CameraState::Ready: return "ready";
    case CameraState::Standby: return "standby";
    case CameraState::Recording: return "recording";
    case CameraState::Error: return "error";
    case CameraState::Stopping: return "stopping";
    }
    return "unknown";
}

inline const char *toString(RecordingFormat format)
{
    switch (format) {
    case RecordingFormat::Cdng: return "cdng";
    case RecordingFormat::Mlv: return "mlv";
    case RecordingFormat::H264: return "h264";
    case RecordingFormat::LossyDng: return "lossydng";
    case RecordingFormat::Prores: return "prores";
    }
    return "cdng";
}

inline const char *toString(CdngCompression compression)
{
    return compression == CdngCompression::LosslessJpeg ? "lossless_jpeg" : "none";
}

inline const char *toWireValue(LossyDngRatio ratio)
{
    return ratio == LossyDngRatio::R5 ? "5to1" : "3to1";
}

inline const char *toString(ProresProfile profile)
{
    switch (profile) {
    case ProresProfile::HQ: return "hq";
    case ProresProfile::Standard: return "422";
    case ProresProfile::LT: return "lt";
    case ProresProfile::Proxy: return "proxy";
    }
    return "422";
}

inline const char *toString(PhotoCaptureFormat format)
{
    switch (format) {
    case PhotoCaptureFormat::Dng: return "dng";
    case PhotoCaptureFormat::Jpeg: return "jpeg";
    case PhotoCaptureFormat::Png: return "png";
    case PhotoCaptureFormat::Tiff: return "tiff";
    case PhotoCaptureFormat::DngJpeg: return "dng_jpeg";
    case PhotoCaptureFormat::DngPng: return "dng_png";
    case PhotoCaptureFormat::DngTiff: return "dng_tiff";
    }
    return "dng";
}

inline const char *toString(TimecodeMode mode)
{
    switch (mode) {
    case TimecodeMode::FreeRun: return "free_run";
    case TimecodeMode::RecRun: return "rec_run";
    }
    return "free_run";
}

inline const char *toString(AutofocusMode mode)
{
    switch (mode) {
    case AutofocusMode::Manual: return "manual";
    case AutofocusMode::Auto: return "auto";
    case AutofocusMode::Continuous: return "continuous";
    }
    return "continuous";
}

inline const char *toString(AutofocusRange range)
{
    switch (range) {
    case AutofocusRange::Normal: return "normal";
    case AutofocusRange::Macro: return "macro";
    case AutofocusRange::Full: return "full";
    }
    return "normal";
}

inline const char *toString(AutofocusSpeed speed)
{
    switch (speed) {
    case AutofocusSpeed::Normal: return "normal";
    case AutofocusSpeed::Fast: return "fast";
    }
    return "normal";
}

} // namespace apertar
