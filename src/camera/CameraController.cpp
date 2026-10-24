#include "camera/CameraController.hpp"
#include <apertar/core/RecordingCapabilities.hpp>
#include "camera/DefaultTuningSelection.hpp"
#include "camera/ExposureMetering.hpp"
#include "camera/StillBracketing.hpp"
#include "color/StillColorProcessing.hpp"
#include "color/StillSrgbProfile.hpp"
#include "color/ProresIspProfile.hpp"
#include "camera/ScalerCropGeometry.hpp"
#include "util/SettingsFile.hpp"
#include "util/AtomicStillFile.hpp"

#include "audio/AudioRecordingSession.hpp"
#include "cdng/CdngEncoder.hpp"
#include "cdng/BlackLevel.hpp"
#include "color/ColorProfileConfig.hpp"
#include "hardware/Mpu6050RecordingSession.hpp"
#include "ipc/ControlProtocol.hpp"
#include "lossydng/LossyDngEncoder.hpp"
#include "mlv/MlvEncoder.hpp"
#include "video/LibavH264EncoderClient.hpp"
#include "video/H264Hardware.hpp"
#include "util/RecordingFailure.hpp"
#include "prores/ProresRecorder.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <deque>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <shared_mutex>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#if defined(__linux__)
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#if APERTARCORE_HAS_TURBOJPEG
#include <turbojpeg.h>
#elif APERTARCORE_HAS_JPEG
#include <jpeglib.h>
#endif

#if APERTARCORE_HAS_LIBCAMERA
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/geometry.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace apertar {
namespace {

std::string escapeJson(const std::string &value)
{
    std::ostringstream out;
    for (char ch : value) {
        switch (ch) {
        case '\\':
            out << "\\\\";
            break;
        case '"':
            out << "\\\"";
            break;
        case '\b':
            out << "\\b";
            break;
        case '\f':
            out << "\\f";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            if (static_cast<unsigned char>(ch) < 0x20) {
                out << "\\u"
                    << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<int>(static_cast<unsigned char>(ch))
                    << std::dec << std::setfill(' ');
            } else {
                out << ch;
            }
            break;
        }
    }
    return out.str();
}

std::string settingsEvent(const CaptureSettings &settings)
{
    std::ostringstream out;
    out << "{\"event\":\"settings\""
        << ",\"recording_capabilities\":" << recordingCapabilitiesJson()
        << ",\"resolution\":\"" << settings.resolution.toString() << "\""
        << ",\"width\":" << settings.resolution.width
        << ",\"height\":" << settings.resolution.height
        << ",\"fps\":" << std::fixed << std::setprecision(3) << settings.fps
        << ",\"iso\":" << settings.iso
        << ",\"iso_auto\":" << (settings.isoAuto ? "true" : "false")
        << ",\"shutter_us\":" << settings.shutterUs
        << ",\"shutter_auto\":" << (settings.shutterAuto ? "true" : "false")
        << ",\"exposure_compensation\":" << settings.exposureCompensation
        << ",\"exposure_compensation_available\":" << (settings.exposureCompensationAvailable ? "true" : "false")
        << "," << exposureMeteringJson(settings)
        << ",\"photo_mode_enabled\":" << (settings.photoModeEnabled ? "true" : "false")
        << ",\"photo_long_exposure_enabled\":" << (settings.longExposureEnabled ? "true" : "false")
        << ",\"photo_bracketing_enabled\":" << (settings.bracketingEnabled ? "true" : "false")
        << ",\"photo_bracketing_ev\":" << settings.bracketingEv
        << ",\"wb_kelvin\":" << settings.whiteBalanceKelvin
        << ",\"wb_auto\":" << (settings.whiteBalanceAuto ? "true" : "false")
        << ",\"wb_available\":" << (settings.whiteBalanceAvailable ? "true" : "false")
        << ",\"wb_measured\":" << (settings.whiteBalanceMeasured.valid() ? "true" : "false")
        << ",\"wb_actual_kelvin\":" << settings.whiteBalanceMeasured.kelvin
        << ",\"wb_actual_red_gain\":" << settings.whiteBalanceMeasured.red
        << ",\"wb_actual_blue_gain\":" << settings.whiteBalanceMeasured.blue
        << ",\"recording_format\":\"" << toString(settings.recordingFormat) << "\""
        << ",\"cdng_compression\":\"" << toString(settings.cdngCompression) << "\""
        << ",\"lossy_dng_ratio\":\"" << toWireValue(settings.lossyDngRatio) << "\""
        << ",\"prores_profile\":\"" << toString(settings.proresProfile) << "\""
        << ",\"prores_color_profile\":\"" << toString(settings.proresColorProfile) << "\""
        << ",\"cdng_preview_color_profile\":\"" << toString(settings.cdngPreviewColorProfile) << "\""
        << ",\"preview_exposure_reference\":" << previewExposureReferenceJson(settings.previewExposureReference.get())
        << ",\"tuning_file\":\"" << escapeJson(settings.tuningFile) << "\""
        << ",\"raw_bit_depth\":" << settings.rawBitDepth
        << ",\"h264_resolution\":\"" << (settings.h264Resolution.width > 0 && settings.h264Resolution.height > 0
                                             ? settings.h264Resolution.toString()
                                             : "match") << "\""
        << ",\"h264_width\":" << settings.h264Resolution.width
        << ",\"h264_height\":" << settings.h264Resolution.height
        << ",\"timecode_mode\":\"" << toString(settings.timecodeMode) << "\""
        << ",\"af_mode\":\"" << toString(settings.autofocusMode) << "\""
        << ",\"af_range\":\"" << toString(settings.autofocusRange) << "\""
        << ",\"af_speed\":\"" << toString(settings.autofocusSpeed) << "\""
        << ",\"af_window_x\":" << std::fixed << std::setprecision(3) << settings.autofocusWindowX
        << ",\"af_window_y\":" << std::fixed << std::setprecision(3) << settings.autofocusWindowY
        << ",\"af_window_w\":" << std::fixed << std::setprecision(3) << settings.autofocusWindowWidth
        << ",\"af_window_h\":" << std::fixed << std::setprecision(3) << settings.autofocusWindowHeight
        << "}";
    return out.str();
}

uint32_t shutterUsForPreservedAngle(uint32_t currentShutterUs, double currentFps, double nextFps)
{
    if (currentShutterUs == 0 || !(currentFps > 0.0) || !(nextFps > 0.0))
        return currentShutterUs;

    const double scaled = static_cast<double>(currentShutterUs) * (currentFps / nextFps);
    const int64_t rounded = std::max<int64_t>(1, std::llround(scaled));
    return std::clamp<uint32_t>(rounded, 1u, 1'000'000u);
}

RecordingFormat recordingFormatFromString(const std::string &value)
{
    std::string normalized;
    normalized.reserve(value.size());
    for (char ch : value)
        normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));

    if (normalized == "mlv")
        return RecordingFormat::Mlv;
    if (normalized == "h264" || normalized == "h.264" || normalized == "mp4")
        return RecordingFormat::H264;
    if (normalized == "prores")
        return RecordingFormat::Prores;
    if (normalized == "lossydng" || normalized == "lossy_dng" ||
        normalized == "cdng3to1" || normalized == "cdng 3to1" || normalized == "cdng 3:1" ||
        normalized == "cinemadng 3to1" || normalized == "cinemadng 3:1" ||
        normalized == "cdng5to1" || normalized == "cdng 5to1" || normalized == "cdng 5:1" ||
        normalized == "cinemadng 5to1" || normalized == "cinemadng 5:1")
        return RecordingFormat::LossyDng;
    return RecordingFormat::Cdng;
}

bool isProcessedVideoFormat(RecordingFormat format)
{
    return format == RecordingFormat::H264 || format == RecordingFormat::Prores;
}

LossyDngRatio lossyDngRatioFromString(const std::string &value)
{
    std::string normalized;
    normalized.reserve(value.size());
    for (char ch : value) {
        if (std::isalnum(static_cast<unsigned char>(ch)))
            normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
    return normalized == "5to1" || normalized == "51" || normalized == "5" ||
           normalized == "cdng5to1" ? LossyDngRatio::R5 : LossyDngRatio::R3;
}

bool isFixedTenBitRecordingFormat(RecordingFormat format)
{
    return isProcessedVideoFormat(format);
}

uint32_t normalizeRawBitDepth(uint32_t bitDepth)
{
    if (bitDepth == 8)
        return 8u;
    if (bitDepth == 10)
        return 10u;
    return 12u;
}

bool isSupportedH264Resolution(const Resolution &resolution)
{
    if (resolution.width == 0 && resolution.height == 0)
        return true;
    if (recordingCapabilities().hardwareH264 && (resolution.width > 1920 || resolution.height > 1080))
        return false;

    struct Size {
        uint32_t width;
        uint32_t height;
    };
    static constexpr Size kSupportedSizes[] = {
        { 1920, 1080 }, { 1920, 1038 }, { 1920, 1012 }, { 1920, 960 }, { 1920, 872 },
        { 1920, 864 }, { 1920, 816 }, { 1920, 804 }, { 1920, 768 }, { 1920, 752 },
        { 2560, 1440 }, { 2560, 1384 }, { 2560, 1348 }, { 2560, 1280 }, { 2560, 1164 },
        { 2560, 1152 }, { 2560, 1088 }, { 2560, 1072 }, { 2560, 1024 }, { 2560, 1004 }
    };

    for (const Size &size : kSupportedSizes) {
        if (resolution.width == size.width && resolution.height == size.height)
            return true;
    }
    return false;
}

bool isSupportedProcessedVideoResolution(RecordingFormat format, const Resolution &resolution)
{
    (void)format;
    return isSupportedH264Resolution(resolution);
}

Resolution normalizeProcessedVideoResolution(RecordingFormat format, const Resolution &resolution)
{
    return isSupportedProcessedVideoResolution(format, resolution)
        ? resolution
        : CaptureSettings{}.h264Resolution;
}

TimecodeMode timecodeModeFromString(const std::string &value)
{
    std::string normalized;
    normalized.reserve(value.size());
    for (char ch : value)
        normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));

    if (normalized == "rec_run" || normalized == "recrun")
        return TimecodeMode::RecRun;
    return TimecodeMode::FreeRun;
}

AutofocusMode autofocusModeFromString(const std::string &value)
{
    std::string normalized;
    normalized.reserve(value.size());
    for (char ch : value)
        normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));

    if (normalized == "manual")
        return AutofocusMode::Manual;
    if (normalized == "auto")
        return AutofocusMode::Auto;
    return AutofocusMode::Continuous;
}

AutofocusRange autofocusRangeFromString(const std::string &value)
{
    std::string normalized;
    normalized.reserve(value.size());
    for (char ch : value)
        normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));

    if (normalized == "macro")
        return AutofocusRange::Macro;
    if (normalized == "full")
        return AutofocusRange::Full;
    return AutofocusRange::Normal;
}

AutofocusSpeed autofocusSpeedFromString(const std::string &value)
{
    std::string normalized;
    normalized.reserve(value.size());
    for (char ch : value)
        normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));

    if (normalized == "fast")
        return AutofocusSpeed::Fast;
    return AutofocusSpeed::Normal;
}

std::filesystem::path persistedSettingsPath()
{
    const char *xdgConfig = std::getenv("XDG_CONFIG_HOME");
    if (xdgConfig && *xdgConfig)
        return std::filesystem::path(xdgConfig) / "apertar-core" / "camera_settings.conf";

    const char *home = std::getenv("HOME");
    if (home && *home)
        return std::filesystem::path(home) / ".config" / "apertar-core" / "camera_settings.conf";

    return std::filesystem::temp_directory_path() / "apertar-core-camera_settings.conf";
}

bool resolutionMatches(const Resolution &resolution, uint32_t width, uint32_t height)
{
    return resolution.width == width && resolution.height == height;
}

bool isImx585LowResolutionAlias(const Resolution &resolution)
{
    return resolutionMatches(resolution, 1920, 1080) ||
           resolutionMatches(resolution, 1928, 1090);
}

bool isImx585FullHdCropResolution(const Resolution &resolution)
{
    return resolutionMatches(resolution, 1920, 1038) ||
           resolutionMatches(resolution, 1920, 1012) ||
           resolutionMatches(resolution, 1920, 960) ||
           resolutionMatches(resolution, 1920, 872) ||
           resolutionMatches(resolution, 1920, 864) ||
           resolutionMatches(resolution, 1920, 816) ||
           resolutionMatches(resolution, 1920, 804) ||
           resolutionMatches(resolution, 1920, 768) ||
           resolutionMatches(resolution, 1920, 752);
}

bool isImx585HighResolutionAlias(const Resolution &resolution)
{
    return resolutionMatches(resolution, 3840, 2160) ||
           resolutionMatches(resolution, 3856, 2180);
}

bool isImx585Raw12CropResolution(const Resolution &resolution)
{
    return resolutionMatches(resolution, 3840, 2076) ||
           resolutionMatches(resolution, 3840, 2024) ||
           resolutionMatches(resolution, 3840, 1920) ||
           resolutionMatches(resolution, 3840, 1744) ||
           resolutionMatches(resolution, 3840, 1728) ||
           resolutionMatches(resolution, 3840, 1632) ||
           resolutionMatches(resolution, 3840, 1608) ||
           resolutionMatches(resolution, 3840, 1536) ||
           resolutionMatches(resolution, 3840, 1504);
}

bool isImx477Resolution(const Resolution &resolution)
{
    if (resolutionMatches(resolution, 1332, 990) ||
        resolutionMatches(resolution, 2028, 1520) ||
        resolutionMatches(resolution, 2028, 1080) ||
        resolutionMatches(resolution, 4056, 3040) ||
        resolutionMatches(resolution, 4056, 2160))
        return true;

    if (resolution.width == 2028) {
        switch (resolution.height) {
        case 1140:
        case 1096:
        case 1068:
        case 1014:
        case 922:
        case 914:
        case 864:
        case 848:
        case 812:
        case 796:
            return true;
        default:
            break;
        }
    }

    if (resolution.width == 4056) {
        switch (resolution.height) {
        case 2280:
        case 2192:
        case 2136:
        case 2028:
        case 1844:
        case 1828:
        case 1728:
        case 1696:
        case 1624:
        case 1592:
            return true;
        default:
            break;
        }
    }

    return false;
}

Resolution normalizedImx477Resolution(Resolution requested)
{
    if (isImx477Resolution(requested))
        return requested;
    if (isImx585LowResolutionAlias(requested) ||
        resolutionMatches(requested, 1920, 1080) ||
        resolutionMatches(requested, 1928, 1090))
        return Resolution{ 2028, 1080 };
    if (isImx585HighResolutionAlias(requested) ||
        resolutionMatches(requested, 4056, 2160))
        return Resolution{ 4056, 2160 };
    if (resolutionMatches(requested, 4056, 3040))
        return Resolution{ 4056, 3040 };
    return Resolution{ 2028, 1080 };
}

bool isImx294Resolution(const Resolution &resolution)
{
    return resolutionMatches(resolution, 4144, 2176) ||
           resolutionMatches(resolution, 4176, 2176) ||
           resolutionMatches(resolution, 3872, 2176) ||
           resolutionMatches(resolution, 4096, 2048) ||
           resolutionMatches(resolution, 4096, 1844) ||
           resolutionMatches(resolution, 4096, 1744) ||
           resolutionMatches(resolution, 4096, 1716) ||
           resolutionMatches(resolution, 4096, 1640) ||
           resolutionMatches(resolution, 2048, 1080) ||
           resolutionMatches(resolution, 1920, 1080) ||
           resolutionMatches(resolution, 3792, 2824) ||
           resolutionMatches(resolution, 3792, 2528) ||
           resolutionMatches(resolution, 3392, 2824);
}

bool isImx294TenBitResolution(const Resolution &resolution)
{
    return resolutionMatches(resolution, 4144, 2176) ||
           resolutionMatches(resolution, 3872, 2176) ||
           resolutionMatches(resolution, 4096, 2048) ||
           resolutionMatches(resolution, 4096, 1844) ||
           resolutionMatches(resolution, 4096, 1744) ||
           resolutionMatches(resolution, 4096, 1716) ||
           resolutionMatches(resolution, 4096, 1640) ||
           resolutionMatches(resolution, 2048, 1080) ||
           resolutionMatches(resolution, 1920, 1080) ||
           resolutionMatches(resolution, 3792, 2824) ||
           resolutionMatches(resolution, 3792, 2528) ||
           resolutionMatches(resolution, 3392, 2824);
}

bool isImx294TwelveBitResolution(const Resolution &resolution)
{
    return resolutionMatches(resolution, 4176, 2176) ||
           resolutionMatches(resolution, 3872, 2176) ||
           resolutionMatches(resolution, 4096, 2048) ||
           resolutionMatches(resolution, 4096, 1844) ||
           resolutionMatches(resolution, 4096, 1744) ||
           resolutionMatches(resolution, 4096, 1716) ||
           resolutionMatches(resolution, 4096, 1640) ||
           resolutionMatches(resolution, 2048, 1080) ||
           resolutionMatches(resolution, 1920, 1080) ||
           resolutionMatches(resolution, 3792, 2824) ||
           resolutionMatches(resolution, 3792, 2528) ||
           resolutionMatches(resolution, 3392, 2824);
}

Resolution normalizedImx294ResolutionForBitDepth(Resolution requested, uint32_t rawBitDepth)
{
    // Retire the small central RAW windows, including saved selections.
    if (resolutionMatches(requested, 1920, 1080))
        return Resolution{ 3872, 2176 };
    if (resolutionMatches(requested, 2048, 1080) ||
        resolutionMatches(requested, 2028, 1080))
        return Resolution{ normalizeRawBitDepth(rawBitDepth) == 10 ? 4144u : 4176u, 2176 };
    if (normalizeRawBitDepth(rawBitDepth) == 10) {
        if (isImx294TenBitResolution(requested))
            return requested;
        if (resolutionMatches(requested, 4056, 3040))
            return Resolution{ 3792, 2824 };
        return Resolution{ 4144, 2176 };
    }

    if (isImx294TwelveBitResolution(requested))
        return requested;
    if (resolutionMatches(requested, 4056, 3040))
        return Resolution{ 3792, 2824 };
    return Resolution{ 4176, 2176 };
}

CaptureSettings clampCaptureSettings(CaptureSettings settings)
{
    const auto metering = normalizeExposureMetering(settings.exposureMetering);
    settings.exposureMetering = metering.empty() ? "centre-weighted" : metering;
    settings.exposureCompensation = std::isfinite(settings.exposureCompensation)
        ? std::round(std::clamp(settings.exposureCompensation, -3.0, 3.0) * 3.0) / 3.0 : 0.0;
    settings.bracketingEv = std::isfinite(settings.bracketingEv)
        ? std::round(std::clamp(settings.bracketingEv, 1.0 / 3.0, 3.0) * 3.0) / 3.0 : 1.0;
    const bool supportedResolution =
        (settings.resolution.width == 1332 && settings.resolution.height == 990) ||
        (settings.resolution.width == 2028 && settings.resolution.height == 1080) ||
        (settings.resolution.width == 2028 && settings.resolution.height == 1520) ||
        (settings.resolution.width == 4056 && settings.resolution.height == 2160) ||
        (settings.resolution.width == 4056 && settings.resolution.height == 3040) ||
        isImx477Resolution(settings.resolution) ||
        isImx585LowResolutionAlias(settings.resolution) ||
        isImx585FullHdCropResolution(settings.resolution) ||
        isImx585HighResolutionAlias(settings.resolution) ||
        isImx585Raw12CropResolution(settings.resolution) ||
        isImx294Resolution(settings.resolution);
    if (!supportedResolution) {
        settings.resolution = CaptureSettings{}.resolution;
    }

    if (!(settings.fps > 0.0 && settings.fps <= 280.0))
        settings.fps = CaptureSettings{}.fps;

    if (settings.iso < 50 || settings.iso > 51200)
        settings.iso = CaptureSettings{}.iso;

    if (settings.shutterUs == 0 || settings.shutterUs > 30'000'000)
        settings.shutterUs = CaptureSettings{}.shutterUs;
    if (!settings.photoModeEnabled) settings.timelapseExposureLimitUs = 0;
    if (settings.timelapseExposureLimitUs)
        settings.shutterUs = std::min(settings.shutterUs, settings.timelapseExposureLimitUs);
    if (!(settings.photoModeEnabled && settings.longExposureEnabled)) {
        const uint32_t maximum = settings.photoModeEnabled
            ? uint32_t(std::ceil(1'000'000 / std::min(24.0, settings.fps)))
            : uint32_t(std::ceil(1'000'000 / settings.fps));
        settings.shutterUs = std::min(settings.shutterUs, maximum);
    }

    if (settings.whiteBalanceKelvin < 1500 || settings.whiteBalanceKelvin > 12000)
        settings.whiteBalanceKelvin = CaptureSettings{}.whiteBalanceKelvin;

    if (settings.recordingFormat != RecordingFormat::Cdng &&
        settings.recordingFormat != RecordingFormat::Mlv &&
        settings.recordingFormat != RecordingFormat::H264 &&
        settings.recordingFormat != RecordingFormat::Prores &&
        settings.recordingFormat != RecordingFormat::LossyDng) {
        settings.recordingFormat = CaptureSettings{}.recordingFormat;
    }

    settings.rawBitDepth = normalizeRawBitDepth(settings.rawBitDepth);
    settings.h264Resolution = supportedRecordingSettings(settings).h264Resolution;
    settings.h264Resolution = normalizeProcessedVideoResolution(settings.recordingFormat, settings.h264Resolution);

    settings.autofocusWindowX = std::clamp(settings.autofocusWindowX, 0.0, 1.0);
    settings.autofocusWindowY = std::clamp(settings.autofocusWindowY, 0.0, 1.0);
    settings.autofocusWindowWidth = std::clamp(settings.autofocusWindowWidth, 0.01, 1.0);
    settings.autofocusWindowHeight = std::clamp(settings.autofocusWindowHeight, 0.01, 1.0);
    if (settings.autofocusWindowX + settings.autofocusWindowWidth > 1.0)
        settings.autofocusWindowWidth = std::max(0.01, 1.0 - settings.autofocusWindowX);
    if (settings.autofocusWindowY + settings.autofocusWindowHeight > 1.0)
        settings.autofocusWindowHeight = std::max(0.01, 1.0 - settings.autofocusWindowY);

    return settings;
}

std::string normalizeTuningFilePath(std::string value);

RecordingFormat settingsProfileFormat(RecordingFormat format)
{
    return format == RecordingFormat::LossyDng ? RecordingFormat::Cdng : format;
}

struct PersistedCaptureSettings {
    CaptureSettings active;
    std::map<RecordingFormat, CaptureSettings> formats;
    std::optional<CaptureSettings> still;
};

CaptureSettings loadPersistedCaptureSettings(std::map<RecordingFormat, CaptureSettings> &formats,
                                           std::optional<CaptureSettings> &still)
{
    CaptureSettings current = CaptureSettings{};
    const std::filesystem::path path = persistedSettingsPath();
    std::ifstream input(path);
    if (!input.is_open())
        return current;

    bool legacyWhiteBalanceLock = false;
    std::string line;
    while (std::getline(input, line)) {
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos)
            continue;
        std::string key = line.substr(0, eq);
        const std::string value = line.substr(eq + 1);
        CaptureSettings *target = &current;
        if (key.starts_with("profile.")) {
            const auto separator = key.find('.', 8);
            if (separator == std::string::npos)
                continue;
            const std::string name = key.substr(8, separator - 8);
            if (name != "still" && name != "cdng" && name != "mlv" && name != "h264" && name != "prores")
                continue;
            if (name == "still") {
                if (!still) still = current;
                target = &*still;
            } else {
                target = &formats.try_emplace(recordingFormatFromString(name), current).first->second;
            }
            key = key.substr(separator + 1);
        }
        CaptureSettings &settings = *target;
        try {
            if (key == "resolution_width")
                settings.resolution.width = static_cast<uint32_t>(std::stoul(value));
            else if (key == "resolution_height")
                settings.resolution.height = static_cast<uint32_t>(std::stoul(value));
            else if (key == "fps")
                settings.fps = std::stod(value);
            else if (key == "iso")
                settings.iso = static_cast<uint32_t>(std::stoul(value));
            else if (key == "iso_auto")
                settings.isoAuto = std::stoul(value) != 0;
            else if (key == "shutter_us")
                settings.shutterUs = static_cast<uint32_t>(std::stoul(value));
            else if (key == "shutter_auto")
                settings.shutterAuto = std::stoul(value) != 0;
            else if (key == "exposure_compensation")
                settings.exposureCompensation = std::stod(value);
            else if (key == "exposure_metering")
                settings.exposureMetering = value;
            else if (key == "photo_mode_enabled")
                settings.photoModeEnabled = std::stoul(value) != 0;
            else if (key == "long_exposure_enabled")
                settings.longExposureEnabled = std::stoul(value) != 0;
            else if (key == "bracketing_enabled")
                settings.bracketingEnabled = std::stoul(value) != 0;
            else if (key == "bracketing_ev")
                settings.bracketingEv = std::stod(value);
            else if (key == "wb_kelvin")
                settings.whiteBalanceKelvin = static_cast<uint32_t>(std::stoul(value));
            else if (key == "wb_auto")
                settings.whiteBalanceAuto = std::stoul(value) != 0;
            else if (key == "wb_locked")
                legacyWhiteBalanceLock = std::stoul(value) != 0;
            else if (key == "recording_format")
                settings.recordingFormat = recordingFormatFromString(value);
            else if (key == "cdng_compression")
                settings.cdngCompression = value == "lossless_jpeg"
                    ? CdngCompression::LosslessJpeg : CdngCompression::Uncompressed;
            else if (key == "lossy_dng_ratio")
                settings.lossyDngRatio = lossyDngRatioFromString(value);
            else if (key == "prores_color_profile")
                parseProresColorProfile(value, settings.proresColorProfile);
            else if (key == "cdng_preview_color_profile")
                parsePreviewColorProfile(value, settings.cdngPreviewColorProfile);
            else if (key == "prores_profile")
                settings.proresProfile = value == "hq" ? ProresProfile::HQ : value == "lt" ? ProresProfile::LT
                    : value == "proxy" ? ProresProfile::Proxy : ProresProfile::Standard;
            else if (key == "tuning_file")
                settings.tuningFile = normalizeTuningFilePath(value);
            else if (key == "raw_bit_depth")
                settings.rawBitDepth = normalizeRawBitDepth(static_cast<uint32_t>(std::stoul(value)));
            else if (key == "h264_width")
                settings.h264Resolution.width = static_cast<uint32_t>(std::stoul(value));
            else if (key == "h264_height")
                settings.h264Resolution.height = static_cast<uint32_t>(std::stoul(value));
            else if (key == "timecode_mode")
                settings.timecodeMode = timecodeModeFromString(value);
            else if (key == "af_mode")
                settings.autofocusMode = autofocusModeFromString(value);
            else if (key == "af_range")
                settings.autofocusRange = autofocusRangeFromString(value);
            else if (key == "af_speed")
                settings.autofocusSpeed = autofocusSpeedFromString(value);
            else if (key == "af_window_x")
                settings.autofocusWindowX = std::stod(value);
            else if (key == "af_window_y")
                settings.autofocusWindowY = std::stod(value);
            else if (key == "af_window_w")
                settings.autofocusWindowWidth = std::stod(value);
            else if (key == "af_window_h")
                settings.autofocusWindowHeight = std::stod(value);
        } catch (...) {
            // Ignore malformed values and keep defaults for resilience.
        }
    }

    // Retired locked gains must not survive as an invisible manual mode.
    if (legacyWhiteBalanceLock)
        current.whiteBalanceAuto = true;
    for (auto &[format, profile] : formats) {
        profile.photoModeEnabled = false;
        profile = clampCaptureSettings(profile);
        if (settingsProfileFormat(profile.recordingFormat) != format)
            profile.recordingFormat = format;
    }
    if (still) {
        still->photoModeEnabled = true;
        *still = clampCaptureSettings(*still);
    }
    return clampCaptureSettings(current);
}

void writeCaptureSettings(std::ostream &output, const CaptureSettings &settings, const std::string &prefix)
{
    const CaptureSettings s = clampCaptureSettings(settings);
    output << prefix << "resolution_width=" << s.resolution.width << '\n';
    output << prefix << "resolution_height=" << s.resolution.height << '\n';
    output << prefix << "fps=" << std::fixed << std::setprecision(6) << s.fps << '\n';
    output << prefix << "iso=" << s.iso << '\n';
    output << prefix << "iso_auto=" << s.isoAuto << '\n';
    output << prefix << "shutter_us=" << s.shutterUs << '\n';
    output << prefix << "shutter_auto=" << s.shutterAuto << '\n';
    output << prefix << "exposure_compensation=" << s.exposureCompensation << '\n';
    output << prefix << "exposure_metering=" << s.exposureMetering << '\n';
    output << prefix << "photo_mode_enabled=" << s.photoModeEnabled << '\n';
    output << prefix << "long_exposure_enabled=" << s.longExposureEnabled << '\n';
    output << prefix << "bracketing_enabled=" << s.bracketingEnabled << '\n';
    output << prefix << "bracketing_ev=" << s.bracketingEv << '\n';
    output << prefix << "wb_kelvin=" << s.whiteBalanceKelvin << '\n';
    output << prefix << "wb_auto=" << s.whiteBalanceAuto << '\n';
    output << prefix << "recording_format=" << toString(s.recordingFormat) << '\n';
    output << prefix << "cdng_compression=" << toString(s.cdngCompression) << '\n';
    output << prefix << "lossy_dng_ratio=" << toWireValue(s.lossyDngRatio) << '\n';
    output << prefix << "prores_profile=" << toString(s.proresProfile) << '\n';
    output << prefix << "prores_color_profile=" << toString(s.proresColorProfile) << '\n';
    output << prefix << "cdng_preview_color_profile=" << toString(s.cdngPreviewColorProfile) << '\n';
    output << prefix << "raw_bit_depth=" << s.rawBitDepth << '\n';
    output << prefix << "h264_width=" << s.h264Resolution.width << '\n';
    output << prefix << "h264_height=" << s.h264Resolution.height << '\n';
    output << prefix << "tuning_file=" << s.tuningFile << '\n';
    output << prefix << "timecode_mode=" << toString(s.timecodeMode) << '\n';
    output << prefix << "af_mode=" << toString(s.autofocusMode) << '\n';
    output << prefix << "af_range=" << toString(s.autofocusRange) << '\n';
    output << prefix << "af_speed=" << toString(s.autofocusSpeed) << '\n';
    output << prefix << "af_window_x=" << std::setprecision(9) << s.autofocusWindowX << '\n';
    output << prefix << "af_window_y=" << s.autofocusWindowY << '\n';
    output << prefix << "af_window_w=" << s.autofocusWindowWidth << '\n';
    output << prefix << "af_window_h=" << s.autofocusWindowHeight << '\n';
}

void persistCaptureSettingsNow(const PersistedCaptureSettings &snapshot)
{
    const std::filesystem::path path = persistedSettingsPath();
    std::ostringstream output;
    writeCaptureSettings(output, snapshot.active, "");
    for (const auto &[format, profile] : snapshot.formats)
        writeCaptureSettings(output, profile, "profile." + std::string(toString(format)) + ".");
    if (snapshot.still)
        writeCaptureSettings(output, *snapshot.still, "profile.still.");
    if (!settingsFile::atomicWrite(path, output.str()))
        std::cerr << "ApertarCore could not persist camera settings: " << path << '\n';
}

class CaptureSettingsPersister {
public:
    ~CaptureSettingsPersister()
    {
        stop();
    }

    void schedule(const CaptureSettings &settings, const std::map<RecordingFormat, CaptureSettings> &formats,
                  const std::optional<CaptureSettings> &still)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latestSettings_ = {clampCaptureSettings(settings), formats, still};
            dirty_ = true;
            if (!worker_.joinable())
                worker_ = std::thread(&CaptureSettingsPersister::run, this);
        }
        cv_.notify_one();
    }

    void flush()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!worker_.joinable()) {
            if (dirty_) {
                const PersistedCaptureSettings latest = latestSettings_;
                dirty_ = false;
                lock.unlock();
                persistCaptureSettingsNow(latest);
            }
            return;
        }

        flushRequested_ = true;
        cv_.notify_one();
        flushedCv_.wait(lock, [this]() { return !dirty_ && !writing_; });
        flushRequested_ = false;
    }

private:
    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopRequested_ = true;
        }
        cv_.notify_one();
        if (worker_.joinable())
            worker_.join();
    }

    void run()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (true) {
            cv_.wait(lock, [this]() { return stopRequested_ || dirty_; });
            if (stopRequested_ && !dirty_)
                break;

            if (!flushRequested_) {
                cv_.wait_for(lock, std::chrono::milliseconds(25), [this]() {
                    return stopRequested_ || flushRequested_;
                });
            }

            const PersistedCaptureSettings latest = latestSettings_;
            dirty_ = false;
            writing_ = true;

            lock.unlock();
            persistCaptureSettingsNow(latest);
            lock.lock();

            writing_ = false;
            flushedCv_.notify_all();

            if (stopRequested_ && !dirty_)
                break;
        }
    }

    std::mutex mutex_;
    std::condition_variable cv_;
    std::condition_variable flushedCv_;
    std::thread worker_;
    PersistedCaptureSettings latestSettings_{};
    bool dirty_ = false;
    bool writing_ = false;
    bool flushRequested_ = false;
    bool stopRequested_ = false;
};

CaptureSettingsPersister &captureSettingsPersister()
{
    static CaptureSettingsPersister persister;
    return persister;
}

std::filesystem::path mediaRootPath(const std::string &mediaRoot)
{
    return mediaRoot.empty() ? std::filesystem::path("/media/RAW")
                             : std::filesystem::path(mediaRoot);
}

std::filesystem::path withPhotoExtension(std::filesystem::path basePath, const char *extension)
{
    basePath += extension;
    return basePath;
}

bool photoBasePathExists(const std::filesystem::path &candidateBase)
{
    static const char *kExtensions[] = {
        ".dng",
        ".jpg",
        ".png",
        ".tif",
        ".tiff",
    };

    for (const char *suffix : {"", "_EVm1", "_EV0", "_EVp1", "_BRm", "_BR0", "_BRp"}) {
        for (const char *extension : kExtensions) {
            std::error_code ec;
            if (std::filesystem::exists(withPhotoExtension(candidateBase.string() + suffix, extension), ec))
                return true;
        }
    }

    return false;
}

std::filesystem::path allocatePhotoBasePath(const std::string &mediaRoot)
{
    const std::filesystem::path directory = mediaRootPath(mediaRoot) / "Photos";
    std::error_code dirEc;
    const bool created = std::filesystem::create_directories(directory, dirEc);
    if (dirEc)
        throw std::runtime_error("Could not create Photos folder: " + dirEc.message());
    if (created) syncStillDirectory(directory.parent_path());

    const auto now = std::chrono::system_clock::now();
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
#if defined(_WIN32)
    localtime_s(&localTime, &seconds);
#else
    localtime_r(&seconds, &localTime);
#endif

    std::ostringstream prefix;
    prefix << "Still_" << std::put_time(&localTime, "%y-%m-%d");

    static std::mutex allocationMutex;
    static std::unordered_map<std::string, int> nextPhotoByPrefix;

    std::lock_guard<std::mutex> lock(allocationMutex);
    const std::string allocationKey = directory.string() + "|" + prefix.str();
    int &nextPhoto = nextPhotoByPrefix[allocationKey];
    if (nextPhoto < 1)
        nextPhoto = 1;

    for (int photo = nextPhoto; photo <= 9999; ++photo) {
        std::ostringstream name;
        name << prefix.str() << "_C" << std::setw(4) << std::setfill('0') << photo;

        const std::filesystem::path candidateBase = directory / name.str();
        if (!photoBasePathExists(candidateBase)) {
            nextPhoto = photo + 1;
            return candidateBase;
        }
    }

    throw std::runtime_error("Could not allocate a unique photo filename.");
}

std::filesystem::path allocateTimelapseFolder(const std::string &mediaRoot)
{
    const std::filesystem::path root = mediaRootPath(mediaRoot);
    std::error_code rootEc;
    std::filesystem::create_directories(root, rootEc);
    if (rootEc)
        throw std::runtime_error("Could not create media folder: " + rootEc.message());

    const auto now = std::chrono::system_clock::now();
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
#if defined(_WIN32)
    localtime_s(&localTime, &seconds);
#else
    localtime_r(&seconds, &localTime);
#endif

    std::ostringstream prefix;
    prefix << "Timelapse_" << std::put_time(&localTime, "%y-%m-%d");

    for (int sequence = 1; sequence <= 9999; ++sequence) {
        std::ostringstream name;
        name << prefix.str() << "_C" << std::setw(4) << std::setfill('0') << sequence;
        const std::filesystem::path candidate = root / name.str();
        std::error_code existsEc;
        if (!std::filesystem::exists(candidate, existsEc)) {
            std::error_code createEc;
            std::filesystem::create_directories(candidate, createEc);
            if (createEc)
                throw std::runtime_error("Could not create timelapse folder: " + createEc.message());
            syncStillDirectory(root);
            return candidate;
        }
    }

    throw std::runtime_error("Could not allocate a unique timelapse folder.");
}

std::filesystem::path timelapseFrameBasePath(const std::string &folderPath, uint64_t frameIndex)
{
    const std::filesystem::path folder(folderPath);
    const std::string sequenceName = folder.filename().string();
    std::ostringstream name;
    if (!sequenceName.empty())
        name << sequenceName << "_";
    name << std::setw(9) << std::setfill('0') << frameIndex;
    return folder / name.str();
}

#if APERTARCORE_HAS_LIBCAMERA

constexpr uint32_t kPreviewWidth = 1280;
constexpr uint32_t kPreviewHeight = 720;
constexpr uint32_t kDefaultH264EncodeWidth = 2560;
constexpr uint32_t kDefaultH264EncodeHeight = 1440;
constexpr unsigned int kConfiguredBufferCount = 4;
constexpr unsigned int kActiveRequestCount = 3;
constexpr int kPhotoJpegQuality = 92;

unsigned int unsignedEnvValue(const char *name,
                              unsigned int defaultValue,
                              unsigned int minimumValue,
                              unsigned int maximumValue)
{
    if (const char *value = std::getenv(name); value && *value) {
        char *end = nullptr;
        const unsigned long parsed = std::strtoul(value, &end, 10);
        if (end && *end == '\0' && parsed >= minimumValue)
            return static_cast<unsigned int>(std::min<unsigned long>(parsed, maximumValue));
    }
    return defaultValue;
}

bool boolEnvValue(const char *name, bool defaultValue)
{
    if (const char *value = std::getenv(name); value && *value) {
        std::string normalized;
        for (char ch : std::string(value))
            normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
        if (normalized == "1" ||
            normalized == "true" ||
            normalized == "yes" ||
            normalized == "on") {
            return true;
        }
        if (normalized == "0" ||
            normalized == "false" ||
            normalized == "no" ||
            normalized == "off") {
            return false;
        }
    }
    return defaultValue;
}

std::string normalizeTuningFilePath(std::string value)
{
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
    value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), value.end());
    if (value.rfind("file://", 0) == 0)
        value.erase(0, 7);
    if (value.empty())
        return {};

    std::error_code ec;
    return std::filesystem::weakly_canonical(std::filesystem::path(value), ec).string();
}

bool validateTuningFilePath(const std::string &path, std::string *error)
{
    if (path.empty())
        return true;

    std::error_code ec;
    const std::filesystem::path fsPath(path);
    if (!std::filesystem::exists(fsPath, ec) || ec) {
        if (error)
            *error = "Tuning file does not exist: " + path;
        return false;
    }
    if (!std::filesystem::is_regular_file(fsPath, ec) || ec) {
        if (error)
            *error = "Tuning file is not a regular file: " + path;
        return false;
    }
    if (fsPath.extension() != ".json" && fsPath.extension() != ".JSON") {
        if (error)
            *error = "Tuning file must be a .json file.";
        return false;
    }
    std::ifstream input(path);
    if (!input.is_open()) {
        if (error)
            *error = "Tuning file is not readable: " + path;
        return false;
    }
    return tuning::compatibleFile(fsPath, tuning::nativePipeline(), error);
}

void applyProcessTuningFile(const std::string &path)
{
    // RAW calibration must always read the original sensor tuning, not an ISP log derivative.
    ::unsetenv("APERTAR_SENSOR_TUNING_FILE");
    if (path.empty()) {
        ::unsetenv("LIBCAMERA_RPI_TUNING_FILE");
        return;
    }
    ::setenv("LIBCAMERA_RPI_TUNING_FILE", path.c_str(), 1);
    ::setenv("APERTAR_SENSOR_TUNING_FILE", path.c_str(), 1);
}


unsigned int h264CameraBufferCount()
{
    const unsigned int defaultCount = recordingCapabilities().hardwareH264 ? 4u : 6u;
    return unsignedEnvValue("APERTAR_H264_CAMERA_BUFFERS", defaultCount, 3, 12);
}

unsigned int proresCameraBufferCount()
{
    // Cover frame-thread borrows, the worker/queue, and camera pipeline headroom.
    return unsignedEnvValue("APERTAR_PRORES_CAMERA_BUFFERS", 12, 8, 32);
}

bool photoFormatWritesDng(PhotoCaptureFormat format)
{
    return format == PhotoCaptureFormat::Dng ||
           format == PhotoCaptureFormat::DngJpeg ||
           format == PhotoCaptureFormat::DngPng ||
           format == PhotoCaptureFormat::DngTiff;
}

bool photoFormatWritesPreview(PhotoCaptureFormat format)
{
    return format == PhotoCaptureFormat::Jpeg ||
           format == PhotoCaptureFormat::Png ||
           format == PhotoCaptureFormat::Tiff ||
           format == PhotoCaptureFormat::DngJpeg ||
           format == PhotoCaptureFormat::DngPng ||
           format == PhotoCaptureFormat::DngTiff;
}

bool photoFormatIsTimelapseSafe(PhotoCaptureFormat format)
{
    return format == PhotoCaptureFormat::Dng ||
           format == PhotoCaptureFormat::Jpeg ||
           format == PhotoCaptureFormat::Png ||
           format == PhotoCaptureFormat::Tiff;
}

void setStillError(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
}

void appendLe16(std::vector<uint8_t> &bytes, uint16_t value)
{
    bytes.push_back(static_cast<uint8_t>(value & 0xffu));
    bytes.push_back(static_cast<uint8_t>((value >> 8) & 0xffu));
}

void appendLe32(std::vector<uint8_t> &bytes, uint32_t value)
{
    bytes.push_back(static_cast<uint8_t>(value & 0xffu));
    bytes.push_back(static_cast<uint8_t>((value >> 8) & 0xffu));
    bytes.push_back(static_cast<uint8_t>((value >> 16) & 0xffu));
    bytes.push_back(static_cast<uint8_t>((value >> 24) & 0xffu));
}

void appendBe32(std::vector<uint8_t> &bytes, uint32_t value)
{
    bytes.push_back(static_cast<uint8_t>((value >> 24) & 0xffu));
    bytes.push_back(static_cast<uint8_t>((value >> 16) & 0xffu));
    bytes.push_back(static_cast<uint8_t>((value >> 8) & 0xffu));
    bytes.push_back(static_cast<uint8_t>(value & 0xffu));
}

bool writeBytesAtomically(const std::filesystem::path &finalPath,
                          const std::vector<uint8_t> &bytes,
                          std::string *error)
{
    if (bytes.empty()) {
        setStillError(error, "Encoded still image is empty.");
        return false;
    }

    try {
        AtomicStillFile output(finalPath);
        output.write(bytes.data(), bytes.size());
        output.commit(bytes.size());
    } catch (const std::exception &e) {
        setStillError(error, e.what());
        return false;
    }

    return true;
}

uint32_t pngCrc32(const uint8_t *data, size_t size, uint32_t crc = 0xffffffffu)
{
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return crc;
}

uint32_t adler32(const std::vector<uint8_t> &bytes)
{
    constexpr uint32_t kMod = 65521u;
    uint32_t a = 1;
    uint32_t b = 0;
    for (uint8_t byte : bytes) {
        a = (a + byte) % kMod;
        b = (b + a) % kMod;
    }
    return (b << 16) | a;
}

void appendPngChunk(std::vector<uint8_t> &png, const char type[4], const std::vector<uint8_t> &payload)
{
    appendBe32(png, static_cast<uint32_t>(payload.size()));
    const size_t typeOffset = png.size();
    png.insert(png.end(), type, type + 4);
    png.insert(png.end(), payload.begin(), payload.end());
    uint32_t crc = pngCrc32(png.data() + typeOffset, 4 + payload.size());
    appendBe32(png, crc ^ 0xffffffffu);
}

std::vector<uint8_t> zlibStoredStream(const std::vector<uint8_t> &raw)
{
    std::vector<uint8_t> out;
    out.reserve(raw.size() + (raw.size() / 65535u + 1u) * 5u + 6u);
    out.push_back(0x78);
    out.push_back(0x01);

    size_t offset = 0;
    while (offset < raw.size()) {
        const size_t blockSize = std::min<size_t>(65535, raw.size() - offset);
        const bool finalBlock = offset + blockSize >= raw.size();
        out.push_back(finalBlock ? 0x01 : 0x00);
        appendLe16(out, static_cast<uint16_t>(blockSize));
        appendLe16(out, static_cast<uint16_t>(~static_cast<uint16_t>(blockSize)));
        out.insert(out.end(), raw.begin() + static_cast<std::ptrdiff_t>(offset),
                   raw.begin() + static_cast<std::ptrdiff_t>(offset + blockSize));
        offset += blockSize;
    }

    appendBe32(out, adler32(raw));
    return out;
}

void appendTiffEntry(std::vector<uint8_t> &tiff,
                     uint16_t tag,
                     uint16_t type,
                     uint32_t count,
                     uint32_t valueOrOffset)
{
    appendLe16(tiff, tag);
    appendLe16(tiff, type);
    appendLe32(tiff, count);
    appendLe32(tiff, valueOrOffset);
}

struct ProcessedRgbStill {
    std::vector<uint16_t> rgb;
    uint32_t width = 0;
    uint32_t height = 0;

    bool valid() const
    {
        return width > 0 && height > 0 &&
               rgb.size() == static_cast<size_t>(width) * static_cast<size_t>(height) * 3u;
    }
};

int rawBitDepthValue(RawBitDepth bitDepth)
{
    switch (bitDepth) {
    case RawBitDepth::Bits8: return 8;
    case RawBitDepth::Bits10: return 10;
    case RawBitDepth::Bits12: return 12;
    case RawBitDepth::Bits16: return 16;
    }
    return 12;
}

RawBitDepth storageRawBitDepthForSettings(const CaptureSettings &settings)
{
    switch (normalizeRawBitDepth(settings.rawBitDepth)) {
    case 8: return RawBitDepth::Bits8;
    case 10: return RawBitDepth::Bits10;
    default: return RawBitDepth::Bits12;
    }
}

size_t packedCsi2RowBytes(uint32_t width, int bits)
{
    return (static_cast<size_t>(width) * static_cast<size_t>(bits) + 7u) / 8u;
}

uint16_t readLe16Sample(const uint8_t *src)
{
    return static_cast<uint16_t>(src[0] | (static_cast<uint16_t>(src[1]) << 8));
}

uint8_t unpackedBytesPerSampleForTransport(RawBitDepth bitDepth)
{
    return bitDepth == RawBitDepth::Bits8 ? 1u : 2u;
}

uint8_t unpackedBytesPerSampleForFrame(const RawFrameView &frame)
{
    if (frame.unpackedBytesPerSample == 1u || frame.unpackedBytesPerSample == 2u)
        return frame.unpackedBytesPerSample;
    return unpackedBytesPerSampleForTransport(frame.bitDepth);
}

uint32_t readLe32Sample(const uint8_t *src)
{
    return static_cast<uint32_t>(src[0])
        | (static_cast<uint32_t>(src[1]) << 8)
        | (static_cast<uint32_t>(src[2]) << 16)
        | (static_cast<uint32_t>(src[3]) << 24);
}

uint16_t pispStillDequantize(uint16_t q, int qmode)
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

uint16_t pispStillPostprocess(uint16_t value)
{
    constexpr uint16_t kPispCompressOffset = 2048;
    return static_cast<uint16_t>(std::min<int>(0xffff, value + kPispCompressOffset));
}

void decodePispStillSubBlock(uint16_t *dst, uint32_t word)
{
    const int qmode = word & 3;
    const int field0 = (word >> 2) & 511;
    const int field1 = (word >> 11) & 127;
    const int field2 = (word >> 18) & 127;
    const int field3 = (word >> 25) & 127;

    if (qmode == 0 || qmode == 1) {
        const bool upper = field1 >= 64;
        const int q1 = upper ? field0 : field0 + 64 - field1;
        const int q2 = upper ? field0 + field1 - 64 : field0;
        const int q0 = std::max(0, q1 - 64) + field2;
        const int q3 = std::max(0, q2 - 64) + field3;
        dst[0] = pispStillDequantize(static_cast<uint16_t>(q0), qmode);
        dst[2] = pispStillDequantize(static_cast<uint16_t>(q1), qmode);
        dst[4] = pispStillDequantize(static_cast<uint16_t>(q2), qmode);
        dst[6] = pispStillDequantize(static_cast<uint16_t>(q3), qmode);
        return;
    }

    if (qmode == 2) {
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
        dst[0] = pispStillDequantize(static_cast<uint16_t>(p1 + field2), 2);
        dst[2] = pispStillDequantize(static_cast<uint16_t>(q1), 2);
        dst[4] = pispStillDequantize(static_cast<uint16_t>(q2), 2);
        dst[6] = pispStillDequantize(static_cast<uint16_t>(p2 + field3), 2);
        return;
    }

    const int pack0 = (word >> 2) & 32767;
    const int pack1 = (word >> 17) & 32767;
    const int q0 = (pack0 & 15) + 16 * ((pack0 >> 8) / 11);
    const int q1 = (pack0 >> 4) % 176;
    const int q2 = (pack1 & 15) + 16 * ((pack1 >> 8) / 11);
    const int q3 = (pack1 >> 4) % 176;
    dst[0] = pispStillDequantize(static_cast<uint16_t>(q0), 3);
    dst[2] = pispStillDequantize(static_cast<uint16_t>(q1), 3);
    dst[4] = pispStillDequantize(static_cast<uint16_t>(q2), 3);
    dst[6] = pispStillDequantize(static_cast<uint16_t>(q3), 3);
}

void decodePispStillBlockTo12(const uint8_t *src, uint16_t *dst, uint32_t count)
{
    uint16_t working[8]{};
    decodePispStillSubBlock(working, readLe32Sample(src));
    decodePispStillSubBlock(working + 1, readLe32Sample(src + 4));
    for (int i = 0; i < 8; ++i)
        working[i] = pispStillPostprocess(working[i]);
    for (uint32_t i = 0; i < count; ++i)
        dst[i] = static_cast<uint16_t>(working[i] >> 4);
}

bool unpackRawFrameToBayer(const RawFrameView &frame,
                           std::vector<uint16_t> *bayer,
                           int *sampleBits,
                           std::string *error)
{
    if (!frame.data || frame.width == 0 || frame.height == 0 || frame.stride == 0) {
        setStillError(error, "Raw still frame has invalid geometry.");
        return false;
    }
    if (!bayer || !sampleBits) {
        setStillError(error, "Raw still decode target is missing.");
        return false;
    }

    const size_t pixelCount = static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height);
    bayer->assign(pixelCount, 0);
    int bits = rawBitDepthValue(frame.bitDepth);

    for (uint32_t y = 0; y < frame.height; ++y) {
        const size_t rowOffset = static_cast<size_t>(y) * static_cast<size_t>(frame.stride);
        if (rowOffset >= frame.size) {
            setStillError(error, "Raw still frame ended before all rows were decoded.");
            return false;
        }
        const uint8_t *src = frame.data + rowOffset;
        const size_t rowAvailable = std::min<size_t>(frame.stride, frame.size - rowOffset);
        uint16_t *dst = bayer->data() + (static_cast<size_t>(y) * frame.width);

        if (frame.packing == PixelPacking::Csi2Packed) {
            const size_t required = packedCsi2RowBytes(frame.width, bits);
            if (rowAvailable < required) {
                setStillError(error, "Packed raw row is shorter than expected.");
                return false;
            }
            if (bits == 10)
                Csi2RawConverter::unpack10Row(src, dst, frame.width);
            else if (bits == 12)
                Csi2RawConverter::unpack12Row(src, dst, frame.width);
            else {
                setStillError(error, "Packed raw bit depth is not supported for processed stills.");
                return false;
            }
        } else if (frame.packing == PixelPacking::Unpacked) {
            const size_t bytesPerSample = unpackedBytesPerSampleForFrame(frame);
            const size_t required = static_cast<size_t>(frame.width) * bytesPerSample;
            if (rowAvailable < required) {
                setStillError(error, "Unpacked raw row is shorter than expected.");
                return false;
            }
            if (bytesPerSample == 1u) {
                for (uint32_t x = 0; x < frame.width; ++x)
                    dst[x] = src[x];
            } else {
                const uint16_t validMask = bits >= 16
                    ? 0xffffu
                    : static_cast<uint16_t>((1u << static_cast<uint32_t>(bits)) - 1u);
                for (uint32_t x = 0; x < frame.width; ++x) {
                    const uint16_t sample = readLe16Sample(src + (static_cast<size_t>(x) * 2u));
                    dst[x] = bits == 8
                        ? static_cast<uint16_t>(sample >> 8)
                        : static_cast<uint16_t>(sample & validMask);
                }
            }
        } else if (frame.packing == PixelPacking::PispCompressed1) {
            bits = 12;
            const size_t required = ((static_cast<size_t>(frame.width) + 7u) / 8u) * 8u;
            if (rowAvailable < required) {
                setStillError(error, "PiSP compressed raw row is shorter than expected.");
                return false;
            }
            uint32_t x = 0;
            for (; x + 7u < frame.width; x += 8u, src += 8u)
                decodePispStillBlockTo12(src, dst + x, 8u);
            if (x < frame.width)
                decodePispStillBlockTo12(src, dst + x, frame.width - x);
        } else {
            setStillError(error, "Unsupported raw packing for processed still.");
            return false;
        }
    }

    *sampleBits = bits;
    return true;
}

int bayerColorAt(BayerPattern pattern, uint32_t x, uint32_t y)
{
    const bool evenX = (x & 1u) == 0;
    const bool evenY = (y & 1u) == 0;

    switch (pattern) {
    case BayerPattern::RGGB:
        if (evenY) return evenX ? 0 : 1;
        return evenX ? 1 : 2;
    case BayerPattern::GRBG:
        if (evenY) return evenX ? 1 : 0;
        return evenX ? 2 : 1;
    case BayerPattern::GBRG:
        if (evenY) return evenX ? 1 : 2;
        return evenX ? 0 : 1;
    case BayerPattern::BGGR:
        if (evenY) return evenX ? 2 : 1;
        return evenX ? 1 : 0;
    case BayerPattern::Monochrome:
        return 1;
    }
    return 0;
}

bool processRawFrameToRgbStill(const RawFrameView &frame,
                               const CdngEncoder::Metadata &metadata,
                               const StillRenderSettings &rendering,
                               ProcessedRgbStill *image,
                               std::string *error)
{
    std::vector<uint16_t> bayer;
    int bits = 12;
    if (!unpackRawFrameToBayer(frame, &bayer, &bits, error))
        return false;

    if (!image) {
        setStillError(error, "Processed still target is missing.");
        return false;
    }

    const uint32_t width = frame.width;
    const uint32_t height = frame.height;
    image->width = width;
    image->height = height;
    image->rgb.assign(static_cast<size_t>(width) * static_cast<size_t>(height) * 3u, 0);

    const float maxSample = bits >= 16 ? 65535.0f : static_cast<float>((1u << bits) - 1u);
    const auto blackIndices = blackLevelCfaIndices(frame.bayerPattern);
    std::array<float, 4> black{};
    std::array<float, 4> range{};
    for (size_t i = 0; i < black.size(); ++i) {
        black[i] = blackLevelForSampleBits(
            metadata.hasBlackLevels ? metadata.blackLevels[blackIndices[i]] : 0.0f,
            static_cast<unsigned>(bits), metadata.blackLevelsOnContainerScale);
        range[i] = std::max(1.0f, maxSample - black[i]);
    }
    const bool monochrome = frame.bayerPattern == BayerPattern::Monochrome;
    StillRenderSettings effectiveRendering = rendering;
    if (monochrome) effectiveRendering.ccm = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const StillColorTransform transform(effectiveRendering, monochrome ? 1.0f : metadata.colourGains[0],
                                         monochrome ? 1.0f : metadata.colourGains[1], metadata.digitalGain);

    auto sample = [&](int x, int y) -> float {
        x = std::clamp(x, 0, static_cast<int>(width) - 1);
        y = std::clamp(y, 0, static_cast<int>(height) - 1);
        const uint16_t raw = bayer[(static_cast<size_t>(y) * width) + static_cast<size_t>(x)];
        const unsigned channel = ((y & 1) << 1) | (x & 1);
        return std::clamp((static_cast<float>(raw) - black[channel]) / range[channel], 0.0f, 1.0f);
    };

    auto avg2 = [&](float a, float b) {
        return (a + b) * 0.5f;
    };
    auto avg4 = [&](float a, float b, float c, float d) {
        return (a + b + c + d) * 0.25f;
    };

    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            float r = 0.0f;
            float g = 0.0f;
            float b = 0.0f;
            const int color = bayerColorAt(frame.bayerPattern, x, y);

            if (monochrome) {
                r = g = b = sample(static_cast<int>(x), static_cast<int>(y));
            } else if (color == 0) {
                r = sample(static_cast<int>(x), static_cast<int>(y));
                g = avg4(sample(static_cast<int>(x) - 1, static_cast<int>(y)),
                         sample(static_cast<int>(x) + 1, static_cast<int>(y)),
                         sample(static_cast<int>(x), static_cast<int>(y) - 1),
                         sample(static_cast<int>(x), static_cast<int>(y) + 1));
                b = avg4(sample(static_cast<int>(x) - 1, static_cast<int>(y) - 1),
                         sample(static_cast<int>(x) + 1, static_cast<int>(y) - 1),
                         sample(static_cast<int>(x) - 1, static_cast<int>(y) + 1),
                         sample(static_cast<int>(x) + 1, static_cast<int>(y) + 1));
            } else if (color == 2) {
                b = sample(static_cast<int>(x), static_cast<int>(y));
                g = avg4(sample(static_cast<int>(x) - 1, static_cast<int>(y)),
                         sample(static_cast<int>(x) + 1, static_cast<int>(y)),
                         sample(static_cast<int>(x), static_cast<int>(y) - 1),
                         sample(static_cast<int>(x), static_cast<int>(y) + 1));
                r = avg4(sample(static_cast<int>(x) - 1, static_cast<int>(y) - 1),
                         sample(static_cast<int>(x) + 1, static_cast<int>(y) - 1),
                         sample(static_cast<int>(x) - 1, static_cast<int>(y) + 1),
                         sample(static_cast<int>(x) + 1, static_cast<int>(y) + 1));
            } else {
                g = sample(static_cast<int>(x), static_cast<int>(y));
                const bool horizontalRed =
                    bayerColorAt(frame.bayerPattern, x > 0 ? x - 1u : x, y) == 0 ||
                    bayerColorAt(frame.bayerPattern, x + 1u < width ? x + 1u : x, y) == 0;
                if (horizontalRed) {
                    r = avg2(sample(static_cast<int>(x) - 1, static_cast<int>(y)),
                             sample(static_cast<int>(x) + 1, static_cast<int>(y)));
                    b = avg2(sample(static_cast<int>(x), static_cast<int>(y) - 1),
                             sample(static_cast<int>(x), static_cast<int>(y) + 1));
                } else {
                    r = avg2(sample(static_cast<int>(x), static_cast<int>(y) - 1),
                             sample(static_cast<int>(x), static_cast<int>(y) + 1));
                    b = avg2(sample(static_cast<int>(x) - 1, static_cast<int>(y)),
                             sample(static_cast<int>(x) + 1, static_cast<int>(y)));
                }
            }

            const size_t out = ((static_cast<size_t>(y) * width) + x) * 3u;
            const auto rgb = transform.render(r, g, b);
            std::copy(rgb.begin(), rgb.end(), image->rgb.begin() + out);
        }
    }

    return image->valid();
}

bool prepare16BitDngStillFrame(const RawFrameView &sourceFrame,
                               const CdngEncoder::Metadata &sourceMetadata,
                               std::vector<uint8_t> *raw16Bytes,
                               RawFrameView *dngFrame,
                               CdngEncoder::Metadata *dngMetadata,
                               std::string *error)
{
    std::vector<uint16_t> bayer;
    int sourceBits = 12;
    if (!unpackRawFrameToBayer(sourceFrame, &bayer, &sourceBits, error))
        return false;

    if (!raw16Bytes || !dngFrame || !dngMetadata) {
        setStillError(error, "16-bit DNG still target is missing.");
        return false;
    }

    const uint32_t maxSource = sourceBits >= 16
        ? 65535u
        : ((1u << static_cast<uint32_t>(sourceBits)) - 1u);
    raw16Bytes->resize(bayer.size() * 2u);
    for (size_t i = 0; i < bayer.size(); ++i) {
        const uint32_t sample = sourceBits >= 16
            ? std::min<uint32_t>(bayer[i], 65535u)
            : ((static_cast<uint32_t>(bayer[i]) * 65535u) + (maxSource / 2u)) / maxSource;
        (*raw16Bytes)[(i * 2u) + 0u] = static_cast<uint8_t>(sample & 0xffu);
        (*raw16Bytes)[(i * 2u) + 1u] = static_cast<uint8_t>((sample >> 8) & 0xffu);
    }

    *dngFrame = sourceFrame;
    dngFrame->data = raw16Bytes->data();
    dngFrame->size = raw16Bytes->size();
    dngFrame->stride = sourceFrame.width * 2u;
    dngFrame->bitDepth = RawBitDepth::Bits16;
    dngFrame->unpackedBytesPerSample = 2u;
    dngFrame->packing = PixelPacking::Unpacked;

    *dngMetadata = sourceMetadata;
    if (dngMetadata->hasBlackLevels && maxSource > 0u) {
        for (float &blackLevel : dngMetadata->blackLevels) {
            const float sourceBlack = blackLevelForSampleBits(
                blackLevel, static_cast<unsigned>(sourceBits),
                sourceMetadata.blackLevelsOnContainerScale);
            const float scaled = sourceBlack * 65535.0f / static_cast<float>(maxSource);
            blackLevel = std::clamp(scaled, 0.0f, 65535.0f);
        }
    }
    dngMetadata->blackLevelsOnContainerScale = true;

    return true;
}

void embedStillJpegColorProfile(std::vector<uint8_t> &bytes)
{
    if (bytes.size() < 2 || bytes[0] != 0xff || bytes[1] != 0xd8)
        return;
    static_assert(sizeof(kStillSrgbProfile) + 16 <= 65535);
    std::vector<uint8_t> marker{0xff, 0xe2};
    const uint16_t length = sizeof(kStillSrgbProfile) + 16;
    marker.push_back(static_cast<uint8_t>(length >> 8));
    marker.push_back(static_cast<uint8_t>(length & 0xff));
    const uint8_t signature[] = {'I','C','C','_','P','R','O','F','I','L','E',0,1,1};
    marker.insert(marker.end(), std::begin(signature), std::end(signature));
    marker.insert(marker.end(), std::begin(kStillSrgbProfile), std::end(kStillSrgbProfile));
    bytes.insert(bytes.begin() + 2, marker.begin(), marker.end());
}

#if APERTARCORE_HAS_TURBOJPEG
bool encodeProcessedJpeg(const ProcessedRgbStill &image, std::vector<uint8_t> *jpegBytes)
{
    if (!image.valid() || !jpegBytes)
        return false;

    std::vector<uint8_t> rgb8(image.rgb.size());
    for (size_t i = 0; i < image.rgb.size(); ++i)
        rgb8[i] = static_cast<uint8_t>(image.rgb[i] >> 8);

    tjhandle compressor = tjInitCompress();
    if (!compressor)
        return false;

    unsigned char *compressed = nullptr;
    unsigned long compressedSize = 0;
    const int rc = tjCompress2(compressor,
                               rgb8.data(),
                               static_cast<int>(image.width),
                               0,
                               static_cast<int>(image.height),
                               TJPF_RGB,
                               &compressed,
                               &compressedSize,
                               TJSAMP_420,
                               kPhotoJpegQuality,
                               TJFLAG_FASTDCT);

    const bool ok = (rc == 0 && compressed && compressedSize > 0);
    if (ok)
        jpegBytes->assign(compressed, compressed + compressedSize);

    if (ok)
        embedStillJpegColorProfile(*jpegBytes);

    if (compressed)
        tjFree(compressed);
    tjDestroy(compressor);
    return ok;
}
#elif APERTARCORE_HAS_JPEG
bool encodeProcessedJpeg(const ProcessedRgbStill &image, std::vector<uint8_t> *jpegBytes)
{
    if (!image.valid() || !jpegBytes)
        return false;

    std::vector<uint8_t> rgb8(image.rgb.size());
    for (size_t i = 0; i < image.rgb.size(); ++i)
        rgb8[i] = static_cast<uint8_t>(image.rgb[i] >> 8);

    jpeg_compress_struct cinfo{};
    jpeg_error_mgr jerr{};
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    unsigned char *compressed = nullptr;
    unsigned long compressedSize = 0;
    jpeg_mem_dest(&cinfo, &compressed, &compressedSize);

    cinfo.image_width = image.width;
    cinfo.image_height = image.height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, kPhotoJpegQuality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    while (cinfo.next_scanline < cinfo.image_height) {
        JSAMPROW row = rgb8.data() + (static_cast<size_t>(cinfo.next_scanline) * image.width * 3u);
        jpeg_write_scanlines(&cinfo, &row, 1);
    }

    jpeg_finish_compress(&cinfo);
    const bool ok = compressed && compressedSize > 0;
    if (ok)
        jpegBytes->assign(compressed, compressed + compressedSize);
    if (ok)
        embedStillJpegColorProfile(*jpegBytes);
    jpeg_destroy_compress(&cinfo);
    std::free(compressed);
    return ok;
}
#else
bool encodeProcessedJpeg(const ProcessedRgbStill &, std::vector<uint8_t> *)
{
    return false;
}
#endif

bool encodeProcessedPng(const ProcessedRgbStill &image, std::vector<uint8_t> *pngBytes)
{
    if (!image.valid() || !pngBytes)
        return false;

    std::vector<uint8_t> scanlines;
    scanlines.reserve((static_cast<size_t>(image.width) * 6u + 1u) * image.height);
    for (uint32_t y = 0; y < image.height; ++y) {
        scanlines.push_back(0);
        const uint16_t *row = image.rgb.data() + (static_cast<size_t>(y) * image.width * 3u);
        for (uint32_t x = 0; x < image.width * 3u; ++x) {
            const uint16_t sample = row[x];
            scanlines.push_back(static_cast<uint8_t>(sample >> 8));
            scanlines.push_back(static_cast<uint8_t>(sample & 0xffu));
        }
    }

    pngBytes->clear();
    const uint8_t signature[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    pngBytes->insert(pngBytes->end(), std::begin(signature), std::end(signature));

    std::vector<uint8_t> ihdr;
    appendBe32(ihdr, image.width);
    appendBe32(ihdr, image.height);
    ihdr.push_back(16);
    ihdr.push_back(2);
    ihdr.push_back(0);
    ihdr.push_back(0);
    ihdr.push_back(0);
    appendPngChunk(*pngBytes, "IHDR", ihdr);
    appendPngChunk(*pngBytes, "sRGB", {0});
    std::vector<uint8_t> gamma;
    appendBe32(gamma, 45455);
    appendPngChunk(*pngBytes, "gAMA", gamma);

    std::vector<uint8_t> idat = zlibStoredStream(scanlines);
    appendPngChunk(*pngBytes, "IDAT", idat);
    appendPngChunk(*pngBytes, "IEND", {});
    return true;
}

bool encodeProcessedTiff(const ProcessedRgbStill &image, std::vector<uint8_t> *tiffBytes)
{
    if (!image.valid() || !tiffBytes)
        return false;

    constexpr uint16_t kTagCount = 11;
    const uint32_t ifdOffset = 8;
    const uint32_t bitsPerSampleOffset = ifdOffset + 2u + (kTagCount * 12u) + 4u;
    const uint32_t profileOffset = bitsPerSampleOffset + 6u;
    const uint32_t dataOffset = profileOffset + sizeof(kStillSrgbProfile);
    const size_t dataSize = image.rgb.size() * 2u;
    if (dataSize > std::numeric_limits<uint32_t>::max())
        return false;

    tiffBytes->clear();
    tiffBytes->reserve(dataOffset + dataSize);
    tiffBytes->push_back('I');
    tiffBytes->push_back('I');
    appendLe16(*tiffBytes, 42);
    appendLe32(*tiffBytes, ifdOffset);

    appendLe16(*tiffBytes, kTagCount);
    appendTiffEntry(*tiffBytes, 256, 4, 1, image.width);
    appendTiffEntry(*tiffBytes, 257, 4, 1, image.height);
    appendTiffEntry(*tiffBytes, 258, 3, 3, bitsPerSampleOffset);
    appendTiffEntry(*tiffBytes, 259, 3, 1, 1);
    appendTiffEntry(*tiffBytes, 262, 3, 1, 2);
    appendTiffEntry(*tiffBytes, 273, 4, 1, dataOffset);
    appendTiffEntry(*tiffBytes, 277, 3, 1, 3);
    appendTiffEntry(*tiffBytes, 278, 4, 1, image.height);
    appendTiffEntry(*tiffBytes, 279, 4, 1, static_cast<uint32_t>(dataSize));
    appendTiffEntry(*tiffBytes, 284, 3, 1, 1);
    appendTiffEntry(*tiffBytes, 34675, 7, sizeof(kStillSrgbProfile), profileOffset);
    appendLe32(*tiffBytes, 0);
    appendLe16(*tiffBytes, 16);
    appendLe16(*tiffBytes, 16);
    appendLe16(*tiffBytes, 16);
    tiffBytes->insert(tiffBytes->end(), std::begin(kStillSrgbProfile), std::end(kStillSrgbProfile));

    for (uint16_t sample : image.rgb)
        appendLe16(*tiffBytes, sample);
    return true;
}

enum class SensorFamily {
    Unknown,
    Imx294,
    Imx477,
    Imx577,
    Imx585
};

Resolution h264CaptureResolutionForSensor(SensorFamily sensorFamily)
{
    switch (sensorFamily) {
    case SensorFamily::Imx294:
        return Resolution{ 4144, 2176 };
    case SensorFamily::Imx585:
        return Resolution{ 3840, 2160 };
    case SensorFamily::Imx577:
        return Resolution{ 4056, 2160 };
    case SensorFamily::Imx477:
        return Resolution{ 2028, 1080 };
    case SensorFamily::Unknown:
        break;
    }

    return Resolution{ 3840, 2160 };
}

Resolution processedVideoCaptureResolutionForSensor(const CaptureSettings &settings,
                                                    SensorFamily sensorFamily)
{
    if (settings.recordingFormat == RecordingFormat::Prores)
        return settings.resolution;
    return h264CaptureResolutionForSensor(sensorFamily);
}

struct SensorModeSelection
{
    Resolution rawResolution{};
    libcamera::PixelFormat rawPixelFormat = libcamera::formats::SRGGB12_CSI2P;
    uint32_t bitDepth = 12;
    std::string dngModel;
};


bool controlInfoHas(const libcamera::ControlInfoMap &controlInfo, const libcamera::ControlId &control)
{
    return controlInfo.find(&control) != controlInfo.end();
}

bool intControlSupportsValue(const libcamera::ControlInfoMap &controlInfo,
                             const libcamera::Control<int32_t> &control,
                             int32_t value)
{
    const auto it = controlInfo.find(&control);
    if (it == controlInfo.end())
        return false;

    const std::vector<libcamera::ControlValue> &values = it->second.values();
    if (values.empty())
        return true;

    return std::any_of(values.begin(), values.end(), [value](const libcamera::ControlValue &candidate) {
        return !candidate.isNone() && candidate.get<int32_t>() == value;
    });
}

#if defined(LIBCAMERA_HAS_LIBCAMERA_VENDOR_CONTROLS_AF_MODE)
int32_t afModeControlValue(AutofocusMode mode)
{
    switch (mode) {
    case AutofocusMode::Manual: return libcamera::controls::AfModeManual;
    case AutofocusMode::Auto: return libcamera::controls::AfModeAuto;
    case AutofocusMode::Continuous: return libcamera::controls::AfModeContinuous;
    }
    return libcamera::controls::AfModeContinuous;
}

int32_t afRangeControlValue(AutofocusRange range)
{
    switch (range) {
    case AutofocusRange::Normal: return libcamera::controls::AfRangeNormal;
    case AutofocusRange::Macro: return libcamera::controls::AfRangeMacro;
    case AutofocusRange::Full: return libcamera::controls::AfRangeFull;
    }
    return libcamera::controls::AfRangeNormal;
}

int32_t afSpeedControlValue(AutofocusSpeed speed)
{
    switch (speed) {
    case AutofocusSpeed::Normal: return libcamera::controls::AfSpeedNormal;
    case AutofocusSpeed::Fast: return libcamera::controls::AfSpeedFast;
    }
    return libcamera::controls::AfSpeedNormal;
}

libcamera::Rectangle autofocusWindowRectangle(const CaptureSettings &settings, Resolution reference)
{
    const uint32_t width = std::max<uint32_t>(1, reference.width);
    const uint32_t height = std::max<uint32_t>(1, reference.height);
    const double x = std::clamp(settings.autofocusWindowX, 0.0, 1.0);
    const double y = std::clamp(settings.autofocusWindowY, 0.0, 1.0);
    const double w = std::clamp(settings.autofocusWindowWidth, 0.01, 1.0);
    const double h = std::clamp(settings.autofocusWindowHeight, 0.01, 1.0);

    const int pixelX = static_cast<int>(std::llround(x * width));
    const int pixelY = static_cast<int>(std::llround(y * height));
    const uint32_t pixelW = std::max<uint32_t>(1, static_cast<uint32_t>(std::llround(w * width)));
    const uint32_t pixelH = std::max<uint32_t>(1, static_cast<uint32_t>(std::llround(h * height)));
    return libcamera::Rectangle(pixelX, pixelY, pixelW, pixelH)
        .boundedTo(libcamera::Rectangle(0, 0, width, height));
}
#endif

constexpr uint32_t stillPreviewExposureLimitUs = 66'667;

bool needsLongStillExposure(const CaptureSettings &settings)
{
    return settings.photoModeEnabled && settings.longExposureEnabled && !settings.shutterAuto &&
           settings.shutterUs > stillPreviewExposureLimitUs;
}

std::pair<uint32_t, uint32_t> h264EncodeBounds(const Resolution &, const CaptureSettings &);

libcamera::ControlList controlsForSettings(const CaptureSettings &settings,
                                           SensorFamily sensorFamily,
                                           const libcamera::ControlInfoMap &controlInfo,
                                           Resolution configuredResolution,
                                    const libcamera::ControlList &cameraProperties,
                                    bool monochrome = false, bool stillCapture = false)
{
    libcamera::ControlList controls(controlInfo);

    if (activeIspColorProfile(settings) != ProresColorProfile::Standard) {
        if (controlInfoHas(controlInfo, libcamera::controls::Contrast)) controls.set(libcamera::controls::Contrast, 1.0f);
        if (controlInfoHas(controlInfo, libcamera::controls::Brightness)) controls.set(libcamera::controls::Brightness, 0.0f);
        if (controlInfoHas(controlInfo, libcamera::controls::Saturation)) controls.set(libcamera::controls::Saturation, 1.0f);
    }

    const bool limitedPreview = needsLongStillExposure(settings) && !stillCapture;
    const uint32_t exposureUs = limitedPreview ? stillPreviewExposureLimitUs : settings.shutterUs;
    const int64_t nominalFrameDurationUs = settings.fps > 0.0
        ? static_cast<int64_t>(1000000.0 / settings.fps)
        : 41666;
    const int64_t frameDurationUs = limitedPreview
        ? std::max(nominalFrameDurationUs, int64_t(exposureUs)) : nominalFrameDurationUs;
    const int64_t maximumFrameDurationUs = settings.photoModeEnabled && !settings.shutterAuto
        ? std::max(frameDurationUs, int64_t(exposureUs) + (limitedPreview ? 0 : 100000)) : frameDurationUs;
    const int64_t frameDurationRange[2] = { frameDurationUs, maximumFrameDurationUs };
    controls.set(libcamera::controls::FrameDurationLimits,
                 libcamera::Span<const int64_t, 2>(frameDurationRange));
    const bool autoExposure = settings.isoAuto || settings.shutterAuto;
    if (settings.exposureMeteringModes & exposureMeteringBit(settings.exposureMetering)) {
        const int32_t mode = settings.exposureMetering == "spot" ? libcamera::controls::MeteringSpot
            : settings.exposureMetering == "matrix" ? libcamera::controls::MeteringMatrix
            : libcamera::controls::MeteringCentreWeighted;
        if (intControlSupportsValue(controlInfo, libcamera::controls::AeMeteringMode, mode))
            controls.set(libcamera::controls::AeMeteringMode, mode);
    }
    if (const auto ev = controlInfo.find(&libcamera::controls::ExposureValue); ev != controlInfo.end()) {
        const float value = autoExposure ? static_cast<float>(settings.exposureCompensation) : 0.0f;
        controls.set(libcamera::controls::ExposureValue,
            std::clamp(value, ev->second.min().get<float>(), ev->second.max().get<float>()));
    }
    controls.set(libcamera::controls::AeEnable, autoExposure);
    controls.set(libcamera::controls::ExposureTimeMode,
                 settings.shutterAuto
                     ? libcamera::controls::ExposureTimeModeAuto
                     : libcamera::controls::ExposureTimeModeManual);
    controls.set(libcamera::controls::AnalogueGainMode,
                 settings.isoAuto
                     ? libcamera::controls::AnalogueGainModeAuto
                     : libcamera::controls::AnalogueGainModeManual);
    if (!settings.shutterAuto) {
        controls.set(libcamera::controls::ExposureTime,
                     static_cast<int32_t>(std::min<uint32_t>(exposureUs, 0x7fffffff)));
    }
    if (!settings.isoAuto) {
        controls.set(libcamera::controls::AnalogueGain,
                     std::max(1.0f, static_cast<float>(settings.iso) / 100.0f));
    }

    if (monochrome) {
        if (controlInfoHas(controlInfo, libcamera::controls::AwbEnable))
            controls.set(libcamera::controls::AwbEnable, false);
        if (controlInfoHas(controlInfo, libcamera::controls::ColourGains))
            controls.set(libcamera::controls::ColourGains,
                         libcamera::Span<const float, 2>({ 1.0f, 1.0f }));
    } else if (!settings.whiteBalanceAvailable) {
        // Leave uncalibrated/NoIR tuning's native WB behavior unchanged.
    } else if (settings.whiteBalanceAuto) {
        controls.set(libcamera::controls::AwbEnable, true);
    } else {
        controls.set(libcamera::controls::AwbEnable, false);
        // Release explicit gains/CCM, then select calibrated manual Kelvin.
        controls.set(libcamera::controls::ColourGains,
                     libcamera::Span<const float, 2>({ 0.0f, 0.0f }));
        controls.set(libcamera::controls::ColourTemperature,
                     static_cast<int32_t>(settings.whiteBalanceKelvin));
    }

    if (controlInfoHas(controlInfo, libcamera::controls::ScalerCrop)) {
        if (const auto maximum = cameraProperties.get(libcamera::properties::ScalerCropMaximum)) {
            const bool hardwareH264 = !settings.photoModeEnabled && settings.recordingFormat == RecordingFormat::H264 &&
                recordingCapabilities().hardwareH264;
            Resolution cropResolution = settings.resolution;
            if (hardwareH264) {
                const auto [width, height] = h264EncodeBounds(settings.resolution, settings);
                cropResolution = {width, height};
            }
            const auto cropFunction = sensorFamily == SensorFamily::Imx294 && !hardwareH264
                ? centeredRawWindowScalerCrop : centeredPreviewScalerCrop;
            if (const auto crop = cropFunction(
                    configuredResolution.width, configuredResolution.height,
                    cropResolution.width, cropResolution.height,
                    {maximum->x, maximum->y, maximum->width, maximum->height})) {
                controls.set(libcamera::controls::ScalerCrop,
                    libcamera::Rectangle(crop->x, crop->y, crop->width, crop->height).boundedTo(*maximum));
            }
        }
        // Without native bounds, leave the pipeline's default crop alone.
    }

#if defined(LIBCAMERA_HAS_LIBCAMERA_VENDOR_CONTROLS_AF_MODE)
    const int32_t afMode = afModeControlValue(settings.autofocusMode);
    if (intControlSupportsValue(controlInfo, libcamera::controls::AfMode, afMode))
        controls.set(libcamera::controls::AfMode, afMode);

    const int32_t afRange = afRangeControlValue(settings.autofocusRange);
    if (intControlSupportsValue(controlInfo, libcamera::controls::AfRange, afRange))
        controls.set(libcamera::controls::AfRange, afRange);

    const int32_t afSpeed = afSpeedControlValue(settings.autofocusSpeed);
    if (intControlSupportsValue(controlInfo, libcamera::controls::AfSpeed, afSpeed))
        controls.set(libcamera::controls::AfSpeed, afSpeed);

    if (controlInfoHas(controlInfo, libcamera::controls::AfMetering) &&
        intControlSupportsValue(controlInfo,
                                libcamera::controls::AfMetering,
                                libcamera::controls::AfMeteringWindows)) {
        controls.set(libcamera::controls::AfMetering, libcamera::controls::AfMeteringWindows);
    }

    if (controlInfoHas(controlInfo, libcamera::controls::AfWindows)) {
        const std::array<libcamera::Rectangle, 1> windows{{
            autofocusWindowRectangle(settings, configuredResolution),
        }};
        controls.set(libcamera::controls::AfWindows,
                     libcamera::Span<const libcamera::Rectangle>(windows));
    }

    if (settings.autofocusMode == AutofocusMode::Auto &&
        controlInfoHas(controlInfo, libcamera::controls::AfTrigger) &&
        intControlSupportsValue(controlInfo,
                                libcamera::controls::AfTrigger,
                                libcamera::controls::AfTriggerStart)) {
        controls.set(libcamera::controls::AfTrigger, libcamera::controls::AfTriggerStart);
    }
#else
    (void)configuredResolution;
#endif

    return controls;
}

std::string cameraIdForLog(const std::shared_ptr<libcamera::Camera> &camera)
{
    return camera ? camera->id() : std::string("<none>");
}

SensorFamily detectSensorFamily(const std::string &cameraId)
{
    std::string lower = cameraId;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });

    if (lower.find("imx577") != std::string::npos)
        return SensorFamily::Imx577;
    if (lower.find("imx477") != std::string::npos)
        return SensorFamily::Imx477;
    if (lower.find("imx294") != std::string::npos)
        return SensorFamily::Imx294;
    if (lower.find("imx585") != std::string::npos)
        return SensorFamily::Imx585;
    return SensorFamily::Unknown;
}

const char *sensorFamilyName(SensorFamily sensorFamily)
{
    switch (sensorFamily) {
    case SensorFamily::Imx294: return "IMX294";
    case SensorFamily::Imx477: return "IMX477";
    case SensorFamily::Imx577: return "IMX577";
    case SensorFamily::Imx585: return "IMX585";
    case SensorFamily::Unknown: return "Unknown";
    }
    return "Unknown";
}

libcamera::PixelFormat rawPixelFormatForBitDepth(uint32_t bitDepth)
{
    switch (normalizeRawBitDepth(bitDepth)) {
    case 8:
        return libcamera::formats::SRGGB8;
    case 10:
        return libcamera::formats::SRGGB10_CSI2P;
    default:
        return libcamera::formats::SRGGB12_CSI2P;
    }
}

Resolution imx294ReadoutForActiveResolution(Resolution active, uint32_t bitDepth)
{
    active = normalizedImx294ResolutionForBitDepth(active, bitDepth);
    // Derived windows have incomplete vertical readouts in this driver.
    // Use the original readout and crop its valid pixels in RAW and the ISP.
    if (active.width == 3792 || active.width == 3392)
        return Resolution{3792, 2824};
    if (active.width == 4096 || active.width == 3872)
        return Resolution{normalizeRawBitDepth(bitDepth) == 10 ? 4144u : 4176u, 2176};
    return active;
}

SensorModeSelection sensorModeForRequestedResolution(SensorFamily sensorFamily,
                                                     Resolution requested,
                                                     uint32_t rawBitDepth = 12)
{
    SensorModeSelection selection;
    selection.rawResolution = requested;
    selection.rawPixelFormat = rawPixelFormatForBitDepth(rawBitDepth);
    selection.bitDepth = normalizeRawBitDepth(rawBitDepth);
    selection.dngModel = "Generic";

    switch (sensorFamily) {
    case SensorFamily::Imx294:
        selection.rawResolution = imx294ReadoutForActiveResolution(requested, selection.bitDepth);
        selection.dngModel = "IMX294";
        break;
    case SensorFamily::Imx477:
        selection.rawResolution = normalizedImx477Resolution(requested);
        selection.dngModel = "IMX477";
        break;
    case SensorFamily::Imx577:
        if (requested.width == 1332 && requested.height == 990) {
            selection.rawResolution = Resolution{ 1332, 990 };
        } else if ((requested.width == 2028 && requested.height == 1080) ||
                   isImx585LowResolutionAlias(requested)) {
            selection.rawResolution = Resolution{ 2028, 1080 };
        } else if (requested.width == 2028 && requested.height == 1520) {
            selection.rawResolution = Resolution{ 2028, 1520 };
        } else if ((requested.width == 4056 && requested.height == 3040) ||
                   (requested.width == 4056 && requested.height == 2160) ||
                   isImx585HighResolutionAlias(requested)) {
            selection.rawResolution = isImx585HighResolutionAlias(requested)
                ? Resolution{ 4056, 2160 }
                : requested;
        } else {
            selection.rawResolution = Resolution{ 2028, 1080 };
        }
        selection.bitDepth = 12;
        selection.dngModel = "IMX577";
        break;
    case SensorFamily::Imx585:
        if (requested.width == 1332 && requested.height == 990) {
            selection.rawResolution = Resolution{ 1920, 1080 };
        } else if ((requested.width == 2028 && requested.height == 1080) ||
                   (requested.width == 2028 && requested.height == 1520) ||
                   isImx585LowResolutionAlias(requested)) {
            selection.rawResolution = Resolution{ 1920, 1080 };
        } else if (isImx585FullHdCropResolution(requested)) {
            selection.rawResolution = requested;
        } else if ((requested.width == 4056 && requested.height == 2160) ||
                   (requested.width == 4056 && requested.height == 3040) ||
                   isImx585HighResolutionAlias(requested)) {
            selection.rawResolution = Resolution{ 3840, 2160 };
        } else if (isImx585Raw12CropResolution(requested)) {
            selection.rawResolution = requested;
        } else {
            selection.rawResolution = Resolution{ 1920, 1080 };
        }
        selection.dngModel = "IMX585-AAQJ1";
        break;
    case SensorFamily::Unknown:
        break;
    }

    return selection;
}

SensorModeSelection imx577ModeSelection(Resolution resolution, uint32_t bitDepth)
{
    SensorModeSelection selection;
    selection.rawResolution = resolution;
    selection.rawPixelFormat = rawPixelFormatForBitDepth(bitDepth);
    selection.bitDepth = normalizeRawBitDepth(bitDepth);
    selection.dngModel = "IMX577";
    return selection;
}

bool sameSensorModeSelection(const SensorModeSelection &left, const SensorModeSelection &right)
{
    return left.rawResolution.width == right.rawResolution.width &&
           left.rawResolution.height == right.rawResolution.height &&
           left.rawPixelFormat == right.rawPixelFormat &&
           left.bitDepth == right.bitDepth;
}

std::vector<SensorModeSelection> sensorModeCandidatesForRequestedResolution(SensorFamily sensorFamily,
                                                                            Resolution requested,
                                                                            uint32_t rawBitDepth = 12)
{
    std::vector<SensorModeSelection> candidates;
    auto addCandidate = [&candidates](const SensorModeSelection &candidate) {
        const auto existing = std::find_if(candidates.begin(), candidates.end(),
                                           [&candidate](const SensorModeSelection &item) {
                                               return sameSensorModeSelection(item, candidate);
                                           });
        if (existing == candidates.end())
            candidates.push_back(candidate);
    };

    addCandidate(sensorModeForRequestedResolution(sensorFamily, requested, rawBitDepth));

    if (sensorFamily == SensorFamily::Imx294)
        return candidates;

    if (sensorFamily == SensorFamily::Imx477) {
        static constexpr std::array<Resolution, 25> kImx477Modes{{
            { 2028, 1080 },
            { 2028, 1140 },
            { 2028, 1096 },
            { 2028, 1068 },
            { 2028, 1014 },
            { 2028, 922 },
            { 2028, 914 },
            { 2028, 864 },
            { 2028, 848 },
            { 2028, 812 },
            { 2028, 796 },
            { 2028, 1520 },
            { 1332, 990 },
            { 4056, 2160 },
            { 4056, 2280 },
            { 4056, 2192 },
            { 4056, 2136 },
            { 4056, 2028 },
            { 4056, 1844 },
            { 4056, 1828 },
            { 4056, 1728 },
            { 4056, 1696 },
            { 4056, 1624 },
            { 4056, 1592 },
            { 4056, 3040 },
        }};
        for (const Resolution &resolution : kImx477Modes) {
            if (resolution.width == requested.width && resolution.height >= requested.height)
                addCandidate(sensorModeForRequestedResolution(sensorFamily, resolution, rawBitDepth));
        }
        for (const Resolution &resolution : kImx477Modes) {
            if (resolution.width == requested.width && resolution.height < requested.height)
                addCandidate(sensorModeForRequestedResolution(sensorFamily, resolution, rawBitDepth));
        }
        for (const Resolution &resolution : kImx477Modes) {
            if (resolution.width != requested.width)
                addCandidate(sensorModeForRequestedResolution(sensorFamily, resolution, rawBitDepth));
        }
        return candidates;
    }

    if (sensorFamily != SensorFamily::Imx577)
        return candidates;

    static constexpr std::array<Resolution, 5> kImx577FallbackModes{{
        { 2028, 1080 },
        { 1332, 990 },
        { 2028, 1520 },
        { 4056, 2160 },
        { 4056, 3040 },
    }};

    for (const Resolution &resolution : kImx577FallbackModes)
        addCandidate(imx577ModeSelection(resolution, 12));
    for (const Resolution &resolution : kImx577FallbackModes)
        addCandidate(imx577ModeSelection(resolution, 10));
    for (const Resolution &resolution : kImx577FallbackModes)
        addCandidate(imx577ModeSelection(resolution, 8));

    return candidates;
}

Resolution normalizedResolutionForSensor(SensorFamily sensorFamily, Resolution requested)
{
    switch (sensorFamily) {
    case SensorFamily::Imx294:
        if (isImx294Resolution(requested))
            return requested;
        if (isImx585HighResolutionAlias(requested) ||
            resolutionMatches(requested, 4056, 2160))
            return Resolution{ 4144, 2176 };
        if (resolutionMatches(requested, 4056, 3040))
            return Resolution{ 3792, 2824 };
        if (isImx585LowResolutionAlias(requested) ||
            resolutionMatches(requested, 2028, 1080) ||
            resolutionMatches(requested, 2028, 1520))
            return Resolution{ 4144, 2176 };
        return Resolution{ 4144, 2176 };
    case SensorFamily::Imx477:
        return normalizedImx477Resolution(requested);
    case SensorFamily::Imx577:
        if (requested.width == 1332 && requested.height == 990)
            return requested;
        if ((requested.width == 2028 && requested.height == 1080) ||
            isImx585LowResolutionAlias(requested))
            return Resolution{ 2028, 1080 };
        if (requested.width == 2028 && requested.height == 1520)
            return requested;
        if ((requested.width == 4056 && requested.height == 2160) ||
            isImx585HighResolutionAlias(requested))
            return Resolution{ 4056, 2160 };
        if (requested.width == 4056 && requested.height == 3040)
            return requested;
        return Resolution{ 2028, 1080 };
    case SensorFamily::Imx585:
        if ((requested.width == 1332 && requested.height == 990) ||
            (requested.width == 2028 && requested.height == 1080) ||
            (requested.width == 2028 && requested.height == 1520) ||
            isImx585LowResolutionAlias(requested))
            return Resolution{ 1920, 1080 };
        if (isImx585FullHdCropResolution(requested))
            return requested;
        if (isImx585HighResolutionAlias(requested))
            return Resolution{ 3840, 2160 };
        if (isImx585Raw12CropResolution(requested))
            return requested;
        return Resolution{ 1920, 1080 };
    case SensorFamily::Unknown:
        return requested;
    }

    return requested;
}

bool isFourKCaptureResolution(SensorFamily sensorFamily, Resolution requested)
{
    const Resolution resolution = sensorFamily == SensorFamily::Unknown
        ? requested
        : normalizedResolutionForSensor(sensorFamily, requested);

    switch (sensorFamily) {
    case SensorFamily::Imx477:
    case SensorFamily::Imx577:
        return resolution.width == 4056;
    case SensorFamily::Imx585:
        return resolution.width == 3840;
    case SensorFamily::Imx294:
        return resolution.width >= 3392;
    case SensorFamily::Unknown:
        return resolution.width >= 3840;
    }

    return false;
}

bool isApproximatelyFourThree(Resolution resolution)
{
    const int64_t widthTerm = static_cast<int64_t>(resolution.width) * 3;
    const int64_t heightTerm = static_cast<int64_t>(resolution.height) * 4;
    const int64_t tolerance = std::max<int64_t>(8, std::max(widthTerm, heightTerm) / 50);
    return std::abs(widthTerm - heightTerm) <= tolerance;
}

bool isLosslessFourKCapture(const CaptureSettings &settings, SensorFamily sensorFamily)
{
    return settings.recordingFormat == RecordingFormat::Cdng &&
           settings.cdngCompression == CdngCompression::LosslessJpeg &&
           isFourKCaptureResolution(sensorFamily, settings.resolution);
}

bool isLosslessTwoKCapture(const CaptureSettings &settings, SensorFamily sensorFamily)
{
    const Resolution resolution = sensorFamily == SensorFamily::Unknown
        ? settings.resolution
        : normalizedResolutionForSensor(sensorFamily, settings.resolution);
    return settings.recordingFormat == RecordingFormat::Cdng &&
           settings.cdngCompression == CdngCompression::LosslessJpeg &&
           resolution.width == 2028;
}

void enforceLosslessCaptureLimits(CaptureSettings &settings, SensorFamily sensorFamily)
{
    if (settings.photoModeEnabled || sensorFamily == SensorFamily::Imx294)
        return;

    const bool imx585 = sensorFamily == SensorFamily::Imx585 &&
                        settings.recordingFormat == RecordingFormat::Cdng &&
                        settings.cdngCompression == CdngCompression::LosslessJpeg;
    const bool fourK = isLosslessFourKCapture(settings, sensorFamily);
    const bool twoK = isLosslessTwoKCapture(settings, sensorFamily);
    if (!imx585 && !fourK && !twoK)
        return;

    settings.iso = std::min<uint32_t>(settings.iso, 800u);
    settings.isoAuto = false;
    const double maximumFps = imx585
        ? 30.0
        : (twoK
        ? 60.0
        : (isApproximatelyFourThree(settings.resolution) ? 24.0 : 30.0));
    settings.fps = std::min(settings.fps, maximumFps);
}

double imx477MaximumSupportedFpsForResolution(Resolution resolution, uint32_t rawBitDepth)
{
    const uint32_t bitDepth = normalizeRawBitDepth(rawBitDepth);

    if (resolution.width == 1332 && resolution.height == 990) {
        if (bitDepth == 8)
            return 282.17;
        if (bitDepth == 10)
            return 233.92;
        return 136.43;
    }

    if (resolution.width == 2028) {
        switch (resolution.height) {
        case 796:
            return bitDepth == 8 ? 242.48 : (bitDepth == 10 ? 198.85 : 167.79);
        case 812:
            return bitDepth == 8 ? 237.98 : (bitDepth == 10 ? 195.12 : 164.66);
        case 848:
            return bitDepth == 8 ? 228.41 : (bitDepth == 10 ? 187.30 : 158.05);
        case 864:
            return bitDepth == 8 ? 224.42 : (bitDepth == 10 ? 183.99 : 155.28);
        case 914:
            return bitDepth == 8 ? 212.72 : (bitDepth == 10 ? 174.43 : 147.21);
        case 922:
            return bitDepth == 8 ? 210.97 : (bitDepth == 10 ? 173.01 : 145.99);
        case 1014:
            return bitDepth == 8 ? 192.72 : (bitDepth == 10 ? 158.00 : 133.35);
        case 1068:
            return bitDepth == 8 ? 183.39 : (bitDepth == 10 ? 150.38 : 126.89);
        case 1080:
            return bitDepth == 8 ? 181.42 : (bitDepth == 10 ? 148.77 : 125.55);
        case 1096:
            return bitDepth == 8 ? 178.89 : (bitDepth == 10 ? 146.69 : 123.79);
        case 1140:
            return bitDepth == 8 ? 172.27 : (bitDepth == 10 ? 141.26 : 119.20);
        case 1520:
            return bitDepth == 8 ? 130.51 : (bitDepth == 10 ? 107.02 : 90.31);
        default:
            break;
        }
    }

    if (resolution.width == 4056) {
        switch (resolution.height) {
        case 1592:
            return bitDepth == 8 ? 66.44 : (bitDepth == 10 ? 53.84 : 45.26);
        case 1624:
            return bitDepth == 8 ? 65.16 : (bitDepth == 10 ? 52.81 : 44.40);
        case 1696:
            return bitDepth == 8 ? 62.47 : (bitDepth == 10 ? 50.63 : 42.56);
        case 1728:
            return bitDepth == 8 ? 61.35 : (bitDepth == 10 ? 50.00 : 41.80);
        case 1828:
            return bitDepth == 8 ? 58.08 : (bitDepth == 10 ? 47.07 : 39.57);
        case 1844:
            return bitDepth == 8 ? 57.59 : (bitDepth == 10 ? 46.67 : 39.24);
        case 2028:
            return bitDepth == 8 ? 52.48 : (bitDepth == 10 ? 42.54 : 35.76);
        case 2136:
            return bitDepth == 8 ? 49.89 : (bitDepth == 10 ? 40.43 : 33.99);
        case 2160:
            return bitDepth == 8 ? 49.34 : (bitDepth == 10 ? 40.00 : 33.62);
        case 2192:
            return bitDepth == 8 ? 48.64 : (bitDepth == 10 ? 39.42 : 33.14);
        case 2280:
            return bitDepth == 8 ? 46.80 : (bitDepth == 10 ? 37.93 : 31.89);
        case 3040:
            return bitDepth == 8 ? 35.28 : (bitDepth == 10 ? 28.60 : 24.04);
        default:
            break;
        }
    }

    return 120.0;
}

double imx477MaximumUncompressedCdngFpsForResolution(Resolution resolution, uint32_t rawBitDepth)
{
    const uint32_t bitDepth = normalizeRawBitDepth(rawBitDepth);

    if (resolution.width == 2028) {
        switch (resolution.height) {
        case 796:
            return bitDepth == 8 ? 140.0 : 120.0;
        case 812:
            return bitDepth == 8 ? 140.0 : 120.0;
        case 848:
            return bitDepth == 8 ? 140.0 : 120.0;
        case 864:
            return 120.0;
        case 914:
            return bitDepth == 10 ? 130.0 : 120.0;
        case 922:
            return bitDepth == 10 ? 130.0 : 120.0;
        case 1014:
            return 100.0;
        case 1068:
            return 100.0;
        case 1080:
            return 100.0;
        case 1096:
            return 90.0;
        case 1140:
            return bitDepth == 12 ? 60.0 : 90.0;
        case 1520:
            return bitDepth == 12 ? 60.0 : 90.0;
        default:
            break;
        }
    }

    if (resolution.width == 4056) {
        switch (resolution.height) {
        case 1592:
        case 1624:
        case 1696:
        case 1728:
        case 1828:
        case 1844:
        case 2028:
        case 2136:
        case 2160:
        case 2192:
        case 2280:
            return 30.0;
        case 3040:
            return bitDepth == 8 ? 25.0 : (bitDepth == 10 ? 24.0 : 24.0);
        default:
            break;
        }
    }

    return 1000.0;
}

double maximumSupportedFpsForResolution(SensorFamily sensorFamily, Resolution requested, uint32_t rawBitDepth = 12)
{
    const Resolution normalized = normalizedResolutionForSensor(sensorFamily, requested);

    switch (sensorFamily) {
    case SensorFamily::Imx294:
        if (normalized.width == 3792 && normalized.height == 2824)
            return 34.70;
        if ((normalized.width == 3792 && normalized.height == 2528) ||
            (normalized.width == 3392 && normalized.height == 2824))
            return 34.70;
        if (normalized.width == 4176 && normalized.height == 2176)
            return 40.95;
        if ((normalized.width == 4096 && normalized.height == 2048) ||
            (normalized.width == 4096 && normalized.height == 1844) ||
            (normalized.width == 4096 && normalized.height == 1744) ||
            (normalized.width == 4096 && normalized.height == 1716) ||
            (normalized.width == 4096 && normalized.height == 1640) ||
            (normalized.width == 2048 && normalized.height == 1080) ||
            (normalized.width == 1920 && normalized.height == 1080))
            return 40.95;
        if (normalized.width == 3872 && normalized.height == 2176)
            return normalizeRawBitDepth(rawBitDepth) == 10 ? 41.08 : 40.95;
        if (normalized.width == 4144 && normalized.height == 2176)
            return 41.08;
        break;
    case SensorFamily::Imx477:
        return imx477MaximumSupportedFpsForResolution(normalized, rawBitDepth);
    case SensorFamily::Imx577:
        if (normalized.width == 1332 && normalized.height == 990)
            return 101.68;
        if (normalized.width == 2028 && normalized.height == 1080)
            return 62.81;
        if (normalized.width == 2028 && normalized.height == 1520)
            return 45.19;
        if (normalized.width == 4056 && normalized.height == 2160)
            return 16.39;
        if (normalized.width == 4056 && normalized.height == 3040)
            return 11.72;
        break;
    case SensorFamily::Imx585:
        if (normalized.width == 3840 && normalized.height == 2160)
            return 30.0;
        if (isImx585Raw12CropResolution(normalized))
            return 60.0;
        if ((normalized.width == 1920 && normalized.height == 1080) ||
            isImx585FullHdCropResolution(normalized))
            return 60.0;
        break;
    case SensorFamily::Unknown:
        break;
    }

    return 120.0;
}

uint32_t supportedRawBitDepthForSensor(SensorFamily sensorFamily, uint32_t requestedBitDepth)
{
    const uint32_t bitDepth = normalizeRawBitDepth(requestedBitDepth);

    switch (sensorFamily) {
    case SensorFamily::Imx477:
    case SensorFamily::Imx577:
        if (bitDepth == 8 || bitDepth == 10 || bitDepth == 12)
            return bitDepth;
        break;
    case SensorFamily::Imx294:
    case SensorFamily::Imx585:
        if (bitDepth == 10 || bitDepth == 12)
            return bitDepth;
        break;
    case SensorFamily::Unknown:
        return bitDepth;
    }

    return 12u;
}

CaptureSettings sanitizeSettingsForSensor(CaptureSettings settings, SensorFamily sensorFamily)
{
    settings = supportedRecordingSettings(settings);
    settings = clampCaptureSettings(settings);
    if (!settings.photoModeEnabled && isProcessedVideoFormat(settings.recordingFormat)) {
        settings.resolution = processedVideoCaptureResolutionForSensor(settings, sensorFamily);
    }
    if (!settings.photoModeEnabled && isProcessedVideoFormat(settings.recordingFormat)) {
        settings.rawBitDepth = 10;
    }

    if (sensorFamily == SensorFamily::Unknown) {
        enforceLosslessCaptureLimits(settings, sensorFamily);
        return settings;
    }

    settings.resolution = normalizedResolutionForSensor(sensorFamily, settings.resolution);
    if (!settings.photoModeEnabled && isProcessedVideoFormat(settings.recordingFormat))
        settings.rawBitDepth = 10;
    else
        settings.rawBitDepth = supportedRawBitDepthForSensor(sensorFamily, settings.rawBitDepth);

    if (sensorFamily == SensorFamily::Imx294)
        settings.resolution = normalizedImx294ResolutionForBitDepth(settings.resolution, settings.rawBitDepth);

    double maxFps = maximumSupportedFpsForResolution(sensorFamily, settings.resolution, settings.rawBitDepth);
    if (!settings.photoModeEnabled && sensorFamily == SensorFamily::Imx477 &&
        settings.recordingFormat == RecordingFormat::Cdng &&
        settings.cdngCompression == CdngCompression::Uncompressed) {
        maxFps = std::min(
            maxFps,
            imx477MaximumUncompressedCdngFpsForResolution(settings.resolution, settings.rawBitDepth));
    }
    if (settings.fps > maxFps)
        settings.fps = maxFps;
    enforceLosslessCaptureLimits(settings, sensorFamily);
    return settings;
}

bool parseSizeEnv(const char *value, uint32_t &width, uint32_t &height)
{
    if (!value || !*value)
        return false;

    char *end = nullptr;
    const unsigned long parsedWidth = std::strtoul(value, &end, 10);
    if (!end || (*end != 'x' && *end != 'X'))
        return false;

    const unsigned long parsedHeight = std::strtoul(end + 1, &end, 10);
    if (!end || *end != '\0')
        return false;

    if (parsedWidth < 64 || parsedHeight < 64 || parsedWidth > 7680 || parsedHeight > 4320)
        return false;

    width = static_cast<uint32_t>(parsedWidth) & ~1u;
    height = static_cast<uint32_t>(parsedHeight) & ~1u;
    return width >= 2 && height >= 2;
}

std::pair<uint32_t, uint32_t> h264EncodeBounds(const Resolution &,
                                               const CaptureSettings &settings)
{
    if (settings.recordingFormat == RecordingFormat::Prores)
        return {settings.resolution.width, settings.resolution.height};
    uint32_t width = recordingCapabilities().hardwareH264 ? 1920 : kDefaultH264EncodeWidth;
    uint32_t height = recordingCapabilities().hardwareH264 ? 1080 : kDefaultH264EncodeHeight;
    if (settings.h264Resolution.width > 0 && settings.h264Resolution.height > 0) {
        width = settings.h264Resolution.width;
        height = settings.h264Resolution.height;
    }
    const char *envSizeName = "APERTAR_H264_SIZE";
    const char *envSize = std::getenv(envSizeName);
    if (envSize) {
        uint32_t envWidth = width;
        uint32_t envHeight = height;
        if (parseSizeEnv(envSize, envWidth, envHeight)) {
            if (isSupportedProcessedVideoResolution(settings.recordingFormat, {envWidth, envHeight})) {
                width = envWidth;
                height = envHeight;
            } else {
                std::cerr << "ApertarCore ignoring unsupported " << envSizeName << "="
                          << envSize << "; "
                          << "H.264"
                          << " output resolution is not supported.\n";
            }
        }
    }
    return {width, height};
}

libcamera::Size boundedAspectSize(const Resolution &sensorMode, uint32_t maxWidth, uint32_t maxHeight)
{
    uint32_t width = std::min(maxWidth, sensorMode.width);
    width &= ~1u;
    if (width < 2)
        width = std::min(2u, sensorMode.width);

    uint32_t height = sensorMode.height;
    if (sensorMode.width > 0) {
        const uint64_t scaledHeight = static_cast<uint64_t>(width) * sensorMode.height / sensorMode.width;
        height = static_cast<uint32_t>(std::max<uint64_t>(2, scaledHeight));
    }
    height = std::min(height, std::min(maxHeight, sensorMode.height));
    height &= ~1u;
    if (height < 2)
        height = std::min(2u, sensorMode.height);

    return libcamera::Size(width, height);
}

libcamera::Size previewSizeForSensorMode(const Resolution &sensorMode)
{
    auto size = boundedAspectSize(sensorMode, kPreviewWidth, kPreviewHeight);
    if (sensorMode.width > 0 && sensorMode.height > 0) {
        // Height-limited modes must also shrink in width, not turn a 4:3
        // sensor image into a 16:9 ISP stream that the UI then squeezes.
        const uint64_t widthForHeight = uint64_t(size.height) * sensorMode.width / sensorMode.height;
        size.width = std::min(size.width, static_cast<uint32_t>(widthForHeight)) & ~1u;
        size.width = std::max(2u, size.width);
    }
    return size;
}

libcamera::Size h264EncodeSizeForSensorMode(const Resolution &sensorMode,
                                            const CaptureSettings &settings)
{
    const auto [maxWidth, maxHeight] = h264EncodeBounds(sensorMode, settings);
    if (settings.recordingFormat == RecordingFormat::H264 && recordingCapabilities().hardwareH264)
        return libcamera::Size(maxWidth, maxHeight);
    return boundedAspectSize(sensorMode, maxWidth, maxHeight);
}

bool h264KeepRawStream()
{
    return boolEnvValue("APERTAR_H264_KEEP_RAW_STREAM", false);
}

bool h264SingleYuvStream()
{
    return boolEnvValue("APERTAR_H264_SINGLE_STREAM", false);
}

bool shouldUseRequestedActiveRawCrop(SensorFamily sensorFamily,
                                     const CaptureSettings &settings,
                                     uint32_t configuredWidth,
                                     uint32_t configuredHeight)
{
    if (sensorFamily != SensorFamily::Imx477 && sensorFamily != SensorFamily::Imx577 &&
        sensorFamily != SensorFamily::Imx294)
        return false;
    if (settings.resolution.width == 0 || settings.resolution.height == 0)
        return false;
    if (settings.resolution.width > configuredWidth ||
        (sensorFamily != SensorFamily::Imx294 && settings.resolution.width != configuredWidth))
        return false;
    if (settings.resolution.height > configuredHeight)
        return false;
    return settings.resolution.height != configuredHeight || settings.resolution.width != configuredWidth;
}

uint32_t activeRawWidthForConfiguredStream(SensorFamily sensorFamily,
                                          const CaptureSettings &settings,
                                          uint32_t configuredWidth,
                                          uint32_t configuredHeight)
{
    return shouldUseRequestedActiveRawCrop(sensorFamily, settings, configuredWidth, configuredHeight)
        ? settings.resolution.width : configuredWidth;
}

uint32_t activeRawLeftPixelsForConfiguredStream(SensorFamily sensorFamily,
                                               const CaptureSettings &settings,
                                               uint32_t configuredWidth,
                                               uint32_t configuredHeight)
{
    if (shouldUseRequestedActiveRawCrop(sensorFamily, settings, configuredWidth, configuredHeight))
        return ((configuredWidth - settings.resolution.width) / 2u) & ~1u;
    return 0;
}

size_t activeRawLeftBytes(uint32_t pixels, PixelPacking packing,
                          RawBitDepth transportBits, uint8_t unpackedBytes)
{
    if (packing == PixelPacking::PispCompressed1) {
        if (pixels % 8u != 0)
            throw std::runtime_error("RAW crop must align to a PiSP compression block");
        return pixels;
    }
    if (packing == PixelPacking::Unpacked)
        return static_cast<size_t>(pixels) * unpackedBytes;
    const unsigned bits = static_cast<unsigned>(transportBits);
    if ((static_cast<size_t>(pixels) * bits) % 8u != 0)
        throw std::runtime_error("RAW crop must align to a CSI2 packing group");
    return static_cast<size_t>(pixels) * bits / 8u;
}

uint32_t activeRawHeightForConfiguredStream(SensorFamily sensorFamily,
                                            const CaptureSettings &settings,
                                            uint32_t configuredWidth,
                                            uint32_t configuredHeight)
{
    if (shouldUseRequestedActiveRawCrop(sensorFamily, settings, configuredWidth, configuredHeight))
        return settings.resolution.height & ~1u;
    return configuredHeight;
}

uint32_t activeRawTopRowsForConfiguredStream(SensorFamily sensorFamily,
                                             const CaptureSettings &settings,
                                             uint32_t configuredWidth,
                                             uint32_t configuredHeight)
{
    if (shouldUseRequestedActiveRawCrop(sensorFamily, settings, configuredWidth, configuredHeight)) {
        const uint32_t activeHeight = settings.resolution.height & ~1u;
        return ((configuredHeight - activeHeight) / 2u) & ~1u;
    }
    return 0;
}

size_t activeRawFrameBytes(uint32_t stride, uint32_t activeHeight)
{
    if (stride == 0 || activeHeight == 0)
        return 0;
    return static_cast<size_t>(stride) * static_cast<size_t>(activeHeight);
}

std::array<float, 4> blackLevelsForSensor(SensorFamily sensorFamily)
{
    float blackLevel = 200.0f;
    if (sensorFamily == SensorFamily::Imx477 ||
        sensorFamily == SensorFamily::Imx577)
        blackLevel = 256.0f;
    return { blackLevel, blackLevel, blackLevel, blackLevel };
}

int64_t normalizeFrameDurationToUs(int64_t rawFrameDuration, double fpsHint)
{
    if (rawFrameDuration <= 0)
        return 0;

    const double safeFps = std::max(1.0, fpsHint);
    const int64_t expectedUs = static_cast<int64_t>(1000000.0 / safeFps);
    const int64_t candidateUs = rawFrameDuration;
    const int64_t candidateNsAsUs = rawFrameDuration / 1000;

    // Different libcamera forks have exposed this metadata in either us or ns.
    // Prefer a physically plausible frame duration before comparing proximity:
    // an unsupported requested rate must not make 33,333 us look like 33 us.
    const int64_t minimumPlausibleUs = std::max<int64_t>(1, expectedUs / 8);
    const int64_t maximumPlausibleUs = expectedUs * 8;
    const auto isPlausible = [minimumPlausibleUs, maximumPlausibleUs](int64_t value) {
        return value >= minimumPlausibleUs && value <= maximumPlausibleUs;
    };

    if (isPlausible(candidateUs))
        return candidateUs;
    if (isPlausible(candidateNsAsUs))
        return candidateNsAsUs;

    return candidateUs;
}

std::string readPiSerial()
{
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    while (std::getline(cpuinfo, line)) {
        const std::string key = "Serial";
        if (line.rfind(key, 0) != 0)
            continue;
        const std::size_t colon = line.find(':');
        if (colon == std::string::npos)
            continue;
        std::string serial = line.substr(colon + 1);
        serial.erase(serial.begin(), std::find_if(serial.begin(), serial.end(), [](unsigned char c) {
            return !std::isspace(c);
        }));
        return serial;
    }

    return {};
}

const std::string &cachedPiSerial()
{
    static const std::string serial = readPiSerial();
    return serial;
}

std::string defaultCameraLabel(const std::string &model, const std::string &serial)
{
    if (!serial.empty()) {
        const std::string suffix = serial.size() > 4 ? serial.substr(serial.size() - 4) : serial;
        return model.empty() ? ("CAM-" + suffix) : (model + "-" + suffix);
    }
    return model.empty() ? std::string("Apertar") : model;
}

const std::string &cachedCameraModel()
{
    static const std::string model = "Apertar D1";
    return model;
}

const std::string &cachedCameraLabel()
{
    static const std::string label = defaultCameraLabel(cachedCameraModel(), cachedPiSerial());
    return label;
}

std::string uniqueCameraModelForSensor(SensorFamily sensorFamily, const std::string &sensorModel)
{
    (void)sensorFamily;
    (void)sensorModel;
    return "Blackmagic Micro Cinema Camera";
}

bool formatToRawView(const libcamera::PixelFormat &format,
                     RawBitDepth *bitDepth,
                     PixelPacking *packing,
                     BayerPattern *pattern)
{
    struct FormatInfo {
        libcamera::PixelFormat format;
        RawBitDepth bitDepth;
        PixelPacking packing;
        BayerPattern pattern;
    };

    static const std::array<FormatInfo, 30> kFormats{{
        { libcamera::formats::R8, RawBitDepth::Bits8, PixelPacking::Unpacked, BayerPattern::Monochrome },
        { libcamera::formats::R10, RawBitDepth::Bits10, PixelPacking::Unpacked, BayerPattern::Monochrome },
        { libcamera::formats::R12, RawBitDepth::Bits12, PixelPacking::Unpacked, BayerPattern::Monochrome },
        { libcamera::formats::R16, RawBitDepth::Bits16, PixelPacking::Unpacked, BayerPattern::Monochrome },
        { libcamera::formats::R10_CSI2P, RawBitDepth::Bits10, PixelPacking::Csi2Packed, BayerPattern::Monochrome },
        { libcamera::formats::R12_CSI2P, RawBitDepth::Bits12, PixelPacking::Csi2Packed, BayerPattern::Monochrome },
        { libcamera::formats::SRGGB8, RawBitDepth::Bits8, PixelPacking::Unpacked, BayerPattern::RGGB },
        { libcamera::formats::SGRBG8, RawBitDepth::Bits8, PixelPacking::Unpacked, BayerPattern::GRBG },
        { libcamera::formats::SGBRG8, RawBitDepth::Bits8, PixelPacking::Unpacked, BayerPattern::GBRG },
        { libcamera::formats::SBGGR8, RawBitDepth::Bits8, PixelPacking::Unpacked, BayerPattern::BGGR },
        { libcamera::formats::SRGGB10_CSI2P, RawBitDepth::Bits10, PixelPacking::Csi2Packed, BayerPattern::RGGB },
        { libcamera::formats::SGRBG10_CSI2P, RawBitDepth::Bits10, PixelPacking::Csi2Packed, BayerPattern::GRBG },
        { libcamera::formats::SGBRG10_CSI2P, RawBitDepth::Bits10, PixelPacking::Csi2Packed, BayerPattern::GBRG },
        { libcamera::formats::SBGGR10_CSI2P, RawBitDepth::Bits10, PixelPacking::Csi2Packed, BayerPattern::BGGR },
        { libcamera::formats::SRGGB12_CSI2P, RawBitDepth::Bits12, PixelPacking::Csi2Packed, BayerPattern::RGGB },
        { libcamera::formats::SGRBG12_CSI2P, RawBitDepth::Bits12, PixelPacking::Csi2Packed, BayerPattern::GRBG },
        { libcamera::formats::SGBRG12_CSI2P, RawBitDepth::Bits12, PixelPacking::Csi2Packed, BayerPattern::GBRG },
        { libcamera::formats::SBGGR12_CSI2P, RawBitDepth::Bits12, PixelPacking::Csi2Packed, BayerPattern::BGGR },
        { libcamera::formats::SRGGB10, RawBitDepth::Bits10, PixelPacking::Unpacked, BayerPattern::RGGB },
        { libcamera::formats::SGRBG10, RawBitDepth::Bits10, PixelPacking::Unpacked, BayerPattern::GRBG },
        { libcamera::formats::SGBRG10, RawBitDepth::Bits10, PixelPacking::Unpacked, BayerPattern::GBRG },
        { libcamera::formats::SBGGR10, RawBitDepth::Bits10, PixelPacking::Unpacked, BayerPattern::BGGR },
        { libcamera::formats::SRGGB12, RawBitDepth::Bits12, PixelPacking::Unpacked, BayerPattern::RGGB },
        { libcamera::formats::SGRBG12, RawBitDepth::Bits12, PixelPacking::Unpacked, BayerPattern::GRBG },
        { libcamera::formats::SGBRG12, RawBitDepth::Bits12, PixelPacking::Unpacked, BayerPattern::GBRG },
        { libcamera::formats::SBGGR12, RawBitDepth::Bits12, PixelPacking::Unpacked, BayerPattern::BGGR },
        { libcamera::formats::SRGGB16, RawBitDepth::Bits16, PixelPacking::Unpacked, BayerPattern::RGGB },
        { libcamera::formats::SGRBG16, RawBitDepth::Bits16, PixelPacking::Unpacked, BayerPattern::GRBG },
        { libcamera::formats::SGBRG16, RawBitDepth::Bits16, PixelPacking::Unpacked, BayerPattern::GBRG },
        { libcamera::formats::SBGGR16, RawBitDepth::Bits16, PixelPacking::Unpacked, BayerPattern::BGGR },
    }};

    const auto it = std::find_if(kFormats.begin(), kFormats.end(), [&format](const FormatInfo &info) {
        return info.format == format;
    });
    if (it == kFormats.end())
    {
        const std::string formatName = format.toString();
        if (formatName.find("PISP_COMP1") == std::string::npos)
            return false;
        if (formatName.rfind("MONO", 0) == 0) {
            *bitDepth = RawBitDepth::Bits16;
            *packing = PixelPacking::PispCompressed1;
            *pattern = BayerPattern::Monochrome;
            return true;
        }

        // Raspberry Pi / downstream libcamera forks have renamed these formats
        // over time (for example RGGB16_PISP_COMP1 -> RGGB_PISP_COMP1). Match
        // by runtime name so ApertarCore builds against both variants.
        if (formatName.rfind("RGGB", 0) == 0) {
            *bitDepth = RawBitDepth::Bits16;
            *packing = PixelPacking::PispCompressed1;
            *pattern = BayerPattern::RGGB;
            return true;
        }
        if (formatName.rfind("GRBG", 0) == 0) {
            *bitDepth = RawBitDepth::Bits16;
            *packing = PixelPacking::PispCompressed1;
            *pattern = BayerPattern::GRBG;
            return true;
        }
        if (formatName.rfind("GBRG", 0) == 0) {
            *bitDepth = RawBitDepth::Bits16;
            *packing = PixelPacking::PispCompressed1;
            *pattern = BayerPattern::GBRG;
            return true;
        }
        if (formatName.rfind("BGGR", 0) == 0) {
            *bitDepth = RawBitDepth::Bits16;
            *packing = PixelPacking::PispCompressed1;
            *pattern = BayerPattern::BGGR;
            return true;
        }
        return false;
    }

    *bitDepth = it->bitDepth;
    *packing = it->packing;
    *pattern = it->pattern;
    return true;
}

const char *pixelPackingName(PixelPacking packing)
{
    switch (packing) {
    case PixelPacking::Unpacked:
        return "unpacked";
    case PixelPacking::Csi2Packed:
        return "csi2_packed";
    case PixelPacking::PispCompressed1:
        return "pisp_comp1";
    }

    return "unknown";
}

CdngEncoder::Metadata metadataFromRequest(const CaptureSettings &settings,
                                          const libcamera::ControlList &metadata,
                                          uint64_t frameIndex,
                                          int64_t sensorTimestampUs,
                                          uint64_t wallClockTimestampUs,
                                          SensorFamily sensorFamily,
                                          const std::string &dngModel,
                                          const WhiteBalanceMeasurement &whiteBalance,
                                          bool monochrome = false)
{
    CdngEncoder::Metadata out;
    const std::string &cameraModel = cachedCameraModel();
    const std::string &cameraSerial = cachedPiSerial();
    out.frameIndex = frameIndex;
    out.sensorTimestampUs = sensorTimestampUs;
    out.wallClockTimestampUs = wallClockTimestampUs;
    out.timecodeMode = settings.timecodeMode;
    out.fps = settings.fps;
    out.iso = settings.iso;
    out.shutterUs = settings.shutterUs;
    out.whiteBalanceKelvin = whiteBalance.kelvin;
    out.make = "Apertar";
    out.model = cameraModel;
    out.sensorModel = dngModel;
    out.uniqueCameraModel = uniqueCameraModelForSensor(sensorFamily, dngModel);
    out.serial = cameraSerial;
    out.hasBlackLevels = true;
    auto blackLevels = blackLevelsForSensor(sensorFamily);
    bool hasFrameBlackLevels = false;
    if (auto levels = metadata.get(libcamera::controls::SensorBlackLevels);
        levels && levels->size() == 4 &&
        std::all_of(levels->begin(), levels->end(), [](int32_t value) {
            return value >= 0 && value <= 65535;
        })) {
        // libcamera reports R, Gr, Gb, B on a left-shifted 16-bit scale.
        for (size_t i = 0; i < blackLevels.size(); ++i)
            blackLevels[i] = static_cast<float>((*levels)[i]) / 16.0f;
        hasFrameBlackLevels = true;
    }
    if (!hasFrameBlackLevels) {
        if (auto tuningLevels = activeTuningBlackLevels())
            blackLevels = *tuningLevels;
    }
    std::copy(blackLevels.begin(), blackLevels.end(), std::begin(out.blackLevels));
    out.colourGains[0] = whiteBalance.red;
    out.colourGains[1] = whiteBalance.blue;
    out.cameraLabel = cachedCameraLabel();

    if (auto gain = metadata.get(libcamera::controls::AnalogueGain))
        out.analogueGain = *gain;
    if (settings.isoAuto && std::isfinite(out.analogueGain) && out.analogueGain > 0.0f)
        out.iso = static_cast<uint32_t>(std::clamp(std::lround(out.analogueGain * 100.0f), 1l, 65535l));
    if (auto gain = metadata.get(libcamera::controls::DigitalGain))
        out.digitalGain = *gain;
    if (auto exposure = metadata.get(libcamera::controls::ExposureTime))
        out.shutterUs = static_cast<uint32_t>(*exposure);
    if (auto temperature = metadata.get(libcamera::controls::ColourTemperature);
        temperature && *temperature > 0) {
        out.whiteBalanceKelvin = static_cast<uint32_t>(*temperature);
    }
    if (auto gains = metadata.get(libcamera::controls::ColourGains);
        gains && gains->size() >= 2) {
        const float red = (*gains)[0];
        const float blue = (*gains)[1];
        if (std::isfinite(red) && red > 0.0f)
            out.colourGains[0] = red;
        if (std::isfinite(blue) && blue > 0.0f)
            out.colourGains[1] = blue;
    }
    if (monochrome) {
        out.uniqueCameraModel = "Apertar " + dngModel + " Monochrome";
        out.colourGains[0] = out.colourGains[1] = 1.0f;
        return out;
    }
    if (sensorFamily != SensorFamily::Imx585) {
        if (auto matrix = metadata.get(libcamera::controls::ColourCorrectionMatrix); matrix && matrix->size() >= 9) {
            out.hasColourMatrix = true;
            for (size_t i = 0; i < 9; ++i)
                out.colourMatrix[i] = (*matrix)[i];
            if (settings.previewExposureReference) {
                // The frame CCM includes the preview gamut; DNG needs sensor -> Rec709.
                std::array<float, 9> source;
                std::copy_n(out.colourMatrix, 9, source.begin());
                for (size_t row = 0; row < 3; ++row)
                    for (size_t col = 0; col < 3; ++col) {
                        double value = 0;
                        for (size_t k = 0; k < 3; ++k)
                            value += settings.previewExposureReference->toRec709[row * 3 + k] * source[k * 3 + col];
                        out.colourMatrix[row * 3 + col] = static_cast<float>(value);
                    }
            }
        }
    }
    if (auto profile = encoderColorProfileForSensor(out.sensorModel, out.whiteBalanceKelvin)) {
        if (profile->hasCcm) {
            out.hasColourMatrix = true;
            std::copy(profile->ccm.begin(), profile->ccm.end(), std::begin(out.colourMatrix));
        }
        // Keep the runtime CCM in its own RGB space. DNG ColorMatrix is
        // XYZ -> native RAW and is selected separately by the DNG writer.
        if (profile->hasForwardMatrix1) {
            out.hasForwardMatrix1 = true;
            std::copy(profile->forwardMatrix1.begin(), profile->forwardMatrix1.end(), std::begin(out.forwardMatrix1));
        }
        if (profile->hasForwardMatrix2) {
            out.hasForwardMatrix2 = true;
            std::copy(profile->forwardMatrix2.begin(), profile->forwardMatrix2.end(), std::begin(out.forwardMatrix2));
        }
        out.calibrationIlluminant1 = profile->illuminant1;
        out.calibrationIlluminant2 = profile->illuminant2;
    }

    return out;
}

class DngRecordingQueue {
public:
    explicit DngRecordingQueue(RecordingDropWarning *warning = nullptr) : dropWarning_(warning) {}
    struct Stats {
        uint64_t enqueued = 0;
        uint64_t written = 0;
        uint64_t droppedQueueFull = 0;
        uint64_t discardedOnStop = 0;
        uint64_t writeFailures = 0;
        uint64_t currentQueueDepth = 0;
        uint64_t peakQueueDepth = 0;
        uint64_t maxQueueDepth = 0;
    };

    ~DngRecordingQueue()
    {
        stop();
    }

    bool start(const std::string &mediaRoot, const CaptureSettings &settings, std::string *error)
    {
        try {
            return startAtFolder(allocateClipFolder(mediaRoot, settings), settings, error);
        } catch (const std::exception &e) {
            if (error)
                *error = e.what();
            return false;
        }
    }

    bool startAtFolder(const std::filesystem::path &folderPath, const CaptureSettings &settings, std::string *error)
    {
        stop();

        settings_ = settings;
        recordingFormat_ = settings.recordingFormat;
        frameIndex_ = 0;
        firstFrameTimestampUs_ = 0;
        droppedFrames_ = 0;
        enqueuedFrames_ = 0;
        writtenFrames_ = 0;
        writeFailures_ = 0;
        failure_.reset();
        losslessTimedJobs_ = 0;
        losslessWorkerWallUs_ = 0;
        losslessWorkerCpuUs_ = 0;
        losslessMaxWorkerWallUs_ = 0;
        losslessEncodeWallUs_ = 0;
        losslessDecodeWallUs_ = 0;
        losslessEntropyWallUs_ = 0;
        losslessPayloadWriteWallUs_ = 0;
        losslessContainerWallUs_ = 0;
        losslessPayloadBytes_ = 0;
        losslessProfiledSourceRows_ = 0;
        losslessTileWrites_ = 0;
        discardedOnStopFrames_ = 0;
        maxObservedQueueDepth_ = 0;
        active_.store(false, std::memory_order_release);
        stopping_ = false;
        pooledRawBytes_ = 0;
        expectedRawBufferSize_ = 0;

        try {
            folderPath_ = folderPath;
            std::filesystem::create_directories(folderPath_);
            clipName_ = folderPath_.filename().string();
            folderPathString_ = folderPath_.string();
            mlvPathString_ = (folderPath_ / (clipName_ + ".MLV")).string();
        } catch (const std::exception &e) {
            if (error)
                *error = e.what();
            return false;
        }

        const unsigned int hardwareThreads = std::thread::hardware_concurrency();
        const unsigned int normalWorkerCount = std::max(2u, std::min(4u, hardwareThreads));
        const bool losslessCdng = recordingFormat_ == RecordingFormat::Cdng &&
            settings_.cdngCompression == CdngCompression::LosslessJpeg;
        const bool compressedDng = losslessCdng || recordingFormat_ == RecordingFormat::LossyDng;
        const unsigned int defaultQueueFrames = compressedDng && hardwareThreads >= 4 ? 24u : 12u;
        maxQueuedFrames_ = unsignedEnvValue(
            "APERTAR_CDNG_QUEUE_FRAMES", defaultQueueFrames, 4, 64);
        // One extra lossless worker overlaps tile-write and scheduling stalls
        // while the other workers remain CPU-active.
        const unsigned int defaultLosslessWorkers = hardwareThreads >= 4
            ? std::min(8u, hardwareThreads + 1u)
            : normalWorkerCount;
        // Lossy DNG is CPU-bound and does not benefit from oversubscribing the
        // four-core Pi. Leave one worker per core so capture can be scheduled
        // promptly; lossless cDNG keeps its separately measured extra worker.
        const unsigned int defaultLossyWorkers = std::max(1u, std::min(4u, hardwareThreads));
        const unsigned int workerCount = recordingFormat_ == RecordingFormat::Mlv
            ? 1u
            : (losslessCdng
                   ? unsignedEnvValue("APERTAR_CDNG_WORKERS", defaultLosslessWorkers, 1, 8)
                   : (recordingFormat_ == RecordingFormat::LossyDng
                          ? unsignedEnvValue("APERTAR_LOSSYDNG_WORKERS", defaultLossyWorkers, 1, 8)
                          : normalWorkerCount));
        workerCount_ = workerCount;
        workers_.reserve(workerCount);
        for (unsigned int i = 0; i < workerCount; ++i)
            workers_.emplace_back(&DngRecordingQueue::workerLoop, this);
        active_.store(true, std::memory_order_release);

        std::cerr << "ApertarCore recording clip: " << folderPath_.string()
                  << " format=" << toString(recordingFormat_)
                  << " compression=" << (recordingFormat_ == RecordingFormat::Cdng ? toString(settings_.cdngCompression)
                                      : recordingFormat_ == RecordingFormat::LossyDng ? toWireValue(settings_.lossyDngRatio)
                                      : "none");
        if (recordingFormat_ == RecordingFormat::Cdng && settings_.cdngCompression == CdngCompression::LosslessJpeg) {
            std::cerr << " worker_nice=" << unsignedEnvValue("APERTAR_CDNG_WORKER_NICE", 0, 0, 19);
            std::cerr << " workers=" << workerCount_;
            std::cerr << " queue=" << maxQueuedFrames_;
            std::cerr << " tile_rows=" << CdngEncoder::losslessTileRows();
            std::cerr << " encoder=quad_single_pass";
#if defined(NDEBUG)
            std::cerr << " build=release";
#else
            std::cerr << " build=debug";
#endif
        } else if (recordingFormat_ == RecordingFormat::LossyDng) {
            std::cerr << " workers=" << workerCount_;
            std::cerr << " queue=" << maxQueuedFrames_;
            std::cerr << " encoder=quantized_sof3";
        }
        std::cerr << "\n";
        return true;
    }

    static std::filesystem::path reserveClipFolder(const std::string &mediaRoot,
                                                   RecordingFormat format)
    {
        return allocateClipFolder(mediaRoot, format);
    }

    static std::filesystem::path reserveClipFolder(const std::string &mediaRoot,
                                                   const CaptureSettings &settings)
    {
        return allocateClipFolder(mediaRoot, settings);
    }

    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_.load(std::memory_order_acquire) && workers_.empty() && !stopping_)
                return;
        }
        const auto stopStarted = std::chrono::steady_clock::now();
        active_.store(false, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        cv_.notify_all();

        for (std::thread &worker : workers_) {
            if (worker.joinable())
                worker.join();
        }
        workers_.clear();
        if (recordingFormat_ == RecordingFormat::LossyDng)
            LossyDngEncoder::drainWrites();

        const Stats summary = stats();
        if (summary.enqueued > 0 || summary.written > 0 || summary.droppedQueueFull > 0 ||
            summary.discardedOnStop > 0 || summary.writeFailures > 0) {
            std::cerr << "ApertarCore recording stats: "
                      << "enqueued=" << summary.enqueued
                      << " written=" << summary.written
                      << " dropped_queue_full=" << summary.droppedQueueFull
                      << " discarded_on_stop=" << summary.discardedOnStop
                      << " write_failures=" << summary.writeFailures
                      << " peak_queue=" << summary.peakQueueDepth << "/" << summary.maxQueueDepth
                      << "\n";
        }
        const uint64_t timedJobs = losslessTimedJobs_.load(std::memory_order_relaxed);
        if (timedJobs > 0) {
            const double averageWallUs = static_cast<double>(
                losslessWorkerWallUs_.load(std::memory_order_relaxed)) / timedJobs;
            const double averageCpuUs = static_cast<double>(
                losslessWorkerCpuUs_.load(std::memory_order_relaxed)) / timedJobs;
            const double estimatedCapacityFps = averageWallUs > 0.0
                ? static_cast<double>(workerCount_) * 1000000.0 / averageWallUs
                : 0.0;
            std::cerr << "ApertarCore lossless timing: jobs=" << timedJobs
                      << std::fixed << std::setprecision(2)
                      << " avg_wall_ms=" << averageWallUs / 1000.0
                      << " avg_thread_cpu_ms=" << averageCpuUs / 1000.0
                      << " avg_wait_or_preempt_ms=" << std::max(0.0, averageWallUs - averageCpuUs) / 1000.0
                      << " max_wall_ms="
                      << static_cast<double>(losslessMaxWorkerWallUs_.load(std::memory_order_relaxed)) / 1000.0
                      << " estimated_capacity_fps=" << estimatedCapacityFps
                      << std::defaultfloat << "\n";
            const double payloadMiB = static_cast<double>(
                losslessPayloadBytes_.load(std::memory_order_relaxed)) / (1024.0 * 1024.0);
            std::cerr << "ApertarCore lossless phases: jobs=" << timedJobs
                      << std::fixed << std::setprecision(2)
                      << " avg_encode_ms="
                      << static_cast<double>(losslessEncodeWallUs_.load(std::memory_order_relaxed)) / timedJobs / 1000.0
                      << " avg_payload_write_ms="
                      << static_cast<double>(losslessPayloadWriteWallUs_.load(std::memory_order_relaxed)) / timedJobs / 1000.0
                      << " avg_container_ms="
                      << static_cast<double>(losslessContainerWallUs_.load(std::memory_order_relaxed)) / timedJobs / 1000.0
                      << " avg_payload_mib=" << payloadMiB / timedJobs
                      << " tile_writes_per_frame="
                      << static_cast<double>(losslessTileWrites_.load(std::memory_order_relaxed)) / timedJobs
                      << std::defaultfloat << "\n";
            if (losslessProfiledSourceRows_.load(std::memory_order_relaxed) > 0) {
                const double decodeUs = static_cast<double>(
                    losslessDecodeWallUs_.load(std::memory_order_relaxed));
                const double entropyUs = static_cast<double>(
                    losslessEntropyWallUs_.load(std::memory_order_relaxed));
                const double encodeUs = static_cast<double>(
                    losslessEncodeWallUs_.load(std::memory_order_relaxed));
                std::cerr << "ApertarCore lossless detail: jobs=" << timedJobs
                          << std::fixed << std::setprecision(2)
                          << " avg_pisp_decode_ms=" << decodeUs / timedJobs / 1000.0
                          << " avg_jpeg_entropy_ms=" << entropyUs / timedJobs / 1000.0
                          << " avg_tile_setup_finish_ms="
                          << std::max(0.0, encodeUs - decodeUs - entropyUs) / timedJobs / 1000.0
                          << " profiled_source_rows="
                          << losslessProfiledSourceRows_.load(std::memory_order_relaxed)
                          << std::defaultfloat << "\n";
            }
        }
        const double stopSeconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - stopStarted).count();
        if (stopSeconds > 0.20) {
            std::cerr << "ApertarCore recorder stop time: "
                      << std::fixed << std::setprecision(3) << stopSeconds
                      << "s format=" << toString(recordingFormat_)
                      << " discarded_on_stop=" << summary.discardedOnStop
                      << "\n";
        }

        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        rawPool_.clear();
        pooledRawBytes_ = 0;
        expectedRawBufferSize_ = 0;
        workerCount_ = 0;
        stopping_ = false;
    }

    bool active() const
    {
        return active_.load(std::memory_order_acquire);
    }

    Stats stats() const
    {
        Stats out;
        out.enqueued = enqueuedFrames_.load(std::memory_order_relaxed);
        out.written = writtenFrames_.load(std::memory_order_relaxed);
        out.writeFailures = writeFailures_.load(std::memory_order_relaxed);
        out.maxQueueDepth = maxQueuedFrames_;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            out.droppedQueueFull = droppedFrames_;
            out.discardedOnStop = discardedOnStopFrames_;
            out.currentQueueDepth = queue_.size();
            out.peakQueueDepth = maxObservedQueueDepth_;
        }
        return out;
    }

    std::string error() const { return failure_.message(); }

    uint64_t nextFrameIndex(int64_t timestampUs)
    {
        if (firstFrameTimestampUs_ == 0) firstFrameTimestampUs_ = timestampUs;
        const uint64_t timeIndex = timestampUs > firstFrameTimestampUs_
            ? uint64_t(std::llround((timestampUs - firstFrameTimestampUs_) * settings_.fps / 1000000)) : 0;
        const auto index = std::max(frameIndex_.load(std::memory_order_relaxed), timeIndex);
        frameIndex_.store(index + 1, std::memory_order_relaxed);
        return index;
    }

    bool hasCapacityForFrame(uint64_t frameIndex)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || !active() || workers_.empty())
            return false;
        if (queue_.size() < maxQueuedFrames_)
            return true;

        ++droppedFrames_;
        if (dropWarning_) dropWarning_->markDropped();
        if (droppedFrames_ == 1 || (droppedFrames_ % 30) == 0) {
            std::cerr << "ApertarCore drop[queue-full]: frame="
                      << frameIndex
                      << " dropped_total=" << droppedFrames_
                      << " queue=" << queue_.size() << "/" << maxQueuedFrames_
                      << "\n";
        }
        return false;
    }

    std::vector<uint8_t> acquireRawBuffer(size_t size)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        expectedRawBufferSize_ = size;
        auto reusable = std::min_element(rawPool_.begin(), rawPool_.end(), [size](const std::vector<uint8_t> &lhs,
                                                                                   const std::vector<uint8_t> &rhs) {
            const bool lhsFits = lhs.capacity() >= size;
            const bool rhsFits = rhs.capacity() >= size;
            if (lhsFits != rhsFits)
                return lhsFits;
            return lhs.capacity() < rhs.capacity();
        });
        if (reusable == rawPool_.end() || reusable->capacity() < size)
            return std::vector<uint8_t>(size);

        std::vector<uint8_t> buffer = std::move(*reusable);
        rawPool_.erase(reusable);
        pooledRawBytes_ = pooledRawBytes_ > buffer.capacity()
            ? (pooledRawBytes_ - buffer.capacity())
            : 0;
        buffer.resize(size);
        return buffer;
    }

    bool enqueue(std::vector<uint8_t> rawBytes,
                 RawFrameView frame,
                 CdngEncoder::Metadata metadata)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || workers_.empty())
            return false;
        if (queue_.size() >= maxQueuedFrames_) {
            ++droppedFrames_;
            if (dropWarning_) dropWarning_->markDropped();
            if (droppedFrames_ == 1 || (droppedFrames_ % 30) == 0) {
                std::cerr << "ApertarCore drop[queue-full]: frame="
                          << metadata.frameIndex
                          << " dropped_total=" << droppedFrames_
                          << " queue=" << queue_.size() << "/" << maxQueuedFrames_
                          << "\n";
            }
            return false;
        }

        const bool mlv = recordingFormat_ == RecordingFormat::Mlv;
        const std::string frameName = mlv
            ? (clipName_ + ".MLV")
            : frameFilename(clipName_, metadata.frameIndex);
        metadata.clipName = clipName_;
        metadata.frameFileName = frameName;
        metadata.reelName = clipName_;
        metadata.compression = mlv ? CdngCompression::Uncompressed : settings_.cdngCompression;
        queue_.push_back(FrameJob{
            std::move(rawBytes),
            frame,
            std::move(metadata),
            mlv ? mlvPathString_ : (folderPathString_ + "/" + frameName),
        });
        ++enqueuedFrames_;
        maxObservedQueueDepth_ = std::max<uint64_t>(maxObservedQueueDepth_, queue_.size());
        cv_.notify_one();
        return true;
    }

    std::string folderPath() const
    {
        return folderPath_.string();
    }

private:
    struct FrameJob {
        std::vector<uint8_t> rawBytes;
        RawFrameView frame;
        CdngEncoder::Metadata metadata;
        std::string path;
    };

    static std::filesystem::path allocateClipFolder(const std::string &mediaRoot,
                                                    RecordingFormat format)
    {
        CaptureSettings settings;
        settings.recordingFormat = format;
        return allocateClipFolder(mediaRoot, settings);
    }

    static std::filesystem::path allocateClipFolder(const std::string &mediaRoot,
                                                    const CaptureSettings &settings)
    {
        const std::filesystem::path root = mediaRootPath(mediaRoot);
        const auto now = std::chrono::system_clock::now();
        const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
        std::tm localTime{};
#if defined(_WIN32)
        localtime_s(&localTime, &seconds);
#else
        localtime_r(&seconds, &localTime);
#endif

        const char *formatPrefix = "cDNG_";
        switch (settings.recordingFormat) {
        case RecordingFormat::Mlv:
            formatPrefix = "MLV_";
            break;
        case RecordingFormat::H264:
            formatPrefix = "H264_";
            break;
        case RecordingFormat::LossyDng:
            formatPrefix = settings.lossyDngRatio == LossyDngRatio::R5 ? "cDNG_5to1_" : "cDNG_3to1_";
            break;
        case RecordingFormat::Prores:
            formatPrefix = "ProRes_";
            break;
        case RecordingFormat::Cdng:
            break;
        }

        std::ostringstream prefix;
        prefix << formatPrefix << std::put_time(&localTime, "%y-%m-%d");

        for (int clip = 1; clip <= 9999; ++clip) {
            std::ostringstream name;
            name << prefix.str() << "_C" << std::setw(4) << std::setfill('0') << clip;
            std::filesystem::path candidate = root / name.str();
            if (!std::filesystem::exists(candidate))
                return candidate;
        }

        throw std::runtime_error("Could not allocate a unique clip folder.");
    }

    static std::string frameFilename(const std::string &clipName, uint64_t frameIndex)
    {
        std::ostringstream out;
        if (!clipName.empty())
            out << clipName << "_";
        out << std::setw(9) << std::setfill('0') << frameIndex << ".dng";
        return out.str();
    }

    void workerLoop()
    {
        if (recordingFormat_ == RecordingFormat::Mlv) {
            mlvWorkerLoop();
            return;
        }

        dngWorkerLoop();
    }

    void dngWorkerLoop()
    {
#if defined(__linux__)
        // Lossy workers must leave the camera-completion thread scheduling
        // headroom. Lossless retains its separately measured normal priority.
        if ((settings_.recordingFormat == RecordingFormat::Cdng &&
             settings_.cdngCompression == CdngCompression::LosslessJpeg) ||
            settings_.recordingFormat == RecordingFormat::LossyDng) {
            const unsigned defaultNice = settings_.recordingFormat == RecordingFormat::LossyDng ? 5u : 0u;
            const int workerNice = static_cast<int>(unsignedEnvValue("APERTAR_CDNG_WORKER_NICE", defaultNice, 0, 19));
            if (workerNice > 0)
                (void)::setpriority(PRIO_PROCESS, static_cast<id_t>(::syscall(SYS_gettid)), workerNice);
        }
#endif
        CdngEncoder encoder;
        std::unique_ptr<LossyDngEncoder> lossyEncoder;
        if (settings_.recordingFormat == RecordingFormat::LossyDng)
            lossyEncoder = std::make_unique<LossyDngEncoder>(settings_.lossyDngRatio);

        while (true) {
            FrameJob job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this]() {
                    return stopping_ || !queue_.empty();
                });

                if (queue_.empty()) {
                    if (stopping_)
                        return;
                    continue;
                }

                job = std::move(queue_.front());
                queue_.pop_front();
            }

            job.frame.data = job.rawBytes.data();
            const bool timeLossless = settings_.recordingFormat == RecordingFormat::Cdng &&
                settings_.cdngCompression == CdngCompression::LosslessJpeg;
            const auto wallStarted = std::chrono::steady_clock::now();
            const uint64_t cpuStartedUs = timeLossless ? currentThreadCpuTimeUs() : 0;
            bool completed = false;
            try {
                if (lossyEncoder)
                    lossyEncoder->writeDngFile(job.path, job.frame, job.metadata);
                else
                    encoder.writeDngFile(job.path, job.frame, job.metadata);
                completed = true;
                ++writtenFrames_;
            } catch (const std::exception &e) {
                ++writeFailures_;
                failRecording(e.what());
                std::cerr << "ApertarCore DNG write failed for "
                          << job.path << ": " << e.what() << "\n";
            }
            if (timeLossless) {
                if (completed)
                    recordLosslessPhases(encoder.lastPerformanceStats());
                const uint64_t wallUs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - wallStarted).count());
                const uint64_t cpuFinishedUs = currentThreadCpuTimeUs();
                recordLosslessTiming(wallUs,
                                     cpuFinishedUs >= cpuStartedUs ? cpuFinishedUs - cpuStartedUs : 0);
            }

            recycleRawBuffer(std::move(job.rawBytes));
        }
    }

    void recordLosslessPhases(const CdngEncoder::PerformanceStats &stats)
    {
        losslessEncodeWallUs_.fetch_add(stats.losslessEncodeWallUs, std::memory_order_relaxed);
        losslessDecodeWallUs_.fetch_add(stats.losslessDecodeWallUs, std::memory_order_relaxed);
        losslessEntropyWallUs_.fetch_add(stats.losslessEntropyWallUs, std::memory_order_relaxed);
        losslessPayloadWriteWallUs_.fetch_add(stats.losslessPayloadWriteWallUs, std::memory_order_relaxed);
        losslessContainerWallUs_.fetch_add(stats.losslessContainerWallUs, std::memory_order_relaxed);
        losslessPayloadBytes_.fetch_add(stats.losslessPayloadBytes, std::memory_order_relaxed);
        losslessProfiledSourceRows_.fetch_add(
            stats.losslessProfiledSourceRows, std::memory_order_relaxed);
        losslessTileWrites_.fetch_add(stats.losslessTileWrites, std::memory_order_relaxed);
    }

    static uint64_t currentThreadCpuTimeUs()
    {
#if defined(__linux__)
        timespec value{};
        if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) == 0) {
            return static_cast<uint64_t>(value.tv_sec) * 1000000ull
                + static_cast<uint64_t>(value.tv_nsec) / 1000ull;
        }
#endif
        return 0;
    }

    void recordLosslessTiming(uint64_t wallUs, uint64_t cpuUs)
    {
        losslessTimedJobs_.fetch_add(1, std::memory_order_relaxed);
        losslessWorkerWallUs_.fetch_add(wallUs, std::memory_order_relaxed);
        losslessWorkerCpuUs_.fetch_add(cpuUs, std::memory_order_relaxed);
        uint64_t currentMax = losslessMaxWorkerWallUs_.load(std::memory_order_relaxed);
        while (currentMax < wallUs &&
               !losslessMaxWorkerWallUs_.compare_exchange_weak(
                   currentMax, wallUs, std::memory_order_relaxed, std::memory_order_relaxed)) {
        }
    }

    void mlvWorkerLoop()
    {
        MlvEncoder encoder;

        while (true) {
            FrameJob job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this]() {
                    return stopping_ || !queue_.empty();
                });

                if (queue_.empty()) {
                    if (stopping_)
                        return;
                    continue;
                }

                job = std::move(queue_.front());
                queue_.pop_front();
            }

            job.frame.data = job.rawBytes.data();
            try {
                std::string mlvError;
                if (!encoder.isOpen() &&
                    !encoder.open(job.path, settings_, job.frame, job.metadata, &mlvError)) {
                    throw std::runtime_error(mlvError.empty() ? "failed to open MLV file" : mlvError);
                }

                if (!encoder.writeFrame(job.frame, job.metadata, &mlvError))
                    throw std::runtime_error(mlvError.empty() ? "failed to write MLV frame" : mlvError);
                ++writtenFrames_;
            } catch (const std::exception &e) {
                ++writeFailures_;
                failRecording(e.what());
                std::cerr << "ApertarCore MLV write failed for "
                          << job.path << ": " << e.what() << "\n";
            }

            recycleRawBuffer(std::move(job.rawBytes));
        }
    }

    void failRecording(const std::string &error)
    {
        failure_.set(error);
        active_.store(false, std::memory_order_release);
        {
            std::lock_guard lock(mutex_);
            stopping_ = true;
            queue_.clear();
        }
        cv_.notify_all();
    }

    void recycleRawBuffer(std::vector<uint8_t> buffer)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const size_t capacity = buffer.capacity();
        if (capacity == 0)
            return;

        const size_t expectedSize = expectedRawBufferSize_;
        const size_t maxRetainedBuffers = preferredRawPoolSize();
        const size_t maxRetainedBytes = expectedSize > 0 ? expectedSize * maxRetainedBuffers : 0;
        const size_t oversizeThreshold = expectedSize > 0 ? expectedSize + (expectedSize / 2) : 0;

        if ((expectedSize > 0 && capacity > oversizeThreshold)
            || rawPool_.size() >= maxRetainedBuffers
            || (maxRetainedBytes > 0 && (pooledRawBytes_ + capacity) > maxRetainedBytes)) {
            return;
        }

        pooledRawBytes_ += capacity;
        rawPool_.push_back(std::move(buffer));
    }

    size_t preferredRawPoolSize() const
    {
        return std::min(maxQueuedFrames_, std::max<size_t>(workerCount_ + 2u, 4u));
    }

    CaptureSettings settings_{};
    RecordingFormat recordingFormat_ = RecordingFormat::Cdng;
    std::filesystem::path folderPath_;
    std::string folderPathString_;
    std::string mlvPathString_;
    std::string clipName_;
    std::atomic<uint64_t> frameIndex_{0};
    int64_t firstFrameTimestampUs_ = 0;
    uint64_t droppedFrames_ = 0;
    RecordingDropWarning *dropWarning_ = nullptr;
    uint64_t discardedOnStopFrames_ = 0;
    std::atomic<uint64_t> enqueuedFrames_{0};
    std::atomic<uint64_t> writtenFrames_{0};
    RecordingFailure failure_;
    std::atomic<uint64_t> writeFailures_{0};
    std::atomic<uint64_t> losslessTimedJobs_{0};
    std::atomic<uint64_t> losslessWorkerWallUs_{0};
    std::atomic<uint64_t> losslessWorkerCpuUs_{0};
    std::atomic<uint64_t> losslessMaxWorkerWallUs_{0};
    std::atomic<uint64_t> losslessEncodeWallUs_{0};
    std::atomic<uint64_t> losslessDecodeWallUs_{0};
    std::atomic<uint64_t> losslessEntropyWallUs_{0};
    std::atomic<uint64_t> losslessPayloadWriteWallUs_{0};
    std::atomic<uint64_t> losslessContainerWallUs_{0};
    std::atomic<uint64_t> losslessPayloadBytes_{0};
    std::atomic<uint64_t> losslessProfiledSourceRows_{0};
    std::atomic<uint64_t> losslessTileWrites_{0};
    std::atomic<bool> active_{false};
    uint64_t maxObservedQueueDepth_ = 0;
    size_t pooledRawBytes_ = 0;
    size_t expectedRawBufferSize_ = 0;
    unsigned int workerCount_ = 0;
    size_t maxQueuedFrames_ = 12;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<FrameJob> queue_;
    std::vector<std::vector<uint8_t>> rawPool_;
    std::vector<std::thread> workers_;
    bool stopping_ = false;
};

class H264PreviewRecorder {
    struct PreviewJob {
        const uint8_t *base = nullptr;
        uint32_t width = 0;
        uint32_t height = 0;
        std::array<uint32_t, 3> planeOffsets{};
        std::array<uint32_t, 3> planePitches{};
        std::function<void()> release;
        int64_t timestampUs = 0;
    };

    struct EncodeJob {
        std::vector<uint8_t> yuv;
        int64_t timestampUs = 0;
    };

public:
    explicit H264PreviewRecorder(RecordingDropWarning *warning = nullptr) : dropWarning_(warning) {}
    ~H264PreviewRecorder()
    {
        stop();
    }

    bool startAtFolder(const std::filesystem::path &folderPath,
                       const CaptureSettings &settings,
                       uint32_t width,
                       uint32_t height,
                       std::string *error,
                       std::function<void(int64_t)> frameWritten = {})
    {
        stop();
        frameWritten_ = std::move(frameWritten);
        freeRunTimecode_ = settings.timecodeMode == TimecodeMode::FreeRun;

        if (width < 2 || height < 2 || (width % 2) != 0 || (height % 2) != 0) {
            setError(error, "H.264 preview recording requires an even-sized YUV preview stream.");
            return false;
        }

        const std::string backendPreference = h264BackendPreference();
        const bool hardware = recordingCapabilities().hardwareH264;
        std::string hardwareDevice;
        if (hardware) {
            if (width > 1920 || height > 1080 || settings.fps > 30.0005) {
                setError(error, "Pi 3/4 hardware H.264 is limited to 1920x1080 at 30 FPS.");
                return false;
            }
            if (backendPreference == "pipe") {
                setError(error, "Pi 3/4 hardware H.264 requires the libav backend. Remove APERTAR_H264_BACKEND=pipe.");
                return false;
            }
            if (const char *encoder = std::getenv("APERTAR_H264_ENCODER"); encoder && *encoder &&
                std::strcmp(encoder, "h264_v4l2m2m") != 0) {
                setError(error, "Pi 3/4 H.264 requires h264_v4l2m2m; remove the APERTAR_H264_ENCODER override.");
                return false;
            }
            hardwareDevice = findRaspberryPiH264Encoder(error);
            if (hardwareDevice.empty()) return false;
        }
#if !APERTARCORE_HAS_LIBAV
        if (hardware || backendPreference == "libav") {
            setError(error, "In-process H.264 recording requires libavcodec/libavformat development libraries at build time.");
            return false;
        }
#endif

        std::error_code ec;
        std::filesystem::create_directories(folderPath, ec);
        if (ec) {
            setError(error, "Could not create H.264 clip folder: " + ec.message());
            return false;
        }

        folderPath_ = folderPath;
        clipName_ = folderPath_.filename().string();
        finalPath_ = folderPath_ / (clipName_ + ".mp4");
        tempPath_ = folderPath_ / (clipName_ + ".mp4.writing");
        width_ = width;
        height_ = height;
        fps_ = settings.fps > 0.0 ? settings.fps : 24.0;
        frameBytes_ = yuv420FrameBytes(width_, height_);
        maxQueuedFrames_ = h264QueueDepth();
        framesQueued_.store(0, std::memory_order_relaxed);
        framesWritten_.store(0, std::memory_order_relaxed);
        droppedFrames_.store(0, std::memory_order_relaxed);
        skippedStoppingFrames_.store(0, std::memory_order_relaxed);
        writeFailures_.store(0, std::memory_order_relaxed);
        failure_.reset();
        peakQueueDepth_.store(0, std::memory_order_relaxed);
        maxCopyMs_.store(0, std::memory_order_relaxed);
        slowCopies_.store(0, std::memory_order_relaxed);
        maxWriteMs_.store(0, std::memory_order_relaxed);
        slowWrites_.store(0, std::memory_order_relaxed);

        std::filesystem::remove(tempPath_, ec);
        ec.clear();
        std::filesystem::remove(finalPath_, ec);

        const std::string sizeArg = std::to_string(width_) + "x" + std::to_string(height_);
        const std::string fpsArg = frameRateArg(fps_);
        const std::string encoder = h264EncoderName();
        const std::string preset = h264PresetName();
        const std::string tune = h264TuneName();
        const std::string crf = h264CrfValue();
        const std::string qp = h264QpValue();
        const std::string threads = h264ThreadCountValue();
        const std::string bitrate = h264BitrateValue();
        const std::string maxrate = h264MaxrateValue();
        const std::string bufsize = h264BufsizeValue();
        const std::string x264Params = h264X264ParamsValue();
        const bool x264Encoder = isX264Encoder(encoder);
        const std::string outputPath = tempPath_.string();
        copyThreadCount_ = h264CopyThreadCount();

        bool startedEncoder = false;
#if APERTARCORE_HAS_LIBAV
        if (backendPreference != "pipe") {
            std::string libavError;
            if (startLibavEncoder(encoder,
                                  preset,
                                  tune,
                                  crf,
                                  qp,
                                  threads,
                                  bitrate,
                                  maxrate,
                                  bufsize,
                                  x264Params,
                                  &libavError)) {
                backend_ = EncoderBackend::Libav;
                startedEncoder = true;
            } else if (hardware || backendPreference == "libav") {
                setError(error, hardware ? "Hardware H.264 could not start: " + libavError + ". CPU encoding was not enabled." : libavError);
                return false;
            } else {
                std::cerr << "ApertarCore H.264 libav backend unavailable: "
                          << libavError
                          << "; falling back to ffmpeg pipe\n";
            }
        }
#endif

        int pipeFds[2] = {-1, -1};
        if (!startedEncoder) {
            const std::string ffmpegPath = findFfmpegPath();
            if (ffmpegPath.empty()) {
                setError(error, "H.264 recording requires ffmpeg. Install ffmpeg or switch back to cDNG/MLV.");
                return false;
            }

            if (::pipe(pipeFds) != 0) {
                setError(error, std::string("Could not create H.264 encoder pipe: ") + std::strerror(errno));
                return false;
            }
            tunePipeSize(pipeFds[1]);

            backend_ = EncoderBackend::Pipe;

            std::vector<std::string> args = {
                ffmpegPath,
                "-hide_banner",
                "-loglevel", "warning",
                "-y",
                "-f", "rawvideo",
                "-pix_fmt", "yuv420p",
                "-s", sizeArg,
                "-r", fpsArg,
                "-i", "pipe:0",
                "-an",
                "-c:v", encoder,
            };

            if (x264Encoder) {
                args.push_back("-preset");
                args.push_back(preset);
                if (!tune.empty()) {
                    args.push_back("-tune");
                    args.push_back(tune);
                }

                if (!threads.empty()) {
                    args.push_back("-threads");
                    args.push_back(threads);
                }

                if (!qp.empty()) {
                    args.push_back("-qp");
                    args.push_back(qp);
                } else if (bitrate.empty()) {
                    args.push_back("-crf");
                    args.push_back(crf);
                } else {
                    args.push_back("-b:v");
                    args.push_back(bitrate);
                }

                if (!maxrate.empty()) {
                    args.push_back("-maxrate");
                    args.push_back(maxrate);
                }
                if (!bufsize.empty()) {
                    args.push_back("-bufsize");
                    args.push_back(bufsize);
                }
                if (!x264Params.empty()) {
                    args.push_back("-x264-params");
                    args.push_back(x264Params);
                }
            } else {
                args.push_back("-b:v");
                args.push_back(bitrate.empty() ? "80M" : bitrate);
            }
            args.insert(args.end(), {
                "-pix_fmt", "yuv420p",
                "-f", "mp4",
                outputPath,
            });

            std::vector<char *> argv;
            argv.reserve(args.size() + 1);
            for (std::string &arg : args)
                argv.push_back(arg.data());
            argv.push_back(nullptr);

            childPid_ = ::fork();
            if (childPid_ == 0) {
                ::close(pipeFds[1]);
                if (::dup2(pipeFds[0], STDIN_FILENO) < 0)
                    _exit(126);
                ::close(pipeFds[0]);

                ::execv(ffmpegPath.c_str(), argv.data());
                _exit(127);
            }

            ::close(pipeFds[0]);
            if (childPid_ < 0) {
                ::close(pipeFds[1]);
                childPid_ = -1;
                setError(error, std::string("Could not start H.264 encoder: ") + std::strerror(errno));
                return false;
            }

            writeFd_ = pipeFds[1];
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = false;
            queue_.clear();
            encodeQueue_.clear();
            copyingDone_ = false;
            framePool_.clear();
        }
        active_.store(true, std::memory_order_release);
        worker_ = std::thread(&H264PreviewRecorder::encoderLoop, this);
        copyWorkers_.clear();
        copyWorkers_.reserve(copyThreadCount_);
        for (unsigned int i = 0; i < copyThreadCount_; ++i)
            copyWorkers_.emplace_back(&H264PreviewRecorder::copyLoop, this);

        std::cerr << "ApertarCore H.264 recording clip: "
                  << finalPath_.string()
                  << " preview=" << sizeArg
                  << " fps=" << fpsArg
                  << " backend=" << (backend_ == EncoderBackend::Libav ? "libav" : "ffmpeg-pipe")
                  << " encoder=" << encoder
                  << (hardwareDevice.empty() ? "" : " device=" + hardwareDevice)
                  << " input=staged-yuv"
                  << " copy_threads=" << copyThreadCount_
                  << (tune.empty() ? " tune=none" : (" tune=" + tune))
                  << (threads.empty() ? " threads=auto" : (" threads=" + threads))
                  << " queue=" << maxQueuedFrames_;
        if (x264Encoder) {
            if (!qp.empty())
                std::cerr << " qp=" << qp;
            else
                std::cerr << (bitrate.empty() ? (" crf=" + crf) : (" bitrate=" + bitrate));
        }
        else
            std::cerr << " bitrate=" << (bitrate.empty() ? "80M" : bitrate);
        if (!maxrate.empty())
            std::cerr << " maxrate=" << maxrate;
        if (!bufsize.empty())
            std::cerr << " bufsize=" << bufsize;
        std::cerr << "\n";
        return true;
    }

    void stop()
    {
        stopInternal(true);
    }

    void stopAsync()
    {
        stopInternal(false);
    }

    std::string error() const { return failure_.message(); }
    bool finalizing() const { return finalizing_.load(std::memory_order_acquire); }

    bool active() const
    {
        return active_.load(std::memory_order_acquire);
    }

    bool timestamped() const { return backend_ == EncoderBackend::Libav; }

    bool enqueueFrame(const uint8_t *base,
                      uint32_t width,
                      uint32_t height,
                      const uint32_t planeOffsets[3],
                      const uint32_t planePitches[3],
                      int64_t timestampUs,
                      std::function<void()> releaseCallback = {})
    {
        if (!active() || !base || !planeOffsets || !planePitches)
            return false;
        if (width != width_ || height != height_)
            return false;

        PreviewJob job;
        job.base = base;
        job.width = width;
        job.height = height;
        std::copy(planeOffsets, planeOffsets + 3, job.planeOffsets.begin());
        std::copy(planePitches, planePitches + 3, job.planePitches.begin());
        job.release = std::move(releaseCallback);
        job.timestampUs = timestampUs;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_) {
                const uint64_t skipped = skippedStoppingFrames_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (skipped == 1 || (skipped % 30) == 0) {
                    std::cerr << "ApertarCore H.264 skip[stopping]: total="
                              << skipped << "\n";
                }
                return false;
            }
            const size_t pendingFrames = queue_.size() + encodeQueue_.size();
            if (pendingFrames >= maxQueuedFrames_) {
                const uint64_t dropped = droppedFrames_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (dropWarning_) dropWarning_->markDropped();
                if (dropped == 1 || (dropped % 30) == 0) {
                    std::cerr << "ApertarCore H.264 drop[queue-full]: total="
                              << dropped << "\n";
                }
                return false;
            }
            queue_.push_back(std::move(job));
            updateAtomicMax(peakQueueDepth_, static_cast<uint64_t>(pendingFrames + 1));
            framesQueued_.fetch_add(1, std::memory_order_relaxed);
        }
        cv_.notify_one();
        return true;
    }

private:
    enum class EncoderBackend {
        Pipe,
        Libav,
    };

    static size_t yuv420FrameBytes(uint32_t width, uint32_t height)
    {
        const size_t yBytes = static_cast<size_t>(width) * height;
        const size_t uvBytes = static_cast<size_t>(width / 2u) * (height / 2u);
        return yBytes + (uvBytes * 2u);
    }

    static void setError(std::string *error, const std::string &message)
    {
        if (error)
            *error = message;
    }

    static std::string findFfmpegPath()
    {
        static const char *paths[] = {
            "/usr/bin/ffmpeg",
            "/bin/ffmpeg",
            "/usr/local/bin/ffmpeg",
        };
        for (const char *path : paths) {
            if (::access(path, X_OK) == 0)
                return path;
        }
        return {};
    }

    static std::string frameRateArg(double fps)
    {
        std::ostringstream out;
        out << std::fixed << std::setprecision(3) << std::max(1.0, fps);
        return out.str();
    }

    static std::string h264EncoderName()
    {
        if (recordingCapabilities().hardwareH264)
            return "h264_v4l2m2m";
        if (const char *value = std::getenv("APERTAR_H264_ENCODER"); value && *value)
            return value;
        return "libx264";
    }

    static std::string h264PresetName()
    {
        if (const char *value = std::getenv("APERTAR_H264_PRESET"); value && *value)
            return value;
        return "ultrafast";
    }

    static std::string h264TuneName()
    {
        if (const char *value = std::getenv("APERTAR_H264_TUNE"); value && *value) {
            std::string normalized;
            for (const char ch : std::string(value))
                normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
            if (normalized == "none" ||
                normalized == "off" ||
                normalized == "false" ||
                normalized == "0")
                return {};
            return value;
        }
        return "zerolatency";
    }

    static std::string h264CrfValue()
    {
        if (const char *value = std::getenv("APERTAR_H264_CRF"); value && *value)
            return value;
        return "18";
    }

    static std::string h264QpValue()
    {
        if (const char *value = std::getenv("APERTAR_H264_QP"); value && *value)
            return value;
        return {};
    }

    static std::string h264BitrateValue()
    {
        if (const char *value = std::getenv("APERTAR_H264_BITRATE"); value && *value)
            return value;
        return recordingCapabilities().hardwareH264 ? "20M" : "";
    }

    static std::string h264MaxrateValue()
    {
        if (const char *value = std::getenv("APERTAR_H264_MAXRATE"); value && *value)
            return value;
        return {};
    }

    static std::string h264BufsizeValue()
    {
        if (const char *value = std::getenv("APERTAR_H264_BUFSIZE"); value && *value)
            return value;
        return {};
    }

    static std::string h264X264ParamsValue()
    {
        if (const char *value = std::getenv("APERTAR_H264_X264_PARAMS"); value && *value)
            return value;
        return {};
    }

    static std::string h264BackendPreference()
    {
        std::string normalized = "auto";
        if (const char *value = std::getenv("APERTAR_H264_BACKEND"); value && *value)
            normalized = value;

        for (char &ch : normalized)
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

        if (normalized == "libav" ||
            normalized == "avcodec" ||
            normalized == "inprocess" ||
            normalized == "in-process")
            return "libav";
        if (normalized == "pipe" ||
            normalized == "ffmpeg" ||
            normalized == "external")
            return "pipe";
        return "auto";
    }

    static bool isX264Encoder(const std::string &encoder)
    {
        std::string normalized;
        normalized.reserve(encoder.size());
        for (const char ch : encoder)
            normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
        return normalized == "libx264" || normalized == "libx264rgb";
    }

    static std::string h264ThreadCountValue()
    {
        if (const char *value = std::getenv("APERTAR_H264_THREADS"); value && *value) {
            std::string normalized;
            for (const char ch : std::string(value))
                normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
            if (normalized == "auto" || normalized == "0")
                return {};

            char *end = nullptr;
            const unsigned long parsed = std::strtoul(value, &end, 10);
            if (end && *end == '\0' && parsed > 0)
                return std::to_string(std::min<unsigned long>(parsed, 16ul));
        }

        const unsigned int cores = std::thread::hardware_concurrency();
        if (cores <= 2)
            return "1";
        return std::to_string(std::min(4u, cores - 1u));
    }

    static int h264PipeSizeBytes()
    {
        if (const char *value = std::getenv("APERTAR_H264_PIPE_BYTES"); value && *value) {
            char *end = nullptr;
            const unsigned long parsed = std::strtoul(value, &end, 10);
            if (end && *end == '\0' && parsed >= 65536ul)
                return static_cast<int>(std::min<unsigned long>(parsed, 16ul * 1024ul * 1024ul));
        }
        return 1024 * 1024;
    }

    static size_t h264QueueDepth()
    {
        if (const char *value = std::getenv("APERTAR_H264_QUEUE_FRAMES"); value && *value) {
            char *end = nullptr;
            const unsigned long parsed = std::strtoul(value, &end, 10);
            if (end && *end == '\0' && parsed >= 2)
                return static_cast<size_t>(std::min<unsigned long>(parsed, 16ul));
        }
        return 6;
    }

    static unsigned int h264CopyThreadCount()
    {
        // One memory-copy worker preserves capture order before timestamped encoding.
        return 1;
    }

    static void updateAtomicMax(std::atomic<uint64_t> &target, uint64_t value)
    {
        uint64_t current = target.load(std::memory_order_relaxed);
        while (value > current &&
               !target.compare_exchange_weak(current, value, std::memory_order_relaxed)) {
        }
    }

    uint64_t frameBudgetMs() const
    {
        if (!(fps_ > 0.0))
            return 33;
        return static_cast<uint64_t>(std::max(1.0, std::ceil(1000.0 / fps_)));
    }

    uint64_t copySlowThresholdMs() const
    {
        return std::max<uint64_t>(8, frameBudgetMs() / 2);
    }

    static void tunePipeSize(int fd)
    {
#ifdef F_SETPIPE_SZ
        if (fd >= 0)
            (void)::fcntl(fd, F_SETPIPE_SZ, h264PipeSizeBytes());
#else
        (void)fd;
#endif
    }

#if APERTARCORE_HAS_LIBAV
    bool startLibavEncoder(const std::string &encoder,
                           const std::string &preset,
                           const std::string &tune,
                           const std::string &crf,
                           const std::string &qp,
                           const std::string &threads,
                           const std::string &bitrate,
                           const std::string &maxrate,
                           const std::string &bufsize,
                           const std::string &x264Params,
                           std::string *error)
    {
        closeLibavEncoder();

        const std::string outputPath = tempPath_.string();
        ApertarLibavH264Config config;
        config.outputPath = outputPath.c_str();
        config.encoder = encoder.c_str();
        config.preset = preset.c_str();
        config.tune = tune.c_str();
        config.crf = crf.c_str();
        config.qp = qp.c_str();
        config.threads = threads.c_str();
        config.bitrate = bitrate.c_str();
        config.maxrate = maxrate.c_str();
        config.bufsize = bufsize.c_str();
        config.x264Params = x264Params.c_str();
        config.width = width_;
        config.height = height_;
        config.fps = fps_;
        config.freeRunTimecode = freeRunTimecode_;
        return libavEncoder_.open(config, error);
    }

    bool encodeLibavFrame(const std::vector<uint8_t> &yuv, int64_t timestampUs)
    {
        if (!libavEncoder_.isOpen() || yuv.size() < frameBytes_)
            return false;

        std::string error;
        const bool success = libavEncoder_.encode(yuv.data(), yuv.size(), timestampUs, &error);
        if (!success) failure_.set(error);
        if (!success && !error.empty())
            std::cerr << "ApertarCore H.264 libav encode failed: " << error << "\n";
        return success;
    }

    bool flushLibavEncoder()
    {
        std::string error;
        const bool success = libavEncoder_.finish(&error);
        if (!success) failure_.set(error);
        if (!success && !error.empty())
            std::cerr << "ApertarCore H.264 libav finalize failed: " << error << "\n";
        return success;
    }

    void closeLibavEncoder()
    {
        libavEncoder_.closeSession();
    }
#endif

    static bool copyYuv420Frame(const uint8_t *base,
                                uint32_t width,
                                uint32_t height,
                                const uint32_t planeOffsets[3],
                                const uint32_t planePitches[3],
                                std::vector<uint8_t> *out)
    {
        if (!base || !planeOffsets || !planePitches || !out || width < 2 || height < 2)
            return false;

        const uint32_t chromaWidth = width / 2;
        const uint32_t chromaHeight = height / 2;
        const size_t yBytes = static_cast<size_t>(width) * height;
        const size_t uvBytes = static_cast<size_t>(chromaWidth) * chromaHeight;
        const size_t frameBytes = yBytes + (uvBytes * 2u);
        if (out->size() != frameBytes)
            out->resize(frameBytes);

        uint8_t *dstY = out->data();
        uint8_t *dstU = dstY + yBytes;
        uint8_t *dstV = dstU + uvBytes;
        const uint8_t *srcY = base + planeOffsets[0];
        const uint8_t *srcU = base + planeOffsets[1];
        const uint8_t *srcV = base + planeOffsets[2];
        const uint32_t yPitch = planePitches[0] ? planePitches[0] : width;
        const uint32_t uPitch = planePitches[1] ? planePitches[1] : chromaWidth;
        const uint32_t vPitch = planePitches[2] ? planePitches[2] : chromaWidth;

        if (yPitch == width &&
            uPitch == chromaWidth &&
            vPitch == chromaWidth &&
            planeOffsets[1] == planeOffsets[0] + yBytes &&
            planeOffsets[2] == planeOffsets[1] + uvBytes) {
            std::memcpy(dstY, srcY, frameBytes);
            return true;
        }

        for (uint32_t row = 0; row < height; ++row) {
            std::memcpy(dstY + static_cast<size_t>(row) * width,
                        srcY + static_cast<size_t>(row) * yPitch,
                        width);
        }
        for (uint32_t row = 0; row < chromaHeight; ++row) {
            std::memcpy(dstU + static_cast<size_t>(row) * chromaWidth,
                        srcU + static_cast<size_t>(row) * uPitch,
                        chromaWidth);
            std::memcpy(dstV + static_cast<size_t>(row) * chromaWidth,
                        srcV + static_cast<size_t>(row) * vPitch,
                        chromaWidth);
        }
        return true;
    }

    std::vector<uint8_t> acquireFrameBuffer(size_t size)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto it = framePool_.begin(); it != framePool_.end(); ++it) {
            if (it->size() == size) {
                std::vector<uint8_t> buffer = std::move(*it);
                framePool_.erase(it);
                return buffer;
            }
        }
        return std::vector<uint8_t>(size);
    }

    void recycleFrameBufferLocked(std::vector<uint8_t> buffer)
    {
        if (buffer.empty() || buffer.size() != frameBytes_)
            return;
        if (framePool_.size() >= maxQueuedFrames_)
            return;
        framePool_.push_back(std::move(buffer));
    }

    void recycleFrameBuffer(std::vector<uint8_t> buffer)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        recycleFrameBufferLocked(std::move(buffer));
    }

    static bool writeAll(int fd, const uint8_t *data, size_t size)
    {
        const uint8_t *cursor = data;
        size_t remaining = size;
        while (remaining > 0) {
            const ssize_t written = ::write(fd, cursor, remaining);
            if (written < 0) {
                if (errno == EINTR)
                    continue;
                return false;
            }
            if (written == 0)
                return false;
            cursor += written;
            remaining -= static_cast<size_t>(written);
        }
        return true;
    }

    void closeWriteFd()
    {
        if (writeFd_ >= 0) {
            ::close(writeFd_);
            writeFd_ = -1;
        }
    }

    void waitForFinalizer()
    {
        if (finalizer_.joinable())
            finalizer_.join();
    }

    static void releasePreviewJob(PreviewJob &job)
    {
        if (!job.release)
            return;
        std::function<void()> release = std::move(job.release);
        release();
    }

    void recycleEncodeJob(EncodeJob &job)
    {
        recycleFrameBuffer(std::move(job.yuv));
    }

    void stopInternal(bool waitForFinalize)
    {
        if (finalizer_.joinable()) {
            if (waitForFinalize)
                waitForFinalizer();
            return;
        }

        const bool hadSession = worker_.joinable() || childPid_ > 0 || writeFd_ >= 0;
        if (!hadSession)
            return;

        active_.store(false, std::memory_order_release);
        std::deque<PreviewJob> cancelledJobs;
        std::deque<EncodeJob> cancelledEncodeJobs;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        for (PreviewJob &job : cancelledJobs)
            releasePreviewJob(job);
        cancelledEncodeJobs.clear();
        cv_.notify_all();
        encodeCv_.notify_all();

        finalizing_.store(true, std::memory_order_release);
        finalizer_ = std::thread(&H264PreviewRecorder::finalizeStop, this);
        if (waitForFinalize)
            waitForFinalizer();
    }

    void finalizeStop()
    {
        for (std::thread &copyWorker : copyWorkers_) {
            if (copyWorker.joinable())
                copyWorker.join();
        }
        copyWorkers_.clear();

        if (worker_.joinable())
            worker_.join();

        closeWriteFd();

        int status = 0;
        if (childPid_ > 0) {
            while (::waitpid(childPid_, &status, 0) < 0 && errno == EINTR) {
            }
        }

        std::error_code ec;
        if (!tempPath_.empty() && std::filesystem::exists(tempPath_, ec)) {
            ec.clear();
            if (std::filesystem::file_size(tempPath_, ec) > 0) {
                ec.clear();
                std::filesystem::rename(tempPath_, finalPath_, ec);
                if (ec) {
                    std::filesystem::copy_file(tempPath_,
                                               finalPath_,
                                               std::filesystem::copy_options::overwrite_existing,
                                               ec);
                    if (ec) failure_.set("Could not finalize the H.264 clip: " + ec.message());
                    else {
                        std::error_code removeEc;
                        std::filesystem::remove(tempPath_, removeEc);
                    }
                }
            }
        }

        if (WIFEXITED(status) && WEXITSTATUS(status) != 0) {
            failure_.set("H.264 encoder exited with code " + std::to_string(WEXITSTATUS(status)));
            std::cerr << "ApertarCore H.264 encoder exited with code "
                      << WEXITSTATUS(status) << "\n";
        } else if (WIFSIGNALED(status)) {
            failure_.set("H.264 encoder was terminated by signal " + std::to_string(WTERMSIG(status)));
            std::cerr << "ApertarCore H.264 encoder stopped by signal "
                      << WTERMSIG(status) << "\n";
        }

        const uint64_t queued = framesQueued_.load(std::memory_order_relaxed);
        const uint64_t written = framesWritten_.load(std::memory_order_relaxed);
        const uint64_t dropped = droppedFrames_.load(std::memory_order_relaxed);
        const uint64_t skippedStopping = skippedStoppingFrames_.load(std::memory_order_relaxed);
        const uint64_t failures = writeFailures_.load(std::memory_order_relaxed);
        const uint64_t peakQueue = peakQueueDepth_.load(std::memory_order_relaxed);
        const uint64_t maxCopyMs = maxCopyMs_.load(std::memory_order_relaxed);
        const uint64_t slowCopies = slowCopies_.load(std::memory_order_relaxed);
        const uint64_t maxWriteMs = maxWriteMs_.load(std::memory_order_relaxed);
        const uint64_t slowWrites = slowWrites_.load(std::memory_order_relaxed);
        if (queued > 0 || written > 0 || dropped > 0 || skippedStopping > 0 || failures > 0) {
            std::cerr << "ApertarCore H.264 recording stats: "
                      << "queued=" << queued
                      << " written=" << written
                      << " dropped_queue_full=" << dropped
                      << " skipped_stopping=" << skippedStopping
                      << " write_failures=" << failures
                      << " peak_queue=" << peakQueue << "/" << maxQueuedFrames_
                      << " max_copy_ms=" << maxCopyMs
                      << " slow_copies=" << slowCopies
                      << " max_write_ms=" << maxWriteMs
                      << " slow_writes=" << slowWrites
                      << "\n";
        }

        std::deque<PreviewJob> cancelledJobs;
        std::deque<EncodeJob> cancelledEncodeJobs;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cancelledJobs.swap(queue_);
            cancelledEncodeJobs.swap(encodeQueue_);
            framePool_.clear();
            stopping_ = false;
        }
        for (PreviewJob &job : cancelledJobs)
            releasePreviewJob(job);
        cancelledEncodeJobs.clear();
        childPid_ = -1;
        folderPath_.clear();
        finalPath_.clear();
        tempPath_.clear();
        clipName_.clear();
        width_ = 0;
        height_ = 0;
        frameBytes_ = 0;
        fps_ = 0.0;
        finalizing_.store(false, std::memory_order_release);
    }

    void copyLoop()
    {
        while (true) {
            PreviewJob job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this]() {
                    return stopping_ || !queue_.empty();
                });

                if (queue_.empty()) {
                    if (stopping_)
                        break;
                    continue;
                }

                job = std::move(queue_.front());
                queue_.pop_front();
            }

            EncodeJob encodeJob;
            encodeJob.timestampUs = job.timestampUs;
            const auto copyStart = std::chrono::steady_clock::now();
            encodeJob.yuv = acquireFrameBuffer(frameBytes_);
            const bool copied = copyYuv420Frame(job.base,
                                                job.width,
                                                job.height,
                                                job.planeOffsets.data(),
                                                job.planePitches.data(),
                                                &encodeJob.yuv);
            const uint64_t copyMs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - copyStart).count());
            updateAtomicMax(maxCopyMs_, copyMs);
            if (copyMs > copySlowThresholdMs())
                slowCopies_.fetch_add(1, std::memory_order_relaxed);

            releasePreviewJob(job);
            if (!copied) {
                writeFailures_.fetch_add(1, std::memory_order_relaxed);
                failure_.set("Could not copy the H.264 recording frame.");
                active_.store(false, std::memory_order_release);
                break;
            }

            bool queuedForEncode = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (encodeQueue_.size() < maxQueuedFrames_) {
                    encodeQueue_.push_back(std::move(encodeJob));
                    updateAtomicMax(peakQueueDepth_, static_cast<uint64_t>(queue_.size() + encodeQueue_.size()));
                    queuedForEncode = true;
                }
            }
            if (queuedForEncode) {
                encodeCv_.notify_one();
            } else {
                const uint64_t dropped = droppedFrames_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (dropWarning_) dropWarning_->markDropped();
                if (dropped == 1 || (dropped % 30) == 0) {
                    std::cerr << "ApertarCore H.264 drop[queue-full]: total="
                              << dropped << "\n";
                }
                recycleEncodeJob(encodeJob);
            }
        }
        { std::lock_guard lock(mutex_); copyingDone_ = true; }
        encodeCv_.notify_all();
    }

    void encoderLoop()
    {
        while (true) {
            EncodeJob job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                encodeCv_.wait(lock, [this]() {
                    return copyingDone_ || !encodeQueue_.empty();
                });

                if (encodeQueue_.empty()) {
                    if (copyingDone_)
                        break;
                    continue;
                }

                job = std::move(encodeQueue_.front());
                encodeQueue_.pop_front();
            }

            const auto writeStart = std::chrono::steady_clock::now();
            bool wroteFrame = false;
#if APERTARCORE_HAS_LIBAV
            if (backend_ == EncoderBackend::Libav)
                wroteFrame = encodeLibavFrame(job.yuv, job.timestampUs);
            else
#endif
                wroteFrame = writeFd_ >= 0 && writeAll(writeFd_, job.yuv.data(), job.yuv.size());
            const uint64_t writeMs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - writeStart).count());
            updateAtomicMax(maxWriteMs_, writeMs);
            if (writeMs > frameBudgetMs())
                slowWrites_.fetch_add(1, std::memory_order_relaxed);

            if (!wroteFrame) {
                writeFailures_.fetch_add(1, std::memory_order_relaxed);
                failure_.set("H.264 encoding or media writing failed.");
                active_.store(false, std::memory_order_release);
                std::deque<PreviewJob> cancelledJobs;
                std::deque<EncodeJob> cancelledEncodeJobs;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    stopping_ = true;
                    cancelledJobs.swap(queue_);
                    cancelledEncodeJobs.swap(encodeQueue_);
                }
                for (PreviewJob &cancelledJob : cancelledJobs)
                    releasePreviewJob(cancelledJob);
                cancelledEncodeJobs.clear();
                cv_.notify_all();
                encodeCv_.notify_all();
                recycleEncodeJob(job);
                break;
            }
            framesWritten_.fetch_add(1, std::memory_order_relaxed);
            if (frameWritten_) frameWritten_(job.timestampUs);
            recycleEncodeJob(job);
        }

#if APERTARCORE_HAS_LIBAV
        if (backend_ == EncoderBackend::Libav) {
            if (!flushLibavEncoder())
                writeFailures_.fetch_add(1, std::memory_order_relaxed);
            closeLibavEncoder();
            return;
        }
#endif
        closeWriteFd();
    }

    RecordingFailure failure_;
    std::function<void(int64_t)> frameWritten_;
    bool freeRunTimecode_ = false;
    bool copyingDone_ = false;
    std::atomic<bool> finalizing_{false};
    std::atomic<bool> active_{false};
    std::atomic<uint64_t> framesQueued_{0};
    std::atomic<uint64_t> framesWritten_{0};
    std::atomic<uint64_t> droppedFrames_{0};
    RecordingDropWarning *dropWarning_ = nullptr;
    std::atomic<uint64_t> skippedStoppingFrames_{0};
    std::atomic<uint64_t> writeFailures_{0};
    std::atomic<uint64_t> peakQueueDepth_{0};
    std::atomic<uint64_t> maxCopyMs_{0};
    std::atomic<uint64_t> slowCopies_{0};
    std::atomic<uint64_t> maxWriteMs_{0};
    std::atomic<uint64_t> slowWrites_{0};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::condition_variable encodeCv_;
    std::deque<PreviewJob> queue_;
    std::deque<EncodeJob> encodeQueue_;
    std::vector<std::vector<uint8_t>> framePool_;
    std::vector<std::thread> copyWorkers_;
    std::thread worker_;
    std::thread finalizer_;
    std::filesystem::path folderPath_;
    std::filesystem::path finalPath_;
    std::filesystem::path tempPath_;
    std::string clipName_;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    size_t frameBytes_ = 0;
    double fps_ = 0.0;
    pid_t childPid_ = -1;
    int writeFd_ = -1;
    bool stopping_ = false;
    size_t maxQueuedFrames_ = 6;
    unsigned int copyThreadCount_ = 2;
    EncoderBackend backend_ = EncoderBackend::Pipe;
#if APERTARCORE_HAS_LIBAV
    LibavH264EncoderClient libavEncoder_;
#endif
};

class PhotoCaptureWriter {
public:
    void setErrorCallback(std::function<void(const std::string &)> callback) { errorCallback_ = std::move(callback); }
    void setDrainedCallback(std::function<void()> callback) { drainedCallback_ = std::move(callback); }
    std::string error() const { return failure_.message(); }
    bool busy() const { return pending_.load(std::memory_order_acquire) != 0; }
    ~PhotoCaptureWriter()
    {
        stop();
    }

    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        cv_.notify_all();

        if (worker_.joinable())
            worker_.join();

        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        stopping_ = false;
    }

    bool enqueue(std::vector<uint8_t> rawBytes,
                 RawFrameView frame,
                 CdngEncoder::Metadata metadata,
                 std::string basePath,
                 PhotoCaptureFormat format,
                 StillRenderSettings rendering,
                 std::function<void()> onSaved)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_) return false;
        if (!worker_.joinable()) {
            stopping_ = false;
            worker_ = std::thread(&PhotoCaptureWriter::workerLoop, this);
        }

        if (queue_.size() >= maxQueuedFrames_) {
            ++droppedFrames_;
            if (droppedFrames_ == 1 || (droppedFrames_ % 10) == 0) {
                std::cerr << "ApertarCore photo drop[queue-full]: total="
                          << droppedFrames_ << "\n";
            }
            return false;
        }

        if (pending_.load(std::memory_order_acquire) == 0) failure_.reset();
        queue_.push_back(FrameJob{
            std::move(rawBytes),
            frame,
            std::move(metadata),
            std::move(basePath),
            format,
            std::move(rendering),
            std::move(onSaved),
        });
        ++pending_;
        cv_.notify_one();
        return true;
    }

private:
    struct FrameJob {
        std::vector<uint8_t> rawBytes;
        RawFrameView frame;
        CdngEncoder::Metadata metadata;
        std::string basePath;
        PhotoCaptureFormat format = PhotoCaptureFormat::Dng;
        StillRenderSettings rendering;
        std::function<void()> onSaved;
    };

    bool writeDngStill(CdngEncoder &encoder, FrameJob &job, const std::filesystem::path &finalPath)
    {
        job.frame.data = job.rawBytes.data();
        std::vector<uint8_t> dng16Bytes;
        RawFrameView dngFrame;
        CdngEncoder::Metadata dngMetadata;
        std::string dngError;
        if (!prepare16BitDngStillFrame(job.frame,
                                       job.metadata,
                                       &dng16Bytes,
                                       &dngFrame,
                                       &dngMetadata,
                                       &dngError)) {
            throw std::runtime_error(dngError.empty()
                                         ? "could not prepare 16-bit DNG still"
                                         : dngError);
        }

        AtomicStillFile output(finalPath);
        encoder.writeDngFile(output.path(), dngFrame, dngMetadata, output.fd());
        const uint64_t minimumSize = dngMetadata.compression == CdngCompression::Uncompressed
            ? uint64_t(dngFrame.width) * dngFrame.height * 2 + 8 : 16;
        output.commit(minimumSize);
        return true;
    }

    bool writeProcessedStill(FrameJob &job,
                             const std::filesystem::path &finalPath,
                             PhotoCaptureFormat format,
                             std::string *error)
    {
        job.frame.data = job.rawBytes.data();
        ProcessedRgbStill image;
        if (!processRawFrameToRgbStill(job.frame, job.metadata, job.rendering, &image, error))
            return false;

        std::vector<uint8_t> encoded;
        bool encodedOk = false;
        if (format == PhotoCaptureFormat::Jpeg || format == PhotoCaptureFormat::DngJpeg)
            encodedOk = encodeProcessedJpeg(image, &encoded);
        else if (format == PhotoCaptureFormat::Png || format == PhotoCaptureFormat::DngPng)
            encodedOk = encodeProcessedPng(image, &encoded);
        else if (format == PhotoCaptureFormat::Tiff || format == PhotoCaptureFormat::DngTiff)
            encodedOk = encodeProcessedTiff(image, &encoded);

        if (!encodedOk) {
            setStillError(error, "Could not encode processed still image.");
            return false;
        }
        return writeBytesAtomically(finalPath, encoded, error);
    }

    void workerLoop()
    {
        CdngEncoder encoder;

        while (true) {
            FrameJob job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this]() {
                    return stopping_ || !queue_.empty();
                });

                if (queue_.empty()) {
                    if (stopping_)
                        return;
                    continue;
                }

                job = std::move(queue_.front());
                queue_.pop_front();
            }

            bool saved = false;
            try {
                const std::filesystem::path basePath(job.basePath);
                if (photoFormatWritesDng(job.format))
                    writeDngStill(encoder, job, withPhotoExtension(basePath, ".dng"));

                if (photoFormatWritesPreview(job.format)) {
                    if (job.rawBytes.empty()) {
                        throw std::runtime_error("processed still requested without a raw frame");
                    }

                    const char *extension = ".jpg";
                    if (job.format == PhotoCaptureFormat::Png ||
                        job.format == PhotoCaptureFormat::DngPng) {
                        extension = ".png";
                    } else if (job.format == PhotoCaptureFormat::Tiff ||
                               job.format == PhotoCaptureFormat::DngTiff) {
                        extension = ".tif";
                    }

                    std::string encodeError;
                    if (!writeProcessedStill(job,
                                             withPhotoExtension(basePath, extension),
                                             job.format,
                                             &encodeError)) {
                        throw std::runtime_error(encodeError.empty()
                                                     ? "processed still write failed"
                                                     : encodeError);
                    }
                }
                saved = true;
            } catch (const std::exception &e) {
                std::cerr << "ApertarCore photo write failed for "
                          << job.basePath << ": " << e.what() << "\n";
                const bool firstFailure = failure_.message().empty();
                failure_.set(e.what());
                if (firstFailure && errorCallback_) errorCallback_(e.what());
            }
            if (saved && job.onSaved) job.onSaved();
            if (pending_.fetch_sub(1, std::memory_order_acq_rel) == 1 && drainedCallback_)
                drainedCallback_();
        }
    }

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<FrameJob> queue_;
    std::thread worker_;
    uint64_t droppedFrames_ = 0;
    bool stopping_ = false;
    std::atomic<unsigned> pending_{0};
    const size_t maxQueuedFrames_ = 12;
    RecordingFailure failure_;
    std::function<void(const std::string &)> errorCallback_;
    std::function<void()> drainedCallback_;
};

#endif

} // namespace

class CameraController::CameraSession {
public:
    explicit CameraSession(PreviewCallback previewCallback,
                           AudioLevelCallback audioLevelCallback,
                           RecordingDropWarning &dropWarning, EventCallback exposureEvent)
        :
#if APERTARCORE_HAS_LIBCAMERA
          recorder_(&dropWarning), h264Recorder_(&dropWarning), proresRecorder_(&dropWarning),
#endif
          previewCallback_(std::move(previewCallback))
        , audioLevelCallback_(std::move(audioLevelCallback))
        , dropWarning_(dropWarning)
        , exposureEvent_(std::move(exposureEvent))
    {
#if APERTARCORE_HAS_LIBCAMERA
        photoWriter_.setDrainedCallback([this] { emitMediaWriteState(); });
        photoWriter_.setErrorCallback([this](const std::string &error) {
            if (exposureEvent_)
                exposureEvent_("{\"event\":\"photo_write_error\",\"photo_write_error\":\"" +
                    apertar::escapeJson(error) + "\"}");
        });
#endif
    }

    bool start(const CaptureSettings &settings, std::string *error)
    {
#if APERTARCORE_HAS_LIBCAMERA
        settings_ = settings;
        {
            std::lock_guard<std::mutex> lock(whiteBalanceMutex_);
            whiteBalanceMeasured_ = {};
            measuredExposureUs_ = 0;
            measuredAnalogueGain_ = 0;
            measuredCcm_.reset();
        }

        const bool automaticTuning = settings.tuningFile.empty();
        const auto tuningPipeline = tuning::nativePipeline();
        const auto tuningDirectories = tuning::defaultDirectories(tuningPipeline);
        std::string tuningPath = automaticTuning
            ? tuning::preselectDefault(tuningDirectories, tuningPipeline) : settings.tuningFile;
        applyProcessTuningFile(tuningPath);
        const auto ispProfile = activeIspColorProfile(settings);
        const bool logPreview = ispProfile != ProresColorProfile::Standard;
        bool derivedTuningApplied = false;
        std::shared_ptr<PreviewExposureReference> exposureReference;

        // Usually one pass: sysfs identifies the sensor before the IPA starts.
        // Multiple sensors need one restart after the actual camera is selected.
        for (unsigned attempt = 0; attempt < 3; ++attempt) {
            cameraManager_ = std::make_unique<libcamera::CameraManager>();
            if (const int ret = cameraManager_->start(); ret != 0) {
                setError(error, "libcamera CameraManager failed to start, code " + std::to_string(ret));
                cameraManager_.reset();
                return false;
            }

            std::vector<std::shared_ptr<libcamera::Camera>> cameras = cameraManager_->cameras();
            if (cameras.empty()) {
                setError(error, "No libcamera cameras are available.");
                cameraManager_->stop();
                cameraManager_.reset();
                return false;
            }

            auto selected = std::find_if(cameras.begin(), cameras.end(), [](const auto &camera) {
                return camera && camera->id().find("/usb") == std::string::npos;
            });
            camera_ = selected != cameras.end() ? *selected : cameras.front();

            if (!camera_) {
                setError(error, "Failed to select a libcamera camera.");
                stop();
                return false;
            }

            monochromeSensor_ = false;
            bool haveCfaProperty = false;
#if defined(LIBCAMERA_HAS_DRAFT_VENDOR_PROPERTIES_COLOR_FILTER_ARRANGEMENT)
            if (auto cfa = camera_->properties().get(libcamera::properties::draft::ColorFilterArrangement)) {
                haveCfaProperty = true;
                monochromeSensor_ = *cfa == libcamera::properties::draft::MONO;
            }
#endif
            if (!haveCfaProperty) {
                const auto raw = camera_->generateConfiguration({libcamera::StreamRole::Raw});
                RawBitDepth depth;
                PixelPacking packing;
                BayerPattern pattern;
                if (raw && raw->size() > 0 && formatToRawView(raw->at(0).pixelFormat, &depth, &packing, &pattern))
                    monochromeSensor_ = pattern == BayerPattern::Monochrome;
            }
            if (automaticTuning) {
                const auto model = camera_->properties().get(libcamera::properties::Model);
                const std::string defaultPath = model
                    ? tuning::defaultForSensor(std::string(*model), tuningDirectories, tuningPipeline,
                                               monochromeSensor_) : std::string{};
                if (defaultPath != tuningPath) {
                    if (attempt != 0 || derivedTuningApplied) {
                        setError(error, "Camera selection changed while resolving default tuning.");
                        cameras.clear();
                        stop();
                        return false;
                    }
                    cameras.clear();
                    camera_.reset();
                    cameraManager_->stop();
                    cameraManager_.reset();
                    tuningPath = defaultPath;
                    applyProcessTuningFile(tuningPath);
                    continue;
                }
                if (tuningPath.empty())
                    std::cerr << "ApertarCore no readable default tuning for "
                              << model.value_or("unknown sensor") << " on " << tuning::pipelineName(tuningPipeline)
                              << "; using libcamera fallback.\n";
            }
            if (logPreview && !derivedTuningApplied) {
                std::string derivedPath, why;
                exposureReference=std::make_shared<PreviewExposureReference>();
                if (!makeProresIspTuning(tuningPath, ispProfile, monochromeSensor_, derivedPath, why, exposureReference.get())) {
                    setError(error, why);
                    cameras.clear();
                    stop();
                    return false;
                }
                cameras.clear();
                camera_.reset();
                cameraManager_->stop();
                cameraManager_.reset();
                ::setenv("LIBCAMERA_RPI_TUNING_FILE", derivedPath.c_str(), 1);
                derivedTuningApplied = true;
                std::cerr << "ApertarCore " << (settings.recordingFormat == RecordingFormat::Prores ? "ProRes colour: " : "cDNG preview colour: ")
                          << proresColorSpace(ispProfile) << " / " << proresGamma(ispProfile)
                          << " (ISP curve approximation)\n";
                continue;
            }
            break;
        }

        std::cerr << "ApertarCore tuning file: "
                  << (tuningPath.empty() ? "libcamera default" : tuningPath)
                  << (automaticTuning ? " (sensor default, " : " (custom, ")
                  << tuning::pipelineName(tuningPipeline) << ")\n";

        sensorFamily_ = detectSensorFamily(camera_->id());
        settings_ = sanitizeSettingsForSensor(settings, sensorFamily_);

        if (camera_->acquire() != 0) {
            setError(error, "Failed to acquire camera " + cameraIdForLog(camera_) + ".");
            stop();
            return false;
        }
        acquired_ = true;

        CaptureSettings configuredSettings=settings;
        configuredSettings.previewExposureReference=exposureReference;
        if (!configureAndStart(configuredSettings, error)) {
            stop();
            return false;
        }

        return true;
#else
        (void)settings;
        setError(error, "ApertarCore was built without libcamera support.");
        return false;
#endif
    }

    bool hasPendingPhotoCaptures()
    {
#if APERTARCORE_HAS_LIBCAMERA
        std::lock_guard<std::mutex> lock(photoCaptureMutex_);
        return stillExposureActive_ || !pendingPhotoCaptures_.empty() ||
            photoFramesInFlight_.load(std::memory_order_acquire) || photoWriter_.busy() ||
            recordingStopInProgress_.load(std::memory_order_acquire) || h264Recorder_.finalizing();
#else
        return false;
#endif
    }

    MediaWriteStatus mediaWriteStatus()
    {
        std::lock_guard lock(mediaWriteEventMutex_);
        return {hasPendingPhotoCaptures(), mediaWriteRevision_};
    }

    void stop()
    {
#if APERTARCORE_HAS_LIBCAMERA
        { std::lock_guard lock(recordingFrameMutex_); recordingFramesEnabled_ = false; }
        running_ = false;

        if (camera_) {
            camera_->requestCompleted.disconnect(this, &CameraSession::requestComplete);
            if (started_)
                camera_->stop();
            if (longExposureTimingUsed_)
                resetStoppedSensorTiming();
        }

        {
            std::lock_guard<std::mutex> lock(controlsMutex_);
            pendingControls_.clear();
        }

        gyroRecorder_.stop();
        h264Recorder_.stop();
        proresRecorder_.stop();
        recorder_.stop();
        audioRecorder_.stop();
        photoWriter_.stop();
        bool cancelledStillExposure = false;
        {
            std::lock_guard<std::mutex> photoLock(photoCaptureMutex_);
            pendingPhotoCaptures_.clear();
            cancelledStillExposure = stillExposureActive_;
            stillExposureActive_ = false;
            restoreStillPreviewControls_ = false;
        }
        if (cancelledStillExposure)
            emitStillExposureEvent();
        emitMediaWriteState();
        unmapRawBuffers();
        unmapPreviewBuffers();
        started_ = false;
        requests_.clear();
        publishedPreviewSlots_.clear();
        cachedProresBuffers_.clear();
        cachedRawBuffers_.clear();
        allocator_.reset();
        previewStream_ = nullptr;
        h264Stream_ = nullptr;
        rawStream_ = nullptr;
        rawStreamInfo_ = {};
        configuration_.reset();

        if (camera_ && acquired_) {
            camera_->release();
            acquired_ = false;
        }
        camera_.reset();

        if (cameraManager_) {
            cameraManager_->stop();
            cameraManager_.reset();
        }
#endif
    }

    bool reconfigure(const CaptureSettings &settings, std::string *error, bool forPhoto = false)
    {
#if APERTARCORE_HAS_LIBCAMERA
        stop();
        photoStreamOverride_ = forPhoto;
        return start(settings, error);
#else
        (void)settings;
        (void)error;
        (void)forPhoto;
        return true;
#endif
    }

    bool applyControls(const CaptureSettings &settings, std::string *error)
    {
#if APERTARCORE_HAS_LIBCAMERA
        if (!camera_) {
            setError(error, "Camera is not open.");
            return false;
        }

        CaptureSettings sanitized = sanitizeSettingsForSensor(settings, sensorFamily_);
        sanitized.whiteBalanceAvailable = !monochromeSensor_ && activeTuningHasCalibratedWhiteBalance();
        constrainExposureMetering(sanitized, exposureMeteringModes());
        longExposureTimingUsed_ |= sanitized.photoModeEnabled && sanitized.longExposureEnabled;

        {
            std::unique_lock settingsLock(sessionSettingsMutex_);
            std::lock_guard<std::mutex> lock(controlsMutex_);
            pendingControls_ = controlsForSettings(sanitized,
                                                   sensorFamily_,
                                                   camera_->controls(),
                                                   Resolution{ configuredSensorWidth_, configuredSensorHeight_ },
                                                   camera_->properties(), monochromeSensor_);
            settings_ = sanitized;
        }
        return true;
#else
        (void)settings;
        (void)error;
        return true;
#endif
    }

    bool startRecording(const std::string &mediaRoot,
                        const CaptureSettings &settings,
                        const AudioRecordingOptions &audioOptions,
                        const GyroRecordingOptions &gyroOptions,
                        std::string *error)
    {
#if APERTARCORE_HAS_LIBCAMERA
        if (photoStreamOverride_ && !reconfigure(settings, error))
            return false;
        if (monochromeSensor_ && settings.recordingFormat == RecordingFormat::Mlv) {
            setError(error, "Monochrome MLV is not supported. Select cDNG or H.264 instead.");
            return false;
        }
        recordingClock_ = RecordingClock();
        recordingGapEstimate_ = 0;
        haveLastRecordingSensorTimestamp_ = false;
        lastRecordingSensorTimestampUs_ = 0;
        h264FrameCounter_.store(0, std::memory_order_relaxed);
        requeueFailures_ = 0;

        std::filesystem::path clipFolder;
        try {
            clipFolder = DngRecordingQueue::reserveClipFolder(mediaRoot, settings);
            std::filesystem::create_directories(clipFolder);
        } catch (const std::exception &e) {
            setError(error, e.what());
            return false;
        }

        if (gyroOptions.enabled) {
            const std::string clipFolderString = clipFolder.string();
            if (clipFolderString.empty()) {
                setError(error, "Gyro recording requires a valid clip folder.");
                return false;
            }

            std::string gyroError;
            if (!gyroRecorder_.start(clipFolderString, gyroOptions, &gyroError)) {
                std::cerr << "ApertarCore gyro recording disabled for this clip: "
                          << (gyroError.empty() ? "unknown gyro startup failure" : gyroError)
                          << "\n";
            }
        }

        if (audioOptions.enabled) {
            const std::string clipFolderString = clipFolder.string();
            if (clipFolderString.empty()) {
                gyroRecorder_.stop();
                setError(error, "Audio recording requires a valid clip folder.");
                return false;
            }

            AudioRecordingOptions resolvedAudioOptions = audioOptions;
            resolvedAudioOptions.fps = settings.fps;
            resolvedAudioOptions.timecodeMode = settings.timecodeMode;
            std::string audioError;
            if (!audioRecorder_.start(clipFolder, resolvedAudioOptions, audioLevelCallback_, &audioError, &recordingClock_)) {
                gyroRecorder_.stop();
                setError(error, "Audio recording could not start: " + audioError);
                return false;
            }
        }

        if (isProcessedVideoFormat(settings.recordingFormat)) {
            const bool proresFormat = settings.recordingFormat == RecordingFormat::Prores;
            const char *formatName = proresFormat ? "ProRes" : "H.264";
            libcamera::Stream *encodeStream = h264Stream_ ? h264Stream_ : previewStream_;
            if (!encodeStream) {
                audioRecorder_.stop();
                gyroRecorder_.stop();
                setError(error, std::string(formatName) + " recording requires an ISP encode stream.");
                return false;
            }

            const libcamera::StreamConfiguration &previewConfig = encodeStream->configuration();
            if (mappedPreviewBuffers_.empty()) {
                audioRecorder_.stop();
                gyroRecorder_.stop();
                setError(error, std::string(formatName) + " recording could not mmap preview buffers.");
                return false;
            }

            if (proresFormat) {
                if (!proresRecorder_.startAtFolder(clipFolder, settings, previewConfig.size.width,
                                                  previewConfig.size.height, error, mappedPreviewBuffers_.size())) {
                    audioRecorder_.stop(); gyroRecorder_.stop(); return false;
                }
                proresFrameCounter_.store(0, std::memory_order_relaxed);
            } else {
                if (!h264Recorder_.startAtFolder(clipFolder,
                                                 settings,
                                                 previewConfig.size.width,
                                                 previewConfig.size.height,
                                                 error, [this](int64_t timestampUs) {
                                                     audioRecorder_.observeVideoFrame(timestampUs, 0);
                                                 })) {
                    audioRecorder_.stop();
                    gyroRecorder_.stop();
                    return false;
                }
                h264FrameCounter_.store(0, std::memory_order_relaxed);
                if (audioOptions.enabled && !h264Recorder_.timestamped()) {
                    h264Recorder_.stop(); audioRecorder_.stop(); gyroRecorder_.stop();
                    setError(error, "Synchronized H.264 audio requires the libav backend; remove APERTAR_H264_BACKEND=pipe.");
                    return false;
                }
            }
            lastRecordingFormat_ = settings.recordingFormat;
            dropWarning_.reset();
            { std::lock_guard lock(recordingFrameMutex_); recordingStartBootUs_ = clockMicroseconds(CLOCK_BOOTTIME); waitingForRecordingCadence_ = true; recordingFramesEnabled_ = true; }
            return true;
        }

        if (!recorder_.startAtFolder(clipFolder, settings, error)) {
            audioRecorder_.stop();
            gyroRecorder_.stop();
            return false;
        }
        lastRecordingFormat_ = settings.recordingFormat;
        dropWarning_.reset();
        { std::lock_guard lock(recordingFrameMutex_); recordingStartBootUs_ = clockMicroseconds(CLOCK_BOOTTIME); waitingForRecordingCadence_ = true; recordingFramesEnabled_ = true; }
        return true;
#else
        (void)mediaRoot;
        (void)settings;
        (void)audioOptions;
        (void)gyroOptions;
        (void)error;
        return true;
#endif
    }

    void stopRecording()
    {
#if APERTARCORE_HAS_LIBCAMERA
        recordingStopInProgress_.store(true, std::memory_order_release);
        emitMediaWriteState();
        // Close the frame gate before finalizing audio, and wait for an in-flight
        // enqueue/observe pair. Video must not grow after the WAV endpoint is fixed.
        { std::lock_guard lock(recordingFrameMutex_); recordingFramesEnabled_ = false; }
        if (recordingGapEstimate_ > 0 || requeueFailures_ > 0) {
            std::cerr << "ApertarCore capture stats: "
                      << "dropped_gap_estimate=" << recordingGapEstimate_
                      << " requeue_failures=" << requeueFailures_
                      << "\n";
        }
        gyroRecorder_.stop();
        h264Recorder_.stop();
        recorder_.stop();
        proresRecorder_.stop();
        audioRecorder_.stop();
        haveLastRecordingSensorTimestamp_ = false;
        lastRecordingSensorTimestampUs_ = 0;
        h264FrameCounter_.store(0, std::memory_order_relaxed);
        recordingStopInProgress_.store(false, std::memory_order_release);
        emitMediaWriteState();
#endif
    }

    bool capturePhoto(const std::string &mediaRoot,
                      const CaptureSettings &settings,
                      PhotoCaptureFormat format,
                      std::string *error,
                      const std::string &basePathOverride = {},
                      std::function<void()> onSaved = {}, unsigned count = 1)
    {
#if APERTARCORE_HAS_LIBCAMERA
        if (!rawStream_) {
            if (!reconfigure(settings, error, true))
                return false;
            if (!rawStream_) {
                setError(error, "The camera could not configure a RAW stream for still capture.");
                return false;
            }
        }

        try {
            // Allocate names before taking the capture lock; filesystem access
            // must not hold up the camera callback.
            const bool bracket = usesStillBracketing(settings) && basePathOverride.empty();
            CaptureSettings captureSettings = settings;
            WhiteBalanceMeasurement lockedWb;
            std::optional<std::array<float, 9>> lockedCcm;
            float lockedGain = 0;
            std::array<uint32_t, 3> bracketTimes{};
            std::string bracketBase;
            if (bracket) {
                count = 3;
                uint32_t baseExposure = settings.shutterUs;
                {
                    std::lock_guard lock(whiteBalanceMutex_);
                    if (settings.shutterAuto) baseExposure = measuredExposureUs_;
                    lockedGain = measuredAnalogueGain_;
                    lockedWb = whiteBalanceMeasured_;
                    lockedCcm = measuredCcm_;
                }
                if (!baseExposure || !std::isfinite(lockedGain) || lockedGain <= 0 ||
                    (settings.whiteBalanceAuto && settings.whiteBalanceAvailable &&
                     (!lockedWb.valid() || !lockedWb.kelvin))) {
                    setError(error, "Wait for preview exposure and white balance to settle before bracketing.");
                    return false;
                }
                const auto times = stillBracketExposures(baseExposure, minimumStillExposureUs(), maximumStillExposureUs(), settings.bracketingEv);
                if (!times) {
                    setError(error, "The selected bracket EV offset exceeds the sensor's exposure limits (maximum 30s). Reduce the offset or change the base shutter.");
                    return false;
                }
                bracketTimes = *times;
                captureSettings.shutterAuto = captureSettings.isoAuto = false;
                captureSettings.longExposureEnabled = true;
                captureSettings.autofocusMode = AutofocusMode::Manual;
                if (settings.isoAuto) captureSettings.iso = uint32_t(std::lround(lockedGain * 100));
                if (settings.whiteBalanceAvailable) {
                    captureSettings.whiteBalanceAuto = false;
                    if (settings.whiteBalanceAuto) captureSettings.whiteBalanceKelvin = lockedWb.kelvin;
                }
                bracketBase = allocatePhotoBasePath(mediaRoot).string();
                longExposureTimingUsed_ = true;
            }
            std::deque<PendingPhotoCapture> burst;
            bool hasLongExposure = false;
            for (unsigned shot = 0; shot < count; ++shot) {
                PendingPhotoCapture pending;
                pending.basePath = bracket ? bracketBase + (shot == 0 ? "_BRm" : shot == 1 ? "_BR0" : "_BRp") : basePathOverride.empty()
                    ? allocatePhotoBasePath(mediaRoot).string() : basePathOverride;
                pending.settings = captureSettings;
                pending.bracket = bracket;
                if (bracket) {
                    pending.settings.shutterUs = bracketTimes[shot];
                    pending.lockedGain = lockedGain;
                    pending.lockedWb = lockedWb;
                    pending.lockedCcm = lockedCcm;
                }
                hasLongExposure |= needsLongStillExposure(pending.settings);
                pending.format = format;
                pending.onSaved = onSaved;
                pending.notifyCapture = count > 1;
                burst.push_back(std::move(pending));
            }
            bool notify = false;
            {
                std::lock_guard<std::mutex> lock(photoCaptureMutex_);
                if (stillExposureActive_ || !pendingPhotoCaptures_.empty() ||
                    photoFramesInFlight_.load(std::memory_order_acquire) || photoWriter_.busy()) {
                    setError(error, "Wait for the current capture to finish writing.");
                    return false;
                }
                if (pendingPhotoCaptures_.size() + count > maxPendingPhotoCaptures_) {
                    setError(error, "Too many pending photo captures.");
                    return false;
                }
                // Queue descriptors, not RAW frames; the normal bounded writer
                // consumes one distinct sensor frame for each shot.
                pendingPhotoCaptures_.insert(pendingPhotoCaptures_.end(),
                    std::make_move_iterator(burst.begin()), std::make_move_iterator(burst.end()));
                stillCaptureError_.clear();
                if (hasLongExposure && !stillExposureActive_) {
                    stillExposureActive_ = true;
                    stillExposureDeadline_ = std::chrono::steady_clock::now() + std::chrono::microseconds(pendingPhotoCaptures_.front().settings.shutterUs);
                    notify = true;
                }
            }
            emitMediaWriteState();
            if (notify) emitStillExposureEvent();
            return true;
        } catch (const std::exception &e) {
            setError(error, e.what());
            return false;
        }
#else
        (void)mediaRoot;
        (void)settings;
        (void)format;
        (void)error;
        (void)count;
        if (onSaved) onSaved();
        return true;
#endif
    }

    CaptureSettings currentSettings() const
    {
        std::shared_lock lock(sessionSettingsMutex_);
        return settings_;
    }

    bool exposureCompensationAvailable() const
    {
#if APERTARCORE_HAS_LIBCAMERA
        return camera_ && controlInfoHas(camera_->controls(), libcamera::controls::ExposureValue);
#else
        return false;
#endif
    }

    uint8_t exposureMeteringModes() const
    {
#if APERTARCORE_HAS_LIBCAMERA
        if (!camera_) return 0;
        uint8_t modes = activeTuningExposureMeteringModes();
        const int32_t values[] = {libcamera::controls::MeteringCentreWeighted,
                                  libcamera::controls::MeteringSpot, libcamera::controls::MeteringMatrix};
        for (int i = 0; i < 3; ++i)
            if (!intControlSupportsValue(camera_->controls(), libcamera::controls::AeMeteringMode, values[i]))
                modes &= ~(1 << i);
        return modes;
#else
        return 0;
#endif
    }

    StillExposureStatus stillExposureStatus() const
    {
#if APERTARCORE_HAS_LIBCAMERA
        std::lock_guard lock(photoCaptureMutex_);
        return {stillExposureActive_, stillExposureActive_ ? std::max(0.0,
            std::chrono::duration<double>(stillExposureDeadline_ - std::chrono::steady_clock::now()).count()) : 0.0};
#else
        return {};
#endif
    }

    std::string photoWriteError() const
    {
#if APERTARCORE_HAS_LIBCAMERA
        std::lock_guard lock(photoCaptureMutex_);
        if (!stillCaptureError_.empty()) return stillCaptureError_;
        return photoWriter_.error();
#else
        return {};
#endif
    }

    bool monochromeSensor() const
    {
#if APERTARCORE_HAS_LIBCAMERA
        return monochromeSensor_;
#else
        return false;
#endif
    }

    uint32_t minimumStillExposureUs() const
    {
#if APERTARCORE_HAS_LIBCAMERA
        if (camera_) {
            const auto info = camera_->controls().find(&libcamera::controls::ExposureTime);
            if (info != camera_->controls().end()) return uint32_t(std::max(1, info->second.min().get<int32_t>()));
        }
#endif
        return 1;
    }

    uint32_t maximumStillExposureUs() const
    {
#if APERTARCORE_HAS_LIBCAMERA
        if (camera_) {
            const auto info = camera_->controls().find(&libcamera::controls::ExposureTime);
            if (info != camera_->controls().end())
                return std::min<uint32_t>(30'000'000, std::max(1, info->second.max().get<int32_t>()));
        }
#endif
        return 30'000'000;
    }

    std::string recordingError() const
    {
#if APERTARCORE_HAS_LIBCAMERA
        if (const auto audioError = audioRecorder_.error(); !audioError.empty()) return audioError;
        if (lastRecordingFormat_ == RecordingFormat::H264) return h264Recorder_.error();
        if (lastRecordingFormat_ == RecordingFormat::Prores) return proresRecorder_.error();
        return recorder_.error();
#else
        return {};
#endif
    }

    WhiteBalanceMeasurement measuredWhiteBalance() const
    {
#if APERTARCORE_HAS_LIBCAMERA
        std::lock_guard<std::mutex> lock(whiteBalanceMutex_);
        return whiteBalanceMeasured_;
#else
        return {};
#endif
    }

    SensorFamily sensorFamily() const
    {
        return sensorFamily_;
    }

private:
    static void setError(std::string *error, const std::string &message)
    {
        if (error)
            *error = message;
    }

#if APERTARCORE_HAS_LIBCAMERA
    void resetStoppedSensorTiming()
    {
        // Long VBLANK can raise a driver's minimum exposure. Restore its timing
        // only after streaming stops, before a new IPA caches those limits.
        const std::string cameraId = camera_->id();
        std::error_code ec;
        for (const auto &entry : std::filesystem::directory_iterator("/sys/class/video4linux", ec)) {
            if (ec) break;
            const std::string node = entry.path().filename().string();
            if (node.rfind("v4l-subdev", 0) != 0) continue;
            const auto ofNode = std::filesystem::canonical(entry.path() / "device/of_node", ec);
            const bool matchesDeviceTree = !ec && ofNode.string() == "/sys/firmware/devicetree" + cameraId;
            ec.clear();
            std::ifstream nameFile(entry.path() / "name");
            std::string name;
            std::getline(nameFile, name);
            if (!matchesDeviceTree && name != cameraId) continue;

            const int fd = ::open(("/dev/" + node).c_str(), O_RDWR | O_CLOEXEC);
            if (fd < 0) break;
            bool reset = true;
            for (uint32_t id : {V4L2_CID_HBLANK, V4L2_CID_VBLANK, V4L2_CID_EXPOSURE}) {
                v4l2_query_ext_ctrl info{};
                info.id = id;
                if (::ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &info) < 0 ||
                    (info.flags & (V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_DISABLED)))
                    continue;
                v4l2_control control{};
                control.id = id;
                control.value = static_cast<int32_t>(info.minimum);
                if (::ioctl(fd, VIDIOC_S_CTRL, &control) < 0) reset = false;
            }
            ::close(fd);
            if (reset) {
                longExposureTimingUsed_ = false;
                return;
            }
            break;
        }
        std::cerr << "ApertarCore could not reset stopped sensor timing after long exposure.\n";
    }

    bool configureAndStart(const CaptureSettings &settings, std::string *error)
    {
        capturePriorityAdjusted_ = false;
        CaptureSettings sanitized = sanitizeSettingsForSensor(settings, sensorFamily_);
        sanitized.whiteBalanceAvailable = !monochromeSensor_ && activeTuningHasCalibratedWhiteBalance();
        constrainExposureMetering(sanitized, exposureMeteringModes());
        longExposureTimingUsed_ |= sanitized.photoModeEnabled && sanitized.longExposureEnabled;
        configuration_.reset();
        previewStream_ = nullptr;
        h264Stream_ = nullptr;
        rawStream_ = nullptr;
        rawStreamInfo_ = {};

        const std::vector<SensorModeSelection> sensorModes =
            sensorModeCandidatesForRequestedResolution(sensorFamily_,
                                                       sanitized.resolution,
                                                       sanitized.rawBitDepth);
        const bool proresMode = !sanitized.photoModeEnabled && sanitized.recordingFormat == RecordingFormat::Prores && !photoStreamOverride_;
        const bool processedYuvMode = !sanitized.photoModeEnabled && isProcessedVideoFormat(sanitized.recordingFormat) && !photoStreamOverride_;
        const bool hardwareH264Mode = processedYuvMode && !proresMode && recordingCapabilities().hardwareH264;
        const bool useDedicatedProcessedStream = processedYuvMode && (proresMode || !h264SingleYuvStream());
        const bool preferProcessedWithoutRaw = processedYuvMode && !h264KeepRawStream();
        const std::array<bool, 2> rawStreamPreference = preferProcessedWithoutRaw
            ? std::array<bool, 2>{ false, true }
            : std::array<bool, 2>{ true, false };
        const bool allowAutoSensorConfigFallback = monochromeSensor_ || sensorFamily_ == SensorFamily::Imx294 ||
                                                   sensorFamily_ == SensorFamily::Imx577 ||
                                                   (sensorFamily_ == SensorFamily::Imx585 &&
                                                    isImx585Raw12CropResolution(sanitized.resolution));
        std::ostringstream rejectionLog;

        for (const SensorModeSelection &sensorMode : sensorModes) {
            for (const bool includeRawStream : rawStreamPreference) {
                // A failed YUV configuration must not silently add large RAW
                // buffers on memory-constrained VC4 boards. Explicit opt-in remains available.
                if (includeRawStream && hardwareH264Mode && preferProcessedWithoutRaw)
                    continue;
                if (!includeRawStream && !processedYuvMode)
                    continue;
                if (!includeRawStream && processedYuvMode && !preferProcessedWithoutRaw)
                    continue;

                for (bool useExplicitSensorConfig : { true, false }) {
                // Colour mode tables cannot describe a mono driver's native depths.
                if (useExplicitSensorConfig && monochromeSensor_)
                    continue;
                if (!useExplicitSensorConfig && !allowAutoSensorConfigFallback)
                    continue;

                std::vector<libcamera::StreamRole> roles;
                if (processedYuvMode) {
                    roles = proresMode
                        ? std::vector<libcamera::StreamRole>{libcamera::StreamRole::VideoRecording, libcamera::StreamRole::Viewfinder}
                        : useDedicatedProcessedStream
                        ? std::vector<libcamera::StreamRole>{
                              libcamera::StreamRole::Viewfinder,
                              libcamera::StreamRole::VideoRecording,
                          }
                        : std::vector<libcamera::StreamRole>{
                              libcamera::StreamRole::VideoRecording,
                          };
                    if (includeRawStream)
                        roles.push_back(libcamera::StreamRole::Raw);
                } else {
                    roles = {
                        // Keep the established Apertar RAW capture model unchanged:
                        // one main YUV monitoring stream plus one raw stream.
                        libcamera::StreamRole::VideoRecording,
                        libcamera::StreamRole::Raw,
                    };
                }

                std::unique_ptr<libcamera::CameraConfiguration> candidateConfig =
                    camera_->generateConfiguration(roles);
                const size_t rawStreamIndex = useDedicatedProcessedStream ? 2u : 1u;
                const size_t requiredStreamCount = includeRawStream
                    ? rawStreamIndex + 1u
                    : (processedYuvMode ? (useDedicatedProcessedStream ? 2u : 1u)
                                        : rawStreamIndex + 1u);
                if (!candidateConfig || candidateConfig->size() < requiredStreamCount) {
                    rejectionLog << sensorMode.rawResolution.toString()
                                 << " " << sensorMode.bitDepth << "-bit "
                                 << (includeRawStream ? "raw-stream=on " : "raw-stream=off ")
                                 << "rejected by generateConfiguration; ";
                    continue;
                }

                libcamera::StreamConfiguration &previewConfig = candidateConfig->at(proresMode ? 1 : 0);
                libcamera::StreamConfiguration *processedConfig = useDedicatedProcessedStream
                    ? &candidateConfig->at(proresMode ? 0 : 1)
                    : nullptr;
                libcamera::StreamConfiguration *rawConfig = includeRawStream
                    ? &candidateConfig->at(rawStreamIndex)
                    : nullptr;

                const libcamera::Size rawSize(sensorMode.rawResolution.width, sensorMode.rawResolution.height);
                const uint32_t requestedActiveRawHeight = activeRawHeightForConfiguredStream(sensorFamily_,
                                                                                              sanitized,
                                                                                              rawSize.width,
                                                                                              rawSize.height);
                Resolution activePreviewReference{
                    activeRawWidthForConfiguredStream(sensorFamily_, sanitized, rawSize.width, rawSize.height),
                    requestedActiveRawHeight > 0 ? requestedActiveRawHeight : rawSize.height,
                };
                if (!sanitized.photoModeEnabled && sanitized.recordingFormat == RecordingFormat::H264 && recordingCapabilities().hardwareH264) {
                    const auto [width, height] = h264EncodeBounds(activePreviewReference, sanitized);
                    activePreviewReference = {width, height};
                }
                const libcamera::Size previewSize = processedYuvMode && !useDedicatedProcessedStream
                    ? h264EncodeSizeForSensorMode(activePreviewReference, sanitized)
                    : previewSizeForSensorMode(activePreviewReference);
                const unsigned int configuredBufferCount = proresMode ? proresCameraBufferCount() : processedYuvMode
                    ? h264CameraBufferCount()
                    : kConfiguredBufferCount;

                previewConfig.pixelFormat = libcamera::formats::YUV420;
                previewConfig.size = previewSize;
                previewConfig.size.alignDownTo(2, 2);
                previewConfig.bufferCount = configuredBufferCount;
                previewConfig.colorSpace = libcamera::ColorSpace::Rec709;

                if (processedConfig) {
                    // DRM names are most-significant-component first: BGR161616
                    // produces little-endian R,G,B samples, as the converter expects.
                    processedConfig->pixelFormat = proresMode ? libcamera::PixelFormat::fromString("BGR161616")
                        : libcamera::formats::YUV420;
                    processedConfig->size = h264EncodeSizeForSensorMode(activePreviewReference, sanitized);
                    processedConfig->size.alignDownTo(2, 2);
                    processedConfig->bufferCount = configuredBufferCount;
                    processedConfig->colorSpace = libcamera::ColorSpace::Rec709;
                }

                if (rawConfig) {
                    if (monochromeSensor_) {
                        const auto preferred = sensorMode.bitDepth == 8 ? libcamera::formats::R8
                            : sensorMode.bitDepth == 10 ? libcamera::formats::R10_CSI2P
                            : sensorMode.bitDepth == 12 ? libcamera::formats::R12_CSI2P
                            : libcamera::formats::R16;
                        const auto formats = rawConfig->formats().pixelformats();
                        if (std::find(formats.begin(), formats.end(), preferred) != formats.end())
                            rawConfig->pixelFormat = preferred;
                    } else
                        rawConfig->pixelFormat = sensorMode.rawPixelFormat;
                    rawConfig->size = rawSize;
                    rawConfig->bufferCount = configuredBufferCount;
                }

                if (useExplicitSensorConfig) {
                    candidateConfig->sensorConfig = libcamera::SensorConfiguration();
                    candidateConfig->sensorConfig->outputSize = rawSize;
                    candidateConfig->sensorConfig->bitDepth = sensorMode.bitDepth;
                } else {
                    candidateConfig->sensorConfig.reset();
                }

                if (sensorFamily_ == SensorFamily::Imx294 ||
                    sensorFamily_ == SensorFamily::Imx577 ||
                    (sensorFamily_ == SensorFamily::Imx585 && sanitized.rawBitDepth == 10)) {
                    std::cerr << "ApertarCore trying " << sensorFamilyName(sensorFamily_) << " mode "
                              << sensorMode.rawResolution.toString()
                              << " " << sensorMode.bitDepth << "-bit"
                              << " format=" << sensorMode.rawPixelFormat.toString()
                              << (includeRawStream ? " raw-stream=on" : " raw-stream=off")
                              << " sensorConfig=" << (useExplicitSensorConfig ? "explicit" : "auto")
                              << "\n";
                }

                libcamera::CameraConfiguration::Status validation = candidateConfig->validate();
                if (validation == libcamera::CameraConfiguration::Invalid) {
                    rejectionLog << sensorMode.rawResolution.toString()
                                 << " " << sensorMode.bitDepth << "-bit "
                                 << (includeRawStream ? "raw-stream=on " : "raw-stream=off ")
                                 << (useExplicitSensorConfig ? "explicit" : "auto")
                                 << " rejected by validate; ";
                    continue;
                }

                if (proresMode && (!processedConfig ||
                    processedConfig->pixelFormat != libcamera::PixelFormat::fromString("BGR161616") ||
                    processedConfig->size.width != sanitized.resolution.width ||
                    processedConfig->size.height != sanitized.resolution.height)) {
                    rejectionLog << "ProRes requires native RGB48 ISP output at the selected resolution; ";
                    continue;
                }

                if (camera_->configure(candidateConfig.get()) != 0) {
                    rejectionLog << sensorMode.rawResolution.toString()
                                 << " " << sensorMode.bitDepth << "-bit "
                                 << (includeRawStream ? "raw-stream=on " : "raw-stream=off ")
                                 << (useExplicitSensorConfig ? "explicit" : "auto")
                                 << " rejected by configure; ";
                    continue;
                }

                if (includeRawStream &&
                    (sensorFamily_ == SensorFamily::Imx477 || sensorFamily_ == SensorFamily::Imx577 ||
                     sensorFamily_ == SensorFamily::Imx294)) {
                    const libcamera::StreamConfiguration &candidateRawConfig =
                        candidateConfig->at(rawStreamIndex);
                    const bool invalidReadout = sensorFamily_ == SensorFamily::Imx294
                        ? candidateRawConfig.size != rawSize
                        : candidateRawConfig.size.width != sanitized.resolution.width ||
                          candidateRawConfig.size.height < sanitized.resolution.height;
                    if (invalidReadout) {
                        rejectionLog << sensorMode.rawResolution.toString()
                                     << " " << sensorMode.bitDepth << "-bit "
                                     << (includeRawStream ? "raw-stream=on " : "raw-stream=off ")
                                     << (useExplicitSensorConfig ? "explicit" : "auto")
                                     << " adjusted to too-small stream "
                                     << candidateRawConfig.size.toString()
                                     << " for requested crop "
                                     << sanitized.resolution.toString()
                                     << "; ";
                        continue;
                    }
                }

                configuration_ = std::move(candidateConfig);
                previewStream_ = configuration_->at(proresMode ? 1 : 0).stream();
                h264Stream_ = useDedicatedProcessedStream ? configuration_->at(proresMode ? 0 : 1).stream() : nullptr;
                rawStream_ = includeRawStream ? configuration_->at(rawStreamIndex).stream() : nullptr;

                const libcamera::StreamConfiguration *configuredRaw = includeRawStream
                    ? &configuration_->at(rawStreamIndex)
                    : nullptr;
                dngModel_ = sensorMode.dngModel;
                warnedUnsupportedRaw_ = false;

                const uint32_t configuredRawWidth = configuredRaw ? configuredRaw->size.width : rawSize.width;
                const uint32_t configuredRawHeight = configuredRaw ? configuredRaw->size.height : rawSize.height;
                const uint32_t activeRawWidth = activeRawWidthForConfiguredStream(sensorFamily_, sanitized,
                                                                                  configuredRawWidth, configuredRawHeight);
                const uint32_t activeRawHeight = activeRawHeightForConfiguredStream(sensorFamily_,
                                                                                    sanitized,
                                                                                    configuredRawWidth,
                                                                                    configuredRawHeight);
                configuredSensorWidth_ = configuredRawWidth;
                configuredSensorHeight_ = configuredRawHeight;

                rawStreamInfo_ = {};
                if (configuredRaw) {
                    rawStreamInfo_.valid = true;
                    rawStreamInfo_.width = activeRawWidth;
                    rawStreamInfo_.height = activeRawHeight;
                    rawStreamInfo_.stride = configuredRaw->stride;
                    rawStreamInfo_.activeTopRows = activeRawTopRowsForConfiguredStream(sensorFamily_,
                                                                                       sanitized,
                                                                                       configuredRaw->size.width,
                                                                                       configuredRaw->size.height);
                    rawStreamInfo_.frameSize = static_cast<size_t>(configuredRaw->frameSize);
                    rawStreamInfo_.formatName = configuredRaw->pixelFormat.toString();
                    rawStreamInfo_.supported = formatToRawView(configuredRaw->pixelFormat,
                                                               &rawStreamInfo_.bitDepth,
                                                               &rawStreamInfo_.packing,
                                                               &rawStreamInfo_.pattern);
                    if (rawStreamInfo_.supported)
                        monochromeSensor_ = rawStreamInfo_.pattern == BayerPattern::Monochrome;
                    sanitized.whiteBalanceAvailable = !monochromeSensor_ && activeTuningHasCalibratedWhiteBalance();
                    if (rawStreamInfo_.supported) {
                        rawStreamInfo_.transportBitDepth = rawStreamInfo_.bitDepth;
                        if (rawStreamInfo_.packing == PixelPacking::Unpacked) {
                            rawStreamInfo_.unpackedBytesPerSample =
                                unpackedBytesPerSampleForTransport(rawStreamInfo_.transportBitDepth);
                        }
                        rawStreamInfo_.activeLeftBytes = activeRawLeftBytes(
                            activeRawLeftPixelsForConfiguredStream(sensorFamily_, sanitized,
                                                                   configuredRawWidth, configuredRawHeight),
                            rawStreamInfo_.packing, rawStreamInfo_.transportBitDepth,
                            rawStreamInfo_.unpackedBytesPerSample);

                        const RawBitDepth requestedStorageBitDepth = storageRawBitDepthForSettings(sanitized);
                        if (rawStreamInfo_.packing == PixelPacking::PispCompressed1) {
                            rawStreamInfo_.bitDepth = requestedStorageBitDepth;
                        } else if (rawStreamInfo_.packing == PixelPacking::Unpacked &&
                                   rawStreamInfo_.transportBitDepth == RawBitDepth::Bits16 &&
                                   requestedStorageBitDepth != RawBitDepth::Bits16) {
                            rawStreamInfo_.bitDepth = requestedStorageBitDepth;
                            rawStreamInfo_.unpackedBytesPerSample = 2u;
                        }
                    }
                }

                CaptureSettings effectiveSettings = sanitized;
                if (monochromeSensor_ && configuredRaw && rawStreamInfo_.supported && !processedYuvMode)
                    effectiveSettings.rawBitDepth = rawBitDepthValue(rawStreamInfo_.bitDepth);
                effectiveSettings.resolution = Resolution{
                    activeRawWidth,
                    activeRawHeight,
                };
                const double maxFps = maximumSupportedFpsForResolution(sensorFamily_,
                                                                       effectiveSettings.resolution,
                                                                       effectiveSettings.rawBitDepth);
                if (effectiveSettings.fps > maxFps)
                    effectiveSettings.fps = maxFps;
                settings_ = effectiveSettings;

                std::cerr << "ApertarCore camera configured: sensor="
                          << sensorFamilyName(sensorFamily_)
                          << " raw=" << effectiveSettings.resolution.toString()
                          << " preview=" << previewStream_->configuration().size.toString();
                if (configuredRaw) {
                    std::cerr << " stream=" << configuredRaw->size.toString()
                              << " format=" << configuredRaw->pixelFormat.toString()
                              << " stride=" << configuredRaw->stride
                              << " storage_bits=" << rawBitDepthValue(rawStreamInfo_.bitDepth)
                              << " packing=" << pixelPackingName(rawStreamInfo_.packing);
                    if (rawStreamInfo_.transportBitDepth != rawStreamInfo_.bitDepth)
                        std::cerr << " transport_bits=" << rawBitDepthValue(rawStreamInfo_.transportBitDepth);
                    if (rawStreamInfo_.packing == PixelPacking::Unpacked &&
                        rawStreamInfo_.unpackedBytesPerSample > 0u) {
                        std::cerr << " container_bytes="
                                  << static_cast<unsigned int>(rawStreamInfo_.unpackedBytesPerSample);
                    }
                } else {
                    std::cerr << " raw_stream=off";
                }
                if (h264Stream_) {
                    std::cerr << (proresMode ? " prores=" : " h264=")
                              << h264Stream_->configuration().size.toString()
                              << "-" << h264Stream_->configuration().pixelFormat.toString();
                }
                else if (processedYuvMode)
                    std::cerr << " h264="
                              << previewStream_->configuration().size.toString()
                              << " single-yuv-stream";
                std::cerr << "\n";
                break;
            }

            if (configuration_)
                break;
            }

            if (configuration_)
                break;
        }

        if (!configuration_) {
            setError(error, rejectionLog.str().empty()
                                ? "libcamera rejected the requested camera configuration."
                                : ("libcamera rejected camera configurations: " + rejectionLog.str()));
            return false;
        }

        if (!previewStream_ || (!processedYuvMode && !rawStream_)) {
            setError(error, "Configured camera streams are missing.");
            return false;
        }

        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        const bool cachedProres = !settings_.photoModeEnabled && settings_.recordingFormat == RecordingFormat::Prores && allocateCachedProresBuffers();
        const bool cachedRaw = rawStream_ && recordingCapabilities().rawCompression &&
            rawStreamInfo_.packing == PixelPacking::PispCompressed1 &&
            boolEnvValue("APERTAR_RAW_CACHED_DMABUF", true) &&
            allocateCachedBuffers(rawStream_, cachedRawBuffers_, "RAW");
        if (allocator_->allocate(previewStream_) < 0 ||
            (h264Stream_ && h264Stream_ != previewStream_ && !cachedProres && allocator_->allocate(h264Stream_) < 0) ||
            (rawStream_ && !cachedRaw && allocator_->allocate(rawStream_) < 0)) {
            setError(error, "Failed to allocate libcamera frame buffers.");
            return false;
        }

        if (rawStream_ && !mapRawBuffers(error))
            return false;

        if (processedYuvMode) {
            std::string previewMapError;
            if (!mapPreviewBuffers(&previewMapError)) {
                std::cerr << "ApertarCore processed video stream mapping unavailable: "
                          << (previewMapError.empty() ? "unknown mapping failure" : previewMapError)
                          << "\n";
            }
        }

        if (!makeRequests(error))
            return false;

        libcamera::ControlList initialControls = controlsForSettings(settings_,
                                                                     sensorFamily_,
                                                                     camera_->controls(),
                                                                     Resolution{ configuredSensorWidth_, configuredSensorHeight_ },
                                                              camera_->properties(), monochromeSensor_);
        if (const auto maximum = camera_->properties().get(libcamera::properties::ScalerCropMaximum)) {
            const auto applied = initialControls.get(libcamera::controls::ScalerCrop);
            std::cerr << "ApertarCore preview crop: native_max=" << maximum->toString()
                      << " applied=" << (applied ? applied->toString() : "pipeline-default") << '\n';
        }
        if (camera_->start(&initialControls) != 0) {
            setError(error, "Failed to start libcamera capture.");
            return false;
        }
        started_ = true;
        running_ = true;

        camera_->requestCompleted.connect(this, &CameraSession::requestComplete);

        for (const auto &request : requests_) {
            if (camera_->queueRequest(request.get()) < 0) {
                setError(error, "Failed to queue an initial camera request.");
                running_ = false;
                camera_->stop();
                started_ = false;
                return false;
            }
        }

        return true;
    }

    const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &processedBuffers(libcamera::Stream *stream)
    {
        if (stream == h264Stream_ && !cachedProresBuffers_.empty())
            return cachedProresBuffers_;
        return allocator_->buffers(stream);
    }

    bool allocateCachedProresBuffers()
    {
        cachedProresBuffers_.clear();
        const char *enabled = std::getenv("APERTAR_PRORES_CACHED_DMABUF");
        if ((enabled && std::atoi(enabled) == 0) || !h264Stream_)
            return false;
        return allocateCachedBuffers(h264Stream_, cachedProresBuffers_, "ProRes");
    }

    bool allocateCachedBuffers(libcamera::Stream *stream,
                               std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers,
                               const char *label)
    {
        buffers.clear();
        const auto &config = stream->configuration();
        if (!config.frameSize) return false;
        for (const char *heap : {"/dev/dma_heap/vidbuf_cached", "/dev/dma_heap/linux,cma"}) {
            const int heapFd = ::open(heap, O_RDWR | O_CLOEXEC);
            if (heapFd < 0) continue;
            for (unsigned i = 0; i < config.bufferCount; ++i) {
                dma_heap_allocation_data allocation{};
                allocation.len = config.frameSize;
                allocation.fd_flags = O_CLOEXEC | O_RDWR;
                if (::ioctl(heapFd, DMA_HEAP_IOCTL_ALLOC, &allocation) < 0) break;
                std::vector<libcamera::FrameBuffer::Plane> planes(1);
                planes[0].fd = libcamera::SharedFD(libcamera::UniqueFD(static_cast<int>(allocation.fd)));
                planes[0].offset = 0;
                planes[0].length = config.frameSize;
                buffers.push_back(std::make_unique<libcamera::FrameBuffer>(planes));
            }
            ::close(heapFd);
            if (buffers.size() == config.bufferCount) {
                std::cerr << "ApertarCore " << label << " buffers: CPU-cached heap=" << heap
                          << " count=" << config.bufferCount << " frame_bytes=" << config.frameSize << "\n";
                return true;
            }
            buffers.clear();
        }
        std::cerr << "ApertarCore " << label << " cached DMA allocation unavailable; using libcamera buffers\n";
        return false;
    }

    static bool syncCpuReadBuffer(int fd, bool begin)
    {
        dma_buf_sync sync{};
        sync.flags = DMA_BUF_SYNC_READ | (begin ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END);
        int ret;
        do { ret = ::ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync); } while (ret < 0 && errno == EINTR);
        if (ret < 0)
            std::cerr << "ApertarCore DMA read sync " << (begin ? "START" : "END")
                      << " failed: " << std::strerror(errno) << "\n";
        return ret == 0;
    }

    const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &rawBuffers()
    {
        return cachedRawBuffers_.empty() ? allocator_->buffers(rawStream_) : cachedRawBuffers_;
    }

    bool copyRawBytes(libcamera::FrameBuffer *buffer, uint8_t *destination,
                      const uint8_t *source, size_t size)
    {
        const int fd = cachedRawBuffers_.empty() ? -1 : buffer->planes().front().fd.get();
        if (fd >= 0 && !syncCpuReadBuffer(fd, true)) return false;
        std::memcpy(destination, source, size);
        return fd < 0 || syncCpuReadBuffer(fd, false);
    }

    bool makeRequests(std::string *error)
    {
        const auto &previewBuffers = allocator_->buffers(previewStream_);
        const auto *rawBuffers = rawStream_ ? &this->rawBuffers() : nullptr;
        const auto *h264Buffers = (h264Stream_ && h264Stream_ != previewStream_)
            ? &processedBuffers(h264Stream_)
            : nullptr;
        const bool processedYuvMode = !settings_.photoModeEnabled && isProcessedVideoFormat(settings_.recordingFormat);
        const size_t activeRequestLimit = processedYuvMode && settings_.recordingFormat == RecordingFormat::Prores ? proresCameraBufferCount()
            : processedYuvMode ? h264CameraBufferCount() : kActiveRequestCount;
        size_t requestCount = std::min<size_t>(previewBuffers.size(), activeRequestLimit);
        if (rawBuffers)
            requestCount = std::min(requestCount, rawBuffers->size());
        if (h264Buffers)
            requestCount = std::min(requestCount, h264Buffers->size());

        if (requestCount == 0) {
            setError(error, "No frame buffers were allocated.");
            return false;
        }

        requests_.clear();
        publishedPreviewSlots_.clear();
        requests_.reserve(requestCount);

        for (size_t i = 0; i < requestCount; ++i) {
            std::unique_ptr<libcamera::Request> request = camera_->createRequest(i);
            if (!request) {
                setError(error, "Failed to create a libcamera request.");
                return false;
            }

            if (request->addBuffer(previewStream_, previewBuffers[i].get()) < 0 ||
                (h264Buffers && request->addBuffer(h264Stream_, (*h264Buffers)[i].get()) < 0) ||
                (rawBuffers && request->addBuffer(rawStream_, (*rawBuffers)[i].get()) < 0)) {
                setError(error, "Failed to attach buffers to a libcamera request.");
                return false;
            }

            requests_.push_back(std::move(request));
        }

        return true;
    }

    void requeueRequest(libcamera::Request *request)
    {
        if (!request)
            return;

        request->reuse(libcamera::Request::ReuseBuffers);
        {
            std::lock_guard<std::mutex> lock(controlsMutex_);
            if (!pendingControls_.empty()) {
                request->controls().merge(pendingControls_);
                pendingControls_.clear();
            }
        }

        bool exposureArmed = false;
        {
            std::lock_guard lock(photoCaptureMutex_);
            auto applyBracketControls = [&](const PendingPhotoCapture &pending, bool expose) {
                auto lockedSettings = pending.settings;
                // Do not send ColourTemperature together with explicit gains:
                // calibrated Kelvin selection would replace the latched AWB gains.
                lockedSettings.whiteBalanceAvailable = false;
                request->controls().merge(controlsForSettings(lockedSettings, sensorFamily_, camera_->controls(),
                    {configuredSensorWidth_, configuredSensorHeight_}, camera_->properties(), monochromeSensor_, expose),
                    libcamera::ControlList::MergePolicy::OverwriteExisting);
                request->controls().set(libcamera::controls::AnalogueGain, pending.lockedGain);
                if (!monochromeSensor_ && pending.lockedWb.valid() && pending.settings.whiteBalanceAvailable) {
                    request->controls().set(libcamera::controls::AwbEnable, false);
                    const std::array<float, 2> gains{pending.lockedWb.red, pending.lockedWb.blue};
                    request->controls().set(libcamera::controls::ColourGains, libcamera::Span<const float, 2>(gains));
                }
            };
            if (restoreStillPreviewControls_) {
                if (!pendingPhotoCaptures_.empty() && pendingPhotoCaptures_.front().bracket)
                    applyBracketControls(pendingPhotoCaptures_.front(), false);
                else
                    request->controls().merge(controlsForSettings(settings_, sensorFamily_, camera_->controls(),
                        {configuredSensorWidth_, configuredSensorHeight_}, camera_->properties(), monochromeSensor_),
                        libcamera::ControlList::MergePolicy::OverwriteExisting);
                restoreStillPreviewControls_ = false;
            } else if (!pendingPhotoCaptures_.empty()) {
                auto &pending = pendingPhotoCaptures_.front();
                if (pending.bracket && !pending.locksApplied) {
                    // ISP WB and sensor gain have different control delays. Settle
                    // both before scheduling the one-request shutter pulse.
                    applyBracketControls(pending, false);
                    pending.locksApplied = true;
                    pending.queuedAt = std::chrono::steady_clock::now();
                } else if ((pending.bracket || needsLongStillExposure(pending.settings)) && !pending.exposureArmed &&
                           (!pending.bracket || pending.locksReady)) {
                    // Apply the slow shutter to one request only. The following request
                    // restores preview timing before another long frame can be scheduled.
                    if (pending.bracket) {
                        applyBracketControls(pending, true);
                        timespec now{};
                        if (::clock_gettime(CLOCK_BOOTTIME, &now) == 0)
                            pending.earliestSensorTimestampNs = int64_t(now.tv_sec) * 1'000'000'000 + now.tv_nsec;
                    } else
                        request->controls().merge(controlsForSettings(pending.settings, sensorFamily_, camera_->controls(),
                            {configuredSensorWidth_, configuredSensorHeight_}, camera_->properties(), monochromeSensor_, true),
                            libcamera::ControlList::MergePolicy::OverwriteExisting);
                    pending.exposureArmed = true;
                    pending.queuedAt = std::chrono::steady_clock::now();
                    stillExposureDeadline_ = pending.queuedAt + std::chrono::microseconds(pending.settings.shutterUs);
                    restoreStillPreviewControls_ = true;
                    exposureArmed = true;
                }
            }
        }
        if (exposureArmed) emitStillExposureEvent();

        if (running_.load() && camera_) {
            if (camera_->queueRequest(request) < 0) {
                ++requeueFailures_;
                if (requeueFailures_ == 1 || (requeueFailures_ % 30) == 0) {
                    std::cerr << "ApertarCore drop[requeue-failed]: total="
                              << requeueFailures_ << "\n";
                }
            }
        }
    }

    void requestComplete(libcamera::Request *request)
    {
        if (!request || request->status() == libcamera::Request::RequestCancelled)
            return;

        if (!running_.load())
            return;

        // Capture callbacks read strings and settings while button/API controls
        // can replace them. Keep the settings -> controls lock order consistent.
        std::shared_lock settingsLock(sessionSettingsMutex_);
        if (!capturePriorityAdjusted_) {
            capturePriorityAdjusted_ = true;
            // LimitNICE=-5 permits this bounded boost without CAP_SYS_NICE.
            // Only this callback thread changes priority, not encoder workers.
            const int tid = static_cast<int>(::syscall(SYS_gettid));
            if (::setpriority(PRIO_PROCESS, static_cast<id_t>(tid), -5) == 0)
                std::cerr << "ApertarCore capture scheduling: thread=" << tid << " nice=-5\n";
            else
                std::cerr << "ApertarCore capture scheduling: normal priority retained ("
                          << std::strerror(errno) << ")\n";
        }

        WhiteBalanceMeasurement whiteBalance;
        if (auto temperature = request->metadata().get(libcamera::controls::ColourTemperature);
            temperature && *temperature > 0)
            whiteBalance.kelvin = static_cast<uint32_t>(*temperature);
        if (auto gains = request->metadata().get(libcamera::controls::ColourGains);
            gains && gains->size() >= 2) {
            whiteBalance.red = (*gains)[0];
            whiteBalance.blue = (*gains)[1];
        }
        if (!whiteBalance.valid()) {
            whiteBalance.red = 0.0f;
            whiteBalance.blue = 0.0f;
        }
        if (monochromeSensor_) {
            whiteBalance.kelvin = 0;
            whiteBalance.red = whiteBalance.blue = 1.0f;
        }
        {
            std::lock_guard<std::mutex> lock(whiteBalanceMutex_);
            whiteBalanceMeasured_ = whiteBalance;
            measuredExposureUs_ = uint32_t(std::max(0, request->metadata().get(libcamera::controls::ExposureTime).value_or(0)));
            measuredAnalogueGain_ = request->metadata().get(libcamera::controls::AnalogueGain).value_or(0);
            measuredCcm_.reset();
            if (usesStillBracketing(settings_)) {
                if (auto matrix = request->metadata().get(libcamera::controls::ColourCorrectionMatrix);
                    matrix && matrix->size() >= 9 && std::all_of(matrix->begin(), matrix->end(), [](float x) { return std::isfinite(x); })) {
                    measuredCcm_.emplace();
                    std::copy_n(matrix->begin(), 9, measuredCcm_->begin());
                }
            }
        }

        capturePhotoFrame(request, whiteBalance);
        bool recoveredPreview = false;
        bool exposePreview = true;
        {
            std::lock_guard lock(photoCaptureMutex_);
            if (stillExposureActive_ && pendingPhotoCaptures_.empty() && !restoreStillPreviewControls_) {
                const auto exposure = request->metadata().get(libcamera::controls::ExposureTime);
                if (exposure && *exposure <= int32_t(stillPreviewExposureLimitUs + 1000)) {
                    stillExposureActive_ = false;
                    recoveredPreview = true;
                }
            }
            exposePreview = !stillExposureActive_;
        }
        if (exposePreview) publishPreviewFrame(request);
        if (recoveredPreview) emitStillExposureEvent();
        {
            std::lock_guard lock(recordingFrameMutex_);
            if (recordingFramesEnabled_) {
                bool ready = true;
                if (waitingForRecordingCadence_) {
                    const auto timestamp = request->metadata().get(libcamera::controls::SensorTimestamp);
                    const auto duration = request->metadata().get(libcamera::controls::FrameDuration);
                    const int64_t expected = int64_t(std::llround(1000000 / settings_.fps));
                    ready = (!timestamp || *timestamp / 1000 >= recordingStartBootUs_) &&
                        (!duration || std::abs(normalizeFrameDurationToUs(*duration, settings_.fps) - expected) <= std::max<int64_t>(500, expected / 50));
                    if (ready) waitingForRecordingCadence_ = false;
                }
                if (ready) {
                    if (recordProcessedPreviewFrame(request)) return;
                    recordRawFrame(request, whiteBalance);
                }
            }
        }
        requeueRequest(request);
    }

    struct MappedPlane {
        void *mapping = nullptr;
        size_t mappingLength = 0;
        uint8_t *data = nullptr;
        size_t length = 0;
    };

    struct MappedPreviewBuffer {
        void *mapping = nullptr;
        size_t mappingLength = 0;
        uint8_t *data = nullptr;
    };

    struct PendingPhotoCapture {
        std::function<void()> onSaved;
        bool notifyCapture = false;
        bool exposureArmed = false;
        bool bracket = false;
        bool locksApplied = false;
        bool locksReady = false;
        float lockedGain = 0;
        WhiteBalanceMeasurement lockedWb;
        std::optional<std::array<float, 9>> lockedCcm;
        std::string basePath;
        CaptureSettings settings{};
        PhotoCaptureFormat format = PhotoCaptureFormat::Dng;
        std::chrono::steady_clock::time_point queuedAt = std::chrono::steady_clock::now();
        int64_t earliestSensorTimestampNs = [] {
            timespec now{};
            return ::clock_gettime(CLOCK_BOOTTIME, &now) == 0
                ? int64_t(now.tv_sec) * 1'000'000'000 + now.tv_nsec : int64_t(0);
        }();
    };

    struct CachedRawStreamInfo {
        bool valid = false;
        bool supported = false;
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t stride = 0;
        uint32_t activeTopRows = 0;
        size_t activeLeftBytes = 0;
        size_t frameSize = 0;
        RawBitDepth bitDepth = RawBitDepth::Bits16;
        RawBitDepth transportBitDepth = RawBitDepth::Bits16;
        uint8_t unpackedBytesPerSample = 0;
        PixelPacking packing = PixelPacking::Unpacked;
        BayerPattern pattern = BayerPattern::RGGB;
        std::string formatName;
    };

    static bool mapPlane(const libcamera::FrameBuffer::Plane &plane, MappedPlane *mapped, std::string *error)
    {
        const uint32_t planeOffset = plane.offset == libcamera::FrameBuffer::Plane::kInvalidOffset
            ? 0
            : plane.offset;
        const long pageSize = std::max<long>(1, ::sysconf(_SC_PAGESIZE));
        const uint32_t pageMask = static_cast<uint32_t>(pageSize - 1);
        const off_t alignedOffset = static_cast<off_t>(planeOffset & ~pageMask);
        const size_t offsetDelta = static_cast<size_t>(planeOffset - static_cast<uint32_t>(alignedOffset));
        const size_t mapLength = offsetDelta + plane.length;

        void *mapping = ::mmap(nullptr, mapLength, PROT_READ, MAP_SHARED, plane.fd.get(), alignedOffset);
        if (mapping == MAP_FAILED) {
            setError(error, std::string("Failed to mmap raw buffer: ") + std::strerror(errno));
            return false;
        }

        mapped->mapping = mapping;
        mapped->mappingLength = mapLength;
        mapped->data = static_cast<uint8_t *>(mapping) + offsetDelta;
        mapped->length = plane.length;
        return true;
    }

    bool mapRawBuffers(std::string *error)
    {
        unmapRawBuffers();
        const auto &rawBuffers = this->rawBuffers();
        for (const auto &buffer : rawBuffers) {
            if (!buffer || buffer->planes().empty()) {
                setError(error, "Raw stream returned an empty frame buffer.");
                return false;
            }

            MappedPlane mapped;
            if (!mapPlane(buffer->planes().front(), &mapped, error)) {
                unmapRawBuffers();
                return false;
            }

            mappedRawBuffers_.emplace(buffer.get(), mapped);
        }

        return true;
    }

    bool mapPreviewBuffers(std::string *error)
    {
        unmapPreviewBuffers();
        libcamera::Stream *stream = h264Stream_ ? h264Stream_ : previewStream_;
        if (!allocator_ || !stream)
            return false;

        const auto &previewBuffers = processedBuffers(stream);
        for (const auto &buffer : previewBuffers) {
            if (!buffer || buffer->planes().empty()) {
                setError(error, "Preview stream returned an empty frame buffer.");
                unmapPreviewBuffers();
                return false;
            }

            const auto &planes = buffer->planes();
            const int fd = planes.front().fd.get();
            if (fd < 0) {
                setError(error, "Preview buffer has no valid DMABUF fd.");
                unmapPreviewBuffers();
                return false;
            }

            size_t mappingLength = 0;
            for (const auto &plane : planes) {
                if (plane.fd.get() != fd) {
                    setError(error, "Processed video recording does not support separate plane fds yet.");
                    unmapPreviewBuffers();
                    return false;
                }
                const size_t offset = plane.offset == libcamera::FrameBuffer::Plane::kInvalidOffset
                    ? 0
                    : static_cast<size_t>(plane.offset);
                mappingLength = std::max(mappingLength, offset + static_cast<size_t>(plane.length));
            }

            if (mappingLength == 0) {
                setError(error, "Preview buffer mapping length is zero.");
                unmapPreviewBuffers();
                return false;
            }

            void *mapping = ::mmap(nullptr, mappingLength, PROT_READ, MAP_SHARED, fd, 0);
            if (mapping == MAP_FAILED) {
                setError(error, std::string("Failed to mmap preview buffer: ") + std::strerror(errno));
                unmapPreviewBuffers();
                return false;
            }

            MappedPreviewBuffer mapped;
            mapped.mapping = mapping;
            mapped.mappingLength = mappingLength;
            mapped.data = static_cast<uint8_t *>(mapping);
            mappedPreviewBuffers_.emplace(buffer.get(), mapped);
        }

        return !mappedPreviewBuffers_.empty();
    }

    void unmapPreviewBuffers()
    {
        for (auto &[buffer, mapped] : mappedPreviewBuffers_) {
            (void)buffer;
            if (mapped.mapping && mapped.mapping != MAP_FAILED)
                ::munmap(mapped.mapping, mapped.mappingLength);
        }
        mappedPreviewBuffers_.clear();
    }

    void unmapRawBuffers()
    {
        for (auto &[buffer, mapped] : mappedRawBuffers_) {
            (void)buffer;
            if (mapped.mapping && mapped.mapping != MAP_FAILED)
                ::munmap(mapped.mapping, mapped.mappingLength);
        }
        mappedRawBuffers_.clear();
    }

    uint64_t wallClockForSensorTimestamp(int64_t sensorTimestampUs)
    {
        return recordingClock_.wallTime(sensorTimestampUs > 0
            ? sensorTimestampUs : clockMicroseconds(CLOCK_BOOTTIME));
    }

    void recordRawFrame(libcamera::Request *request, const WhiteBalanceMeasurement &whiteBalance)
    {
        if (!recorder_.active() || !rawStream_)
            return;
        if (!whiteBalance.valid()) {
            if (!warnedMissingWhiteBalance_) {
                std::cerr << "ApertarCore RAW frame missing measured white balance; not writing guessed metadata.\n";
                warnedMissingWhiteBalance_ = true;
            }
            return;
        }

        const CachedRawStreamInfo &rawInfo = rawStreamInfo_;
        if (!rawInfo.valid)
            return;
        if (!rawInfo.supported) {
            if (!warnedUnsupportedRaw_) {
                warnedUnsupportedRaw_ = true;
                std::cerr << "ApertarCore recording skipped: unsupported raw format "
                          << rawInfo.formatName
                          << ". Expected a supported Bayer or monochrome RAW format.\n";
            }
            return;
        }

        libcamera::FrameBuffer *buffer = request->findBuffer(rawStream_);
        if (!buffer)
            return;

        auto mappedIt = mappedRawBuffers_.find(buffer);
        if (mappedIt == mappedRawBuffers_.end())
            return;

        const MappedPlane &mapped = mappedIt->second;
        const size_t bytesUsed = !buffer->metadata().planes().empty()
            ? buffer->metadata().planes().front().bytesused
            : 0;
        const size_t activeFrameSize = activeRawFrameBytes(rawInfo.stride, rawInfo.height);
        const size_t candidateRawSize = std::min(mapped.length, bytesUsed > 0 ? bytesUsed : rawInfo.frameSize);
        const size_t activeOffsetBytes = static_cast<size_t>(rawInfo.stride) *
                                         static_cast<size_t>(rawInfo.activeTopRows) + rawInfo.activeLeftBytes;
        if (activeOffsetBytes >= candidateRawSize)
            return;
        const size_t rawSize = activeFrameSize > 0
            ? std::min(candidateRawSize - activeOffsetBytes, activeFrameSize)
            : candidateRawSize - activeOffsetBytes;
        if (rawSize == 0 || rawSize > mapped.length)
            return;

        int64_t timestampUs = 0;
        if (auto ts = request->metadata().get(libcamera::controls::SensorTimestamp))
            timestampUs = static_cast<int64_t>(*ts / 1000);
        else
            timestampUs = static_cast<int64_t>(buffer->metadata().timestamp / 1000);

        int64_t frameDurationUs = 0;
        if (auto frameDuration = request->metadata().get(libcamera::controls::FrameDuration);
            frameDuration && *frameDuration > 0) {
            frameDurationUs = normalizeFrameDurationToUs(static_cast<int64_t>(*frameDuration), settings_.fps);
        }

        trackRecordingCadence(timestampUs, frameDurationUs);

        const uint64_t frameIndex = recorder_.nextFrameIndex(timestampUs);
        const uint64_t wallClockTimestampUs = wallClockForSensorTimestamp(timestampUs);
        CdngEncoder::Metadata metadata = metadataFromRequest(
            settings_,
            request->metadata(),
            frameIndex,
            timestampUs,
            wallClockTimestampUs,
            sensorFamily_,
            dngModel_, whiteBalance, rawInfo.pattern == BayerPattern::Monochrome);

        // The completion callback is the only producer. If workers are already
        // behind, reject before copying a full RAW frame that cannot be queued.
        if (!recorder_.hasCapacityForFrame(frameIndex))
            return;

        std::vector<uint8_t> rawCopy = recorder_.acquireRawBuffer(rawSize);
        if (!copyRawBytes(buffer, rawCopy.data(), mapped.data + activeOffsetBytes, rawSize)) {
            dropWarning_.markDropped();
            return;
        }

        if (frameIndex == 0)
            gyroRecorder_.setVideoStartTimestampUs(static_cast<uint64_t>(std::max<int64_t>(0, timestampUs)), frameIndex);

        RawFrameView view;
        view.data = rawCopy.data();
        view.size = rawCopy.size();
        view.width = rawInfo.width;
        view.height = rawInfo.height;
        view.stride = rawInfo.stride;
        view.bitDepth = rawInfo.bitDepth;
        view.unpackedBytesPerSample = rawInfo.unpackedBytesPerSample;
        view.sampleBits = settings_.rawBitDepth;
        view.packing = rawInfo.packing;
        view.bayerPattern = rawInfo.pattern;
        view.timestampUs = timestampUs;

        if (recorder_.enqueue(std::move(rawCopy), view, std::move(metadata)))
            audioRecorder_.observeVideoFrame(timestampUs, frameDurationUs);
    }

    void capturePhotoFrame(libcamera::Request *request, const WhiteBalanceMeasurement &whiteBalance)
    {
        struct InFlightPhoto {
            std::atomic<unsigned> *counter = nullptr;
            CameraSession *session;
            ~InFlightPhoto() {
                if (counter) {
                    counter->fetch_sub(1, std::memory_order_release);
                    session->emitMediaWriteState();
                }
            }
        } inFlight{nullptr, this};
        PendingPhotoCapture pending;
        std::string captureError;
        {
            std::lock_guard<std::mutex> lock(photoCaptureMutex_);
            if (pendingPhotoCaptures_.empty())
                return;
            const auto &wanted = pendingPhotoCaptures_.front().settings;
            auto &front = pendingPhotoCaptures_.front();
            if (!front.bracket && needsLongStillExposure(wanted) && !front.exposureArmed)
                return;
            if (wanted.photoModeEnabled && wanted.longExposureEnabled && !wanted.shutterAuto) {
                const auto exposure = request->metadata().get(libcamera::controls::ExposureTime);
                const auto timestamp = request->metadata().get(libcamera::controls::SensorTimestamp);
                const bool preTriggerFrame = timestamp && *timestamp < pendingPhotoCaptures_.front().earliestSensorTimestampNs;
                // Do not save a preview exposure that started before the shutter press.
                const int64_t tolerance = front.bracket
                    ? std::max<int64_t>(std::max<uint32_t>(1, minimumStillExposureUs() * 3 / 4), wanted.shutterUs / 100)
                    : std::max<int64_t>(1000, wanted.shutterUs / 100);
                bool matching = !preTriggerFrame && exposure && std::abs(int64_t(*exposure) - wanted.shutterUs) <= tolerance;
                if (front.bracket) {
                    const auto gain = request->metadata().get(libcamera::controls::AnalogueGain);
                    bool locked = gain && std::abs(*gain - front.lockedGain) <= std::max(.001f, front.lockedGain * .005f);
                    if (!monochromeSensor_ && front.lockedWb.valid() && wanted.whiteBalanceAvailable)
                        locked &= whiteBalance.valid() && std::abs(whiteBalance.red - front.lockedWb.red) <= .005f &&
                                  std::abs(whiteBalance.blue - front.lockedWb.blue) <= .005f;
                    front.locksReady = front.locksApplied && locked;
                    matching &= front.exposureArmed && front.locksReady;
                }
                if (!matching) {
                    if (std::chrono::steady_clock::now() - pendingPhotoCaptures_.front().queuedAt >
                        std::chrono::microseconds(std::max<int64_t>(10'000'000, int64_t(wanted.shutterUs) * 3 + 5'000'000))) {
                        std::cerr << "ApertarCore still exposure timed out: requested_us=" << wanted.shutterUs
                                  << " actual_us=" << exposure.value_or(0)
                                  << " gain=" << request->metadata().get(libcamera::controls::AnalogueGain).value_or(0)
                                  << " locked_gain=" << front.lockedGain
                                  << " wb=" << whiteBalance.red << ',' << whiteBalance.blue
                                  << " locked_wb=" << front.lockedWb.red << ',' << front.lockedWb.blue << '\n';
                        captureError = "Still exposure or white balance did not settle; capture cancelled.";
                        stillCaptureError_ = captureError;
                        if (front.bracket) pendingPhotoCaptures_.clear();
                        else pendingPhotoCaptures_.pop_front();
                        restoreStillPreviewControls_ = true;
                        stillExposureActive_ = false;
                    }
                    if (captureError.empty()) return;
                }
            }
            if (captureError.empty()) {
                // Cover the descriptor -> RAW copy -> writer hand-off. A mode
                // change must not see an idle queue during that interval.
                photoFramesInFlight_.fetch_add(1, std::memory_order_release);
                inFlight.counter = &photoFramesInFlight_;
                pending = std::move(pendingPhotoCaptures_.front());
                pendingPhotoCaptures_.pop_front();
                if (pending.bracket && pendingPhotoCaptures_.empty())
                    restoreStillPreviewControls_ = true;
            }
        }
        if (!captureError.empty()) {
            if (exposureEvent_) exposureEvent_("{\"event\":\"photo_write_error\",\"photo_write_error\":\"" + apertar::escapeJson(captureError) + "\"}");
            emitStillExposureEvent();
            return;
        }

        const bool needsDng = photoFormatWritesDng(pending.format);
        const bool needsPreview = photoFormatWritesPreview(pending.format);
        const bool needsRaw = needsDng || needsPreview;

        std::vector<uint8_t> rawCopy;
        RawFrameView view;
        CdngEncoder::Metadata metadata;
        StillRenderSettings rendering;
        if (needsRaw) {
            if (!rawStream_)
                return;

            libcamera::FrameBuffer *buffer = request->findBuffer(rawStream_);
            if (!buffer)
                return;

            const auto mappedIt = mappedRawBuffers_.find(buffer);
            if (mappedIt == mappedRawBuffers_.end())
                return;

            const libcamera::StreamConfiguration &config = rawStream_->configuration();
            RawBitDepth bitDepth = RawBitDepth::Bits16;
            PixelPacking packing = PixelPacking::Unpacked;
            BayerPattern pattern = BayerPattern::RGGB;
            if (!formatToRawView(config.pixelFormat, &bitDepth, &packing, &pattern)) {
                std::cerr << "ApertarCore photo skipped: unsupported raw format "
                          << config.pixelFormat.toString() << "\n";
                return;
            }
            const CaptureSettings sanitizedPhotoSettings = sanitizeSettingsForSensor(pending.settings, sensorFamily_);
            const RawBitDepth transportBitDepth = bitDepth;
            uint8_t unpackedBytesPerSample = packing == PixelPacking::Unpacked
                ? unpackedBytesPerSampleForTransport(transportBitDepth)
                : 0u;
            const RawBitDepth requestedStorageBitDepth = storageRawBitDepthForSettings(sanitizedPhotoSettings);
            if (packing == PixelPacking::PispCompressed1) {
                bitDepth = requestedStorageBitDepth;
            } else if (packing == PixelPacking::Unpacked &&
                       transportBitDepth == RawBitDepth::Bits16 &&
                       requestedStorageBitDepth != RawBitDepth::Bits16) {
                bitDepth = requestedStorageBitDepth;
                unpackedBytesPerSample = 2u;
            }

            const MappedPlane &mapped = mappedIt->second;
            const size_t bytesUsed = !buffer->metadata().planes().empty()
                ? buffer->metadata().planes().front().bytesused
                : 0;
            const uint32_t activeHeight = activeRawHeightForConfiguredStream(sensorFamily_,
                                                                             sanitizedPhotoSettings,
                                                                             config.size.width,
                                                                             config.size.height);
            const size_t activeFrameSize = activeRawFrameBytes(config.stride, activeHeight);
            const size_t candidateRawSize = std::min(mapped.length,
                                                     bytesUsed > 0 ? bytesUsed : static_cast<size_t>(config.frameSize));
            const uint32_t activeTopRows = activeRawTopRowsForConfiguredStream(sensorFamily_,
                                                                               sanitizedPhotoSettings,
                                                                               config.size.width,
                                                                               config.size.height);
            const size_t activeOffsetBytes = static_cast<size_t>(config.stride) *
                                             static_cast<size_t>(activeTopRows) + activeRawLeftBytes(
                activeRawLeftPixelsForConfiguredStream(sensorFamily_, sanitizedPhotoSettings,
                                                       config.size.width, config.size.height),
                packing, transportBitDepth, unpackedBytesPerSample);
            if (activeOffsetBytes >= candidateRawSize)
                return;
            const size_t rawSize = activeFrameSize > 0
                ? std::min(candidateRawSize - activeOffsetBytes, activeFrameSize)
                : candidateRawSize - activeOffsetBytes;
            if (rawSize == 0 || rawSize > mapped.length)
                return;

            rawCopy.resize(rawSize);
            if (!copyRawBytes(buffer, rawCopy.data(), mapped.data + activeOffsetBytes, rawSize)) {
                std::cerr << "ApertarCore photo skipped: RAW DMA synchronization failed\n";
                return;
            }

            int64_t timestampUs = 0;
            if (auto ts = request->metadata().get(libcamera::controls::SensorTimestamp))
                timestampUs = static_cast<int64_t>(*ts / 1000);
            else
                timestampUs = static_cast<int64_t>(buffer->metadata().timestamp / 1000);

            const uint64_t frameIndex = photoFrameCounter_.fetch_add(1, std::memory_order_relaxed);
            view.data = rawCopy.data();
            view.size = rawCopy.size();
            view.width = activeRawWidthForConfiguredStream(sensorFamily_, sanitizedPhotoSettings,
                                                           config.size.width, config.size.height);
            view.height = activeHeight;
            view.stride = config.stride;
            view.bitDepth = bitDepth;
            view.unpackedBytesPerSample = unpackedBytesPerSample;
            view.packing = packing;
            view.bayerPattern = pattern;
            view.timestampUs = timestampUs;

            metadata = metadataFromRequest(
                sanitizedPhotoSettings,
                request->metadata(),
                frameIndex,
                timestampUs,
                wallClockForSensorTimestamp(timestampUs),
                sensorFamily_,
                dngModel_, whiteBalance, pattern == BayerPattern::Monochrome);
            if (pending.bracket) {
                if (pending.settings.whiteBalanceAvailable) {
                    metadata.whiteBalanceKelvin = pending.settings.whiteBalanceKelvin;
                    if (pending.lockedWb.valid()) {
                        metadata.colourGains[0] = pending.lockedWb.red;
                        metadata.colourGains[1] = pending.lockedWb.blue;
                    }
                }
                if (pending.lockedCcm) {
                    metadata.hasColourMatrix = true;
                    std::copy_n(pending.lockedCcm->begin(), 9, metadata.colourMatrix);
                }
            }
            if (needsPreview) {
                // Use the actual ISP CCM, not a DNG calibration matrix or a later frame's WB.
                if (pending.bracket && pending.lockedCcm) {
                    rendering.ccm = *pending.lockedCcm;
                } else if (auto matrix = request->metadata().get(libcamera::controls::ColourCorrectionMatrix);
                    matrix && matrix->size() >= 9 &&
                    std::all_of(matrix->begin(), matrix->end(), [](float value) { return std::isfinite(value); })) {
                    std::copy_n(matrix->begin(), 9, rendering.ccm.begin());
                } else if (metadata.hasColourMatrix) {
                    std::copy_n(metadata.colourMatrix, 9, rendering.ccm.begin());
                }
                rendering.toneCurve = activeTuningGammaCurve();
            }
        }

        if (!photoWriter_.enqueue(std::move(rawCopy),
                                  view,
                                  std::move(metadata),
                                  pending.basePath,
                                  pending.format,
                                  std::move(rendering),
                                  std::move(pending.onSaved))) {
            std::cerr << "ApertarCore photo skipped: could not queue write for "
                      << pending.basePath << "\n";
        } else if (pending.notifyCapture && exposureEvent_) {
            exposureEvent_("{\"event\":\"photo_capture\"}");
        }
    }

    void trackRecordingCadence(int64_t timestampUs, int64_t frameDurationUs)
    {
        if (timestampUs <= 0)
            return;

        if (!haveLastRecordingSensorTimestamp_) {
            haveLastRecordingSensorTimestamp_ = true;
            lastRecordingSensorTimestampUs_ = timestampUs;
            return;
        }

        if (timestampUs <= lastRecordingSensorTimestampUs_) {
            lastRecordingSensorTimestampUs_ = timestampUs;
            return;
        }

        const int64_t deltaUs = timestampUs - lastRecordingSensorTimestampUs_;
        lastRecordingSensorTimestampUs_ = timestampUs;

        int64_t nominalFrameUs = frameDurationUs;
        if (nominalFrameUs <= 0) {
            const double safeFps = std::max(1.0, settings_.fps);
            nominalFrameUs = static_cast<int64_t>(1000000.0 / safeFps);
        }
        if (nominalFrameUs <= 0)
            return;

        if (deltaUs > (nominalFrameUs * 3) / 2) {
            const int64_t estimate = (deltaUs + (nominalFrameUs / 2)) / nominalFrameUs - 1;
            if (estimate <= 0)
                return;

            const uint64_t droppedEstimate = static_cast<uint64_t>(estimate);
            recordingGapEstimate_ += droppedEstimate;
            dropWarning_.markDropped();
            if (recordingGapEstimate_ == 1 || (recordingGapEstimate_ % 30) == 0 || droppedEstimate > 1) {
                std::cerr << "ApertarCore drop[capture-gap]: +"
                          << droppedEstimate
                          << " frame(s) estimate"
                          << " dt_us=" << deltaUs
                          << " nominal_us=" << nominalFrameUs
                          << " total_estimate=" << recordingGapEstimate_
                          << "\n";
            }
        }
    }

    void publishPreviewFrame(libcamera::Request *request)
    {
        if (!previewCallback_ || !previewStream_)
            return;

        libcamera::FrameBuffer *buffer = request->findBuffer(previewStream_);
        if (!buffer || buffer->planes().empty())
            return;

        const libcamera::StreamConfiguration &config = previewStream_->configuration();
        const auto &planes = buffer->planes();
        const libcamera::FrameBuffer::Plane &plane = planes.front();
        const size_t sourcePlaneCount = std::min<size_t>(planes.size(), 3);

        int64_t timestampUs = 0;
        if (auto ts = request->metadata().get(libcamera::controls::SensorTimestamp))
            timestampUs = static_cast<int64_t>(*ts / 1000);
        else
            timestampUs = static_cast<int64_t>(buffer->metadata().timestamp / 1000);

        const uint32_t slotIndex = static_cast<uint32_t>(request->cookie());
        if (publishedPreviewSlots_.find(slotIndex) == publishedPreviewSlots_.end()) {
            PreviewFrameDescriptor registration;
            registration.registration = true;
            registration.slotIndex = slotIndex;
            if (sourcePlaneCount >= 3) {
                bool duplicatedAllPlanes = true;
                for (size_t i = 0; i < sourcePlaneCount; ++i) {
                    registration.planeFds[i] = ::dup(planes[i].fd.get());
                    if (registration.planeFds[i] < 0) {
                        duplicatedAllPlanes = false;
                        break;
                    }
                }
                if (duplicatedAllPlanes)
                    registration.fd = registration.planeFds[0];
                else {
                    for (int &fd : registration.planeFds) {
                        if (fd >= 0) {
                            ::close(fd);
                            fd = -1;
                        }
                    }
                }
            } else {
                registration.fd = ::dup(plane.fd.get());
            }
            if (registration.fd >= 0) {
                registration.width = config.size.width;
                registration.height = config.size.height;
                registration.stride = config.stride;
                registration.captureWidth = settings_.resolution.width;
                registration.captureHeight = settings_.resolution.height;
                registration.size = plane.length;
                registration.fourcc = config.pixelFormat.toString();
                registration.timestampUs = timestampUs;
                registration.frameId = frameCounter_.load(std::memory_order_relaxed);

                const unsigned int yPitch = config.stride;
                const unsigned int chromaPitch = std::max(1u, config.stride / 2);

                if (sourcePlaneCount >= 3) {
                    registration.planeCount = static_cast<int>(sourcePlaneCount);
                    for (size_t i = 0; i < sourcePlaneCount; ++i) {
                        registration.planeOffsets[i] = planes[i].offset == libcamera::FrameBuffer::Plane::kInvalidOffset
                            ? 0
                            : planes[i].offset;
                        registration.planeLengths[i] = planes[i].length;
                    }
                    registration.planePitches[0] = yPitch;
                    registration.planePitches[1] = chromaPitch;
                    registration.planePitches[2] = chromaPitch;
                } else {
                    const uint32_t baseOffset = plane.offset == libcamera::FrameBuffer::Plane::kInvalidOffset
                        ? 0
                        : plane.offset;
                    const unsigned int yHeight = config.size.height;
                    const unsigned int chromaHeight = std::max(1u, config.size.height / 2);

                    registration.planeCount = 3;
                    registration.planeOffsets[0] = baseOffset;
                    registration.planePitches[0] = yPitch;
                    registration.planeLengths[0] = yPitch * yHeight;
                    registration.planeOffsets[1] = baseOffset + registration.planeLengths[0];
                    registration.planePitches[1] = chromaPitch;
                    registration.planeLengths[1] = chromaPitch * chromaHeight;
                    registration.planeOffsets[2] = registration.planeOffsets[1] + registration.planeLengths[1];
                    registration.planePitches[2] = chromaPitch;
                    registration.planeLengths[2] = chromaPitch * chromaHeight;
                }

                publishedPreviewSlots_.insert(slotIndex);
                previewCallback_(std::move(registration));
            }
        }

        PreviewFrameDescriptor frame;
        frame.registration = false;
        frame.slotIndex = slotIndex;
        frame.frameId = frameCounter_.fetch_add(1, std::memory_order_relaxed) + 1;
        frame.width = config.size.width;
        frame.height = config.size.height;
        frame.stride = config.stride;
        frame.captureWidth = settings_.resolution.width;
        frame.captureHeight = settings_.resolution.height;
        frame.size = plane.length;
        frame.fourcc = config.pixelFormat.toString();
        frame.timestampUs = timestampUs;

        if (sourcePlaneCount >= 3) {
            frame.planeCount = static_cast<int>(sourcePlaneCount);
            for (size_t i = 0; i < sourcePlaneCount; ++i) {
                frame.planeFds[i] = planes[i].fd.get();
                frame.planeOffsets[i] = planes[i].offset == libcamera::FrameBuffer::Plane::kInvalidOffset
                    ? 0
                    : planes[i].offset;
                frame.planeLengths[i] = planes[i].length;
            }
            frame.planePitches[0] = config.stride;
            frame.planePitches[1] = std::max(1u, config.stride / 2);
            frame.planePitches[2] = std::max(1u, config.stride / 2);
        } else {
            const uint32_t baseOffset = plane.offset == libcamera::FrameBuffer::Plane::kInvalidOffset
                ? 0
                : plane.offset;
            const uint32_t ySize = config.stride * config.size.height;
            const uint32_t chromaPitch = std::max(1u, config.stride / 2);
            const uint32_t chromaHeight = std::max(1u, config.size.height / 2);

            frame.planeCount = 3;
            frame.planeFds[0] = plane.fd.get();
            frame.planeFds[1] = plane.fd.get();
            frame.planeFds[2] = plane.fd.get();
            frame.planeOffsets[0] = baseOffset;
            frame.planeOffsets[1] = baseOffset + ySize;
            frame.planeOffsets[2] = baseOffset + ySize + (chromaPitch * chromaHeight);
            frame.planePitches[0] = config.stride;
            frame.planePitches[1] = chromaPitch;
            frame.planePitches[2] = chromaPitch;
            frame.planeLengths[0] = ySize;
            frame.planeLengths[1] = chromaPitch * chromaHeight;
            frame.planeLengths[2] = chromaPitch * chromaHeight;
        }

        previewCallback_(std::move(frame));
    }

    bool recordProcessedPreviewFrame(libcamera::Request *request)
    {
        libcamera::Stream *encodeStream = h264Stream_ ? h264Stream_ : previewStream_;
        const bool h264Active = h264Recorder_.active();
        const bool proresActive = proresRecorder_.active();
        if ((!h264Active && !proresActive) || !encodeStream)
            return false;

        libcamera::FrameBuffer *buffer = request->findBuffer(encodeStream);
        if (!buffer || buffer->planes().empty())
            return false;

        auto mappedIt = mappedPreviewBuffers_.find(buffer);
        if (mappedIt == mappedPreviewBuffers_.end() || !mappedIt->second.data)
            return false;

        const libcamera::StreamConfiguration &config = encodeStream->configuration();
        const auto &planes = buffer->planes();
        const size_t sourcePlaneCount = std::min<size_t>(planes.size(), 3);
        const uint32_t width = config.size.width;
        const uint32_t height = config.size.height;
        const uint32_t yPitch = config.stride ? config.stride : width;
        const uint32_t chromaPitch = std::max(1u, yPitch / 2);
        const uint32_t chromaHeight = std::max(1u, height / 2);
        uint32_t planeOffsets[3] = { 0, 0, 0 };
        uint32_t planePitches[3] = { yPitch, chromaPitch, chromaPitch };
        int64_t timestampUs = 0;
        if (auto ts = request->metadata().get(libcamera::controls::SensorTimestamp))
            timestampUs = static_cast<int64_t>(*ts / 1000);
        else
            timestampUs = static_cast<int64_t>(buffer->metadata().timestamp / 1000);

        int64_t frameDurationUs = 0;
        if (auto frameDuration = request->metadata().get(libcamera::controls::FrameDuration);
            frameDuration && *frameDuration > 0) {
            frameDurationUs = normalizeFrameDurationToUs(static_cast<int64_t>(*frameDuration), settings_.fps);
        }

        if (proresActive) {
            // Count camera cadence before encoder backpressure rejects frames.
            trackRecordingCadence(timestampUs, frameDurationUs);
            const auto &plane = planes.front();
            const size_t offset = plane.offset == libcamera::FrameBuffer::Plane::kInvalidOffset ? 0 : plane.offset;
            if (offset > mappedIt->second.mappingLength) return false;
            // Cached CPU reads must be bracketed until the encoder releases the buffer.
            const int cachedFd = cachedProresBuffers_.empty() ? -1 : plane.fd.get();
            if (cachedFd >= 0 && !syncCpuReadBuffer(cachedFd, true)) return false;
            const bool queued = proresRecorder_.enqueueFrame(mappedIt->second.data + offset,
                mappedIt->second.mappingLength - offset, width, height, config.stride, timestampUs,
                [this, request, cachedFd] {
                    if (cachedFd >= 0) syncCpuReadBuffer(cachedFd, false);
                    requeueRequest(request);
                });
            if (!queued && cachedFd >= 0) syncCpuReadBuffer(cachedFd, false);
            if (queued) {
                const auto frameIndex = proresFrameCounter_.fetch_add(1, std::memory_order_relaxed);
                if (frameIndex == 0) gyroRecorder_.setVideoStartTimestampUs(uint64_t(std::max<int64_t>(0, timestampUs)), 0);
                audioRecorder_.observeVideoFrame(timestampUs, frameDurationUs);
            }
            return queued;
        }

        if (sourcePlaneCount >= 3) {
            const int fd = planes.front().fd.get();
            for (size_t i = 0; i < sourcePlaneCount; ++i) {
                if (planes[i].fd.get() != fd)
                    return false;
                planeOffsets[i] = planes[i].offset == libcamera::FrameBuffer::Plane::kInvalidOffset
                    ? 0
                    : planes[i].offset;
            }
        } else {
            const uint32_t baseOffset = planes.front().offset == libcamera::FrameBuffer::Plane::kInvalidOffset
                ? 0
                : planes.front().offset;
            const uint32_t ySize = yPitch * height;
            planeOffsets[0] = baseOffset;
            planeOffsets[1] = baseOffset + ySize;
            planeOffsets[2] = baseOffset + ySize + (chromaPitch * chromaHeight);
        }

        const auto releaseRequest = [this, request]() {
            requeueRequest(request);
        };
        const bool queued = h264Recorder_.enqueueFrame(mappedIt->second.data,
                                         width,
                                         height,
                                         planeOffsets,
                                         planePitches,
                                         timestampUs,
                                         releaseRequest);
        if (queued) {
            const uint64_t frameIndex = h264FrameCounter_.fetch_add(1, std::memory_order_relaxed);
            trackRecordingCadence(timestampUs, frameDurationUs);
            if (frameIndex == 0)
                gyroRecorder_.setVideoStartTimestampUs(static_cast<uint64_t>(std::max<int64_t>(0, timestampUs)), frameIndex);
            return true;
        }
        return false;
    }

    std::unique_ptr<libcamera::CameraManager> cameraManager_;
    bool photoStreamOverride_ = false;
    bool monochromeSensor_ = false;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> configuration_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::FrameBuffer>> cachedProresBuffers_;
    std::vector<std::unique_ptr<libcamera::FrameBuffer>> cachedRawBuffers_;
    libcamera::Stream *previewStream_ = nullptr;
    libcamera::Stream *h264Stream_ = nullptr;
    libcamera::Stream *rawStream_ = nullptr;
    CachedRawStreamInfo rawStreamInfo_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::unordered_map<libcamera::FrameBuffer *, MappedPlane> mappedRawBuffers_;
    std::unordered_map<libcamera::FrameBuffer *, MappedPreviewBuffer> mappedPreviewBuffers_;
    libcamera::ControlList pendingControls_;
    bool longExposureTimingUsed_ = false;
    std::mutex controlsMutex_;
    mutable std::mutex whiteBalanceMutex_;
    WhiteBalanceMeasurement whiteBalanceMeasured_;
    uint32_t measuredExposureUs_ = 0;
    float measuredAnalogueGain_ = 0;
    std::optional<std::array<float, 9>> measuredCcm_;
    bool warnedMissingWhiteBalance_ = false;
    std::atomic<bool> running_{false};
    bool capturePriorityAdjusted_ = false;
    std::atomic<uint64_t> frameCounter_{0};
    CaptureSettings settings_{};
    uint32_t configuredSensorWidth_ = 0;
    RecordingFormat lastRecordingFormat_ = RecordingFormat::Cdng;
    uint32_t configuredSensorHeight_ = 0;
    DngRecordingQueue recorder_;
    H264PreviewRecorder h264Recorder_;
    ProresRecorder proresRecorder_;
    std::atomic<uint64_t> proresFrameCounter_{0};
    AudioRecordingSession audioRecorder_;
    RecordingClock recordingClock_;
    std::mutex recordingFrameMutex_;
    bool recordingFramesEnabled_ = false;
    bool waitingForRecordingCadence_ = false;
    int64_t recordingStartBootUs_ = 0;
    Mpu6050RecordingSession gyroRecorder_;
    PhotoCaptureWriter photoWriter_;
    mutable std::shared_mutex sessionSettingsMutex_;
    mutable std::mutex photoCaptureMutex_;
    std::atomic<unsigned> photoFramesInFlight_{0};
    std::atomic<bool> recordingStopInProgress_{false};
    std::string stillCaptureError_;
    bool stillExposureActive_ = false;
    bool restoreStillPreviewControls_ = false;
    std::chrono::steady_clock::time_point stillExposureDeadline_{};
    std::deque<PendingPhotoCapture> pendingPhotoCaptures_;
    std::atomic<uint64_t> photoFrameCounter_{0};
    std::unordered_set<uint32_t> publishedPreviewSlots_;
    SensorFamily sensorFamily_ = SensorFamily::Unknown;
    std::string dngModel_ = "Generic";
    bool warnedUnsupportedRaw_ = false;
    uint64_t recordingGapEstimate_ = 0;
    bool haveLastRecordingSensorTimestamp_ = false;
    int64_t lastRecordingSensorTimestampUs_ = 0;
    std::atomic<uint64_t> h264FrameCounter_{0};
    uint64_t requeueFailures_ = 0;
    const size_t maxPendingPhotoCaptures_ = 12;
    bool acquired_ = false;
    bool started_ = false;
#endif

    PreviewCallback previewCallback_;
    AudioLevelCallback audioLevelCallback_;
    RecordingDropWarning &dropWarning_;
    EventCallback exposureEvent_;
    std::mutex mediaWriteEventMutex_;
    bool lastMediaWritesPending_ = false;
    uint64_t mediaWriteRevision_ = 0;

    void emitMediaWriteState()
    {
        // Serialize edges, and never call this while holding the photo queue lock.
        std::lock_guard lock(mediaWriteEventMutex_);
        const bool pending = hasPendingPhotoCaptures();
        if (pending == lastMediaWritesPending_) return;
        lastMediaWritesPending_ = pending;
        ++mediaWriteRevision_;
        if (exposureEvent_)
            exposureEvent_(std::string("{\"event\":\"media_write_state\",\"media_writes_pending\":") +
                           (pending ? "true" : "false") + ",\"media_write_revision\":" +
                           std::to_string(mediaWriteRevision_) + "}");
    }

    void emitStillExposureEvent()
    {
        if (!exposureEvent_) return;
        const auto status = stillExposureStatus();
        std::ostringstream event;
        event << "{\"event\":\"still_exposure\",\"still_exposure_active\":" << (status.active ? "true" : "false")
              << ",\"still_exposure_remaining_seconds\":" << status.remainingSeconds << "}";
        exposureEvent_(event.str());
        emitMediaWriteState();
    }
};

CameraController::CameraController(EventCallback eventCallback, PreviewCallback previewCallback)
    : eventCallback_(std::move(eventCallback))
    , previewCallback_(std::move(previewCallback))
    , recordingDropWarning_([this](bool dropped) {
          if (eventCallback_)
              eventCallback_(std::string("{\"event\":\"recording_health\",\"recording_has_dropped_frames\":")
                             + (dropped ? "true}" : "false}"));
      })
    , session_(std::make_unique<CameraSession>(
          previewCallback_,
          [this](int level) {
              if (!eventCallback_)
                  return;

              std::ostringstream out;
              out << "{\"event\":\"audio_level\",\"input_level\":"
                  << std::clamp(level, 0, 100)
                  << "}";
              eventCallback_(out.str());
          }, recordingDropWarning_, eventCallback_))
{
    settings_ = loadPersistedCaptureSettings(formatSettings_, stillSettings_);
    if (unsupportedRecordingSettings(settings_))
        std::cerr << "ApertarCore: saved recording settings are unavailable on this board; selecting supported recording settings.\n";
    settings_ = supportedRecordingSettings(settings_);
    if (settings_.tuningFile.empty()) {
        if (const char *path = std::getenv("LIBCAMERA_RPI_TUNING_FILE"); path && *path)
            settings_.tuningFile = normalizeTuningFilePath(path);
    }
    if (!settings_.tuningFile.empty()) {
        std::string tuningError;
        if (!validateTuningFilePath(settings_.tuningFile, &tuningError)) {
            std::cerr << "ApertarCore ignoring tuning file: " << tuningError << '\n';
            settings_.tuningFile.clear();
        }
    }
    applyProcessTuningFile(settings_.tuningFile);
}

CameraController::~CameraController()
{
    stop();
    captureSettingsPersister().flush();
}

bool CameraController::start(const std::string &mediaRoot, bool simulateCamera)
{
    std::lock_guard<std::mutex> lock(mutex_);
    mediaRoot_ = mediaRoot;
    simulateCamera_ = simulateCamera;
    state_ = CameraState::Starting;
    applyProcessTuningFile(settings_.tuningFile);

    if (!simulateCamera_ && !session_->start(settings_, &lastError_)) {
        state_ = CameraState::Error;
        return false;
    }

    if (!simulateCamera_ && session_)
        settings_ = session_->currentSettings();
    else if (simulateCamera_)
        settings_ = sanitizeSettingsForSensor(settings_, SensorFamily::Unknown);

    if (simulateCamera_)
        constrainExposureMetering(settings_, std::getenv("LIBCAMERA_RPI_TUNING_FILE")
            ? activeTuningExposureMeteringModes() : 7);

    state_ = CameraState::Ready;
    lastError_.clear();
    persistSettingsLocked();
    emitSettingsEventLocked();
    return true;
}

void CameraController::stop()
{
    stopTimelapse();

    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == CameraState::Offline)
        return;

    recording_ = false;
    state_ = CameraState::Stopping;

    if (session_)
        session_->stop();

    state_ = CameraState::Offline;
}

bool CameraController::enterStandby()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == CameraState::Standby)
        return true;
    if (recording_ || timelapseActive() || state_ != CameraState::Ready) {
        setErrorLocked("Standby requires an idle camera, with recording and timelapse stopped.");
        return false;
    }
    // Drain still writes before releasing the camera; never discard a pending photo.
    if (session_ && !simulateCamera_) {
        if (session_->hasPendingPhotoCaptures()) {
            setErrorLocked("Wait for the still capture to finish before entering standby.");
            return false;
        }
        session_->stop();
    }
    state_ = CameraState::Standby;
    lastError_.clear();
    if (eventCallback_)
        eventCallback_("{\"event\":\"standby\",\"standby\":true}");
    return true;
}

bool CameraController::wakeFromStandby()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CameraState::Standby)
        return state_ == CameraState::Ready;
    if (!simulateCamera_ && session_ && !session_->start(settings_, &lastError_)) {
        // Keep the wake action available after a failure, rather than showing a dead UI.
        return false;
    }
    if (!simulateCamera_ && session_)
        settings_ = session_->currentSettings();
    state_ = CameraState::Ready;
    lastError_.clear();
    emitSettingsEventLocked();
    if (eventCallback_)
        eventCallback_("{\"event\":\"standby\",\"standby\":false}");
    return true;
}

bool CameraController::setResolution(Resolution resolution)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (recording_) {
        setErrorLocked("Resolution changes are blocked while recording.");
        return false;
    }

    if (!validateResolution(resolution))
        return false;

    const SensorFamily sensorFamily = (!simulateCamera_ && session_)
        ? session_->sensorFamily()
        : SensorFamily::Unknown;
    CaptureSettings current = sanitizeSettingsForSensor(settings_, sensorFamily);
    CaptureSettings requested = settings_;
    requested.resolution = resolution;
    requested = sanitizeSettingsForSensor(requested, sensorFamily);
    if (current.resolution.width == requested.resolution.width &&
        current.resolution.height == requested.resolution.height) {
        lastError_.clear();
        emitSettingsEventLocked();
        return true;
    }

    CaptureSettings next = settings_;
    next.resolution = resolution;
    return applySettingsLocked(next, true);
}

bool CameraController::setFps(double fps)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (recording_ && std::abs(fps - settings_.fps) > 0.0001) {
        setErrorLocked("Stop recording before changing FPS so video and audio keep the same timeline.");
        return false;
    }
    if (fps <= 0.0 || fps > 280.0) {
        setErrorLocked("FPS must be between 0 and 280.");
        return false;
    }

    const SensorFamily sensorFamily = (!simulateCamera_ && session_)
        ? session_->sensorFamily()
        : SensorFamily::Unknown;
    CaptureSettings requested = settings_;
    requested.fps = fps;
    requested = sanitizeSettingsForSensor(requested, sensorFamily);
    const double effectiveFps = requested.fps;

    if (std::abs(settings_.fps - effectiveFps) < 0.0001) {
        lastError_.clear();
        emitSettingsEventLocked();
        return true;
    }

    CaptureSettings next = settings_;
    if (!next.shutterAuto && !(next.photoModeEnabled && next.longExposureEnabled))
        next.shutterUs = shutterUsForPreservedAngle(next.shutterUs, next.fps, effectiveFps);
    next.fps = effectiveFps;
    return applySettingsLocked(next, false);
}

bool CameraController::setIso(uint32_t iso)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (iso < 50 || iso > 51200) {
        setErrorLocked("ISO must be between 50 and 51200.");
        return false;
    }

    CaptureSettings next = settings_;
    next.iso = iso;
    next.isoAuto = false;
    return applySettingsLocked(next, false);
}

bool CameraController::setIsoAuto()
{
    std::lock_guard<std::mutex> lock(mutex_);
    CaptureSettings next = settings_;
    next.isoAuto = true;
    return applySettingsLocked(next, false);
}

bool CameraController::setShutterUs(uint32_t shutterUs)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutterUs == 0 || shutterUs > 30'000'000) {
        setErrorLocked("Shutter time must be greater than zero and at most 30 seconds.");
        return false;
    }
    if (settings_.photoModeEnabled && settings_.timelapseExposureLimitUs &&
        shutterUs > settings_.timelapseExposureLimitUs) {
        setErrorLocked("Shutter time cannot exceed the timelapse interval.");
        return false;
    }
    const uint32_t ordinaryMaximum = uint32_t(std::ceil(1'000'000 / (settings_.photoModeEnabled ? std::min(24.0, settings_.fps) : settings_.fps)));
    if (shutterUs > ordinaryMaximum &&
        !(settings_.photoModeEnabled && settings_.longExposureEnabled)) {
        setErrorLocked("Enable Long Exposure in Still Mode for shutter times longer than a frame.");
        return false;
    }
    if (settings_.photoModeEnabled && settings_.longExposureEnabled && !simulateCamera_ && session_ &&
        shutterUs > session_->maximumStillExposureUs()) {
        setErrorLocked("This sensor mode supports at most " +
                       std::to_string(session_->maximumStillExposureUs() / 1000000.0) + " seconds exposure.");
        return false;
    }

    CaptureSettings next = settings_;
    next.shutterUs = shutterUs;
    next.shutterAuto = false;
    return applySettingsLocked(next, false);
}

bool CameraController::setShutterAuto()
{
    std::lock_guard<std::mutex> lock(mutex_);
    CaptureSettings next = settings_;
    next.shutterAuto = true;
    return applySettingsLocked(next, false);
}

bool CameraController::setExposureCompensation(double ev)
{
    std::lock_guard lock(mutex_);
    if (!std::isfinite(ev) || ev < -3.0 || ev > 3.0) {
        setErrorLocked("Exposure compensation must be between -3 and +3 EV.");
        return false;
    }
    if (!simulateCamera_ && session_ && !session_->exposureCompensationAvailable()) {
        setErrorLocked("Exposure compensation is unavailable for this camera pipeline.");
        return false;
    }
    CaptureSettings next = settings_;
    next.exposureCompensation = std::round(ev * 3.0) / 3.0;
    return applySettingsLocked(next, false);
}

bool CameraController::setExposureMetering(const std::string &value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto mode = normalizeExposureMetering(value);
    if (mode.empty() || !(settings_.exposureMeteringModes & exposureMeteringBit(mode))) {
        setErrorLocked("This metering mode is not supported by the active tuning/pipeline.");
        return false;
    }
    CaptureSettings next = settings_;
    next.exposureMetering = mode;
    return applySettingsLocked(next, false);
}

bool CameraController::setStillExposureMode(bool photoMode, bool longExposure, double intervalSeconds)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (recording_ || timelapseActive()) {
        setErrorLocked("Stop recording or timelapse before changing still exposure mode.");
        return false;
    }
    CaptureSettings next = settings_;
    next.photoModeEnabled = photoMode;
    if (photoMode != settings_.photoModeEnabled)
        restoreModeGeometryLocked(next);
    next.longExposureEnabled = longExposure;
    next.timelapseExposureLimitUs = photoMode ? timelapseExposureLimitUs(intervalSeconds) : 0;
    next = clampCaptureSettings(next);
    if (next.photoModeEnabled == settings_.photoModeEnabled &&
        next.longExposureEnabled == settings_.longExposureEnabled && next.shutterUs == settings_.shutterUs) {
        settings_.timelapseExposureLimitUs = next.timelapseExposureLimitUs;
        lastError_.clear();
        emitSettingsEventLocked();
        return true;
    }
    return applySettingsLocked(next, photoMode != settings_.photoModeEnabled);
}

bool CameraController::setWhiteBalanceKelvin(uint32_t kelvin)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!settings_.whiteBalanceAvailable) {
        setErrorLocked("White balance is unavailable: the active tuning has no calibrated AWB/CT curve.");
        return false;
    }
    if (kelvin < 1500 || kelvin > 12000) {
        setErrorLocked("White balance must be between 1500K and 12000K.");
        return false;
    }

    CaptureSettings next = settings_;
    next.whiteBalanceKelvin = kelvin;
    next.whiteBalanceAuto = false;
    return applySettingsLocked(next, false);
}

bool CameraController::setWhiteBalanceAuto()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!settings_.whiteBalanceAvailable) {
        setErrorLocked("White balance is unavailable: the active tuning has no calibrated AWB/CT curve.");
        return false;
    }
    CaptureSettings next = settings_;
    next.whiteBalanceAuto = true;
    return applySettingsLocked(next, false);
}

bool CameraController::setRecordingFormat(RecordingFormat format)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto requested = settings_;
    requested.recordingFormat = format;
    if (const char *error = unsupportedRecordingSettings(requested)) {
        setErrorLocked(error);
        return false;
    }
    if (recording_) {
        setErrorLocked("Recording format changes are blocked while recording.");
        return false;
    }

    const bool fixedTenBitFormat = !settings_.photoModeEnabled && isFixedTenBitRecordingFormat(format);
    if (settings_.recordingFormat == format && (!fixedTenBitFormat || settings_.rawBitDepth == 10)) {
        lastError_.clear();
        emitSettingsEventLocked();
        return true;
    }

    CaptureSettings next = settings_;
    const auto oldProfile = settingsProfileFormat(settings_.recordingFormat);
    const auto newProfile = settingsProfileFormat(format);
    if (!settings_.photoModeEnabled && oldProfile != newProfile) {
        const auto saved = formatSettings_.find(newProfile);
        if (saved != formatSettings_.end()) {
            const std::string tuningFile = next.tuningFile;
            const TimecodeMode timecodeMode = next.timecodeMode;
            const std::string metering = next.exposureMetering;
            next = saved->second;
            next.tuningFile = tuningFile;
            next.timecodeMode = timecodeMode;
            next.exposureMetering = metering;
            if (format == RecordingFormat::Cdng)
                format = next.recordingFormat;
        }
    }
    next.recordingFormat = format;
    next.photoModeEnabled = settings_.photoModeEnabled;
    next.longExposureEnabled = settings_.longExposureEnabled;
    next.bracketingEnabled = settings_.bracketingEnabled;
    next.bracketingEv = settings_.bracketingEv;
    next.timelapseExposureLimitUs = settings_.timelapseExposureLimitUs;
    next = supportedRecordingSettings(next);
    if (fixedTenBitFormat)
        next.rawBitDepth = 10;
    const bool currentUsesProcessedStream = !settings_.photoModeEnabled && isProcessedVideoFormat(settings_.recordingFormat);
    const bool nextUsesProcessedStream = !next.photoModeEnabled && isProcessedVideoFormat(format);
    const bool requiresReconfigure = currentUsesProcessedStream || nextUsesProcessedStream ||
        next.resolution.width != settings_.resolution.width || next.resolution.height != settings_.resolution.height ||
        next.rawBitDepth != settings_.rawBitDepth;
    return applySettingsLocked(next, requiresReconfigure);
}

bool CameraController::setCdngCompression(CdngCompression compression)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (recording_) {
        setErrorLocked("Compression changes are blocked while recording.");
        return false;
    }
    CaptureSettings next = settings_;
    next.cdngCompression = compression;
    return applySettingsLocked(next, false);
}

bool CameraController::setLossyDngRatio(LossyDngRatio ratio)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!recordingCapabilities().rawCompression) {
        setErrorLocked("Lossy cDNG recording requires Raspberry Pi 5-class hardware.");
        return false;
    }
    if (recording_) {
        setErrorLocked("Lossy cDNG ratio changes are blocked while recording.");
        return false;
    }
    CaptureSettings next = settings_;
    next.lossyDngRatio = ratio;
    return applySettingsLocked(next, false);
}

bool CameraController::setProresProfile(ProresProfile profile)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!recordingCapabilities().prores) {
        setErrorLocked("ProRes recording requires Raspberry Pi 5-class hardware.");
        return false;
    }
    if (recording_) { setErrorLocked("ProRes profile changes are blocked while recording."); return false; }
    CaptureSettings next = settings_;
    next.proresProfile = profile;
    return applySettingsLocked(next, false);
}

bool CameraController::setProresColorProfile(ProresColorProfile profile)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!recordingCapabilities().prores) {
        setErrorLocked("ProRes recording requires Raspberry Pi 5-class hardware.");
        return false;
    }
    if (recording_ || timelapseActive()) {
        setErrorLocked("Stop recording/timelapse before changing ProRes colour.");
        return false;
    }
    if (settings_.proresColorProfile == profile) return true;
    CaptureSettings next = settings_;
    next.proresColorProfile = profile;
    return applySettingsLocked(next, !next.photoModeEnabled && next.recordingFormat == RecordingFormat::Prores);
}

bool CameraController::setCdngPreviewColorProfile(ProresColorProfile profile)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (recording_ || timelapseActive()) {
        setErrorLocked("Stop recording/timelapse before changing the cDNG preview profile.");
        return false;
    }
    if (settings_.cdngPreviewColorProfile == profile) return true;
    CaptureSettings next = settings_;
    next.cdngPreviewColorProfile = profile;
    return applySettingsLocked(next, activeIspColorProfile(next) != activeIspColorProfile(settings_));
}

bool CameraController::setTuningFile(const std::string &path)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == CameraState::Standby) {
        setErrorLocked("Wake the camera before changing tuning files.");
        return false;
    }
    if (recording_) {
        setErrorLocked("Tuning file changes are blocked while recording.");
        return false;
    }
    if (timelapseActive() || (!simulateCamera_ && session_ && session_->hasPendingPhotoCaptures())) {
        setErrorLocked("Wait for still captures and timelapse to finish before changing tuning files.");
        return false;
    }

    const std::string normalized = normalizeTuningFilePath(path);
    std::string validationError;
    if (!validateTuningFilePath(normalized, &validationError)) {
        setErrorLocked(validationError);
        return false;
    }

    if (settings_.tuningFile == normalized) {
        lastError_.clear();
        emitSettingsEventLocked();
        return true;
    }

    CaptureSettings next = settings_;
    next.tuningFile = normalized;
    // A new calibration must not retain a CCM locked from the previous tuning.

    if (simulateCamera_ || !session_) {
        settings_ = next;
        applyProcessTuningFile(settings_.tuningFile);
        constrainExposureMetering(settings_, normalized.empty() ? 7 : activeTuningExposureMeteringModes());
        persistSettingsLocked();
        lastError_.clear();
        emitSettingsEventLocked();
        return true;
    }

    const CaptureSettings previous = settings_;
    applyProcessTuningFile(normalized);
    if (!session_->reconfigure(next, &lastError_)) {
        const std::string applyError = lastError_;
        applyProcessTuningFile(previous.tuningFile);

        std::string recoverError;
        if (session_->reconfigure(previous, &recoverError)) {
            settings_ = session_->currentSettings();
            state_ = CameraState::Ready;
            lastError_ = "Could not apply tuning file: " + applyError;
        } else {
            state_ = CameraState::Error;
            lastError_ = "Could not apply tuning file: " + applyError
                + "; recovery failed: " + recoverError;
        }
        emitSettingsEventLocked();
        return false;
    }

    settings_ = session_->currentSettings();
    persistSettingsLocked();
    lastError_.clear();
    emitSettingsEventLocked();
    return true;
}

bool CameraController::setRawBitDepth(uint32_t bitDepth)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (recording_) {
        setErrorLocked("Raw bit depth changes are blocked while recording.");
        return false;
    }

    const SensorFamily sensorFamily = (!simulateCamera_ && session_)
        ? session_->sensorFamily()
        : SensorFamily::Unknown;
    if (!settings_.photoModeEnabled && isFixedTenBitRecordingFormat(settings_.recordingFormat)) {
        CaptureSettings next = settings_;
        next.rawBitDepth = 10;
        lastError_.clear();
        if (settings_.rawBitDepth == next.rawBitDepth) {
            emitSettingsEventLocked();
            return true;
        }
        return applySettingsLocked(next, true);
    }

    CaptureSettings requested = settings_;
    requested.rawBitDepth = normalizeRawBitDepth(bitDepth);
    requested = sanitizeSettingsForSensor(requested, sensorFamily);
    if (settings_.rawBitDepth == requested.rawBitDepth) {
        lastError_.clear();
        emitSettingsEventLocked();
        return true;
    }

    CaptureSettings next = settings_;
    next.rawBitDepth = requested.rawBitDepth;
    return applySettingsLocked(next, true);
}

bool CameraController::setH264Resolution(Resolution resolution)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (recording_) {
        setErrorLocked("H.264 resolution changes are blocked while recording.");
        return false;
    }

    if (!isSupportedProcessedVideoResolution(settings_.recordingFormat, resolution)) {
        setErrorLocked("Unsupported H.264 recording resolution.");
        return false;
    }

    const Resolution normalized = normalizeProcessedVideoResolution(settings_.recordingFormat, resolution);
    if (settings_.h264Resolution.width == normalized.width &&
        settings_.h264Resolution.height == normalized.height) {
        lastError_.clear();
        emitSettingsEventLocked();
        return true;
    }

    CaptureSettings next = settings_;
    next.h264Resolution = normalized;
    return applySettingsLocked(next, !settings_.photoModeEnabled && isProcessedVideoFormat(settings_.recordingFormat));
}

bool CameraController::setTimecodeMode(TimecodeMode mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    CaptureSettings next = settings_;
    next.timecodeMode = mode;
    return applySettingsLocked(next, false);
}

bool CameraController::setAutofocusMode(AutofocusMode mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    CaptureSettings next = settings_;
    next.autofocusMode = mode;
    return applySettingsLocked(next, false);
}

bool CameraController::setAutofocusRange(AutofocusRange range)
{
    std::lock_guard<std::mutex> lock(mutex_);
    CaptureSettings next = settings_;
    next.autofocusRange = range;
    return applySettingsLocked(next, false);
}

bool CameraController::setAutofocusSpeed(AutofocusSpeed speed)
{
    std::lock_guard<std::mutex> lock(mutex_);
    CaptureSettings next = settings_;
    next.autofocusSpeed = speed;
    return applySettingsLocked(next, false);
}

bool CameraController::setAutofocusWindow(double x, double y, double width, double height)
{
    std::lock_guard<std::mutex> lock(mutex_);
    CaptureSettings next = settings_;
    next.autofocusWindowX = x;
    next.autofocusWindowY = y;
    next.autofocusWindowWidth = width;
    next.autofocusWindowHeight = height;
    return applySettingsLocked(next, false);
}

bool CameraController::startRecording(const AudioRecordingOptions &audioOptions,
                                      const GyroRecordingOptions &gyroOptions)
{
    if (timelapseActive()) {
        std::lock_guard<std::mutex> lock(mutex_);
        setErrorLocked("Stop timelapse before recording.");
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CameraState::Ready) {
        setErrorLocked("Camera is not ready.");
        return false;
    }

    if (const char *error = unsupportedRecordingSettings(settings_)) {
        setErrorLocked(error);
        return false;
    }
    if (settings_.photoModeEnabled && settings_.longExposureEnabled &&
        settings_.shutterUs > uint32_t(std::ceil(1'000'000 / settings_.fps))) {
        setErrorLocked("Leave Still Mode before starting a long-exposure clip recording.");
        return false;
    }
    if (!simulateCamera_ && session_ && session_->hasPendingPhotoCaptures()) {
        setErrorLocked("Wait for the still capture to finish before recording.");
        return false;
    }
    if (!simulateCamera_ && session_ && !session_->startRecording(mediaRoot_, settings_, audioOptions, gyroOptions, &lastError_)) {
        // Recording startup failures are usually media/audio setup problems, not
        // fatal camera-session failures. Keep the controller ready so the user
        // can retry without restarting the whole app.
        recording_ = false;
        state_ = CameraState::Ready;
        return false;
    }

    if (simulateCamera_) recordingDropWarning_.reset();
    recording_ = true;
    recordingFailureReported_ = false;
    recordingFailureMessage_.clear();
    state_ = CameraState::Recording;
    lastError_.clear();
    emitRecordingEventLocked();
    return true;
}

bool CameraController::capturePhoto(PhotoCaptureFormat format, unsigned count)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (timelapseActive() || (!simulateCamera_ && session_ && session_->hasPendingPhotoCaptures())) {
        setErrorLocked("Wait for the current capture to finish writing.");
        return false;
    }
    if (count < 1 || count > 10) {
        setErrorLocked("Photo burst count must be between 1 and 10.");
        return false;
    }
    if (usesStillBracketing(settings_)) count = 3;
    return queuePhotoCaptureLocked(format, {}, {}, count);
}

bool CameraController::queuePhotoCaptureLocked(PhotoCaptureFormat format, const std::string &basePath,
                                               std::function<void()> onSaved, unsigned count)
{
    if (recording_) {
        setErrorLocked("Photo capture is unavailable while recording.");
        return false;
    }

    if (state_ != CameraState::Ready) {
        setErrorLocked("Camera is not ready.");
        return false;
    }

    if (!simulateCamera_ && session_ && !session_->capturePhoto(mediaRoot_, settings_, format, &lastError_, basePath, std::move(onSaved), count))
        return false;
    if (simulateCamera_ && onSaved) onSaved();

    lastError_.clear();
    return true;
}

bool CameraController::startTimelapse(PhotoCaptureFormat format, double intervalSeconds)
{
#if APERTARCORE_HAS_LIBCAMERA
    if (!photoFormatIsTimelapseSafe(format)) {
        std::lock_guard<std::mutex> lock(mutex_);
        setErrorLocked("Timelapse supports DNG, JPEG, PNG, or TIFF only.");
        return false;
    }
#endif

    if (!std::isfinite(intervalSeconds))
        return false;
    intervalSeconds = std::round(std::clamp(intervalSeconds, 0.1, 86400.0) * 10.0) / 10.0;

    std::string mediaRoot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (recording_) {
            setErrorLocked("Timelapse is unavailable while recording.");
            return false;
        }

        if (state_ != CameraState::Ready) {
            setErrorLocked("Camera is not ready.");
            return false;
        }
        if (!timelapseActive() && !simulateCamera_ && session_ && session_->hasPendingPhotoCaptures()) {
            setErrorLocked("Wait for the current capture to finish writing before starting timelapse.");
            return false;
        }
        CaptureSettings next = settings_;
        next.photoModeEnabled = true;
        if (!settings_.photoModeEnabled)
            restoreModeGeometryLocked(next);
        next.timelapseExposureLimitUs = timelapseExposureLimitUs(intervalSeconds);
        next = clampCaptureSettings(next);
        if (next.shutterUs != settings_.shutterUs || next.photoModeEnabled != settings_.photoModeEnabled) {
            if (!applySettingsLocked(next, false)) return false;
        } else {
            settings_.timelapseExposureLimitUs = next.timelapseExposureLimitUs;
        }
        mediaRoot = mediaRoot_;
    }

    bool updatedActiveTimelapse = false;
    std::string activeFolderPath;
    {
        std::lock_guard<std::mutex> lock(timelapseMutex_);
        if (timelapseActive_) {
            timelapseFormat_ = format;
            timelapseIntervalSeconds_ = intervalSeconds;
            activeFolderPath = timelapseFolderPath_;
            updatedActiveTimelapse = true;
        }
    }
    if (updatedActiveTimelapse) {
        timelapseCv_.notify_all();
        emitTimelapseEvent(true, intervalSeconds, activeFolderPath);
        return true;
    }

    std::string folderPath;
    try {
        folderPath = allocateTimelapseFolder(mediaRoot).string();
    } catch (const std::exception &e) {
        std::lock_guard<std::mutex> lock(mutex_);
        setErrorLocked(e.what());
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(timelapseMutex_);
        timelapseStopRequested_ = false;
        timelapseActive_ = true;
        timelapseIntervalSeconds_ = intervalSeconds;
        timelapseFrameIndex_ = 1;
        timelapseShotCount_ = 0;
        ++timelapseSequence_;
        timelapseFormat_ = format;
        timelapseFolderPath_ = folderPath;
        emitTimelapseProgressLocked();
    }

    emitTimelapseEvent(true, intervalSeconds, folderPath);
    timelapseThread_ = std::thread(&CameraController::timelapseLoop, this);
    return true;
}

bool CameraController::stopTimelapse()
{
    std::string folderPath;
    double intervalSeconds = 0;
    {
        std::lock_guard<std::mutex> lock(timelapseMutex_);
        if (!timelapseActive_ && !timelapseThread_.joinable())
            return true;

        timelapseStopRequested_ = true;
        timelapseActive_ = false;
        intervalSeconds = timelapseIntervalSeconds_;
        folderPath = timelapseFolderPath_;
    }

    timelapseCv_.notify_all();
    if (timelapseThread_.joinable())
        timelapseThread_.join();

    emitTimelapseEvent(false, intervalSeconds, folderPath);
    return true;
}

bool CameraController::timelapseActive() const
{
    std::lock_guard<std::mutex> lock(timelapseMutex_);
    return timelapseActive_;
}

double CameraController::timelapseIntervalSeconds() const
{
    std::lock_guard<std::mutex> lock(timelapseMutex_);
    return timelapseIntervalSeconds_;
}

uint64_t CameraController::timelapseShotCount() const
{
    std::lock_guard<std::mutex> lock(timelapseMutex_);
    return timelapseShotCount_;
}

void CameraController::completeTimelapseShot(uint64_t sequence)
{
    std::lock_guard<std::mutex> lock(timelapseMutex_);
    // A pending exposure may finish after Stop, or after a new sequence starts.
    if (sequence != timelapseSequence_) return;
    ++timelapseShotCount_;
    emitTimelapseProgressLocked();
}

void CameraController::emitTimelapseProgressLocked()
{
    if (eventCallback_)
        eventCallback_("{\"event\":\"timelapse_progress\",\"timelapse_shot_count\":" +
                       std::to_string(timelapseShotCount_) + "}");
}

void CameraController::timelapseLoop()
{
    while (true) {
        PhotoCaptureFormat format = PhotoCaptureFormat::Dng;
        double intervalSeconds = 1;
        std::string folderPath;
        uint64_t frameIndex = 1;
        uint64_t sequence = 0;
        {
            std::lock_guard<std::mutex> lock(timelapseMutex_);
            if (timelapseStopRequested_)
                break;

            format = timelapseFormat_;
            intervalSeconds = std::max(0.1, timelapseIntervalSeconds_);
            folderPath = timelapseFolderPath_;
            frameIndex = timelapseFrameIndex_;
            sequence = timelapseSequence_;
        }

        const std::string basePath = timelapseFrameBasePath(folderPath, frameIndex).string();
        bool queued = false;
        std::string queueError;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!simulateCamera_ && session_ && session_->hasPendingPhotoCaptures()) {
                queueError = "Previous still is still exposing or writing.";
            } else {
                queued = queuePhotoCaptureLocked(format, basePath,
                    [this, sequence] { completeTimelapseShot(sequence); });
            }
            queueError = lastError_;
        }

        if (queued) {
            {
                std::lock_guard<std::mutex> lock(timelapseMutex_);
                if (timelapseFrameIndex_ == frameIndex)
                    ++timelapseFrameIndex_;
            }
            emitTimelapseFrameEvent(frameIndex);
        } else {
            std::cerr << "ApertarCore timelapse skipped frame "
                      << frameIndex << ": " << queueError << "\n";
        }

        std::unique_lock<std::mutex> lock(timelapseMutex_);
        timelapseCv_.wait_for(lock, std::chrono::milliseconds(std::llround(intervalSeconds * 1000)), [this, intervalSeconds] {
            return timelapseStopRequested_ || timelapseIntervalSeconds_ != intervalSeconds;
        });
        if (timelapseStopRequested_)
            break;
    }
}

bool CameraController::stopRecording()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!recording_)
        return true;

    recording_ = false;
    state_ = CameraState::Ready;
    if (!simulateCamera_ && session_)
        session_->stopRecording();
    lastError_ = !simulateCamera_ && session_ ? session_->recordingError() : std::string();
    recordingFailureReported_ = !lastError_.empty();
    recordingFailureMessage_ = lastError_;
    emitRecordingEventLocked();
    return lastError_.empty();
}

bool CameraController::checkRecordingFailure()
{
    std::lock_guard lock(mutex_);
    if (simulateCamera_ || !session_ || recordingFailureReported_ ||
        (state_ != CameraState::Ready && state_ != CameraState::Recording)) return false;
    const auto error = session_->recordingError();
    if (error.empty()) return false;
    recordingFailureReported_ = true;
    session_->stopRecording();
    recording_ = false;
    state_ = CameraState::Ready;
    lastError_ = error;
    recordingFailureMessage_ = error;
    emitRecordingEventLocked();
    return true;
}

bool CameraController::mediaWritesPending() const
{
    std::lock_guard lock(mutex_);
    return !simulateCamera_ && session_ && session_->hasPendingPhotoCaptures();
}

CameraController::MediaWriteStatus CameraController::mediaWriteStatus() const
{
    std::lock_guard lock(mutex_);
    return !simulateCamera_ && session_ ? session_->mediaWriteStatus() : MediaWriteStatus{};
}

CameraController::StillExposureStatus CameraController::stillExposureStatus() const
{
    std::lock_guard lock(mutex_);
    return !simulateCamera_ && session_ ? session_->stillExposureStatus() : StillExposureStatus{};
}

std::string CameraController::photoWriteError() const
{
    std::lock_guard lock(mutex_);
    return !simulateCamera_ && session_ ? session_->photoWriteError() : std::string{};
}

CameraState CameraController::state() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

CaptureSettings CameraController::settings() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    CaptureSettings result = settings_;
    result.exposureCompensationAvailable = simulateCamera_ || (session_ && session_->exposureCompensationAvailable());
    if (!simulateCamera_ && session_)
        result.whiteBalanceMeasured = session_->measuredWhiteBalance();
    return result;
}

std::string CameraController::sensorName() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (simulateCamera_)
        return "Simulated Sensor";
    if (!session_)
        return "Unknown";
    return sensorFamilyName(session_->sensorFamily());
}

bool CameraController::isRecording() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return recording_;
}

std::string CameraController::lastError() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return lastError_;
}

std::string CameraController::recordingError() const
{
    std::lock_guard lock(mutex_);
    return recordingFailureMessage_;
}

bool CameraController::applyCapturePreset(const CaptureSettings &requested, const std::function<bool()> &commitUi)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (recording_ || timelapseActive() || state_ != CameraState::Ready) {
        setErrorLocked("Stop recording/timelapse and wake the camera before loading a preset.");
        return false;
    }
    if (!simulateCamera_ && session_ && session_->hasPendingPhotoCaptures()) {
        setErrorLocked("Wait for still captures to finish before loading a preset.");
        return false;
    }
    if (!validateResolution(requested.resolution)) return false;
    CaptureSettings next = supportedRecordingSettings(requested);
    next.tuningFile = normalizeTuningFilePath(next.tuningFile);
    if (!validateTuningFilePath(next.tuningFile, &lastError_)) return false;
    const CaptureSettings previous = settings_;
    const bool tuningChanged = next.tuningFile != previous.tuningFile;
    const bool reconfigure = tuningChanged || next.photoModeEnabled != previous.photoModeEnabled || next.recordingFormat != previous.recordingFormat ||
        next.resolution.width != previous.resolution.width || next.resolution.height != previous.resolution.height ||
        next.rawBitDepth != previous.rawBitDepth || next.h264Resolution.width != previous.h264Resolution.width ||
        next.h264Resolution.height != previous.h264Resolution.height;
    if (tuningChanged) applyProcessTuningFile(next.tuningFile);
    // Apply the complete mode once; intermediate modes must not clamp FPS/ISO.
    const bool applied = applySettingsLocked(next, reconfigure, false);
    if (!applied && state_ != CameraState::Error) {
        // Validation/busy rejection never changed the session. Restarting here
        // would stop an in-flight burst despite the pending-capture guard.
        if (tuningChanged) applyProcessTuningFile(previous.tuningFile);
        return false;
    }
    if (applied) {
        if (!commitUi || commitUi()) {
            persistSettingsLocked();
            emitSettingsEventLocked();
            return true;
        }
        lastError_ = "Could not update shared UI settings from the preset.";
    }
    const auto error = lastError_;
    if (tuningChanged) applyProcessTuningFile(previous.tuningFile);
    if (!simulateCamera_ && session_) {
        std::string recoveryError;
        if (session_->reconfigure(previous, &recoveryError)) {
            settings_ = session_->currentSettings();
            state_ = CameraState::Ready;
        } else {
            lastError_ = error + "; recovery failed: " + recoveryError;
            return false;
        }
    } else settings_ = previous;
    lastError_ = error;
    emitSettingsEventLocked();
    return false;
}

bool CameraController::applySettingsLocked(CaptureSettings nextSettings, bool requiresReconfigure, bool commit)
{
    requiresReconfigure |= nextSettings.photoModeEnabled != settings_.photoModeEnabled;
    requiresReconfigure |= activeIspColorProfile(nextSettings) != activeIspColorProfile(settings_);
    if (!nextSettings.photoModeEnabled) nextSettings.timelapseExposureLimitUs = 0;
    if (nextSettings.timelapseExposureLimitUs)
        nextSettings.shutterUs = std::min(nextSettings.shutterUs, nextSettings.timelapseExposureLimitUs);
    if (!simulateCamera_ && session_ && (requiresReconfigure || (settings_.photoModeEnabled &&
        (settings_.longExposureEnabled || settings_.bracketingEnabled || nextSettings.bracketingEnabled))) &&
        session_->hasPendingPhotoCaptures()) {
        setErrorLocked("Wait for still captures to finish before changing camera mode or capture settings.");
        return false;
    }
    if (const char *error = unsupportedRecordingSettings(nextSettings)) {
        setErrorLocked(error);
        return false;
    }
    if (state_ == CameraState::Standby) {
        setErrorLocked("Wake the camera before changing capture settings.");
        return false;
    }
    if (!simulateCamera_ && session_) {
        const auto reconfigure = [&](const CaptureSettings &settings, std::string *error) {
            try {
                return session_->reconfigure(settings, error, settings.photoModeEnabled);
            } catch (const std::exception &e) {
                *error = e.what();
                return false;
            }
        };
        const bool applied = requiresReconfigure
            ? reconfigure(nextSettings, &lastError_)
            : session_->applyControls(nextSettings, &lastError_);
        if (!applied) {
            if (requiresReconfigure && commit) {
                const std::string switchError = lastError_.empty()
                    ? "Camera rejected the requested capture settings." : lastError_;
                std::string recoveryError;
                // Reconfiguration stops capture first. Keep the last working
                // mode usable when the replacement cannot allocate or configure.
                if (reconfigure(settings_, &recoveryError)) {
                    settings_ = session_->currentSettings();
                    state_ = CameraState::Ready;
                    lastError_ = switchError;
                    emitSettingsEventLocked();
                    return false;
                }
                lastError_ = switchError + "; recovery failed: " + recoveryError;
            }
            state_ = CameraState::Error;
            return false;
        }
    }

    settings_ = simulateCamera_
        ? sanitizeSettingsForSensor(nextSettings, SensorFamily::Unknown)
        : session_->currentSettings();
    if (simulateCamera_)
        constrainExposureMetering(settings_, std::getenv("LIBCAMERA_RPI_TUNING_FILE")
            ? activeTuningExposureMeteringModes() : 7);
    if (commit) persistSettingsLocked();
    lastError_.clear();
    if (commit) emitSettingsEventLocked();
    return true;
}

void CameraController::persistSettingsLocked()
{
    if (settings_.photoModeEnabled)
        stillSettings_ = settings_;
    else
        formatSettings_[settingsProfileFormat(settings_.recordingFormat)] = settings_;
    captureSettingsPersister().schedule(settings_, formatSettings_, stillSettings_);
}

void CameraController::restoreModeGeometryLocked(CaptureSettings &next) const
{
    const CaptureSettings *saved = nullptr;
    if (next.photoModeEnabled) {
        if (stillSettings_) saved = &*stillSettings_;
        else if (!isProcessedVideoFormat(settings_.recordingFormat)) saved = &settings_;
    } else {
        const auto profile = formatSettings_.find(settingsProfileFormat(next.recordingFormat));
        if (profile != formatSettings_.end()) saved = &profile->second;
    }
    const CaptureSettings fallback;
    if (!saved) saved = &fallback;
    next.resolution = saved->resolution;
    next.rawBitDepth = saved->rawBitDepth;
    next.fps = saved->fps;
}

bool CameraController::validateResolution(Resolution resolution)
{
    if ((resolution.width == 1332 && resolution.height == 990) ||
        (resolution.width == 2028 && resolution.height == 1080) ||
        (resolution.width == 2028 && resolution.height == 1520) ||
        (resolution.width == 4056 && resolution.height == 2160) ||
        (resolution.width == 4056 && resolution.height == 3040) ||
        isImx477Resolution(resolution) ||
        isImx294Resolution(resolution) ||
        isImx585LowResolutionAlias(resolution) ||
        isImx585FullHdCropResolution(resolution) ||
        isImx585HighResolutionAlias(resolution) ||
        isImx585Raw12CropResolution(resolution)) {
        return true;
    }

    setErrorLocked("Unsupported resolution.");
    return false;
}

void CameraController::setErrorLocked(std::string error)
{
    lastError_ = std::move(error);
    state_ = state_ == CameraState::Offline ? CameraState::Offline : state_;
}

void CameraController::emitSettingsEventLocked()
{
    settings_.exposureCompensationAvailable = simulateCamera_ || (session_ && session_->exposureCompensationAvailable());
    settings_.whiteBalanceAvailable = simulateCamera_ ||
        ((!session_ || !session_->monochromeSensor()) && activeTuningHasCalibratedWhiteBalance());
    if (eventCallback_)
        eventCallback_(settingsEvent(settings_));
}

void CameraController::emitRecordingEventLocked()
{
    if (!eventCallback_)
        return;

    std::ostringstream out;
    out << "{\"event\":\"recording\",\"active\":" << (recording_ ? "true" : "false")
        << ",\"recording_has_dropped_frames\":" << (recordingDropWarning_.dropped() ? "true" : "false")
        << ",\"error\":\"" << apertar::escapeJson(lastError_) << "\"}";
    eventCallback_(out.str());
}

void CameraController::emitTimelapseEvent(bool active, double intervalSeconds, const std::string &folderPath)
{
    if (!eventCallback_)
        return;

    std::ostringstream out;
    out << "{\"event\":\"timelapse\""
        << ",\"active\":" << (active ? "true" : "false")
        << ",\"interval_seconds\":" << intervalSeconds
        << ",\"folder\":\"";

    for (char ch : folderPath) {
        switch (ch) {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default: out << ch; break;
        }
    }

    out << "\"}";
    eventCallback_(out.str());
}

void CameraController::emitTimelapseFrameEvent(uint64_t frameIndex)
{
    if (!eventCallback_)
        return;

    std::ostringstream out;
    out << "{\"event\":\"timelapse_frame\",\"index\":" << frameIndex << "}";
    eventCallback_(out.str());
}

} // namespace apertar
