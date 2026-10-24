#include "ApertarCoreApp.hpp"
#include <apertar/core/PreviewExposureReference.hpp>
#include <apertar/core/RecordingCapabilities.hpp>
#include "camera/ExposureMetering.hpp"
#include "hardware/ButtonActionList.hpp"
#include "hardware/Mpu6050RecordingSession.hpp"
#include "web/WebControlPage.hpp"
#include "util/SettingsFile.hpp"
#include "audio/AudioSample.hpp"
#include "audio/TimestampedAudioCapture.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ifaddrs.h>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <net/if.h>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

#if APERTARCORE_HAS_TURBOJPEG
#include <turbojpeg.h>
#elif APERTARCORE_HAS_JPEG
#include <jpeglib.h>
#endif

namespace apertar {
namespace {

constexpr uint32_t kWebPreviewMaxWidth = 640;
constexpr uint32_t kWebPreviewMaxHeight = 360;
constexpr double kWebPreviewMaxFps = 30.0;
constexpr int kWebPreviewJpegQuality = 75;
constexpr size_t kMaxEditableHardwareConfigBytes = 256 * 1024;

std::string resolveExecutablePath(std::initializer_list<const char *> candidates,
                                  const char *fallbackName)
{
    for (const char *candidate : candidates) {
        if (candidate && ::access(candidate, X_OK) == 0)
            return candidate;
    }

    return fallbackName ? std::string(fallbackName) : std::string();
}

std::string existingExecutablePath(std::initializer_list<const char *> candidates)
{
    for (const char *candidate : candidates) {
        if (candidate && ::access(candidate, X_OK) == 0)
            return candidate;
    }
    return {};
}

const std::string &timedatectlProgram()
{
    static const std::string path = resolveExecutablePath(
        {"/usr/bin/timedatectl",
         "/bin/timedatectl",
         "/usr/sbin/timedatectl",
         "/sbin/timedatectl"},
        "timedatectl");
    return path;
}

const std::string &hwclockProgram()
{
    static const std::string path = resolveExecutablePath(
        {"/usr/sbin/hwclock",
         "/sbin/hwclock",
         "/usr/bin/hwclock",
         "/bin/hwclock"},
        "hwclock");
    return path;
}

std::string okReply(int id, const std::string &body = {})
{
    std::ostringstream out;
    out << "{\"ok\":true";
    if (id >= 0)
        out << ",\"id\":" << id;
    if (!body.empty())
        out << "," << body;
    out << "}";
    return out.str();
}

std::string errorReply(int id, const std::string &message)
{
    std::ostringstream out;
    out << "{\"ok\":false";
    if (id >= 0)
        out << ",\"id\":" << id;
    out << ",\"error\":\"" << escapeJson(message) << "\"}";
    return out.str();
}

std::string imuStatusBody(bool includeLiveSample)
{
    const Mpu6050RecordingSession::ConfigSnapshot imu =
        Mpu6050RecordingSession::configuredSensorSnapshot(includeLiveSample);
    std::ostringstream out;
    out << "\"imu\":{"
        << "\"enabled\":" << (imu.enabled ? "true" : "false")
        << ",\"driver\":\"" << escapeJson(imu.driver) << "\""
        << ",\"driver_label\":\"" << escapeJson(imu.driverLabel) << "\""
        << ",\"sync_mode\":\"" << escapeJson(imu.syncMode) << "\""
        << ",\"bus\":" << imu.busNumber
        << ",\"address\":" << imu.address
        << ",\"address_hex\":\"0x" << std::hex << imu.address << std::dec << "\""
        << ",\"sample_rate_hz\":" << imu.sampleRateHz
        << ",\"gyro_range_dps\":" << imu.gyroRangeDps
        << ",\"accel_range_g\":" << imu.accelRangeG
        << ",\"orientation\":\"" << escapeJson(imu.orientation) << "\""
        << ",\"sync_offset_ms\":" << std::fixed << std::setprecision(3) << imu.syncOffsetMs
        << ",\"sync_offset_text\":\"" << escapeJson(imu.syncOffsetText) << "\""
        << ",\"level_reference_set\":" << (imu.levelReferenceSet ? "true" : "false")
        << ",\"level_reference_text\":\"" << escapeJson(imu.levelReferenceText) << "\""
        << ",\"level_accel_x_g\":" << std::fixed << std::setprecision(4) << imu.levelAccelXG
        << ",\"level_accel_y_g\":" << std::fixed << std::setprecision(4) << imu.levelAccelYG
        << ",\"level_accel_z_g\":" << std::fixed << std::setprecision(4) << imu.levelAccelZG
        << ",\"live_available\":" << (imu.liveAvailable ? "true" : "false")
        << ",\"live_status_text\":\"" << escapeJson(imu.liveStatusText) << "\""
        << ",\"accel_x_g\":" << std::fixed << std::setprecision(3) << imu.accelXG
        << ",\"accel_y_g\":" << std::fixed << std::setprecision(3) << imu.accelYG
        << ",\"accel_z_g\":" << std::fixed << std::setprecision(3) << imu.accelZG
        << ",\"gyro_x_dps\":" << std::fixed << std::setprecision(2) << imu.gyroXDps
        << ",\"gyro_y_dps\":" << std::fixed << std::setprecision(2) << imu.gyroYDps
        << ",\"gyro_z_dps\":" << std::fixed << std::setprecision(2) << imu.gyroZDps
        << ",\"config_path\":\"" << escapeJson(imu.configPath) << "\""
        << "}";
    return out.str();
}

std::string settingsBody(const CaptureSettings &settings)
{
    std::ostringstream out;
    out << "\"resolution\":\"" << settings.resolution.toString() << "\""
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
        << ",\"recording_capabilities\":" << recordingCapabilitiesJson()
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
        << ",\"af_window_h\":" << std::fixed << std::setprecision(3) << settings.autofocusWindowHeight;
    return out.str();
}

void closeOwnedPreviewFrame(PreviewFrameDescriptor *frame)
{
    if (!frame)
        return;

    // Only preview buffer registrations own duplicated DMABUF fds. Per-frame
    // preview markers reference libcamera-owned fds and must never close them.
    if (!frame->registration)
        return;

    const int primaryFd = frame->fd;
    if (primaryFd >= 0) {
        ::close(primaryFd);
        frame->fd = -1;
    }

    for (int &planeFd : frame->planeFds) {
        if (planeFd >= 0 && planeFd != primaryFd)
            ::close(planeFd);
        planeFd = -1;
    }
}

PreviewFrameDescriptor duplicatePreviewFrameDescriptor(const PreviewFrameDescriptor &frame)
{
    PreviewFrameDescriptor duplicate = frame;
    duplicate.fd = -1;
    for (int &planeFd : duplicate.planeFds)
        planeFd = -1;

    if (frame.fd >= 0) {
        duplicate.fd = ::dup(frame.fd);
        if (duplicate.fd < 0) {
            closeOwnedPreviewFrame(&duplicate);
            return PreviewFrameDescriptor{};
        }
    }

    for (int i = 0; i < duplicate.planeCount && i < 3; ++i) {
        if (frame.planeFds[i] < 0)
            continue;

        if (frame.fd >= 0 && frame.planeFds[i] == frame.fd) {
            if (duplicate.fd < 0) {
                closeOwnedPreviewFrame(&duplicate);
                return PreviewFrameDescriptor{};
            }
            duplicate.planeFds[i] = duplicate.fd;
            continue;
        }

        duplicate.planeFds[i] = ::dup(frame.planeFds[i]);
        if (duplicate.planeFds[i] < 0) {
            closeOwnedPreviewFrame(&duplicate);
            return PreviewFrameDescriptor{};
        }
    }

    return duplicate;
}

std::string jsonNumber(double value, int precision = 1)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

std::string jsonNumberOrNull(const std::optional<double> &value, int precision = 1)
{
    if (!value.has_value())
        return "null";
    return jsonNumber(value.value(), precision);
}

std::string jsonStringArray(const std::vector<std::string> &values)
{
    std::ostringstream out;
    out << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0)
            out << ',';
        out << '"' << escapeJson(values[i]) << '"';
    }
    out << ']';
    return out.str();
}

std::optional<long long> readLongLongFile(const std::filesystem::path &path)
{
    std::ifstream input(path);
    long long value = 0;
    if (!(input >> value))
        return std::nullopt;
    return value;
}

std::filesystem::path nearestExistingPath(std::filesystem::path path)
{
    std::error_code ec;
    while (!path.empty() && !std::filesystem::exists(path, ec))
        path = path.parent_path();
    return path;
}

std::string trimAscii(std::string value)
{
    const auto notSpace = [](unsigned char ch) {
        return !std::isspace(ch);
    };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
    return value;
}

std::string lowerAscii(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

std::filesystem::path sharedUiSettingsPath()
{
    if (const char *xdgConfig = std::getenv("XDG_CONFIG_HOME"); xdgConfig && *xdgConfig)
        return std::filesystem::path(xdgConfig) / "CinePi" / "UI.conf";
    if (const char *home = std::getenv("HOME"); home && *home)
        return std::filesystem::path(home) / ".config" / "CinePi" / "UI.conf";
    return std::filesystem::temp_directory_path() / "CinePi-UI.conf";
}

bool parseUiSettingsBool(const std::string &value, bool fallback)
{
    const std::string normalized = trimAscii(value);
    if (normalized == "true" || normalized == "1")
        return true;
    if (normalized == "false" || normalized == "0")
        return false;
    return fallback;
}

std::string toLowerAsciiCopy(std::string value);

std::string normalizeUiOrientationValue(const std::string &value)
{
    if (value == "Left Side" || value == "Right Side" || value == "Upside Down")
        return value;
    if (value == "Left")
        return "Right Side";
    if (value == "Right")
        return "Left Side";
    return "Landscape";
}

std::string normalizeExternalMonitorOrientationValue(const std::string &value)
{
    if (value == "Landscape" || value == "Upside Down")
        return value;
    return "Landscape";
}

std::string normalizeTimecodeModeValue(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "rec run" || normalized == "rec_run")
        return "Rec Run";
    return "Free Run";
}

std::string normalizeCameraControlsModeValue(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    return normalized == "dark" ? "Dark" : "Light";
}

AutofocusMode autofocusModeFromCommandValue(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "manual")
        return AutofocusMode::Manual;
    if (normalized == "auto")
        return AutofocusMode::Auto;
    return AutofocusMode::Continuous;
}

AutofocusRange autofocusRangeFromCommandValue(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "macro")
        return AutofocusRange::Macro;
    if (normalized == "full")
        return AutofocusRange::Full;
    return AutofocusRange::Normal;
}

AutofocusSpeed autofocusSpeedFromCommandValue(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "fast")
        return AutofocusSpeed::Fast;
    return AutofocusSpeed::Normal;
}

std::string normalizePhotoTimerValue(const std::string &value)
{
    const std::string trimmed = trimAscii(value);
    if (trimmed == "2s" || trimmed == "5s" || trimmed == "10s" || trimmed == "15s" ||
        trimmed == "20s" || trimmed == "25s" || trimmed == "30s") {
        return trimmed;
    }
    return "Off";
}

std::string normalizePhotoBurstValue(const std::string &value)
{
    const std::string trimmed = trimAscii(value);
    if (trimmed == "3 Shots" || trimmed == "5 Shots" || trimmed == "10 Shots")
        return trimmed;
    return "Single";
}

unsigned photoBurstCountFromValue(const std::string &value)
{
    return value == "3 Shots" ? 3 : value == "5 Shots" ? 5 : value == "10 Shots" ? 10 : 1;
}

std::string normalizePhotoFormatValue(const std::string &value)
{
    const std::string trimmed = trimAscii(value);
    const std::string normalized = toLowerAsciiCopy(trimmed);
    if (normalized == "jpg" || normalized == "jpeg")
        return "JPEG";
    if (normalized == "png")
        return "PNG";
    if (normalized == "tif" || normalized == "tiff")
        return "TIFF";
    if (normalized == "dng+jpeg" || normalized == "dng + jpeg" ||
        normalized == "dng_jpeg" || normalized == "dng-jpeg") {
        return "DNG + JPEG";
    }
    if (normalized == "dng+png" || normalized == "dng + png" ||
        normalized == "dng_png" || normalized == "dng-png") {
        return "DNG + PNG";
    }
    if (normalized == "dng+tif" || normalized == "dng + tif" ||
        normalized == "dng_tif" || normalized == "dng-tif" ||
        normalized == "dng+tiff" || normalized == "dng + tiff" ||
        normalized == "dng_tiff" || normalized == "dng-tiff") {
        return "DNG + TIFF";
    }
    return "DNG";
}

std::string normalizePhotoTimelapseValue(const std::string &value)
{
    const std::string trimmed = trimAscii(value);
    const std::string normalized = toLowerAsciiCopy(trimmed);
    if (normalized.empty() ||
        normalized == "off" ||
        normalized == "none" ||
        normalized == "disabled") {
        return "Off";
    }

    const bool minutes = normalized.back() == 'm';
    const bool hasUnit = minutes || normalized.back() == 's';
    const std::string digits = hasUnit ? normalized.substr(0, normalized.size() - 1) : normalized;
    const auto dot = digits.find('.');
    if (digits.empty() || (dot != std::string::npos && (minutes || dot + 2 != digits.size())))
        return "Off";
    std::string wholeDigits = digits;
    if (dot != std::string::npos)
        wholeDigits.erase(dot, 1);
    if (wholeDigits.empty() || !std::all_of(wholeDigits.begin(), wholeDigits.end(), [](unsigned char ch) { return std::isdigit(ch); }))
        return "Off";
    uint32_t tenths = 0;
    try {
        const auto parsed = std::stoull(wholeDigits);
        const auto limit = minutes ? 1440u : (dot == std::string::npos ? 86400u : 864000u);
        if (parsed < 1 || parsed > limit)
            return "Off";
        tenths = static_cast<uint32_t>(parsed) * (minutes ? 600u : (dot == std::string::npos ? 10u : 1u));
    } catch (...) {
        return "Off";
    }

    if ((minutes || tenths >= 1200) && tenths % 600 == 0)
        return std::to_string(tenths / 600) + "m";
    return std::to_string(tenths / 10) + (tenths % 10 ? "." + std::to_string(tenths % 10) : "") + "s";
}

double photoTimelapseSecondsFromValue(const std::string &value)
{
    const std::string normalized = normalizePhotoTimelapseValue(value);
    if (normalized == "Off")
        return 0;

    const double amount = std::stod(normalized.substr(0, normalized.size() - 1));
    return normalized.back() == 'm' ? amount * 60 : amount;
}

bool photoFormatIsTimelapseSafeValue(const std::string &value)
{
    const std::string normalized = normalizePhotoFormatValue(value);
    return normalized == "DNG" ||
           normalized == "JPEG" ||
           normalized == "PNG" ||
           normalized == "TIFF";
}

PhotoCaptureFormat photoCaptureFormatFromValue(const std::string &value)
{
    const std::string normalized = normalizePhotoFormatValue(value);
    if (normalized == "JPEG")
        return PhotoCaptureFormat::Jpeg;
    if (normalized == "PNG")
        return PhotoCaptureFormat::Png;
    if (normalized == "TIFF")
        return PhotoCaptureFormat::Tiff;
    if (normalized == "DNG + JPEG")
        return PhotoCaptureFormat::DngJpeg;
    if (normalized == "DNG + PNG")
        return PhotoCaptureFormat::DngPng;
    if (normalized == "DNG + TIFF")
        return PhotoCaptureFormat::DngTiff;
    return PhotoCaptureFormat::Dng;
}

std::string normalizeBatteryCapacityValue(const std::string &value)
{
    const std::string trimmed = trimAscii(value);
    if (trimmed == "Custom")
        return trimmed;

    std::string digits;
    for (const char ch : trimmed) {
        if (std::isdigit(static_cast<unsigned char>(ch)))
            digits.push_back(ch);
    }

    if (!digits.empty()) {
        try {
            const int wh = std::clamp(std::stoi(digits), 10, 500);
            return std::to_string(wh) + "Wh";
        } catch (const std::exception &) {
            return "150Wh";
        }
    }

    return "150Wh";
}

std::string normalizeBatteryTypeValue(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "np-f" || normalized == "npf" || normalized == "sony np-f")
        return "NP-F";
    if (normalized == "lp-e6" || normalized == "lpe6" || normalized == "canon lp-e6")
        return "LP-E6";
    if (normalized == "usb-c / dc" || normalized == "usb-c" || normalized == "dc" ||
        normalized == "dc input" || normalized == "external")
        return "USB-C / DC";
    if (normalized == "custom")
        return "Custom";
    return "V-Mount";
}

double clampBatteryVoltageValue(double value, double fallback)
{
    if (!std::isfinite(value))
        return fallback;
    return std::clamp(value, 0.5, 60.0);
}

double batteryDefaultFullVoltage(const std::string &type)
{
    if (type == "NP-F" || type == "LP-E6")
        return 8.4;
    return 16.8;
}

double batteryDefaultEmptyVoltage(const std::string &type)
{
    if (type == "NP-F" || type == "LP-E6")
        return 6.0;
    return 13.2;
}

std::string defaultBatteryCapacityForType(const std::string &type)
{
    if (type == "NP-F")
        return "37Wh";
    if (type == "LP-E6")
        return "16Wh";
    if (type == "Custom" || type == "USB-C / DC")
        return "Custom";
    return "150Wh";
}

bool batteryCapacityFitsType(const std::string &type, const std::string &capacity)
{
    if (capacity == "Custom")
        return true;
    if (type == "NP-F")
        return capacity == "15Wh" || capacity == "24Wh" || capacity == "37Wh" || capacity == "50Wh";
    if (type == "LP-E6")
        return capacity == "13Wh" || capacity == "16Wh" || capacity == "20Wh";
    if (type == "USB-C / DC")
        return false;
    return capacity == "50Wh" || capacity == "95Wh" || capacity == "150Wh" || capacity == "190Wh";
}

std::string normalizedBatteryCapacityForType(const std::string &type, const std::string &capacity)
{
    const std::string normalized = normalizeBatteryCapacityValue(capacity);
    if (batteryCapacityFitsType(type, normalized))
        return normalized;
    return defaultBatteryCapacityForType(type);
}

std::string normalizeSleepModeValue(const std::string &value)
{
    const std::string trimmed = trimAscii(value);
    if (trimmed == "1 min" || trimmed == "2 min" || trimmed == "5 min" || trimmed == "10 min")
        return trimmed;
    return "Off";
}

std::string normalizeFocusPeakingColorValue(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "green")
        return "Green";
    if (normalized == "blue")
        return "Blue";
    if (normalized == "yellow")
        return "Yellow";
    if (normalized == "pink")
        return "Pink";
    return "Red";
}

std::string normalizeAnamorphicRatioValue(const std::string &value)
{
    const std::string trimmed = trimAscii(value);
    if (trimmed == "1.5x" || trimmed == "1.8x" || trimmed == "2.0x")
        return trimmed;
    return "1.33x";
}

std::string normalizeGuidesTypeValue(const std::string &value)
{
    const std::string trimmed = trimAscii(value);
    if (trimmed == "16:9" || trimmed == "2.39:1" || trimmed == "4:3" || trimmed == "1:1" ||
        trimmed == "Academy 1.37:1" || trimmed == "5:4" || trimmed == "9:16" || trimmed == "14:9") {
        return trimmed;
    }
    return "Thirds";
}

std::string normalizeCenterMarkerTypeValue(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "crosshair")
        return "Crosshair";
    return "Circle/Dot";
}

std::string normalizeExternalMonitorModeValue(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "assist feed")
        return "Assist Feed";
    return "Clean Feed";
}

std::string normalizeExternalMonitorInfoOverlayValue(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "off")
        return "Off";
    return "On";
}

std::string normalizeFalseColorModeLabel(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "skin tone")
        return "Skin Tone";
    if (normalized == "highlight priority")
        return "Highlight Priority";
    if (normalized == "shadow priority")
        return "Shadow Priority";
    return "Exposure Based";
}

int falseColorModeValueFromLabel(const std::string &value)
{
    const std::string normalized = normalizeFalseColorModeLabel(value);
    if (normalized == "Skin Tone")
        return 1;
    if (normalized == "Highlight Priority")
        return 2;
    if (normalized == "Shadow Priority")
        return 3;
    return 0;
}

const std::vector<std::string> &favoriteCandidateOrder()
{
    static const std::vector<std::string> kOrder = {
        "externalMonitor",
        "lutPreview",
        "focusPeaking",
        "zebra",
        "falseColor",
        "guides",
        "centerMarker",
        "greyscale",
        "anamorphicDesqueeze",
        "stillMode",
        "format",
        "timecode",
        "audioOptions",
        "levels",
        "battery",
        "batteryInfo",
        "autoPowerOff",
        "cameraStandby",
        "uiOrientation",
        "dateTime",
        "media",
        "mediaSpeed",
        "thermal",
        "powerActions",
        "wifiToggle",
        "currentNetwork",
        "availableNetworks",
        "camera",
        "sbc",
        "systemInfo",
        "systemDrive",
    };
    return kOrder;
}

bool isFavoriteCandidateId(const std::string &value)
{
    const auto &candidates = favoriteCandidateOrder();
    return std::find(candidates.begin(), candidates.end(), value) != candidates.end();
}

std::vector<std::string> splitSimpleListValue(const std::string &value)
{
    const std::string trimmed = trimAscii(value);
    if (trimmed.empty())
        return {};

    const char delimiter = trimmed.find('|') != std::string::npos ? '|' : ',';
    std::vector<std::string> out;
    size_t start = 0;
    while (start <= trimmed.size()) {
        const size_t end = trimmed.find(delimiter, start);
        std::string entry = trimAscii(trimmed.substr(start, end == std::string::npos ? std::string::npos : (end - start)));
        if (!entry.empty() && entry.size() >= 2 && entry.front() == '"' && entry.back() == '"')
            entry = trimAscii(entry.substr(1, entry.size() - 2));
        if (!entry.empty())
            out.push_back(entry);
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return out;
}

std::vector<std::string> normalizeFavoritesOrderList(const std::vector<std::string> &values)
{
    std::vector<std::string> out;
    out.reserve(values.size());
    for (const std::string &value : values) {
        const std::string trimmed = trimAscii(value);
        if (!isFavoriteCandidateId(trimmed))
            continue;
        if (std::find(out.begin(), out.end(), trimmed) == out.end())
            out.push_back(trimmed);
    }
    return out;
}

std::vector<std::string> parseFavoritesOrderValue(const std::string &value)
{
    return normalizeFavoritesOrderList(splitSimpleListValue(value));
}

std::string encodeSimpleListValue(const std::vector<std::string> &values)
{
    std::ostringstream out;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0)
            out << ", ";
        out << values[i];
    }
    return out.str();
}

std::string normalizeRecordingFormatValue(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "mlv")
        return "mlv";
    if (normalized == "h264" || normalized == "h.264" || normalized == "mp4")
        return "h264";
    if (normalized == "lossydng" || normalized == "lossy_dng" ||
        normalized == "cdng 3:1" || normalized == "cdng 3to1" || normalized == "cdng3to1" ||
        normalized == "cinemadng 3:1" || normalized == "cinemadng 3to1" ||
        normalized == "cdng 5:1" || normalized == "cdng 5to1" || normalized == "cdng5to1")
        return "lossydng";
    if (normalized == "cinemadng 5:1" || normalized == "cinemadng 5to1")
        return "lossydng";
    if (normalized == "prores" || normalized == "prores 422" || normalized == "prores hq" ||
        normalized == "prores lt" || normalized == "prores proxy")
        return "prores";
    if (normalized == "cdng" || normalized == "cinemadng" || normalized == "cinema dng" || normalized == "dng")
        return "cdng";
    return {};
}

RecordingFormat recordingFormatFromNormalizedValue(const std::string &normalized)
{
    if (normalized == "mlv")
        return RecordingFormat::Mlv;
    if (normalized == "h264")
        return RecordingFormat::H264;
    if (normalized == "lossydng")
        return RecordingFormat::LossyDng;
    if (normalized == "prores")
        return RecordingFormat::Prores;
    return RecordingFormat::Cdng;
}

LossyDngRatio lossyDngRatioFromString(const std::string &value)
{
    const std::string normalized = toLowerAsciiCopy(trimAscii(value));
    if (normalized == "5" || normalized == "5:1" || normalized == "5to1" ||
        normalized == "r5" || normalized == "cdng 5:1" || normalized == "cdng 5to1")
        return LossyDngRatio::R5;
    return LossyDngRatio::R3;
}

uint32_t parseRawBitDepthValue(const std::string &value)
{
    std::string digits;
    for (const char ch : value) {
        if (std::isdigit(static_cast<unsigned char>(ch)))
            digits.push_back(ch);
    }
    if (digits == "8")
        return 8u;
    return digits == "10" ? 10u : 12u;
}

std::string formatRawBitDepthValue(uint32_t value)
{
    if (value == 8)
        return "8-bit";
    return value == 10 ? "10-bit" : "12-bit";
}

bool isAutoValue(const std::string &value)
{
    return toLowerAsciiCopy(trimAscii(value)) == "auto";
}

std::optional<double> parseDoubleValue(const std::string &value)
{
    const std::string trimmed = trimAscii(value);
    if (trimmed.empty())
        return std::nullopt;

    try {
        size_t parsed = 0;
        const double parsedValue = std::stod(trimmed, &parsed);
        if (parsed != trimmed.size())
            return std::nullopt;
        return parsedValue;
    } catch (const std::exception &) {
        return std::nullopt;
    }
}

std::optional<long long> parseIntegerValue(const std::string &value)
{
    const std::string trimmed = trimAscii(value);
    if (trimmed.empty())
        return std::nullopt;

    try {
        size_t parsed = 0;
        const long long parsedValue = std::stoll(trimmed, &parsed);
        if (parsed != trimmed.size())
            return std::nullopt;
        return parsedValue;
    } catch (const std::exception &) {
        return std::nullopt;
    }
}

std::string compactNumber(double value)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << value;
    std::string text = out.str();
    while (!text.empty() && text.back() == '0')
        text.pop_back();
    if (!text.empty() && text.back() == '.')
        text.pop_back();
    return text.empty() ? "0" : text;
}

double snapToNearestPreset(double value,
                           const double *presets,
                           size_t presetCount,
                           double tolerance)
{
    double closest = value;
    double smallestDelta = tolerance;
    for (size_t i = 0; i < presetCount; ++i) {
        const double delta = std::fabs(value - presets[i]);
        if (delta <= smallestDelta) {
            smallestDelta = delta;
            closest = presets[i];
        }
    }
    return closest;
}

double normalizedShutterAngleForPreset(double value)
{
    static const double kAnglePresets[] = {
        11.25, 15.0, 22.5, 30.0, 37.5, 45.0, 60.0, 72.0, 75.0, 90.0,
        108.0, 120.0, 144.0, 150.0, 172.8, 180.0, 216.0, 270.0, 324.0, 360.0
    };
    return snapToNearestPreset(value,
                               kAnglePresets,
                               sizeof(kAnglePresets) / sizeof(kAnglePresets[0]),
                               0.05);
}

double normalizedShutterSpeedForPreset(double value)
{
    static const double kSpeedPresets[] = {
        24.0, 30.0, 40.0, 48.0, 50.0, 60.0, 80.0, 100.0,
        125.0, 160.0, 200.0, 250.0, 320.0, 500.0, 1000.0
    };
    return snapToNearestPreset(value,
                               kSpeedPresets,
                               sizeof(kSpeedPresets) / sizeof(kSpeedPresets[0]),
                               0.05);
}

std::string formatPresetFps(double fps)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << fps;
    return out.str();
}

std::string formatPresetIso(uint32_t iso)
{
    return std::to_string(iso);
}

std::string formatPresetWhiteBalance(uint32_t kelvin, bool whiteBalanceAuto)
{
    if (whiteBalanceAuto)
        return "Auto";
    return std::to_string(kelvin) + "K";
}

std::string formatPresetShutterAngle(uint32_t shutterUs, double fps, bool shutterAuto)
{
    if (shutterAuto)
        return "Auto";

    const double safeFps = fps > 0.0 ? fps : 24.0;
    const double angle = (static_cast<double>(shutterUs) * safeFps * 360.0) / 1000000.0;
    return compactNumber(normalizedShutterAngleForPreset(angle)) + "\xC2\xB0";
}

std::string formatPresetShutterSpeed(uint32_t shutterUs, bool shutterAuto)
{
    if (shutterAuto)
        return "Auto";

    const double denominator = 1000000.0 / std::max(1u, shutterUs);
    if (shutterUs >= 1'000'000)
        return compactNumber(shutterUs / 1000000.0) + "s";
    return "1/" + compactNumber(normalizedShutterSpeedForPreset(denominator));
}

std::string formatPresetResolution(uint32_t width, uint32_t height)
{
    return std::to_string(width) + "x" + std::to_string(height);
}

enum class PcfSettingStepMode {
    Cycle,
    Up,
    Down
};

std::optional<PcfSettingStepMode> pcfSettingStepMode(const std::string &action,
                                                     const std::string &baseAction)
{
    if (action == baseAction)
        return PcfSettingStepMode::Cycle;
    if (action == baseAction + "_up")
        return PcfSettingStepMode::Up;
    if (action == baseAction + "_down")
        return PcfSettingStepMode::Down;
    return std::nullopt;
}

size_t nearestPcfPresetIndex(const std::vector<double> &values, double currentValue)
{
    if (values.empty())
        return 0;

    size_t bestIndex = 0;
    double bestDelta = std::numeric_limits<double>::max();
    for (size_t i = 0; i < values.size(); ++i) {
        const double delta = std::fabs(values[i] - currentValue);
        if (delta < bestDelta) {
            bestDelta = delta;
            bestIndex = i;
        }
    }
    return bestIndex;
}

size_t nextPcfPresetIndex(size_t currentIndex, size_t count, PcfSettingStepMode mode)
{
    if (count == 0)
        return 0;

    currentIndex = std::min(currentIndex, count - 1);
    switch (mode) {
    case PcfSettingStepMode::Cycle:
        return (currentIndex + 1) % count;
    case PcfSettingStepMode::Up:
        return std::min(currentIndex + 1, count - 1);
    case PcfSettingStepMode::Down:
        return currentIndex == 0 ? 0 : currentIndex - 1;
    }
    return currentIndex;
}

double maximumPcfFpsForSettings(const CaptureSettings &settings, const std::string &sensorName)
{
    const std::string sensor = lowerAscii(sensorName);
    const uint32_t width = settings.resolution.width;
    const uint32_t height = settings.resolution.height;

    if (sensor.find("imx577") != std::string::npos) {
        if (width == 1332 && height == 990)
            return 101.68;
        if ((width == 2028 && height == 1080) ||
            (width == 1928 && height == 1090) ||
            (width == 1920 && height == 1080))
            return 62.81;
        if (width == 2028 && height == 1520)
            return 45.19;
        if ((width == 4056 && height == 2160) ||
            (width == 3856 && height == 2180) ||
            (width == 3840 && height == 2160))
            return 16.39;
        if (width == 4056 && height == 3040)
            return 11.72;
    }

    if (sensor.find("imx477") != std::string::npos) {
        if (width == 1332 && height == 990)
            return 100.0;
        if (width == 2028 && height != 1520)
            return 60.0;
        if (width == 2028 && height == 1520)
            return 30.0;
        if ((width == 1928 && height == 1090) ||
            (width == 1920 && height == 1080))
            return 60.0;
        if (width == 4056 && height == 3040)
            return 24.0;
        if (width == 4056)
            return 30.0;
        if ((width == 3856 && height == 2180) ||
            (width == 3840 && height == 2160))
            return 30.0;
    }

    if (sensor.find("imx294") != std::string::npos) {
        if ((width == 3792 && height == 2824) ||
            (width == 3792 && height == 2528) ||
            (width == 3392 && height == 2824))
            return 34.70;
        if (width == 4176 && height == 2176)
            return 40.95;
        if ((width == 4096 && height == 2048) ||
            (width == 4096 && height == 1844) ||
            (width == 4096 && height == 1744) ||
            (width == 4096 && height == 1716) ||
            (width == 4096 && height == 1640) ||
            (width == 2048 && height == 1080) ||
            (width == 1920 && height == 1080))
            return 40.95;
        if ((width == 4144 && height == 2176) ||
            (width == 3872 && height == 2176))
            return width == 4144 ? 41.08 : 44.17;
    }

    if (sensor.find("imx585") != std::string::npos) {
        if ((width == 3856 && height == 2180) ||
            (width == 3840 && height == 2160))
            return 30.0;
        if ((width == 1928 && height == 1090) ||
            (width == 1920 && height == 1080))
            return 60.0;
    }

    return 60.0;
}

std::vector<double> availablePcfFpsOptions(const CaptureSettings &settings,
                                           const std::string &sensorName)
{
    const std::string sensor = lowerAscii(sensorName);
    const std::vector<double> presets =
        sensor.find("imx477") != std::string::npos || sensor.find("imx577") != std::string::npos
            ? std::vector<double>{24.0, 25.0, 30.0, 50.0, 60.0, 100.0}
            : std::vector<double>{24.0, 25.0, 30.0, 50.0, 60.0};

    const double maxFps = maximumPcfFpsForSettings(settings, sensorName);
    std::vector<double> filtered;
    for (double fps : presets) {
        if (fps <= maxFps + 0.0005)
            filtered.push_back(fps);
    }
    if (filtered.empty())
        filtered.push_back(maxFps);
    return filtered;
}

bool sameResolution(const Resolution &left, const Resolution &right)
{
    return left.width == right.width && left.height == right.height;
}

uint64_t resolutionDistance(const Resolution &left, const Resolution &right)
{
    const int64_t widthDelta = static_cast<int64_t>(left.width) - static_cast<int64_t>(right.width);
    const int64_t heightDelta = static_cast<int64_t>(left.height) - static_cast<int64_t>(right.height);
    return static_cast<uint64_t>(widthDelta * widthDelta + heightDelta * heightDelta);
}

std::vector<Resolution> availablePcfResolutionOptions(const std::string &sensorName)
{
    const std::string sensor = lowerAscii(sensorName);
    if (sensor.find("imx577") != std::string::npos) {
        return {
            { 1332, 990 },
            { 2028, 1080 },
            { 2028, 1520 },
            { 4056, 2160 },
            { 4056, 3040 },
        };
    }

    if (sensor.find("imx477") != std::string::npos) {
        return {
            { 1332, 990 },
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
        };
    }

    if (sensor.find("imx294") != std::string::npos) {
        return {
            { 4144, 2176 },
            { 4096, 2048 },
            { 4096, 1844 },
            { 4096, 1744 },
            { 4096, 1716 },
            { 4096, 1640 },
            { 2048, 1080 },
            { 1920, 1080 },
            { 4176, 2176 },
            { 3872, 2176 },
            { 3792, 2824 },
            { 3792, 2528 },
            { 3392, 2824 },
        };
    }

    if (sensor.find("imx585") != std::string::npos) {
        return {
            { 1920, 1080 },
            { 3840, 2160 },
        };
    }

    return {
        { 1920, 1080 },
        { 3840, 2160 },
    };
}

size_t nearestPcfResolutionIndex(const std::vector<Resolution> &values,
                                 const Resolution &currentValue)
{
    if (values.empty())
        return 0;

    for (size_t i = 0; i < values.size(); ++i) {
        if (sameResolution(values[i], currentValue))
            return i;
    }

    size_t bestIndex = 0;
    uint64_t bestDelta = std::numeric_limits<uint64_t>::max();
    for (size_t i = 0; i < values.size(); ++i) {
        const uint64_t delta = resolutionDistance(values[i], currentValue);
        if (delta < bestDelta) {
            bestDelta = delta;
            bestIndex = i;
        }
    }
    return bestIndex;
}

std::vector<double> pcfIsoOptions()
{
    return {100.0, 200.0, 400.0, 800.0, 1600.0, 3200.0, 6400.0, 12800.0, 25600.0, 51200.0};
}

std::vector<double> pcfShutterAngleOptions()
{
    return {
        11.25, 15.0, 22.5, 30.0, 37.5, 45.0, 60.0, 72.0, 75.0, 90.0,
        108.0, 120.0, 144.0, 150.0, 172.8, 180.0, 216.0, 270.0, 324.0, 360.0
    };
}

std::vector<double> pcfShutterSpeedDenominatorOptions(bool longExposure = false, uint32_t maximumUs = 0)
{
    std::vector<double> options;
    if (longExposure) {
        options = {
            1.0 / 30.0, 1.0 / 25.0, 1.0 / 20.0, 1.0 / 15.0,
            1.0 / 10.0, 1.0 / 8.0, 1.0 / 6.0, 1.0 / 5.0,
            1.0 / 4.0, 1.0 / 3.0, 1.0 / 2.0, 1.0,
            2.0, 4.0, 8.0, 12.0, 15.0
        };
    }
    options.insert(options.end(), {
        24.0, 30.0, 40.0, 48.0, 50.0, 60.0, 80.0, 100.0,
        125.0, 160.0, 200.0, 250.0, 320.0, 500.0, 1000.0
    });
    if (maximumUs)
        std::erase_if(options, [maximumUs](double denominator) {
            return std::llround(1'000'000.0 / denominator) > maximumUs;
        });
    return options;
}

std::vector<double> pcfWhiteBalanceOptions()
{
    return {
        2500.0, 2800.0, 3000.0, 3200.0, 3400.0, 3600.0, 4000.0, 4500.0,
        4800.0, 5000.0, 5200.0, 5400.0, 5600.0, 5800.0, 6000.0, 6500.0,
        7000.0, 7500.0, 8000.0
    };
}

bool parseResolutionText(const std::string &value, uint32_t *width, uint32_t *height)
{
    const std::string trimmed = trimAscii(value);
    const size_t xPos = trimmed.find('x');
    if (xPos == std::string::npos)
        return false;

    const auto parsedWidth = parseIntegerValue(trimmed.substr(0, xPos));
    const auto parsedHeight = parseIntegerValue(trimmed.substr(xPos + 1));
    if (!parsedWidth || !parsedHeight || *parsedWidth <= 0 || *parsedHeight <= 0)
        return false;

    if (width)
        *width = static_cast<uint32_t>(*parsedWidth);
    if (height)
        *height = static_cast<uint32_t>(*parsedHeight);
    return true;
}

std::optional<std::string> hardwareActionSuffix(const std::string &action,
                                                const std::vector<std::string> &prefixes)
{
    for (const std::string &prefix : prefixes) {
        if (action.rfind(prefix, 0) != 0)
            continue;

        std::string suffix = action.substr(prefix.size());
        while (!suffix.empty() && suffix.front() == '_')
            suffix.erase(suffix.begin());
        if (!suffix.empty())
            return suffix;
    }
    return std::nullopt;
}

std::optional<double> parseHardwareActionNumber(const std::string &action,
                                                const std::vector<std::string> &prefixes)
{
    auto suffix = hardwareActionSuffix(action, prefixes);
    if (!suffix)
        return std::nullopt;

    std::string number = *suffix;
    if (!number.empty() && (number.back() == 'k' || number.back() == 'p'))
        number.pop_back();
    std::replace(number.begin(), number.end(), '_', '.');
    return parseDoubleValue(number);
}

std::optional<Resolution> parseHardwareActionResolution(const std::string &action)
{
    auto suffix = hardwareActionSuffix(action, { "resolution", "res" });
    if (!suffix)
        return std::nullopt;

    std::string value = *suffix;
    std::replace(value.begin(), value.end(), '*', 'x');
    if (value.find('x') == std::string::npos) {
        if (const size_t underscore = value.find('_'); underscore != std::string::npos)
            value[underscore] = 'x';
    }

    uint32_t width = 0;
    uint32_t height = 0;
    if (!parseResolutionText(value, &width, &height))
        return std::nullopt;
    return Resolution{width, height};
}

std::optional<double> parseHardwareActionShutterSpeedDenominator(const std::string &action,
                                                                 const std::vector<std::string> &prefixes)
{
    auto suffix = hardwareActionSuffix(action, prefixes);
    if (!suffix)
        return std::nullopt;

    std::string value = *suffix;
    if (value.rfind("1/", 0) == 0)
        value = value.substr(2);
    else if (value.rfind("1_", 0) == 0)
        value = value.substr(2);
    std::replace(value.begin(), value.end(), '_', '.');
    return parseDoubleValue(value);
}

std::optional<uint32_t> parseHardwareActionShutterSeconds(const std::string &action)
{
    if (action.empty() || action.back() != 's')
        return std::nullopt;
    auto suffix = hardwareActionSuffix(action, {
        "shutter_seconds", "shutterseconds", "shutter_speed", "shutterspeed", "shutter_s", "shutter"
    });
    if (!suffix)
        return std::nullopt;
    suffix->pop_back();
    // Hardware action normalization turns decimal points into underscores.
    std::replace(suffix->begin(), suffix->end(), '_', '.');
    const auto seconds = parseDoubleValue(*suffix);
    if (!seconds || !std::isfinite(*seconds) || *seconds <= 0.0 || *seconds > 30.0)
        return std::nullopt;
    return static_cast<uint32_t>(std::max(1.0, std::round(*seconds * 1000000.0)));
}

std::string currentUtcIsoTimestamp()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
    std::tm utcTm{};
#if defined(_WIN32)
    gmtime_s(&utcTm, &nowTime);
#else
    gmtime_r(&nowTime, &utcTm);
#endif
    char buffer[32]{};
    if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &utcTm) == 0)
        return {};
    return buffer;
}

std::filesystem::path allocateSoundRecordingFolder(const std::string &mediaRoot)
{
    const std::filesystem::path audioRoot = std::filesystem::path(mediaRoot) / "Audio";
    std::error_code ec;
    std::filesystem::create_directories(audioRoot, ec);
    if (ec)
        throw std::runtime_error("Could not create audio folder: " + ec.message());

    const std::time_t now = std::time(nullptr);
    std::tm localTime{};
#if defined(_WIN32)
    localtime_s(&localTime, &now);
#else
    localtime_r(&now, &localTime);
#endif

    std::ostringstream prefix;
    prefix << "Audio_" << std::put_time(&localTime, "%y-%m-%d");

    for (uint32_t index = 1; index <= 9999; ++index) {
        std::ostringstream folderName;
        folderName << prefix.str() << "_C"
                   << std::setw(4) << std::setfill('0') << index;

        const std::filesystem::path folder = audioRoot / folderName.str();
        if (std::filesystem::exists(folder, ec)) {
            ec.clear();
            continue;
        }

        std::filesystem::create_directories(folder, ec);
        if (!ec)
            return folder;

        throw std::runtime_error("Could not create audio recording folder: " + ec.message());
    }

    throw std::runtime_error("Could not allocate a unique audio recording folder.");
}

std::string shellQuote(const std::string &value)
{
    std::string quoted = "'";
    for (const char ch : value) {
        if (ch == '\'')
            quoted += "'\"'\"'";
        else
            quoted.push_back(ch);
    }
    quoted.push_back('\'');
    return quoted;
}

struct ShellCommandResult
{
    int exitCode = -1;
    std::string output;
};

ShellCommandResult runShellCommandCapture(const std::string &command)
{
    ShellCommandResult result;
    const std::string fullCommand = command + " 2>&1";
    FILE *pipe = ::popen(fullCommand.c_str(), "r");
    if (!pipe) {
        result.output = "Could not start shell command.";
        return result;
    }

    std::array<char, 4096> buffer{};
    while (true) {
        const size_t bytesRead = std::fread(buffer.data(), 1, buffer.size(), pipe);
        if (bytesRead > 0)
            result.output.append(buffer.data(), bytesRead);
        if (bytesRead < buffer.size()) {
            if (std::feof(pipe))
                break;
            if (std::ferror(pipe))
                break;
        }
    }

    const int status = ::pclose(pipe);
    if (WIFEXITED(status))
        result.exitCode = WEXITSTATUS(status);
    result.output = trimAscii(result.output);
    return result;
}

bool runShellCommand(const std::string &command, std::string *errorMessage)
{
    const ShellCommandResult result = runShellCommandCapture(command);
    if (result.exitCode == 0)
        return true;
    if (errorMessage)
        *errorMessage = result.output.empty() ? "Command failed." : result.output;
    return false;
}

ShellCommandResult standbyUsbPower(const char *action)
{
    // Fixed executable and arguments only; never accept paths from WebUI/config.
    return runShellCommandCapture(std::string("/usr/bin/timeout 6s /usr/bin/sudo -n /usr/local/libexec/apertar-standby-usb ") + action);
}

ShellCommandResult standbyLowPower(const char *action)
{
    return runShellCommandCapture(std::string("/usr/bin/timeout 8s /usr/bin/sudo -n /usr/local/libexec/apertar-standby-low-power ") + action);
}

std::string readTextFileRaw(const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open())
        return {};
    return std::string((std::istreambuf_iterator<char>(input)),
                       std::istreambuf_iterator<char>());
}

bool writeTextFileRaw(const std::filesystem::path &path, const std::string &text)
{
    std::ofstream output(path, std::ios::trunc | std::ios::binary);
    if (!output.is_open())
        return false;
    output << text;
    output.flush();
    return output.good();
}

std::vector<std::string> splitWhitespace(const std::string &value)
{
    std::istringstream input(value);
    std::vector<std::string> parts;
    std::string part;
    while (input >> part)
        parts.push_back(part);
    return parts;
}

struct SharedUiSettingsSnapshot
{
    bool focusPeakingEnabled = false;
    double focusPeakingThreshold = 0.04;
    std::string focusPeakingColor = "Red";
    bool zebraEnabled = false;
    double zebraThreshold = 0.70;
    bool anamorphicDesqueezeEnabled = false;
    std::string anamorphicRatio = "1.33x";
    bool falseColorEnabled = false;
    int falseColorMode = 0;
    bool lutPreviewEnabled = false;
    std::string lutPreviewFile;
    bool guidesEnabled = true;
    std::string guidesType = "Thirds";
    int guidesThickness = 1;
    bool centerMarkerEnabled = true;
    std::string centerMarkerType = "Circle/Dot";
    bool grayscaleEnabled = false;
    bool smpteEnabled = false;
    bool externalMonitorEnabled = false;
    std::string externalMonitorMode = "Clean Feed";
    std::string externalMonitorInfoOverlay = "On";
    std::string externalMonitorOrientation = "Landscape";
    std::string timecodeMode = "Free Run";
    bool photoModeEnabled = false;
    bool photoLongExposureEnabled = false;
    bool photoBracketingEnabled = false;
    double photoBracketingEv = 1.0;
    std::string photoTimer = "Off";
    std::string photoBurst = "Single";
    std::string photoFormat = "DNG";
    std::string photoTimelapse = "Off";
    std::string uiOrientation = "Landscape";
    double cameraControlsOpacity = 1.0;
    std::string cameraControlsMode = "Light";
    bool dateTimeOverlayEnabled = false;
    bool recordAudioEnabled = false;
    bool soundRecorderEnabled = false;
    bool recordGyroEnabled = false;
    bool liveAudioMonitoringEnabled = true;
    bool audioMeterEnabled = false;
    std::string audioInputDevice;
    std::string audioOutputDevice;
    std::string audioSampleFormat = "auto";
    int audioSampleRate = 0;
    std::string audioChannels = "auto";
    int inputVolume = 60;
    int headphoneVolume = 55;
    std::string batteryType = "V-Mount";
    std::string batteryCapacity = "150Wh";
    int customBatteryWh = 150;
    double customBatteryFullVoltage = 16.8;
    double customBatteryEmptyVoltage = 13.2;
    std::string sleepMode = "Off";
    int standbyTimeoutSeconds = 0;
    bool standbyUsbPowerOff = false;
    std::vector<std::string> favoritesOrder;
};

SharedUiSettingsSnapshot readSharedUiSettings()
{
    SharedUiSettingsSnapshot snapshot;
    std::ifstream input(sharedUiSettingsPath());
    if (!input.is_open())
        return snapshot;

    bool inGeneral = false;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        const std::string trimmed = trimAscii(line);
        if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';')
            continue;

        if (trimmed.front() == '[' && trimmed.back() == ']') {
            inGeneral = trimAscii(trimmed.substr(1, trimmed.size() - 2)) == "General";
            continue;
        }

        if (!inGeneral)
            continue;

        const size_t eq = line.find('=');
        if (eq == std::string::npos)
            continue;

        const std::string key = trimAscii(line.substr(0, eq));
        const std::string value = trimAscii(line.substr(eq + 1));
        if (key == "focusPeakingEnabled")
            snapshot.focusPeakingEnabled = parseUiSettingsBool(value, snapshot.focusPeakingEnabled);
        else if (key == "focusPeakingThreshold") {
            if (const auto parsed = parseDoubleValue(value))
                snapshot.focusPeakingThreshold = std::clamp(*parsed, 0.0, 1.0);
        } else if (key == "focusPeakingColor")
            snapshot.focusPeakingColor = normalizeFocusPeakingColorValue(value);
        else if (key == "zebraEnabled")
            snapshot.zebraEnabled = parseUiSettingsBool(value, snapshot.zebraEnabled);
        else if (key == "zebraThreshold") {
            if (const auto parsed = parseDoubleValue(value))
                snapshot.zebraThreshold = std::clamp(*parsed, 0.0, 1.0);
        } else if (key == "anamorphicDesqueezeEnabled")
            snapshot.anamorphicDesqueezeEnabled = parseUiSettingsBool(value, snapshot.anamorphicDesqueezeEnabled);
        else if (key == "anamorphicRatio")
            snapshot.anamorphicRatio = normalizeAnamorphicRatioValue(value);
        else if (key == "falseColorEnabled")
            snapshot.falseColorEnabled = parseUiSettingsBool(value, snapshot.falseColorEnabled);
        else if (key == "falseColorMode") {
            if (const auto parsed = parseIntegerValue(value))
                snapshot.falseColorMode = static_cast<int>(std::clamp<long long>(*parsed, 0, 3));
        }
        else if (key == "lutPreviewEnabled")
            snapshot.lutPreviewEnabled = parseUiSettingsBool(value, snapshot.lutPreviewEnabled);
        else if (key == "lutPreviewFile")
            snapshot.lutPreviewFile = value;
        else if (key == "guidesEnabled")
            snapshot.guidesEnabled = parseUiSettingsBool(value, snapshot.guidesEnabled);
        else if (key == "guidesType")
            snapshot.guidesType = normalizeGuidesTypeValue(value);
        else if (key == "guidesThickness") {
            if (const auto parsed = parseIntegerValue(value))
                snapshot.guidesThickness = static_cast<int>(std::clamp<long long>(*parsed, 1, 4));
        }
        else if (key == "centerMarkerEnabled")
            snapshot.centerMarkerEnabled = parseUiSettingsBool(value, snapshot.centerMarkerEnabled);
        else if (key == "centerMarkerType")
            snapshot.centerMarkerType = normalizeCenterMarkerTypeValue(value);
        else if (key == "grayscaleEnabled")
            snapshot.grayscaleEnabled = parseUiSettingsBool(value, snapshot.grayscaleEnabled);
        else if (key == "smpteEnabled")
            snapshot.smpteEnabled = parseUiSettingsBool(value, snapshot.smpteEnabled);
        else if (key == "externalMonitorEnabled")
            snapshot.externalMonitorEnabled = parseUiSettingsBool(value, snapshot.externalMonitorEnabled);
        else if (key == "externalMonitorMode")
            snapshot.externalMonitorMode = normalizeExternalMonitorModeValue(value);
        else if (key == "externalMonitorInfoOverlay")
            snapshot.externalMonitorInfoOverlay = normalizeExternalMonitorInfoOverlayValue(value);
        else if (key == "externalMonitorOrientation")
            snapshot.externalMonitorOrientation = normalizeExternalMonitorOrientationValue(value);
        else if (key == "timecodeMode")
            snapshot.timecodeMode = value == "Rec Run" ? "Rec Run" : "Free Run";
        else if (key == "photoModeEnabled")
            snapshot.photoModeEnabled = parseUiSettingsBool(value, snapshot.photoModeEnabled);
        else if (key == "photoLongExposureEnabled")
            snapshot.photoLongExposureEnabled = parseUiSettingsBool(value, false);
        else if (key == "photoBracketingEnabled")
            snapshot.photoBracketingEnabled = parseUiSettingsBool(value, false);
        else if (key == "photoBracketingEv") {
            if (const auto parsed = parseDoubleValue(value); parsed && std::isfinite(*parsed))
                snapshot.photoBracketingEv = std::round(std::clamp(*parsed, 1.0 / 3.0, 3.0) * 3.0) / 3.0;
        }
        else if (key == "photoTimer")
            snapshot.photoTimer = normalizePhotoTimerValue(value);
        else if (key == "photoBurst")
            snapshot.photoBurst = normalizePhotoBurstValue(value);
        else if (key == "photoFormat")
            snapshot.photoFormat = normalizePhotoFormatValue(value);
        else if (key == "photoTimelapse")
            snapshot.photoTimelapse = normalizePhotoTimelapseValue(value);
        else if (key == "uiOrientation")
            snapshot.uiOrientation = normalizeUiOrientationValue(value);
        else if (key == "cameraControlsOpacity") {
            if (const auto parsed = parseDoubleValue(value))
                snapshot.cameraControlsOpacity = std::clamp(*parsed, 0.15, 1.0);
        }
        else if (key == "cameraControlsMode")
            snapshot.cameraControlsMode = normalizeCameraControlsModeValue(value);
        else if (key == "dateTimeOverlayEnabled")
            snapshot.dateTimeOverlayEnabled = parseUiSettingsBool(value, snapshot.dateTimeOverlayEnabled);
        else if (key == "recordAudioEnabled")
            snapshot.recordAudioEnabled = parseUiSettingsBool(value, snapshot.recordAudioEnabled);
        else if (key == "soundRecorderEnabled")
            snapshot.soundRecorderEnabled = parseUiSettingsBool(value, snapshot.soundRecorderEnabled);
        else if (key == "recordGyroEnabled")
            snapshot.recordGyroEnabled = parseUiSettingsBool(value, snapshot.recordGyroEnabled);
        else if (key == "liveAudioMonitoringEnabled")
            snapshot.liveAudioMonitoringEnabled = parseUiSettingsBool(value, snapshot.liveAudioMonitoringEnabled);
        else if (key == "audioMeterEnabled")
            snapshot.audioMeterEnabled = parseUiSettingsBool(value, snapshot.audioMeterEnabled);
        else if (key == "audioInputDevice")
            snapshot.audioInputDevice = value;
        else if (key == "audioOutputDevice")
            snapshot.audioOutputDevice = value;
        else if (key == "audioSampleFormat")
            snapshot.audioSampleFormat = validAudioFormat(value) ? value : "auto";
        else if (key == "audioChannels")
            snapshot.audioChannels = value;
        else if (key == "audioSampleRate") {
            if (const auto parsed = parseIntegerValue(value))
                snapshot.audioSampleRate = int(std::clamp<long long>(*parsed, 0, 192000));
        }
        else if (key == "inputVolume") {
            if (const auto parsed = parseIntegerValue(value))
                snapshot.inputVolume = static_cast<int>(std::clamp<long long>(*parsed, 0, 100));
        } else if (key == "headphoneVolume") {
            if (const auto parsed = parseIntegerValue(value))
                snapshot.headphoneVolume = static_cast<int>(std::clamp<long long>(*parsed, 0, 100));
        } else if (key == "batteryType") {
            snapshot.batteryType = normalizeBatteryTypeValue(value);
        } else if (key == "batteryCapacity") {
            snapshot.batteryCapacity = normalizedBatteryCapacityForType(snapshot.batteryType, value);
        } else if (key == "customBatteryWh") {
            if (const auto parsed = parseIntegerValue(value))
                snapshot.customBatteryWh = static_cast<int>(std::clamp<long long>(*parsed, 10, 500));
        } else if (key == "customBatteryFullVoltage") {
            if (const auto parsed = parseDoubleValue(value)) {
                snapshot.customBatteryFullVoltage =
                    clampBatteryVoltageValue(*parsed, batteryDefaultFullVoltage(snapshot.batteryType));
                if (snapshot.customBatteryFullVoltage <= snapshot.customBatteryEmptyVoltage)
                    snapshot.customBatteryEmptyVoltage = std::max(0.5, snapshot.customBatteryFullVoltage - 0.1);
            }
        } else if (key == "customBatteryEmptyVoltage") {
            if (const auto parsed = parseDoubleValue(value)) {
                snapshot.customBatteryEmptyVoltage =
                    std::min(clampBatteryVoltageValue(*parsed, batteryDefaultEmptyVoltage(snapshot.batteryType)),
                             snapshot.customBatteryFullVoltage - 0.1);
            }
        } else if (key == "sleepMode") {
            snapshot.sleepMode = normalizeSleepModeValue(value);
        } else if (key == "standbyTimeoutSeconds") {
            if (const auto parsed = parseDoubleValue(value))
                snapshot.standbyTimeoutSeconds = static_cast<int>(std::clamp(*parsed, 0.0, 86400.0));
        } else if (key == "standbyUsbPowerOff") {
            snapshot.standbyUsbPowerOff = parseUiSettingsBool(value, false);
        } else if (key == "favoritesOrder") {
            snapshot.favoritesOrder = parseFavoritesOrderValue(value);
        }
    }

    return snapshot;
}

std::map<std::string, std::string> readSharedUiGeneralSettingsMap()
{
    std::map<std::string, std::string> values;
    std::ifstream input(sharedUiSettingsPath());
    if (!input.is_open())
        return values;

    bool inGeneral = false;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        const std::string trimmed = trimAscii(line);
        if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';')
            continue;

        if (trimmed.front() == '[' && trimmed.back() == ']') {
            inGeneral = trimAscii(trimmed.substr(1, trimmed.size() - 2)) == "General";
            continue;
        }

        if (!inGeneral)
            continue;

        const size_t eq = line.find('=');
        if (eq == std::string::npos)
            continue;

        values[trimAscii(line.substr(0, eq))] = trimAscii(line.substr(eq + 1));
    }

    return values;
}

bool updateSharedUiGeneralSettings(const std::vector<std::pair<std::string, std::string>> &updates)
{
    if (updates.empty())
        return true;

    const std::filesystem::path path = sharedUiSettingsPath();
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    if (ec)
        return false;

    settingsFile::QtIniLock settingsLock(path);
    if (!settingsLock) {
        std::cerr << "ApertarCore could not lock shared settings: " << path << '\n';
        return false;
    }
    std::vector<std::string> lines;
    {
        std::ifstream input(path);
        std::string line;
        while (std::getline(input, line)) {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            lines.push_back(line);
        }
    }

    size_t generalHeader = lines.size();
    size_t generalBegin = lines.size();
    size_t generalEnd = lines.size();
    bool inGeneral = false;
    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string trimmed = trimAscii(lines[i]);
        if (trimmed.size() >= 2 && trimmed.front() == '[' && trimmed.back() == ']') {
            const bool isGeneral = trimAscii(trimmed.substr(1, trimmed.size() - 2)) == "General";
            if (inGeneral && generalEnd == lines.size())
                generalEnd = i;
            inGeneral = isGeneral;
            if (isGeneral) {
                generalHeader = i;
                generalBegin = i + 1;
            }
        }
    }
    if (inGeneral && generalEnd == lines.size())
        generalEnd = lines.size();

    if (generalHeader == lines.size()) {
        if (!lines.empty() && !lines.back().empty())
            lines.push_back(std::string());
        generalHeader = lines.size();
        lines.push_back("[General]");
        generalBegin = generalHeader + 1;
        generalEnd = generalBegin;
    }

    for (const auto &[key, value] : updates) {
        bool replaced = false;
        for (size_t i = generalBegin; i < generalEnd; ++i) {
            const std::string trimmed = trimAscii(lines[i]);
            if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';')
                continue;
            if (trimmed.front() == '[' && trimmed.back() == ']')
                break;

            const size_t eq = lines[i].find('=');
            if (eq == std::string::npos)
                continue;

            if (trimAscii(lines[i].substr(0, eq)) == key) {
                lines[i] = key + "=" + value;
                replaced = true;
                break;
            }
        }

        if (!replaced) {
            lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(generalEnd), key + "=" + value);
            ++generalEnd;
        }
    }

    std::ostringstream output;
    for (const auto &line : lines) output << line << '\n';
    return settingsFile::atomicWrite(path, output.str());
}

std::string normalizeMountPath(std::string path)
{
    while (path.size() > 1 && path.back() == '/')
        path.pop_back();
    return path;
}

std::string decodeProcMountField(const std::string &value)
{
    std::string decoded;
    decoded.reserve(value.size());
    for (size_t i = 0; i < value.size(); ++i) {
        if (value[i] == '\\' && i + 3 < value.size() &&
            std::isdigit(static_cast<unsigned char>(value[i + 1])) &&
            std::isdigit(static_cast<unsigned char>(value[i + 2])) &&
            std::isdigit(static_cast<unsigned char>(value[i + 3]))) {
            const int code =
                ((value[i + 1] - '0') * 64) +
                ((value[i + 2] - '0') * 8) +
                (value[i + 3] - '0');
            decoded.push_back(static_cast<char>(code));
            i += 3;
            continue;
        }
        decoded.push_back(value[i]);
    }
    return decoded;
}

std::string normalizedVolumeLabel(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });

    size_t start = 0;
    while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])))
        ++start;

    size_t end = value.size();
    while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])))
        --end;

    return value.substr(start, end - start);
}

std::string mediaTypeLabelForVolumeLabel(const std::string &volumeLabel)
{
    const std::string normalized = normalizedVolumeLabel(volumeLabel);
    if (normalized == "CFE-RAW")
        return "CFExpress Type B";
    if (normalized == "SSD-RAW" || normalized == "NVME-RAW")
        return "NVMe M.2 SSD";
    if (normalized == "USB-RAW")
        return "External USB";
    if (normalized == "RAW")
        return "CFExpress Type B";
    return "CFExpress Type B";
}

std::string mediaTypeLabelForDeviceAndVolumeLabel(const std::string &devicePath,
                                                  const std::string &volumeLabel)
{
    const std::string normalized = normalizedVolumeLabel(volumeLabel);
    if (normalized == "CFE-RAW" || normalized == "RAW")
        return "CFExpress Type B";
    if (normalized == "SSD-RAW" || normalized == "NVME-RAW")
        return "NVMe M.2 SSD";
    if (normalized == "USB-RAW")
        return "External USB";
    if (devicePath.rfind("/dev/sd", 0) == 0)
        return "External USB";
    if (devicePath.rfind("/dev/nvme", 0) == 0)
        return "NVMe M.2 SSD";
    return mediaTypeLabelForVolumeLabel(volumeLabel);
}

bool isMountedPath(const std::string &mountPath)
{
    const std::string wanted = normalizeMountPath(mountPath);
    std::ifstream input("/proc/mounts");
    std::string device;
    std::string mountedAt;
    std::string type;
    std::string options;
    while (input >> device >> mountedAt >> type >> options) {
        std::string ignored;
        std::getline(input, ignored);
        if (normalizeMountPath(decodeProcMountField(mountedAt)) == wanted)
            return true;
    }
    return false;
}

std::string mountedDeviceForPath(const std::string &mountPath)
{
    const std::string wanted = normalizeMountPath(mountPath);
    std::ifstream input("/proc/mounts");
    std::string device;
    std::string mountedAt;
    std::string type;
    std::string options;
    while (input >> device >> mountedAt >> type >> options) {
        std::string ignored;
        std::getline(input, ignored);
        if (normalizeMountPath(decodeProcMountField(mountedAt)) == wanted)
            return decodeProcMountField(device);
    }
    return {};
}

std::string volumeLabelForDevicePath(const std::string &devicePath)
{
    if (devicePath.empty() || devicePath.rfind("/dev/", 0) != 0)
        return {};

    const ShellCommandResult lsblkLabel =
        runShellCommandCapture("lsblk -no LABEL " + shellQuote(devicePath));
    if (lsblkLabel.exitCode == 0) {
        const std::string label = trimAscii(lsblkLabel.output);
        if (!label.empty())
            return label;
    }

    std::error_code ec;
    const std::filesystem::path canonicalDevice = std::filesystem::weakly_canonical(devicePath, ec);
    if (ec)
        return {};

    const std::filesystem::path labelsDir("/dev/disk/by-label");
    if (!std::filesystem::exists(labelsDir, ec) || !std::filesystem::is_directory(labelsDir, ec))
        return {};

    for (const auto &entry : std::filesystem::directory_iterator(labelsDir, ec)) {
        if (ec)
            break;
        const std::filesystem::path resolved = std::filesystem::weakly_canonical(entry.path(), ec);
        if (ec) {
            ec.clear();
            continue;
        }
        if (resolved == canonicalDevice)
            return entry.path().filename().string();
    }

    return {};
}

std::string normalizedFormatFilesystemType(const std::string &value)
{
    std::string normalized = lowerAscii(trimAscii(value));
    normalized.erase(std::remove_if(normalized.begin(), normalized.end(), [](unsigned char ch) {
        return std::isspace(ch) || ch == '-';
    }), normalized.end());

    if (normalized == "exfat")
        return "exfat";
    if (normalized == "fat32" || normalized == "vfat")
        return "vfat";
    if (normalized == "ntfs")
        return "ntfs";
    if (normalized == "ext4")
        return "ext4";

    return {};
}

std::string formatFilesystemDisplayName(const std::string &filesystemType)
{
    const std::string normalized = normalizedFormatFilesystemType(filesystemType);
    if (normalized == "exfat")
        return "exFAT";
    if (normalized == "vfat")
        return "FAT32";
    if (normalized == "ntfs")
        return "NTFS";
    if (normalized == "ext4")
        return "ext4";
    return "selected filesystem";
}

std::string formatFilesystemToolName(const std::string &filesystemType)
{
    const std::string normalized = normalizedFormatFilesystemType(filesystemType);
    if (normalized == "exfat")
        return "mkfs.exfat";
    if (normalized == "vfat")
        return "mkfs.vfat";
    if (normalized == "ntfs")
        return "mkfs.ntfs";
    if (normalized == "ext4")
        return "mkfs.ext4";
    return "mkfs";
}

std::string formatFilesystemToolPath(const std::string &filesystemType)
{
    const std::string normalized = normalizedFormatFilesystemType(filesystemType);
    if (normalized == "exfat") {
        return existingExecutablePath({
            "/usr/sbin/mkfs.exfat",
            "/sbin/mkfs.exfat",
            "/usr/bin/mkfs.exfat",
            "/bin/mkfs.exfat",
        });
    }
    if (normalized == "vfat") {
        return existingExecutablePath({
            "/usr/sbin/mkfs.vfat",
            "/sbin/mkfs.vfat",
            "/usr/sbin/mkdosfs",
            "/sbin/mkdosfs",
        });
    }
    if (normalized == "ntfs") {
        return existingExecutablePath({
            "/usr/sbin/mkfs.ntfs",
            "/sbin/mkfs.ntfs",
            "/usr/sbin/mkntfs",
            "/sbin/mkntfs",
        });
    }
    return {};
}

std::string volumeLabelForFormat(std::string label, const std::string &filesystemType)
{
    label = trimAscii(label);
    if (label.empty())
        label = "RAW";

    const std::string normalized = normalizedFormatFilesystemType(filesystemType);
    if (normalized == "vfat") {
        std::string sanitized;
        for (unsigned char ch : label) {
            if (std::isalnum(ch) || ch == ' ' || ch == '-' || ch == '_')
                sanitized.push_back(static_cast<char>(std::toupper(ch)));
        }
        sanitized = trimAscii(sanitized);
        if (sanitized.size() > 11)
            sanitized.resize(11);
        return sanitized.empty() ? "RAW" : sanitized;
    }

    const size_t maxLength = normalized == "exfat" ? 15
        : (normalized == "ext4" ? 16
           : (normalized == "ntfs" ? 128 : label.size()));
    if (label.size() > maxLength)
        label.resize(maxLength);
    return label.empty() ? "RAW" : label;
}

std::string decodeLsblkValue(const std::string &value)
{
    std::string out;
    out.reserve(value.size());
    for (size_t i = 0; i < value.size(); ++i) {
        if (value[i] == '\\' && i + 3 < value.size() && value[i + 1] == 'x') {
            const std::string hex = value.substr(i + 2, 2);
            char *end = nullptr;
            const long parsed = std::strtol(hex.c_str(), &end, 16);
            if (end && *end == '\0') {
                out.push_back(static_cast<char>(parsed));
                i += 3;
                continue;
            }
        }
        if (value[i] == '\\' && i + 1 < value.size()) {
            out.push_back(value[i + 1]);
            ++i;
            continue;
        }
        out.push_back(value[i]);
    }
    return out;
}

std::map<std::string, std::string> parseLsblkPairs(const std::string &line)
{
    std::map<std::string, std::string> fields;
    static const std::regex pairPattern(R"LSBLK(([A-Z0-9_]+)="((?:\\.|[^"])*)")LSBLK");
    std::sregex_iterator it(line.begin(), line.end(), pairPattern);
    std::sregex_iterator end;
    while (it != end) {
        fields[(*it)[1].str()] = decodeLsblkValue((*it)[2].str());
        ++it;
    }
    return fields;
}

struct MediaDriveCandidate
{
    std::string devicePath;
    std::string displayName;
    std::string label;
    std::string model;
    std::string filesystemType;
    std::string sizeText;
    std::string mountPoint;
};

std::string formatDriveBytesHuman(uint64_t bytes)
{
    static constexpr const char *kUnits[] = { "B", "KB", "MB", "GB", "TB" };
    double value = static_cast<double>(bytes);
    size_t unitIndex = 0;
    while (value >= 1024.0 && unitIndex < (sizeof(kUnits) / sizeof(kUnits[0])) - 1) {
        value /= 1024.0;
        ++unitIndex;
    }

    std::ostringstream out;
    const int precision = (value >= 100.0 || unitIndex == 0) ? 0 : 1;
    out << std::fixed << std::setprecision(precision) << value << ' ' << kUnits[unitIndex];
    return out.str();
}

bool isSystemMountPoint(const std::string &mountPoint)
{
    const std::string normalized = normalizeMountPath(mountPoint);
    return normalized == "/" || normalized.rfind("/boot", 0) == 0;
}

std::string mapValue(const std::map<std::string, std::string> &fields, const std::string &key)
{
    const auto it = fields.find(key);
    return it == fields.end() ? std::string() : it->second;
}

std::vector<MediaDriveCandidate> scanMountableMediaDrives(const std::string &mountPath,
                                                          bool forceRefresh = false)
{
    static std::mutex cacheMutex;
    static std::chrono::steady_clock::time_point lastScanTime{};
    static std::string lastScanMountPath;
    static std::vector<MediaDriveCandidate> lastScanResult;

    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        if (!forceRefresh &&
            lastScanMountPath == mountPath &&
            now - lastScanTime < std::chrono::seconds(3)) {
            return lastScanResult;
        }
    }

    const ShellCommandResult result = runShellCommandCapture(
        "lsblk -P -b -o NAME,PATH,TYPE,SIZE,FSTYPE,LABEL,MODEL,TRAN,RM,RO,MOUNTPOINT,PKNAME");
    if (result.exitCode != 0) {
        std::lock_guard<std::mutex> lock(cacheMutex);
        lastScanTime = now;
        lastScanMountPath = mountPath;
        lastScanResult.clear();
        return {};
    }

    std::vector<std::map<std::string, std::string>> rows;
    std::istringstream input(result.output);
    std::string line;
    while (std::getline(input, line)) {
        line = trimAscii(line);
        if (line.empty())
            continue;
        auto fields = parseLsblkPairs(line);
        if (!fields.empty())
            rows.push_back(std::move(fields));
    }

    std::set<std::string> systemDevices;
    for (const auto &fields : rows) {
        if (!isSystemMountPoint(mapValue(fields, "MOUNTPOINT")))
            continue;

        const std::string path = mapValue(fields, "PATH");
        if (path.empty())
            continue;
        systemDevices.insert(path);

        const std::string parentName = mapValue(fields, "PKNAME");
        if (!parentName.empty())
            systemDevices.insert("/dev/" + parentName);
    }

    const std::string wantedMountPath = normalizeMountPath(mountPath);
    std::set<std::string> seenDevices;
    std::set<std::string> seenDisplayNames;
    std::vector<MediaDriveCandidate> candidates;

    for (const auto &fields : rows) {
        const std::string path = mapValue(fields, "PATH");
        const std::string type = mapValue(fields, "TYPE");
        const std::string fsType = lowerAscii(mapValue(fields, "FSTYPE"));
        const std::string mountPoint = normalizeMountPath(mapValue(fields, "MOUNTPOINT"));
        const std::string parentName = mapValue(fields, "PKNAME");
        const std::string parentPath = parentName.empty() ? path : "/dev/" + parentName;

        if (path.empty() || seenDevices.count(path) > 0)
            continue;
        if (mapValue(fields, "RO") == "1")
            continue;
        if (type != "part" && !(type == "disk" && !fsType.empty()))
            continue;
        if (fsType.empty())
            continue;
        if (systemDevices.count(path) > 0 || systemDevices.count(parentPath) > 0)
            continue;
        if (!mountPoint.empty() && mountPoint != wantedMountPath)
            continue;

        std::string baseName = trimAscii(mapValue(fields, "LABEL"));
        if (baseName.empty())
            baseName = trimAscii(mapValue(fields, "MODEL"));
        if (baseName.empty())
            baseName = path;

        uint64_t sizeBytes = 0;
        try {
            sizeBytes = static_cast<uint64_t>(std::stoull(mapValue(fields, "SIZE")));
        } catch (...) {
            sizeBytes = 0;
        }

        const std::string sizeText = formatDriveBytesHuman(sizeBytes);
        std::string fsUpper = fsType;
        std::transform(fsUpper.begin(), fsUpper.end(), fsUpper.begin(), [](unsigned char ch) {
            return static_cast<char>(std::toupper(ch));
        });

        std::string displayName = baseName + " • " + path + " • " + fsUpper + " • " + sizeText;
        if (seenDisplayNames.count(displayName) > 0)
            displayName += " • " + path;

        candidates.push_back(MediaDriveCandidate{
            path,
            displayName,
            trimAscii(mapValue(fields, "LABEL")),
            trimAscii(mapValue(fields, "MODEL")),
            fsType,
            sizeText,
            mountPoint,
        });
        seenDevices.insert(path);
        seenDisplayNames.insert(displayName);
    }

    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        lastScanTime = now;
        lastScanMountPath = mountPath;
        lastScanResult = candidates;
    }
    return candidates;
}

std::string jsonMediaDriveArray(const std::vector<MediaDriveCandidate> &drives)
{
    std::ostringstream out;
    out << '[';
    for (size_t i = 0; i < drives.size(); ++i) {
        if (i > 0)
            out << ',';
        out << "{"
            << "\"device_path\":\"" << escapeJson(drives[i].devicePath) << "\""
            << ",\"display_name\":\"" << escapeJson(drives[i].displayName) << "\""
            << ",\"label\":\"" << escapeJson(drives[i].label) << "\""
            << ",\"model\":\"" << escapeJson(drives[i].model) << "\""
            << ",\"filesystem_type\":\"" << escapeJson(drives[i].filesystemType) << "\""
            << ",\"size_text\":\"" << escapeJson(drives[i].sizeText) << "\""
            << ",\"mount_point\":\"" << escapeJson(drives[i].mountPoint) << "\""
            << "}";
    }
    out << ']';
    return out.str();
}

std::filesystem::path executableDirectory()
{
    std::error_code ec;
    const std::filesystem::path exePath = std::filesystem::read_symlink("/proc/self/exe", ec);
    return ec ? std::filesystem::path() : exePath.parent_path();
}

std::optional<std::filesystem::path> webIconsDirectory()
{
    const std::filesystem::path exeDir = executableDirectory();
    const std::array<std::filesystem::path, 5> candidates{{
        std::filesystem::current_path() / "src" / "web" / "icons",
        exeDir / ".." / "src" / "web" / "icons",
        exeDir / ".." / "ApertarCore" / "src" / "web" / "icons",
        exeDir / ".." / ".." / "src" / "web" / "icons",
        exeDir / ".." / ".." / "ApertarCore" / "src" / "web" / "icons",
    }};

    std::error_code ec;
    for (const auto &candidate : candidates) {
        if (candidate.empty())
            continue;
        if (std::filesystem::exists(candidate, ec) && std::filesystem::is_directory(candidate, ec))
            return std::filesystem::weakly_canonical(candidate, ec);
        ec.clear();
    }

    return std::nullopt;
}

std::optional<std::string> readBinaryFileString(const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open())
        return std::nullopt;

    return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

std::string readTextFileTrimmed(const std::filesystem::path &path)
{
    if (const auto bytes = readBinaryFileString(path)) {
        std::string text = *bytes;
        std::replace(text.begin(), text.end(), '\0', ' ');
        return trimAscii(text);
    }
    return {};
}

struct AudioDeviceOption
{
    std::string id;
    std::string label;
    std::string displayLabel;
    bool usb = false;
    bool hdmi = false;
};

struct AudioDeviceSnapshot
{
    std::vector<AudioDeviceOption> inputOptions;
    std::vector<AudioDeviceOption> outputOptions;
};

std::string preferredHardwareAudioLabel(const std::string &cardLabel,
                                        const std::string &deviceLabel,
                                        const std::string &cardId)
{
    const std::string trimmedCardLabel = trimAscii(cardLabel);
    const std::string trimmedDeviceLabel = trimAscii(deviceLabel);
    const std::string trimmedCardId = trimAscii(cardId);

    const auto isGeneric = [](const std::string &value) {
        const std::string lower = toLowerAsciiCopy(trimAscii(value));
        return lower.empty()
            || lower == "usb audio"
            || lower == "audio"
            || lower.find("playback") != std::string::npos
            || lower.find("capture") != std::string::npos;
    };

    if (!trimmedCardLabel.empty() && !isGeneric(trimmedCardLabel))
        return trimmedCardLabel;
    if (!trimmedDeviceLabel.empty())
        return trimmedDeviceLabel;
    if (!trimmedCardLabel.empty())
        return trimmedCardLabel;
    return trimmedCardId;
}

std::vector<AudioDeviceOption> parseAlsaHardwareDeviceList(const std::string &text, bool outputMode)
{
    static const std::regex pattern(
        R"(^\s*card\s+(\d+):\s*([^\s\[]+)\s*\[(.*?)\],\s*device\s+(\d+):\s*(.*?)\s*\[(.*?)\]\s*$)",
        std::regex::icase);

    std::vector<AudioDeviceOption> devices;
    std::istringstream input(text);
    std::string line;
    while (std::getline(input, line)) {
        std::smatch match;
        if (!std::regex_match(line, match, pattern))
            continue;

        const std::string cardNumber = trimAscii(match[1].str());
        const std::string cardId = trimAscii(match[2].str());
        const std::string cardLabel = trimAscii(match[3].str());
        const std::string deviceNumber = trimAscii(match[4].str());
        const std::string devicePrimaryLabel = trimAscii(match[5].str());
        const std::string deviceLabel = trimAscii(match[6].str());

        const std::string combined = toLowerAsciiCopy(
            trimAscii(cardId + " " + cardLabel + " " + devicePrimaryLabel + " " + deviceLabel));
        AudioDeviceOption device;
        device.usb = combined.find("usb") != std::string::npos;
        device.hdmi = combined.find("hdmi") != std::string::npos || combined.find("vc4") != std::string::npos;
        device.label = preferredHardwareAudioLabel(cardLabel, deviceLabel, cardId);
        if (device.label.empty())
            device.label = preferredHardwareAudioLabel(cardLabel, devicePrimaryLabel, cardId);

        if (outputMode && device.hdmi && !cardId.empty())
            device.id = "hdmi:CARD=" + cardId + ",DEV=" + deviceNumber;
        else
            device.id = "hw:" + cardNumber + "," + deviceNumber;

        devices.push_back(std::move(device));
    }

    return devices;
}

std::string preferredAudioDeviceLabel(const std::string &primaryName, const std::string &secondaryName)
{
    const std::string trimmedPrimary = trimAscii(primaryName);
    const std::string trimmedSecondary = trimAscii(secondaryName);
    if (trimmedSecondary.empty())
        return trimmedPrimary;

    const std::string lowerSecondary = toLowerAsciiCopy(trimmedSecondary);
    static const std::regex genericSubdevicePattern(R"(^(playback|capture)\s+\d+$)");
    const bool genericSecondary =
        std::regex_match(lowerSecondary, genericSubdevicePattern)
        || lowerSecondary == "playback"
        || lowerSecondary == "capture";
    if (genericSecondary && !trimmedPrimary.empty())
        return trimmedPrimary;

    return trimmedSecondary;
}

std::string normalizeAudioSelectionValue(const std::string &value, bool outputMode)
{
    const std::string normalized = trimAscii(value);
    const std::string lower = toLowerAsciiCopy(normalized);
    if (normalized.empty() || lower == "auto")
        return {};
    if (outputMode && lower == "none")
        return {};
    return normalized;
}

std::string canonicalAudioSelectionId(const std::string &value)
{
    const std::string normalized = trimAscii(value);
    const std::string lower = toLowerAsciiCopy(normalized);
    if (lower.rfind("plughw:", 0) == 0)
        return "hw:" + normalized.substr(7);
    return normalized;
}

std::string audioDeviceIdFromSelectionValue(const std::string &displayValue)
{
    const std::string normalized = normalizeAudioSelectionValue(displayValue, false);
    if (normalized.empty())
        return {};

    const std::string lower = toLowerAsciiCopy(normalized);
    if (lower.rfind("hdmi:", 0) == 0
        || lower.rfind("sysdefault:", 0) == 0
        || lower == "default"
        || lower.rfind("plughw:", 0) == 0) {
        return normalized;
    }

    static const std::regex pattern(R"(((?:plug)?hw:\d+,\d+))", std::regex::icase);
    std::smatch match;
    if (std::regex_search(normalized, match, pattern)) {
        const std::string deviceId = trimAscii(match.str(1));
        const std::string deviceLower = toLowerAsciiCopy(deviceId);
        if (deviceLower.rfind("hw:", 0) == 0)
            return "plughw:" + deviceId.substr(3);
        return deviceId;
    }

    return {};
}

std::string resolveAudioDeviceSelection(const std::string &selection,
                                        const std::vector<AudioDeviceOption> &options,
                                        bool outputMode)
{
    const std::string normalized = normalizeAudioSelectionValue(selection, outputMode);
    if (normalized.empty())
        return {};

    const std::string canonical = canonicalAudioSelectionId(normalized);
    for (const auto &device : options) {
        if (canonicalAudioSelectionId(device.id) == canonical)
            return device.id;
    }

    const std::string normalizedId = audioDeviceIdFromSelectionValue(normalized);
    if (!normalizedId.empty()) {
        const std::string normalizedCanonical = canonicalAudioSelectionId(normalizedId);
        for (const auto &device : options) {
            if (canonicalAudioSelectionId(device.id) == normalizedCanonical)
                return device.id;
        }
    }

    for (const auto &device : options) {
        if (normalized == device.label || normalized == device.displayLabel)
            return device.id;
    }

    return {};
}

std::vector<AudioDeviceOption> parseAlsaDeviceList(const std::string &text, bool outputMode)
{
    static const std::regex pattern(
        R"(^\s*(\d+)-(\d+):\s*(.*?)\s*:\s*(.*?)\s*:\s*(playback|capture)\s+\d+\s*$)");

    std::vector<AudioDeviceOption> devices;
    std::map<std::string, std::string> cardIds;
    std::istringstream input(text);
    std::string line;
    while (std::getline(input, line)) {
        std::smatch match;
        if (!std::regex_match(line, match, pattern))
            continue;

        const bool isPlayback = toLowerAsciiCopy(trimAscii(match[5].str())) == "playback";
        if (isPlayback != outputMode)
            continue;

        const std::string cardNumber = trimAscii(match[1].str());
        const std::string deviceNumber = trimAscii(match[2].str());
        const std::string primaryName = trimAscii(match[3].str());
        const std::string secondaryName = trimAscii(match[4].str());

        std::string cardId = cardIds[cardNumber];
        if (cardId.empty()) {
            cardId = readTextFileTrimmed(std::filesystem::path("/proc/asound") / ("card" + cardNumber) / "id");
            cardIds[cardNumber] = cardId;
        }

        const std::string combined = toLowerAsciiCopy(trimAscii(cardId + " " + primaryName + " " + secondaryName));
        AudioDeviceOption device;
        device.usb = combined.find("usb") != std::string::npos;
        device.hdmi = combined.find("hdmi") != std::string::npos || combined.find("vc4") != std::string::npos;
        device.label = preferredAudioDeviceLabel(primaryName, secondaryName);
        if (device.label.empty())
            device.label = !primaryName.empty() ? primaryName : cardId;

        if (outputMode && device.hdmi && !cardId.empty())
            device.id = "hdmi:CARD=" + cardId + ",DEV=" + deviceNumber;
        else
            device.id = "hw:" + cardNumber + "," + deviceNumber;

        devices.push_back(std::move(device));
    }

    return devices;
}

std::string friendlyOutputLabel(const AudioDeviceOption &device)
{
    if (!device.hdmi)
        return device.label;

    const std::string combined = toLowerAsciiCopy(device.id + " " + device.label);
    if (combined.find("hdmi1") != std::string::npos || combined.find("hdmi-1") != std::string::npos)
        return "HDMI 2";
    if (combined.find("hdmi0") != std::string::npos || combined.find("hdmi-0") != std::string::npos)
        return "HDMI 1";
    return "HDMI Output";
}

std::vector<AudioDeviceOption> formatAudioDeviceOptions(const std::vector<AudioDeviceOption> &devices,
                                                        bool outputMode)
{
    std::vector<AudioDeviceOption> filtered;
    std::set<std::string> seenIds;
    for (const auto &device : devices) {
        const bool keep = device.usb;
        if (!keep || !seenIds.insert(device.id).second)
            continue;

        AudioDeviceOption formatted = device;
        if (outputMode)
            formatted.label = friendlyOutputLabel(device);
        filtered.push_back(std::move(formatted));
    }

    std::map<std::string, int> labelCounts;
    for (const auto &device : filtered)
        labelCounts[device.label] += 1;

    std::map<std::string, int> labelIndex;
    for (auto &device : filtered) {
        const int total = labelCounts[device.label];
        const int index = ++labelIndex[device.label];
        device.displayLabel = total > 1
            ? device.label + " (" + std::to_string(index) + ")"
            : device.label;
    }

    if (outputMode) {
        AudioDeviceOption noneOption;
        noneOption.label = "None";
        noneOption.displayLabel = "None";
        filtered.insert(filtered.begin(), std::move(noneOption));
    }

    return filtered;
}

bool audioChannelRange(const std::string &selection, unsigned &first, unsigned &count)
{
    first = 1; count = 0;
    if (selection == "auto") return true;
    if (selection == "all") { count = 65535; return true; }
    const auto dash = selection.find('-');
    const auto start = parseIntegerValue(selection.substr(0, dash));
    const auto end = dash == std::string::npos ? start : parseIntegerValue(selection.substr(dash + 1));
    if (!start || !end || *start < 1 || *end < *start || *end > 32) return false;
    first = unsigned(*start); count = unsigned(*end - *start + 1);
    return true;
}

AudioRecordingOptions audioRecordingOptionsFromUiSettings(const SharedUiSettingsSnapshot &uiSettings,
                                                          const AudioDeviceSnapshot &audioDevices)
{
    AudioRecordingOptions options;
    options.inputVolume = static_cast<uint8_t>(std::clamp(uiSettings.inputVolume, 0, 100));
    options.outputVolume = static_cast<uint8_t>(std::clamp(uiSettings.headphoneVolume, 0, 100));
    options.sampleFormat = uiSettings.audioSampleFormat;
    options.sampleRate = uiSettings.audioSampleRate;
    unsigned first = 1, count = 0;
    if (!audioChannelRange(uiSettings.audioChannels, first, count)) first = 33;
    options.firstChannel = first;
    options.channels = count;

    const std::string resolvedInputSelection =
        resolveAudioDeviceSelection(uiSettings.audioInputDevice, audioDevices.inputOptions, false);
    const std::string resolvedOutputSelection =
        resolveAudioDeviceSelection(uiSettings.audioOutputDevice, audioDevices.outputOptions, true);
    options.inputDevice = audioDeviceIdFromSelectionValue(resolvedInputSelection);
    options.outputDevice = audioDeviceIdFromSelectionValue(resolvedOutputSelection);

    options.enabled = uiSettings.recordAudioEnabled && !options.inputDevice.empty();
    options.liveMonitoringEnabled =
        options.enabled
        && uiSettings.liveAudioMonitoringEnabled
        && !options.outputDevice.empty()
        && options.outputVolume > 0;

    if (!options.liveMonitoringEnabled)
        options.outputDevice.clear();
    return options;
}

AudioDeviceSnapshot detectAudioDevices()
{
    static std::mutex cacheMutex;
    static auto cacheTime = std::chrono::steady_clock::time_point{};
    static AudioDeviceSnapshot cachedSnapshot;

    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        if (cacheTime.time_since_epoch().count() != 0
            && (now - cacheTime) < std::chrono::seconds(2)) {
            return cachedSnapshot;
        }
    }

    AudioDeviceSnapshot snapshot;
    const ShellCommandResult inputList = runShellCommandCapture("arecord -l");
    const ShellCommandResult outputList = runShellCommandCapture("aplay -l");
    if (inputList.exitCode == 0)
        snapshot.inputOptions = formatAudioDeviceOptions(parseAlsaHardwareDeviceList(inputList.output, false), false);
    if (outputList.exitCode == 0)
        snapshot.outputOptions = formatAudioDeviceOptions(parseAlsaHardwareDeviceList(outputList.output, true), true);

    if (snapshot.inputOptions.empty() || snapshot.outputOptions.empty()) {
        const std::string pcmText = readTextFileTrimmed("/proc/asound/pcm");
        if (!pcmText.empty()) {
            if (snapshot.inputOptions.empty())
                snapshot.inputOptions = formatAudioDeviceOptions(parseAlsaDeviceList(pcmText, false), false);
            if (snapshot.outputOptions.empty())
                snapshot.outputOptions = formatAudioDeviceOptions(parseAlsaDeviceList(pcmText, true), true);
        }
    }

    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        cachedSnapshot = snapshot;
        cacheTime = now;
    }
    return snapshot;
}

std::string jsonAudioDeviceOptions(const std::vector<AudioDeviceOption> &options)
{
    std::ostringstream out;
    out << '[';
    for (size_t i = 0; i < options.size(); ++i) {
        if (i > 0)
            out << ',';
        out << "{"
            << "\"id\":\"" << escapeJson(options[i].id) << "\""
            << ",\"label\":\"" << escapeJson(options[i].displayLabel.empty() ? options[i].label : options[i].displayLabel) << "\""
            << "}";
    }
    out << ']';
    return out.str();
}

constexpr const char *kFanBlockStart = "# BEGIN APERTAR FAN MODE";
constexpr const char *kFanBlockEnd = "# END APERTAR FAN MODE";
constexpr const char *kBootConfigPath = "/boot/firmware/config.txt";

const std::vector<std::string> &availableTimeZones()
{
    static const std::vector<std::string> zones = []() {
        std::set<std::string> collected;
        for (const std::filesystem::path &path : {
                 std::filesystem::path("/usr/share/zoneinfo/zone1970.tab"),
                 std::filesystem::path("/usr/share/zoneinfo/zone.tab") }) {
            std::ifstream input(path);
            if (!input.is_open())
                continue;

            std::string line;
            while (std::getline(input, line)) {
                const std::string trimmed = trimAscii(line);
                if (trimmed.empty() || trimmed[0] == '#')
                    continue;

                const std::vector<std::string> parts = splitWhitespace(trimmed);
                if (parts.size() >= 3)
                    collected.insert(parts[2]);
            }
        }

        if (collected.empty()) {
            collected.insert("UTC");
            collected.insert("Europe/Bucharest");
            collected.insert("Europe/London");
            collected.insert("America/New_York");
            collected.insert("America/Los_Angeles");
            collected.insert("Asia/Tokyo");
        }

        return std::vector<std::string>(collected.begin(), collected.end());
    }();
    return zones;
}

std::string currentTimeZoneId()
{
    const std::string fromEtcTimezone = readTextFileTrimmed("/etc/timezone");
    if (!fromEtcTimezone.empty())
        return fromEtcTimezone;

    std::error_code ec;
    const std::filesystem::path symlinkPath = std::filesystem::read_symlink("/etc/localtime", ec);
    if (!ec) {
        const std::string value = symlinkPath.string();
        const size_t marker = value.find("zoneinfo/");
        if (marker != std::string::npos)
            return value.substr(marker + std::strlen("zoneinfo/"));
    }

    return "UTC";
}

std::string currentLocalDateTimeInputValue()
{
    const std::time_t now = std::time(nullptr);
    std::tm localTm{};
#if defined(_WIN32)
    localtime_s(&localTm, &now);
#else
    localtime_r(&now, &localTm);
#endif
    char buffer[32]{};
    if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M", &localTm) == 0)
        return {};
    return buffer;
}

uint64_t currentLocalMillisecondsOfDay()
{
    const auto now = std::chrono::system_clock::now();
    const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
    std::tm localTm{};
#if defined(_WIN32)
    localtime_s(&localTm, &nowTime);
#else
    localtime_r(&nowTime, &localTm);
#endif

    const uint64_t millisecondsPart = static_cast<uint64_t>(nowMs % 1000ll);
    return (((static_cast<uint64_t>(localTm.tm_hour) * 60ull
              + static_cast<uint64_t>(localTm.tm_min)) * 60ull
             + static_cast<uint64_t>(localTm.tm_sec)) * 1000ull)
           + millisecondsPart;
}

bool parseBooleanToken(const std::string &value, bool *parsedValue)
{
    if (!parsedValue)
        return false;

    std::string normalized = trimAscii(value);
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    if (normalized == "true" || normalized == "yes" || normalized == "1") {
        *parsedValue = true;
        return true;
    }
    if (normalized == "false" || normalized == "no" || normalized == "0") {
        *parsedValue = false;
        return true;
    }

    return false;
}

bool timedatectlBoolProperty(const std::string &propertyName, bool *value)
{
    if (!value)
        return false;

    const ShellCommandResult result =
        runShellCommandCapture(timedatectlProgram() + " show -p "
                               + shellQuote(propertyName) + " --value");
    if (result.exitCode != 0)
        return false;

    return parseBooleanToken(result.output, value);
}

bool waitForTimedatectlNetworkSync(std::chrono::milliseconds timeout,
                                   std::chrono::milliseconds pollInterval = std::chrono::milliseconds(250))
{
    bool canNtp = false;
    if (timedatectlBoolProperty("CanNTP", &canNtp) && !canNtp)
        return false;

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        bool synced = false;
        if (timedatectlBoolProperty("NTPSynchronized", &synced) && synced)
            return true;
        std::this_thread::sleep_for(pollInterval);
    }

    bool synced = false;
    return timedatectlBoolProperty("NTPSynchronized", &synced) && synced;
}

bool hardwareClockDeviceAvailable()
{
    if (std::filesystem::exists("/dev/rtc")
        || std::filesystem::exists("/dev/rtc0"))
        return true;

    const std::filesystem::path rtcClassDir("/sys/class/rtc");
    std::error_code ec;
    if (!std::filesystem::is_directory(rtcClassDir, ec))
        return false;

    for (const auto &entry : std::filesystem::directory_iterator(rtcClassDir, ec)) {
        if (ec)
            break;
        const std::string name = entry.path().filename().string();
        if (name.rfind("rtc", 0) == 0)
            return true;
    }

    return false;
}

bool isMissingHardwareClockError(const std::string &errorMessage)
{
    std::string lower = errorMessage;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    return lower.find("cannot access the hardware clock via any known method") != std::string::npos
        || lower.find("no usable clock interface found") != std::string::npos
        || lower.find("no such file or directory") != std::string::npos;
}

bool syncHardwareClockIfAvailable(std::string *errorMessage)
{
    if (!hardwareClockDeviceAvailable())
        return true;

    std::string stepError;
    if (runShellCommand("sudo -n " + shellQuote(hwclockProgram()) + " --systohc --utc", &stepError))
        return true;

    if (isMissingHardwareClockError(stepError))
        return true;

    if (errorMessage)
        *errorMessage = stepError;
    return false;
}

std::string fanModeBlock(const std::string &mode)
{
    if (mode == "Auto")
        return {};

    std::ostringstream out;
    out << kFanBlockStart << '\n'
        << "# mode: " << mode << '\n'
        << "dtparam=cooling_fan=on\n";

    if (mode == "Silent") {
        out << "dtparam=fan_temp0=60000\n"
            << "dtparam=fan_temp0_hyst=5000\n"
            << "dtparam=fan_temp0_speed=90\n"
            << "dtparam=fan_temp1=70000\n"
            << "dtparam=fan_temp1_hyst=5000\n"
            << "dtparam=fan_temp1_speed=140\n"
            << "dtparam=fan_temp2=77500\n"
            << "dtparam=fan_temp2_hyst=4000\n"
            << "dtparam=fan_temp2_speed=190\n"
            << "dtparam=fan_temp3=82000\n"
            << "dtparam=fan_temp3_hyst=3000\n"
            << "dtparam=fan_temp3_speed=255\n";
    } else if (mode == "Full Blast") {
        out << "dtparam=fan_temp0=0\n"
            << "dtparam=fan_temp0_hyst=0\n"
            << "dtparam=fan_temp0_speed=255\n"
            << "dtparam=fan_temp1=1000\n"
            << "dtparam=fan_temp1_hyst=0\n"
            << "dtparam=fan_temp1_speed=255\n"
            << "dtparam=fan_temp2=2000\n"
            << "dtparam=fan_temp2_hyst=0\n"
            << "dtparam=fan_temp2_speed=255\n"
            << "dtparam=fan_temp3=3000\n"
            << "dtparam=fan_temp3_hyst=0\n"
            << "dtparam=fan_temp3_speed=255\n";
    }

    out << kFanBlockEnd << '\n';
    return out.str();
}

std::string stripManagedFanBlock(std::string text)
{
    const size_t start = text.find(kFanBlockStart);
    if (start == std::string::npos)
        return text;

    size_t end = text.find(kFanBlockEnd, start);
    if (end == std::string::npos)
        return text;

    end += std::strlen(kFanBlockEnd);
    while (end < text.size() && (text[end] == '\n' || text[end] == '\r'))
        ++end;

    text.erase(start, end - start);
    while (text.find("\n\n\n") != std::string::npos)
        text.replace(text.find("\n\n\n"), 3, "\n\n");
    return text;
}

std::string detectManagedFanMode(const std::string &text)
{
    const size_t start = text.find(kFanBlockStart);
    if (start == std::string::npos)
        return "Auto";

    const size_t end = text.find(kFanBlockEnd, start);
    const std::string block = end == std::string::npos ? text.substr(start) : text.substr(start, end - start);
    if (block.find("# mode: Silent") != std::string::npos)
        return "Silent";
    if (block.find("# mode: Full Blast") != std::string::npos)
        return "Full Blast";
    return "Auto";
}

std::string currentFanMode()
{
    const std::string config = readTextFileRaw(kBootConfigPath);
    if (config.empty())
        return "Auto";
    return detectManagedFanMode(config);
}

bool applyFanMode(const std::string &mode, std::string *errorMessage)
{
    if (mode != "Silent" && mode != "Auto" && mode != "Full Blast") {
        if (errorMessage)
            *errorMessage = "Unknown fan mode selected.";
        return false;
    }

    const std::string existing = readTextFileRaw(kBootConfigPath);
    if (existing.empty()) {
        if (errorMessage)
            *errorMessage = "Could not read /boot/firmware/config.txt.";
        return false;
    }

    std::string updated = trimAscii(stripManagedFanBlock(existing));
    if (!updated.empty())
        updated += "\n\n";
    updated += fanModeBlock(mode);

    std::filesystem::path tempPath = std::filesystem::temp_directory_path() /
        ("apertar-fan-mode-" + std::to_string(::getpid()) + ".tmp");
    if (!writeTextFileRaw(tempPath, updated)) {
        if (errorMessage)
            *errorMessage = "Could not prepare the fan mode update.";
        return false;
    }

    std::string installError;
    const bool ok = runShellCommand(
        "sudo -n install -m 644 " + shellQuote(tempPath.string()) + " " + shellQuote(kBootConfigPath),
        &installError);
    std::error_code rmEc;
    std::filesystem::remove(tempPath, rmEc);

    if (ok)
        return true;

    const std::string lower = installError;
    if (errorMessage) {
        if (lower.find("password") != std::string::npos ||
            lower.find("permission denied") != std::string::npos ||
            lower.find("not allowed") != std::string::npos) {
            *errorMessage = "Changing the fan mode needs sudo permission.";
        } else {
            *errorMessage = installError.empty()
                ? "Could not update the fan mode configuration."
                : installError;
        }
    }
    return false;
}

std::string normalizedPowerError(const std::string &errorMessage)
{
    std::string normalized = trimAscii(errorMessage);
    std::string lower = normalized;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    if (lower.find("multiple-sessions") != std::string::npos ||
        lower.find("authentication is required") != std::string::npos ||
        lower.find("interactive authentication required") != std::string::npos ||
        lower.find("access denied") != std::string::npos ||
        lower.find("a password is required") != std::string::npos ||
        lower.find("sudo") != std::string::npos) {
        return "Shutdown needs a passwordless sudo rule for systemctl or a polkit rule that allows power off with multiple sessions.";
    }

    return normalized.empty()
        ? "Permission denied or system power control is unavailable."
        : normalized;
}

bool invokeSystemPowerAction(const std::string &action, std::string *errorMessage)
{
    std::string commandError;
    if (runShellCommand("sudo -n systemctl --no-ask-password " + shellQuote(action), &commandError))
        return true;
    if (runShellCommand("systemctl --no-ask-password " + shellQuote(action), &commandError))
        return true;

    if (errorMessage)
        *errorMessage = normalizedPowerError(commandError);
    return false;
}

bool applyDateTimeSettings(int year,
                           int month,
                           int day,
                           int hour,
                           int minute,
                           const std::string &timeZone,
                           std::string *errorMessage)
{
    if (year < 2000 || year > 2099 || month < 1 || month > 12 ||
        day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59) {
        if (errorMessage)
            *errorMessage = "The selected date or time is invalid.";
        return false;
    }

    std::ostringstream dateTime;
    dateTime << std::setfill('0')
             << std::setw(4) << year << '-'
             << std::setw(2) << month << '-'
             << std::setw(2) << day << ' '
             << std::setw(2) << hour << ':'
             << std::setw(2) << minute << ":00";

    std::string stepError;
    if (!runShellCommand("sudo -n " + shellQuote(timedatectlProgram()) + " set-ntp false", &stepError)) {
        if (errorMessage)
            *errorMessage = stepError.empty() ? "Could not disable automatic network time." : stepError;
        return false;
    }

    if (!runShellCommand("sudo -n " + shellQuote(timedatectlProgram()) + " set-timezone "
                         + shellQuote(timeZone), &stepError)) {
        if (errorMessage)
            *errorMessage = stepError.empty() ? "Could not update the system timezone." : stepError;
        return false;
    }

    if (!runShellCommand("sudo -n " + shellQuote(timedatectlProgram()) + " set-time "
                         + shellQuote(dateTime.str()), &stepError)) {
        if (errorMessage)
            *errorMessage = stepError.empty() ? "Could not update the system clock." : stepError;
        return false;
    }

    if (!syncHardwareClockIfAvailable(&stepError)) {
        if (errorMessage)
            *errorMessage = stepError.empty()
                ? "The system time changed, but writing it to the RTC failed."
                : stepError;
        return false;
    }

    return true;
}

bool applyTimeZoneSettings(const std::string &timeZone,
                           bool *networkTimeSynced,
                           std::string *errorMessage)
{
    if (networkTimeSynced)
        *networkTimeSynced = false;

    if (trimAscii(timeZone).empty()) {
        if (errorMessage)
            *errorMessage = "A timezone must be selected.";
        return false;
    }

    std::string stepError;
    if (!runShellCommand("sudo -n " + shellQuote(timedatectlProgram()) + " set-timezone "
                         + shellQuote(timeZone), &stepError)) {
        if (errorMessage)
            *errorMessage = stepError.empty() ? "Could not update the system timezone." : stepError;
        return false;
    }

    stepError.clear();
    runShellCommand("sudo -n " + shellQuote(timedatectlProgram()) + " set-ntp true", &stepError);

    const bool synced = waitForTimedatectlNetworkSync(std::chrono::milliseconds(2500));
    if (networkTimeSynced)
        *networkTimeSynced = synced;

    if (!synced)
        return true;

    if (!syncHardwareClockIfAvailable(&stepError)) {
        if (errorMessage)
            *errorMessage = stepError.empty()
                ? "The system clock synced, but writing it to the RTC failed."
                : stepError;
        return false;
    }

    return true;
}

std::string parentDriveForDevice(const std::string &devicePath)
{
    if (devicePath.rfind("/dev/", 0) != 0)
        return {};

    const ShellCommandResult result =
        runShellCommandCapture("lsblk -no PKNAME " + shellQuote(devicePath));
    const std::string parentName = trimAscii(result.output);
    if (result.exitCode != 0 || parentName.empty())
        return {};
    return "/dev/" + parentName;
}

std::string pciAddressForDevice(const std::string &devicePath)
{
    if (devicePath.rfind("/dev/", 0) != 0)
        return {};

    const ShellCommandResult result =
        runShellCommandCapture("udevadm info -q path -n " + shellQuote(devicePath));
    if (result.exitCode != 0 || result.output.empty())
        return {};

    const std::regex pciPattern(R"((\d{4}:\d{2}:\d{2}\.\d))");
    std::sregex_iterator it(result.output.begin(), result.output.end(), pciPattern);
    std::sregex_iterator end;
    std::string lastMatch;
    while (it != end) {
        lastMatch = (*it)[1].str();
        ++it;
    }
    return lastMatch;
}

void flushMediaPath(const std::string &mountPath, const std::string &devicePath)
{
    std::string ignoredError;
    if (!runShellCommand("sync -f " + shellQuote(mountPath), &ignoredError))
        runShellCommand("sync", nullptr);

    if (devicePath.rfind("/dev/", 0) == 0) {
        runShellCommand("sudo -n blockdev --flushbufs " + shellQuote(devicePath), nullptr);
    }
}

bool detachNvmeController(const std::string &devicePath, std::string *errorMessage)
{
    const std::string pciAddress = pciAddressForDevice(devicePath);
    if (pciAddress.empty()) {
        if (errorMessage)
            *errorMessage = "Media was unmounted, but the NVMe controller path could not be resolved.";
        return false;
    }

    const std::string removePath = "/sys/bus/pci/devices/" + pciAddress + "/remove";
    if (!std::filesystem::exists(removePath)) {
        if (errorMessage)
            *errorMessage = "Media was unmounted, but the NVMe controller remove path was not found.";
        return false;
    }

    if (!runShellCommand("sudo -n sh -c " + shellQuote("echo 1 > " + removePath), errorMessage))
        return false;

    runShellCommand("udevadm settle", nullptr);
    return true;
}

bool startNvmeReinsertWatcherAtPath(const std::string &mountPath, std::string *errorMessage)
{
    static const std::string helperScript = R"PY(
import configparser
import os
import pwd
import re
import smbus
import subprocess
import time

I2C_BUS = 10
I2C_ADDR = 0x34
CFE_HAT_ENABLED = True
MOUNT_PATH = os.environ.get("APERTAR_MEDIA_ROOT", "/media/RAW")
PCIE_CONTROLLER = "1000110000.pcie"

def hardware_config_candidate_paths():
    candidates = []
    seen = set()

    def add_candidate(path):
        if not path or path in seen:
            return
        seen.add(path)
        candidates.append(path)

    add_candidate(os.environ.get("APERTAR_HARDWARE_CONFIG"))
    add_candidate("/home/pi/apertar-hardware.conf")
    add_candidate("/boot/firmware/apertar-hardware.conf")
    add_candidate("/boot/apertar-hardware.conf")
    return candidates

def parse_config_int(raw_value, default_value):
    if raw_value is None:
        return default_value
    value = str(raw_value).strip()
    if not value:
        return default_value
    try:
        return int(value, 0)
    except ValueError:
        return default_value

def parse_config_bool(raw_value, default_value):
    if raw_value is None:
        return default_value
    value = str(raw_value).split("#", 1)[0].split(";", 1)[0].strip().lower()
    if value in ("1", "true", "yes", "on", "enabled"):
        return True
    if value in ("0", "false", "no", "off", "disabled"):
        return False
    return default_value

def load_hardware_config():
    global I2C_BUS, I2C_ADDR, CFE_HAT_ENABLED, MOUNT_PATH, PCIE_CONTROLLER

    parser = configparser.ConfigParser()
    parser.optionxform = str.lower

    config_path = next((path for path in hardware_config_candidate_paths() if path and os.path.isfile(path)), None)
    if not config_path:
        return

    try:
        parser.read(config_path)
    except OSError:
        return

    if parser.has_section("cfexpress_hat"):
        section = parser["cfexpress_hat"]
        CFE_HAT_ENABLED = parse_config_bool(section.get("enabled"), CFE_HAT_ENABLED)
        I2C_BUS = parse_config_int(section.get("bus", section.get("channel", section.get("i2c_bus"))), I2C_BUS)
        I2C_ADDR = parse_config_int(section.get("address", section.get("addr")), I2C_ADDR)

    if parser.has_section("storage"):
        section = parser["storage"]
        if "APERTAR_MEDIA_ROOT" not in os.environ:
            mount_path = section.get("mount_path", "").strip()
            if mount_path:
                MOUNT_PATH = mount_path
        pcie_controller = section.get("pcie_controller", "").strip()
        if pcie_controller:
            PCIE_CONTROLLER = pcie_controller

def run_command(arguments):
    return subprocess.run(arguments, text=True, capture_output=True, check=False)

def run_privileged_command(arguments):
    if os.geteuid() == 0:
        return run_command(arguments)
    return run_command(["sudo", "-n", *arguments])

def write_privileged_file(path, value):
    if os.geteuid() == 0:
        try:
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(str(value))
            return True
        except OSError:
            return False

    return run_command(["sudo", "-n", "sh", "-c", f"echo {value} > {path}"]).returncode == 0

def read_insert_state(bus, timeout_seconds=0.5, default_state=True):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        try:
            data = bus.read_byte(I2C_ADDR)
        except OSError:
            time.sleep(0.1)
            continue
        if data != 0x69:
            return (data & 0x01) == 0x01
        time.sleep(0.1)
    return default_state

def get_filesystem_type(device_path):
    try:
        result = run_privileged_command(["blkid", "-s", "TYPE", "-o", "value", device_path])
        if result.returncode == 0:
            return result.stdout.strip()
    except Exception:
        pass
    return None

def find_nvme_partition():
    try:
        output = subprocess.check_output(
            "lsblk -rno NAME,TYPE | awk '$2==\"part\" && $1 ~ /^nvme[0-9]+n[0-9]+p[0-9]+$/ {print $1}'",
            shell=True,
            text=True
        ).strip().splitlines()
        if not output:
            return None

        def sort_key(name):
            match = re.search(r"nvme(\d+)n(\d+)p(\d+)$", name)
            if match:
                return tuple(map(int, match.groups()))
            return (9999, 9999, 9999)

        output.sort(key=sort_key)
        return "/dev/" + output[-1]
    except Exception:
        return None

def wait_for_nvme_partition(timeout_seconds=3.0, interval_seconds=0.15):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        device_path = find_nvme_partition()
        if device_path:
            return device_path
        time.sleep(interval_seconds)
    return None

def mount_partition(device_path):
    fs_type = get_filesystem_type(device_path)
    if not fs_type:
        return False

    if run_privileged_command(["mkdir", "-p", MOUNT_PATH]).returncode != 0:
        return False

    if fs_type == "ntfs":
        command = ["mount", "-t", "ntfs3", "-o", "uid=1000,gid=1000", device_path, MOUNT_PATH]
    elif fs_type == "ext4":
        command = ["mount", "-t", "ext4", device_path, MOUNT_PATH]
    elif fs_type == "exfat":
        command = ["mount", "-t", "exfat", "-o", "uid=1000,gid=1000", device_path, MOUNT_PATH]
    else:
        return False

    return run_privileged_command(command).returncode == 0

def reset_pcie_controller():
    controller_path = f"/sys/bus/platform/drivers/brcm-pcie/{PCIE_CONTROLLER}"
    if os.path.exists(controller_path):
        write_privileged_file("/sys/bus/platform/drivers/brcm-pcie/unbind", PCIE_CONTROLLER)

    write_privileged_file("/sys/bus/platform/drivers/brcm-pcie/bind", PCIE_CONTROLLER)
    if wait_for_nvme_partition(timeout_seconds=2.0):
        return

    write_privileged_file("/sys/bus/pci/rescan", 1)
    wait_for_nvme_partition(timeout_seconds=3.0)
    run_command(["udevadm", "settle"])

load_hardware_config()
if not CFE_HAT_ENABLED:
    raise SystemExit(0)
bus = smbus.SMBus(I2C_BUS)

for _ in range(600):
    if read_insert_state(bus):
        break
    time.sleep(0.1)
else:
    raise SystemExit(0)

for _ in range(600):
    if not read_insert_state(bus):
        break
    time.sleep(0.1)
else:
    raise SystemExit(0)

reset_pcie_controller()

device_path = find_nvme_partition() or wait_for_nvme_partition(timeout_seconds=3.0)
if device_path and mount_partition(device_path):
    raise SystemExit(0)
)PY";

    const std::string command = "APERTAR_MEDIA_ROOT=" + shellQuote(mountPath)
        + " python3 -c " + shellQuote(helperScript) + " >/dev/null 2>&1 &";
    if (runShellCommand(command, nullptr))
        return true;

    if (errorMessage)
        *errorMessage = "Media was ejected, but the automatic remount watcher could not be started.";
    return false;
}

bool mountDeviceAtMediaPath(const std::string &devicePath,
                            const std::string &mountPath,
                            const std::string &filesystemType,
                            std::string *errorMessage)
{
    if (!runShellCommand("sudo -n mkdir -p " + shellQuote(mountPath), errorMessage))
        return false;

    std::string fsType = lowerAscii(trimAscii(filesystemType));
    std::string mountType = fsType;
    std::string options;

    if (fsType == "ntfs") {
        mountType = "ntfs3";
        options = "uid=1000,gid=1000,umask=0002";
    } else if (fsType == "exfat" || fsType == "vfat" || fsType == "fat32" || fsType == "fat" || fsType == "msdos") {
        options = "uid=1000,gid=1000,umask=0002";
    }

    std::string command = "sudo -n mount";
    if (!mountType.empty())
        command += " -t " + shellQuote(mountType);
    if (!options.empty())
        command += " -o " + shellQuote(options);
    command += " " + shellQuote(devicePath) + " " + shellQuote(mountPath);

    if (fsType == "ntfs") {
        std::string ntfs3Error;
        if (runShellCommand(command, &ntfs3Error))
            return true;

        std::string fallbackCommand =
            "sudo -n mount -t ntfs-3g -o " + shellQuote(options)
            + " " + shellQuote(devicePath) + " " + shellQuote(mountPath);
        if (runShellCommand(fallbackCommand, errorMessage))
            return true;

        if (errorMessage && errorMessage->empty())
            *errorMessage = ntfs3Error;
        return false;
    }

    return runShellCommand(command, errorMessage);
}

bool mountFormattedDeviceAtPath(const std::string &devicePath,
                                const std::string &mountPath,
                                const std::string &filesystemType,
                                std::string *errorMessage)
{
    return mountDeviceAtMediaPath(devicePath, mountPath, filesystemType, errorMessage);
}

bool mountMediaDeviceAtPath(const std::string &devicePath,
                            const std::string &mountPath,
                            std::string *errorMessage)
{
    const std::string normalizedDevicePath = trimAscii(devicePath);
    if (isMountedPath(mountPath)) {
        if (errorMessage)
            *errorMessage = "Eject the currently mounted media before mounting another drive.";
        return false;
    }
    if (normalizedDevicePath.rfind("/dev/", 0) != 0) {
        if (errorMessage)
            *errorMessage = "Select a valid drive to mount.";
        return false;
    }
    std::string fsType;
    const std::vector<MediaDriveCandidate> candidates = scanMountableMediaDrives(mountPath, true);
    for (const MediaDriveCandidate &candidate : candidates) {
        if (candidate.devicePath == normalizedDevicePath) {
            fsType = candidate.filesystemType;
            break;
        }
    }

    if (fsType.empty()) {
        if (errorMessage)
            *errorMessage = "That drive is not available for camera media mounting.";
        return false;
    }

    std::string commandError;
    if (mountDeviceAtMediaPath(normalizedDevicePath, mountPath, fsType, &commandError)) {
        scanMountableMediaDrives(mountPath, true);
        return true;
    }

    const std::string lower = lowerAscii(commandError);
    if (errorMessage) {
        if (lower.find("password") != std::string::npos ||
            lower.find("permission denied") != std::string::npos ||
            lower.find("not allowed") != std::string::npos) {
            *errorMessage = "Mounting this drive needs sudo permission for mkdir and mount.";
        } else if (lower.find("unknown filesystem") != std::string::npos ||
                   lower.find("wrong fs type") != std::string::npos ||
                   lower.find("bad option") != std::string::npos) {
            *errorMessage = "The selected drive format is not supported by this Pi image.";
        } else {
            *errorMessage = commandError.empty() ? "The selected drive could not be mounted." : commandError;
        }
    }
    return false;
}

bool ejectMediaAtPath(const std::string &mountPath, std::string *errorMessage)
{
    const std::string devicePath = mountedDeviceForPath(mountPath);
    if (devicePath.empty()) {
        if (errorMessage)
            *errorMessage = "No media is currently mounted.";
        return false;
    }

    flushMediaPath(mountPath, devicePath);
    const bool isNvme = devicePath.rfind("/dev/nvme", 0) == 0;
    bool unmounted = false;
    std::string commandError;

    if (devicePath.rfind("/dev/", 0) == 0) {
        unmounted = runShellCommand(
            "udisksctl unmount -b " + shellQuote(devicePath) + " --no-user-interaction",
            &commandError);

        if (unmounted && !isNvme) {
            const std::string drivePath = parentDriveForDevice(devicePath);
            if (!drivePath.empty())
                runShellCommand("udisksctl power-off -b " + shellQuote(drivePath) + " --no-user-interaction", nullptr);
        }
    }

    if (!unmounted)
        unmounted = runShellCommand("umount " + shellQuote(mountPath), &commandError);

    if (unmounted && isNvme && !detachNvmeController(devicePath, &commandError))
        unmounted = false;

    if (unmounted && isNvme && !startNvmeReinsertWatcherAtPath(mountPath, &commandError))
        unmounted = false;

    if (unmounted)
        return true;

    const std::string lower = commandError;
    if (errorMessage) {
        if (lower.find("Authentication is required") != std::string::npos ||
            lower.find("not authorized") != std::string::npos ||
            lower.find("another user") != std::string::npos) {
            *errorMessage = isNvme
                ? "Media eject needs sudo permission to detach the NVMe controller."
                : "Media eject needs permission. Add a UDisks polkit rule for this user.";
        } else {
            *errorMessage = commandError.empty() ? "Media could not be ejected." : commandError;
        }
    }
    return false;
}

bool formatMediaAtPath(const std::string &mountPath,
                       const std::string &filesystemType,
                       std::string *errorMessage)
{
    const std::string normalizedFilesystemType = normalizedFormatFilesystemType(filesystemType);
    if (normalizedFilesystemType.empty() || normalizedFilesystemType == "ext4") {
        if (errorMessage)
            *errorMessage = "Select exFAT, FAT32, or NTFS before formatting.";
        return false;
    }

    runShellCommand("sync", nullptr);

    const std::string devicePath = mountedDeviceForPath(mountPath);
    if (devicePath.rfind("/dev/", 0) != 0) {
        if (errorMessage)
            *errorMessage = "Could not determine which card device to format.";
        return false;
    }

    const std::string volumeLabel = volumeLabelForDevicePath(devicePath);
    const std::string formatLabel = volumeLabelForFormat(volumeLabel, normalizedFilesystemType);

    bool unmounted = runShellCommand(
        "udisksctl unmount -b " + shellQuote(devicePath) + " --no-user-interaction",
        errorMessage);
    if (!unmounted)
        unmounted = runShellCommand("umount " + shellQuote(mountPath), errorMessage);

    if (!unmounted) {
        if (errorMessage && errorMessage->find("Authentication is required") != std::string::npos)
            *errorMessage = "Formatting needs permission to unmount the card first.";
        else if (errorMessage && errorMessage->empty())
            *errorMessage = "The card could not be unmounted for formatting.";
        return false;
    }

    const std::string toolName = formatFilesystemToolName(normalizedFilesystemType);
    const std::string toolPath = formatFilesystemToolPath(normalizedFilesystemType);
    if (toolPath.empty()) {
        if (errorMessage)
            *errorMessage = toolName + " is not installed on the Pi.";
        return false;
    }

    std::string formatCommand;
    if (normalizedFilesystemType == "exfat") {
        formatCommand = "sudo -n " + shellQuote(toolPath) + " -n " + shellQuote(formatLabel)
            + " " + shellQuote(devicePath);
    } else if (normalizedFilesystemType == "vfat") {
        formatCommand = "sudo -n " + shellQuote(toolPath) + " -F 32 -n " + shellQuote(formatLabel)
            + " " + shellQuote(devicePath);
    } else if (normalizedFilesystemType == "ntfs") {
        formatCommand = "sudo -n " + shellQuote(toolPath) + " -f -F -L " + shellQuote(formatLabel)
            + " " + shellQuote(devicePath);
    }

    std::string formatError;
    if (formatCommand.empty() || !runShellCommand(formatCommand, &formatError)) {
        if (errorMessage) {
            if (formatError.find("password") != std::string::npos ||
                formatError.find("permission denied") != std::string::npos ||
                formatError.find("not allowed") != std::string::npos) {
                *errorMessage = "Formatting the card needs sudo permission.";
            } else if ((formatError.find("not found") != std::string::npos ||
                        formatError.find("Could not start") != std::string::npos) &&
                       !toolName.empty()) {
                *errorMessage = toolName + " is not installed on the Pi.";
            } else {
                *errorMessage = formatError.empty()
                    ? ("The card could not be formatted as " + formatFilesystemDisplayName(normalizedFilesystemType) + ".")
                    : formatError;
            }
        }
        return false;
    }

    runShellCommand("sync", nullptr);
    runShellCommand("udevadm settle", nullptr);

    bool remounted = false;
    for (int attempt = 0; attempt < 10 && !remounted; ++attempt) {
        if (errorMessage)
            errorMessage->clear();
        remounted = mountFormattedDeviceAtPath(devicePath, mountPath, normalizedFilesystemType, errorMessage);
        if (!remounted) {
            std::this_thread::sleep_for(std::chrono::milliseconds(350));
            runShellCommand("udevadm settle", nullptr);
        }
    }

    if (!remounted) {
        if (errorMessage &&
            (errorMessage->find("password") != std::string::npos ||
             errorMessage->find("permission denied") != std::string::npos ||
             errorMessage->find("not allowed") != std::string::npos)) {
            *errorMessage = "The card was formatted, but remounting it needs sudo permission.";
        } else if (errorMessage && errorMessage->empty()) {
            *errorMessage = "The card was formatted, but it could not be mounted again.";
        }
        return false;
    }

    return true;
}

std::string writeSpeedResultText(bool mediaMounted,
                                 bool running,
                                 const std::string &error,
                                 double lastWriteSpeedMBps)
{
    if (!mediaMounted)
        return "No Media";
    if (running)
        return "Testing...";
    if (!error.empty())
        return "Test Failed";
    if (lastWriteSpeedMBps > 0.0) {
        std::ostringstream out;
        out << std::fixed << std::setprecision(lastWriteSpeedMBps >= 100.0 ? 0 : 1)
            << lastWriteSpeedMBps << " MB/s";
        return out.str();
    }
    return "Not Tested";
}

std::string writeSpeedDetailText(bool mediaMounted,
                                 bool running,
                                 const std::string &error,
                                 uint64_t currentBytes,
                                 uint64_t lastBytes,
                                 bool currentUsedFio,
                                 bool lastUsedFio,
                                 double lastWriteSpeedMBps)
{
    if (!mediaMounted)
        return "Mount /media/RAW to run a media speed test.";

    const uint64_t bytes = running ? currentBytes : lastBytes;
    const uint64_t testMiB = bytes / (1024ull * 1024ull);

    if (running) {
        return currentUsedFio
            ? ("Running a " + std::to_string(testMiB) + " MB direct fio write test (QD32)...")
            : ("Running a " + std::to_string(testMiB) + " MB sequential write test...");
    }

    if (!error.empty())
        return error;

    if (lastWriteSpeedMBps > 0.0) {
        return lastUsedFio
            ? ("Measured with a " + std::to_string(testMiB) + " MB direct fio write test (QD32).")
            : ("Measured with a " + std::to_string(testMiB) + " MB sequential write test.");
    }

    return "Runs a temporary direct write test and removes the file afterwards.";
}

std::string formatBytesHuman(uint64_t bytes)
{
    static constexpr const char *kUnits[] = { "B", "KB", "MB", "GB", "TB" };
    double value = static_cast<double>(bytes);
    size_t unitIndex = 0;
    while (value >= 1024.0 && unitIndex < (sizeof(kUnits) / sizeof(kUnits[0])) - 1) {
        value /= 1024.0;
        ++unitIndex;
    }

    std::ostringstream out;
    const int precision = (value >= 100.0 || unitIndex == 0) ? 0 : 1;
    out << std::fixed << std::setprecision(precision) << value << ' ' << kUnits[unitIndex];
    return out.str();
}

std::string formatUptimeHuman(uint64_t uptimeSeconds)
{
    const uint64_t days = uptimeSeconds / 86400ull;
    const uint64_t hours = (uptimeSeconds % 86400ull) / 3600ull;
    const uint64_t minutes = (uptimeSeconds % 3600ull) / 60ull;

    std::ostringstream out;
    if (days > 0) {
        out << days << "d " << hours << "h " << minutes << "m";
        return out.str();
    }
    if (hours > 0) {
        out << hours << "h " << minutes << "m";
        return out.str();
    }
    out << minutes << "m";
    return out.str();
}

uint64_t readMemInfoValueKb(const std::string &key)
{
    std::ifstream input("/proc/meminfo");
    std::string name;
    uint64_t value = 0;
    std::string unit;
    while (input >> name >> value >> unit) {
        if (!name.empty() && name.back() == ':')
            name.pop_back();
        if (name == key)
            return value;
    }
    return 0;
}

void closeFdIfOpen(int *fd)
{
    if (!fd || *fd < 0)
        return;
    ::close(*fd);
    *fd = -1;
}

bool sendAllBytes(int fd, const void *data, size_t size)
{
    const auto *bytes = static_cast<const uint8_t *>(data);
    size_t remaining = size;
    while (remaining > 0) {
        const ssize_t written = ::send(fd, bytes, remaining, MSG_NOSIGNAL);
        if (written <= 0)
            return false;
        bytes += written;
        remaining -= static_cast<size_t>(written);
    }
    return true;
}

uint8_t clampByte(int value)
{
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

std::pair<uint32_t, uint32_t> scaledWebPreviewSize(uint32_t sourceWidth, uint32_t sourceHeight)
{
    if (sourceWidth == 0 || sourceHeight == 0)
        return {0, 0};

    const double widthScale = static_cast<double>(kWebPreviewMaxWidth) / static_cast<double>(sourceWidth);
    const double heightScale = static_cast<double>(kWebPreviewMaxHeight) / static_cast<double>(sourceHeight);
    const double scale = std::min({1.0, widthScale, heightScale});

    uint32_t width = static_cast<uint32_t>(std::lround(static_cast<double>(sourceWidth) * scale));
    uint32_t height = static_cast<uint32_t>(std::lround(static_cast<double>(sourceHeight) * scale));
    width = std::max(2u, width & ~1u);
    height = std::max(2u, height & ~1u);
    return {width, height};
}

void downsampleNearestPlane(const uint8_t *source,
                            uint32_t sourceWidth,
                            uint32_t sourceHeight,
                            uint32_t sourceStride,
                            uint8_t *target,
                            uint32_t targetWidth,
                            uint32_t targetHeight,
                            uint32_t targetStride)
{
    if (!source || !target || sourceWidth == 0 || sourceHeight == 0
        || targetWidth == 0 || targetHeight == 0) {
        return;
    }

    const bool sameWidth = sourceWidth == targetWidth;
    const bool halfWidth = sourceWidth == targetWidth * 2u;
    const uint64_t xStep = halfWidth || sameWidth
        ? 0
        : (static_cast<uint64_t>(sourceWidth) << 32u) / targetWidth;

    for (uint32_t y = 0; y < targetHeight; ++y) {
        const uint32_t sourceY = std::min(
            sourceHeight - 1u,
            static_cast<uint32_t>((static_cast<uint64_t>(y) * sourceHeight) / targetHeight));
        const uint8_t *sourceRow = source + static_cast<size_t>(sourceY) * sourceStride;
        uint8_t *targetRow = target + static_cast<size_t>(y) * targetStride;

        if (sameWidth) {
            std::memcpy(targetRow, sourceRow, targetWidth);
            continue;
        }

        if (halfWidth) {
            uint32_t x = 0;
#if defined(__aarch64__) || defined(__ARM_NEON)
            for (; x + 16u <= targetWidth; x += 16u) {
                const uint8x16x2_t pixels = vld2q_u8(sourceRow + static_cast<size_t>(x) * 2u);
                vst1q_u8(targetRow + x, pixels.val[0]);
            }
#endif
            for (; x < targetWidth; ++x)
                targetRow[x] = sourceRow[x * 2u];
            continue;
        }

        uint64_t sourceX = 0;
        for (uint32_t x = 0; x < targetWidth; ++x) {
            targetRow[x] = sourceRow[std::min(sourceWidth - 1u,
                                              static_cast<uint32_t>(sourceX >> 32u))];
            sourceX += xStep;
        }
    }
}

bool convertPlanarYuv420ToRgb(const uint8_t *yPlane,
                              uint32_t yStride,
                              const uint8_t *uPlane,
                              uint32_t uStride,
                              const uint8_t *vPlane,
                              uint32_t vStride,
                              uint32_t width,
                              uint32_t height,
                              std::vector<uint8_t> *rgb)
{
    if (!yPlane || !uPlane || !vPlane || width == 0 || height == 0 || !rgb)
        return false;

    rgb->resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 3u);
    const uint32_t chromaWidth = std::max(1u, width / 2);
    const uint32_t chromaHeight = std::max(1u, height / 2);

    for (uint32_t y = 0; y < height; ++y) {
        const uint32_t chromaY = std::min(chromaHeight - 1, y / 2);
        uint8_t *row = rgb->data() + (static_cast<size_t>(y) * static_cast<size_t>(width) * 3u);
        for (uint32_t x = 0; x < width; ++x) {
            const uint32_t chromaX = std::min(chromaWidth - 1, x / 2);
            const int yValue = static_cast<int>(yPlane[(static_cast<size_t>(y) * yStride) + x]);
            const int uValue = static_cast<int>(uPlane[(static_cast<size_t>(chromaY) * uStride) + chromaX]);
            const int vValue = static_cast<int>(vPlane[(static_cast<size_t>(chromaY) * vStride) + chromaX]);

            const int c = yValue - 16;
            const int d = uValue - 128;
            const int e = vValue - 128;
            row[(static_cast<size_t>(x) * 3u) + 0] = clampByte((298 * c + 409 * e + 128) >> 8);
            row[(static_cast<size_t>(x) * 3u) + 1] = clampByte((298 * c - 100 * d - 208 * e + 128) >> 8);
            row[(static_cast<size_t>(x) * 3u) + 2] = clampByte((298 * c + 516 * d + 128) >> 8);
        }
    }

    return true;
}

#if APERTARCORE_HAS_TURBOJPEG
struct TurboJpegCompressor {
    TurboJpegCompressor()
        : handle(tjInitCompress())
    {
    }

    ~TurboJpegCompressor()
    {
        if (handle)
            tjDestroy(handle);
    }

    tjhandle handle = nullptr;
};

bool encodeYuv420ToJpeg(const uint8_t *yuv,
                        uint32_t width,
                        uint32_t height,
                        const uint32_t planeOffsets[3],
                        const uint32_t planeStrides[3],
                        int quality,
                        std::vector<uint8_t> *outJpeg)
{
    if (!yuv || width == 0 || height == 0 || !planeOffsets || !planeStrides || !outJpeg)
        return false;

    thread_local TurboJpegCompressor compressor;
    if (!compressor.handle)
        return false;

    const unsigned long outputBound = tjBufSize(static_cast<int>(width),
                                                static_cast<int>(height),
                                                TJSAMP_420);
    if (outputBound == 0 || outputBound == std::numeric_limits<unsigned long>::max())
        return false;

    outJpeg->resize(static_cast<size_t>(outputBound));
    unsigned char *compressed = outJpeg->data();
    unsigned long compressedSize = outputBound;
    const unsigned char *planes[3] = {
        yuv + planeOffsets[0],
        yuv + planeOffsets[1],
        yuv + planeOffsets[2],
    };
    const int strides[3] = {
        static_cast<int>(planeStrides[0]),
        static_cast<int>(planeStrides[1]),
        static_cast<int>(planeStrides[2]),
    };
    const int rc = tjCompressFromYUVPlanes(compressor.handle,
                                           planes,
                                           static_cast<int>(width),
                                           strides,
                                           static_cast<int>(height),
                                           TJSAMP_420,
                                           &compressed,
                                           &compressedSize,
                                           quality,
                                           TJFLAG_FASTDCT | TJFLAG_NOREALLOC);
    const bool ok = (rc == 0 && compressed && compressedSize > 0);
    if (ok)
        outJpeg->resize(static_cast<size_t>(compressedSize));
    else
        outJpeg->clear();
    return ok;
}
#elif APERTARCORE_HAS_JPEG
bool encodeYuv420ToJpeg(const uint8_t *yuv,
                        uint32_t width,
                        uint32_t height,
                        const uint32_t planeOffsets[3],
                        const uint32_t planeStrides[3],
                        int quality,
                        std::vector<uint8_t> *outJpeg)
{
    if (!yuv || width == 0 || height == 0 || !planeOffsets || !planeStrides || !outJpeg)
        return false;

    std::vector<uint8_t> rgb;
    if (!convertPlanarYuv420ToRgb(yuv + planeOffsets[0],
                                  planeStrides[0],
                                  yuv + planeOffsets[1],
                                  planeStrides[1],
                                  yuv + planeOffsets[2],
                                  planeStrides[2],
                                  width,
                                  height,
                                  &rgb)) {
        return false;
    }

    jpeg_compress_struct cinfo{};
    jpeg_error_mgr jerr{};
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    unsigned char *compressed = nullptr;
    unsigned long compressedSize = 0;
    jpeg_mem_dest(&cinfo, &compressed, &compressedSize);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    const size_t rowStride = static_cast<size_t>(width) * 3;
    while (cinfo.next_scanline < cinfo.image_height) {
        JSAMPROW row = rgb.data() + (static_cast<size_t>(cinfo.next_scanline) * rowStride);
        jpeg_write_scanlines(&cinfo, &row, 1);
    }

    jpeg_finish_compress(&cinfo);
    outJpeg->assign(compressed, compressed + compressedSize);
    jpeg_destroy_compress(&cinfo);
    std::free(compressed);
    return true;
}
#endif

struct PowerSupplySnapshot
{
    std::optional<double> powerW;
    std::optional<double> voltageV;
    std::optional<double> currentA;
    std::optional<double> capacityPercent;
};

struct Ina219HardwareConfig
{
    bool enabled = true;
    int busNumber = 4;
    int address = 0x40;
};

std::filesystem::path defaultHardwareConfigPath()
{
    return "/home/pi/apertar-hardware.conf";
}

std::vector<std::filesystem::path> hardwareConfigCandidatePaths()
{
    std::vector<std::filesystem::path> candidates;
    if (const char *envPath = std::getenv("APERTAR_HARDWARE_CONFIG"); envPath && *envPath)
        candidates.emplace_back(envPath);

    candidates.push_back(defaultHardwareConfigPath());
    candidates.emplace_back("/boot/firmware/apertar-hardware.conf");
    candidates.emplace_back("/boot/apertar-hardware.conf");

    std::vector<std::filesystem::path> unique;
    for (const auto &path : candidates) {
        if (path.empty())
            continue;
        if (std::find(unique.begin(), unique.end(), path) == unique.end())
            unique.push_back(path);
    }
    return unique;
}

std::filesystem::path editableHardwareConfigPath()
{
    if (const char *envPath = std::getenv("APERTAR_HARDWARE_CONFIG"); envPath && *envPath)
        return std::filesystem::path(envPath);
    return defaultHardwareConfigPath();
}

std::string hardwareConfigDisplayPath(const std::filesystem::path &path)
{
    const std::string value = path.string();
    if (const char *home = std::getenv("HOME"); home && *home) {
        const std::string homeText(home);
        if (value == homeText)
            return "~";
        if (value.rfind(homeText + "/", 0) == 0)
            return "~/" + value.substr(homeText.size() + 1);
    }
    return value;
}

std::string defaultHardwareConfigTemplate()
{
    return R"CONFIG(# Apertar hardware configuration
# Primary location: /home/pi/apertar-hardware.conf
# INA219 I2C bus example: /dev/i2c-4 -> bus=4
# Address can be decimal or hex.

[ina219]
enabled=true
bus=4
address=0x40

[cfexpress_hat]
enabled=true
bus=10
address=0x34

[storage]
mount_path=/media/RAW
pcie_controller=1000110000.pcie

[display]
# Resolutions: 720x720(Default), 640x480, 800x480, 1280x720, 1024x600, 1920x1080
# Layouts:
# square (For 720x720)
# landscape_compact (For 640x480 and 800x480)
# landscape_medium (For 1024x600)
# landscape (For 1280x720)
# landscape_large (For 1920x1080)
ui_width=720
ui_height=720
ui_layout=square
controls_opacity=1.0
# Zoom button magnification: 1.0-4.0; restart the UI after changing this.
zoom_toggle_factor=4.0
controls_mode=Light
rotation=0
windowed=false
hdmi_external_only=false

[run_script]
# run_script (alias script) runs all configured scripts sequentially, up to five.
# run_script1..run_script5 (aliases script1..script5) select individual slots.
# Group slots: P2=run_script1,run_script3. Runs as pi, without automatic sudo.
# Absolute paths; path is script 1 (path1 is an alias). Leave unused slots empty.
# Duplicate pending/running slots are ignored. Shutdown cancels remaining jobs.
path=
path2=
path3=
path4=
path5=

[imu]
enabled=false
driver=mpu6050
bus=10
address=0x68
sample_rate_hz=500
gyro_range_dps=2000
accel_range_g=8
orientation=XYZ
sync_offset_ms=0

[pcf8574]
# Pin actions include capture, record_toggle, iso_up/down, fps_up/down, wb_up/down,
# exact shortcuts like iso200, wb5000, fps24, resolution4056x2280,
# EV actions: ev0, ev1, ev-1, ev_up, ev_down (1/3 EV steps, -3..+3).
# These adjust auto exposure without enabling Auto ISO/shutter.
# and recindicatorled for one recording LED output pin.
# standby (alias standby_toggle) sleeps on press; a button press wakes only.
# Example: P2=standby or encoder0_press=standby. Put standby last in a group.
enabled=false
bus=10
address=0x20

[gpio]
# GPIO button actions use the same names as [pcf8574].
# Example: d17=recindicatorled reserves GPIO17 as the recording LED output.
enabled=false

[trigger_output]
# Optional capture trigger pulse for DIY hot shoe, flash sync, tally, or accessories.
# target=gpio uses a Raspberry Pi GPIO line. target=pcf8574 uses a PCF8574 pin.
# For hot-shoe style trigger circuits use an optocoupler/transistor; do not connect flash trigger voltage directly to a Pi GPIO.
enabled=false
target=gpio
events=recording_start,still,timelapse_frame
pulse_ms=80
active_high=false
gpiochip=auto
line=17
open_drain=true
# For target=pcf8574, bus/address default to the [pcf8574] section unless overridden here.
# bus=1
# address=0x20
pcf_pin=0

[recording_indicator]
# Optional recording tally LED. You can also set pinX=recindicatorled in [pcf8574]
# or dX=recindicatorled in [gpio]/[gpio_buttons].
# Default active_low wiring: 3.3V -> resistor -> LED -> output pin.
enabled=false
target=gpio
gpiochip=auto
line=17
active_high=false
open_drain=false
# For target=pcf8574, bus/address default to [pcf8574] unless overridden here.
# bus=10
# address=0x20
pcf_pin=7

)CONFIG";
}

std::optional<int> parseConfigInteger(const std::string &text)
{
    const std::string trimmed = trimAscii(text);
    if (trimmed.empty())
        return std::nullopt;

    char *end = nullptr;
    errno = 0;
    const long value = std::strtol(trimmed.c_str(), &end, 0);
    if (errno != 0 || end == trimmed.c_str())
        return std::nullopt;

    while (*end != '\0') {
        if (!std::isspace(static_cast<unsigned char>(*end)))
            return std::nullopt;
        ++end;
    }

    return static_cast<int>(value);
}

bool parseConfigBool(const std::string &text, bool fallback)
{
    std::string value = text;
    const std::size_t hash = value.find('#');
    const std::size_t semi = value.find(';');
    std::size_t comment = std::string::npos;
    if (hash != std::string::npos)
        comment = hash;
    if (semi != std::string::npos)
        comment = comment == std::string::npos ? semi : std::min(comment, semi);
    if (comment != std::string::npos)
        value.resize(comment);

    const std::string normalized = lowerAscii(trimAscii(value));
    if (normalized == "1" || normalized == "true" || normalized == "yes" ||
        normalized == "on" || normalized == "enabled")
        return true;
    if (normalized == "0" || normalized == "false" || normalized == "no" ||
        normalized == "off" || normalized == "disabled")
        return false;
    return fallback;
}

Ina219HardwareConfig loadIna219HardwareConfig()
{
    Ina219HardwareConfig config;
    std::error_code ec;

    for (const auto &path : hardwareConfigCandidatePaths()) {
        if (!std::filesystem::exists(path, ec) || !std::filesystem::is_regular_file(path, ec))
            continue;

        std::ifstream input(path);
        if (!input.is_open())
            continue;

        bool inIna219Section = false;
        std::string line;
        while (std::getline(input, line)) {
            std::string trimmed = trimAscii(line);
            if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';')
                continue;

            if (trimmed.front() == '[' && trimmed.back() == ']') {
                inIna219Section = lowerAscii(trimmed) == "[ina219]";
                continue;
            }

            if (!inIna219Section)
                continue;

            const std::size_t eq = trimmed.find('=');
            if (eq == std::string::npos)
                continue;

            const std::string key = lowerAscii(trimAscii(trimmed.substr(0, eq)));
            const std::string value = trimmed.substr(eq + 1);
            if (key == "enabled") {
                config.enabled = parseConfigBool(value, config.enabled);
                continue;
            }

            const auto parsed = parseConfigInteger(value);
            if (!parsed.has_value())
                continue;

            if (key == "bus" || key == "channel" || key == "i2c_bus") {
                if (*parsed >= 0 && *parsed <= 255)
                    config.busNumber = *parsed;
            } else if (key == "address" || key == "addr") {
                if (*parsed >= 0x03 && *parsed <= 0x77)
                    config.address = *parsed;
            }
        }

        return config;
    }

    return config;
}

constexpr uint8_t kIna219RegConfig = 0x00;
constexpr uint8_t kIna219RegShuntVoltage = 0x01;
constexpr uint8_t kIna219RegBusVoltage = 0x02;
constexpr uint8_t kIna219RegPower = 0x03;
constexpr uint8_t kIna219RegCurrent = 0x04;
constexpr uint8_t kIna219RegCalibration = 0x05;

constexpr uint16_t kIna219Config32V3A = 0x399F;
constexpr uint16_t kIna219Calibration32V3A = 4096;

constexpr double kIna219CurrentLsbAmps = 0.0001;
constexpr double kIna219PowerLsbWatts = 0.002;
constexpr double kIna219ShuntMillivoltsPerBit = 0.01;
constexpr double kIna219ShuntOhms = 0.1;
constexpr double kIna219VoltageCalibrationOffset = 0.08;
constexpr double kIna219EstimatedPackSagOhms = 0.26;

bool writeIna219Register(int fd, uint8_t reg, uint16_t value)
{
    const char buffer[3] = {
        static_cast<char>(reg),
        static_cast<char>((value >> 8) & 0xff),
        static_cast<char>(value & 0xff)
    };

    return ::write(fd, buffer, sizeof(buffer)) == sizeof(buffer);
}

bool readIna219Register(int fd, uint8_t reg, uint16_t *value)
{
    if (!value)
        return false;

    const char regByte = static_cast<char>(reg);
    if (::write(fd, &regByte, 1) != 1)
        return false;

    unsigned char data[2] = { 0, 0 };
    if (::read(fd, data, sizeof(data)) != static_cast<ssize_t>(sizeof(data)))
        return false;

    *value = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
    return true;
}

double estimateIna219BatteryPercent(double voltageV, double currentA)
{
    struct Sample {
        double voltage;
        int percent;
    };

    const double compensatedVoltage = std::clamp(
        voltageV + (std::max(0.0, currentA) * kIna219EstimatedPackSagOhms),
        13.2,
        16.8);

    static const std::array<Sample, 14> curve{{
        {13.2, 0},
        {14.0, 5},
        {14.3, 10},
        {14.6, 20},
        {14.8, 32},
        {15.0, 42},
        {15.2, 52},
        {15.4, 60},
        {15.6, 68},
        {15.8, 76},
        {16.0, 84},
        {16.2, 91},
        {16.4, 95},
        {16.8, 100}
    }};

    if (compensatedVoltage <= curve.front().voltage)
        return curve.front().percent;
    if (compensatedVoltage >= curve.back().voltage)
        return curve.back().percent;

    for (std::size_t i = 1; i < curve.size(); ++i) {
        if (compensatedVoltage <= curve[i].voltage) {
            const double x0 = curve[i - 1].voltage;
            const double x1 = curve[i].voltage;
            const double t = (compensatedVoltage - x0) / (x1 - x0);
            const double interpolated = curve[i - 1].percent + t * (curve[i].percent - curve[i - 1].percent);
            return std::clamp(std::round(interpolated), 0.0, 100.0);
        }
    }

    return 0.0;
}

PowerSupplySnapshot readIna219Snapshot()
{
    static const Ina219HardwareConfig config = loadIna219HardwareConfig();

    PowerSupplySnapshot snapshot;
    if (!config.enabled)
        return snapshot;

    const std::filesystem::path devicePath = std::filesystem::path("/dev") / ("i2c-" + std::to_string(config.busNumber));
    const int fd = ::open(devicePath.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0)
        return snapshot;

    auto closeFd = [&fd]() {
        ::close(fd);
    };

    if (::ioctl(fd, I2C_SLAVE, config.address) < 0) {
        closeFd();
        return snapshot;
    }

    if (!writeIna219Register(fd, kIna219RegConfig, kIna219Config32V3A) ||
        !writeIna219Register(fd, kIna219RegCalibration, kIna219Calibration32V3A)) {
        closeFd();
        return snapshot;
    }

    uint16_t busRaw = 0;
    uint16_t shuntRaw = 0;
    uint16_t currentRaw = 0;
    uint16_t powerRaw = 0;
    if (!readIna219Register(fd, kIna219RegBusVoltage, &busRaw) ||
        !readIna219Register(fd, kIna219RegShuntVoltage, &shuntRaw) ||
        !readIna219Register(fd, kIna219RegCurrent, &currentRaw) ||
        !readIna219Register(fd, kIna219RegPower, &powerRaw)) {
        closeFd();
        return snapshot;
    }

    closeFd();

    const double busVoltage = ((busRaw >> 3) * 4.0) / 1000.0;
    const auto signedShunt = static_cast<int16_t>(shuntRaw);
    const auto signedCurrent = static_cast<int16_t>(currentRaw);
    const double shuntMillivolts = signedShunt * kIna219ShuntMillivoltsPerBit;
    const double sourceVoltage = busVoltage + (shuntMillivolts / 1000.0);
    const double calibratedSourceVoltage = sourceVoltage + kIna219VoltageCalibrationOffset;

    double currentAmps = signedCurrent * kIna219CurrentLsbAmps;
    if (std::abs(currentAmps) < 1e-9 && std::abs(shuntMillivolts) > 1e-9)
        currentAmps = (shuntMillivolts / 1000.0) / kIna219ShuntOhms;

    double powerWatts = powerRaw * kIna219PowerLsbWatts;
    if (powerWatts <= 0.0 && calibratedSourceVoltage > 0.0 && currentAmps > 0.0)
        powerWatts = calibratedSourceVoltage * currentAmps;

    snapshot.voltageV = calibratedSourceVoltage;
    snapshot.currentA = currentAmps;
    snapshot.powerW = powerWatts;
    snapshot.capacityPercent = estimateIna219BatteryPercent(calibratedSourceVoltage, currentAmps);
    return snapshot;
}

PowerSupplySnapshot readPowerSupplySnapshot()
{
    PowerSupplySnapshot snapshot = readIna219Snapshot();
    if (snapshot.voltageV.has_value() || snapshot.powerW.has_value() ||
        snapshot.currentA.has_value() || snapshot.capacityPercent.has_value()) {
        return snapshot;
    }

    const std::filesystem::path base("/sys/class/power_supply");
    std::error_code ec;
    if (!std::filesystem::exists(base, ec))
        return snapshot;

    for (const auto &entry : std::filesystem::directory_iterator(base, ec)) {
        if (ec || !entry.is_directory())
            continue;

        const std::filesystem::path dir = entry.path();
        if (!snapshot.capacityPercent) {
            if (const auto capacity = readLongLongFile(dir / "capacity"))
                snapshot.capacityPercent = std::clamp(static_cast<double>(*capacity), 0.0, 100.0);
        }

        if (!snapshot.voltageV) {
            if (const auto voltageNow = readLongLongFile(dir / "voltage_now"))
                snapshot.voltageV = static_cast<double>(*voltageNow) / 1'000'000.0;
        }

        if (!snapshot.powerW) {
            if (const auto powerNow = readLongLongFile(dir / "power_now")) {
                snapshot.powerW = std::abs(static_cast<double>(*powerNow) / 1'000'000.0);
            } else {
                const auto currentNow = readLongLongFile(dir / "current_now");
                const auto voltageNow = readLongLongFile(dir / "voltage_now");
                if (!snapshot.currentA && currentNow)
                    snapshot.currentA = std::abs(static_cast<double>(*currentNow) / 1'000'000.0);
                if (currentNow && voltageNow) {
                    snapshot.powerW = std::abs((static_cast<double>(*currentNow) / 1'000'000.0) *
                                               (static_cast<double>(*voltageNow) / 1'000'000.0));
                }
            }
        }

        if (!snapshot.currentA) {
            if (const auto currentNow = readLongLongFile(dir / "current_now"))
                snapshot.currentA = std::abs(static_cast<double>(*currentNow) / 1'000'000.0);
        }
    }

    return snapshot;
}

std::optional<double> readCpuTemperatureC()
{
    const auto milliDegrees = readLongLongFile("/sys/class/thermal/thermal_zone0/temp");
    if (!milliDegrees)
        return std::nullopt;
    return static_cast<double>(*milliDegrees) / 1000.0;
}

std::optional<std::pair<uint64_t, uint64_t>> readCpuTicks()
{
    std::ifstream input("/proc/stat");
    std::string label;
    uint64_t user = 0;
    uint64_t nice = 0;
    uint64_t system = 0;
    uint64_t idle = 0;
    uint64_t iowait = 0;
    uint64_t irq = 0;
    uint64_t softirq = 0;
    uint64_t steal = 0;
    if (!(input >> label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal))
        return std::nullopt;
    const uint64_t idleTicks = idle + iowait;
    const uint64_t totalTicks = user + nice + system + idle + iowait + irq + softirq + steal;
    return std::make_pair(idleTicks, totalTicks);
}

uint64_t estimatedRawBytesPerSecond(const CaptureSettings &settings)
{
    if (settings.recordingFormat == RecordingFormat::Prores) {
        const double nominal = settings.proresProfile == ProresProfile::Proxy ? 45e6
            : settings.proresProfile == ProresProfile::LT ? 102e6
            : settings.proresProfile == ProresProfile::HQ ? 220e6 : 147e6;
        return static_cast<uint64_t>(nominal / 8.0 * settings.resolution.width * settings.resolution.height
            / (1920.0 * 1080.0) * settings.fps / 30.0 * 1.05);
    }
    if (settings.recordingFormat == RecordingFormat::H264) {
        constexpr uint64_t kExperimentalH264BytesPerSecond = 25'000'000u / 8u;
        return kExperimentalH264BytesPerSecond;
    }

    const double pixelsPerFrame =
        static_cast<double>(settings.resolution.width) * static_cast<double>(settings.resolution.height);
    const double bytesPerFrame = pixelsPerFrame * 1.5;
    const double containerOverhead = settings.recordingFormat == RecordingFormat::Cdng ? 1.08 : 1.03;
    const double bytesPerSecond = bytesPerFrame * std::max(1.0, settings.fps) * containerOverhead;
    return static_cast<uint64_t>(std::max(0.0, bytesPerSecond));
}

double estimatedStillFrameSizeMiB(const CaptureSettings &settings, const std::string &sensorName)
{
    const int width = settings.resolution.width;
    const int height = settings.resolution.height;
    const std::string sensor = lowerAscii(sensorName);
    const bool imx477 = sensor.find("imx477") != std::string::npos;
    const bool imx577 = sensor.find("imx577") != std::string::npos;
    const bool imx294 = sensor.find("imx294") != std::string::npos;
    const bool imx585 = sensor.find("imx585") != std::string::npos;

    auto bitsPerPixelForEstimate = [&settings]() {
        if (settings.rawBitDepth == 8)
            return 8.0;
        if (settings.rawBitDepth == 10)
            return 10.0;
        return 12.0;
    };

    if (imx294) {
        const double bitsPerPixel = bitsPerPixelForEstimate();
        const double bytes =
            static_cast<double>(std::max(1, width)) *
            static_cast<double>(std::max(1, height)) *
            bitsPerPixel / 8.0;
        return std::max(1.0, (bytes * 1.08) / (1024.0 * 1024.0));
    }

    if (imx577) {
        const double bytes =
            static_cast<double>(std::max(1, width)) *
            static_cast<double>(std::max(1, height)) *
            bitsPerPixelForEstimate() / 8.0;
        return std::max(1.0, (bytes * 1.08) / (1024.0 * 1024.0));
    }

    if (imx477) {
        const double bitsPerPixel = bitsPerPixelForEstimate();
        const double bytes =
            static_cast<double>(std::max(1, width)) *
            static_cast<double>(std::max(1, height)) *
            bitsPerPixel / 8.0;
        return std::max(1.0, (bytes * 1.08) / (1024.0 * 1024.0));
    }

    if (imx585) {
        if ((width == 3856 && height == 2180) ||
            (width == 3840 && height == 2160) ||
            (width == 4056 && height == 2160) ||
            (width == 4056 && height == 3040))
            return 12.4;
        if ((width == 1928 && height == 1090) ||
            (width == 1920 && height == 1080) ||
            (width == 1332 && height == 990) ||
            (width == 2028 && height == 1080) ||
            (width == 2028 && height == 1520))
            return 3.1;
    }

    if (width == 1332 && height == 990)
        return 2.0;
    if (width == 2028 && height == 1080)
        return 3.3;
    if (width == 2028 && height == 1520)
        return 4.6;
    if ((width == 3856 && height == 2180) || (width == 3840 && height == 2160))
        return 12.4;
    if ((width == 1928 && height == 1090) || (width == 1920 && height == 1080))
        return 3.1;

    return 5.3;
}

struct NetworkSnapshot
{
    std::string transport = "Wi-Fi / Ethernet";
    std::string hostLabel = "localhost";
    std::string captureLabel = "Ethernet / Wi-Fi";
    std::string ipAddress = "Unavailable";
};

NetworkSnapshot readNetworkSnapshot(uint16_t port)
{
    NetworkSnapshot snapshot;
    ifaddrs *addresses = nullptr;
    if (::getifaddrs(&addresses) != 0)
        return snapshot;

    bool hasWifi = false;
    bool hasEthernet = false;
    std::string firstAddress;

    for (ifaddrs *entry = addresses; entry; entry = entry->ifa_next) {
        if (!entry->ifa_addr || entry->ifa_addr->sa_family != AF_INET)
            continue;
        if (entry->ifa_flags & IFF_LOOPBACK)
            continue;

        char host[INET_ADDRSTRLEN]{};
        const auto *addr = reinterpret_cast<sockaddr_in *>(entry->ifa_addr);
        if (!::inet_ntop(AF_INET, &addr->sin_addr, host, sizeof(host)))
            continue;

        const std::string name = entry->ifa_name ? entry->ifa_name : "";
        if (name.rfind("wl", 0) == 0 || name.rfind("wlan", 0) == 0)
            hasWifi = true;
        if (name.rfind("en", 0) == 0 || name.rfind("eth", 0) == 0)
            hasEthernet = true;

        if (firstAddress.empty())
            firstAddress = host;
    }

    ::freeifaddrs(addresses);

    if (hasWifi && hasEthernet) {
        snapshot.transport = "Wi-Fi / Ethernet";
        snapshot.captureLabel = "Ethernet / Wi-Fi";
    } else if (hasWifi) {
        snapshot.transport = "Wi-Fi";
        snapshot.captureLabel = "Wi-Fi";
    } else if (hasEthernet) {
        snapshot.transport = "Ethernet";
        snapshot.captureLabel = "Ethernet";
    } else {
        snapshot.transport = "Local Network";
        snapshot.captureLabel = "Local Network";
    }

    if (!firstAddress.empty()) {
        snapshot.ipAddress = firstAddress;
        snapshot.hostLabel = firstAddress;
        if (port != 0 && port != 80)
            snapshot.hostLabel += ":" + std::to_string(port);
    }

    return snapshot;
}

struct DeviceSnapshot
{
    std::string piModel = "Raspberry Pi";
    std::string serialNumber = "Unavailable";
    std::string osVersion = "Unavailable";
    std::string kernelVersion = "Unavailable";
    std::string uptimeText = "Unavailable";
    std::string hostname = "Unavailable";
    std::string ramTotalText = "Unavailable";
    std::string ramAvailableText = "Unavailable";
    std::string ramUsedText = "Unavailable";
    std::string systemStorageTotalText = "Unavailable";
    std::string systemStorageFreeText = "Unavailable";
    std::string systemStorageMountPoint = "/";
};

std::string readOsPrettyName()
{
    std::ifstream input("/etc/os-release");
    std::string line;
    while (std::getline(input, line)) {
        if (line.rfind("PRETTY_NAME=", 0) != 0)
            continue;
        std::string value = line.substr(std::string("PRETTY_NAME=").size());
        value = trimAscii(value);
        if (!value.empty() && value.front() == '"' && value.back() == '"' && value.size() >= 2)
            value = value.substr(1, value.size() - 2);
        return value;
    }
    return {};
}

DeviceSnapshot readDeviceSnapshot()
{
    DeviceSnapshot snapshot;

    const std::string model = readTextFileTrimmed("/proc/device-tree/model");
    if (!model.empty())
        snapshot.piModel = model;

    std::string serial = readTextFileTrimmed("/proc/device-tree/serial-number");
    if (serial.empty()) {
        std::ifstream cpuInfo("/proc/cpuinfo");
        std::string line;
        while (std::getline(cpuInfo, line)) {
            const std::string trimmed = trimAscii(line);
            if (trimmed.rfind("Serial", 0) != 0 && trimmed.rfind("serial", 0) != 0)
                continue;
            const size_t colon = line.find(':');
            if (colon != std::string::npos) {
                serial = trimAscii(line.substr(colon + 1));
                break;
            }
        }
    }
    if (!serial.empty())
        snapshot.serialNumber = serial;

    const std::string osVersion = readOsPrettyName();
    if (!osVersion.empty())
        snapshot.osVersion = osVersion;

    const std::string kernelVersion = readTextFileTrimmed("/proc/sys/kernel/osrelease");
    if (!kernelVersion.empty())
        snapshot.kernelVersion = kernelVersion;

    const std::string hostname = readTextFileTrimmed("/proc/sys/kernel/hostname");
    if (!hostname.empty())
        snapshot.hostname = hostname;

    std::ifstream uptimeFile("/proc/uptime");
    double uptimeSeconds = 0.0;
    if (uptimeFile >> uptimeSeconds)
        snapshot.uptimeText = formatUptimeHuman(static_cast<uint64_t>(std::max(0.0, uptimeSeconds)));

    const uint64_t memTotalBytes = readMemInfoValueKb("MemTotal") * 1024ull;
    const uint64_t memAvailableBytes = readMemInfoValueKb("MemAvailable") * 1024ull;
    const uint64_t memFreeBytes = readMemInfoValueKb("MemFree") * 1024ull;
    const uint64_t buffersBytes = readMemInfoValueKb("Buffers") * 1024ull;
    const uint64_t cachedBytes = readMemInfoValueKb("Cached") * 1024ull;
    const uint64_t reclaimableBytes = readMemInfoValueKb("SReclaimable") * 1024ull;
    const uint64_t shmemBytes = readMemInfoValueKb("Shmem") * 1024ull;
    uint64_t memUsedBytes = 0;
    if (memTotalBytes > 0 && memFreeBytes > 0) {
        const uint64_t reclaimableTotal = std::min<uint64_t>(
            memTotalBytes,
            memFreeBytes + buffersBytes + cachedBytes + reclaimableBytes);
        memUsedBytes = std::min<uint64_t>(
            memTotalBytes,
            memTotalBytes - reclaimableTotal + std::min<uint64_t>(memTotalBytes, shmemBytes));
    } else if (memTotalBytes > memAvailableBytes) {
        memUsedBytes = memTotalBytes - memAvailableBytes;
    }

    if (memTotalBytes > 0)
        snapshot.ramTotalText = formatBytesHuman(memTotalBytes);
    if (memAvailableBytes > 0)
        snapshot.ramAvailableText = formatBytesHuman(memAvailableBytes);
    if (memUsedBytes > 0)
        snapshot.ramUsedText = formatBytesHuman(memUsedBytes);

    std::error_code spaceEc;
    const auto rootSpace = std::filesystem::space("/", spaceEc);
    if (!spaceEc) {
        snapshot.systemStorageTotalText = formatBytesHuman(rootSpace.capacity);
        snapshot.systemStorageFreeText = formatBytesHuman(rootSpace.available);
        snapshot.systemStorageMountPoint = "/";
    }

    return snapshot;
}

std::string parseJsonStringLiteralValue(const std::string &text, size_t *pos)
{
    if (!pos || *pos >= text.size() || text[*pos] != '"')
        return {};

    ++(*pos);
    std::string value;
    while (*pos < text.size()) {
        const char ch = text[(*pos)++];
        if (ch == '"')
            return value;
        if (ch == '\\') {
            if (*pos >= text.size())
                return {};
            const char escaped = text[(*pos)++];
            switch (escaped) {
            case '"': value.push_back('"'); break;
            case '\\': value.push_back('\\'); break;
            case '/': value.push_back('/'); break;
            case 'b': value.push_back('\b'); break;
            case 'f': value.push_back('\f'); break;
            case 'n': value.push_back('\n'); break;
            case 'r': value.push_back('\r'); break;
            case 't': value.push_back('\t'); break;
            default: value.push_back(escaped); break;
            }
        } else {
            value.push_back(ch);
        }
    }

    return {};
}

std::optional<std::string> extractJsonStringField(const std::string &text, const std::string &fieldName)
{
    const std::string needle = "\"" + fieldName + "\"";
    size_t pos = 0;
    while (true) {
        pos = text.find(needle, pos);
        if (pos == std::string::npos)
            return std::nullopt;

        size_t cursor = pos + needle.size();
        while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor])))
            ++cursor;
        if (cursor >= text.size() || text[cursor] != ':') {
            pos = cursor;
            continue;
        }
        ++cursor;
        while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor])))
            ++cursor;
        if (cursor >= text.size() || text[cursor] != '"') {
            pos = cursor;
            continue;
        }

        const std::string value = parseJsonStringLiteralValue(text, &cursor);
        if (!value.empty() || (cursor > 0 && text[cursor - 1] == '"'))
            return value;
        return std::nullopt;
    }
}

std::string urlEncodeComponent(const std::string &value)
{
    std::ostringstream out;
    out << std::uppercase << std::hex;
    for (const unsigned char ch : value) {
        if ((ch >= 'A' && ch <= 'Z') ||
            (ch >= 'a' && ch <= 'z') ||
            (ch >= '0' && ch <= '9') ||
            ch == '-' || ch == '_' || ch == '.' || ch == '~') {
            out << static_cast<char>(ch);
        } else {
            out << '%' << std::setw(2) << std::setfill('0') << static_cast<int>(ch);
        }
    }
    return out.str();
}

std::string urlDecodeComponent(const std::string &value)
{
    std::string decoded;
    decoded.reserve(value.size());
    for (size_t i = 0; i < value.size(); ++i) {
        const char ch = value[i];
        if (ch == '%' && i + 2 < value.size()) {
            const auto hexValue = value.substr(i + 1, 2);
            char *end = nullptr;
            const long parsed = std::strtol(hexValue.c_str(), &end, 16);
            if (end && *end == '\0') {
                decoded.push_back(static_cast<char>(parsed));
                i += 2;
                continue;
            }
        }
        decoded.push_back(ch == '+' ? ' ' : ch);
    }
    return decoded;
}

std::optional<std::string> queryParameterValue(const std::string &query, const std::string &key)
{
    size_t start = 0;
    while (start <= query.size()) {
        const size_t end = query.find('&', start);
        const std::string part = query.substr(start, end == std::string::npos ? std::string::npos : end - start);
        const size_t eq = part.find('=');
        const std::string rawKey = eq == std::string::npos ? part : part.substr(0, eq);
        if (urlDecodeComponent(rawKey) == key) {
            const std::string rawValue = eq == std::string::npos ? std::string() : part.substr(eq + 1);
            return urlDecodeComponent(rawValue);
        }
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return std::nullopt;
}

std::vector<std::string> queryParameterValues(const std::string &query, const std::string &key)
{
    std::vector<std::string> values;
    size_t start = 0;
    while (start <= query.size()) {
        const size_t end = query.find('&', start);
        const std::string part = query.substr(start, end == std::string::npos ? std::string::npos : end - start);
        const size_t eq = part.find('=');
        const std::string rawKey = eq == std::string::npos ? part : part.substr(0, eq);
        if (urlDecodeComponent(rawKey) == key) {
            const std::string rawValue = eq == std::string::npos ? std::string() : part.substr(eq + 1);
            values.push_back(urlDecodeComponent(rawValue));
        }
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return values;
}

bool isSafePresetFileName(const std::string &fileName)
{
    return !fileName.empty() &&
           fileName.find('/') == std::string::npos &&
           fileName.find('\\') == std::string::npos &&
           fileName.find("..") == std::string::npos &&
           fileName.size() >= std::strlen(".apertar-preset.json") &&
           fileName.rfind(".apertar-preset.json") == fileName.size() - std::strlen(".apertar-preset.json");
}

std::string toLowerAsciiCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

std::string stripPresetFileSuffix(std::string fileName)
{
    static const std::string suffix = ".apertar-preset.json";
    if (fileName.size() >= suffix.size() &&
        fileName.rfind(suffix) == fileName.size() - suffix.size()) {
        fileName.resize(fileName.size() - suffix.size());
    }
    return fileName;
}

std::string sanitizePresetFileStem(std::string name)
{
    name = trimAscii(name);
    if (name.empty())
        name = "Preset";

    static const std::string invalidCharacters = "\\/:*?\"<>|";
    for (char &ch : name) {
        if (invalidCharacters.find(ch) != std::string::npos || ch == '\n' || ch == '\r')
            ch = '_';
    }

    std::string simplified;
    simplified.reserve(name.size());
    bool previousSpace = false;
    for (const unsigned char ch : name) {
        if (std::isspace(ch)) {
            if (!previousSpace) {
                simplified.push_back(' ');
                previousSpace = true;
            }
        } else {
            simplified.push_back(static_cast<char>(ch));
            previousSpace = false;
        }
    }

    simplified = trimAscii(simplified);
    return simplified.empty() ? std::string("Preset") : simplified;
}

std::vector<std::filesystem::path> presetDirectoryCandidates()
{
    std::vector<std::filesystem::path> candidates;
    if (const char *xdgData = std::getenv("XDG_DATA_HOME"); xdgData && *xdgData) {
        candidates.emplace_back(std::filesystem::path(xdgData) / "Apertar-UI" / "presets");
        candidates.emplace_back(std::filesystem::path(xdgData) / "ApertarUI" / "presets");
    }

    if (const char *home = std::getenv("HOME"); home && *home) {
        const std::filesystem::path homePath(home);
        candidates.emplace_back(homePath / ".local" / "share" / "Apertar-UI" / "presets");
        candidates.emplace_back(homePath / ".local" / "share" / "ApertarUI" / "presets");
        candidates.emplace_back(homePath / ".config" / "CinePi" / "UI" / "presets");
    }

    std::vector<std::filesystem::path> unique;
    std::set<std::string> seen;
    std::error_code ec;
    for (const auto &candidate : candidates) {
        const auto normalized = std::filesystem::weakly_canonical(candidate, ec);
        const std::string key = ec ? candidate.lexically_normal().string() : normalized.string();
        ec.clear();
        if (seen.insert(key).second)
            unique.push_back(candidate);
    }
    return unique;
}

std::filesystem::path preferredPresetDirectory()
{
    std::error_code ec;
    for (const auto &candidate : presetDirectoryCandidates()) {
        if (std::filesystem::exists(candidate, ec) && std::filesystem::is_directory(candidate, ec))
            return candidate;
        ec.clear();
    }

    if (const char *xdgData = std::getenv("XDG_DATA_HOME"); xdgData && *xdgData)
        return std::filesystem::path(xdgData) / "Apertar-UI" / "presets";

    if (const char *home = std::getenv("HOME"); home && *home)
        return std::filesystem::path(home) / ".local" / "share" / "Apertar-UI" / "presets";

    return std::filesystem::current_path() / "presets";
}

struct PresetFileEntry
{
    std::filesystem::path path;
    std::string fileName;
    std::string displayName;
    uint64_t sizeBytes = 0;
    std::string sizeText = "0 B";
    std::string modifiedText = "Unknown";
    std::string downloadUrl;
};

std::string formatFileTimestampHuman(const std::filesystem::file_time_type &timestamp)
{
    const auto systemPoint = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        timestamp - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
    const std::time_t tt = std::chrono::system_clock::to_time_t(systemPoint);
    std::tm localTm{};
#if defined(_WIN32)
    localtime_s(&localTm, &tt);
#else
    localtime_r(&tt, &localTm);
#endif
    char buffer[32]{};
    if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M", &localTm) == 0)
        return "Unknown";
    return buffer;
}

std::string buildHardwareConfigFileJson()
{
    const std::filesystem::path path = editableHardwareConfigPath();
    std::error_code ec;
    const bool exists = std::filesystem::exists(path, ec) && std::filesystem::is_regular_file(path, ec);
    ec.clear();

    const std::string text = exists ? readTextFileRaw(path) : defaultHardwareConfigTemplate();
    uint64_t sizeBytes = static_cast<uint64_t>(text.size());
    if (exists) {
        const auto fileSize = std::filesystem::file_size(path, ec);
        if (!ec)
            sizeBytes = static_cast<uint64_t>(fileSize);
    }
    ec.clear();

    std::string modifiedText = exists ? formatFileTimestampHuman(std::filesystem::last_write_time(path, ec)) : "Not saved yet";
    if (ec)
        modifiedText = "Unknown";

    std::ostringstream out;
    out << "{\"ok\":true"
        << ",\"path\":\"" << escapeJson(path.string()) << "\""
        << ",\"display_path\":\"" << escapeJson(hardwareConfigDisplayPath(path)) << "\""
        << ",\"exists\":" << (exists ? "true" : "false")
        << ",\"size_bytes\":" << sizeBytes
        << ",\"size_text\":\"" << escapeJson(formatBytesHuman(sizeBytes)) << "\""
        << ",\"modified_text\":\"" << escapeJson(modifiedText) << "\""
        << ",\"text\":\"" << escapeJson(text) << "\""
        << "}";
    return out.str();
}

bool saveHardwareConfigFile(const std::string &text, std::string *backupPath, std::string *errorMessage)
{
    if (trimAscii(text).empty()) {
        if (errorMessage)
            *errorMessage = "Config file cannot be empty.";
        return false;
    }

    if (text.size() > kMaxEditableHardwareConfigBytes) {
        if (errorMessage)
            *errorMessage = "Config file is too large.";
        return false;
    }

    if (text.find('\0') != std::string::npos) {
        if (errorMessage)
            *errorMessage = "Config file contains invalid binary data.";
        return false;
    }

    const std::filesystem::path path = editableHardwareConfigPath();
    const std::filesystem::path parent = path.parent_path();
    std::error_code ec;
    if (!parent.empty() && !std::filesystem::exists(parent, ec)) {
        std::filesystem::create_directories(parent, ec);
        if (ec) {
            if (errorMessage)
                *errorMessage = "Could not create config directory: " + ec.message();
            return false;
        }
    }
    ec.clear();

    const bool existed = std::filesystem::exists(path, ec) && std::filesystem::is_regular_file(path, ec);
    ec.clear();
    if (existed) {
        const std::filesystem::path backup = path.string() + ".bak";
        std::filesystem::copy_file(path, backup, std::filesystem::copy_options::overwrite_existing, ec);
        if (ec) {
            if (errorMessage)
                *errorMessage = "Could not create config backup: " + ec.message();
            return false;
        }
        if (backupPath)
            *backupPath = backup.string();
    }

    const std::filesystem::path tempPath = path.string() + ".tmp";
    if (!writeTextFileRaw(tempPath, text)) {
        if (errorMessage)
            *errorMessage = "Could not write temporary config file.";
        return false;
    }

    std::filesystem::rename(tempPath, path, ec);
    if (ec) {
        std::filesystem::remove(tempPath, ec);
        if (errorMessage)
            *errorMessage = "Could not save config file: " + ec.message();
        return false;
    }

    return true;
}

std::string presetDisplayNameForFile(const std::filesystem::path &path)
{
    const std::string payload = readTextFileRaw(path);
    if (!payload.empty()) {
        if (const auto name = extractJsonStringField(payload, "name")) {
            const std::string trimmed = trimAscii(*name);
            if (!trimmed.empty())
                return trimmed;
        }
    }

    std::string fallback = path.filename().string();
    const std::string suffix = ".apertar-preset.json";
    if (fallback.size() > suffix.size() &&
        fallback.rfind(suffix) == fallback.size() - suffix.size()) {
        fallback.resize(fallback.size() - suffix.size());
    }
    return fallback.empty() ? "Preset" : fallback;
}

std::vector<PresetFileEntry> discoverPresetFiles()
{
    std::vector<PresetFileEntry> presets;
    std::set<std::string> seenPaths;
    std::error_code ec;

    for (const auto &directory : presetDirectoryCandidates()) {
        if (!std::filesystem::exists(directory, ec) || !std::filesystem::is_directory(directory, ec)) {
            ec.clear();
            continue;
        }

        for (const auto &entry : std::filesystem::directory_iterator(directory, ec)) {
            if (ec)
                break;
            if (!entry.is_regular_file(ec)) {
                ec.clear();
                continue;
            }

            const std::string fileName = entry.path().filename().string();
            if (!isSafePresetFileName(fileName))
                continue;

            const auto canonical = std::filesystem::weakly_canonical(entry.path(), ec);
            const std::string key = ec ? entry.path().lexically_normal().string() : canonical.string();
            ec.clear();
            if (!seenPaths.insert(key).second)
                continue;

            PresetFileEntry preset;
            preset.path = entry.path();
            preset.fileName = fileName;
            preset.displayName = presetDisplayNameForFile(entry.path());
            preset.sizeBytes = entry.file_size(ec);
            ec.clear();
            preset.sizeText = formatBytesHuman(preset.sizeBytes);
            preset.modifiedText = formatFileTimestampHuman(entry.last_write_time(ec));
            ec.clear();
            preset.downloadUrl = "/api/presets/download/" + urlEncodeComponent(fileName);
            presets.push_back(std::move(preset));
        }
    }

    std::sort(presets.begin(), presets.end(), [](const PresetFileEntry &lhs, const PresetFileEntry &rhs) {
        std::string left = lhs.displayName;
        std::string right = rhs.displayName;
        std::transform(left.begin(), left.end(), left.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        std::transform(right.begin(), right.end(), right.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        if (left != right)
            return left < right;
        return lhs.fileName < rhs.fileName;
    });

    return presets;
}

struct MediaLibraryItem
{
    std::string id;
    std::string kind;
    std::string category;
    std::string type;
    std::string name;
    uint64_t sizeBytes = 0;
    std::string sizeText = "0 B";
    uint64_t fileCount = 0;
    uint64_t frameCount = 0;
    std::string durationText = "--";
    std::string fpsText = "-- FPS";
    std::string modifiedText = "Unknown";
    int64_t modifiedMs = 0;
    std::filesystem::file_time_type modifiedTime{};
    std::string downloadUrl;
};

struct MediaSelection
{
    std::string kind;
    std::string id;
};

struct MediaDeletionTargets
{
    std::vector<std::filesystem::path> folders;
    std::vector<std::filesystem::path> files;
};

bool isSafeMediaRelativeId(const std::string &id)
{
    if (id.empty() || id.front() == '/' || id.find('\\') != std::string::npos || id.find('\0') != std::string::npos)
        return false;

    std::filesystem::path path(id);
    if (path.is_absolute())
        return false;

    for (const auto &part : path) {
        const std::string segment = part.string();
        if (segment.empty() || segment == "." || segment == "..")
            return false;
    }
    return true;
}

std::vector<MediaSelection> mediaSelectionsFromQuery(const std::string &query, std::string *errorMessage)
{
    std::vector<MediaSelection> selections;
    std::set<std::string> seen;
    for (const std::string &value : queryParameterValues(query, "item")) {
        const size_t separator = value.find(':');
        if (separator == std::string::npos) {
            if (errorMessage)
                *errorMessage = "Selected media item is invalid.";
            return {};
        }

        MediaSelection selection;
        selection.kind = value.substr(0, separator);
        selection.id = value.substr(separator + 1);

        if ((selection.kind != "folder" && selection.kind != "still") || !isSafeMediaRelativeId(selection.id)) {
            if (errorMessage)
                *errorMessage = "Selected media item is invalid.";
            return {};
        }

        const std::string key = selection.kind + "\n" + selection.id;
        if (seen.insert(key).second)
            selections.push_back(std::move(selection));
    }

    if (selections.empty() && errorMessage)
        *errorMessage = "Select at least one media item.";

    return selections;
}

bool pathIsInsideRoot(const std::filesystem::path &root, const std::filesystem::path &candidate)
{
    std::error_code ec;
    const std::filesystem::path canonicalRoot = std::filesystem::weakly_canonical(root, ec);
    if (ec || canonicalRoot.empty())
        return false;
    ec.clear();

    const std::filesystem::path canonicalCandidate = std::filesystem::weakly_canonical(candidate, ec);
    if (ec || canonicalCandidate.empty())
        return false;

    auto rootIt = canonicalRoot.begin();
    auto candidateIt = canonicalCandidate.begin();
    for (; rootIt != canonicalRoot.end(); ++rootIt, ++candidateIt) {
        if (candidateIt == canonicalCandidate.end() || *rootIt != *candidateIt)
            return false;
    }
    return true;
}

std::vector<std::filesystem::path> lutPreviewSearchDirectories(const std::string &mediaRoot)
{
    std::vector<std::filesystem::path> directories;
    directories.emplace_back("/home/pi/LUTs");
    if (!mediaRoot.empty())
        directories.emplace_back(std::filesystem::path(mediaRoot) / "LUTs");
    directories.emplace_back("/media/RAW/LUTs");

    std::error_code ec;
    const std::filesystem::path mediaDirectory("/media");
    if (std::filesystem::exists(mediaDirectory, ec) && std::filesystem::is_directory(mediaDirectory, ec)) {
        for (const auto &entry : std::filesystem::directory_iterator(mediaDirectory, ec)) {
            if (ec)
                break;
            if (entry.is_directory(ec))
                directories.push_back(entry.path() / "LUTs");
        }
    }

    std::vector<std::filesystem::path> unique;
    std::set<std::string> seen;
    for (const auto &directory : directories) {
        std::error_code canonicalError;
        const std::filesystem::path clean = std::filesystem::weakly_canonical(directory, canonicalError);
        const std::string key = (canonicalError ? directory.lexically_normal() : clean).string();
        if (!key.empty() && seen.insert(key).second)
            unique.push_back(canonicalError ? directory.lexically_normal() : clean);
    }
    return unique;
}

std::vector<std::string> discoverLutPreviewFiles(const std::string &mediaRoot)
{
    std::vector<std::string> files;
    std::set<std::string> seen;

    for (const auto &directory : lutPreviewSearchDirectories(mediaRoot)) {
        std::error_code ec;
        if (!std::filesystem::exists(directory, ec) || !std::filesystem::is_directory(directory, ec))
            continue;

        for (const auto &entry : std::filesystem::directory_iterator(directory, ec)) {
            if (ec)
                break;
            if (!entry.is_regular_file(ec))
                continue;

            const std::string extension = lowerAscii(entry.path().extension().string());
            if (extension != ".cube")
                continue;

            std::error_code canonicalError;
            const std::filesystem::path clean = std::filesystem::weakly_canonical(entry.path(), canonicalError);
            const std::string path = (canonicalError ? entry.path().lexically_normal() : clean).string();
            if (!path.empty() && seen.insert(path).second)
                files.push_back(path);
        }
    }

    std::sort(files.begin(), files.end());
    return files;
}

bool lutPreviewFileAllowed(const std::string &mediaRoot, const std::string &value)
{
    const std::filesystem::path requested = std::filesystem::path(trimAscii(value)).lexically_normal();
    if (requested.empty())
        return false;

    std::error_code ec;
    const std::filesystem::path requestedCanonical = std::filesystem::weakly_canonical(requested, ec);
    if (ec)
        return false;
    const std::string requestedPath = requestedCanonical.string();

    const std::vector<std::string> options = discoverLutPreviewFiles(mediaRoot);
    return std::find(options.begin(), options.end(), requestedPath) != options.end();
}

std::optional<std::filesystem::path> resolveMediaLibraryPath(const std::filesystem::path &mediaRoot,
                                                             const std::string &id)
{
    if (!isSafeMediaRelativeId(id))
        return std::nullopt;

    std::error_code ec;
    if (!std::filesystem::exists(mediaRoot, ec) || !std::filesystem::is_directory(mediaRoot, ec))
        return std::nullopt;
    ec.clear();

    const std::filesystem::path candidate = mediaRoot / std::filesystem::path(id);
    if (!std::filesystem::exists(candidate, ec) || !pathIsInsideRoot(mediaRoot, candidate))
        return std::nullopt;
    return candidate;
}

bool hasStillImageExtension(const std::filesystem::path &path)
{
    const std::string ext = toLowerAsciiCopy(path.extension().string());
    return ext == ".dng" || ext == ".jpg" || ext == ".jpeg" ||
           ext == ".png" || ext == ".tif" || ext == ".tiff";
}

bool hasAudioExtension(const std::filesystem::path &path)
{
    return toLowerAsciiCopy(path.extension().string()) == ".wav";
}

bool hasClipContainerExtension(const std::filesystem::path &path)
{
    const std::string ext = toLowerAsciiCopy(path.extension().string());
    return ext == ".mp4" || ext == ".m4v" || ext == ".mov" || ext == ".mlv";
}

std::string clipContainerTypeLabel(const std::filesystem::path &path)
{
    const std::string ext = toLowerAsciiCopy(path.extension().string());
    const std::string name = toLowerAsciiCopy(path.filename().string());
    if (ext == ".mlv")
        return "MLV Clip";
    if (ext == ".mov" && name.rfind("prores_", 0) == 0)
        return "ProRes Clip";
    if (ext == ".mp4" && name.rfind("h264_", 0) == 0)
        return "H.264 Clip";
    return "Video Clip";
}

std::string stillFormatNameForExtension(const std::filesystem::path &path)
{
    const std::string ext = toLowerAsciiCopy(path.extension().string());
    if (ext == ".jpg" || ext == ".jpeg")
        return "JPEG";
    if (ext == ".png")
        return "PNG";
    if (ext == ".tif" || ext == ".tiff")
        return "TIFF";
    return "DNG";
}

std::string stillTypeLabelForFiles(const std::vector<std::filesystem::path> &files)
{
    std::vector<std::string> formats;
    for (const auto &file : files) {
        const std::string format = stillFormatNameForExtension(file);
        if (std::find(formats.begin(), formats.end(), format) == formats.end())
            formats.push_back(format);
    }

    const auto formatRank = [](const std::string &format) {
        if (format == "DNG")
            return 0;
        if (format == "JPEG")
            return 1;
        if (format == "PNG")
            return 2;
        if (format == "TIFF")
            return 3;
        return 4;
    };
    std::sort(formats.begin(), formats.end(), [&](const std::string &lhs, const std::string &rhs) {
        return formatRank(lhs) < formatRank(rhs);
    });

    if (formats.empty())
        return "Still";

    std::ostringstream out;
    for (size_t i = 0; i < formats.size(); ++i) {
        if (i > 0)
            out << " + ";
        out << formats[i];
    }
    out << " Still";
    return out.str();
}

std::string durationTextForFrameCount(uint64_t frameCount, double fps)
{
    if (frameCount == 0 || fps <= 0.0)
        return "--";

    const uint64_t totalSeconds = static_cast<uint64_t>(
        std::max(0.0, std::round(static_cast<double>(frameCount) / fps)));
    const uint64_t hours = totalSeconds / 3600;
    const uint64_t minutes = (totalSeconds % 3600) / 60;
    const uint64_t seconds = totalSeconds % 60;

    std::ostringstream out;
    out << std::setfill('0');
    if (hours > 0)
        out << hours << ':' << std::setw(2) << minutes << ':' << std::setw(2) << seconds;
    else
        out << std::setw(2) << minutes << ':' << std::setw(2) << seconds;
    return out.str();
}

std::string durationTextForSeconds(double seconds)
{
    if (seconds <= 0.0)
        return "--";

    const uint64_t totalSeconds = static_cast<uint64_t>(std::max(0.0, std::round(seconds)));
    const uint64_t hours = totalSeconds / 3600;
    const uint64_t minutes = (totalSeconds % 3600) / 60;
    const uint64_t secs = totalSeconds % 60;

    std::ostringstream out;
    out << std::setfill('0');
    if (hours > 0)
        out << hours << ':' << std::setw(2) << minutes << ':' << std::setw(2) << secs;
    else
        out << std::setw(2) << minutes << ':' << std::setw(2) << secs;
    return out.str();
}

std::string wavDurationTextForFile(const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
        return "--";

    auto readU16 = [](const char *bytes) -> uint16_t {
        return static_cast<uint16_t>(static_cast<unsigned char>(bytes[0])) |
               static_cast<uint16_t>(static_cast<unsigned char>(bytes[1]) << 8);
    };
    auto readU32 = [](const char *bytes) -> uint32_t {
        return static_cast<uint32_t>(static_cast<unsigned char>(bytes[0])) |
               (static_cast<uint32_t>(static_cast<unsigned char>(bytes[1])) << 8) |
               (static_cast<uint32_t>(static_cast<unsigned char>(bytes[2])) << 16) |
               (static_cast<uint32_t>(static_cast<unsigned char>(bytes[3])) << 24);
    };

    char riff[12]{};
    input.read(riff, sizeof(riff));
    if (input.gcount() != static_cast<std::streamsize>(sizeof(riff)) ||
        (std::memcmp(riff, "RIFF", 4) != 0 && std::memcmp(riff, "RF64", 4) != 0) ||
        std::memcmp(riff + 8, "WAVE", 4) != 0) {
        return "--";
    }

    uint16_t channels = 0;
    uint32_t sampleRate = 0;
    uint16_t bitsPerSample = 0;
    uint64_t dataBytes = 0;
    uint64_t rf64DataBytes = 0;

    while (input.good()) {
        char chunkId[4]{};
        std::array<char, 4> sizeBytes{};
        input.read(chunkId, sizeof(chunkId));
        input.read(sizeBytes.data(), static_cast<std::streamsize>(sizeBytes.size()));
        if (input.gcount() != static_cast<std::streamsize>(sizeBytes.size()))
            break;

        const uint32_t chunkSize = readU32(sizeBytes.data());
        const std::streampos chunkDataStart = input.tellg();
        if (std::memcmp(chunkId, "ds64", 4) == 0 && chunkSize >= 28) {
            std::array<char, 28> ds64{};
            input.read(ds64.data(), ds64.size());
            if (input.gcount() == 28) rf64DataBytes = readU32(ds64.data() + 8) | (uint64_t(readU32(ds64.data() + 12)) << 32);
        } else if (std::memcmp(chunkId, "fmt ", 4) == 0 && chunkSize >= 16) {
            std::array<char, 16> fmt{};
            input.read(fmt.data(), static_cast<std::streamsize>(fmt.size()));
            if (input.gcount() != static_cast<std::streamsize>(fmt.size()))
                break;
            channels = readU16(fmt.data() + 2);
            sampleRate = readU32(fmt.data() + 4);
            bitsPerSample = readU16(fmt.data() + 14);
        } else if (std::memcmp(chunkId, "data", 4) == 0) {
            dataBytes = chunkSize == 0xffffffffu && rf64DataBytes ? rf64DataBytes : chunkSize;
        }

        if (channels > 0 && sampleRate > 0 && bitsPerSample > 0 && dataBytes > 0)
            break;

        input.seekg(chunkDataStart + static_cast<std::streamoff>(chunkSize + (chunkSize % 2)), std::ios::beg);
    }

    const uint32_t bytesPerSecond = sampleRate * channels * (bitsPerSample / 8);
    if (bytesPerSecond == 0 || dataBytes == 0)
        return "--";
    return durationTextForSeconds(static_cast<double>(dataBytes) / static_cast<double>(bytesPerSecond));
}

std::string wavDurationTextForFolder(const std::filesystem::path &folder)
{
    std::error_code ec;
    for (const auto &entry : std::filesystem::directory_iterator(folder, ec)) {
        if (ec)
            break;
        if (entry.is_regular_file(ec) && hasAudioExtension(entry.path()))
            return wavDurationTextForFile(entry.path());
        ec.clear();
    }
    return "--";
}

int64_t fileTimeToUnixMilliseconds(const std::filesystem::file_time_type &timestamp)
{
    const auto systemPoint = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        timestamp - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        systemPoint.time_since_epoch()).count();
}

struct MediaPathStats
{
    uint64_t sizeBytes = 0;
    uint64_t fileCount = 0;
    uint64_t frameCount = 0;
    std::filesystem::file_time_type modifiedTime{};
    bool modifiedValid = false;
};

void updateMediaModifiedTime(MediaPathStats *stats, const std::filesystem::file_time_type &time)
{
    if (!stats)
        return;
    if (!stats->modifiedValid || time > stats->modifiedTime) {
        stats->modifiedTime = time;
        stats->modifiedValid = true;
    }
}

MediaPathStats mediaStatsForFileList(const std::vector<std::filesystem::path> &files)
{
    MediaPathStats stats;
    std::error_code ec;
    for (const auto &file : files) {
        if (!std::filesystem::is_regular_file(file, ec)) {
            ec.clear();
            continue;
        }
        stats.sizeBytes += static_cast<uint64_t>(std::filesystem::file_size(file, ec));
        ec.clear();
        stats.fileCount += 1;
        if (hasStillImageExtension(file))
            stats.frameCount += 1;
        updateMediaModifiedTime(&stats, std::filesystem::last_write_time(file, ec));
        ec.clear();
    }
    return stats;
}

MediaPathStats mediaStatsForPath(const std::filesystem::path &path)
{
    MediaPathStats stats;
    std::error_code ec;
    if (std::filesystem::is_regular_file(path, ec)) {
        stats.sizeBytes = static_cast<uint64_t>(std::filesystem::file_size(path, ec));
        ec.clear();
        stats.fileCount = 1;
        if (hasStillImageExtension(path))
            stats.frameCount = 1;
        updateMediaModifiedTime(&stats, std::filesystem::last_write_time(path, ec));
        return stats;
    }
    ec.clear();

    if (!std::filesystem::is_directory(path, ec))
        return stats;
    updateMediaModifiedTime(&stats, std::filesystem::last_write_time(path, ec));
    ec.clear();

    for (const auto &entry : std::filesystem::recursive_directory_iterator(
             path, std::filesystem::directory_options::skip_permission_denied, ec)) {
        if (ec)
            break;
        if (!entry.is_regular_file(ec)) {
            ec.clear();
            continue;
        }
        stats.sizeBytes += static_cast<uint64_t>(entry.file_size(ec));
        ec.clear();
        stats.fileCount += 1;
        if (hasStillImageExtension(entry.path()))
            stats.frameCount += 1;
        updateMediaModifiedTime(&stats, entry.last_write_time(ec));
        ec.clear();
    }
    return stats;
}

std::vector<std::filesystem::path> stillGroupFilesForId(const std::filesystem::path &mediaRoot,
                                                        const std::string &id)
{
    if (!isSafeMediaRelativeId(id))
        return {};

    const std::filesystem::path relative(id);
    auto it = relative.begin();
    if (it == relative.end() || it->string() != "Photos")
        return {};
    ++it;
    if (it == relative.end())
        return {};
    const std::string stem = it->string();
    ++it;
    if (it != relative.end() || stem.empty())
        return {};

    const std::filesystem::path photosDir = mediaRoot / "Photos";
    if (!pathIsInsideRoot(mediaRoot, photosDir))
        return {};

    std::vector<std::filesystem::path> files;
    std::error_code ec;
    if (!std::filesystem::exists(photosDir, ec) || !std::filesystem::is_directory(photosDir, ec))
        return files;
    ec.clear();

    const std::string wantedStem = toLowerAsciiCopy(stem);
    for (const auto &entry : std::filesystem::directory_iterator(photosDir, ec)) {
        if (ec)
            break;
        if (!entry.is_regular_file(ec) || !hasStillImageExtension(entry.path())) {
            ec.clear();
            continue;
        }
        if (toLowerAsciiCopy(entry.path().stem().string()) == wantedStem &&
            pathIsInsideRoot(mediaRoot, entry.path())) {
            files.push_back(entry.path());
        }
        ec.clear();
    }

    std::sort(files.begin(), files.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.filename().string() < rhs.filename().string();
    });
    return files;
}

std::optional<std::vector<std::string>> mediaSelectionZipEntries(const std::filesystem::path &mediaRoot,
                                                                 const std::vector<MediaSelection> &selections,
                                                                 std::string *errorMessage)
{
    std::vector<std::string> relativeEntries;
    std::set<std::string> seenEntries;
    std::error_code ec;

    for (const MediaSelection &selection : selections) {
        if (selection.kind == "folder") {
            const auto itemPath = resolveMediaLibraryPath(mediaRoot, selection.id);
            if (!itemPath || !std::filesystem::is_directory(*itemPath, ec)) {
                if (errorMessage)
                    *errorMessage = "Selected media item not found.";
                return std::nullopt;
            }
            ec.clear();

            const auto relative = std::filesystem::relative(*itemPath, mediaRoot, ec);
            if (ec || relative.empty()) {
                if (errorMessage)
                    *errorMessage = "Could not prepare selected media.";
                return std::nullopt;
            }
            ec.clear();

            const std::string entry = relative.lexically_normal().string();
            if (isSafeMediaRelativeId(entry) && seenEntries.insert(entry).second)
                relativeEntries.push_back(entry);
            continue;
        }

        if (selection.kind == "still") {
            const std::vector<std::filesystem::path> files = stillGroupFilesForId(mediaRoot, selection.id);
            if (files.empty()) {
                if (errorMessage)
                    *errorMessage = "Selected still not found.";
                return std::nullopt;
            }

            for (const auto &file : files) {
                const auto relative = std::filesystem::relative(file, mediaRoot, ec);
                if (ec || relative.empty() || !pathIsInsideRoot(mediaRoot, file)) {
                    if (errorMessage)
                        *errorMessage = "Could not prepare selected still.";
                    return std::nullopt;
                }
                ec.clear();

                const std::string entry = relative.lexically_normal().string();
                if (isSafeMediaRelativeId(entry) && seenEntries.insert(entry).second)
                    relativeEntries.push_back(entry);
            }
        }
    }

    if (relativeEntries.empty()) {
        if (errorMessage)
            *errorMessage = "No downloadable media found.";
        return std::nullopt;
    }

    return relativeEntries;
}

std::optional<MediaDeletionTargets> mediaSelectionDeletionTargets(const std::filesystem::path &mediaRoot,
                                                                  const std::vector<MediaSelection> &selections,
                                                                  std::string *errorMessage)
{
    MediaDeletionTargets targets;
    std::set<std::string> seenPaths;
    std::error_code ec;

    const auto addTarget = [&](const std::filesystem::path &path, bool folder) -> bool {
        if (!pathIsInsideRoot(mediaRoot, path)) {
            if (errorMessage)
                *errorMessage = "Selected media item is outside the media drive.";
            return false;
        }

        const std::filesystem::path canonical = std::filesystem::weakly_canonical(path, ec);
        if (ec || canonical.empty()) {
            if (errorMessage)
                *errorMessage = "Selected media item not found.";
            return false;
        }
        ec.clear();

        const std::string key = canonical.string();
        if (!seenPaths.insert(key).second)
            return true;

        if (folder)
            targets.folders.push_back(canonical);
        else
            targets.files.push_back(canonical);
        return true;
    };

    for (const MediaSelection &selection : selections) {
        if (selection.kind == "folder") {
            const auto itemPath = resolveMediaLibraryPath(mediaRoot, selection.id);
            if (!itemPath || !std::filesystem::is_directory(*itemPath, ec)) {
                if (errorMessage)
                    *errorMessage = "Selected media item not found.";
                return std::nullopt;
            }
            ec.clear();

            if (!addTarget(*itemPath, true))
                return std::nullopt;
            continue;
        }

        if (selection.kind == "still") {
            const std::vector<std::filesystem::path> files = stillGroupFilesForId(mediaRoot, selection.id);
            if (files.empty()) {
                if (errorMessage)
                    *errorMessage = "Selected still not found.";
                return std::nullopt;
            }
            for (const auto &file : files) {
                if (!addTarget(file, false))
                    return std::nullopt;
            }
        }
    }

    if (targets.folders.empty() && targets.files.empty()) {
        if (errorMessage)
            *errorMessage = "No selected media found.";
        return std::nullopt;
    }

    return targets;
}

std::string mediaDownloadUrl(const std::string &kind, const std::string &id)
{
    return "/api/media/download?kind=" + urlEncodeComponent(kind) + "&id=" + urlEncodeComponent(id);
}

std::string mediaContentTypeForFile(const std::filesystem::path &path)
{
    const std::string ext = toLowerAsciiCopy(path.extension().string());
    if (ext == ".jpg" || ext == ".jpeg")
        return "image/jpeg";
    if (ext == ".png")
        return "image/png";
    if (ext == ".tif" || ext == ".tiff")
        return "image/tiff";
    if (ext == ".dng")
        return "image/x-adobe-dng";
    if (ext == ".wav")
        return "audio/wav";
    if (ext == ".mp4" || ext == ".m4v")
        return "video/mp4";
    if (ext == ".mov")
        return "video/quicktime";
    if (ext == ".gcsv")
        return "text/csv; charset=utf-8";
    return "application/octet-stream";
}

std::string sanitizeDownloadFileName(std::string name)
{
    name = trimAscii(name);
    if (name.empty())
        name = "Apertar-Media";
    for (char &ch : name) {
        if (ch == '"' || ch == '\\' || ch == '/' || ch == '\r' || ch == '\n' || ch == '\0')
            ch = '_';
    }
    return name;
}

bool sendFileChunk(int clientFd, const char *data, size_t size)
{
    const char *cursor = data;
    size_t remaining = size;
    while (remaining > 0) {
        const ssize_t written = ::send(clientFd, cursor, remaining, MSG_NOSIGNAL);
        if (written <= 0)
            return false;
        cursor += written;
        remaining -= static_cast<size_t>(written);
    }
    return true;
}

void streamFileToClient(int clientFd, const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open())
        return;

    std::array<char, 64 * 1024> buffer{};
    while (input.good()) {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize bytesRead = input.gcount();
        if (bytesRead <= 0)
            break;
        if (!sendFileChunk(clientFd, buffer.data(), static_cast<size_t>(bytesRead)))
            break;
    }
}

std::string zipProgramPath()
{
    return existingExecutablePath({"/usr/bin/zip", "/bin/zip"});
}

void streamZipArchiveToClient(int clientFd,
                              const std::filesystem::path &workingDirectory,
                              const std::vector<std::string> &relativeEntries)
{
    const std::string zipPath = zipProgramPath();
    if (zipPath.empty() || relativeEntries.empty())
        return;

    int pipeFds[2] = {-1, -1};
    if (::pipe(pipeFds) != 0)
        return;

    const pid_t child = ::fork();
    if (child == 0) {
        ::close(pipeFds[0]);
        if (::chdir(workingDirectory.c_str()) != 0)
            _exit(126);
        if (::dup2(pipeFds[1], STDOUT_FILENO) < 0)
            _exit(126);
        ::close(pipeFds[1]);

        const int nullFd = ::open("/dev/null", O_WRONLY);
        if (nullFd >= 0) {
            ::dup2(nullFd, STDERR_FILENO);
            ::close(nullFd);
        }

        std::vector<std::string> args;
        args.reserve(relativeEntries.size() + 6);
        args.push_back(zipPath);
        args.push_back("-r");
        args.push_back("-0");
        args.push_back("-q");
        args.push_back("-");
        args.push_back("--");
        for (const auto &entry : relativeEntries)
            args.push_back(entry);

        std::vector<char *> argv;
        argv.reserve(args.size() + 1);
        for (std::string &arg : args)
            argv.push_back(arg.data());
        argv.push_back(nullptr);
        ::execv(zipPath.c_str(), argv.data());
        _exit(127);
    }

    ::close(pipeFds[1]);
    if (child < 0) {
        ::close(pipeFds[0]);
        return;
    }

    std::array<char, 64 * 1024> buffer{};
    while (true) {
        const ssize_t bytesRead = ::read(pipeFds[0], buffer.data(), buffer.size());
        if (bytesRead <= 0)
            break;
        if (!sendFileChunk(clientFd, buffer.data(), static_cast<size_t>(bytesRead)))
            break;
    }

    ::close(pipeFds[0]);
    int status = 0;
    ::waitpid(child, &status, 0);
}

std::vector<MediaLibraryItem> discoverMediaLibraryItems(const std::filesystem::path &mediaRoot, double fps)
{
    std::vector<MediaLibraryItem> items;
    std::error_code ec;
    if (!std::filesystem::exists(mediaRoot, ec) || !std::filesystem::is_directory(mediaRoot, ec))
        return items;
    ec.clear();

    const auto addItemStats = [](MediaLibraryItem *item, const MediaPathStats &stats) {
        if (!item)
            return;
        item->sizeBytes = stats.sizeBytes;
        item->sizeText = formatBytesHuman(stats.sizeBytes);
        item->fileCount = stats.fileCount;
        if (stats.modifiedValid) {
            item->modifiedTime = stats.modifiedTime;
            item->modifiedText = formatFileTimestampHuman(stats.modifiedTime);
            item->modifiedMs = fileTimeToUnixMilliseconds(stats.modifiedTime);
        }
    };

    for (const auto &entry : std::filesystem::directory_iterator(mediaRoot, ec)) {
        if (ec)
            break;
        if (!entry.is_directory(ec)) {
            ec.clear();
            continue;
        }

        const std::string folderName = entry.path().filename().string();
        if (folderName == "Photos" || folderName == "Audio") {
            ec.clear();
            continue;
        }

        bool containsMediaFrame = false;
        std::string containerType;
        std::error_code scanEc;
        for (const auto &child : std::filesystem::recursive_directory_iterator(
                 entry.path(), std::filesystem::directory_options::skip_permission_denied, scanEc)) {
            if (scanEc)
                break;
            if (child.is_regular_file(scanEc)) {
                if (hasStillImageExtension(child.path()))
                    containsMediaFrame = true;
                if (hasClipContainerExtension(child.path()))
                    containerType = clipContainerTypeLabel(child.path());
            }
            if (containsMediaFrame || !containerType.empty())
                break;
            scanEc.clear();
        }
        if (!containsMediaFrame && containerType.empty()) {
            ec.clear();
            continue;
        }

        MediaLibraryItem item;
        item.id = folderName;
        item.kind = "folder";
        item.category = folderName.rfind("Timelapse_", 0) == 0 ? "timelapse" : "clip";
        item.name = folderName;
        item.type = item.category == "timelapse"
            ? "Timelapse"
            : (!containerType.empty() ? containerType : "cDNG Clip");
        const MediaPathStats stats = mediaStatsForPath(entry.path());
        addItemStats(&item, stats);
        item.frameCount = stats.frameCount;
        item.durationText = !containerType.empty()
            ? "--"
            : durationTextForFrameCount(stats.frameCount, fps);
        item.fpsText = compactNumber(fps) + " FPS";
        item.downloadUrl = mediaDownloadUrl(item.kind, item.id);
        items.push_back(std::move(item));
        ec.clear();
    }

    const std::filesystem::path photosDir = mediaRoot / "Photos";
    if (std::filesystem::exists(photosDir, ec) && std::filesystem::is_directory(photosDir, ec)) {
        std::map<std::string, std::vector<std::filesystem::path>> stillGroups;
        std::map<std::string, std::string> displayStems;
        for (const auto &entry : std::filesystem::directory_iterator(photosDir, ec)) {
            if (ec)
                break;
            if (!entry.is_regular_file(ec) || !hasStillImageExtension(entry.path())) {
                ec.clear();
                continue;
            }
            const std::string stem = entry.path().stem().string();
            const std::string key = toLowerAsciiCopy(stem);
            stillGroups[key].push_back(entry.path());
            displayStems.emplace(key, stem);
            ec.clear();
        }

        for (auto &[key, files] : stillGroups) {
            std::sort(files.begin(), files.end(), [](const auto &lhs, const auto &rhs) {
                return lhs.filename().string() < rhs.filename().string();
            });

            MediaLibraryItem item;
            item.id = "Photos/" + displayStems[key];
            item.kind = "still";
            item.category = "still";
            item.type = stillTypeLabelForFiles(files);
            item.name = displayStems[key];
            const MediaPathStats stats = mediaStatsForFileList(files);
            addItemStats(&item, stats);
            item.frameCount = 1;
            item.durationText = "Still";
            item.fpsText = compactNumber(fps) + " FPS";
            item.downloadUrl = mediaDownloadUrl(item.kind, item.id);
            items.push_back(std::move(item));
        }
    }

    const std::filesystem::path audioDir = mediaRoot / "Audio";
    if (std::filesystem::exists(audioDir, ec) && std::filesystem::is_directory(audioDir, ec)) {
        for (const auto &entry : std::filesystem::directory_iterator(audioDir, ec)) {
            if (ec)
                break;
            if (!entry.is_directory(ec)) {
                ec.clear();
                continue;
            }

            bool containsWav = false;
            std::error_code scanEc;
            for (const auto &child : std::filesystem::directory_iterator(entry.path(), scanEc)) {
                if (scanEc)
                    break;
                if (child.is_regular_file(scanEc) && hasAudioExtension(child.path())) {
                    containsWav = true;
                    break;
                }
                scanEc.clear();
            }
            if (!containsWav) {
                ec.clear();
                continue;
            }

            const std::string folderName = entry.path().filename().string();
            MediaLibraryItem item;
            item.id = "Audio/" + folderName;
            item.kind = "folder";
            item.category = "audio";
            item.type = "WAV Audio";
            item.name = folderName;
            const MediaPathStats stats = mediaStatsForPath(entry.path());
            addItemStats(&item, stats);
            item.frameCount = 0;
            item.durationText = wavDurationTextForFolder(entry.path());
            item.fpsText = "Audio";
            item.downloadUrl = mediaDownloadUrl(item.kind, item.id);
            items.push_back(std::move(item));
            ec.clear();
        }
    }

    std::sort(items.begin(), items.end(), [](const MediaLibraryItem &lhs, const MediaLibraryItem &rhs) {
        if (lhs.modifiedTime != rhs.modifiedTime)
            return lhs.modifiedTime > rhs.modifiedTime;
        return lhs.name > rhs.name;
    });
    return items;
}

std::optional<std::filesystem::path> findPresetFileByName(const std::string &fileName)
{
    if (!isSafePresetFileName(fileName))
        return std::nullopt;

    std::error_code ec;
    for (const auto &directory : presetDirectoryCandidates()) {
        const auto path = directory / fileName;
        if (std::filesystem::exists(path, ec) && std::filesystem::is_regular_file(path, ec))
            return path;
        ec.clear();
    }

    return std::nullopt;
}

bool importPresetFileToCamera(const std::string &uploadedFileName,
                              const std::string &payload,
                              std::string *importedName,
                              bool *alreadyExists,
                              std::string *errorMessage)
{
    const std::string trimmedPayload = trimAscii(payload);
    if (trimmedPayload.empty() || trimmedPayload.front() != '{' || trimmedPayload.back() != '}') {
        if (errorMessage)
            *errorMessage = "The uploaded preset file is not valid JSON.";
        return false;
    }

    std::string resolvedName;
    if (const auto parsedName = extractJsonStringField(payload, "name")) {
        resolvedName = sanitizePresetFileStem(*parsedName);
    } else {
        resolvedName = sanitizePresetFileStem(stripPresetFileSuffix(uploadedFileName));
    }

    const auto existing = discoverPresetFiles();
    const std::string normalizedResolvedName = toLowerAsciiCopy(resolvedName);
    for (const auto &preset : existing) {
        if (toLowerAsciiCopy(preset.displayName) == normalizedResolvedName) {
            if (importedName)
                *importedName = preset.displayName;
            if (alreadyExists)
                *alreadyExists = true;
            return true;
        }
    }

    const std::filesystem::path directory = preferredPresetDirectory();
    std::error_code ec;
    std::filesystem::create_directories(directory, ec);
    if (ec) {
        if (errorMessage)
            *errorMessage = "Could not create the preset folder on the camera.";
        return false;
    }

    const std::string fileName = sanitizePresetFileStem(resolvedName) + ".apertar-preset.json";
    const std::filesystem::path finalPath = directory / fileName;
    if (!settingsFile::atomicWrite(finalPath, payload)) {
        if (errorMessage)
            *errorMessage = "Could not save the uploaded preset on the camera.";
        return false;
    }

    if (importedName)
        *importedName = resolvedName;
    if (alreadyExists)
        *alreadyExists = false;
    return true;
}

std::string buildCurrentPresetPayload(const std::string &presetName, const CaptureSettings &settings)
{
    const SharedUiSettingsSnapshot uiSnapshot = readSharedUiSettings();
    const std::map<std::string, std::string> uiValues = readSharedUiGeneralSettingsMap();

    std::ostringstream out;
    out << "{";
    out << "\"type\":\"apertar-preset\""
        << ",\"version\":1"
        << ",\"name\":\"" << escapeJson(trimAscii(presetName)) << "\""
        << ",\"createdAt\":\"" << escapeJson(currentUtcIsoTimestamp()) << "\""
        << ",\"settings\":{";

    bool firstSetting = true;
    auto addComma = [&]() {
        if (!firstSetting)
            out << ',';
        firstSetting = false;
    };
    auto addBool = [&](const char *key, bool value) {
        addComma();
        out << '"' << key << "\":" << (value ? "true" : "false");
    };
    auto addNumberString = [&](const char *key, const std::string &value) {
        addComma();
        out << '"' << key << "\":" << value;
    };
    auto addString = [&](const char *key, const std::string &value) {
        addComma();
        out << '"' << key << "\":\"" << escapeJson(value) << '"';
    };
    auto addBoolFromMap = [&](const char *key, bool fallback) {
        const auto it = uiValues.find(key);
        addBool(key, it != uiValues.end() ? parseUiSettingsBool(it->second, fallback) : fallback);
    };
    auto addDoubleFromMap = [&](const char *key) {
        const auto it = uiValues.find(key);
        if (it == uiValues.end())
            return;
        if (const auto parsed = parseDoubleValue(it->second))
            addNumberString(key, compactNumber(*parsed));
    };
    auto addIntFromMap = [&](const char *key) {
        const auto it = uiValues.find(key);
        if (it == uiValues.end())
            return;
        if (const auto parsed = parseIntegerValue(it->second))
            addNumberString(key, std::to_string(*parsed));
    };
    auto addStringFromMap = [&](const char *key) {
        const auto it = uiValues.find(key);
        if (it == uiValues.end())
            return;
        addString(key, it->second);
    };

    addBoolFromMap("zebraEnabled", uiSnapshot.zebraEnabled);
    addDoubleFromMap("zebraThreshold");
    addBoolFromMap("focusPeakingEnabled", uiSnapshot.focusPeakingEnabled);
    addDoubleFromMap("focusPeakingThreshold");
    addStringFromMap("focusPeakingColor");
    addBoolFromMap("grayscaleEnabled", uiSnapshot.grayscaleEnabled);
    addBoolFromMap("smpteEnabled", uiSnapshot.smpteEnabled);
    addBoolFromMap("anamorphicDesqueezeEnabled", false);
    addStringFromMap("anamorphicRatio");
    addBoolFromMap("falseColorEnabled", uiSnapshot.falseColorEnabled);
    addIntFromMap("falseColorMode");
    addBoolFromMap("guidesEnabled", uiSnapshot.guidesEnabled);
    addStringFromMap("guidesType");
    addIntFromMap("guidesThickness");
    addBoolFromMap("centerMarkerEnabled", uiSnapshot.centerMarkerEnabled);
    addStringFromMap("centerMarkerType");
    addString("uiOrientation",
              uiValues.count("uiOrientation")
                  ? normalizeUiOrientationValue(uiValues.at("uiOrientation"))
                  : uiSnapshot.uiOrientation);
    if (const auto it = uiValues.find("cameraControlsOpacity"); it != uiValues.end()) {
        if (const auto parsed = parseDoubleValue(it->second))
            addNumberString("cameraControlsOpacity", compactNumber(std::clamp(*parsed, 0.15, 1.0)));
        else
            addNumberString("cameraControlsOpacity", compactNumber(uiSnapshot.cameraControlsOpacity));
    } else {
        addNumberString("cameraControlsOpacity", compactNumber(uiSnapshot.cameraControlsOpacity));
    }
    addString("cameraControlsMode",
              uiValues.count("cameraControlsMode")
                  ? normalizeCameraControlsModeValue(uiValues.at("cameraControlsMode"))
                  : uiSnapshot.cameraControlsMode);
    addBoolFromMap("dateTimeOverlayEnabled", uiSnapshot.dateTimeOverlayEnabled);
    addBoolFromMap("externalMonitorEnabled", false);
    if (const auto it = uiValues.find("externalMonitorMode"); it != uiValues.end())
        addString("externalMonitorMode", it->second);
    if (const auto it = uiValues.find("externalMonitorInfoOverlay"); it != uiValues.end())
        addString("externalMonitorInfoOverlay", it->second);
    if (const auto it = uiValues.find("externalMonitorOrientation"); it != uiValues.end())
        addString("externalMonitorOrientation", normalizeExternalMonitorOrientationValue(it->second));
    addBoolFromMap("lutPreviewEnabled", uiSnapshot.lutPreviewEnabled);
    addStringFromMap("lutPreviewFile");
    addBoolFromMap("horizontalLevelEnabled", false);
    addStringFromMap("horizontalLevelColor");
    addString("timecodeMode",
              uiValues.count("timecodeMode")
                  ? normalizeTimecodeModeValue(uiValues.at("timecodeMode"))
                  : uiSnapshot.timecodeMode);
    addBoolFromMap("photoModeEnabled", false);
    addBoolFromMap("photoLongExposureEnabled", uiSnapshot.photoLongExposureEnabled);
    addBoolFromMap("photoBracketingEnabled", uiSnapshot.photoBracketingEnabled);
    addNumberString("photoBracketingEv", compactNumber(settings.bracketingEv));
    addStringFromMap("photoTimer");
    addStringFromMap("photoBurst");
    addString("photoFormat",
              uiValues.count("photoFormat")
                  ? normalizePhotoFormatValue(uiValues.at("photoFormat"))
                  : uiSnapshot.photoFormat);
    addString("photoTimelapse",
              uiValues.count("photoTimelapse")
                  ? normalizePhotoTimelapseValue(uiValues.at("photoTimelapse"))
                  : uiSnapshot.photoTimelapse);
    addBoolFromMap("soundRecorderEnabled", false);
    addBoolFromMap("recordAudioEnabled", false);
    addBoolFromMap("recordGyroEnabled", false);
    addBoolFromMap("liveAudioMonitoringEnabled", false);
    addBoolFromMap("audioMeterEnabled", false);
    addStringFromMap("audioInputDevice");
    addStringFromMap("audioOutputDevice");
    addString("audioSampleFormat", uiSnapshot.audioSampleFormat);
    addNumberString("audioSampleRate", std::to_string(uiSnapshot.audioSampleRate));
    addString("audioChannels", uiSnapshot.audioChannels);
    addIntFromMap("inputVolume");
    addIntFromMap("headphoneVolume");
    addStringFromMap("batteryType");
    addStringFromMap("batteryCapacity");
    addIntFromMap("customBatteryWh");
    addDoubleFromMap("customBatteryFullVoltage");
    addDoubleFromMap("customBatteryEmptyVoltage");
    addStringFromMap("sleepMode");
    addIntFromMap("standbyTimeoutSeconds");
    addBoolFromMap("standbyUsbPowerOff", false);

    const bool photoModeEnabled =
        uiValues.count("photoModeEnabled") ? parseUiSettingsBool(uiValues.at("photoModeEnabled"), false) : false;
    addString("cameraFps", formatPresetFps(settings.fps));
    addString("cameraIso", settings.isoAuto ? "Auto" : formatPresetIso(settings.iso));
    addNumberString("cameraExposureCompensation", compactNumber(settings.exposureCompensation));
    addString("cameraExposureMetering", settings.exposureMetering);
    addString("cameraShutterMode", photoModeEnabled ? "speed" : "angle");
    addString("cameraShutterAngle", formatPresetShutterAngle(settings.shutterUs, settings.fps, settings.shutterAuto));
    addString("cameraShutterSpeed", formatPresetShutterSpeed(settings.shutterUs, settings.shutterAuto));
    addString("cameraWhiteBalance", formatPresetWhiteBalance(settings.whiteBalanceKelvin, settings.whiteBalanceAuto));
    addString("cameraResolution", formatPresetResolution(settings.resolution.width, settings.resolution.height));
    addNumberString("cameraResolutionWidth", std::to_string(settings.resolution.width));
    addNumberString("cameraResolutionHeight", std::to_string(settings.resolution.height));
    addString("cameraRecordingFormat", toString(settings.recordingFormat));
    addString("cameraCdngCompression", toString(settings.cdngCompression));
    addString("cameraLossyDngRatio", toWireValue(settings.lossyDngRatio));
    addString("cameraProresProfile", toString(settings.proresProfile));
    addString("cameraProresColorProfile", toString(settings.proresColorProfile));
    addString("cameraCdngPreviewColorProfile", toString(settings.cdngPreviewColorProfile));
    addString("cameraTuningFile", settings.tuningFile);
    addString("cameraH264Resolution", settings.h264Resolution.width > 0 && settings.h264Resolution.height > 0
        ? settings.h264Resolution.toString() : "Match Capture");
    addString("cameraRawBitDepth", formatRawBitDepthValue(settings.rawBitDepth));
    addString("cameraAutofocusMode", toString(settings.autofocusMode));
    addString("cameraAutofocusRange", toString(settings.autofocusRange));
    addString("cameraAutofocusSpeed", toString(settings.autofocusSpeed));
    addNumberString("cameraAutofocusWindowX", jsonNumber(settings.autofocusWindowX, 3));
    addNumberString("cameraAutofocusWindowY", jsonNumber(settings.autofocusWindowY, 3));
    addNumberString("cameraAutofocusWindowWidth", jsonNumber(settings.autofocusWindowWidth, 3));
    addNumberString("cameraAutofocusWindowHeight", jsonNumber(settings.autofocusWindowHeight, 3));
    for (const char *key : {"monitoringOrder", "recordOrder", "audioOrder", "powerOrder", "systemOrder",
                           "wifiOrder", "infoOrder", "sidebarOrder", "favoritesOrder"})
        addStringFromMap(key);

    out << "}}";
    return out.str();
}

bool saveCurrentPresetToCamera(const std::string &requestedName,
                               const CaptureSettings &settings,
                               std::string *savedName,
                               std::string *savedFileName,
                               bool *overwritten,
                               std::string *errorMessage)
{
    const std::string trimmedName = trimAscii(requestedName);
    if (trimmedName.empty()) {
        if (errorMessage)
            *errorMessage = "Preset name cannot be empty.";
        return false;
    }

    const std::string normalizedRequestedName = toLowerAsciiCopy(trimmedName);
    std::filesystem::path finalPath;
    std::string fileName;
    bool replacingExisting = false;
    for (const auto &preset : discoverPresetFiles()) {
        if (toLowerAsciiCopy(preset.displayName) == normalizedRequestedName) {
            finalPath = preset.path;
            fileName = preset.fileName;
            replacingExisting = true;
            break;
        }
    }

    if (finalPath.empty()) {
        const std::filesystem::path directory = preferredPresetDirectory();
        std::error_code ec;
        std::filesystem::create_directories(directory, ec);
        if (ec) {
            if (errorMessage)
                *errorMessage = "Could not create the preset folder on the camera.";
            return false;
        }

        fileName = sanitizePresetFileStem(trimmedName) + ".apertar-preset.json";
        finalPath = directory / fileName;
    }

    if (!settingsFile::atomicWrite(finalPath, buildCurrentPresetPayload(trimmedName, settings))) {
        if (errorMessage)
            *errorMessage = "Could not save the preset on the camera.";
        return false;
    }

    if (savedName)
        *savedName = trimmedName;
    if (savedFileName)
        *savedFileName = fileName;
    if (overwritten)
        *overwritten = replacingExisting;
    return true;
}

bool deletePresetFileFromCamera(const std::string &fileName,
                                std::string *deletedName,
                                std::string *errorMessage)
{
    const auto presetPath = findPresetFileByName(fileName);
    if (!presetPath) {
        if (errorMessage)
            *errorMessage = "Preset not found.";
        return false;
    }

    const std::string displayName = presetDisplayNameForFile(*presetPath);
    std::error_code ec;
    if (!std::filesystem::remove(*presetPath, ec) || ec) {
        if (errorMessage)
            *errorMessage = "Could not delete the preset from the camera.";
        return false;
    }

    if (deletedName)
        *deletedName = displayName;
    return true;
}

} // namespace

ApertarCoreApp::ApertarCoreApp(Options options)
    : options_(std::move(options))
    , camera_(
          [this](const std::string &eventJson) { publishEvent(eventJson); },
          [this](PreviewFrameDescriptor frame) { enqueuePreviewFrame(std::move(frame)); })
    , gpioInput_([this](const std::string &action) { handleHardwareInputAction(action); },
                 [this](const std::string &action) { handleHardwareInputAction(action, false); })
    , standbyPowerButton_([this](bool doublePress) { return handlePowerButtonPress(doublePress); })
    , groveAnalogInput_([this](const GroveAnalogInput::AnalogEvent &event) { handleHardwareAnalogInput(event); })
    , neoRotary4Input_([this](const std::string &action) { handleHardwareInputAction(action); },
                       [this](const std::string &action) { handleHardwareInputAction(action, false); })
    , pcf8574Input_([this](const std::string &action) { handleHardwareInputAction(action); })
{
}

ApertarCoreApp::~ApertarCoreApp()
{
    stop();
}

bool ApertarCoreApp::start()
{
    if (running_.exchange(true))
        return true;

    shutdownRequested_ = false;
    if (!options_.simulateCamera && std::filesystem::exists("/run/apertar-standby-low-power/settings")) {
        const auto result = standbyLowPower("restore");
        if (result.exitCode != 0) {
            std::cerr << "Apertar low-power startup recovery failed: " << result.output << '\n';
            running_ = false;
            return false;
        }
    }
    if (!options_.simulateCamera && std::filesystem::exists("/run/apertar-standby-usb/ports")) {
        const auto result = standbyUsbPower("on");
        if (result.exitCode != 0)
            std::cerr << "Apertar standby USB recovery: " << result.output << '\n';
    }

    {
        std::lock_guard<std::mutex> lock(previewMutex_);
        previewPublisherRunning_ = true;
    }
    previewThread_ = std::thread(&ApertarCoreApp::runPreviewPublisher, this);

#if APERTARCORE_HAS_JPEG
    if (options_.webPort != 0) {
        {
            std::lock_guard<std::mutex> lock(mjpegMutex_);
            mjpegEncoderRunning_ = true;
            lastWebPreviewCaptureAt_ = {};
        }
        mjpegThread_ = std::thread(&ApertarCoreApp::runMjpegEncoder, this);
    }
#endif

    if (!camera_.start(options_.mediaRoot, options_.simulateCamera)) {
        stopMjpegEncoder();
        stopPreviewPublisher();
        running_ = false;
        return false;
    }

    const auto stillSettings = readSharedUiSettings();
    if (camera_.settings().photoModeEnabled != stillSettings.photoModeEnabled ||
        camera_.settings().longExposureEnabled != stillSettings.photoLongExposureEnabled ||
        (stillSettings.photoModeEnabled && photoTimelapseSecondsFromValue(stillSettings.photoTimelapse) > 0))
        camera_.setStillExposureMode(stillSettings.photoModeEnabled, stillSettings.photoLongExposureEnabled,
                                    photoTimelapseSecondsFromValue(stillSettings.photoTimelapse));
    if (camera_.settings().bracketingEnabled != stillSettings.photoBracketingEnabled ||
        std::abs(camera_.settings().bracketingEv - stillSettings.photoBracketingEv) > 0.0001) {
        auto next = camera_.settings();
        next.bracketingEnabled = stillSettings.photoBracketingEnabled;
        next.bracketingEv = stillSettings.photoBracketingEv;
        camera_.applyCapturePreset(next);
    }

    if (!controlServer_.start(options_.socketPath, [this](const std::string &line) {
            return handleRequest(line);
        })) {
        camera_.stop();
        stopMjpegEncoder();
        stopPreviewPublisher();
        running_ = false;
        return false;
    }

    if (options_.webPort != 0 &&
        !webServer_.start(options_.webBindAddress, options_.webPort, [this](const WebServer::Request &request) {
            return handleWebRequest(request);
        })) {
        controlServer_.stop();
        camera_.stop();
        stopMjpegEncoder();
        stopPreviewPublisher();
        running_ = false;
        return false;
    }

    if (!triggerOutput_.start() && !triggerOutput_.lastError().empty()) {
        std::cerr << "ApertarCore trigger output disabled: "
                  << triggerOutput_.lastError() << "\n";
    }

    pcf8574Input_.start();
    neoRotary4Input_.start();
    gpioInput_.start();
    if (!options_.simulateCamera)
        standbyPowerButton_.start();
    groveAnalogInput_.start();

    if (!recordingIndicatorOutput_.start() && !recordingIndicatorOutput_.lastError().empty()) {
        std::cerr << "ApertarCore recording indicator LED disabled: "
                  << recordingIndicatorOutput_.lastError() << "\n";
    }
    recordingIndicatorOutput_.setRecordingActive(camera_.isRecording() || soundRecordingActive());
    recordingIndicatorOutput_.setStillExposureActive(camera_.stillExposureStatus().active);
    syncRecordingTracking(camera_.isRecording());

    publishEvent("{\"event\":\"camera_state\",\"state\":\"ready\"}");
    return true;
}

void ApertarCoreApp::stop()
{
    {
        std::lock_guard<std::mutex> lock(waitMutex_);
        if (!running_.exchange(false))
            return;
    }

    publishEvent("{\"event\":\"camera_state\",\"state\":\"stopping\"}");
    pcf8574Input_.stop();
    standbyPowerButton_.stop();
    neoRotary4Input_.stop();
    gpioInput_.stop();
    groveAnalogInput_.stop();
    hardwareScriptRunner_.stop();
    recordingIndicatorOutput_.stop();
    triggerOutput_.stop();
    stopSoundRecording();
    camera_.stop();
    stopMjpegEncoder();
    stopPreviewPublisher();
    webServer_.stop();
    controlServer_.stop();
    syncRecordingTracking(false);
    joinMediaSpeedTest();
    waitCv_.notify_all();
    if (!options_.simulateCamera && std::filesystem::exists("/run/apertar-standby-low-power/settings")) {
        const auto result = standbyLowPower("restore");
        if (result.exitCode != 0)
            std::cerr << "Apertar low-power shutdown recovery: " << result.output << '\n';
    }
    if (!options_.simulateCamera && std::filesystem::exists("/run/apertar-standby-usb/ports")) {
        const auto result = standbyUsbPower("on");
        if (result.exitCode != 0)
            std::cerr << "Apertar standby USB shutdown recovery: " << result.output << '\n';
    }
}

void ApertarCoreApp::enqueuePreviewFrame(PreviewFrameDescriptor frame)
{
    if (frame.registration && frame.fd < 0)
        return;

    if (frame.registration)
        updateWebPreviewRegistration(frame);

    std::optional<PreviewFrameDescriptor> webPreviewFrame;
    if (!frame.registration && mjpegClientCount_.load(std::memory_order_relaxed) > 0)
        webPreviewFrame = frame;

    {
        std::lock_guard<std::mutex> lock(previewMutex_);
        if (!previewPublisherRunning_) {
            closeOwnedPreviewFrame(&frame);
            return;
        }

        if (frame.registration) {
            auto existing = registeredPreviewFrames_.find(frame.slotIndex);
            if (existing != registeredPreviewFrames_.end())
                closeOwnedPreviewFrame(&existing->second);

            PreviewFrameDescriptor cachedFrame = std::move(frame);
            PreviewFrameDescriptor sendFrame = duplicatePreviewFrameDescriptor(cachedFrame);
            registeredPreviewFrames_[cachedFrame.slotIndex] = std::move(cachedFrame);
            if (sendFrame.fd >= 0) {
                pendingPreviewRegistrations_.push_back(std::move(sendFrame));
                previewCv_.notify_one();
            }
            return;
        }

        if (pendingPreviewFrame_)
            closeOwnedPreviewFrame(&*pendingPreviewFrame_);

        PreviewFrameDescriptor latestMarker = frame;
        latestMarker.fd = -1;
        for (int &planeFd : latestMarker.planeFds)
            planeFd = -1;
        lastPreviewFrame_ = latestMarker;

        pendingPreviewFrame_ = std::move(frame);
        previewCv_.notify_one();
    }

    if (webPreviewFrame.has_value())
        captureWebPreviewFrame(*webPreviewFrame);
}

void ApertarCoreApp::enqueueCurrentPreviewRegistrations()
{
    std::lock_guard<std::mutex> lock(previewMutex_);
    if (!previewPublisherRunning_ || registeredPreviewFrames_.empty())
        return;

    bool queuedAny = false;
    for (const auto &[slotIndex, frame] : registeredPreviewFrames_) {
        (void)slotIndex;
        PreviewFrameDescriptor sendFrame = duplicatePreviewFrameDescriptor(frame);
        if (sendFrame.fd < 0)
            continue;
        pendingPreviewRegistrations_.push_back(std::move(sendFrame));
        queuedAny = true;
    }

    if (queuedAny)
        previewCv_.notify_one();

    if (lastPreviewFrame_) {
        if (pendingPreviewFrame_)
            closeOwnedPreviewFrame(&*pendingPreviewFrame_);
        pendingPreviewFrame_ = *lastPreviewFrame_;
        previewCv_.notify_one();
    }
}

void ApertarCoreApp::wait()
{
    std::unique_lock<std::mutex> lock(waitMutex_);
    int previousTimeout = -1;
    while (running_.load() && !shutdownRequested_.load()) {
        if (waitCv_.wait_for(lock, std::chrono::seconds(1), [this]() {
                return !running_.load() || shutdownRequested_.load();
            }))
            break;
        lock.unlock();
        {
            std::lock_guard commandLock(commandMutex_);
            if (camera_.checkRecordingFailure()) syncRecordingTracking(false);
        }
        const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const int timeout = readSharedUiSettings().standbyTimeoutSeconds;
        if (timeout != previousTimeout || camera_.isRecording() ||
            camera_.timelapseActive() || soundRecordingActive()) {
            lastUserActivityMs_.store(now);
            previousTimeout = timeout;
        }
        if (timeout > 0 && camera_.state() == CameraState::Ready &&
            now - lastUserActivityMs_.load() >= int64_t(timeout) * 1000) {
            ControlCommand command;
            command.name = "enter_standby";
            command.id = -1;
            command.fields["idle_timeout"] = std::to_string(timeout);
            handleCommand(command);
        }
        lock.lock();
    }
}

std::string ApertarCoreApp::handleRequest(const std::string &line)
{
    const ParseResult parsed = parseControlCommand(line);
    if (!parsed.ok)
        return errorReply(-1, parsed.error);

    {
        std::lock_guard<std::mutex> lock(eventMutex_);
        collectingCommandEvents_ = true;
        pendingCommandEvents_.clear();
    }

    std::string reply = handleCommand(parsed.command);

    std::vector<std::string> events;
    {
        std::lock_guard<std::mutex> lock(eventMutex_);
        events = std::move(pendingCommandEvents_);
        pendingCommandEvents_.clear();
        collectingCommandEvents_ = false;
    }

    for (const std::string &event : events) {
        reply.push_back('\n');
        reply += event;
    }

    return reply;
}

std::string ApertarCoreApp::handleWebCommand(const std::string &payload)
{
    const ParseResult parsed = parseControlCommand(payload);
    if (!parsed.ok)
        return errorReply(-1, parsed.error);
    return handleCommand(parsed.command);
}

WebServer::Response ApertarCoreApp::handleWebRequest(const WebServer::Request &request)
{
    if (request.method == "OPTIONS") {
        WebServer::Response response;
        response.status = 204;
        response.contentType = "text/plain; charset=utf-8";
        return response;
    }

    if (request.method == "GET" && (request.path == "/" || request.path == "/index.html")) {
        WebServer::Response response;
        response.contentType = "text/html; charset=utf-8";
        response.body = std::string(webControlPageHtml());
        return response;
    }

    if (request.method == "GET" && request.path == "/preview.mjpeg") {
        WebServer::Response response;
#if APERTARCORE_HAS_JPEG
        response.contentType = "multipart/x-mixed-replace; boundary=frame";
        response.streaming = true;
        response.headers.push_back({"X-Accel-Buffering", "no"});
        response.streamWriter = [this](int clientFd) { streamMjpeg(clientFd); };
#else
        response.status = 503;
        response.contentType = "application/json; charset=utf-8";
        response.body = errorReply(-1, "MJPEG preview support was not compiled in.");
#endif
        return response;
    }

    if (request.method == "GET" && request.path.rfind("/icons/", 0) == 0) {
        const std::string requestedName = request.path.substr(std::string("/icons/").size());
        if (requestedName.empty() ||
            requestedName.find('/') != std::string::npos ||
            requestedName.find('\\') != std::string::npos ||
            requestedName.find("..") != std::string::npos) {
            WebServer::Response response;
            response.status = 404;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Icon not found.");
            return response;
        }

        if (const auto iconsDir = webIconsDirectory()) {
            const std::filesystem::path iconPath = *iconsDir / requestedName;
            if (const auto bytes = readBinaryFileString(iconPath)) {
                WebServer::Response response;
                response.contentType = "image/png";
                response.body = *bytes;
                return response;
            }
        }

        WebServer::Response response;
        response.status = 404;
        response.contentType = "application/json; charset=utf-8";
        response.body = errorReply(-1, "Icon not found.");
        return response;
    }

    if (request.method == "GET" && request.path == "/api/state") {
        WebServer::Response response;
        response.contentType = "application/json; charset=utf-8";
        response.body = buildWebStateJson();
        return response;
    }

    if (request.method == "GET" && request.path == "/api/media") {
        WebServer::Response response;
        response.contentType = "application/json; charset=utf-8";
        response.body = buildMediaLibraryJson();
        return response;
    }

    if (request.method == "GET" && request.path == "/api/config") {
        WebServer::Response response;
        response.contentType = "application/json; charset=utf-8";
        response.body = buildHardwareConfigFileJson();
        return response;
    }

    if (request.method == "POST" && request.path == "/api/config") {
        std::string backupPath;
        std::string errorMessage;
        if (!saveHardwareConfigFile(request.body, &backupPath, &errorMessage)) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, errorMessage.empty() ? "Could not save config file." : errorMessage);
            return response;
        }

        WebServer::Response response;
        response.contentType = "application/json; charset=utf-8";
        response.body = okReply(
            -1,
            "\"path\":\"" + escapeJson(editableHardwareConfigPath().string()) + "\""
            + ",\"display_path\":\"" + escapeJson(hardwareConfigDisplayPath(editableHardwareConfigPath())) + "\""
            + ",\"backup_path\":\"" + escapeJson(backupPath) + "\""
            + ",\"backup_display_path\":\"" + escapeJson(backupPath.empty() ? std::string() : hardwareConfigDisplayPath(backupPath)) + "\"");
        return response;
    }

    if (request.method == "GET" && request.path == "/api/media/download") {
        const auto requestedKind = queryParameterValue(request.query, "kind");
        const auto requestedId = queryParameterValue(request.query, "id");
        if (!requestedKind || !requestedId) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Media item is missing.");
            return response;
        }

        const std::string kind = *requestedKind;
        const std::string id = *requestedId;
        const std::filesystem::path mediaRoot(options_.mediaRoot);

        if (kind == "folder") {
            const auto itemPath = resolveMediaLibraryPath(mediaRoot, id);
            if (!itemPath || !std::filesystem::is_directory(*itemPath)) {
                WebServer::Response response;
                response.status = 404;
                response.contentType = "application/json; charset=utf-8";
                response.body = errorReply(-1, "Media item not found.");
                return response;
            }

            if (zipProgramPath().empty()) {
                WebServer::Response response;
                response.status = 503;
                response.contentType = "application/json; charset=utf-8";
                response.body = errorReply(-1, "Folder downloads need the zip package installed.");
                return response;
            }

            WebServer::Response response;
            const std::string folderName = itemPath->filename().string();
            std::error_code ec;
            const auto relative = std::filesystem::relative(*itemPath, mediaRoot, ec);
            if (ec || relative.empty() || !isSafeMediaRelativeId(relative.lexically_normal().string())) {
                WebServer::Response response;
                response.status = 404;
                response.contentType = "application/json; charset=utf-8";
                response.body = errorReply(-1, "Media item not found.");
                return response;
            }
            const std::string relativeEntry = relative.lexically_normal().string();
            response.contentType = "application/zip";
            response.streaming = true;
            response.headers.push_back({
                "Content-Disposition",
                "attachment; filename=\"" + sanitizeDownloadFileName(folderName) + ".zip\""
            });
            response.streamWriter = [mediaRoot, relativeEntry](int clientFd) {
                streamZipArchiveToClient(clientFd, mediaRoot, {relativeEntry});
            };
            return response;
        }

        if (kind == "still") {
            const std::vector<std::filesystem::path> files = stillGroupFilesForId(mediaRoot, id);
            if (files.empty()) {
                WebServer::Response response;
                response.status = 404;
                response.contentType = "application/json; charset=utf-8";
                response.body = errorReply(-1, "Still not found.");
                return response;
            }

            if (files.size() == 1) {
                WebServer::Response response;
                const std::filesystem::path filePath = files.front();
                response.contentType = mediaContentTypeForFile(filePath);
                response.streaming = true;
                response.headers.push_back({
                    "Content-Disposition",
                    "attachment; filename=\"" + sanitizeDownloadFileName(filePath.filename().string()) + "\""
                });
                response.streamWriter = [filePath](int clientFd) {
                    streamFileToClient(clientFd, filePath);
                };
                return response;
            }

            if (zipProgramPath().empty()) {
                WebServer::Response response;
                response.status = 503;
                response.contentType = "application/json; charset=utf-8";
                response.body = errorReply(-1, "Grouped still downloads need the zip package installed.");
                return response;
            }

            std::vector<std::string> relativeFiles;
            relativeFiles.reserve(files.size());
            std::error_code ec;
            for (const auto &file : files) {
                const auto relative = std::filesystem::relative(file, mediaRoot, ec);
                if (!ec)
                    relativeFiles.push_back(relative.string());
                ec.clear();
            }

            WebServer::Response response;
            const std::string stemName = std::filesystem::path(id).filename().string();
            response.contentType = "application/zip";
            response.streaming = true;
            response.headers.push_back({
                "Content-Disposition",
                "attachment; filename=\"" + sanitizeDownloadFileName(stemName) + ".zip\""
            });
            response.streamWriter = [mediaRoot, relativeFiles](int clientFd) {
                streamZipArchiveToClient(clientFd, mediaRoot, relativeFiles);
            };
            return response;
        }

        WebServer::Response response;
        response.status = 400;
        response.contentType = "application/json; charset=utf-8";
        response.body = errorReply(-1, "Unsupported media item type.");
        return response;
    }

    if (request.method == "GET" && request.path == "/api/media/download-selected") {
        std::string errorMessage;
        const std::vector<MediaSelection> selections = mediaSelectionsFromQuery(request.query, &errorMessage);
        if (selections.empty()) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, errorMessage.empty() ? "Select at least one media item." : errorMessage);
            return response;
        }

        const std::filesystem::path mediaRoot(options_.mediaRoot);
        const auto relativeEntries = mediaSelectionZipEntries(mediaRoot, selections, &errorMessage);
        if (!relativeEntries) {
            WebServer::Response response;
            response.status = 404;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, errorMessage.empty() ? "Selected media not found." : errorMessage);
            return response;
        }

        if (zipProgramPath().empty()) {
            WebServer::Response response;
            response.status = 503;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Selected downloads need the zip package installed.");
            return response;
        }

        WebServer::Response response;
        response.contentType = "application/zip";
        response.streaming = true;
        response.headers.push_back({
            "Content-Disposition",
            "attachment; filename=\"Apertar-Media-Selection.zip\""
        });
        response.streamWriter = [mediaRoot, entries = *relativeEntries](int clientFd) {
            streamZipArchiveToClient(clientFd, mediaRoot, entries);
        };
        return response;
    }

    if (request.method == "GET" && request.path == "/api/presets") {
        WebServer::Response response;
        response.contentType = "application/json; charset=utf-8";
        response.body = buildPresetLibraryJson();
        return response;
    }

    if (request.method == "GET" && request.path.rfind("/api/presets/download/", 0) == 0) {
        const std::string encodedFileName =
            request.path.substr(std::string("/api/presets/download/").size());
        const std::string fileName = urlDecodeComponent(encodedFileName);

        if (const auto presetPath = findPresetFileByName(fileName)) {
            if (const auto bytes = readBinaryFileString(*presetPath)) {
                WebServer::Response response;
                response.contentType = "application/json; charset=utf-8";
                response.body = *bytes;
                response.headers.push_back({
                    "Content-Disposition",
                    "attachment; filename=\"" + fileName + "\""
                });
                return response;
            }
        }

        WebServer::Response response;
        response.status = 404;
        response.contentType = "application/json; charset=utf-8";
        response.body = errorReply(-1, "Preset not found.");
        return response;
    }

    if (request.method == "POST" &&
        (request.path == "/api/presets/import" || request.path.rfind("/api/presets/import/", 0) == 0)) {
        std::string uploadedFileName;
        if (request.path == "/api/presets/import") {
            if (const auto fromQuery = queryParameterValue(request.query, "name")) {
                uploadedFileName = *fromQuery;
            } else if (const auto headerIt = request.headers.find("x-preset-filename");
                       headerIt != request.headers.end()) {
                uploadedFileName = headerIt->second;
            }
        } else {
            const std::string encodedFileName =
                request.path.substr(std::string("/api/presets/import/").size());
            uploadedFileName = urlDecodeComponent(encodedFileName);
        }

        if (uploadedFileName.empty()) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Preset file name is missing.");
            return response;
        }

        std::string importedName;
        bool alreadyExists = false;
        std::string errorMessage;
        if (!importPresetFileToCamera(uploadedFileName,
                                      request.body,
                                      &importedName,
                                      &alreadyExists,
                                      &errorMessage)) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, errorMessage);
            return response;
        }

        WebServer::Response response;
        response.contentType = "application/json; charset=utf-8";
        response.body = okReply(
            -1,
            "\"imported_name\":\"" + escapeJson(importedName) + "\""
            + std::string(",\"already_exists\":") + (alreadyExists ? "true" : "false"));
        return response;
    }

    if (request.method == "POST" && request.path == "/api/presets/save") {
        const auto requestedName = queryParameterValue(request.query, "name");
        if (!requestedName || trimAscii(*requestedName).empty()) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Preset name is missing.");
            return response;
        }

        std::string savedName;
        std::string savedFileName;
        bool overwritten = false;
        std::string errorMessage;
        if (!saveCurrentPresetToCamera(*requestedName,
                                       camera_.settings(),
                                       &savedName,
                                       &savedFileName,
                                       &overwritten,
                                       &errorMessage)) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, errorMessage);
            return response;
        }

        WebServer::Response response;
        response.contentType = "application/json; charset=utf-8";
        response.body = okReply(
            -1,
            "\"saved_name\":\"" + escapeJson(savedName) + "\""
            + ",\"file_name\":\"" + escapeJson(savedFileName) + "\""
            + std::string(",\"overwritten\":") + (overwritten ? "true" : "false"));
        return response;
    }

    if (request.method == "POST" && request.path == "/api/presets/delete") {
        const auto requestedFile = queryParameterValue(request.query, "file");
        if (!requestedFile || requestedFile->empty()) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Preset file name is missing.");
            return response;
        }

        std::string deletedName;
        std::string errorMessage;
        if (!deletePresetFileFromCamera(*requestedFile, &deletedName, &errorMessage)) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, errorMessage);
            return response;
        }

        WebServer::Response response;
        response.contentType = "application/json; charset=utf-8";
        response.body = okReply(-1, "\"deleted_name\":\"" + escapeJson(deletedName) + "\"");
        return response;
    }

    if (request.method == "POST" && request.path == "/api/media/delete") {
        const auto requestedKind = queryParameterValue(request.query, "kind");
        const auto requestedId = queryParameterValue(request.query, "id");
        if (!requestedKind || !requestedId) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Media item is missing.");
            return response;
        }

        if (camera_.isRecording() || soundRecordingActive() || camera_.timelapseActive()) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Stop recording before deleting media.");
            return response;
        }

        const std::string kind = *requestedKind;
        const std::string id = *requestedId;
        const std::filesystem::path mediaRoot(options_.mediaRoot);
        std::error_code ec;
        uintmax_t removedCount = 0;

        if (kind == "folder") {
            const auto itemPath = resolveMediaLibraryPath(mediaRoot, id);
            if (!itemPath || !std::filesystem::is_directory(*itemPath)) {
                WebServer::Response response;
                response.status = 404;
                response.contentType = "application/json; charset=utf-8";
                response.body = errorReply(-1, "Media item not found.");
                return response;
            }
            removedCount = std::filesystem::remove_all(*itemPath, ec);
        } else if (kind == "still") {
            const std::vector<std::filesystem::path> files = stillGroupFilesForId(mediaRoot, id);
            if (files.empty()) {
                WebServer::Response response;
                response.status = 404;
                response.contentType = "application/json; charset=utf-8";
                response.body = errorReply(-1, "Still not found.");
                return response;
            }
            for (const auto &file : files) {
                if (std::filesystem::remove(file, ec))
                    ++removedCount;
                if (ec)
                    break;
            }
        } else {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Unsupported media item type.");
            return response;
        }

        if (ec) {
            WebServer::Response response;
            response.status = 500;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Could not delete media: " + ec.message());
            return response;
        }

        WebServer::Response response;
        response.contentType = "application/json; charset=utf-8";
        response.body = okReply(-1, "\"removed_count\":" + std::to_string(static_cast<uint64_t>(removedCount)));
        return response;
    }

    if (request.method == "POST" && request.path == "/api/media/delete-selected") {
        std::string errorMessage;
        const std::vector<MediaSelection> selections = mediaSelectionsFromQuery(request.query, &errorMessage);
        if (selections.empty()) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, errorMessage.empty() ? "Select at least one media item." : errorMessage);
            return response;
        }

        if (camera_.isRecording() || soundRecordingActive() || camera_.timelapseActive()) {
            WebServer::Response response;
            response.status = 400;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Stop recording before deleting media.");
            return response;
        }

        const std::filesystem::path mediaRoot(options_.mediaRoot);
        const auto targets = mediaSelectionDeletionTargets(mediaRoot, selections, &errorMessage);
        if (!targets) {
            WebServer::Response response;
            response.status = 404;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, errorMessage.empty() ? "Selected media not found." : errorMessage);
            return response;
        }

        std::error_code ec;
        uintmax_t removedCount = 0;
        for (const auto &folder : targets->folders) {
            removedCount += std::filesystem::remove_all(folder, ec);
            if (ec)
                break;
        }
        if (!ec) {
            for (const auto &file : targets->files) {
                if (std::filesystem::remove(file, ec))
                    ++removedCount;
                if (ec)
                    break;
            }
        }

        if (ec) {
            WebServer::Response response;
            response.status = 500;
            response.contentType = "application/json; charset=utf-8";
            response.body = errorReply(-1, "Could not delete selected media: " + ec.message());
            return response;
        }

        WebServer::Response response;
        response.contentType = "application/json; charset=utf-8";
        response.body = okReply(
            -1,
            "\"removed_count\":" + std::to_string(static_cast<uint64_t>(removedCount)) +
            ",\"item_count\":" + std::to_string(static_cast<uint64_t>(selections.size())));
        return response;
    }

    if (request.method == "POST" && request.path == "/api/command") {
        WebServer::Response response;
        response.contentType = "application/json; charset=utf-8";
        response.body = handleWebCommand(request.body);
        return response;
    }

    WebServer::Response response;
    response.status = request.method == "GET" || request.method == "POST" ? 404 : 405;
    response.contentType = "application/json; charset=utf-8";
    response.body = errorReply(-1, response.status == 404 ? "Not found." : "Method not allowed.");
    return response;
}

std::string ApertarCoreApp::buildPresetLibraryJson() const
{
    const auto presets = discoverPresetFiles();
    std::ostringstream out;
    out << "{\"ok\":true,\"presets\":[";
    for (size_t i = 0; i < presets.size(); ++i) {
        const auto &preset = presets[i];
        if (i > 0)
            out << ',';
        out << '{'
            << "\"name\":\"" << escapeJson(preset.displayName) << "\""
            << ",\"file_name\":\"" << escapeJson(preset.fileName) << "\""
            << ",\"size_bytes\":" << preset.sizeBytes
            << ",\"size_text\":\"" << escapeJson(preset.sizeText) << "\""
            << ",\"modified_text\":\"" << escapeJson(preset.modifiedText) << "\""
            << ",\"download_url\":\"" << escapeJson(preset.downloadUrl) << "\""
            << '}';
    }
    out << "]}";
    return out.str();
}

std::string ApertarCoreApp::buildMediaLibraryJson() const
{
    const CaptureSettings settings = camera_.settings();
    const bool mediaMounted = isMountedPath(options_.mediaRoot);
    const std::vector<MediaLibraryItem> items = mediaMounted
        ? discoverMediaLibraryItems(std::filesystem::path(options_.mediaRoot), settings.fps)
        : std::vector<MediaLibraryItem>{};

    std::ostringstream out;
    out << "{\"ok\":true"
        << ",\"mounted\":" << (mediaMounted ? "true" : "false")
        << ",\"media_root\":\"" << escapeJson(options_.mediaRoot) << "\""
        << ",\"items\":[";
    for (size_t i = 0; i < items.size(); ++i) {
        const auto &item = items[i];
        if (i > 0)
            out << ',';
        out << '{'
            << "\"id\":\"" << escapeJson(item.id) << "\""
            << ",\"kind\":\"" << escapeJson(item.kind) << "\""
            << ",\"category\":\"" << escapeJson(item.category) << "\""
            << ",\"type\":\"" << escapeJson(item.type) << "\""
            << ",\"name\":\"" << escapeJson(item.name) << "\""
            << ",\"size_bytes\":" << item.sizeBytes
            << ",\"size_text\":\"" << escapeJson(item.sizeText) << "\""
            << ",\"file_count\":" << item.fileCount
            << ",\"frame_count\":" << item.frameCount
            << ",\"duration_text\":\"" << escapeJson(item.durationText) << "\""
            << ",\"fps_text\":\"" << escapeJson(item.fpsText) << "\""
            << ",\"modified_text\":\"" << escapeJson(item.modifiedText) << "\""
            << ",\"modified_ms\":" << item.modifiedMs
            << ",\"download_url\":\"" << escapeJson(item.downloadUrl) << "\""
            << '}';
    }
    out << "]}";
    return out.str();
}

std::string ApertarCoreApp::buildWebStateJson() const
{
    const auto mediaWrite = camera_.mediaWriteStatus();
    const CameraState cameraState = camera_.state();
    const CaptureSettings settings = camera_.settings();
    const bool soundRecording = soundRecordingActive();
    const bool recording = camera_.isRecording() || soundRecording;
    const bool timelapseActive = camera_.timelapseActive();
    const double timelapseIntervalSeconds = camera_.timelapseIntervalSeconds();
    const std::string sensorName = camera_.sensorName();
    const bool mediaMounted = isMountedPath(options_.mediaRoot);
    const std::string mountedDevice = mediaMounted ? mountedDeviceForPath(options_.mediaRoot) : std::string();
    const std::string mountedVolumeLabel = mediaMounted ? volumeLabelForDevicePath(mountedDevice) : std::string();
    const std::string mediaTypeLabel = mediaMounted
        ? mediaTypeLabelForDeviceAndVolumeLabel(mountedDevice, mountedVolumeLabel)
        : std::string("No Media Mounted");
    const std::vector<MediaDriveCandidate> mountableMediaDrives = scanMountableMediaDrives(options_.mediaRoot);
    const SharedUiSettingsSnapshot uiSettings = readSharedUiSettings();
    const std::vector<std::string> lutPreviewOptions = discoverLutPreviewFiles(options_.mediaRoot);
    const AudioDeviceSnapshot audioDevices = detectAudioDevices();
    const std::string resolvedAudioInputDevice =
        resolveAudioDeviceSelection(uiSettings.audioInputDevice, audioDevices.inputOptions, false);
    const std::string resolvedAudioOutputDevice =
        resolveAudioDeviceSelection(uiSettings.audioOutputDevice, audioDevices.outputOptions, true);
    const NetworkSnapshot network = readNetworkSnapshot(options_.webPort);
    const DeviceSnapshot device = readDeviceSnapshot();
    const PowerSupplySnapshot power = readPowerSupplySnapshot();
    const std::optional<double> cpuTempC = readCpuTemperatureC();
    const std::string currentTimeZone = currentTimeZoneId();
    const std::vector<std::string> &timeZones = availableTimeZones();
    const std::string localDateTimeValue = currentLocalDateTimeInputValue();
    const uint64_t localMillisecondsOfDay = currentLocalMillisecondsOfDay();
    const std::string fanMode = currentFanMode();

    bool mediaSpeedRunning = false;
    bool mediaSpeedCurrentUsedFio = false;
    bool mediaSpeedLastUsedFio = false;
    uint64_t mediaSpeedCurrentBytes = 0;
    uint64_t mediaSpeedLastBytes = 0;
    double mediaSpeedLastWriteSpeedMBps = 0.0;
    std::string mediaSpeedError;
    {
        std::lock_guard<std::mutex> lock(mediaSpeedMutex_);
        mediaSpeedRunning = mediaSpeedTestRunning_;
        mediaSpeedCurrentUsedFio = mediaSpeedCurrentUsedFio_;
        mediaSpeedLastUsedFio = mediaSpeedLastUsedFio_;
        mediaSpeedCurrentBytes = mediaSpeedCurrentBytes_;
        mediaSpeedLastBytes = mediaSpeedLastBytes_;
        mediaSpeedLastWriteSpeedMBps = mediaSpeedLastWriteSpeedMBps_;
        mediaSpeedError = mediaSpeedError_;
    }

    std::optional<double> cpuUsagePercent;
    if (const auto cpuTicks = readCpuTicks()) {
        std::lock_guard<std::mutex> lock(telemetryMutex_);
        if (cpuSampleValid_) {
            const uint64_t idleDelta = cpuTicks->first - lastCpuIdleTicks_;
            const uint64_t totalDelta = cpuTicks->second - lastCpuTotalTicks_;
            if (totalDelta > 0) {
                cpuUsagePercent = std::clamp(
                    (1.0 - (static_cast<double>(idleDelta) / static_cast<double>(totalDelta))) * 100.0,
                    0.0,
                    100.0);
            }
        }
        lastCpuIdleTicks_ = cpuTicks->first;
        lastCpuTotalTicks_ = cpuTicks->second;
        cpuSampleValid_ = true;
    }

    std::error_code spaceEc;
    const std::filesystem::path existingMediaPath = mediaMounted
        ? nearestExistingPath(options_.mediaRoot)
        : std::filesystem::path();
    const auto mediaSpace = (!mediaMounted || existingMediaPath.empty())
        ? std::filesystem::space_info{}
        : std::filesystem::space(existingMediaPath, spaceEc);
    const uint64_t mediaFreeBytes = spaceEc ? 0 : mediaSpace.available;
    const uint64_t mediaTotalBytes = spaceEc ? 0 : mediaSpace.capacity;
    const uint64_t mediaUsedBytes = mediaTotalBytes > mediaFreeBytes ? (mediaTotalBytes - mediaFreeBytes) : 0;
    std::optional<double> mediaUsedPercent;
    if (mediaTotalBytes > 0) {
        mediaUsedPercent = std::clamp(
            (static_cast<double>(mediaUsedBytes) / static_cast<double>(mediaTotalBytes)) * 100.0,
            0.0,
            100.0);
    }

    const uint64_t estimatedBytesPerSecond = estimatedRawBytesPerSecond(settings);
    std::optional<double> recordTimeLeftMinutes;
    if (estimatedBytesPerSecond > 0 && mediaFreeBytes > 0) {
        recordTimeLeftMinutes =
            static_cast<double>(mediaFreeBytes) /
            static_cast<double>(estimatedBytesPerSecond) /
            60.0;
    }
    std::optional<double> recordStillsLeft;
    if (mediaFreeBytes > 0) {
        const double estimatedStillFrameBytes =
            estimatedStillFrameSizeMiB(settings, sensorName) * 1024.0 * 1024.0;
        if (estimatedStillFrameBytes > 0.0)
            recordStillsLeft = std::floor(static_cast<double>(mediaFreeBytes) / estimatedStillFrameBytes);
    }

    std::optional<double> timeLeftPercent;
    if (recordTimeLeftMinutes.has_value())
        timeLeftPercent = std::clamp((recordTimeLeftMinutes.value() / 120.0) * 100.0, 0.0, 100.0);

    std::optional<double> batteryPercent;
    if (power.capacityPercent.has_value()) {
        batteryPercent = power.capacityPercent.value();
    } else if (power.voltageV.has_value()) {
        batteryPercent = std::clamp(((power.voltageV.value() - 10.0) / 6.8) * 100.0, 0.0, 100.0);
    }

    std::optional<double> cpuTempPercent;
    if (cpuTempC.has_value())
        cpuTempPercent = std::clamp((cpuTempC.value() / 100.0) * 100.0, 0.0, 100.0);

    std::optional<double> powerPercent;
    if (power.powerW.has_value())
        powerPercent = std::clamp((power.powerW.value() / 15.0) * 100.0, 0.0, 100.0);

    std::ostringstream out;
    out << "{"
        << "\"state\":\"" << toString(cameraState) << "\""
        << ",\"standby\":" << (cameraState == CameraState::Standby ? "true" : "false")
        << ",\"standby_timeout_seconds\":" << uiSettings.standbyTimeoutSeconds
        << ",\"standby_usb_power_off\":" << (uiSettings.standbyUsbPowerOff ? "true" : "false")
        << ",\"standby_usb_powered_off\":" << (standbyUsbPoweredOff_.load() ? "true" : "false")
        << ",\"recording\":" << (recording ? "true" : "false")
        << ",\"recording_error\":\"" << escapeJson(camera_.recordingError()) << "\""
        << ",\"recording_has_dropped_frames\":" << (camera_.recordingHasDroppedFrames() ? "true" : "false")
        << ",\"sound_recording\":" << (soundRecording ? "true" : "false")
        << ",\"timelapse_active\":" << (timelapseActive ? "true" : "false")
        << ",\"timelapse_interval_seconds\":" << timelapseIntervalSeconds
        << ",\"timelapse_shot_count\":" << camera_.timelapseShotCount()
        << ",\"still_exposure_active\":" << (camera_.stillExposureStatus().active ? "true" : "false")
        << ",\"still_exposure_remaining_seconds\":" << camera_.stillExposureStatus().remainingSeconds
        << ",\"photo_write_error\":\"" << escapeJson(camera_.photoWriteError()) << "\""
        << ",\"media_writes_pending\":" << (mediaWrite.pending ? "true" : "false")
        << ",\"media_write_revision\":" << mediaWrite.revision
        << ",\"recording_elapsed_seconds\":" << jsonNumber(recordingElapsedSeconds(), 2)
        << ",\"timecode_local_ms_of_day\":" << localMillisecondsOfDay
        << ",\"backend_name\":\"Apertar-Core / libcamera\""
        << ",\"sensor\":\"" << escapeJson(sensorName) << "\""
        << ",\"resolution\":\"" << settings.resolution.toString() << "\""
        << ",\"width\":" << settings.resolution.width
        << ",\"height\":" << settings.resolution.height
        << ",\"fps\":" << jsonNumber(settings.fps, 3)
        << ",\"iso\":" << settings.iso
        << ",\"iso_auto\":" << (settings.isoAuto ? "true" : "false")
        << ",\"shutter_us\":" << settings.shutterUs
        << ",\"shutter_auto\":" << (settings.shutterAuto ? "true" : "false")
        << ",\"exposure_compensation\":" << settings.exposureCompensation
        << ",\"exposure_compensation_available\":" << (settings.exposureCompensationAvailable ? "true" : "false")
        << "," << exposureMeteringJson(settings)
        << ",\"wb_kelvin\":" << settings.whiteBalanceKelvin
        << ",\"wb_auto\":" << (settings.whiteBalanceAuto ? "true" : "false")
        << ",\"wb_available\":" << (settings.whiteBalanceAvailable ? "true" : "false")
        << ",\"timecode_mode\":\"" << toString(settings.timecodeMode) << "\""
        << ",\"wb_measured\":" << (settings.whiteBalanceMeasured.valid() ? "true" : "false")
        << ",\"wb_actual_kelvin\":" << settings.whiteBalanceMeasured.kelvin
        << ",\"wb_actual_red_gain\":" << settings.whiteBalanceMeasured.red
        << ",\"wb_actual_blue_gain\":" << settings.whiteBalanceMeasured.blue
        << ",\"recording_capabilities\":" << recordingCapabilitiesJson()
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
        << ",\"af_mode\":\"" << toString(settings.autofocusMode) << "\""
        << ",\"af_range\":\"" << toString(settings.autofocusRange) << "\""
        << ",\"af_speed\":\"" << toString(settings.autofocusSpeed) << "\""
        << ",\"af_window_x\":" << jsonNumber(settings.autofocusWindowX, 3)
        << ",\"af_window_y\":" << jsonNumber(settings.autofocusWindowY, 3)
        << ",\"af_window_w\":" << jsonNumber(settings.autofocusWindowWidth, 3)
        << ",\"af_window_h\":" << jsonNumber(settings.autofocusWindowHeight, 3)
        << ",\"media_mounted\":" << (mediaMounted ? "true" : "false")
        << ",\"network_transport\":\"" << escapeJson(network.transport) << "\""
        << ",\"network_host\":\"" << escapeJson(network.hostLabel) << "\""
        << ",\"system\":{"
        << "\"cpu_usage_percent\":" << jsonNumberOrNull(cpuUsagePercent, 1)
        << ",\"cpu_usage_label\":\"" << std::thread::hardware_concurrency() << " cores active\""
        << ",\"cpu_temp_c\":" << jsonNumberOrNull(cpuTempC, 1)
        << ",\"cpu_temp_percent\":" << jsonNumberOrNull(cpuTempPercent, 1)
        << ",\"cpu_temp_label\":\""
        << (cpuTempC ? (*cpuTempC < 70.0 ? "Normal" : (*cpuTempC < 82.0 ? "Warm" : "Hot")) : "Unavailable")
        << "\""
        << ",\"power_draw_w\":" << jsonNumberOrNull(power.powerW, 1)
        << ",\"power_draw_percent\":" << jsonNumberOrNull(powerPercent, 1)
        << ",\"power_draw_label\":\"" << (power.powerW ? "System input" : "No power sensor") << "\""
        << ",\"battery_voltage_v\":" << jsonNumberOrNull(power.voltageV, 2)
        << ",\"battery_current_a\":" << jsonNumberOrNull(power.currentA, 2)
        << ",\"battery_percent\":" << jsonNumberOrNull(batteryPercent, 1)
        << ",\"battery_label\":\"" << (power.voltageV ? "Input rail" : "No battery sensor") << "\""
        << ",\"power_sensor_available\":"
        << ((power.voltageV.has_value() || power.powerW.has_value() || power.capacityPercent.has_value() || power.currentA.has_value()) ? "true" : "false")
        << ",\"media_free_bytes\":" << mediaFreeBytes
        << ",\"media_total_bytes\":" << mediaTotalBytes
        << ",\"media_used_percent\":" << jsonNumberOrNull(mediaUsedPercent, 1)
        << ",\"media_label\":\"" << escapeJson(mediaMounted ? options_.mediaRoot : "Mount /media/RAW to manage media") << "\""
        << ",\"media_drives\":" << jsonMediaDriveArray(mountableMediaDrives)
        << ",\"record_time_left_minutes\":" << jsonNumberOrNull(recordTimeLeftMinutes, 1)
        << ",\"record_stills_left\":" << jsonNumberOrNull(recordStillsLeft, 0)
        << ",\"time_left_percent\":" << jsonNumberOrNull(timeLeftPercent, 1)
        << ",\"time_left_label\":\"Current format estimate\""
        << ",\"time_zone\":\"" << escapeJson(currentTimeZone) << "\""
        << ",\"time_zone_options\":" << jsonStringArray(timeZones)
        << ",\"local_time_input\":\"" << escapeJson(localDateTimeValue) << "\""
        << ",\"fan_mode\":\"" << escapeJson(fanMode) << "\""
        << ",\"write_speed_running\":" << (mediaSpeedRunning ? "true" : "false")
        << ",\"write_speed_result_text\":\""
        << escapeJson(writeSpeedResultText(mediaMounted,
                                           mediaSpeedRunning,
                                           mediaSpeedError,
                                           mediaSpeedLastWriteSpeedMBps))
        << "\""
        << ",\"write_speed_detail_text\":\""
        << escapeJson(writeSpeedDetailText(mediaMounted,
                                           mediaSpeedRunning,
                                           mediaSpeedError,
                                           mediaSpeedCurrentBytes,
                                           mediaSpeedLastBytes,
                                           mediaSpeedCurrentUsedFio,
                                           mediaSpeedLastUsedFio,
                                           mediaSpeedLastWriteSpeedMBps))
        << "\""
        << ",\"write_speed_error\":\"" << escapeJson(mediaSpeedError) << "\""
        << "}"
        << ",\"capture\":{"
        << "\"codec\":\"" << (settings.recordingFormat == RecordingFormat::Cdng
                                 ? "CinemaDNG RAW"
                                 : (settings.recordingFormat == RecordingFormat::Prores ? "ProRes 422"
                                    : settings.recordingFormat == RecordingFormat::H264 ? "H.264"
                                    : "MLV RAW")) << "\""
        << ",\"sensor\":\"" << escapeJson(sensorName) << "\""
        << ",\"storage\":\"" << escapeJson(mediaTypeLabel) << "\""
#if APERTARCORE_HAS_JPEG
        << ",\"preview\":\"MJPEG preview\""
#else
        << ",\"preview\":\"Preview unavailable\""
#endif
        << ",\"network\":\"" << escapeJson(network.captureLabel) << "\""
        << "}"
        << ",\"ui\":{"
        << "\"timecode_mode\":\"" << escapeJson(uiSettings.timecodeMode) << "\""
        << ",\"focus_peaking_enabled\":" << (uiSettings.focusPeakingEnabled ? "true" : "false")
        << ",\"focus_peaking_threshold\":" << jsonNumber(uiSettings.focusPeakingThreshold, 2)
        << ",\"focus_peaking_color\":\"" << escapeJson(uiSettings.focusPeakingColor) << "\""
        << ",\"zebra_enabled\":" << (uiSettings.zebraEnabled ? "true" : "false")
        << ",\"zebra_threshold\":" << jsonNumber(uiSettings.zebraThreshold, 2)
        << ",\"anamorphic_desqueeze_enabled\":" << (uiSettings.anamorphicDesqueezeEnabled ? "true" : "false")
        << ",\"anamorphic_ratio\":\"" << escapeJson(uiSettings.anamorphicRatio) << "\""
        << ",\"false_color_enabled\":" << (uiSettings.falseColorEnabled ? "true" : "false")
        << ",\"false_color_mode\":" << uiSettings.falseColorMode
        << ",\"lut_preview_enabled\":" << (uiSettings.lutPreviewEnabled ? "true" : "false")
        << ",\"lut_preview_file\":\"" << escapeJson(uiSettings.lutPreviewFile) << "\""
        << ",\"lut_preview_options\":" << jsonStringArray(lutPreviewOptions)
        << ",\"guides_enabled\":" << (uiSettings.guidesEnabled ? "true" : "false")
        << ",\"guides_type\":\"" << escapeJson(uiSettings.guidesType) << "\""
        << ",\"guides_thickness\":" << uiSettings.guidesThickness
        << ",\"center_marker_enabled\":" << (uiSettings.centerMarkerEnabled ? "true" : "false")
        << ",\"center_marker_type\":\"" << escapeJson(uiSettings.centerMarkerType) << "\""
        << ",\"grayscale_enabled\":" << (uiSettings.grayscaleEnabled ? "true" : "false")
        << ",\"smpte_enabled\":" << (uiSettings.smpteEnabled ? "true" : "false")
        << ",\"external_monitor_enabled\":" << (uiSettings.externalMonitorEnabled ? "true" : "false")
        << ",\"external_monitor_mode\":\"" << escapeJson(uiSettings.externalMonitorMode) << "\""
        << ",\"external_monitor_info_overlay\":\"" << escapeJson(uiSettings.externalMonitorInfoOverlay) << "\""
        << ",\"external_monitor_orientation\":\"" << escapeJson(uiSettings.externalMonitorOrientation) << "\""
        << ",\"ui_orientation\":\"" << escapeJson(uiSettings.uiOrientation) << "\""
        << ",\"camera_controls_opacity\":" << jsonNumber(uiSettings.cameraControlsOpacity, 2)
        << ",\"camera_controls_mode\":\"" << escapeJson(uiSettings.cameraControlsMode) << "\""
        << ",\"date_time_overlay_enabled\":" << (uiSettings.dateTimeOverlayEnabled ? "true" : "false")
        << ",\"photo_mode_enabled\":" << (uiSettings.photoModeEnabled ? "true" : "false")
        << ",\"photo_long_exposure_enabled\":" << (uiSettings.photoLongExposureEnabled ? "true" : "false")
        << ",\"photo_bracketing_enabled\":" << (uiSettings.photoBracketingEnabled ? "true" : "false")
        << ",\"photo_bracketing_ev\":" << settings.bracketingEv
        << ",\"photo_timer\":\"" << escapeJson(uiSettings.photoTimer) << "\""
        << ",\"photo_burst\":\"" << escapeJson(uiSettings.photoBurst) << "\""
        << ",\"photo_format\":\"" << escapeJson(uiSettings.photoFormat) << "\""
        << ",\"photo_timelapse\":\"" << escapeJson(uiSettings.photoTimelapse) << "\""
        << ",\"sound_recorder_enabled\":" << (uiSettings.soundRecorderEnabled ? "true" : "false")
        << ",\"record_audio_enabled\":" << (uiSettings.recordAudioEnabled ? "true" : "false")
        << ",\"record_gyro_enabled\":" << (uiSettings.recordGyroEnabled ? "true" : "false")
        << ",\"live_audio_monitoring_enabled\":" << (uiSettings.liveAudioMonitoringEnabled ? "true" : "false")
        << ",\"audio_meter_enabled\":" << (uiSettings.audioMeterEnabled ? "true" : "false")
        << ",\"audio_input_device\":\"" << escapeJson(resolvedAudioInputDevice) << "\""
        << ",\"audio_output_device\":\"" << escapeJson(resolvedAudioOutputDevice) << "\""
        << ",\"audio_sample_format\":\"" << escapeJson(uiSettings.audioSampleFormat) << "\""
        << ",\"audio_sample_rate\":" << uiSettings.audioSampleRate
        << ",\"audio_channels\":\"" << escapeJson(uiSettings.audioChannels) << "\""
        << ",\"input_volume\":" << uiSettings.inputVolume
        << ",\"headphone_volume\":" << uiSettings.headphoneVolume
        << ",\"battery_type\":\"" << escapeJson(uiSettings.batteryType) << "\""
        << ",\"battery_capacity\":\"" << escapeJson(uiSettings.batteryCapacity) << "\""
        << ",\"custom_battery_wh\":" << uiSettings.customBatteryWh
        << ",\"custom_battery_full_voltage\":" << jsonNumber(uiSettings.customBatteryFullVoltage, 2)
        << ",\"custom_battery_empty_voltage\":" << jsonNumber(uiSettings.customBatteryEmptyVoltage, 2)
        << ",\"sleep_mode\":\"" << escapeJson(uiSettings.sleepMode) << "\""
        << ",\"favorites_order\":" << jsonStringArray(uiSettings.favoritesOrder)
        << "}"
        << ",\"audio\":{"
        << "\"input_devices\":" << jsonAudioDeviceOptions(audioDevices.inputOptions)
        << ",\"output_devices\":" << jsonAudioDeviceOptions(audioDevices.outputOptions)
        << ",\"has_input_devices\":" << (!audioDevices.inputOptions.empty() ? "true" : "false")
        << ",\"has_output_devices\":" << (!audioDevices.outputOptions.empty() ? "true" : "false")
        << "}"
        << "," << imuStatusBody(true)
        << ",\"device\":{"
        << "\"pi_model\":\"" << escapeJson(device.piModel) << "\""
        << ",\"serial_number\":\"" << escapeJson(device.serialNumber) << "\""
        << ",\"os_version\":\"" << escapeJson(device.osVersion) << "\""
        << ",\"kernel_version\":\"" << escapeJson(device.kernelVersion) << "\""
        << ",\"uptime_text\":\"" << escapeJson(device.uptimeText) << "\""
        << ",\"ip_address\":\"" << escapeJson(network.ipAddress) << "\""
        << ",\"hostname\":\"" << escapeJson(device.hostname) << "\""
        << ",\"ram_total_text\":\"" << escapeJson(device.ramTotalText) << "\""
        << ",\"ram_available_text\":\"" << escapeJson(device.ramAvailableText) << "\""
        << ",\"ram_used_text\":\"" << escapeJson(device.ramUsedText) << "\""
        << ",\"system_storage_total_text\":\"" << escapeJson(device.systemStorageTotalText) << "\""
        << ",\"system_storage_free_text\":\"" << escapeJson(device.systemStorageFreeText) << "\""
        << ",\"system_storage_mount_point\":\"" << escapeJson(device.systemStorageMountPoint) << "\""
        << "}"
        << "}";
    return out.str();
}

void ApertarCoreApp::updateWebPreviewRegistration(const PreviewFrameDescriptor &frame)
{
#if !APERTARCORE_HAS_JPEG
    (void)frame;
#else
    if (frame.fd < 0 || frame.width == 0 || frame.height == 0)
        return;

    const int planeCount = std::max(0, std::min(frame.planeCount, 3));
    size_t mappingLength = frame.size;
    for (int i = 0; i < planeCount; ++i) {
        const size_t planeEnd =
            static_cast<size_t>(frame.planeOffsets[i]) + static_cast<size_t>(frame.planeLengths[i]);
        mappingLength = std::max(mappingLength, planeEnd);
    }
    if (mappingLength == 0)
        return;

    void *mapping = ::mmap(nullptr, mappingLength, PROT_READ, MAP_SHARED, frame.fd, 0);
    if (mapping == MAP_FAILED)
        return;

    WebPreviewSlotMapping slot;
    slot.mapping = mapping;
    slot.mappingLength = mappingLength;
    slot.data = static_cast<uint8_t *>(mapping);
    slot.width = frame.width;
    slot.height = frame.height;
    slot.stride = frame.stride;
    slot.planeCount = planeCount;
    for (int i = 0; i < planeCount; ++i) {
        slot.planeOffsets[i] = frame.planeOffsets[i];
        slot.planePitches[i] = frame.planePitches[i];
        slot.planeLengths[i] = frame.planeLengths[i];
    }

    std::lock_guard<std::mutex> lock(mjpegMutex_);
    auto existing = webPreviewSlots_.find(frame.slotIndex);
    if (existing != webPreviewSlots_.end()) {
        if (existing->second.mapping && existing->second.mappingLength > 0)
            ::munmap(existing->second.mapping, existing->second.mappingLength);
        webPreviewSlots_.erase(existing);
    }
    webPreviewSlots_[frame.slotIndex] = slot;
#endif
}

void ApertarCoreApp::captureWebPreviewFrame(const PreviewFrameDescriptor &frame)
{
#if !APERTARCORE_HAS_JPEG
    (void)frame;
#else
    if (mjpegClientCount_.load(std::memory_order_relaxed) == 0)
        return;

    // This runs on libcamera's completion thread. Camera stop/reconfigure holds
    // the control mutex while waiting for that thread, so never query it here.
    const auto minInterval = std::chrono::microseconds(
        static_cast<int64_t>(1'000'000.0 / kWebPreviewMaxFps));
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mjpegMutex_);
        if (lastWebPreviewCaptureAt_.time_since_epoch().count() != 0 &&
            (now - lastWebPreviewCaptureAt_) < minInterval) {
            return;
        }
        lastWebPreviewCaptureAt_ = now;
    }

    std::vector<uint8_t> yuv;
    uint32_t targetWidth = 0;
    uint32_t targetHeight = 0;
    uint64_t frameId = frame.frameId;
    int64_t timestampUs = frame.timestampUs;

    {
        std::lock_guard<std::mutex> lock(mjpegMutex_);
        const auto slotIt = webPreviewSlots_.find(frame.slotIndex);
        if (slotIt == webPreviewSlots_.end())
            return;

        const WebPreviewSlotMapping &slot = slotIt->second;
        if (!slot.data || slot.planeCount < 3)
            return;

        const uint32_t sourceWidth = frame.width ? frame.width : slot.width;
        const uint32_t sourceHeight = frame.height ? frame.height : slot.height;
        if (sourceWidth == 0 || sourceHeight == 0)
            return;

        std::tie(targetWidth, targetHeight) = scaledWebPreviewSize(sourceWidth, sourceHeight);
        if (targetWidth == 0 || targetHeight == 0)
            return;

        const uint8_t *yPlane = slot.data + slot.planeOffsets[0];
        const uint8_t *uPlane = slot.data + slot.planeOffsets[1];
        const uint8_t *vPlane = slot.data + slot.planeOffsets[2];
        const uint32_t yPitch = slot.planePitches[0] ? slot.planePitches[0] : sourceWidth;
        const uint32_t uPitch = slot.planePitches[1] ? slot.planePitches[1] : std::max(1u, sourceWidth / 2);
        const uint32_t vPitch = slot.planePitches[2] ? slot.planePitches[2] : std::max(1u, sourceWidth / 2);

        const uint32_t yStrideOut = targetWidth;
        const uint32_t uvStrideOut = std::max(1u, targetWidth / 2);
        const uint32_t yPlaneBytes = yStrideOut * targetHeight;
        const uint32_t uvPlaneHeight = std::max(1u, targetHeight / 2);
        const uint32_t uvPlaneBytes = uvStrideOut * uvPlaneHeight;
        const uint32_t sourceChromaWidth = std::max(1u, sourceWidth / 2);
        const uint32_t sourceChromaHeight = std::max(1u, sourceHeight / 2);

        if (pendingMjpegFrame_.has_value()) {
            yuv = std::move(pendingMjpegFrame_->yuv);
            pendingMjpegFrame_.reset();
        } else if (!recycledMjpegYuv_.empty()) {
            yuv = std::move(recycledMjpegYuv_);
        }
        yuv.resize(static_cast<size_t>(yPlaneBytes) + static_cast<size_t>(uvPlaneBytes) * 2u);
        uint8_t *yOut = yuv.data();
        uint8_t *uOut = yuv.data() + yPlaneBytes;
        uint8_t *vOut = yuv.data() + yPlaneBytes + uvPlaneBytes;

        downsampleNearestPlane(yPlane, sourceWidth, sourceHeight, yPitch,
                               yOut, targetWidth, targetHeight, yStrideOut);
        downsampleNearestPlane(uPlane, sourceChromaWidth, sourceChromaHeight, uPitch,
                               uOut, uvStrideOut, uvPlaneHeight, uvStrideOut);
        downsampleNearestPlane(vPlane, sourceChromaWidth, sourceChromaHeight, vPitch,
                               vOut, uvStrideOut, uvPlaneHeight, uvStrideOut);

        PendingWebPreviewFrame pending;
        pending.width = targetWidth;
        pending.height = targetHeight;
        pending.frameId = frameId;
        pending.timestampUs = timestampUs;
        pending.planeOffsets[0] = 0;
        pending.planeStrides[0] = yStrideOut;
        pending.planeOffsets[1] = yPlaneBytes;
        pending.planeStrides[1] = uvStrideOut;
        pending.planeOffsets[2] = yPlaneBytes + uvPlaneBytes;
        pending.planeStrides[2] = uvStrideOut;
        pending.yuv = std::move(yuv);
        pendingMjpegFrame_ = std::move(pending);
    }

    mjpegCv_.notify_one();
#endif
}

void ApertarCoreApp::runMjpegEncoder()
{
#if !APERTARCORE_HAS_JPEG
    return;
#else
    std::vector<uint8_t> jpegBytes;
    std::unique_lock<std::mutex> lock(mjpegMutex_);
    while (true) {
        mjpegCv_.wait(lock, [this]() {
            return !mjpegEncoderRunning_ || pendingMjpegFrame_.has_value();
        });

        if (!mjpegEncoderRunning_ && !pendingMjpegFrame_.has_value())
            break;
        if (!pendingMjpegFrame_.has_value())
            continue;

        PendingWebPreviewFrame frame = std::move(*pendingMjpegFrame_);
        pendingMjpegFrame_.reset();

        lock.unlock();
        const bool encoded = encodeYuv420ToJpeg(
            frame.yuv.data(),
            frame.width,
            frame.height,
            frame.planeOffsets,
            frame.planeStrides,
            kWebPreviewJpegQuality,
            &jpegBytes);
        lock.lock();

        if (recycledMjpegYuv_.capacity() < frame.yuv.capacity())
            recycledMjpegYuv_ = std::move(frame.yuv);

        if (!encoded)
            continue;

        latestMjpegFrame_.swap(jpegBytes);
        ++latestMjpegGeneration_;
        mjpegCv_.notify_all();
    }
#endif
}

void ApertarCoreApp::stopMjpegEncoder()
{
    {
        std::lock_guard<std::mutex> lock(mjpegMutex_);
        mjpegEncoderRunning_ = false;
    }
    mjpegCv_.notify_all();

    if (mjpegThread_.joinable())
        mjpegThread_.join();

    std::lock_guard<std::mutex> lock(mjpegMutex_);
    pendingMjpegFrame_.reset();
    latestMjpegFrame_.clear();
    lastWebPreviewCaptureAt_ = {};
    for (auto &[slotIndex, slot] : webPreviewSlots_) {
        (void)slotIndex;
        if (slot.mapping && slot.mappingLength > 0)
            ::munmap(slot.mapping, slot.mappingLength);
        closeFdIfOpen(&slot.fd);
    }
    webPreviewSlots_.clear();
}

void ApertarCoreApp::streamMjpeg(int clientFd)
{
#if !APERTARCORE_HAS_JPEG
    (void)clientFd;
#else
    mjpegClientCount_.fetch_add(1, std::memory_order_relaxed);
    struct ViewerGuard {
        std::atomic<uint32_t> &count;
        ~ViewerGuard() { count.fetch_sub(1, std::memory_order_relaxed); }
    } viewerGuard{mjpegClientCount_};

    uint64_t seenGeneration = 0;
    while (running_.load(std::memory_order_relaxed)) {
        std::vector<uint8_t> jpegBytes;
        {
            std::unique_lock<std::mutex> lock(mjpegMutex_);
            mjpegCv_.wait_for(lock, std::chrono::seconds(1), [this, seenGeneration]() mutable {
                return !mjpegEncoderRunning_ || latestMjpegGeneration_ != seenGeneration;
            });

            if (!mjpegEncoderRunning_)
                break;
            if (latestMjpegGeneration_ == seenGeneration)
                continue;

            seenGeneration = latestMjpegGeneration_;
            jpegBytes = latestMjpegFrame_;
        }

        if (jpegBytes.empty())
            continue;

        std::ostringstream header;
        header << "--frame\r\n"
               << "Content-Type: image/jpeg\r\n"
               << "Content-Length: " << jpegBytes.size() << "\r\n\r\n";
        const std::string headerText = header.str();
        static constexpr char kPartTerminator[] = "\r\n";
        if (!sendAllBytes(clientFd, headerText.data(), headerText.size()) ||
            !sendAllBytes(clientFd, jpegBytes.data(), jpegBytes.size()) ||
            !sendAllBytes(clientFd, kPartTerminator, sizeof(kPartTerminator) - 1)) {
            break;
        }
    }

#endif
}

void ApertarCoreApp::syncRecordingTracking(bool active)
{
    std::lock_guard<std::mutex> lock(telemetryMutex_);
    if (!active) {
        recordingStartKnown_ = false;
        return;
    }

    recordingStartedAt_ = std::chrono::steady_clock::now();
    recordingStartKnown_ = true;
}

double ApertarCoreApp::recordingElapsedSeconds() const
{
    std::lock_guard<std::mutex> lock(telemetryMutex_);
    if (!recordingStartKnown_)
        return 0.0;

    const auto elapsed = std::chrono::steady_clock::now() - recordingStartedAt_;
    return std::chrono::duration<double>(elapsed).count();
}

bool ApertarCoreApp::soundRecordingActive() const
{
    return soundRecordingActive_.load(std::memory_order_relaxed);
}

bool ApertarCoreApp::startSoundRecording(AudioRecordingOptions audioOptions, std::string *errorMessage)
{
    if (soundRecordingActive()) {
        if (errorMessage)
            *errorMessage = "Sound recorder is already running.";
        return false;
    }

    if (camera_.isRecording() || camera_.timelapseActive()) {
        if (errorMessage)
            *errorMessage = "Stop camera recording or timelapse before starting Sound Recorder.";
        return false;
    }

    if (!isMountedPath(options_.mediaRoot)) {
        if (errorMessage)
            *errorMessage = "Sound Recorder needs mounted media.";
        return false;
    }

    if (audioOptions.inputDevice.empty()) {
        if (errorMessage)
            *errorMessage = "Sound Recorder requires an audio input device.";
        return false;
    }

    audioOptions.enabled = true;
    const CaptureSettings settings = camera_.settings();
    audioOptions.fps = settings.fps;
    audioOptions.timecodeMode = settings.timecodeMode;

    std::filesystem::path folder;
    try {
        folder = allocateSoundRecordingFolder(options_.mediaRoot);
    } catch (const std::exception &e) {
        if (errorMessage)
            *errorMessage = e.what();
        return false;
    }

    std::string audioError;
    if (!soundRecorder_.start(
            folder,
            audioOptions,
            [this](int level) {
                const int clamped = std::clamp(level, 0, 100);
                publishEvent("{\"event\":\"audio_level\",\"input_level\":" + std::to_string(clamped) + "}");
            },
            &audioError)) {
        if (errorMessage)
            *errorMessage = audioError.empty() ? "Could not start Sound Recorder." : audioError;
        return false;
    }

    soundRecordingFolder_ = folder.string();
    soundRecordingActive_.store(true, std::memory_order_relaxed);
    syncRecordingTracking(true);
    publishEvent("{\"event\":\"recording\",\"active\":true,\"sound_only\":true,\"folder\":\"" +
                 escapeJson(soundRecordingFolder_) + "\"}");
    return true;
}

void ApertarCoreApp::stopSoundRecording()
{
    if (!soundRecordingActive_.exchange(false, std::memory_order_relaxed))
        return;

    soundRecorder_.stop();
    syncRecordingTracking(false);
    publishEvent("{\"event\":\"recording\",\"active\":false,\"sound_only\":true}");
    soundRecordingFolder_.clear();
}

std::string normalizeHardwareAnalogTarget(std::string target)
{
    target = lowerAscii(trimAscii(std::move(target)));
    for (char &ch : target) {
        if (ch == '-' || ch == ' ' || ch == '.')
            ch = '_';
    }
    if (target == "white_balance" || target == "whitebalance")
        return "wb";
    if (target == "focus" || target == "focus_peaking" || target == "focus_peaking_value")
        return "focus_peaking_threshold";
    if (target == "zebra" || target == "zebras")
        return "zebra_threshold";
    if (target == "volume" || target == "input_gain")
        return "input_volume";
    if (target == "headphone" || target == "headphones")
        return "headphone_volume";
    if (target == "opacity" || target == "controls_opacity")
        return "camera_controls_opacity";
    if (target == "res")
        return "resolution";
    return target;
}

size_t analogPresetIndex(double normalized, size_t count)
{
    if (count <= 1)
        return 0;
    normalized = std::clamp(normalized, 0.0, 1.0);
    return std::min(count - 1, static_cast<size_t>(std::llround(normalized * static_cast<double>(count - 1))));
}

double analogMappedValue(const GroveAnalogInput::AnalogEvent &event,
                         double fallbackMin,
                         double fallbackMax,
                         double fallbackStep)
{
    const double minimum = std::isfinite(event.minValue) ? event.minValue : fallbackMin;
    const double maximum = std::isfinite(event.maxValue) ? event.maxValue : fallbackMax;
    const double low = std::min(minimum, maximum);
    const double high = std::max(minimum, maximum);
    double value = low + std::clamp(event.normalized, 0.0, 1.0) * (high - low);
    const double step = std::isfinite(event.step) && event.step > 0.0 ? event.step : fallbackStep;
    if (step > 0.0)
        value = std::round(value / step) * step;
    return std::clamp(value, low, high);
}

std::string ApertarCoreApp::dispatchHardwareInputCommand(ControlCommand command)
{
    std::lock_guard<std::mutex> lock(hardwareActionMutex_);
    command.id = -1;
    const std::string reply = handleCommand(command);
    if (reply.find("\"ok\":false") != std::string::npos)
        std::cerr << "Hardware input action failed: " << reply << "\n";
    return reply;
}

std::string ApertarCoreApp::toggleSharedUiBoolAction(const std::string &action,
                                                     const std::string &commandName,
                                                     bool currentValue)
{
    ControlCommand command;
    command.name = commandName;
    command.fields["enabled"] = currentValue ? "false" : "true";
    const std::string reply = dispatchHardwareInputCommand(std::move(command));
    publishEvent("{\"event\":\"hardware_action\",\"action\":\"" + escapeJson(action) + "\"}");
    return reply;
}

void ApertarCoreApp::handleHardwareAnalogInput(const GroveAnalogInput::AnalogEvent &event)
{
    if (!running_.load(std::memory_order_relaxed) || event.target.empty())
        return;

    std::lock_guard<std::mutex> sequenceLock(hardwareInputSequenceMutex_);

    auto dispatch = [this](std::string commandName,
                           std::map<std::string, std::string> fields = {}) {
        ControlCommand command;
        command.name = std::move(commandName);
        command.fields = std::move(fields);
        return dispatchHardwareInputCommand(std::move(command));
    };

    const std::string target = normalizeHardwareAnalogTarget(event.target);
    const SharedUiSettingsSnapshot uiSettings = readSharedUiSettings();
    const CaptureSettings cameraSettings = camera_.settings();

    if (target == "iso") {
        const std::vector<double> options = pcfIsoOptions();
        const size_t index = analogPresetIndex(event.normalized, options.size());
        const uint32_t iso = static_cast<uint32_t>(options[index]);
        if (!cameraSettings.isoAuto && cameraSettings.iso == iso)
            return;
        dispatch("set_iso", { { "value", std::to_string(iso) } });
        return;
    }

    if (target == "fps") {
        const std::vector<double> options = availablePcfFpsOptions(cameraSettings, camera_.sensorName());
        const size_t index = analogPresetIndex(event.normalized, options.size());
        const double fps = options[index];
        if (std::abs(cameraSettings.fps - fps) < 0.001)
            return;
        dispatch("set_fps", { { "value", compactNumber(fps) } });
        return;
    }

    if (target == "resolution") {
        const std::vector<Resolution> options = availablePcfResolutionOptions(camera_.sensorName());
        const size_t index = analogPresetIndex(event.normalized, options.size());
        const Resolution resolution = options[index];
        if (sameResolution(cameraSettings.resolution, resolution))
            return;
        dispatch("set_resolution", {
            { "width", std::to_string(resolution.width) },
            { "height", std::to_string(resolution.height) },
        });
        return;
    }

    if (target == "wb") {
        uint32_t kelvin = 0;
        if (event.usePresets) {
            const std::vector<double> options = pcfWhiteBalanceOptions();
            kelvin = static_cast<uint32_t>(options[analogPresetIndex(event.normalized, options.size())]);
        } else {
            kelvin = static_cast<uint32_t>(std::llround(analogMappedValue(event, 2500.0, 8000.0, 50.0)));
        }
        kelvin = std::clamp<uint32_t>(kelvin, 2500, 8000);
        if (cameraSettings.whiteBalanceKelvin == kelvin)
            return;
        dispatch("set_wb", { { "kelvin", std::to_string(kelvin) } });
        return;
    }

    if (target == "shutter") {
        const bool useShutterSpeed = uiSettings.photoModeEnabled || camera_.timelapseActive();
        const std::vector<double> options = useShutterSpeed
            ? pcfShutterSpeedDenominatorOptions(cameraSettings.photoModeEnabled && cameraSettings.longExposureEnabled,
                                                cameraSettings.timelapseExposureLimitUs)
            : pcfShutterAngleOptions();
        const double selected = options[analogPresetIndex(event.normalized, options.size())];
        const uint32_t shutterUs = useShutterSpeed
            ? static_cast<uint32_t>(std::max(1.0, std::round(1000000.0 / selected)))
            : static_cast<uint32_t>(std::max(
                  1.0,
                  std::round((selected / 360.0) * (1000000.0 / std::max(0.001, cameraSettings.fps)))));
        if (std::abs(static_cast<int64_t>(cameraSettings.shutterUs) - static_cast<int64_t>(shutterUs)) < 25)
            return;
        dispatch("set_shutter_us", { { "value", std::to_string(shutterUs) } });
        return;
    }

    if (target == "focus_peaking_threshold") {
        const double value = analogMappedValue(event, 0.0, 1.0, 0.01);
        if (std::abs(uiSettings.focusPeakingThreshold - value) < 0.005)
            return;
        dispatch("set_focus_peaking_threshold", { { "value", compactNumber(value) } });
        return;
    }

    if (target == "zebra_threshold") {
        const double value = analogMappedValue(event, 0.0, 1.0, 0.01);
        if (std::abs(uiSettings.zebraThreshold - value) < 0.005)
            return;
        dispatch("set_zebra_threshold", { { "value", compactNumber(value) } });
        return;
    }

    if (target == "input_volume") {
        const int volume = static_cast<int>(std::llround(analogMappedValue(event, 0.0, 100.0, 1.0)));
        if (uiSettings.inputVolume == volume)
            return;
        dispatch("set_input_volume", { { "value", std::to_string(volume) } });
        return;
    }

    if (target == "headphone_volume") {
        const int volume = static_cast<int>(std::llround(analogMappedValue(event, 0.0, 100.0, 1.0)));
        if (uiSettings.headphoneVolume == volume)
            return;
        dispatch("set_headphone_volume", { { "value", std::to_string(volume) } });
        return;
    }

    if (target == "camera_controls_opacity") {
        const double opacity = analogMappedValue(event, 0.15, 1.0, 0.01);
        if (std::abs(uiSettings.cameraControlsOpacity - opacity) < 0.005)
            return;
        dispatch("set_camera_controls_opacity", { { "value", compactNumber(opacity) } });
        return;
    }

    std::cerr << "Hardware analog input ignored: unknown target '" << event.target << "'.\n";
}

bool ApertarCoreApp::handlePowerButtonPress(bool doublePress)
{
    if (!running_.load(std::memory_order_relaxed)) return true;
    if (camera_.state() == CameraState::Standby) {
        publishEvent("{\"event\":\"hardware_action\",\"action\":\"standby_power_wake\"}");
        handleHardwareInputAction("standby_power_wake");
        return true;
    }
    lastUserActivityMs_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    if (!doublePress) {
        publishEvent("{\"event\":\"hardware_action\",\"action\":\"power_button_press\"}");
        return false;
    }
    // Cancel the UI's shutdown hold before attempting standby, even when busy.
    publishEvent("{\"event\":\"hardware_action\",\"action\":\"standby_power_sleep\"}");
    ControlCommand command;
    command.name = "enter_standby";
    dispatchHardwareInputCommand(std::move(command));
    return true;
}

void ApertarCoreApp::handleHardwareInputAction(const std::string &action, bool buttonPress)
{
    if (!running_.load(std::memory_order_relaxed) || action.empty())
        return;

    const auto actions = hardware::parseButtonActions(action, [](const std::string &token) { return token; });
    if (!actions.error.empty()) {
        std::cerr << "Hardware input mapping ignored: " << actions.error << '\n';
        return;
    }
    // Keep one button's sequence together across GPIO, I2C and analog inputs.
    std::lock_guard<std::mutex> sequenceLock(hardwareInputSequenceMutex_);
    if (camera_.state() == CameraState::Standby) {
        if (!buttonPress)
            return;
        ControlCommand wake;
        wake.name = "wake";
        dispatchHardwareInputCommand(std::move(wake));
        return;
    }
    lastUserActivityMs_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    for (const auto &singleAction : actions.actions) {
        if (!running_.load(std::memory_order_relaxed)) break;
        if (!buttonPress && (singleAction == "standby" || singleAction == "standby_toggle")) continue;
        handleSingleHardwareInputAction(singleAction);
        if (camera_.state() == CameraState::Standby) break;
    }
}

void ApertarCoreApp::handleSingleHardwareInputAction(const std::string &action)
{
    if (const int slot = hardware::scriptActionSlot(action); slot >= 0) {
        hardwareScriptRunner_.request(editableHardwareConfigPath().string(), unsigned(slot));
        return;
    }
    if (action == "zoom_toggle") {
        publishEvent("{\"event\":\"hardware_action\",\"action\":\"zoom_toggle\"}");
        return;
    }

    auto dispatch = [this](std::string commandName,
                           std::map<std::string, std::string> fields = {}) {
        ControlCommand command;
        command.name = std::move(commandName);
        command.fields = std::move(fields);
        return dispatchHardwareInputCommand(std::move(command));
    };

    if (action == "standby" || action == "standby_toggle") {
        dispatch("enter_standby");
        return;
    }

    const SharedUiSettingsSnapshot uiSettings = readSharedUiSettings();
    const CaptureSettings cameraSettings = camera_.settings();

    if (action == "ev_up" || action == "ev_down") {
        const double ev = std::clamp(cameraSettings.exposureCompensation +
            (action == "ev_up" ? 1.0 / 3.0 : -1.0 / 3.0), -3.0, 3.0);
        dispatch("set_exposure_compensation", {{"value", compactNumber(ev)}});
        return;
    }
    if (const auto ev = parseHardwareActionNumber(action, {"evm", "ev"})) {
        dispatch("set_exposure_compensation", {{"value", compactNumber(action.rfind("evm", 0) == 0 ? -*ev : *ev)}});
        return;
    }

    if (const auto fps = parseHardwareActionNumber(action, { "fps" })) {
        if (*fps > 0.0) {
            dispatch("set_fps", { { "value", compactNumber(*fps) } });
            return;
        }
    }

    if (const auto resolution = parseHardwareActionResolution(action)) {
        dispatch("set_resolution", {
            { "width", std::to_string(resolution->width) },
            { "height", std::to_string(resolution->height) },
        });
        return;
    }

    if (const auto iso = parseHardwareActionNumber(action, { "iso" })) {
        if (*iso > 0.0) {
            dispatch("set_iso", { { "value", std::to_string(static_cast<uint32_t>(std::llround(*iso))) } });
            return;
        }
    }

    if (const auto shutterUs = parseHardwareActionShutterSeconds(action)) {
        dispatch("set_shutter_us", { { "value", std::to_string(*shutterUs) } });
        return;
    }

    if (const auto shutterAngle = parseHardwareActionNumber(action, { "shutter_angle", "shutterangle" })) {
        if (*shutterAngle > 0.0) {
            const uint32_t shutterUs = static_cast<uint32_t>(std::max(
                1.0,
                std::round((*shutterAngle / 360.0) * (1000000.0 / std::max(0.001, cameraSettings.fps)))));
            dispatch("set_shutter_us", { { "value", std::to_string(shutterUs) } });
            return;
        }
    }

    if (const auto shutterDenominator = parseHardwareActionShutterSpeedDenominator(
            action, { "shutter_speed", "shutterspeed", "shutter_s" })) {
        if (*shutterDenominator > 0.0) {
            const uint32_t shutterUs = static_cast<uint32_t>(
                std::max(1.0, std::round(1000000.0 / *shutterDenominator)));
            dispatch("set_shutter_us", { { "value", std::to_string(shutterUs) } });
            return;
        }
    }

    if (const auto shutterValue = parseHardwareActionShutterSpeedDenominator(action, { "shutter" })) {
        if (*shutterValue > 0.0) {
            const auto shutterSuffix = hardwareActionSuffix(action, { "shutter" });
            const bool speedSyntax = shutterSuffix &&
                (shutterSuffix->rfind("1/", 0) == 0 || shutterSuffix->rfind("1_", 0) == 0);
            const bool useShutterSpeed = speedSyntax || uiSettings.photoModeEnabled || camera_.timelapseActive();
            const uint32_t shutterUs = useShutterSpeed
                ? static_cast<uint32_t>(std::max(1.0, std::round(1000000.0 / *shutterValue)))
                : static_cast<uint32_t>(std::max(
                      1.0,
                      std::round((*shutterValue / 360.0) * (1000000.0 / std::max(0.001, cameraSettings.fps)))));
            dispatch("set_shutter_us", { { "value", std::to_string(shutterUs) } });
            return;
        }
    }

    if (const auto kelvin = parseHardwareActionNumber(action, { "wb", "white_balance", "whitebalance" })) {
        if (*kelvin > 0.0) {
            const auto clamped = std::clamp<uint32_t>(
                static_cast<uint32_t>(std::llround(*kelvin)), 2500u, 8000u);
            dispatch("set_wb", { { "kelvin", std::to_string(clamped) } });
            return;
        }
    }

    if (const auto mode = pcfSettingStepMode(action, "fps")) {
        const std::vector<double> options = availablePcfFpsOptions(cameraSettings, camera_.sensorName());
        const size_t currentIndex = nearestPcfPresetIndex(options, cameraSettings.fps);
        const size_t nextIndex = nextPcfPresetIndex(currentIndex, options.size(), *mode);
        dispatch("set_fps", { { "value", compactNumber(options[nextIndex]) } });
        return;
    }

    if (const auto mode = pcfSettingStepMode(action, "resolution")) {
        const std::vector<Resolution> options = availablePcfResolutionOptions(camera_.sensorName());
        const size_t currentIndex = nearestPcfResolutionIndex(options, cameraSettings.resolution);
        const size_t nextIndex = nextPcfPresetIndex(currentIndex, options.size(), *mode);
        dispatch("set_resolution", {
            { "width", std::to_string(options[nextIndex].width) },
            { "height", std::to_string(options[nextIndex].height) },
        });
        return;
    }

    if (const auto mode = pcfSettingStepMode(action, "iso")) {
        const std::vector<double> options = pcfIsoOptions();
        const size_t currentIndex = cameraSettings.isoAuto
            ? 0
            : nearestPcfPresetIndex(options, static_cast<double>(cameraSettings.iso)) + 1;
        const size_t nextIndex = nextPcfPresetIndex(currentIndex, options.size() + 1, *mode);
        if (nextIndex == 0)
            dispatch("set_iso_auto");
        else
            dispatch("set_iso", { { "value", std::to_string(static_cast<uint32_t>(options[nextIndex - 1])) } });
        return;
    }

    if (const auto mode = pcfSettingStepMode(action, "shutter")) {
        const bool useShutterSpeed = uiSettings.photoModeEnabled || camera_.timelapseActive();
        const std::vector<double> options = useShutterSpeed
            ? pcfShutterSpeedDenominatorOptions(cameraSettings.photoModeEnabled && cameraSettings.longExposureEnabled,
                                                cameraSettings.timelapseExposureLimitUs)
            : pcfShutterAngleOptions();
        size_t currentIndex = 0;
        if (!cameraSettings.shutterAuto) {
            const double currentValue = useShutterSpeed
                ? 1000000.0 / std::max<uint32_t>(1u, cameraSettings.shutterUs)
                : (static_cast<double>(cameraSettings.shutterUs) * cameraSettings.fps * 360.0) / 1000000.0;
            currentIndex = nearestPcfPresetIndex(options, currentValue) + 1;
        }

        const size_t nextIndex = nextPcfPresetIndex(currentIndex, options.size() + 1, *mode);
        if (nextIndex == 0) {
            dispatch("set_shutter_auto");
        } else {
            const double selected = options[nextIndex - 1];
            const uint32_t shutterUs = useShutterSpeed
                ? static_cast<uint32_t>(std::max(1.0, std::round(1000000.0 / selected)))
                : static_cast<uint32_t>(std::max(
                      1.0,
                      std::round((selected / 360.0) * (1000000.0 / std::max(0.001, cameraSettings.fps)))));
            dispatch("set_shutter_us", { { "value", std::to_string(shutterUs) } });
        }
        return;
    }

    if (const auto mode = pcfSettingStepMode(action, "wb")) {
        const std::vector<double> options = pcfWhiteBalanceOptions();
        const size_t currentIndex =
            nearestPcfPresetIndex(options, static_cast<double>(cameraSettings.whiteBalanceKelvin));
        const size_t nextIndex = nextPcfPresetIndex(currentIndex, options.size(), *mode);
        dispatch("set_wb", { { "kelvin", std::to_string(static_cast<uint32_t>(options[nextIndex])) } });
        return;
    }

    if (action == "capture") {
        if (camera_.timelapseActive()) {
            dispatch("timelapse_stop");
        } else if (uiSettings.photoModeEnabled) {
            if (photoTimelapseSecondsFromValue(uiSettings.photoTimelapse) > 0) {
                dispatch("timelapse_start", {
                    { "value", uiSettings.photoTimelapse },
                    { "format", uiSettings.photoFormat },
                });
            } else {
                dispatch("capture_photo", {
                    { "format", uiSettings.photoFormat },
                    { "count", std::to_string(photoBurstCountFromValue(uiSettings.photoBurst)) },
                });
            }
        } else if (camera_.isRecording() || soundRecordingActive()) {
            dispatch("record_stop");
        } else {
            dispatch("record_start");
        }
        return;
    }

    if (action == "record_toggle") {
        if (camera_.isRecording() || soundRecordingActive())
            dispatch("record_stop");
        else
            dispatch("record_start");
        return;
    }

    if (action == "photo") {
        dispatch("capture_photo", {
            { "format", uiSettings.photoFormat },
            { "count", std::to_string(photoBurstCountFromValue(uiSettings.photoBurst)) },
        });
        return;
    }

    if (action == "timelapse") {
        if (camera_.timelapseActive()) {
            dispatch("timelapse_stop");
        } else {
            dispatch("timelapse_start", {
                { "value", uiSettings.photoTimelapse },
                { "format", uiSettings.photoFormat },
            });
        }
        return;
    }

    if (action == "eject") {
        if (camera_.isRecording() || soundRecordingActive() || camera_.timelapseActive()) {
            std::cerr << "Hardware input eject ignored while recording or timelapse is active.\n";
            return;
        }
        dispatch("eject_media");
        return;
    }

    if (action == "still_mode") {
        dispatch("set_photo_mode_enabled", {
            { "enabled", uiSettings.photoModeEnabled ? "false" : "true" },
        });
        publishEvent("{\"event\":\"hardware_action\",\"action\":\"still_mode\"}");
        return;
    }

    if (action == "focus_peaking") {
        toggleSharedUiBoolAction(action, "set_focus_peaking_enabled", uiSettings.focusPeakingEnabled);
        return;
    }
    if (action == "zebra") {
        toggleSharedUiBoolAction(action, "set_zebra_enabled", uiSettings.zebraEnabled);
        return;
    }
    if (action == "false_color") {
        toggleSharedUiBoolAction(action, "set_false_color_enabled", uiSettings.falseColorEnabled);
        return;
    }
    if (action == "guides") {
        toggleSharedUiBoolAction(action, "set_guides_enabled", uiSettings.guidesEnabled);
        return;
    }
    if (action == "center_marker") {
        toggleSharedUiBoolAction(action, "set_center_marker_enabled", uiSettings.centerMarkerEnabled);
        return;
    }
    if (action == "grayscale") {
        toggleSharedUiBoolAction(action, "set_grayscale_enabled", uiSettings.grayscaleEnabled);
        return;
    }
    if (action == "smpte") {
        toggleSharedUiBoolAction(action, "set_smpte_enabled", uiSettings.smpteEnabled);
        return;
    }
    if (action == "external_monitor") {
        toggleSharedUiBoolAction(action, "set_external_monitor_enabled", uiSettings.externalMonitorEnabled);
        return;
    }
    if (action == "date_time_overlay") {
        toggleSharedUiBoolAction(action, "set_date_time_overlay_enabled", uiSettings.dateTimeOverlayEnabled);
        return;
    }
    if (action == "audio_meter") {
        toggleSharedUiBoolAction(action, "set_audio_meter_enabled", uiSettings.audioMeterEnabled);
        return;
    }

    if (action == "settings" || action == "clip_browser" || action == "quick_menu") {
        publishEvent("{\"event\":\"hardware_action\",\"action\":\"" + escapeJson(action) + "\"}");
        return;
    }

    std::cerr << "Hardware input action ignored: unknown action '" << action << "'.\n";
}

bool ApertarCoreApp::startMediaSpeedTest(std::string *errorMessage)
{
    {
        std::lock_guard<std::mutex> lock(mediaSpeedMutex_);
        if (mediaSpeedTestRunning_) {
            if (errorMessage)
                *errorMessage = "A write speed test is already running.";
            return false;
        }
    }

    joinMediaSpeedTest();

    const bool mediaMounted = isMountedPath(options_.mediaRoot);
    if (!mediaMounted) {
        if (errorMessage)
            *errorMessage = "No media is currently mounted.";
        return false;
    }

    std::error_code spaceEc;
    const std::filesystem::path existingMediaPath = nearestExistingPath(options_.mediaRoot);
    const auto mediaSpace = existingMediaPath.empty()
        ? std::filesystem::space_info{}
        : std::filesystem::space(existingMediaPath, spaceEc);
    const uint64_t freeBytes = spaceEc ? 0 : mediaSpace.available;
    const int64_t freeMiB = static_cast<int64_t>(freeBytes / (1024ull * 1024ull));
    if (freeMiB < 96) {
        if (errorMessage)
            *errorMessage = "Need at least 96 MB free to run the write speed test.";
        return false;
    }

    uint64_t testMiB = 64;
    if (freeMiB >= 1152)
        testMiB = 1024;
    else if (freeMiB >= 640)
        testMiB = 512;
    else if (freeMiB >= 320)
        testMiB = 256;
    else if (freeMiB >= 160)
        testMiB = 128;

    {
        std::lock_guard<std::mutex> lock(mediaSpeedMutex_);
        mediaSpeedTestRunning_ = true;
        mediaSpeedCurrentUsedFio_ = true;
        mediaSpeedCurrentBytes_ = testMiB * 1024ull * 1024ull;
        mediaSpeedError_.clear();
    }

    mediaSpeedThread_ = std::thread(&ApertarCoreApp::runMediaSpeedTest, this, testMiB, options_.mediaRoot);
    return true;
}

void ApertarCoreApp::runMediaSpeedTest(uint64_t testMiB, std::string mountPath)
{
    const std::string tempPath = mountPath + "/.apertar_write_speed_test.bin";
    std::error_code rmEc;
    std::filesystem::remove(tempPath, rmEc);

    bool usedFio = true;
    const auto startedAt = std::chrono::steady_clock::now();
    ShellCommandResult result = runShellCommandCapture(
        "fio --name=write_test "
        "--filename=" + shellQuote(tempPath) + " "
        "--size=" + std::to_string(testMiB) + "M "
        "--bs=1M "
        "--rw=write "
        "--direct=1 "
        "--numjobs=1 "
        "--iodepth=32");

    if (result.exitCode != 0) {
        usedFio = false;
        const uint64_t count = std::max<uint64_t>(1, testMiB / 8ull);
        result = runShellCommandCapture(
            "dd if=/dev/zero of=" + shellQuote(tempPath) + " "
            "bs=8M count=" + std::to_string(count) + " conv=fdatasync status=none");
    }

    const auto finishedAt = std::chrono::steady_clock::now();
    const double elapsedSeconds = std::max(
        0.001,
        std::chrono::duration<double>(finishedAt - startedAt).count());

    double measuredMBps = 0.0;
    std::string errorMessage;
    if (result.exitCode == 0) {
        bool parsedSpeed = false;
        if (usedFio) {
            const std::regex mbRegex(R"(WRITE:\s*bw=.*\(([-+]?\d+(?:\.\d+)?)MB/s\))");
            std::smatch mbMatch;
            if (std::regex_search(result.output, mbMatch, mbRegex)) {
                measuredMBps = std::stod(mbMatch[1].str());
                parsedSpeed = measuredMBps > 0.0;
            }
        }

        if (!parsedSpeed && usedFio) {
            const std::regex mibRegex(R"(WRITE:\s*bw=([-+]?\d+(?:\.\d+)?)MiB/s)");
            std::smatch mibMatch;
            if (std::regex_search(result.output, mibMatch, mibRegex)) {
                measuredMBps = std::stod(mibMatch[1].str()) * 1.048576;
                parsedSpeed = measuredMBps > 0.0;
            }
        }

        if (!parsedSpeed) {
            measuredMBps =
                (static_cast<double>(testMiB)) / elapsedSeconds;
        }
    } else {
        errorMessage = result.output.empty() ? "The write speed test failed." : result.output;
    }

    std::filesystem::remove(tempPath, rmEc);
    runShellCommand("sync", nullptr);

    std::lock_guard<std::mutex> lock(mediaSpeedMutex_);
    mediaSpeedTestRunning_ = false;
    mediaSpeedCurrentUsedFio_ = false;
    mediaSpeedLastUsedFio_ = usedFio;
    mediaSpeedLastBytes_ = mediaSpeedCurrentBytes_;
    mediaSpeedLastWriteSpeedMBps_ = result.exitCode == 0 ? measuredMBps : 0.0;
    mediaSpeedError_ = errorMessage;
}

void ApertarCoreApp::joinMediaSpeedTest()
{
    if (mediaSpeedThread_.joinable())
        mediaSpeedThread_.join();
}

std::string ApertarCoreApp::handleCommand(const ControlCommand &command)
{
    std::lock_guard<std::mutex> commandLock(commandMutex_);
    if (camera_.checkRecordingFailure()) syncRecordingTracking(false);
    if ((command.name == "capture_photo" || command.name == "record_start" ||
         (command.name == "timelapse_start" && !camera_.timelapseActive())) && camera_.mediaWritesPending())
        return errorReply(command.id, "Wait for the current capture to finish writing to media.");
    const bool mediaAction = command.name == "format_media" || command.name == "eject_media" ||
        command.name == "mount_media" || command.name == "run_media_speed_test";
    bool speedTestActive;
    {
        std::lock_guard lock(mediaSpeedMutex_);
        speedTestActive = mediaSpeedTestRunning_;
    }
    if (mediaAction && (camera_.isRecording() || soundRecordingActive() ||
                       camera_.timelapseActive() || camera_.mediaWritesPending() || speedTestActive))
        return errorReply(command.id, "Stop recording, still captures and media tests before changing media.");
    if (speedTestActive && (command.name == "record_start" || command.name == "capture_photo" ||
                            command.name == "timelapse_start" || command.name == "enter_standby"))
        return errorReply(command.id, "Wait for the media speed test to finish first.");
    if (command.name == "user_activity" || command.name == "wake" ||
        command.name.rfind("set_", 0) == 0 || command.name == "capture_photo" ||
        command.name == "record_start" || command.name == "record_stop" ||
        command.name.rfind("timelapse_", 0) == 0) {
        lastUserActivityMs_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }
    if (command.name == "user_activity")
        return okReply(command.id);
    if (command.name == "set_standby_timeout") {
        const auto seconds = command.number("value");
        if (!seconds || !std::isfinite(*seconds) || *seconds < 0 || *seconds > 86400 ||
            std::floor(*seconds) != *seconds)
            return errorReply(command.id, "Standby timeout must be whole seconds between 0 and 86400.");
        if (!updateSharedUiGeneralSettings({{"standbyTimeoutSeconds", std::to_string(int(*seconds))}}))
            return errorReply(command.id, "Could not save standby timeout.");
        return okReply(command.id, "\"standby_timeout_seconds\":" + std::to_string(int(*seconds)));
    }
    if (command.name == "set_standby_usb_power_off") {
        const auto enabled = command.string("enabled");
        if (!enabled || (*enabled != "true" && *enabled != "false" && *enabled != "1" && *enabled != "0"))
            return errorReply(command.id, "USB standby power requires a boolean enabled value.");
        const bool value = *enabled == "true" || *enabled == "1";
        if (!updateSharedUiGeneralSettings({{"standbyUsbPowerOff", value ? "true" : "false"}}))
            return errorReply(command.id, "Could not save standby USB setting.");
        return okReply(command.id, std::string("\"standby_usb_power_off\":") + (value ? "true" : "false"));
    }
    if (command.name == "enter_standby") {
        if (const auto timeout = command.number("idle_timeout")) {
            const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (now - lastUserActivityMs_.load() < *timeout * 1000)
                return okReply(command.id);
        }
        if (soundRecordingActive())
            return errorReply(command.id, "Standby is unavailable while recording audio.");
        if (!camera_.enterStandby())
            return errorReply(command.id, camera_.lastError());
        stopPreviewPublisher();
        stopMjpegEncoder();
        std::string warning;
        if (!options_.simulateCamera && std::filesystem::exists("/usr/local/libexec/apertar-standby-low-power")) {
            const auto result = standbyLowPower("all");
            if (result.exitCode != 0) {
                warning = "Extra power saving skipped: " + result.output;
                std::cerr << "Apertar standby: " << warning << '\n';
                standbyLowPower("restore");
            }
        }
        if (!options_.simulateCamera && readSharedUiSettings().standbyUsbPowerOff) {
            const auto result = standbyUsbPower("off");
            standbyUsbPoweredOff_.store(result.exitCode == 0);
            if (result.exitCode != 0) {
                if (!warning.empty()) warning += "; ";
                warning += "USB kept powered: " + result.output;
                std::cerr << "Apertar standby: " << warning << '\n';
                standbyUsbPower("on");
            }
        }
        if (!warning.empty())
            return okReply(command.id, "\"standby\":true,\"warning\":\"" + escapeJson(warning) + "\"");
        return okReply(command.id, "\"standby\":true");
    }
    if (command.name == "wake") {
        if (camera_.state() == CameraState::Standby) {
            if (!options_.simulateCamera && std::filesystem::exists("/run/apertar-standby-low-power/settings")) {
                const auto result = standbyLowPower("restore");
                if (result.exitCode != 0)
                    return errorReply(command.id, "Could not restore normal performance: " + result.output);
            }
            if (!options_.simulateCamera &&
                std::filesystem::exists("/run/apertar-standby-usb/ports")) {
                const auto result = standbyUsbPower("on");
                if (result.exitCode != 0)
                    return errorReply(command.id, "Could not restore USB power: " + result.output);
            }
            standbyUsbPoweredOff_.store(false);
            {
                std::lock_guard<std::mutex> lock(previewMutex_);
                previewPublisherRunning_ = true;
            }
            previewThread_ = std::thread(&ApertarCoreApp::runPreviewPublisher, this);
#if APERTARCORE_HAS_JPEG
            {
                std::lock_guard<std::mutex> lock(mjpegMutex_);
                mjpegEncoderRunning_ = true;
            }
            mjpegThread_ = std::thread(&ApertarCoreApp::runMjpegEncoder, this);
#endif
            if (!camera_.wakeFromStandby()) {
                stopPreviewPublisher();
                stopMjpegEncoder();
                return errorReply(command.id, camera_.lastError());
            }
        }
        return okReply(command.id, "\"standby\":false");
    }
    if (camera_.state() == CameraState::Standby &&
        (command.name == "record_start" || command.name == "capture_photo" ||
         command.name == "timelapse_start"))
        return errorReply(command.id, "Wake the camera before capturing.");
    if (command.name == "ping")
        return okReply(command.id, "\"pong\":true");

    if (command.name == "help")
        return okReply(command.id, "\"commands\":[\"get_state\",\"set_iso\",\"set_iso_auto\",\"set_exposure_compensation\",\"set_exposure_metering\",\"set_fps\",\"set_shutter_us\",\"set_shutter_auto\",\"set_wb\",\"set_wb_auto\",\"set_resolution\",\"set_record_format\",\"set_cdng_compression\",\"set_lossy_dng_ratio\",\"set_prores_profile\",\"set_prores_color_profile\",\"set_cdng_preview_color_profile\",\"set_tuning_file\",\"set_raw_bit_depth\",\"set_h264_resolution\",\"set_timecode_mode\",\"set_af_mode\",\"set_af_range\",\"set_af_speed\",\"set_af_window\",\"set_photo_mode_enabled\",\"set_photo_long_exposure_enabled\",\"set_photo_bracketing_enabled\",\"set_photo_bracketing_ev\",\"set_photo_timer\",\"set_photo_burst\",\"set_photo_format\",\"set_photo_timelapse\",\"timelapse_start\",\"timelapse_stop\",\"set_focus_peaking_enabled\",\"set_focus_peaking_threshold\",\"set_focus_peaking_color\",\"set_zebra_enabled\",\"set_zebra_threshold\",\"set_false_color_enabled\",\"set_false_color_mode\",\"set_lut_preview_enabled\",\"set_lut_preview_file\",\"set_guides_enabled\",\"set_guides_type\",\"set_guides_thickness\",\"set_center_marker_enabled\",\"set_center_marker_type\",\"set_grayscale_enabled\",\"set_smpte_enabled\",\"set_anamorphic_desqueeze_enabled\",\"set_anamorphic_ratio\",\"set_external_monitor_enabled\",\"set_external_monitor_mode\",\"set_external_monitor_info_overlay\",\"set_external_monitor_orientation\",\"set_ui_orientation\",\"set_camera_controls_opacity\",\"set_camera_controls_mode\",\"set_date_time_overlay_enabled\",\"set_sound_recorder_enabled\",\"set_record_audio_enabled\",\"set_record_gyro_enabled\",\"set_imu_driver\",\"set_imu_orientation\",\"set_imu_sync_offset\",\"adjust_imu_sync_offset\",\"reset_imu_sync_offset\",\"set_imu_level_reference\",\"clear_imu_level_reference\",\"calibrate_gyro\",\"set_live_audio_monitoring_enabled\",\"set_audio_meter_enabled\",\"set_audio_input_device\",\"set_audio_output_device\",\"set_input_volume\",\"set_headphone_volume\",\"set_battery_type\",\"set_battery_capacity\",\"set_custom_battery_wh\",\"set_custom_battery_voltage\",\"set_sleep_mode\",\"set_favorites_order\",\"apply_time_zone\",\"apply_date_time\",\"set_fan_mode\",\"format_media\",\"eject_media\",\"mount_media\",\"run_media_speed_test\",\"capture_photo\",\"record_start\",\"record_stop\",\"preview_subscribe\",\"restart_system\",\"power_off_system\",\"shutdown\"]");

    if (command.name == "get_state") {
        const auto mediaWrite = camera_.mediaWriteStatus();
        const bool soundRecording = soundRecordingActive();
        const bool recording = camera_.isRecording() || soundRecording;
        std::ostringstream body;
        body << "\"state\":\"" << toString(camera_.state()) << "\""
             << ",\"standby\":" << (camera_.state() == CameraState::Standby ? "true" : "false")
             << ",\"recording\":" << (recording ? "true" : "false")
             << ",\"recording_error\":\"" << escapeJson(camera_.recordingError()) << "\""
             << ",\"recording_has_dropped_frames\":" << (camera_.recordingHasDroppedFrames() ? "true" : "false")
             << ",\"sound_recording\":" << (soundRecording ? "true" : "false")
             << ",\"timelapse_active\":" << (camera_.timelapseActive() ? "true" : "false")
             << ",\"timelapse_interval_seconds\":" << camera_.timelapseIntervalSeconds()
             << ",\"timelapse_shot_count\":" << camera_.timelapseShotCount()
             << ",\"still_exposure_active\":" << (camera_.stillExposureStatus().active ? "true" : "false")
             << ",\"still_exposure_remaining_seconds\":" << camera_.stillExposureStatus().remainingSeconds
             << ",\"photo_write_error\":\"" << escapeJson(camera_.photoWriteError()) << "\""
             << ",\"media_writes_pending\":" << (mediaWrite.pending ? "true" : "false")
             << ",\"media_write_revision\":" << mediaWrite.revision
             << ",\"sensor\":\"" << escapeJson(camera_.sensorName()) << "\""
             << "," << settingsBody(camera_.settings());
        return okReply(command.id, body.str());
    }

    if (command.name == "set_iso") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_iso requires value.");
        return camera_.setIso(static_cast<uint32_t>(*value))
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_exposure_compensation") {
        const auto value = command.number("value");
        if (!value || !std::isfinite(*value))
            return errorReply(command.id, "set_exposure_compensation requires a finite EV value.");
        return camera_.setExposureCompensation(*value)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_exposure_metering") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_exposure_metering requires value.");
        return camera_.setExposureMetering(*value)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_iso_auto") {
        return camera_.setIsoAuto()
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_fps") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_fps requires value.");
        return camera_.setFps(*value)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_shutter_us") {
        const auto value = command.number("value");
        if (!value || !std::isfinite(*value) || *value < 1 || *value > 30'000'000)
            return errorReply(command.id, "set_shutter_us requires value.");
        return camera_.setShutterUs(static_cast<uint32_t>(*value))
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_shutter_auto") {
        return camera_.setShutterAuto()
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_wb") {
        const auto kelvin = command.number("kelvin");
        if (!kelvin)
            return errorReply(command.id, "set_wb requires kelvin.");
        return camera_.setWhiteBalanceKelvin(static_cast<uint32_t>(*kelvin))
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_wb_auto") {
        return camera_.setWhiteBalanceAuto()
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_resolution") {
        const auto width = command.number("width");
        const auto height = command.number("height");
        if (!width || !height)
            return errorReply(command.id, "set_resolution requires width and height.");

        Resolution resolution{static_cast<uint32_t>(*width), static_cast<uint32_t>(*height)};
        return camera_.setResolution(resolution)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_record_format") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_record_format requires value.");

        const std::string normalized = normalizeRecordingFormatValue(*value);
        if (normalized.empty())
            return errorReply(command.id, "Unsupported recording format value.");
        const RecordingFormat format = recordingFormatFromNormalizedValue(normalized);
        return camera_.setRecordingFormat(format)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_cdng_compression") {
        const auto value = command.string("value");
        if (!value || (*value != "none" && *value != "lossless_jpeg"))
            return errorReply(command.id, "Compression must be none or lossless_jpeg.");
        const auto compression = *value == "lossless_jpeg"
            ? CdngCompression::LosslessJpeg : CdngCompression::Uncompressed;
        return camera_.setCdngCompression(compression)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_lossy_dng_ratio") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_lossy_dng_ratio requires value.");
        return camera_.setLossyDngRatio(lossyDngRatioFromString(*value))
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_cdng_preview_color_profile") {
        const auto value = command.string("value");
        ProresColorProfile profile;
        if (!value || !parsePreviewColorProfile(*value, profile))
            return errorReply(command.id, "Preview profile must be standard, arri_logc4, sony_slog3, davinci_intermediate or p3_d60_linear.");
        return camera_.setCdngPreviewColorProfile(profile) ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_prores_color_profile") {
        const auto value = command.string("value");
        ProresColorProfile profile;
        if (!value || !parseProresColorProfile(*value, profile))
            return errorReply(command.id, "ProRes colour must be standard, arri_logc4, sony_slog3, davinci_intermediate or p3_d60_linear.");
        return camera_.setProresColorProfile(profile) ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_prores_profile") {
        const auto value = command.string("value");
        if (!value || (*value != "proxy" && *value != "lt" && *value != "422" && *value != "hq"))
            return errorReply(command.id, "ProRes profile must be proxy, lt, 422 or hq.");
        const auto profile = *value == "proxy" ? ProresProfile::Proxy : *value == "lt" ? ProresProfile::LT
            : *value == "hq" ? ProresProfile::HQ : ProresProfile::Standard;
        return camera_.setProresProfile(profile) ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_tuning_file") {
        const auto value = command.string("path");
        if (!value)
            return errorReply(command.id, "set_tuning_file requires path.");
        return camera_.setTuningFile(*value)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_raw_bit_depth") {
        uint32_t bitDepth = 12;
        if (const auto value = command.number("value")) {
            bitDepth = *value == 8.0 ? 8u : (*value == 10.0 ? 10u : 12u);
        } else if (const auto value = command.string("value")) {
            bitDepth = parseRawBitDepthValue(*value);
        } else {
            return errorReply(command.id, "set_raw_bit_depth requires value.");
        }

        return camera_.setRawBitDepth(bitDepth)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_h264_resolution") {
        const auto width = command.number("width");
        const auto height = command.number("height");
        if (!width || !height)
            return errorReply(command.id, "set_h264_resolution requires width and height.");

        Resolution resolution{static_cast<uint32_t>(*width), static_cast<uint32_t>(*height)};
        return camera_.setH264Resolution(resolution)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_timecode_mode") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_timecode_mode requires value.");

        const TimecodeMode mode = (*value == "rec_run") ? TimecodeMode::RecRun : TimecodeMode::FreeRun;
        if (!camera_.setTimecodeMode(mode))
            return errorReply(command.id, camera_.lastError());

        (void)updateSharedUiGeneralSettings({
            { "timecodeMode", mode == TimecodeMode::RecRun ? "Rec Run" : "Free Run" }
        });
        return okReply(command.id, settingsBody(camera_.settings()));
    }

    if (command.name == "set_af_mode") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_af_mode requires value.");
        return camera_.setAutofocusMode(autofocusModeFromCommandValue(*value))
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_af_range") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_af_range requires value.");
        return camera_.setAutofocusRange(autofocusRangeFromCommandValue(*value))
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_af_speed") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_af_speed requires value.");
        return camera_.setAutofocusSpeed(autofocusSpeedFromCommandValue(*value))
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_af_window") {
        const auto x = command.number("x");
        const auto y = command.number("y");
        const auto width = command.number("width");
        const auto height = command.number("height");
        if (!x || !y || !width || !height)
            return errorReply(command.id, "set_af_window requires x, y, width, and height.");
        return camera_.setAutofocusWindow(*x, *y, *width, *height)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    auto handleUiToggle = [this, &command](const char *name, const char *settingKey) -> std::optional<std::string> {
        if (command.name != name)
            return std::nullopt;

        const auto enabled = command.string("enabled");
        if (!enabled)
            return errorReply(command.id, std::string(name) + " requires enabled.");

        const bool value = (*enabled == "true" || *enabled == "1");
        if (!updateSharedUiGeneralSettings({ { settingKey, value ? "true" : "false" } }))
            return errorReply(command.id, "Could not update shared UI settings.");

        return okReply(command.id, settingsBody(camera_.settings()));
    };

    if (const auto reply = handleUiToggle("set_focus_peaking_enabled", "focusPeakingEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_zebra_enabled", "zebraEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_false_color_enabled", "falseColorEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_lut_preview_enabled", "lutPreviewEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_guides_enabled", "guidesEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_center_marker_enabled", "centerMarkerEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_grayscale_enabled", "grayscaleEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_smpte_enabled", "smpteEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_anamorphic_desqueeze_enabled", "anamorphicDesqueezeEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_external_monitor_enabled", "externalMonitorEnabled"))
        return *reply;
    if (command.name == "set_sound_recorder_enabled") {
        const auto enabled = command.string("enabled");
        if (!enabled)
            return errorReply(command.id, "set_sound_recorder_enabled requires enabled.");

        const bool value = (*enabled == "true" || *enabled == "1");
        if (value && detectAudioDevices().inputOptions.empty())
            return errorReply(command.id, "Sound Recorder requires an audio input device.");

        std::vector<std::pair<std::string, std::string>> updates{
            { "soundRecorderEnabled", value ? "true" : "false" },
        };
        if (value) {
            camera_.stopTimelapse();
            if (!camera_.setStillExposureMode(false, readSharedUiSettings().photoLongExposureEnabled))
                return errorReply(command.id, camera_.lastError());
            updates.emplace_back("photoModeEnabled", "false");
            updates.emplace_back("photoTimelapse", "Off");
        }

        if (!updateSharedUiGeneralSettings(updates))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, settingsBody(camera_.settings()));
    }
    if (command.name == "set_capture_mode") {
        const auto mode = command.string("value").value_or("");
        if (mode != "clip" && mode != "still" && mode != "timelapse")
            return errorReply(command.id, "Capture mode must be clip, still or timelapse.");
        if (camera_.isRecording() || soundRecordingActive() || camera_.timelapseActive())
            return errorReply(command.id, "Stop capture before changing mode.");
        const auto current = readSharedUiSettings();
        const auto ui = readSharedUiGeneralSettingsMap();
        std::string interval = "Off";
        if (mode == "timelapse") {
            interval = current.photoTimelapse;
            if (interval == "Off") {
                const auto saved = ui.find("lastPhotoTimelapse");
                interval = saved == ui.end() ? "5s" : normalizePhotoTimelapseValue(saved->second);
            }
            if (interval == "Off") interval = "5s";
        }
        const bool still = mode != "clip";
        const std::string format = mode == "timelapse" && !photoFormatIsTimelapseSafeValue(current.photoFormat)
            ? "DNG" : current.photoFormat;
        const auto previous = camera_.settings();
        if (!camera_.setStillExposureMode(still, current.photoLongExposureEnabled, photoTimelapseSecondsFromValue(interval)))
            return errorReply(command.id, camera_.lastError());
        std::vector<std::pair<std::string,std::string>> updates{
            {"photoModeEnabled", still ? "true" : "false"}, {"photoTimelapse", interval},
            {"photoFormat", format}, {"soundRecorderEnabled", "false"}
        };
        if (current.photoTimelapse != "Off") updates.emplace_back("lastPhotoTimelapse", current.photoTimelapse);
        if (mode == "timelapse") updates.emplace_back("lastPhotoTimelapse", interval);
        if (!updateSharedUiGeneralSettings(updates)) {
            camera_.applyCapturePreset(previous);
            return errorReply(command.id, "Could not save capture mode.");
        }
        return okReply(command.id, settingsBody(camera_.settings()) +
            ",\"photo_timelapse\":\"" + escapeJson(interval) + "\",\"photo_format\":\"" + escapeJson(format) + "\"");
    }
    if (command.name == "set_photo_mode_enabled") {
        const auto enabled = command.string("enabled");
        if (!enabled)
            return errorReply(command.id, "set_photo_mode_enabled requires enabled.");

        const bool value = (*enabled == "true" || *enabled == "1");
        if (!value)
            camera_.stopTimelapse();

        const SharedUiSettingsSnapshot currentUi = readSharedUiSettings();
        const CaptureSettings previous = camera_.settings();
        if (!camera_.setStillExposureMode(value, currentUi.photoLongExposureEnabled,
                                          photoTimelapseSecondsFromValue(currentUi.photoTimelapse)))
            return errorReply(command.id, camera_.lastError());
        std::vector<std::pair<std::string, std::string>> updates{
            { "photoModeEnabled", value ? "true" : "false" },
            { "photoTimelapse", value ? currentUi.photoTimelapse : "Off" },
        };
        if (value)
            updates.emplace_back("soundRecorderEnabled", "false");

        if (!updateSharedUiGeneralSettings(updates)) {
            camera_.applyCapturePreset(previous);
            return errorReply(command.id, "Could not update shared UI settings.");
        }
        return okReply(command.id, settingsBody(camera_.settings()));
    }
    if (command.name == "set_photo_bracketing_enabled") {
        const auto enabled = command.string("enabled");
        if (!enabled || (*enabled != "true" && *enabled != "false" && *enabled != "1" && *enabled != "0"))
            return errorReply(command.id, "set_photo_bracketing_enabled requires enabled=true or false.");
        const bool value = *enabled == "true" || *enabled == "1";
        auto next = camera_.settings();
        next.bracketingEnabled = value;
        if (!camera_.applyCapturePreset(next, [&] {
                return updateSharedUiGeneralSettings({{"photoBracketingEnabled", value ? "true" : "false"}});
            })) return errorReply(command.id, camera_.lastError());
        return okReply(command.id, settingsBody(camera_.settings()));
    }
    if (command.name == "set_photo_bracketing_ev") {
        const auto value = command.number("value");
        if (!value || !std::isfinite(*value) || *value < 1.0 / 3.0 - 0.0001 || *value > 3.0)
            return errorReply(command.id, "Bracket EV offset must be between 1/3 and 3 EV.");
        auto next = camera_.settings();
        next.bracketingEv = std::round(*value * 3.0) / 3.0;
        if (!camera_.applyCapturePreset(next, [&] {
                return updateSharedUiGeneralSettings({{"photoBracketingEv", compactNumber(next.bracketingEv)}});
            })) return errorReply(command.id, camera_.lastError());
        return okReply(command.id, settingsBody(camera_.settings()));
    }
    if (command.name == "set_photo_long_exposure_enabled") {
        const auto enabled = command.string("enabled");
        if (!enabled)
            return errorReply(command.id, "set_photo_long_exposure_enabled requires enabled.");
        const bool value = *enabled == "true" || *enabled == "1";
        const auto current = readSharedUiSettings();
        auto next = camera_.settings();
        next.photoModeEnabled = current.photoModeEnabled;
        next.longExposureEnabled = value;
        next.timelapseExposureLimitUs = next.photoModeEnabled
            ? timelapseExposureLimitUs(photoTimelapseSecondsFromValue(current.photoTimelapse)) : 0;
        if (!camera_.applyCapturePreset(next, [&] {
                return updateSharedUiGeneralSettings({{"photoLongExposureEnabled", value ? "true" : "false"}});
            }))
            return errorReply(command.id, camera_.lastError());
        return okReply(command.id, settingsBody(camera_.settings()));
    }
    if (const auto reply = handleUiToggle("set_date_time_overlay_enabled", "dateTimeOverlayEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_record_audio_enabled", "recordAudioEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_record_gyro_enabled", "recordGyroEnabled"))
        return *reply;
    if (command.name == "set_imu_driver") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_imu_driver requires value.");

        std::string errorMessage;
        if (!Mpu6050RecordingSession::setConfiguredDriver(*value, &errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, imuStatusBody(false));
    }
    if (command.name == "set_imu_orientation") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_imu_orientation requires value.");

        std::string errorMessage;
        if (!Mpu6050RecordingSession::setConfiguredOrientation(*value, &errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, imuStatusBody(false));
    }
    if (command.name == "set_imu_sync_offset") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_imu_sync_offset requires value.");

        std::string errorMessage;
        if (!Mpu6050RecordingSession::setConfiguredSyncOffsetMs(*value, &errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, imuStatusBody(false));
    }
    if (command.name == "adjust_imu_sync_offset") {
        const auto delta = command.number("delta");
        if (!delta)
            return errorReply(command.id, "adjust_imu_sync_offset requires delta.");

        const auto imu = Mpu6050RecordingSession::configuredSensorSnapshot(false);
        std::string errorMessage;
        if (!Mpu6050RecordingSession::setConfiguredSyncOffsetMs(imu.syncOffsetMs + *delta, &errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, imuStatusBody(false));
    }
    if (command.name == "reset_imu_sync_offset") {
        std::string errorMessage;
        if (!Mpu6050RecordingSession::setConfiguredSyncOffsetMs(0.0, &errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, imuStatusBody(false));
    }
    if (command.name == "set_imu_level_reference") {
        std::string errorMessage;
        if (!Mpu6050RecordingSession::setConfiguredLevelReference(&errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, imuStatusBody(false));
    }
    if (command.name == "clear_imu_level_reference") {
        std::string errorMessage;
        if (!Mpu6050RecordingSession::clearConfiguredLevelReference(&errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, imuStatusBody(false));
    }
    if (command.name == "calibrate_gyro") {
        if (camera_.isRecording() || soundRecordingActive() || camera_.timelapseActive())
            return errorReply(command.id, "Stop recording before calibrating gyro.");

        const double seconds = command.number("seconds").value_or(2.5);
        Mpu6050RecordingSession::CalibrationResult result;
        std::string errorMessage;
        if (!Mpu6050RecordingSession::calibrateConfiguredSensor(seconds, &result, &errorMessage))
            return errorReply(command.id, errorMessage);

        std::ostringstream body;
        body << "\"gyro_calibrated\":true"
             << ",\"samples\":" << result.sampleCount
             << ",\"gyro_bias_x\":" << jsonNumber(result.gyroBiasXRaw, 3)
             << ",\"gyro_bias_y\":" << jsonNumber(result.gyroBiasYRaw, 3)
             << ",\"gyro_bias_z\":" << jsonNumber(result.gyroBiasZRaw, 3)
             << ",\"config_path\":\"" << escapeJson(result.configPath) << "\"";
        return okReply(command.id, body.str());
    }
    if (const auto reply = handleUiToggle("set_live_audio_monitoring_enabled", "liveAudioMonitoringEnabled"))
        return *reply;
    if (const auto reply = handleUiToggle("set_audio_meter_enabled", "audioMeterEnabled"))
        return *reply;

    if (command.name == "set_focus_peaking_threshold") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_focus_peaking_threshold requires value.");
        const double clampedValue = std::clamp(*value, 0.0, 1.0);
        if (!updateSharedUiGeneralSettings({ { "focusPeakingThreshold", jsonNumber(clampedValue, 2) } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"focus_peaking_threshold\":" + jsonNumber(clampedValue, 2));
    }

    if (command.name == "set_focus_peaking_color") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_focus_peaking_color requires value.");

        const std::string normalized = normalizeFocusPeakingColorValue(*value);
        if (trimAscii(*value) != normalized)
            return errorReply(command.id, "Unsupported focus peaking color.");
        if (!updateSharedUiGeneralSettings({ { "focusPeakingColor", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"focus_peaking_color\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_zebra_threshold") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_zebra_threshold requires value.");
        const double clampedValue = std::clamp(*value, 0.0, 1.0);
        if (!updateSharedUiGeneralSettings({ { "zebraThreshold", jsonNumber(clampedValue, 2) } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"zebra_threshold\":" + jsonNumber(clampedValue, 2));
    }

    if (command.name == "set_false_color_mode") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_false_color_mode requires value.");

        const std::string normalized = normalizeFalseColorModeLabel(*value);
        if (trimAscii(*value) != normalized)
            return errorReply(command.id, "Unsupported false color mode.");
        const int modeValue = falseColorModeValueFromLabel(normalized);
        if (!updateSharedUiGeneralSettings({ { "falseColorMode", std::to_string(modeValue) } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"false_color_mode\":" + std::to_string(modeValue));
    }

    if (command.name == "set_lut_preview_file") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_lut_preview_file requires value.");

        const std::string trimmedValue = trimAscii(*value);
        if (trimmedValue.empty()) {
            if (!updateSharedUiGeneralSettings({ { "lutPreviewFile", "" } }))
                return errorReply(command.id, "Could not update shared UI settings.");
            return okReply(command.id, "\"lut_preview_file\":\"\"");
        }

        if (!lutPreviewFileAllowed(options_.mediaRoot, trimmedValue))
            return errorReply(command.id, "Unsupported LUT preview file.");

        std::error_code ec;
        const std::filesystem::path canonical = std::filesystem::weakly_canonical(trimmedValue, ec);
        const std::string normalized = ec
            ? std::filesystem::path(trimmedValue).lexically_normal().string()
            : canonical.string();

        if (!updateSharedUiGeneralSettings({ { "lutPreviewFile", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"lut_preview_file\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_guides_type") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_guides_type requires value.");

        const std::string normalized = normalizeGuidesTypeValue(*value);
        if (trimAscii(*value) != normalized)
            return errorReply(command.id, "Unsupported guides type.");
        if (!updateSharedUiGeneralSettings({ { "guidesType", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"guides_type\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_guides_thickness") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_guides_thickness requires value.");
        const int clampedValue = static_cast<int>(std::clamp(std::llround(*value), 1ll, 4ll));
        if (!updateSharedUiGeneralSettings({ { "guidesThickness", std::to_string(clampedValue) } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"guides_thickness\":" + std::to_string(clampedValue));
    }

    if (command.name == "set_center_marker_type") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_center_marker_type requires value.");

        const std::string normalized = normalizeCenterMarkerTypeValue(*value);
        if (trimAscii(*value) != normalized)
            return errorReply(command.id, "Unsupported center marker type.");
        if (!updateSharedUiGeneralSettings({ { "centerMarkerType", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"center_marker_type\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_anamorphic_ratio") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_anamorphic_ratio requires value.");

        const std::string normalized = normalizeAnamorphicRatioValue(*value);
        if (trimAscii(*value) != normalized)
            return errorReply(command.id, "Unsupported anamorphic ratio.");
        if (!updateSharedUiGeneralSettings({ { "anamorphicRatio", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"anamorphic_ratio\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_external_monitor_mode") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_external_monitor_mode requires value.");

        const std::string normalized = normalizeExternalMonitorModeValue(*value);
        if (trimAscii(*value) != normalized)
            return errorReply(command.id, "Unsupported external monitor mode.");
        if (!updateSharedUiGeneralSettings({ { "externalMonitorMode", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"external_monitor_mode\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_external_monitor_info_overlay") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_external_monitor_info_overlay requires value.");

        const std::string normalized = normalizeExternalMonitorInfoOverlayValue(*value);
        if (trimAscii(*value) != normalized)
            return errorReply(command.id, "Unsupported external monitor info overlay value.");
        if (!updateSharedUiGeneralSettings({ { "externalMonitorInfoOverlay", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"external_monitor_info_overlay\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_external_monitor_orientation") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_external_monitor_orientation requires value.");

        const std::string normalized = normalizeExternalMonitorOrientationValue(*value);
        if (trimAscii(*value) != normalized)
            return errorReply(command.id, "Unsupported external monitor orientation.");
        if (!updateSharedUiGeneralSettings({ { "externalMonitorOrientation", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"external_monitor_orientation\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_photo_timer") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_photo_timer requires value.");

        const std::string normalized = normalizePhotoTimerValue(*value);
        if (trimAscii(*value) != normalized)
            return errorReply(command.id, "Unsupported photo timer value.");
        if (!updateSharedUiGeneralSettings({ { "photoTimer", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"photo_timer\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_photo_burst") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_photo_burst requires value.");

        const std::string normalized = normalizePhotoBurstValue(*value);
        if (trimAscii(*value) != normalized)
            return errorReply(command.id, "Unsupported photo burst value.");
        if (!updateSharedUiGeneralSettings({ { "photoBurst", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"photo_burst\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_photo_format") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_photo_format requires value.");

        const std::string trimmed = trimAscii(*value);
        const std::string lower = toLowerAsciiCopy(trimmed);
        const bool supported = lower == "dng" || lower == "jpg" || lower == "jpeg" ||
                               lower == "png" || lower == "tif" || lower == "tiff" ||
                               lower == "dng+jpeg" || lower == "dng + jpeg" ||
                               lower == "dng_jpeg" || lower == "dng-jpeg" ||
                               lower == "dng+png" || lower == "dng + png" ||
                               lower == "dng_png" || lower == "dng-png" ||
                               lower == "dng+tif" || lower == "dng + tif" ||
                               lower == "dng_tif" || lower == "dng-tif" ||
                               lower == "dng+tiff" || lower == "dng + tiff" ||
                               lower == "dng_tiff" || lower == "dng-tiff";
        if (!supported)
            return errorReply(command.id, "Unsupported photo format value.");

        const std::string normalized = normalizePhotoFormatValue(*value);
        const SharedUiSettingsSnapshot uiSettings = readSharedUiSettings();
        if (uiSettings.photoTimelapse != "Off" && !photoFormatIsTimelapseSafeValue(normalized))
            return errorReply(command.id, "Timelapse supports DNG, JPEG, PNG, or TIFF only.");

        if (!updateSharedUiGeneralSettings({ { "photoFormat", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"photo_format\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_photo_timelapse") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_photo_timelapse requires value.");

        const std::string normalized = normalizePhotoTimelapseValue(*value);
        const auto requestedInterval = toLowerAsciiCopy(trimAscii(*value));
        if (normalized == "Off" && requestedInterval != "off" && requestedInterval != "none" && requestedInterval != "disabled")
            return errorReply(command.id, "Timelapse interval must be 0.1-86400 seconds (one decimal place) or Off.");
        const double intervalSeconds = photoTimelapseSecondsFromValue(normalized);
        const SharedUiSettingsSnapshot uiSettings = readSharedUiSettings();
        const std::string selectedFormat = normalizePhotoFormatValue(
            command.string("format").value_or(uiSettings.photoFormat));

        if (intervalSeconds > 0 && !photoFormatIsTimelapseSafeValue(selectedFormat))
            return errorReply(command.id, "Timelapse supports DNG, JPEG, PNG, or TIFF only.");

        if (intervalSeconds == 0) {
            camera_.stopTimelapse();
            if (!camera_.setStillExposureMode(camera_.settings().photoModeEnabled, uiSettings.photoLongExposureEnabled, 0))
                return errorReply(command.id, camera_.lastError());
            if (!updateSharedUiGeneralSettings({ { "photoTimelapse", "Off" } }))
                return errorReply(command.id, "Could not update shared UI settings.");
            return okReply(command.id, "\"photo_timelapse\":\"Off\",\"timelapse_active\":false," + settingsBody(camera_.settings()));
        }

        const CaptureSettings previous = camera_.settings();
        if (camera_.timelapseActive()) {
            if (!camera_.startTimelapse(photoCaptureFormatFromValue(selectedFormat), intervalSeconds))
                return errorReply(command.id, camera_.lastError());
        } else if (!camera_.setStillExposureMode(true, uiSettings.photoLongExposureEnabled, intervalSeconds))
            return errorReply(command.id, camera_.lastError());

        if (!updateSharedUiGeneralSettings({
                { "photoTimelapse", normalized },
                { "lastPhotoTimelapse", normalized },
                { "photoFormat", selectedFormat },
                { "photoModeEnabled", "true" },
                { "soundRecorderEnabled", "false" },
            })) {
            if (!camera_.timelapseActive()) camera_.applyCapturePreset(previous);
            return errorReply(command.id, "Could not update shared UI settings.");
        }

        return okReply(command.id,
                       "\"photo_timelapse\":\"" + escapeJson(normalized) +
                       "\",\"photo_format\":\"" + escapeJson(selectedFormat) +
                       "\",\"timelapse_active\":" + std::string(camera_.timelapseActive() ? "true" : "false") +
                       ",\"interval_seconds\":" +
                       std::to_string(intervalSeconds) + "," + settingsBody(camera_.settings()));
    }

    if (command.name == "set_ui_orientation") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_ui_orientation requires value.");

        const std::string normalized = normalizeUiOrientationValue(*value);
        if (!updateSharedUiGeneralSettings({ { "uiOrientation", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, settingsBody(camera_.settings()));
    }

    if (command.name == "set_camera_controls_opacity") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_camera_controls_opacity requires value.");

        const double clampedValue = std::clamp(*value, 0.15, 1.0);
        if (!updateSharedUiGeneralSettings({ { "cameraControlsOpacity", compactNumber(clampedValue) } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, settingsBody(camera_.settings()));
    }

    if (command.name == "set_camera_controls_mode") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_camera_controls_mode requires value.");

        const std::string normalized = normalizeCameraControlsModeValue(*value);
        if (!updateSharedUiGeneralSettings({ { "cameraControlsMode", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, settingsBody(camera_.settings()));
    }

    if (command.name == "audio_capabilities") {
        if (camera_.isRecording() || soundRecordingActive() || camera_.timelapseActive())
            return errorReply(command.id, "Stop recording before checking audio capabilities.");
        const auto ui = readSharedUiSettings();
        const auto devices = detectAudioDevices();
        const auto selected = resolveAudioDeviceSelection(command.string("device").value_or(ui.audioInputDevice), devices.inputOptions, false);
        if (selected.empty()) return errorReply(command.id, "No audio input device detected.");
        std::string error;
        const auto modes = TimestampedAudioCapture::probe(audioDeviceIdFromSelectionValue(selected), &error);
        if (modes.empty()) return errorReply(command.id, error);
        std::ostringstream body;
        body << "\"device\":\"" << escapeJson(selected) << "\",\"modes\":[";
        for (size_t i = 0; i < modes.size(); ++i) {
            if (i) body << ',';
            body << "{\"format\":\"" << modes[i].format << "\",\"rate\":" << modes[i].rate
                 << ",\"channels\":" << modes[i].channels << '}';
        }
        body << ']';
        return okReply(command.id, body.str());
    }

    if (command.name == "set_audio_capture_options") {
        if (camera_.isRecording() || soundRecordingActive() || camera_.timelapseActive())
            return errorReply(command.id, "Stop recording before changing audio capture settings.");
        auto ui = readSharedUiSettings();
        if (const auto format = command.string("format")) {
            if (!validAudioFormat(*format)) return errorReply(command.id, "Invalid audio sample format.");
            ui.audioSampleFormat = *format;
        }
        if (const auto rate = command.number("rate")) {
            if (!std::isfinite(*rate) || *rate < 0 || *rate > 192000 || *rate != std::floor(*rate))
                return errorReply(command.id, "Invalid audio sample rate.");
            ui.audioSampleRate = int(*rate);
        }
        if (const auto channels = command.string("channels")) {
            unsigned first = 0, count = 0;
            if (!audioChannelRange(*channels, first, count)) return errorReply(command.id, "Invalid audio input channels.");
            ui.audioChannels = *channels;
        }
        if (!updateSharedUiGeneralSettings({{"audioSampleFormat", ui.audioSampleFormat},
                {"audioSampleRate", std::to_string(ui.audioSampleRate)}, {"audioChannels", ui.audioChannels}}))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"audio_options_saved\":true");
    }

    if (command.name == "set_audio_input_device") {
        if (camera_.isRecording() || soundRecordingActive())
            return errorReply(command.id, "Stop recording before changing audio input.");
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_audio_input_device requires value.");
        const AudioDeviceSnapshot audioDevices = detectAudioDevices();
        const std::string normalized = resolveAudioDeviceSelection(*value, audioDevices.inputOptions, false);
        if (!value->empty() && normalized.empty())
            return errorReply(command.id, "Selected audio input device is unavailable.");
        if (!updateSharedUiGeneralSettings({ { "audioInputDevice", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"audio_input_device\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_audio_output_device") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_audio_output_device requires value.");
        const AudioDeviceSnapshot audioDevices = detectAudioDevices();
        const std::string normalized = resolveAudioDeviceSelection(*value, audioDevices.outputOptions, true);
        if (!normalizeAudioSelectionValue(*value, true).empty() && normalized.empty())
            return errorReply(command.id, "Selected audio output device is unavailable.");
        if (!updateSharedUiGeneralSettings({ { "audioOutputDevice", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"audio_output_device\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_input_volume") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_input_volume requires value.");
        const int clampedValue = static_cast<int>(std::clamp(std::llround(*value), 0ll, 100ll));
        if (!updateSharedUiGeneralSettings({ { "inputVolume", std::to_string(clampedValue) } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"input_volume\":" + std::to_string(clampedValue));
    }

    if (command.name == "set_headphone_volume") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_headphone_volume requires value.");
        const int clampedValue = static_cast<int>(std::clamp(std::llround(*value), 0ll, 100ll));
        if (!updateSharedUiGeneralSettings({ { "headphoneVolume", std::to_string(clampedValue) } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"headphone_volume\":" + std::to_string(clampedValue));
    }

    if (command.name == "set_battery_type") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_battery_type requires value.");

        const std::string normalizedType = normalizeBatteryTypeValue(*value);
        const SharedUiSettingsSnapshot currentUi = readSharedUiSettings();
        const std::string nextCapacity = normalizedBatteryCapacityForType(normalizedType, currentUi.batteryCapacity);
        const double fullVoltage = normalizedType == "Custom"
            ? currentUi.customBatteryFullVoltage
            : batteryDefaultFullVoltage(normalizedType);
        const double emptyVoltage = normalizedType == "Custom"
            ? currentUi.customBatteryEmptyVoltage
            : batteryDefaultEmptyVoltage(normalizedType);

        if (!updateSharedUiGeneralSettings({
                { "batteryType", normalizedType },
                { "batteryCapacity", nextCapacity },
                { "customBatteryFullVoltage", compactNumber(fullVoltage) },
                { "customBatteryEmptyVoltage", compactNumber(std::min(emptyVoltage, fullVoltage - 0.1)) },
            })) {
            return errorReply(command.id, "Could not update shared UI settings.");
        }

        return okReply(command.id,
                       "\"battery_type\":\"" + escapeJson(normalizedType) + "\""
                       + ",\"battery_capacity\":\"" + escapeJson(nextCapacity) + "\"");
    }

    if (command.name == "set_battery_capacity") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_battery_capacity requires value.");

        const SharedUiSettingsSnapshot currentUi = readSharedUiSettings();
        const std::string normalized = normalizedBatteryCapacityForType(currentUi.batteryType, *value);
        if (normalized != normalizeBatteryCapacityValue(*value))
            return errorReply(command.id, "Unsupported battery capacity value.");
        if (!updateSharedUiGeneralSettings({ { "batteryCapacity", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"battery_capacity\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_custom_battery_wh") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_custom_battery_wh requires value.");

        const int clampedValue = static_cast<int>(std::clamp(std::llround(*value), 10ll, 500ll));
        if (!updateSharedUiGeneralSettings({ { "customBatteryWh", std::to_string(clampedValue) } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"custom_battery_wh\":" + std::to_string(clampedValue));
    }

    if (command.name == "set_custom_battery_voltage") {
        const SharedUiSettingsSnapshot currentUi = readSharedUiSettings();
        const auto full = command.number("full");
        const auto empty = command.number("empty");
        if (!full && !empty)
            return errorReply(command.id, "set_custom_battery_voltage requires full or empty.");

        double fullVoltage = full
            ? clampBatteryVoltageValue(*full, currentUi.customBatteryFullVoltage)
            : currentUi.customBatteryFullVoltage;
        double emptyVoltage = empty
            ? clampBatteryVoltageValue(*empty, currentUi.customBatteryEmptyVoltage)
            : currentUi.customBatteryEmptyVoltage;
        if (emptyVoltage >= fullVoltage)
            emptyVoltage = std::max(0.5, fullVoltage - 0.1);

        if (!updateSharedUiGeneralSettings({
                { "customBatteryFullVoltage", compactNumber(fullVoltage) },
                { "customBatteryEmptyVoltage", compactNumber(emptyVoltage) },
            })) {
            return errorReply(command.id, "Could not update shared UI settings.");
        }

        return okReply(command.id,
                       "\"custom_battery_full_voltage\":" + compactNumber(fullVoltage)
                       + ",\"custom_battery_empty_voltage\":" + compactNumber(emptyVoltage));
    }

    if (command.name == "set_sleep_mode") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_sleep_mode requires value.");

        const std::string normalized = normalizeSleepModeValue(*value);
        if (trimAscii(*value) != normalized)
            return errorReply(command.id, "Unsupported sleep mode value.");
        if (!updateSharedUiGeneralSettings({ { "sleepMode", normalized } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"sleep_mode\":\"" + escapeJson(normalized) + "\"");
    }

    if (command.name == "set_favorites_order") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_favorites_order requires value.");

        const std::vector<std::string> normalized = parseFavoritesOrderValue(*value);
        if (!updateSharedUiGeneralSettings({ { "favoritesOrder", encodeSimpleListValue(normalized) } }))
            return errorReply(command.id, "Could not update shared UI settings.");
        return okReply(command.id, "\"favorites_order\":" + jsonStringArray(normalized));
    }

    if (command.name == "apply_preset") {
        std::vector<std::pair<std::string, std::string>> uiUpdates;
        auto addBoolUpdate = [&](const char *key) -> bool {
            const auto value = command.string(key);
            if (!value)
                return true;
            uiUpdates.emplace_back(key, parseUiSettingsBool(*value, false) ? "true" : "false");
            return true;
        };
        auto addIntegerUpdate = [&](const char *key) -> bool {
            const auto value = command.string(key);
            if (!value)
                return true;
            const auto parsed = parseIntegerValue(*value);
            if (!parsed) {
                return false;
            }
            uiUpdates.emplace_back(key, std::to_string(*parsed));
            return true;
        };
        auto addDoubleUpdate = [&](const char *key) -> bool {
            const auto value = command.string(key);
            if (!value)
                return true;
            const auto parsed = parseDoubleValue(*value);
            if (!parsed) {
                return false;
            }
            uiUpdates.emplace_back(key, compactNumber(*parsed));
            return true;
        };
        auto addStringUpdate = [&](const char *key) {
            if (const auto value = command.string(key))
                uiUpdates.emplace_back(key, trimAscii(*value));
        };

        const std::string presetLabel =
            command.string("presetName").value_or(std::string("preset"));

        CaptureSettings presetSettings = camera_.settings();
        if (const auto value = command.string("cameraExposureMetering")) {
            const auto mode = normalizeExposureMetering(*value);
            if (mode.empty()) return errorReply(command.id, "Invalid preset metering mode.");
            presetSettings.exposureMetering = mode;
        }
        if (const auto value = command.string("cameraExposureCompensation")) {
            const auto ev = parseDoubleValue(*value);
            if (!ev || !std::isfinite(*ev) || *ev < -3.0 || *ev > 3.0)
                return errorReply(command.id, "Invalid preset exposure compensation.");
            presetSettings.exposureCompensation = std::round(*ev * 3.0) / 3.0;
        }
        if (const auto value = command.string("photoModeEnabled"))
            presetSettings.photoModeEnabled = parseUiSettingsBool(*value, false);
        if (const auto value = command.string("photoLongExposureEnabled"))
            presetSettings.longExposureEnabled = parseUiSettingsBool(*value, false);
        addBoolUpdate("photoLongExposureEnabled");
        if (const auto value = command.string("photoBracketingEnabled"))
            presetSettings.bracketingEnabled = parseUiSettingsBool(*value, false);
        if (!addBoolUpdate("photoBracketingEnabled"))
            return errorReply(command.id, "Invalid preset bracketing toggle.");
        if (const auto value = command.string("photoBracketingEv")) {
            const auto ev = parseDoubleValue(*value);
            if (!ev || !std::isfinite(*ev) || *ev < 1.0 / 3.0 - 0.0001 || *ev > 3.0)
                return errorReply(command.id, "Invalid preset bracket EV offset.");
            presetSettings.bracketingEv = std::round(*ev * 3.0) / 3.0;
            uiUpdates.emplace_back("photoBracketingEv", compactNumber(presetSettings.bracketingEv));
        }
        if (const auto value = command.string("cameraRecordingFormat")) {
            const auto normalized = normalizeRecordingFormatValue(*value);
            if (normalized.empty()) return errorReply(command.id, "Invalid preset recording format.");
            presetSettings.recordingFormat = recordingFormatFromNormalizedValue(normalized);
        }
        if (const auto value = command.string("cameraTuningFile"))
            presetSettings.tuningFile = *value;
        if (const auto value = command.string("cameraProresColorProfile")) {
            if (!parseProresColorProfile(*value, presetSettings.proresColorProfile))
                return errorReply(command.id, "Invalid preset ProRes colour profile.");
        }
        if (const auto value = command.string("cameraCdngPreviewColorProfile")) {
            if (!parsePreviewColorProfile(*value, presetSettings.cdngPreviewColorProfile))
                return errorReply(command.id, "Invalid preset cDNG preview colour profile.");
        }
        if (const auto value = command.string("cameraProresProfile")) {
            if (*value != "hq" && *value != "422" && *value != "lt" && *value != "proxy")
                return errorReply(command.id, "Invalid preset ProRes profile.");
            presetSettings.proresProfile = *value == "hq" ? ProresProfile::HQ : *value == "lt"
                ? ProresProfile::LT : *value == "proxy" ? ProresProfile::Proxy : ProresProfile::Standard;
        }
        if (const auto value = command.string("cameraCdngCompression")) {
            if (*value == "lossy_3to1" || *value == "lossy_5to1") {
                presetSettings.recordingFormat = RecordingFormat::LossyDng;
                presetSettings.lossyDngRatio = *value == "lossy_5to1" ? LossyDngRatio::R5 : LossyDngRatio::R3;
                presetSettings.cdngCompression = CdngCompression::Uncompressed;
            } else {
                if (*value != "none" && *value != "lossless_jpeg")
                    return errorReply(command.id, "Invalid preset cDNG compression.");
                presetSettings.cdngCompression = *value == "lossless_jpeg"
                    ? CdngCompression::LosslessJpeg : CdngCompression::Uncompressed;
            }
        }
        if (const auto value = command.string("cameraLossyDngRatio")) {
            if (*value != "3to1" && *value != "5to1")
                return errorReply(command.id, "Invalid preset lossy cDNG ratio.");
            presetSettings.lossyDngRatio = lossyDngRatioFromString(*value);
        }
        if (const auto value = command.string("cameraRawBitDepth")) {
            const auto bits = parseRawBitDepthValue(*value);
            if (bits != 8 && bits != 10 && bits != 12)
                return errorReply(command.id, "Invalid preset bit depth.");
            presetSettings.rawBitDepth = bits;
        }
        if (const auto width = command.number("cameraResolutionWidth")) {
            const auto height = command.number("cameraResolutionHeight");
            if (!height || !std::isfinite(*width) || !std::isfinite(*height) ||
                *width < 2 || *height < 2 || *width > 16384 || *height > 16384)
                return errorReply(command.id, "Invalid preset resolution.");
            presetSettings.resolution = {uint32_t(*width), uint32_t(*height)};
        } else if (const auto value = command.string("cameraResolution")) {
            if (!parseResolutionText(*value, &presetSettings.resolution.width, &presetSettings.resolution.height))
                return errorReply(command.id, "Invalid preset resolution.");
        }
        if (const auto value = command.string("cameraH264Resolution")) {
            const auto text = toLowerAsciiCopy(trimAscii(*value));
            if (text == "match" || text == "match capture")
                presetSettings.h264Resolution = {0, 0};
            else if (!parseResolutionText(text, &presetSettings.h264Resolution.width, &presetSettings.h264Resolution.height))
                return errorReply(command.id, "Invalid preset processed-video resolution.");
        }
        if (const auto value = command.string("cameraFps")) {
            const auto fps = parseDoubleValue(*value);
            if (!fps || !std::isfinite(*fps) || *fps <= 0 || *fps > 280)
                return errorReply(command.id, "Invalid preset FPS.");
            presetSettings.fps = *fps;
        }
        if (const auto value = command.string("cameraIso")) {
            presetSettings.isoAuto = isAutoValue(*value);
            if (!presetSettings.isoAuto) {
                const auto iso = parseDoubleValue(*value);
                if (!iso || !std::isfinite(*iso) || *iso < 50 || *iso > 51200)
                    return errorReply(command.id, "Invalid preset ISO.");
                presetSettings.iso = uint32_t(std::llround(*iso));
            }
        }
        bool useSpeed = command.string("cameraShutterMode").value_or("") == "speed";
        auto shutter = useSpeed ? command.string("cameraShutterSpeed") : command.string("cameraShutterAngle");
        if (!shutter) {
            shutter = useSpeed ? command.string("cameraShutterAngle") : command.string("cameraShutterSpeed");
            useSpeed = !useSpeed;
        }
        if (shutter) {
            presetSettings.shutterAuto = isAutoValue(*shutter);
            if (!presetSettings.shutterAuto) {
                std::string value = trimAscii(*shutter);
                const bool seconds = useSpeed && !value.empty() && value.back() == 's';
                if (seconds) value.pop_back();
                if (useSpeed && value.rfind("1/", 0) == 0) value = value.substr(2);
                if (!useSpeed && value.size() >= 2 &&
                    static_cast<unsigned char>(value[value.size()-2]) == 0xC2 &&
                    static_cast<unsigned char>(value.back()) == 0xB0) value.resize(value.size()-2);
                const auto parsed = parseDoubleValue(value);
                if (!parsed || !std::isfinite(*parsed) || *parsed <= 0 || (!useSpeed && *parsed > 360))
                    return errorReply(command.id, "Invalid preset shutter.");
                const double microseconds = seconds ? 1000000.0 * *parsed : useSpeed ? 1000000.0 / *parsed
                    : *parsed / 360.0 * 1000000.0 / presetSettings.fps;
                if (microseconds > 30000000) return errorReply(command.id, "Invalid preset exposure time.");
                presetSettings.shutterUs = uint32_t(std::max(1ll, std::llround(microseconds)));
            }
        }
        if (const auto value = command.string("cameraWhiteBalance")) {
            presetSettings.whiteBalanceAuto = isAutoValue(*value);
            if (!presetSettings.whiteBalanceAuto) {
                auto text = trimAscii(*value);
                if (!text.empty() && (text.back() == 'K' || text.back() == 'k')) text.pop_back();
                const auto kelvin = parseDoubleValue(text);
                if (!kelvin || !std::isfinite(*kelvin) || *kelvin < 1500 || *kelvin > 12000)
                    return errorReply(command.id, "Invalid preset white balance.");
                presetSettings.whiteBalanceKelvin = uint32_t(std::llround(*kelvin));
            }
        }
        if (const auto value = command.string("cameraAutofocusMode"))
            presetSettings.autofocusMode = autofocusModeFromCommandValue(*value);
        if (const auto value = command.string("cameraAutofocusRange"))
            presetSettings.autofocusRange = autofocusRangeFromCommandValue(*value);
        if (const auto value = command.string("cameraAutofocusSpeed"))
            presetSettings.autofocusSpeed = autofocusSpeedFromCommandValue(*value);
        if (const auto x = command.number("cameraAutofocusWindowX")) {
            const auto y = command.number("cameraAutofocusWindowY");
            const auto w = command.number("cameraAutofocusWindowWidth");
            const auto h = command.number("cameraAutofocusWindowHeight");
            if (!y || !w || !h || !std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*w) || !std::isfinite(*h) ||
                *x < 0 || *y < 0 || *w <= 0 || *h <= 0 || *x+*w > 1 || *y+*h > 1)
                return errorReply(command.id, "Invalid preset autofocus window.");
            presetSettings.autofocusWindowX = *x; presetSettings.autofocusWindowY = *y;
            presetSettings.autofocusWindowWidth = *w; presetSettings.autofocusWindowHeight = *h;
        }
        if (const auto value = command.string("timecodeMode")) {
            const auto normalized = normalizeTimecodeModeValue(*value);
            presetSettings.timecodeMode = normalized == "Rec Run" ? TimecodeMode::RecRun : TimecodeMode::FreeRun;
            uiUpdates.emplace_back("timecodeMode", normalized);
        }

        if (!addBoolUpdate("zebraEnabled") ||
            !addDoubleUpdate("zebraThreshold") ||
            !addBoolUpdate("focusPeakingEnabled") ||
            !addDoubleUpdate("focusPeakingThreshold") ||
            !addBoolUpdate("grayscaleEnabled") ||
            !addBoolUpdate("smpteEnabled") ||
            !addBoolUpdate("anamorphicDesqueezeEnabled") ||
            !addBoolUpdate("falseColorEnabled") ||
            !addIntegerUpdate("falseColorMode") ||
            !addBoolUpdate("lutPreviewEnabled") ||
            !addBoolUpdate("horizontalLevelEnabled") ||
            !addBoolUpdate("guidesEnabled") ||
            !addIntegerUpdate("guidesThickness") ||
            !addBoolUpdate("centerMarkerEnabled") ||
            !addBoolUpdate("externalMonitorEnabled") ||
            !addBoolUpdate("photoModeEnabled") ||
            !addBoolUpdate("dateTimeOverlayEnabled") ||
            !addBoolUpdate("soundRecorderEnabled") ||
            !addBoolUpdate("recordAudioEnabled") ||
            !addBoolUpdate("recordGyroEnabled") ||
            !addBoolUpdate("liveAudioMonitoringEnabled") ||
            !addBoolUpdate("audioMeterEnabled") ||
            !addDoubleUpdate("cameraControlsOpacity") ||
            !addIntegerUpdate("inputVolume") ||
            !addIntegerUpdate("audioSampleRate") ||
            !addIntegerUpdate("headphoneVolume") ||
            !addIntegerUpdate("customBatteryWh") ||
            !addDoubleUpdate("customBatteryFullVoltage") ||
            !addDoubleUpdate("customBatteryEmptyVoltage")) {
            return errorReply(command.id, "One of the preset numeric values is invalid.");
        }

        addStringUpdate("focusPeakingColor");
        addStringUpdate("lutPreviewFile");
        addStringUpdate("horizontalLevelColor");
        addStringUpdate("anamorphicRatio");
        addStringUpdate("guidesType");
        addStringUpdate("centerMarkerType");
        if (const auto uiOrientation = command.string("uiOrientation"))
            uiUpdates.emplace_back("uiOrientation", normalizeUiOrientationValue(*uiOrientation));
        if (const auto cameraControlsMode = command.string("cameraControlsMode"))
            uiUpdates.emplace_back("cameraControlsMode", normalizeCameraControlsModeValue(*cameraControlsMode));
        addStringUpdate("externalMonitorMode");
        addStringUpdate("externalMonitorInfoOverlay");
        if (const auto externalMonitorOrientation = command.string("externalMonitorOrientation")) {
            uiUpdates.emplace_back("externalMonitorOrientation",
                                   normalizeExternalMonitorOrientationValue(*externalMonitorOrientation));
        }
        addStringUpdate("photoTimer");
        addStringUpdate("photoBurst");
        if (const auto photoFormat = command.string("photoFormat")) {
            uiUpdates.emplace_back("photoFormat", normalizePhotoFormatValue(*photoFormat));
        }
        if (const auto photoTimelapse = command.string("photoTimelapse")) {
            uiUpdates.emplace_back("photoTimelapse", normalizePhotoTimelapseValue(*photoTimelapse));
        }
        presetSettings.timelapseExposureLimitUs = presetSettings.photoModeEnabled
            ? timelapseExposureLimitUs(photoTimelapseSecondsFromValue(command.string("photoTimelapse")
                .value_or(readSharedUiSettings().photoTimelapse))) : 0;
        addStringUpdate("audioInputDevice");
        addStringUpdate("audioOutputDevice");
        if (const auto format = command.string("audioSampleFormat"); format && !validAudioFormat(*format))
            return errorReply(command.id, "Invalid preset audio sample format.");
        if (const auto rate = command.number("audioSampleRate"); rate && (!std::isfinite(*rate) || *rate < 0 || *rate > 192000))
            return errorReply(command.id, "Invalid preset audio sample rate.");
        if (const auto channels = command.string("audioChannels")) {
            unsigned first = 0, count = 0;
            if (!audioChannelRange(*channels, first, count)) return errorReply(command.id, "Invalid preset audio input channels.");
        }
        addStringUpdate("audioSampleFormat");
        addStringUpdate("audioChannels");
        if (const auto batteryType = command.string("batteryType"))
            uiUpdates.emplace_back("batteryType", normalizeBatteryTypeValue(*batteryType));
        if (const auto batteryCapacity = command.string("batteryCapacity")) {
            const auto batteryType = command.string("batteryType");
            const std::string normalizedType = batteryType
                ? normalizeBatteryTypeValue(*batteryType)
                : readSharedUiSettings().batteryType;
            uiUpdates.emplace_back("batteryCapacity",
                                   normalizedBatteryCapacityForType(normalizedType, *batteryCapacity));
        }
        addStringUpdate("sleepMode");
        for (const char *key : {"monitoringOrder", "recordOrder", "audioOrder", "powerOrder", "systemOrder",
                               "wifiOrder", "infoOrder", "sidebarOrder", "favoritesOrder"})
            addStringUpdate(key);
        addStringUpdate("standbyTimeoutSeconds");
        addBoolUpdate("standbyUsbPowerOff");

        if (!camera_.applyCapturePreset(presetSettings, [&] {
                return uiUpdates.empty() || updateSharedUiGeneralSettings(uiUpdates);
            }))
            return errorReply(command.id, camera_.lastError());

        return okReply(command.id,
                       "\"applied_preset\":\"" + escapeJson(trimAscii(presetLabel)) + "\""
                       + std::string(",\"state\":{") + settingsBody(camera_.settings()) + "}");
    }

    if (command.name == "apply_date_time") {
        const auto year = command.number("year");
        const auto month = command.number("month");
        const auto day = command.number("day");
        const auto hour = command.number("hour");
        const auto minute = command.number("minute");
        const auto timeZone = command.string("time_zone");
        if (!year || !month || !day || !hour || !minute || !timeZone || timeZone->empty())
            return errorReply(command.id, "apply_date_time requires year, month, day, hour, minute, and time_zone.");

        std::string errorMessage;
        if (!applyDateTimeSettings(static_cast<int>(*year),
                                   static_cast<int>(*month),
                                   static_cast<int>(*day),
                                   static_cast<int>(*hour),
                                   static_cast<int>(*minute),
                                   *timeZone,
                                   &errorMessage)) {
            return errorReply(command.id, errorMessage);
        }
        return okReply(command.id, "\"date_time_applied\":true");
    }

    if (command.name == "apply_time_zone") {
        const auto timeZone = command.string("time_zone");
        if (!timeZone || trimAscii(*timeZone).empty())
            return errorReply(command.id, "apply_time_zone requires time_zone.");

        bool networkTimeSynced = false;
        std::string errorMessage;
        if (!applyTimeZoneSettings(*timeZone, &networkTimeSynced, &errorMessage))
            return errorReply(command.id, errorMessage);

        return okReply(command.id,
                       "\"time_zone\":\"" + escapeJson(currentTimeZoneId()) + "\""
                       ",\"local_time_input\":\"" + escapeJson(currentLocalDateTimeInputValue()) + "\""
                       ",\"network_time_synced\":" + std::string(networkTimeSynced ? "true" : "false"));
    }

    if (command.name == "set_fan_mode") {
        const auto value = command.string("value");
        if (!value)
            return errorReply(command.id, "set_fan_mode requires value.");

        std::string errorMessage;
        if (!applyFanMode(*value, &errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, "\"fan_mode\":\"" + escapeJson(*value) + "\"");
    }

    if (command.name == "format_media") {
        const std::string filesystemType = command.string("filesystem").value_or("exfat");
        std::string errorMessage;
        if (!formatMediaAtPath(options_.mediaRoot, filesystemType, &errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id,
                       "\"media_formatted\":true,\"filesystem\":\""
                           + escapeJson(formatFilesystemDisplayName(filesystemType)) + "\"");
    }

    if (command.name == "eject_media") {
        std::string errorMessage;
        if (!ejectMediaAtPath(options_.mediaRoot, &errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, "\"media_ejected\":true");
    }

    if (command.name == "mount_media") {
        const auto devicePath = command.string("device_path");
        if (!devicePath || trimAscii(*devicePath).empty())
            return errorReply(command.id, "mount_media requires device_path.");

        std::string errorMessage;
        if (!mountMediaDeviceAtPath(*devicePath, options_.mediaRoot, &errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id,
                       "\"media_mounted\":true,\"device_path\":\"" + escapeJson(trimAscii(*devicePath)) + "\"");
    }

    if (command.name == "get_media_test_state") {
        std::lock_guard lock(mediaSpeedMutex_);
        return okReply(command.id, std::string("\"running\":") + (mediaSpeedTestRunning_ ? "true" : "false") +
            ",\"mbps\":" + jsonNumber(mediaSpeedLastWriteSpeedMBps_, 2) +
            ",\"bytes\":" + std::to_string(mediaSpeedTestRunning_ ? mediaSpeedCurrentBytes_ : mediaSpeedLastBytes_) +
            ",\"used_fio\":" + ((mediaSpeedTestRunning_ ? mediaSpeedCurrentUsedFio_ : mediaSpeedLastUsedFio_) ? "true" : "false") +
            ",\"error\":\"" + escapeJson(mediaSpeedError_) + "\"");
    }
    if (command.name == "run_media_speed_test") {
        std::string errorMessage;
        if (!startMediaSpeedTest(&errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, "\"media_speed_test_started\":true");
    }

    if (command.name == "record_start") {
        const SharedUiSettingsSnapshot uiSettings = readSharedUiSettings();
        const AudioDeviceSnapshot audioDevices = detectAudioDevices();
        AudioRecordingOptions audioOptions = audioRecordingOptionsFromUiSettings(uiSettings, audioDevices);
        auto boolField = [](const std::optional<std::string> &value, bool fallback) {
            if (!value)
                return fallback;
            return *value == "true" || *value == "1";
        };
        const bool soundOnly = boolField(command.string("sound_only"), uiSettings.soundRecorderEnabled);
        GyroRecordingOptions gyroOptions;
        gyroOptions.enabled = soundOnly ? false : uiSettings.recordGyroEnabled;
        if (const auto gyroEnabled = command.string("gyro_enabled")) {
            gyroOptions.enabled = soundOnly ? false : (*gyroEnabled == "true" || *gyroEnabled == "1");
        }
        if (const auto audioEnabled = command.string("audio_enabled")) {
            audioOptions.enabled = (*audioEnabled == "true" || *audioEnabled == "1");
        }
        if (const auto monitoringEnabled = command.string("audio_monitoring_enabled")) {
            audioOptions.liveMonitoringEnabled = (*monitoringEnabled == "true" || *monitoringEnabled == "1");
        }
        if (const auto inputVolume = command.number("audio_input_volume")) {
            const auto clamped = std::clamp(*inputVolume, 0.0, 100.0);
            audioOptions.inputVolume = static_cast<uint8_t>(std::lround(clamped));
        }
        if (const auto outputVolume = command.number("audio_output_volume")) {
            const auto clamped = std::clamp(*outputVolume, 0.0, 100.0);
            audioOptions.outputVolume = static_cast<uint8_t>(std::lround(clamped));
        }
        if (const auto inputDevice = command.string("audio_input_device"); inputDevice && !inputDevice->empty()) {
            const std::string resolvedSelection =
                resolveAudioDeviceSelection(*inputDevice, audioDevices.inputOptions, false);
            const std::string recorderDevice =
                audioDeviceIdFromSelectionValue(resolvedSelection.empty() ? *inputDevice : resolvedSelection);
            if (audioOptions.enabled && recorderDevice.empty())
                return errorReply(command.id, "Selected audio input device is unavailable.");
            audioOptions.inputDevice = recorderDevice;
        }
        if (const auto outputDevice = command.string("audio_output_device"); outputDevice && !outputDevice->empty()) {
            const std::string resolvedSelection =
                resolveAudioDeviceSelection(*outputDevice, audioDevices.outputOptions, true);
            const std::string recorderDevice =
                audioDeviceIdFromSelectionValue(resolvedSelection.empty() ? *outputDevice : resolvedSelection);
            if (!recorderDevice.empty())
                audioOptions.outputDevice = recorderDevice;
            else
                audioOptions.liveMonitoringEnabled = false;
        }

        if (soundOnly) {
            audioOptions.enabled = true;
            if (audioOptions.inputDevice.empty() && !audioDevices.inputOptions.empty())
                audioOptions.inputDevice = audioDeviceIdFromSelectionValue(audioDevices.inputOptions.front().id);
        }

        if (audioOptions.inputDevice.empty())
            audioOptions.enabled = false;
        if (!audioOptions.enabled) {
            audioOptions.liveMonitoringEnabled = false;
            audioOptions.outputDevice.clear();
        } else if (audioOptions.outputDevice.empty() || audioOptions.outputVolume == 0) {
            audioOptions.liveMonitoringEnabled = false;
            audioOptions.outputDevice.clear();
        }

        if (soundOnly) {
            if (!audioOptions.enabled)
                return errorReply(command.id, "Sound Recorder requires an audio input device.");

            controlServer_.broadcast("{\"event\":\"prepare_recording\"}");
            std::this_thread::sleep_for(std::chrono::milliseconds(220));

            std::string errorMessage;
            if (!startSoundRecording(audioOptions, &errorMessage))
                return errorReply(command.id, errorMessage);
            return okReply(command.id, "\"recording\":true,\"sound_recording\":true");
        }

        if (soundRecordingActive())
            return errorReply(command.id, "Stop Sound Recorder before starting camera recording.");

        if (audioOptions.enabled) {
            controlServer_.broadcast("{\"event\":\"prepare_recording\"}");
            std::this_thread::sleep_for(std::chrono::milliseconds(220));
        }

        if (!camera_.startRecording(audioOptions, gyroOptions))
            return errorReply(command.id, camera_.lastError());
        syncRecordingTracking(true);
        return okReply(command.id, std::string("\"recording\":true,\"recording_has_dropped_frames\":")
                       + (camera_.recordingHasDroppedFrames() ? "true" : "false"));
    }

    if (command.name == "capture_photo") {
        const SharedUiSettingsSnapshot uiSettings = readSharedUiSettings();
        const std::string selectedFormat = command.string("format").value_or(uiSettings.photoFormat);
        const auto requestedCount = command.number("count");
        if (command.fields.count("count") && (!requestedCount || !std::isfinite(*requestedCount) ||
            *requestedCount < 1 || *requestedCount > 10 || std::floor(*requestedCount) != *requestedCount))
            return errorReply(command.id, "Photo burst count must be a whole number between 1 and 10.");
        const auto captureSettings = camera_.settings();
        const unsigned count = usesStillBracketing(captureSettings)
            ? 3 : requestedCount ? static_cast<unsigned>(*requestedCount) : 1;
        if (!camera_.capturePhoto(photoCaptureFormatFromValue(selectedFormat), count))
            return errorReply(command.id, camera_.lastError());

        if (count == 1) {
            const std::string event = "{\"event\":\"photo_capture\"}";
            handleTriggerEvent(event);
            controlServer_.broadcast(event);
        }
        const auto mediaWrite = camera_.mediaWriteStatus();
        return okReply(command.id, "\"photo_capture\":true,\"photo_count\":" + std::to_string(count) +
                       ",\"media_writes_pending\":" + (mediaWrite.pending ? "true" : "false") +
                       ",\"media_write_revision\":" + std::to_string(mediaWrite.revision));
    }

    if (command.name == "timelapse_start") {
        const SharedUiSettingsSnapshot uiSettings = readSharedUiSettings();
        const std::string normalized = normalizePhotoTimelapseValue(
            command.string("value").value_or(uiSettings.photoTimelapse));
        const double intervalSeconds = photoTimelapseSecondsFromValue(normalized);
        if (intervalSeconds == 0)
            return errorReply(command.id, "Enable a timelapse interval first.");

        const std::string selectedFormat = normalizePhotoFormatValue(
            command.string("format").value_or(uiSettings.photoFormat));
        if (!photoFormatIsTimelapseSafeValue(selectedFormat))
            return errorReply(command.id, "Timelapse supports DNG, JPEG, PNG, or TIFF only.");

        if (!camera_.timelapseActive() &&
            !camera_.setStillExposureMode(true, uiSettings.photoLongExposureEnabled, intervalSeconds))
            return errorReply(command.id, camera_.lastError());
        if (!camera_.startTimelapse(photoCaptureFormatFromValue(selectedFormat), intervalSeconds))
            return errorReply(command.id, camera_.lastError());

        if (!updateSharedUiGeneralSettings({
                { "photoModeEnabled", "true" },
                { "photoTimelapse", normalized },
                { "photoFormat", selectedFormat },
                { "soundRecorderEnabled", "false" },
            })) {
            camera_.stopTimelapse();
            return errorReply(command.id, "Could not update shared UI settings.");
        }

        return okReply(command.id,
                       "\"timelapse_active\":true,\"photo_timelapse\":\"" + escapeJson(normalized) +
                       "\",\"photo_format\":\"" + escapeJson(selectedFormat) +
                       "\",\"interval_seconds\":" + std::to_string(intervalSeconds));
    }

    if (command.name == "timelapse_stop") {
        camera_.stopTimelapse();
        const auto mediaWrite = camera_.mediaWriteStatus();
        return okReply(command.id, std::string("\"timelapse_active\":false,\"media_writes_pending\":") +
                       (mediaWrite.pending ? "true" : "false") +
                       ",\"media_write_revision\":" + std::to_string(mediaWrite.revision));
    }

    if (command.name == "record_stop") {
        if (soundRecordingActive()) {
            stopSoundRecording();
            return okReply(command.id, "\"recording\":false,\"sound_recording\":false");
        }

        if (!camera_.stopRecording())
            return errorReply(command.id, camera_.lastError());
        syncRecordingTracking(false);
        return okReply(command.id, std::string("\"recording\":false,\"recording_has_dropped_frames\":")
                       + (camera_.recordingHasDroppedFrames() ? "true" : "false"));
    }

    if (command.name == "preview_subscribe") {
        enqueueCurrentPreviewRegistrations();
        return okReply(command.id, "\"preview_subscribed\":true");
    }

    if (command.name == "restart_system") {
        std::string errorMessage;
        if (!invokeSystemPowerAction("reboot", &errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, "\"restarting\":true");
    }

    if (command.name == "power_off_system") {
        std::string errorMessage;
        if (!invokeSystemPowerAction("poweroff", &errorMessage))
            return errorReply(command.id, errorMessage);
        return okReply(command.id, "\"powering_off\":true");
    }

    if (command.name == "shutdown") {
        requestShutdown();
        return okReply(command.id, "\"shutting_down\":true");
    }

    return errorReply(command.id, "Unknown command: " + command.name);
}

void ApertarCoreApp::publishEvent(const std::string &eventJson)
{
    handleTriggerEvent(eventJson);

    // Settings must also reach the separate HDMI process immediately, not just
    // the command's requester. These events cannot be collected into its reply.
    if (eventJson.find("\"event\":\"settings\"") != std::string::npos ||
        eventJson.find("\"event\":\"recording_health\"") != std::string::npos ||
        eventJson.find("\"event\":\"standby\"") != std::string::npos ||
        eventJson.find("\"event\":\"still_exposure\"") != std::string::npos ||
        eventJson.find("\"event\":\"media_write_state\"") != std::string::npos ||
        eventJson.find("\"event\":\"photo_write_error\"") != std::string::npos ||
        eventJson.find("\"event\":\"timelapse_progress\"") != std::string::npos ||
        eventJson.find("\"action\":\"standby_power_wake\"") != std::string::npos ||
        eventJson.find("\"action\":\"standby_power_sleep\"") != std::string::npos ||
        eventJson.find("\"action\":\"power_button_press\"") != std::string::npos) {
        controlServer_.broadcast(eventJson);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(eventMutex_);
        if (collectingCommandEvents_) {
            pendingCommandEvents_.push_back(eventJson);
            return;
        }
    }

    controlServer_.broadcast(eventJson);
}

void ApertarCoreApp::handleTriggerEvent(const std::string &eventJson)
{
    if (eventJson.find("\"event\":\"still_exposure\"") != std::string::npos) {
        recordingIndicatorOutput_.setStillExposureActive(
            eventJson.find("\"still_exposure_active\":true") != std::string::npos);
        return;
    }

    if (eventJson.find("\"event\":\"recording\"") != std::string::npos) {
        if (eventJson.find("\"active\":true") != std::string::npos) {
            recordingIndicatorOutput_.setRecordingActive(true);
            triggerOutput_.pulse(TriggerOutput::Event::RecordingStart);
            return;
        }
        if (eventJson.find("\"active\":false") != std::string::npos)
            recordingIndicatorOutput_.setRecordingActive(false);
        return;
    }

    if (eventJson.find("\"event\":\"photo_capture\"") != std::string::npos) {
        recordingIndicatorOutput_.pulseCapture();
        triggerOutput_.pulse(TriggerOutput::Event::Still);
        return;
    }

    if (eventJson.find("\"event\":\"timelapse_frame\"") != std::string::npos) {
        recordingIndicatorOutput_.pulseCapture();
        triggerOutput_.pulse(TriggerOutput::Event::TimelapseFrame);
        return;
    }

    if (eventJson.find("\"event\":\"timelapse\"") != std::string::npos &&
        eventJson.find("\"active\":true") != std::string::npos) {
        triggerOutput_.pulse(TriggerOutput::Event::TimelapseStart);
    }
}

void ApertarCoreApp::publishPreviewFrame(const PreviewFrameDescriptor &frame)
{
    std::vector<int> attachedFds;
    if (frame.registration) {
        const int layoutPlaneCount = std::max(0, std::min(frame.planeCount, 3));
        for (int i = 0; i < layoutPlaneCount; ++i) {
            if (frame.planeFds[i] >= 0)
                attachedFds.push_back(frame.planeFds[i]);
        }
        if (attachedFds.empty() && frame.fd >= 0)
            attachedFds.push_back(frame.fd);
        if (attachedFds.empty())
            return;
    }

    std::ostringstream out;
    out << "{\"event\":\"" << (frame.registration ? "preview_buffer" : "preview_frame") << "\""
        << ",\"slot\":" << frame.slotIndex
        << ",\"frame_id\":" << frame.frameId
        << ",\"timestamp_us\":" << frame.timestampUs;

    const int planeCount = std::max(0, std::min(frame.planeCount, 3));
    if (frame.registration) {
        out << ",\"width\":" << frame.width
            << ",\"height\":" << frame.height
            << ",\"stride\":" << frame.stride
            << ",\"capture_width\":" << frame.captureWidth
            << ",\"capture_height\":" << frame.captureHeight
            << ",\"size\":" << frame.size
            << ",\"fourcc\":\"" << escapeJson(frame.fourcc) << "\"";

        if (planeCount > 0) {
            const int attachedPlaneCount = std::max(1, static_cast<int>(attachedFds.size()));
            out << ",\"plane_count\":" << attachedPlaneCount
                << ",\"layout_plane_count\":" << planeCount;
            for (int i = 0; i < planeCount; ++i) {
                out << ",\"plane" << i << "_offset\":" << frame.planeOffsets[i]
                    << ",\"plane" << i << "_pitch\":" << frame.planePitches[i]
                    << ",\"plane" << i << "_length\":" << frame.planeLengths[i];
            }
        } else {
            out << ",\"plane_count\":1";
        }

        out << ",\"fd_attached\":true";
    }
    out << "}";

    if (frame.registration) {
        controlServer_.broadcastWithFds(out.str(), attachedFds);
    } else {
        controlServer_.broadcast(out.str());
    }
}

void ApertarCoreApp::runPreviewPublisher()
{
    std::unique_lock<std::mutex> lock(previewMutex_);
    while (true) {
        previewCv_.wait(lock, [this]() {
            return !previewPublisherRunning_ ||
                   !pendingPreviewRegistrations_.empty() ||
                   pendingPreviewFrame_.has_value();
        });

        if (!previewPublisherRunning_ &&
            pendingPreviewRegistrations_.empty() &&
            !pendingPreviewFrame_)
            break;

        PreviewFrameDescriptor frame;
        if (!pendingPreviewRegistrations_.empty()) {
            frame = std::move(pendingPreviewRegistrations_.front());
            pendingPreviewRegistrations_.pop_front();
        } else if (pendingPreviewFrame_) {
            frame = std::move(*pendingPreviewFrame_);
            pendingPreviewFrame_.reset();
        } else {
            continue;
        }
        lock.unlock();
        publishPreviewFrame(frame);
        closeOwnedPreviewFrame(&frame);
        lock.lock();
    }

    while (!pendingPreviewRegistrations_.empty()) {
        closeOwnedPreviewFrame(&pendingPreviewRegistrations_.front());
        pendingPreviewRegistrations_.pop_front();
    }
    if (pendingPreviewFrame_) {
        closeOwnedPreviewFrame(&*pendingPreviewFrame_);
        pendingPreviewFrame_.reset();
    }
    lastPreviewFrame_.reset();
}

void ApertarCoreApp::stopPreviewPublisher()
{
    {
        std::lock_guard<std::mutex> lock(previewMutex_);
        previewPublisherRunning_ = false;
    }
    previewCv_.notify_all();

    if (previewThread_.joinable())
        previewThread_.join();

    std::lock_guard<std::mutex> lock(previewMutex_);
    while (!pendingPreviewRegistrations_.empty()) {
        closeOwnedPreviewFrame(&pendingPreviewRegistrations_.front());
        pendingPreviewRegistrations_.pop_front();
    }
    if (pendingPreviewFrame_) {
        closeOwnedPreviewFrame(&*pendingPreviewFrame_);
        pendingPreviewFrame_.reset();
    }
    lastPreviewFrame_.reset();
    for (auto &[slotIndex, frame] : registeredPreviewFrames_) {
        (void)slotIndex;
        closeOwnedPreviewFrame(&frame);
    }
    registeredPreviewFrames_.clear();
}

void ApertarCoreApp::requestShutdown()
{
    {
        std::lock_guard<std::mutex> lock(waitMutex_);
        shutdownRequested_ = true;
    }
    waitCv_.notify_all();
}

} // namespace apertar
