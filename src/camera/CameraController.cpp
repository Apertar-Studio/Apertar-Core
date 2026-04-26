#include "camera/CameraController.hpp"

#include "cdng/CdngEncoder.hpp"
#include "ipc/ControlProtocol.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <cerrno>
#include <condition_variable>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

#if APERTARCORE_HAS_LIBCAMERA
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <sys/mman.h>
#include <unistd.h>
#endif

namespace apertar {
namespace {

std::string settingsEvent(const CaptureSettings &settings)
{
    std::ostringstream out;
    out << "{\"event\":\"settings\""
        << ",\"resolution\":\"" << settings.resolution.toString() << "\""
        << ",\"width\":" << settings.resolution.width
        << ",\"height\":" << settings.resolution.height
        << ",\"fps\":" << std::fixed << std::setprecision(3) << settings.fps
        << ",\"iso\":" << settings.iso
        << ",\"iso_auto\":" << (settings.isoAuto ? "true" : "false")
        << ",\"shutter_us\":" << settings.shutterUs
        << ",\"shutter_auto\":" << (settings.shutterAuto ? "true" : "false")
        << ",\"wb_kelvin\":" << settings.whiteBalanceKelvin
        << "}";
    return out.str();
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

CaptureSettings clampCaptureSettings(CaptureSettings settings)
{
    const bool supportedResolution =
        (settings.resolution.width == 1332 && settings.resolution.height == 990) ||
        (settings.resolution.width == 1928 && settings.resolution.height == 1090) ||
        (settings.resolution.width == 2028 && settings.resolution.height == 1080) ||
        (settings.resolution.width == 2028 && settings.resolution.height == 1520) ||
        (settings.resolution.width == 3856 && settings.resolution.height == 2180);
    if (!supportedResolution) {
        settings.resolution = CaptureSettings{}.resolution;
    }

    if (!(settings.fps > 0.0 && settings.fps <= 120.0))
        settings.fps = CaptureSettings{}.fps;

    if (settings.iso < 50 || settings.iso > 12800)
        settings.iso = CaptureSettings{}.iso;

    if (settings.shutterUs == 0 || settings.shutterUs > 1'000'000)
        settings.shutterUs = CaptureSettings{}.shutterUs;

    if (settings.whiteBalanceKelvin < 1500 || settings.whiteBalanceKelvin > 12000)
        settings.whiteBalanceKelvin = CaptureSettings{}.whiteBalanceKelvin;

    return settings;
}

CaptureSettings loadPersistedCaptureSettings()
{
    CaptureSettings settings = CaptureSettings{};
    const std::filesystem::path path = persistedSettingsPath();
    std::ifstream input(path);
    if (!input.is_open())
        return settings;

    std::string line;
    while (std::getline(input, line)) {
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos)
            continue;
        const std::string key = line.substr(0, eq);
        const std::string value = line.substr(eq + 1);
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
            else if (key == "wb_kelvin")
                settings.whiteBalanceKelvin = static_cast<uint32_t>(std::stoul(value));
        } catch (...) {
            // Ignore malformed values and keep defaults for resilience.
        }
    }

    return clampCaptureSettings(settings);
}

void persistCaptureSettings(const CaptureSettings &settings)
{
    const CaptureSettings sanitized = clampCaptureSettings(settings);
    const std::filesystem::path path = persistedSettingsPath();
    const std::filesystem::path dir = path.parent_path();
    if (!dir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
    }

    const std::filesystem::path tmp = path.string() + ".tmp";
    std::ofstream output(tmp, std::ios::trunc);
    if (!output.is_open())
        return;

    output << "resolution_width=" << sanitized.resolution.width << '\n';
    output << "resolution_height=" << sanitized.resolution.height << '\n';
    output << "fps=" << std::fixed << std::setprecision(6) << sanitized.fps << '\n';
    output << "iso=" << sanitized.iso << '\n';
    output << "iso_auto=" << (sanitized.isoAuto ? 1 : 0) << '\n';
    output << "shutter_us=" << sanitized.shutterUs << '\n';
    output << "shutter_auto=" << (sanitized.shutterAuto ? 1 : 0) << '\n';
    output << "wb_kelvin=" << sanitized.whiteBalanceKelvin << '\n';
    output.flush();
    output.close();

    std::error_code renameEc;
    std::filesystem::rename(tmp, path, renameEc);
    if (renameEc) {
        std::error_code copyEc;
        std::filesystem::copy_file(tmp, path, std::filesystem::copy_options::overwrite_existing, copyEc);
        std::error_code removeEc;
        std::filesystem::remove(tmp, removeEc);
    }
}

std::filesystem::path mediaRootPath(const std::string &mediaRoot)
{
    return mediaRoot.empty() ? std::filesystem::path("/media/RAW")
                             : std::filesystem::path(mediaRoot);
}

std::filesystem::path allocatePhotoPath(const std::string &mediaRoot)
{
    const std::filesystem::path directory = mediaRootPath(mediaRoot) / "Photos";
    std::error_code dirEc;
    std::filesystem::create_directories(directory, dirEc);
    if (dirEc)
        throw std::runtime_error("Could not create Photos folder: " + dirEc.message());

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

    for (int photo = 1; photo <= 9999; ++photo) {
        std::ostringstream name;
        name << prefix.str() << "_C" << std::setw(4) << std::setfill('0') << photo << ".dng";

        const std::filesystem::path candidate = directory / name.str();
        if (!std::filesystem::exists(candidate))
            return candidate;
    }

    throw std::runtime_error("Could not allocate a unique photo filename.");
}

#if APERTARCORE_HAS_LIBCAMERA

constexpr uint32_t kPreviewWidth = 1928;
constexpr uint32_t kPreviewHeight = 1090;
constexpr unsigned int kBufferCount = 4;

enum class SensorFamily {
    Unknown,
    Imx477,
    Imx585
};

struct SensorModeSelection
{
    Resolution rawResolution{};
    libcamera::PixelFormat rawPixelFormat = libcamera::formats::SRGGB12_CSI2P;
    uint32_t bitDepth = 12;
    std::string dngModel;
};

struct WhiteBalancePoint
{
    double kelvin = 0.0;
    double red = 1.0;
    double blue = 1.0;
};

struct WhiteBalanceGains
{
    float red = 1.0f;
    float blue = 1.0f;
};

WhiteBalanceGains gainsForWhiteBalance(SensorFamily sensorFamily, uint32_t kelvin)
{
    static constexpr std::array<WhiteBalancePoint, 6> kImx585CtCurve{{
        {2220.0, 1.1476, 0.2153},
        {5250.0, 0.6343, 0.4928},
        {8450.0, 0.5082, 0.7584},
        {8700.0, 0.5056, 0.7658},
        {9035.0, 0.4952, 0.7963},
        {9400.0, 0.4909, 0.8092},
    }};
    static constexpr std::array<WhiteBalancePoint, 6> kImx477CtCurve{{
        {2850.0, 0.4307, 0.3957},
        {2960.0, 0.4159, 0.4313},
        {3580.0, 0.3771, 0.5176},
        {4559.0, 0.3031, 0.6573},
        {5881.0, 0.2809, 0.6942},
        {7600.0, 0.2263, 0.7762},
    }};

    auto invert = [](double red, double blue) {
        return WhiteBalanceGains{
            static_cast<float>(1.0 / std::max(red, 0.0001)),
            static_cast<float>(1.0 / std::max(blue, 0.0001)),
        };
    };

    const WhiteBalancePoint *curve = sensorFamily == SensorFamily::Imx477
        ? kImx477CtCurve.data()
        : kImx585CtCurve.data();
    const size_t curveSize = sensorFamily == SensorFamily::Imx477
        ? kImx477CtCurve.size()
        : kImx585CtCurve.size();
    const double targetKelvin = static_cast<double>(kelvin);
    const double firstKelvin = curve[0].kelvin;
    const double lastKelvin = curve[curveSize - 1].kelvin;

    if (targetKelvin <= firstKelvin)
        return invert(curve[0].red, curve[0].blue);
    if (targetKelvin >= lastKelvin)
        return invert(curve[curveSize - 1].red, curve[curveSize - 1].blue);

    for (size_t i = 0; i + 1 < curveSize; ++i) {
        const WhiteBalancePoint &left = curve[i];
        const WhiteBalancePoint &right = curve[i + 1];
        if (targetKelvin < left.kelvin || targetKelvin > right.kelvin)
            continue;

        const double range = right.kelvin - left.kelvin;
        const double t = range == 0.0 ? 0.0 : ((targetKelvin - left.kelvin) / range);
        const double red = left.red + ((right.red - left.red) * t);
        const double blue = left.blue + ((right.blue - left.blue) * t);
        return invert(red, blue);
    }

    return invert(curve[curveSize - 1].red, curve[curveSize - 1].blue);
}

libcamera::ControlList controlsForSettings(const CaptureSettings &settings,
                                           SensorFamily sensorFamily,
                                           const libcamera::ControlInfoMap &controlInfo)
{
    libcamera::ControlList controls(controlInfo);

    const int64_t frameDurationUs = settings.fps > 0.0
        ? static_cast<int64_t>(1000000.0 / settings.fps)
        : 41666;
    const int64_t frameDurationRange[2] = { frameDurationUs, frameDurationUs };
    controls.set(libcamera::controls::FrameDurationLimits,
                 libcamera::Span<const int64_t, 2>(frameDurationRange));
    const bool autoExposure = settings.isoAuto || settings.shutterAuto;
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
                     static_cast<int32_t>(std::min<uint32_t>(settings.shutterUs, 0x7fffffff)));
    }
    if (!settings.isoAuto) {
        controls.set(libcamera::controls::AnalogueGain,
                     std::max(1.0f, static_cast<float>(settings.iso) / 100.0f));
    }

    const WhiteBalanceGains gains = gainsForWhiteBalance(sensorFamily, settings.whiteBalanceKelvin);
    controls.set(libcamera::controls::AwbEnable, false);
    controls.set(libcamera::controls::ColourTemperature,
                 static_cast<int32_t>(settings.whiteBalanceKelvin));
    controls.set(libcamera::controls::ColourGains,
                 libcamera::Span<const float, 2>({ gains.red, gains.blue }));

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

    if (lower.find("imx477") != std::string::npos)
        return SensorFamily::Imx477;
    if (lower.find("imx585") != std::string::npos)
        return SensorFamily::Imx585;
    return SensorFamily::Unknown;
}

SensorModeSelection sensorModeForRequestedResolution(SensorFamily sensorFamily, Resolution requested)
{
    SensorModeSelection selection;
    selection.rawResolution = requested;
    selection.dngModel = "Generic";

    switch (sensorFamily) {
    case SensorFamily::Imx477:
        if (requested.width == 1332 && requested.height == 990) {
            selection.rawResolution = Resolution{ 1332, 990 };
        } else if ((requested.width == 1928 && requested.height == 1090) ||
                   (requested.width == 2028 && requested.height == 1080)) {
            selection.rawResolution = Resolution{ 2028, 1080 };
        } else if (requested.width == 2028 && requested.height == 1520) {
            selection.rawResolution = Resolution{ 2028, 1520 };
        } else {
            selection.rawResolution = Resolution{ 2028, 1080 };
        }
        selection.dngModel = "IMX477";
        break;
    case SensorFamily::Imx585:
        if (requested.width == 1332 && requested.height == 990) {
            selection.rawResolution = Resolution{ 1928, 1090 };
        } else if ((requested.width == 2028 && requested.height == 1080) ||
                   (requested.width == 2028 && requested.height == 1520) ||
                   (requested.width == 1928 && requested.height == 1090)) {
            selection.rawResolution = Resolution{ 1928, 1090 };
        } else if ((requested.width == 4056 && requested.height == 2160) ||
                   (requested.width == 4056 && requested.height == 3040) ||
                   (requested.width == 3856 && requested.height == 2180)) {
            selection.rawResolution = Resolution{ 3856, 2180 };
        } else {
            selection.rawResolution = Resolution{ 1928, 1090 };
        }
        selection.dngModel = "IMX585-AAQJ1";
        break;
    case SensorFamily::Unknown:
        break;
    }

    return selection;
}

Resolution normalizedResolutionForSensor(SensorFamily sensorFamily, Resolution requested)
{
    switch (sensorFamily) {
    case SensorFamily::Imx477:
        if (requested.width == 1332 && requested.height == 990)
            return requested;
        if ((requested.width == 1928 && requested.height == 1090) ||
            (requested.width == 2028 && requested.height == 1080) ||
            (requested.width == 3856 && requested.height == 2180) ||
            (requested.width == 4056 && requested.height == 2160))
            return Resolution{ 2028, 1080 };
        if (requested.width == 2028 && requested.height == 1520)
            return requested;
        if (requested.width == 4056 && requested.height == 3040)
            return Resolution{ 2028, 1520 };
        return Resolution{ 2028, 1080 };
    case SensorFamily::Imx585:
        if ((requested.width == 1332 && requested.height == 990) ||
            (requested.width == 2028 && requested.height == 1080) ||
            (requested.width == 2028 && requested.height == 1520) ||
            (requested.width == 1928 && requested.height == 1090))
            return Resolution{ 1928, 1090 };
        if (requested.width == 3856 && requested.height == 2180)
            return Resolution{ 3856, 2180 };
        return Resolution{ 1928, 1090 };
    case SensorFamily::Unknown:
        return requested;
    }

    return requested;
}

double maximumSupportedFpsForResolution(SensorFamily sensorFamily, Resolution requested)
{
    const Resolution normalized = normalizedResolutionForSensor(sensorFamily, requested);

    switch (sensorFamily) {
    case SensorFamily::Imx477:
        if (normalized.width == 1332 && normalized.height == 990)
            return 100.0;
        if (normalized.width == 2028 && normalized.height == 1080)
            return 60.0;
        if (normalized.width == 2028 && normalized.height == 1520)
            return 30.0;
        break;
    case SensorFamily::Imx585:
        if (normalized.width == 3856 && normalized.height == 2180)
            return 30.0;
        if (normalized.width == 1928 && normalized.height == 1090)
            return 60.0;
        break;
    case SensorFamily::Unknown:
        break;
    }

    return 120.0;
}

CaptureSettings sanitizeSettingsForSensor(CaptureSettings settings, SensorFamily sensorFamily)
{
    settings = clampCaptureSettings(settings);
    if (sensorFamily == SensorFamily::Unknown)
        return settings;

    settings.resolution = normalizedResolutionForSensor(sensorFamily, settings.resolution);
    const double maxFps = maximumSupportedFpsForResolution(sensorFamily, settings.resolution);
    if (settings.fps > maxFps)
        settings.fps = maxFps;
    return settings;
}

libcamera::Size previewSizeForSensorMode(const Resolution &sensorMode)
{
    uint32_t width = std::min(kPreviewWidth, sensorMode.width);
    width &= ~1u;
    if (width < 2)
        width = std::min(2u, sensorMode.width);

    uint32_t height = sensorMode.height;
    if (sensorMode.width > 0) {
        const uint64_t scaledHeight = static_cast<uint64_t>(width) * sensorMode.height / sensorMode.width;
        height = static_cast<uint32_t>(std::max<uint64_t>(2, scaledHeight));
    }
    height = std::min(height, std::min(kPreviewHeight, sensorMode.height));
    height &= ~1u;
    if (height < 2)
        height = std::min(2u, sensorMode.height);

    return libcamera::Size(width, height);
}

std::array<float, 4> blackLevelsForSensor(SensorFamily sensorFamily)
{
    const float blackLevel = sensorFamily == SensorFamily::Imx477 ? 256.0f : 206.0f;
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

    const int64_t directDiff = std::llabs(candidateUs - expectedUs);
    const int64_t scaledDiff = std::llabs(candidateNsAsUs - expectedUs);
    return scaledDiff < directDiff ? candidateNsAsUs : candidateUs;
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

    static const std::array<FormatInfo, 20> kFormats{{
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

CdngEncoder::Metadata metadataFromRequest(const CaptureSettings &settings,
                                          const libcamera::ControlList &metadata,
                                          uint64_t frameIndex,
                                          int64_t sensorTimestampUs,
                                          uint64_t wallClockTimestampUs,
                                          SensorFamily sensorFamily,
                                          const std::string &dngModel)
{
    CdngEncoder::Metadata out;
    out.frameIndex = frameIndex;
    out.sensorTimestampUs = sensorTimestampUs;
    out.wallClockTimestampUs = wallClockTimestampUs;
    out.fps = settings.fps;
    out.iso = settings.iso;
    out.shutterUs = settings.shutterUs;
    out.whiteBalanceKelvin = settings.whiteBalanceKelvin;
    out.serial = readPiSerial();
    out.hasBlackLevels = true;
    const auto blackLevels = blackLevelsForSensor(sensorFamily);
    std::copy(blackLevels.begin(), blackLevels.end(), std::begin(out.blackLevels));
    if (!dngModel.empty())
        out.model = dngModel;

    if (auto gain = metadata.get(libcamera::controls::AnalogueGain))
        out.analogueGain = *gain;
    if (auto gain = metadata.get(libcamera::controls::DigitalGain))
        out.digitalGain = *gain;
    if (auto exposure = metadata.get(libcamera::controls::ExposureTime))
        out.shutterUs = static_cast<uint32_t>(*exposure);
    if (auto colourGains = metadata.get(libcamera::controls::ColourGains); colourGains && colourGains->size() >= 2) {
        out.colourGains[0] = (*colourGains)[0];
        out.colourGains[1] = (*colourGains)[1];
    }
    if (auto matrix = metadata.get(libcamera::controls::ColourCorrectionMatrix); matrix && matrix->size() >= 9) {
        out.hasColourMatrix = true;
        for (size_t i = 0; i < 9; ++i)
            out.colourMatrix[i] = (*matrix)[i];
    }

    return out;
}

class DngRecordingQueue {
public:
    struct Stats {
        uint64_t enqueued = 0;
        uint64_t written = 0;
        uint64_t droppedQueueFull = 0;
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
        stop();

        settings_ = settings;
        frameIndex_ = 0;
        droppedFrames_ = 0;
        enqueuedFrames_ = 0;
        writtenFrames_ = 0;
        writeFailures_ = 0;
        maxObservedQueueDepth_ = 0;
        stopping_ = false;

        try {
            folderPath_ = allocateClipFolder(mediaRoot);
            std::filesystem::create_directories(folderPath_);
        } catch (const std::exception &e) {
            if (error)
                *error = e.what();
            return false;
        }

        const unsigned int workerCount = std::max(2u, std::min(4u, std::thread::hardware_concurrency()));
        workers_.reserve(workerCount);
        for (unsigned int i = 0; i < workerCount; ++i)
            workers_.emplace_back(&DngRecordingQueue::workerLoop, this);

        std::cerr << "ApertarCore recording clip: " << folderPath_.string() << "\n";
        return true;
    }

    void stop()
    {
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

        const Stats summary = stats();
        if (summary.enqueued > 0 || summary.written > 0 || summary.droppedQueueFull > 0 || summary.writeFailures > 0) {
            std::cerr << "ApertarCore recording stats: "
                      << "enqueued=" << summary.enqueued
                      << " written=" << summary.written
                      << " dropped_queue_full=" << summary.droppedQueueFull
                      << " write_failures=" << summary.writeFailures
                      << " peak_queue=" << summary.peakQueueDepth << "/" << summary.maxQueueDepth
                      << "\n";
        }

        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        rawPool_.clear();
        stopping_ = false;
    }

    bool active() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return !workers_.empty() && !stopping_;
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
            out.currentQueueDepth = queue_.size();
            out.peakQueueDepth = maxObservedQueueDepth_;
        }
        return out;
    }

    uint64_t nextFrameIndex()
    {
        return frameIndex_.fetch_add(1, std::memory_order_relaxed);
    }

    std::vector<uint8_t> acquireRawBuffer(size_t size)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto reusable = std::find_if(rawPool_.begin(), rawPool_.end(), [size](const std::vector<uint8_t> &buffer) {
            return buffer.capacity() >= size;
        });
        if (reusable == rawPool_.end())
            return std::vector<uint8_t>(size);

        std::vector<uint8_t> buffer = std::move(*reusable);
        rawPool_.erase(reusable);
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
            if (droppedFrames_ == 1 || (droppedFrames_ % 30) == 0) {
                std::cerr << "ApertarCore drop[queue-full]: frame="
                          << metadata.frameIndex
                          << " dropped_total=" << droppedFrames_
                          << " queue=" << queue_.size() << "/" << maxQueuedFrames_
                          << "\n";
            }
            return false;
        }

        const std::filesystem::path path = folderPath_ / frameFilename(metadata.frameIndex);
        queue_.push_back(FrameJob{
            std::move(rawBytes),
            frame,
            std::move(metadata),
            path.string(),
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

    static std::filesystem::path allocateClipFolder(const std::string &mediaRoot)
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

        std::ostringstream prefix;
        prefix << "cDNG_" << std::put_time(&localTime, "%y-%m-%d");

        for (int clip = 1; clip <= 9999; ++clip) {
            std::ostringstream name;
            name << prefix.str() << "_C" << std::setw(4) << std::setfill('0') << clip;
            std::filesystem::path candidate = root / name.str();
            if (!std::filesystem::exists(candidate))
                return candidate;
        }

        throw std::runtime_error("Could not allocate a unique clip folder.");
    }

    static std::string frameFilename(uint64_t frameIndex)
    {
        std::ostringstream out;
        out << "frame_" << std::setw(9) << std::setfill('0') << frameIndex << ".dng";
        return out.str();
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

            job.frame.data = job.rawBytes.data();
            try {
                encoder.writeDngFile(job.path, job.frame, job.metadata);
                ++writtenFrames_;
            } catch (const std::exception &e) {
                ++writeFailures_;
                std::cerr << "ApertarCore DNG write failed for "
                          << job.path << ": " << e.what() << "\n";
            }

            recycleRawBuffer(std::move(job.rawBytes));
        }
    }

    void recycleRawBuffer(std::vector<uint8_t> buffer)
    {
        buffer.clear();

        std::lock_guard<std::mutex> lock(mutex_);
        if (rawPool_.size() < maxQueuedFrames_ * 2)
            rawPool_.push_back(std::move(buffer));
    }

    CaptureSettings settings_{};
    std::filesystem::path folderPath_;
    std::atomic<uint64_t> frameIndex_{0};
    uint64_t droppedFrames_ = 0;
    std::atomic<uint64_t> enqueuedFrames_{0};
    std::atomic<uint64_t> writtenFrames_{0};
    std::atomic<uint64_t> writeFailures_{0};
    uint64_t maxObservedQueueDepth_ = 0;
    const size_t maxQueuedFrames_ = 12;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<FrameJob> queue_;
    std::vector<std::vector<uint8_t>> rawPool_;
    std::vector<std::thread> workers_;
    bool stopping_ = false;
};

class PhotoCaptureWriter {
public:
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
                 std::string path)
    {
        std::lock_guard<std::mutex> lock(mutex_);
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

        queue_.push_back(FrameJob{
            std::move(rawBytes),
            frame,
            std::move(metadata),
            std::move(path),
        });
        cv_.notify_one();
        return true;
    }

private:
    struct FrameJob {
        std::vector<uint8_t> rawBytes;
        RawFrameView frame;
        CdngEncoder::Metadata metadata;
        std::string path;
    };

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

            job.frame.data = job.rawBytes.data();
            try {
                encoder.writeDngFile(job.path, job.frame, job.metadata);
            } catch (const std::exception &e) {
                std::cerr << "ApertarCore photo write failed for "
                          << job.path << ": " << e.what() << "\n";
            }
        }
    }

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<FrameJob> queue_;
    std::thread worker_;
    uint64_t droppedFrames_ = 0;
    bool stopping_ = false;
    const size_t maxQueuedFrames_ = 4;
};

#endif

} // namespace

class CameraController::CameraSession {
public:
    explicit CameraSession(PreviewCallback previewCallback)
        : previewCallback_(std::move(previewCallback))
    {
    }

    bool start(const CaptureSettings &settings, std::string *error)
    {
#if APERTARCORE_HAS_LIBCAMERA
        settings_ = settings;

        cameraManager_ = std::make_unique<libcamera::CameraManager>();
        if (const int ret = cameraManager_->start(); ret != 0) {
            setError(error, "libcamera CameraManager failed to start, code " + std::to_string(ret));
            cameraManager_.reset();
            return false;
        }

        const std::vector<std::shared_ptr<libcamera::Camera>> cameras = cameraManager_->cameras();
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

        sensorFamily_ = detectSensorFamily(camera_->id());
        settings_ = sanitizeSettingsForSensor(settings, sensorFamily_);

        if (camera_->acquire() != 0) {
            setError(error, "Failed to acquire camera " + cameraIdForLog(camera_) + ".");
            stop();
            return false;
        }
        acquired_ = true;

        if (!configureAndStart(settings, error)) {
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

    void stop()
    {
#if APERTARCORE_HAS_LIBCAMERA
        running_ = false;

        if (camera_) {
            camera_->requestCompleted.disconnect(this, &CameraSession::requestComplete);
            if (started_)
                camera_->stop();
        }

        recorder_.stop();
        photoWriter_.stop();
        {
            std::lock_guard<std::mutex> photoLock(photoCaptureMutex_);
            pendingPhotoCaptures_.clear();
        }
        unmapRawBuffers();
        started_ = false;
        requests_.clear();
        allocator_.reset();
        previewStream_ = nullptr;
        rawStream_ = nullptr;
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

    bool reconfigure(const CaptureSettings &settings, std::string *error)
    {
#if APERTARCORE_HAS_LIBCAMERA
        if (!camera_) {
            setError(error, "Camera is not open.");
            return false;
        }

        running_ = false;
        camera_->requestCompleted.disconnect(this, &CameraSession::requestComplete);
        if (started_)
            camera_->stop();
        started_ = false;

        recorder_.stop();
        unmapRawBuffers();
        requests_.clear();
        allocator_.reset();
        previewStream_ = nullptr;
        rawStream_ = nullptr;
        configuration_.reset();

        settings_ = sanitizeSettingsForSensor(settings, sensorFamily_);
        return configureAndStart(settings, error);
#else
        (void)settings;
        (void)error;
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

        const CaptureSettings sanitized = sanitizeSettingsForSensor(settings, sensorFamily_);

        {
            std::lock_guard<std::mutex> lock(controlsMutex_);
            pendingControls_ = controlsForSettings(sanitized, sensorFamily_, camera_->controls());
        }

        settings_ = sanitized;
        return true;
#else
        (void)settings;
        (void)error;
        return true;
#endif
    }

    bool startRecording(const std::string &mediaRoot, const CaptureSettings &settings, std::string *error)
    {
#if APERTARCORE_HAS_LIBCAMERA
        haveWallClockBase_ = false;
        recordingGapEstimate_ = 0;
        haveLastRecordingSensorTimestamp_ = false;
        lastRecordingSensorTimestampUs_ = 0;
        requeueFailures_ = 0;
        return recorder_.start(mediaRoot, settings, error);
#else
        (void)mediaRoot;
        (void)settings;
        (void)error;
        return true;
#endif
    }

    void stopRecording()
    {
#if APERTARCORE_HAS_LIBCAMERA
        if (recordingGapEstimate_ > 0 || requeueFailures_ > 0) {
            std::cerr << "ApertarCore capture stats: "
                      << "dropped_gap_estimate=" << recordingGapEstimate_
                      << " requeue_failures=" << requeueFailures_
                      << "\n";
        }
        recorder_.stop();
        haveLastRecordingSensorTimestamp_ = false;
        lastRecordingSensorTimestampUs_ = 0;
#endif
    }

    bool capturePhoto(const std::string &mediaRoot, const CaptureSettings &settings, std::string *error)
    {
#if APERTARCORE_HAS_LIBCAMERA
        try {
            PendingPhotoCapture pending;
            pending.path = allocatePhotoPath(mediaRoot).string();
            pending.settings = settings;

            std::lock_guard<std::mutex> lock(photoCaptureMutex_);
            if (pendingPhotoCaptures_.size() >= maxPendingPhotoCaptures_) {
                setError(error, "Too many pending photo captures.");
                return false;
            }

            pendingPhotoCaptures_.push_back(std::move(pending));
            return true;
        } catch (const std::exception &e) {
            setError(error, e.what());
            return false;
        }
#else
        (void)mediaRoot;
        (void)settings;
        (void)error;
        return true;
#endif
    }

    CaptureSettings currentSettings() const
    {
        return settings_;
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
    bool configureAndStart(const CaptureSettings &settings, std::string *error)
    {
        const CaptureSettings sanitized = sanitizeSettingsForSensor(settings, sensorFamily_);
        settings_ = sanitized;
        configuration_ = camera_->generateConfiguration({
            // Match the old Apertar capture model more closely: one main YUV
            // monitoring stream plus one raw stream from the selected sensor mode.
            libcamera::StreamRole::VideoRecording,
            libcamera::StreamRole::Raw
        });
        if (!configuration_ || configuration_->size() < 2) {
            setError(error, "Failed to generate preview + raw camera configuration.");
            return false;
        }

        libcamera::StreamConfiguration &previewConfig = configuration_->at(0);
        libcamera::StreamConfiguration &rawConfig = configuration_->at(1);

        const SensorModeSelection sensorMode = sensorModeForRequestedResolution(sensorFamily_, sanitized.resolution);
        configuredSensorWidth_ = sensorMode.rawResolution.width;
        configuredSensorHeight_ = sensorMode.rawResolution.height;
        dngModel_ = sensorMode.dngModel;

        const libcamera::Size previewSize = previewSizeForSensorMode(sensorMode.rawResolution);

        previewConfig.pixelFormat = libcamera::formats::YUV420;
        previewConfig.size = previewSize;
        previewConfig.size.alignDownTo(2, 2);
        previewConfig.bufferCount = kBufferCount;
        previewConfig.colorSpace = libcamera::ColorSpace::Rec709;

        rawConfig.pixelFormat = sensorMode.rawPixelFormat;
        rawConfig.size = libcamera::Size(sensorMode.rawResolution.width, sensorMode.rawResolution.height);
        rawConfig.bufferCount = kBufferCount;

        configuration_->sensorConfig = libcamera::SensorConfiguration();
        configuration_->sensorConfig->outputSize = rawConfig.size;
        configuration_->sensorConfig->bitDepth = sensorMode.bitDepth;

        const libcamera::CameraConfiguration::Status validation = configuration_->validate();
        if (validation == libcamera::CameraConfiguration::Invalid) {
            setError(error, "libcamera rejected the preview + raw configuration.");
            return false;
        }

        if (camera_->configure(configuration_.get()) != 0) {
            setError(error, "Failed to configure libcamera streams.");
            return false;
        }

        previewStream_ = previewConfig.stream();
        rawStream_ = rawConfig.stream();
        if (!previewStream_ || !rawStream_) {
            setError(error, "Configured camera streams are missing.");
            return false;
        }

        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        if (allocator_->allocate(previewStream_) < 0 || allocator_->allocate(rawStream_) < 0) {
            setError(error, "Failed to allocate libcamera frame buffers.");
            return false;
        }

        if (!mapRawBuffers(error))
            return false;

        if (!makeRequests(error))
            return false;

        libcamera::ControlList initialControls = controlsForSettings(sanitized, sensorFamily_, camera_->controls());
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

    bool makeRequests(std::string *error)
    {
        const auto &previewBuffers = allocator_->buffers(previewStream_);
        const auto &rawBuffers = allocator_->buffers(rawStream_);
        const size_t requestCount = std::min(previewBuffers.size(), rawBuffers.size());

        if (requestCount == 0) {
            setError(error, "No frame buffers were allocated.");
            return false;
        }

        requests_.clear();
        requests_.reserve(requestCount);

        for (size_t i = 0; i < requestCount; ++i) {
            std::unique_ptr<libcamera::Request> request = camera_->createRequest(i);
            if (!request) {
                setError(error, "Failed to create a libcamera request.");
                return false;
            }

            if (request->addBuffer(previewStream_, previewBuffers[i].get()) < 0 ||
                request->addBuffer(rawStream_, rawBuffers[i].get()) < 0) {
                setError(error, "Failed to attach buffers to a libcamera request.");
                return false;
            }

            requests_.push_back(std::move(request));
        }

        return true;
    }

    void requestComplete(libcamera::Request *request)
    {
        if (!request || request->status() == libcamera::Request::RequestCancelled)
            return;

        if (!running_.load())
            return;

        publishPreviewFrame(request);
        recordRawFrame(request);
        capturePhotoFrame(request);

        request->reuse(libcamera::Request::ReuseBuffers);
        {
            std::lock_guard<std::mutex> lock(controlsMutex_);
            if (!pendingControls_.empty()) {
                request->controls().merge(pendingControls_);
                pendingControls_.clear();
            }
        }

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

    struct MappedPlane {
        void *mapping = nullptr;
        size_t mappingLength = 0;
        uint8_t *data = nullptr;
        size_t length = 0;
    };

    struct PendingPhotoCapture {
        std::string path;
        CaptureSettings settings{};
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
        const auto &rawBuffers = allocator_->buffers(rawStream_);
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
        const uint64_t nowUs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

        if (sensorTimestampUs <= 0)
            return nowUs;

        if (!haveWallClockBase_) {
            haveWallClockBase_ = true;
            wallClockBaseSensorUs_ = sensorTimestampUs;
            wallClockBaseEpochUs_ = nowUs;
        }

        return wallClockBaseEpochUs_ + static_cast<uint64_t>(sensorTimestampUs - wallClockBaseSensorUs_);
    }

    void recordRawFrame(libcamera::Request *request)
    {
        if (!recorder_.active() || !rawStream_)
            return;

        libcamera::FrameBuffer *buffer = request->findBuffer(rawStream_);
        if (!buffer)
            return;

        auto mappedIt = mappedRawBuffers_.find(buffer);
        if (mappedIt == mappedRawBuffers_.end())
            return;

        const libcamera::StreamConfiguration &config = rawStream_->configuration();
        RawBitDepth bitDepth = RawBitDepth::Bits16;
        PixelPacking packing = PixelPacking::Unpacked;
        BayerPattern pattern = BayerPattern::RGGB;
        if (!formatToRawView(config.pixelFormat, &bitDepth, &packing, &pattern)) {
            if (!warnedUnsupportedRaw_) {
                warnedUnsupportedRaw_ = true;
                std::cerr << "ApertarCore recording skipped: unsupported raw format "
                          << config.pixelFormat.toString()
                          << ". Expected an uncompressed Bayer format.\n";
            }
            return;
        }

        const MappedPlane &mapped = mappedIt->second;
        const size_t bytesUsed = !buffer->metadata().planes().empty()
            ? buffer->metadata().planes().front().bytesused
            : 0;
        const size_t rawSize = std::min(mapped.length, bytesUsed > 0 ? bytesUsed : static_cast<size_t>(config.frameSize));
        if (rawSize == 0 || rawSize > mapped.length)
            return;

        std::vector<uint8_t> rawCopy = recorder_.acquireRawBuffer(rawSize);
        std::memcpy(rawCopy.data(), mapped.data, rawSize);

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

        const uint64_t frameIndex = recorder_.nextFrameIndex();
        RawFrameView view;
        view.data = rawCopy.data();
        view.size = rawCopy.size();
        view.width = config.size.width;
        view.height = config.size.height;
        view.stride = config.stride;
        view.bitDepth = bitDepth;
        view.packing = packing;
        view.bayerPattern = pattern;
        view.timestampUs = timestampUs;

        CdngEncoder::Metadata metadata = metadataFromRequest(
            settings_,
            request->metadata(),
            frameIndex,
            timestampUs,
            wallClockForSensorTimestamp(timestampUs),
            sensorFamily_,
            dngModel_);

        recorder_.enqueue(std::move(rawCopy), view, std::move(metadata));
    }

    void capturePhotoFrame(libcamera::Request *request)
    {
        if (!rawStream_)
            return;

        PendingPhotoCapture pending;
        {
            std::lock_guard<std::mutex> lock(photoCaptureMutex_);
            if (pendingPhotoCaptures_.empty())
                return;
            pending = std::move(pendingPhotoCaptures_.front());
            pendingPhotoCaptures_.pop_front();
        }

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

        const MappedPlane &mapped = mappedIt->second;
        const size_t bytesUsed = !buffer->metadata().planes().empty()
            ? buffer->metadata().planes().front().bytesused
            : 0;
        const size_t rawSize = std::min(mapped.length, bytesUsed > 0 ? bytesUsed : static_cast<size_t>(config.frameSize));
        if (rawSize == 0 || rawSize > mapped.length)
            return;

        std::vector<uint8_t> rawCopy(rawSize);
        std::memcpy(rawCopy.data(), mapped.data, rawSize);

        int64_t timestampUs = 0;
        if (auto ts = request->metadata().get(libcamera::controls::SensorTimestamp))
            timestampUs = static_cast<int64_t>(*ts / 1000);
        else
            timestampUs = static_cast<int64_t>(buffer->metadata().timestamp / 1000);

        const uint64_t frameIndex = photoFrameCounter_.fetch_add(1, std::memory_order_relaxed);
        RawFrameView view;
        view.data = rawCopy.data();
        view.size = rawCopy.size();
        view.width = config.size.width;
        view.height = config.size.height;
        view.stride = config.stride;
        view.bitDepth = bitDepth;
        view.packing = packing;
        view.bayerPattern = pattern;
        view.timestampUs = timestampUs;

        CdngEncoder::Metadata metadata = metadataFromRequest(
            pending.settings,
            request->metadata(),
            frameIndex,
            timestampUs,
            wallClockForSensorTimestamp(timestampUs),
            sensorFamily_,
            dngModel_);

        if (!photoWriter_.enqueue(std::move(rawCopy), view, std::move(metadata), pending.path)) {
            std::cerr << "ApertarCore photo skipped: could not queue write for "
                      << pending.path << "\n";
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
        const libcamera::FrameBuffer::Plane &plane = buffer->planes().front();

        int64_t timestampUs = 0;
        if (auto ts = request->metadata().get(libcamera::controls::SensorTimestamp))
            timestampUs = static_cast<int64_t>(*ts / 1000);
        else
            timestampUs = static_cast<int64_t>(buffer->metadata().timestamp / 1000);

        PreviewFrameDescriptor frame;
        frame.fd = plane.fd.get();
        frame.frameId = frameCounter_.fetch_add(1, std::memory_order_relaxed) + 1;
        frame.width = config.size.width;
        frame.height = config.size.height;
        frame.stride = config.stride;
        frame.captureWidth = settings_.resolution.width;
        frame.captureHeight = settings_.resolution.height;
        frame.size = plane.length;
        frame.fourcc = "YUV420";
        frame.timestampUs = timestampUs;

        const auto &planes = buffer->planes();
        const size_t sourcePlaneCount = std::min<size_t>(planes.size(), 3);
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

        previewCallback_(frame);
    }

    std::unique_ptr<libcamera::CameraManager> cameraManager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> configuration_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream *previewStream_ = nullptr;
    libcamera::Stream *rawStream_ = nullptr;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::unordered_map<libcamera::FrameBuffer *, MappedPlane> mappedRawBuffers_;
    libcamera::ControlList pendingControls_;
    std::mutex controlsMutex_;
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> frameCounter_{0};
    CaptureSettings settings_{};
    uint32_t configuredSensorWidth_ = 0;
    uint32_t configuredSensorHeight_ = 0;
    DngRecordingQueue recorder_;
    PhotoCaptureWriter photoWriter_;
    std::mutex photoCaptureMutex_;
    std::deque<PendingPhotoCapture> pendingPhotoCaptures_;
    std::atomic<uint64_t> photoFrameCounter_{0};
    SensorFamily sensorFamily_ = SensorFamily::Unknown;
    std::string dngModel_ = "Generic";
    bool warnedUnsupportedRaw_ = false;
    bool haveWallClockBase_ = false;
    int64_t wallClockBaseSensorUs_ = 0;
    uint64_t wallClockBaseEpochUs_ = 0;
    uint64_t recordingGapEstimate_ = 0;
    bool haveLastRecordingSensorTimestamp_ = false;
    int64_t lastRecordingSensorTimestampUs_ = 0;
    uint64_t requeueFailures_ = 0;
    const size_t maxPendingPhotoCaptures_ = 4;
    bool acquired_ = false;
    bool started_ = false;
#endif

    PreviewCallback previewCallback_;
};

CameraController::CameraController(EventCallback eventCallback, PreviewCallback previewCallback)
    : eventCallback_(std::move(eventCallback))
    , previewCallback_(std::move(previewCallback))
    , session_(std::make_unique<CameraSession>(previewCallback_))
{
    settings_ = loadPersistedCaptureSettings();
}

CameraController::~CameraController()
{
    stop();
}

bool CameraController::start(const std::string &mediaRoot, bool simulateCamera)
{
    std::lock_guard<std::mutex> lock(mutex_);
    mediaRoot_ = mediaRoot;
    simulateCamera_ = simulateCamera;
    state_ = CameraState::Starting;

    if (!simulateCamera_ && !session_->start(settings_, &lastError_)) {
        state_ = CameraState::Error;
        return false;
    }

    if (!simulateCamera_ && session_)
        settings_ = session_->currentSettings();

    state_ = CameraState::Ready;
    lastError_.clear();
    persistCaptureSettings(settings_);
    emitSettingsEventLocked();
    return true;
}

void CameraController::stop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == CameraState::Offline)
        return;

    recording_ = false;
    state_ = CameraState::Stopping;

    if (session_)
        session_->stop();

    state_ = CameraState::Offline;
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

    CaptureSettings next = settings_;
    next.resolution = resolution;
    return applySettingsLocked(next, true);
}

bool CameraController::setFps(double fps)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fps <= 0.0 || fps > 120.0) {
        setErrorLocked("FPS must be between 0 and 120.");
        return false;
    }

    CaptureSettings next = settings_;
    next.fps = fps;
    return applySettingsLocked(next, false);
}

bool CameraController::setIso(uint32_t iso)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (iso < 50 || iso > 12800) {
        setErrorLocked("ISO must be between 50 and 12800.");
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
    if (shutterUs == 0) {
        setErrorLocked("Shutter time must be greater than zero.");
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

bool CameraController::setWhiteBalanceKelvin(uint32_t kelvin)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (kelvin < 1500 || kelvin > 12000) {
        setErrorLocked("White balance must be between 1500K and 12000K.");
        return false;
    }

    CaptureSettings next = settings_;
    next.whiteBalanceKelvin = kelvin;
    return applySettingsLocked(next, false);
}

bool CameraController::startRecording()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CameraState::Ready) {
        setErrorLocked("Camera is not ready.");
        return false;
    }

    if (!simulateCamera_ && session_ && !session_->startRecording(mediaRoot_, settings_, &lastError_)) {
        state_ = CameraState::Error;
        return false;
    }

    recording_ = true;
    state_ = CameraState::Recording;
    lastError_.clear();
    emitRecordingEventLocked();
    return true;
}

bool CameraController::capturePhoto()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (recording_) {
        setErrorLocked("Photo capture is unavailable while recording.");
        return false;
    }

    if (state_ != CameraState::Ready) {
        setErrorLocked("Camera is not ready.");
        return false;
    }

    if (!simulateCamera_ && session_ && !session_->capturePhoto(mediaRoot_, settings_, &lastError_))
        return false;

    lastError_.clear();
    return true;
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
    lastError_.clear();
    emitRecordingEventLocked();
    return true;
}

CameraState CameraController::state() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

CaptureSettings CameraController::settings() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return settings_;
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

bool CameraController::applySettingsLocked(const CaptureSettings &nextSettings, bool requiresReconfigure)
{
    if (!simulateCamera_ && session_) {
        const bool applied = requiresReconfigure
            ? session_->reconfigure(nextSettings, &lastError_)
            : session_->applyControls(nextSettings, &lastError_);
        if (!applied) {
            state_ = CameraState::Error;
            return false;
        }
    }

    settings_ = simulateCamera_ ? clampCaptureSettings(nextSettings) : session_->currentSettings();
    persistCaptureSettings(settings_);
    lastError_.clear();
    emitSettingsEventLocked();
    return true;
}

bool CameraController::validateResolution(Resolution resolution)
{
    if ((resolution.width == 1332 && resolution.height == 990) ||
        (resolution.width == 1928 && resolution.height == 1090) ||
        (resolution.width == 2028 && resolution.height == 1080) ||
        (resolution.width == 2028 && resolution.height == 1520) ||
        (resolution.width == 3856 && resolution.height == 2180)) {
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
    if (eventCallback_)
        eventCallback_(settingsEvent(settings_));
}

void CameraController::emitRecordingEventLocked()
{
    if (!eventCallback_)
        return;

    std::ostringstream out;
    out << "{\"event\":\"recording\",\"active\":" << (recording_ ? "true" : "false") << "}";
    eventCallback_(out.str());
}

} // namespace apertar
