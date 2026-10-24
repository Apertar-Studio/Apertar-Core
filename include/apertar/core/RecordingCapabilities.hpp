#pragma once

#include <apertar/core/Types.hpp>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <string_view>

namespace apertar {

struct RecordingCapabilities {
    bool rawCompression = false;
    bool prores = false;
    bool hardwareH264 = false;
    bool rawRecording = true;
};

inline RecordingCapabilities recordingCapabilitiesFromCompatible(std::string_view compatible)
{
    bool legacy = false;
    bool usb3 = false;
    while (!compatible.empty()) {
        const auto end = compatible.find('\0');
        const auto soc = compatible.substr(0, end);
        if (soc == "brcm,bcm2712")
            return {true, true}; // Pi 5 / CM5-class hardware.
        if (soc == "brcm,bcm2711")
            usb3 = true; // Pi 4 / Pi 400 / CM4-class hardware.
        if (soc == "brcm,bcm2711" || soc == "brcm,bcm2837" || soc == "brcm,bcm2836" || soc == "brcm,bcm2835")
            legacy = true;
        if (end == std::string_view::npos) break;
        compatible.remove_prefix(end + 1);
    }
    return {false, false, legacy, !legacy || usb3};
}

inline const RecordingCapabilities &recordingCapabilities()
{
    static const RecordingCapabilities capabilities = [] {
        for (const char *path : {"/sys/firmware/devicetree/base/compatible", "/proc/device-tree/compatible"}) {
            std::ifstream input(path, std::ios::binary);
            if (input) {
                const std::string compatible((std::istreambuf_iterator<char>(input)), {});
                return recordingCapabilitiesFromCompatible(compatible);
            }
        }
        return RecordingCapabilities{};
    }();
    return capabilities;
}

inline std::string recordingCapabilitiesJson()
{
    const auto &caps = recordingCapabilities();
    return std::string("{\"raw_compression\":") + (caps.rawCompression ? "true" : "false") +
        ",\"raw_recording\":" + (caps.rawRecording ? "true" : "false") +
        ",\"prores\":" + (caps.prores ? "true" : "false") +
        ",\"h264_hardware\":" + (caps.hardwareH264 ? "true" : "false") +
        ",\"h264_max_width\":" + (caps.hardwareH264 ? "1920" : "2560") +
        ",\"h264_max_height\":" + (caps.hardwareH264 ? "1080" : "1440") +
        ",\"h264_max_fps\":" + (caps.hardwareH264 ? "30" : "0") + "}";
}

inline const char *unsupportedRecordingSettings(const CaptureSettings &settings,
                                               const RecordingCapabilities &caps = recordingCapabilities())
{
    if (!caps.rawRecording && settings.recordingFormat != RecordingFormat::H264)
        return "Only H.264 clip recording is available on USB 2.0-only Raspberry Pi boards.";
    if (!caps.prores && settings.recordingFormat == RecordingFormat::Prores)
        return "ProRes recording requires Raspberry Pi 5-class hardware.";
    if (!caps.rawCompression && (settings.recordingFormat == RecordingFormat::LossyDng ||
                                settings.cdngCompression != CdngCompression::Uncompressed))
        return caps.rawRecording
            ? "RAW compression requires Raspberry Pi 5-class hardware. Use uncompressed cDNG or MLV."
            : "RAW compression is not available on this board. Use H.264.";
    return nullptr;
}

inline CaptureSettings supportedRecordingSettings(CaptureSettings settings,
                                                 const RecordingCapabilities &caps = recordingCapabilities())
{
    if (!caps.rawRecording)
        settings.recordingFormat = RecordingFormat::H264;
    else if ((!caps.prores && settings.recordingFormat == RecordingFormat::Prores) ||
        (!caps.rawCompression && settings.recordingFormat == RecordingFormat::LossyDng))
        settings.recordingFormat = RecordingFormat::Cdng;
    if (!caps.rawCompression) settings.cdngCompression = CdngCompression::Uncompressed;
    if (caps.hardwareH264) {
        auto &output = settings.h264Resolution;
        if (output.width == 0 || output.height == 0)
            output = {1920, 1080};
        else if (output.width > 1920 || output.height > 1080) {
            // Preserve the selected crop when migrating a saved 1440p mode.
            constexpr uint32_t from[] = {1440,1384,1348,1280,1164,1152,1088,1072,1024,1004};
            constexpr uint32_t to[] = {1080,1038,1012,960,872,864,816,804,768,752};
            const auto height = output.height;
            const auto width = output.width;
            output = {1920,1080};
            for (size_t i = 0; i < std::size(from); ++i)
                if (width == 2560 && height == from[i]) output.height = to[i];
        }
        if (!settings.photoModeEnabled && settings.recordingFormat == RecordingFormat::H264)
            settings.fps = std::min(settings.fps, 30.0);
    }
    return settings;
}

} // namespace apertar
