#pragma once

#include <apertar/core/Types.hpp>
#include <string_view>

namespace apertar {

inline uint8_t exposureMeteringBit(std::string_view mode)
{
    return mode == "centre-weighted" ? 1 : mode == "spot" ? 2 : mode == "matrix" ? 4 : 0;
}

inline std::string normalizeExposureMetering(std::string_view value)
{
    if (value == "average") return "matrix";
    if (value == "center-weighted") return "centre-weighted";
    return exposureMeteringBit(value) ? std::string(value) : std::string();
}

inline void constrainExposureMetering(CaptureSettings &settings, uint8_t supported)
{
    settings.exposureMeteringModes = supported;
    if (supported & exposureMeteringBit(settings.exposureMetering)) return;
    settings.exposureMetering = supported & 1 ? "centre-weighted" : supported & 2 ? "spot"
        : supported & 4 ? "matrix" : "centre-weighted";
}

inline std::string exposureMeteringJson(const CaptureSettings &settings)
{
    std::string json = "\"exposure_metering\":\"" + settings.exposureMetering + "\",\"exposure_metering_modes\":[";
    bool first = true;
    for (const char *mode : {"centre-weighted", "matrix", "spot"}) {
        if (!(settings.exposureMeteringModes & exposureMeteringBit(mode))) continue;
        if (!first) json += ',';
        json += std::string("\"") + mode + '"';
        first = false;
    }
    return json + ']';
}

} // namespace apertar
