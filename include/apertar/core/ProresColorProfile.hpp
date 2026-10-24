#pragma once

#include <array>
#include <cmath>
#include <string_view>

namespace apertar {
enum class ProresColorProfile { Standard, ArriLogC4, SonySLog3, DavinciIntermediate, P3D60Linear };

inline const char *toString(ProresColorProfile profile)
{
    switch (profile) {
    case ProresColorProfile::ArriLogC4: return "arri_logc4";
    case ProresColorProfile::SonySLog3: return "sony_slog3";
    case ProresColorProfile::DavinciIntermediate: return "davinci_intermediate";
    case ProresColorProfile::P3D60Linear: return "p3_d60_linear";
    default: return "standard";
    }
}
inline bool parseProresColorProfile(std::string_view value, ProresColorProfile &profile)
{
    for (auto p : {ProresColorProfile::Standard, ProresColorProfile::ArriLogC4,
                   ProresColorProfile::SonySLog3, ProresColorProfile::DavinciIntermediate,
                   ProresColorProfile::P3D60Linear}) {
        if (value == toString(p)) { profile = p; return true; }
    }
    return false;
}
inline bool parsePreviewColorProfile(std::string_view value, ProresColorProfile &profile)
{
    return parseProresColorProfile(value, profile);
}
inline const char *proresColorSpace(ProresColorProfile profile)
{
    switch (profile) {
    case ProresColorProfile::ArriLogC4: return "ARRI Wide Gamut 4";
    case ProresColorProfile::SonySLog3: return "Sony S-Gamut3.Cine";
    case ProresColorProfile::DavinciIntermediate: return "DaVinci Wide Gamut";
    case ProresColorProfile::P3D60Linear: return "P3-D60";
    default: return "Rec.709 primaries / selected tuning";
    }
}
inline const char *proresGamma(ProresColorProfile profile)
{
    switch (profile) {
    case ProresColorProfile::ArriLogC4: return "ARRI LogC4";
    case ProresColorProfile::SonySLog3: return "Sony S-Log3";
    case ProresColorProfile::DavinciIntermediate: return "DaVinci Intermediate";
    case ProresColorProfile::P3D60Linear: return "Linear";
    default: return "Selected tuning gamma";
    }
}

// Published scene-linear OETFs (identity for Linear), before ISP approximation.
// ARRI LogC4 specification (2025), Sony S-Log3 technical summary, and
// Blackmagic DaVinci Wide Gamut / Intermediate specification (August 2021).
inline double proresLogEncode(ProresColorProfile profile, double linear)
{
    switch (profile) {
    case ProresColorProfile::ArriLogC4: {
        const double a = (262144.0 - 16) / 117.45, b = 928.0 / 1023, c = 95.0 / 1023;
        const double t = (std::exp2(6 - 14 * c / b) - 64) / a;
        const double s = 7 * std::log(2.0) * std::exp2(7 - 14 * c / b) / (a * b);
        return linear < t ? (linear - t) / s : (std::log2(a * linear + 64) - 6) / 14 * b + c;
    }
    case ProresColorProfile::SonySLog3:
        return linear < 0.01125
            ? (linear * (171.2102946929 - 95) / 0.01125 + 95) / 1023
            : (420 + std::log10((linear + 0.01) / 0.19) * 261.5) / 1023;
    case ProresColorProfile::DavinciIntermediate:
        return linear <= 0.00262409 ? linear * 10.44426855
            : (std::log2(linear + 0.0075) + 7) * 0.07329248;
    case ProresColorProfile::P3D60Linear: return linear;
    default: return linear;
    }
}
using ProresColorMatrix = std::array<double, 9>;
ProresColorMatrix proresGamutMatrix(ProresColorProfile profile);
}
