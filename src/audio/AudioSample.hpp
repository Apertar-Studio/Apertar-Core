#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace apertar {
inline unsigned audioSampleBytes(const std::string &format)
{ return format == "s16" ? 2 : format == "s24" ? 3 : 4; }
inline bool validAudioFormat(const std::string &format)
{ return format == "auto" || format == "s16" || format == "s24" || format == "s32" || format == "float32"; }
inline double readAudioSample(const char *p, const std::string &format)
{
    if (format == "float32") { float v; std::memcpy(&v, p, 4); return std::isfinite(v) ? v : 0; }
    const unsigned bytes = audioSampleBytes(format);
    uint32_t bits = 0;
    for (unsigned i = 0; i < bytes; ++i) bits |= uint32_t(uint8_t(p[i])) << (i * 8);
    const int32_t sample = bytes == 2 ? int16_t(bits) : bytes == 3 ? int32_t(bits << 8) / 256 : int32_t(bits);
    return sample / std::ldexp(1.0, bytes * 8 - 1);
}
inline void writeAudioSample(char *p, const std::string &format, double value)
{
    if (format == "float32") { const float v = std::isfinite(value) ? float(value) : 0; std::memcpy(p, &v, 4); return; }
    const unsigned bytes = audioSampleBytes(format);
    const double scale = std::ldexp(1.0, bytes * 8 - 1);
    const auto v = uint32_t(int32_t(std::llround(std::clamp(value * scale, -scale, scale - 1))));
    for (unsigned i = 0; i < bytes; ++i) p[i] = char(v >> (i * 8));
}
} // namespace apertar
