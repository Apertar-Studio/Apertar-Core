#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace apertar {

struct StillRenderSettings {
    std::array<float, 9> ccm{1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::vector<std::array<float, 2>> toneCurve;
};

class StillColorTransform {
public:
    StillColorTransform(const StillRenderSettings &settings, float red, float blue, float digitalGain)
        : ccm_(settings.ccm)
    {
        const auto gain = [](float value) { return std::isfinite(value) && value > 0 ? value : 1.0f; };
        red = gain(red); blue = gain(blue); digitalGain = gain(digitalGain);
        // Match the ISP's common WB gain: no colour channel is attenuated.
        const float common = digitalGain / std::max(0.1f, std::min({red, blue, 1.0f}));
        gains_ = {red * common, common, blue * common};
        if (!std::all_of(ccm_.begin(), ccm_.end(), [](float value) { return std::isfinite(value); }))
            ccm_ = {1, 0, 0, 0, 1, 0, 0, 0, 1};

        bool valid = settings.toneCurve.size() >= 2;
        float previous = -1;
        for (const auto &point : settings.toneCurve) {
            valid = valid && std::isfinite(point[0]) && std::isfinite(point[1]) &&
                point[0] >= 0 && point[0] <= 65535 && point[1] >= 0 && point[1] <= 65535 && point[0] > previous;
            previous = point[0];
        }
        size_t segment = 0;
        for (size_t i = 0; i < tone_.size(); ++i) {
            float encoded;
            if (valid) {
                while (segment + 1 < settings.toneCurve.size() - 1 && i > settings.toneCurve[segment + 1][0])
                    ++segment;
                const auto &a = settings.toneCurve[segment];
                const auto &b = settings.toneCurve[segment + 1];
                const float t = std::clamp((static_cast<float>(i) - a[0]) / (b[0] - a[0]), 0.0f, 1.0f);
                encoded = a[1] + t * (b[1] - a[1]);
            } else {
                const float linear = static_cast<float>(i) / 65535.0f;
                encoded = 65535.0f * (linear <= 0.0031308f ? 12.92f * linear
                    : 1.055f * std::pow(linear, 1.0f / 2.4f) - 0.055f);
            }
            tone_[i] = static_cast<uint16_t>(std::lround(std::clamp(encoded, 0.0f, 65535.0f)));
        }
    }

    std::array<uint16_t, 3> render(float red, float green, float blue) const
    {
        const std::array<float, 3> balanced{
            std::clamp(red * gains_[0], 0.0f, 1.0f),
            std::clamp(green * gains_[1], 0.0f, 1.0f),
            std::clamp(blue * gains_[2], 0.0f, 1.0f),
        };
        std::array<uint16_t, 3> output;
        for (size_t row = 0; row < 3; ++row) {
            const float linear = ccm_[row * 3] * balanced[0] +
                ccm_[row * 3 + 1] * balanced[1] + ccm_[row * 3 + 2] * balanced[2];
            const auto index = static_cast<size_t>(std::lround(std::clamp(linear, 0.0f, 1.0f) * 65535.0f));
            output[row] = tone_[index];
        }
        return output;
    }

private:
    std::array<float, 9> ccm_;
    std::array<float, 3> gains_;
    std::array<uint16_t, 65536> tone_;
};

} // namespace apertar
