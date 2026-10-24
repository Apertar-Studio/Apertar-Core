#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <apertar/core/Types.hpp>

namespace apertar {

inline float blackLevelForSampleBits(float black, unsigned bits, bool onContainerScale = false)
{
    if (!std::isfinite(black) || bits == 0 || bits > 16)
        return 0.0f;
    const float white = static_cast<float>((1u << bits) - 1u);
    const float scaled = onContainerScale ? black : black * white / 4095.0f;
    return std::clamp(scaled, 0.0f, white - 1.0f);
}

// Metadata is R, Gr, Gb, B; a DNG's 2x2 BlackLevel array follows the CFA tile.
inline std::array<unsigned, 4> blackLevelCfaIndices(BayerPattern pattern)
{
    switch (pattern) {
    case BayerPattern::RGGB: return {0, 1, 2, 3};
    case BayerPattern::GRBG: return {1, 0, 3, 2};
    case BayerPattern::GBRG: return {2, 3, 0, 1};
    case BayerPattern::BGGR: return {3, 2, 1, 0};
    case BayerPattern::Monochrome: return {0, 0, 0, 0};
    }
    return {0, 1, 2, 3};
}

} // namespace apertar
