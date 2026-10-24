#pragma once
#include <array>
#include <cstdint>
#include <cmath>
#include <optional>

namespace apertar {
inline std::optional<std::array<uint32_t, 3>> stillBracketExposures(
    uint32_t baseUs, uint32_t minimumUs, uint32_t maximumUs, double offsetEv = 1.0)
{
    if (!std::isfinite(offsetEv) || offsetEv < 1.0 / 3.0 || offsetEv > 3.0)
        return std::nullopt;
    const double factor = std::exp2(offsetEv);
    const uint64_t under = uint64_t(std::llround(baseUs / factor));
    const uint64_t over = uint64_t(std::llround(baseUs * factor));
    if (!baseUs || under < minimumUs || over > maximumUs || under >= baseUs || over <= baseUs)
        return std::nullopt;
    return std::array<uint32_t, 3>{uint32_t(under), baseUs, uint32_t(over)};
}
}
