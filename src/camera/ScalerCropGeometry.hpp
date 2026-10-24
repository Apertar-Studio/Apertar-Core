#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>

namespace apertar {

struct NativeScalerCrop {
    int x = 0;
    int y = 0;
    uint32_t width = 0;
    uint32_t height = 0;
};

inline std::optional<NativeScalerCrop> centeredRawWindowScalerCrop(
    uint32_t sourceWidth, uint32_t sourceHeight,
    uint32_t targetWidth, uint32_t targetHeight, NativeScalerCrop nativeBounds)
{
    if (targetWidth < 2 || targetHeight < 2 || targetWidth > sourceWidth ||
        targetHeight > sourceHeight || nativeBounds.width < 2 || nativeBounds.height < 2)
        return std::nullopt;
    const uint32_t left = ((sourceWidth - targetWidth) / 2u) & ~1u;
    const uint32_t top = ((sourceHeight - targetHeight) / 2u) & ~1u;
    return NativeScalerCrop{
        nativeBounds.x + static_cast<int>((uint64_t(nativeBounds.width) * left / sourceWidth) & ~1ull),
        nativeBounds.y + static_cast<int>((uint64_t(nativeBounds.height) * top / sourceHeight) & ~1ull),
        static_cast<uint32_t>((uint64_t(nativeBounds.width) * targetWidth / sourceWidth) & ~1ull),
        static_cast<uint32_t>((uint64_t(nativeBounds.height) * targetHeight / sourceHeight) & ~1ull)};
}

inline std::optional<NativeScalerCrop> centeredPreviewScalerCrop(
    uint32_t sourceWidth, uint32_t sourceHeight,
    uint32_t targetWidth, uint32_t targetHeight, NativeScalerCrop nativeBounds)
{
    if (sourceWidth < 2 || sourceHeight < 2 || targetWidth < 2 || targetHeight < 2 ||
        nativeBounds.width < 2 || nativeBounds.height < 2)
        return std::nullopt;

    uint32_t cropWidth = sourceWidth;
    uint32_t cropHeight = sourceHeight;
    const uint64_t sourceAspect = uint64_t(sourceWidth) * targetHeight;
    const uint64_t targetAspect = uint64_t(sourceHeight) * targetWidth;
    if (sourceAspect > targetAspect) {
        cropWidth = std::min(sourceWidth, static_cast<uint32_t>(
            std::max<uint64_t>(2, (uint64_t(sourceHeight) * targetWidth / targetHeight) & ~1ull)));
    } else if (sourceAspect < targetAspect) {
        cropHeight = std::min(sourceHeight, static_cast<uint32_t>(
            std::max<uint64_t>(2, (uint64_t(sourceWidth) * targetHeight / targetWidth) & ~1ull)));
    }

    // ScalerCrop uses native pixels, not the (possibly binned/padded) RAW output.
    // Preserve the driver's whole active area unless a software aspect crop is needed.
    const uint32_t nativeWidth = cropWidth == sourceWidth ? nativeBounds.width
        : std::min(nativeBounds.width, static_cast<uint32_t>(std::max<uint64_t>(2,
            (uint64_t(nativeBounds.width) * cropWidth / sourceWidth) & ~1ull)));
    const uint32_t nativeHeight = cropHeight == sourceHeight ? nativeBounds.height
        : std::min(nativeBounds.height, static_cast<uint32_t>(std::max<uint64_t>(2,
            (uint64_t(nativeBounds.height) * cropHeight / sourceHeight) & ~1ull)));
    return NativeScalerCrop{
        nativeBounds.x + static_cast<int>(((nativeBounds.width - nativeWidth) / 2) & ~1u),
        nativeBounds.y + static_cast<int>(((nativeBounds.height - nativeHeight) / 2) & ~1u),
        nativeWidth, nativeHeight};
}

} // namespace apertar
