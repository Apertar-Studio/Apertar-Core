#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace apertar {

struct EncoderColorProfile {
    std::string name;
    std::string cacheKey;

    bool hasColorMatrix1 = false;
    bool hasColorMatrix2 = false;
    std::array<float, 9> colorMatrix1{};
    std::array<float, 9> colorMatrix2{};

    bool hasForwardMatrix1 = false;
    bool hasForwardMatrix2 = false;
    std::array<float, 9> forwardMatrix1{};
    std::array<float, 9> forwardMatrix2{};

    bool hasCcm = false;
    std::array<float, 9> ccm{};

    uint16_t illuminant1 = 21;
    uint16_t illuminant2 = 21;
};

std::optional<EncoderColorProfile> encoderColorProfileForSensor(const std::string &sensorModel,
                                                                uint32_t whiteBalanceKelvin);

// MLV readers interpret their single native-RAW ColorMatrix as daylight/D65.
std::optional<std::array<float, 9>> encoderDaylightColorMatrixForSensor(const std::string &sensorModel);

bool activeTuningHasCalibratedWhiteBalance();
uint8_t activeTuningExposureMeteringModes();

// R, Gr, Gb, B black levels on the encoders' 12-bit reference scale.
std::optional<std::array<float, 4>> activeTuningBlackLevels();

// Snapshot the selected tuning's display tone curve (16-bit input/output domain).
std::vector<std::array<float, 2>> activeTuningGammaCurve();

// CCM operates on white-balanced RGB; ColorMatrix operates on native RAW.
std::array<float, 9> dngColorMatrixFromCcm(const std::array<float, 9> &ccm,
                                        const std::array<float, 2> &gains,
                                        uint16_t illuminant = 21);
std::array<float, 9> dngForwardMatrixFromCcm(const std::array<float, 9> &ccm);

} // namespace apertar
