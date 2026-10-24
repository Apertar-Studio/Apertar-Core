#include "hardware/Mpu6050RecordingSession.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace apertar {
namespace {

constexpr uint8_t kRegSampleRateDivider = 0x19;
constexpr uint8_t kRegConfig = 0x1a;
constexpr uint8_t kRegGyroConfig = 0x1b;
constexpr uint8_t kRegAccelConfig = 0x1c;
constexpr uint8_t kRegFifoEnable = 0x23;
constexpr uint8_t kRegAccelXoutH = 0x3b;
constexpr uint8_t kRegUserControl = 0x6a;
constexpr uint8_t kRegPwrMgmt1 = 0x6b;
constexpr uint8_t kRegFifoCountH = 0x72;
constexpr uint8_t kRegFifoRw = 0x74;
constexpr uint8_t kRegWhoAmI = 0x75;
constexpr uint8_t kUserControlFifoEnable = 0x40;
constexpr uint8_t kUserControlFifoReset = 0x04;
constexpr uint8_t kFifoEnableAccelGyro = 0x78;
constexpr uint16_t kFifoCapacityBytes = 1024;
constexpr size_t kFifoPacketBytes = 12;

constexpr uint8_t kBmi160RegChipId = 0x00;
constexpr uint8_t kBmi160RegGyroData = 0x0c;
constexpr uint8_t kBmi160RegAccelConfig = 0x40;
constexpr uint8_t kBmi160RegAccelRange = 0x41;
constexpr uint8_t kBmi160RegGyroConfig = 0x42;
constexpr uint8_t kBmi160RegGyroRange = 0x43;
constexpr uint8_t kBmi160RegCommand = 0x7e;
constexpr uint8_t kBmi160ChipId = 0xd1;

constexpr uint8_t kBmi270RegChipId = 0x00;
constexpr uint8_t kBmi270RegAccelData = 0x0c;
constexpr uint8_t kBmi270RegInternalStatus = 0x21;
constexpr uint8_t kBmi270RegAccelConfig = 0x40;
constexpr uint8_t kBmi270RegAccelRange = 0x41;
constexpr uint8_t kBmi270RegGyroConfig = 0x42;
constexpr uint8_t kBmi270RegGyroRange = 0x43;
constexpr uint8_t kBmi270RegPowerConf = 0x7c;
constexpr uint8_t kBmi270RegPowerCtrl = 0x7d;
constexpr uint8_t kBmi270ChipId = 0x24;

constexpr uint8_t kIcm42688RegDeviceConfig = 0x11;
constexpr uint8_t kIcm42688RegAccelData = 0x1f;
constexpr uint8_t kIcm42688RegIntStatus = 0x2d;
constexpr uint8_t kIcm42688RegPowerMgmt0 = 0x4e;
constexpr uint8_t kIcm42688RegGyroConfig0 = 0x4f;
constexpr uint8_t kIcm42688RegAccelConfig0 = 0x50;
constexpr uint8_t kIcm42688RegWhoAmI = 0x75;
constexpr uint8_t kIcm42688RegBankSelect = 0x76;
constexpr uint8_t kIcm42688WhoAmI = 0x47;

void logAutoProbeSuccessOnce(const std::string &driverLabel, int bus, int address, uint8_t chipId)
{
    static std::mutex mutex;
    static std::set<std::string> loggedProbeKeys;

    std::ostringstream key;
    key << driverLabel << ':' << bus << ':' << address << ':' << static_cast<int>(chipId);

    std::lock_guard<std::mutex> lock(mutex);
    if (!loggedProbeKeys.insert(key.str()).second)
        return;

    std::cerr << "ApertarCore " << driverLabel << " auto-probe found "
              << "i2c-" << bus
              << " address=0x" << std::hex << address
              << " chip_id=0x" << static_cast<int>(chipId) << std::dec << "\n";
}

std::string trimAscii(std::string value)
{
    const auto notSpace = [](unsigned char ch) {
        return !std::isspace(ch);
    };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
    return value;
}

std::string lowerAscii(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

std::string normalizeToken(std::string value)
{
    value = lowerAscii(trimAscii(std::move(value)));
    for (char &ch : value) {
        if (ch == '-' || ch == ' ' || ch == '.')
            ch = '_';
    }
    return value;
}

std::string canonicalDriverName(const std::string &value)
{
    const std::string normalized = normalizeToken(value);
    if (normalized == "bmi160")
        return "bmi160";
    if (normalized == "bmi270")
        return "bmi270";
    if (normalized == "icm42688p" ||
        normalized == "icm42688_p" ||
        normalized == "icm_42688_p" ||
        normalized == "icm_42688p")
        return "icm42688p";
    return "mpu6050";
}

Mpu6050RecordingSession::DriverKind driverKindFromName(const std::string &driver)
{
    const std::string canonical = canonicalDriverName(driver);
    if (canonical == "bmi160")
        return Mpu6050RecordingSession::DriverKind::Bmi160;
    if (canonical == "bmi270")
        return Mpu6050RecordingSession::DriverKind::Bmi270;
    if (canonical == "icm42688p")
        return Mpu6050RecordingSession::DriverKind::Icm42688p;
    return Mpu6050RecordingSession::DriverKind::Mpu6050;
}

std::string driverConfigSection(const std::string &driver)
{
    return canonicalDriverName(driver);
}

std::string driverLabel(Mpu6050RecordingSession::DriverKind kind)
{
    switch (kind) {
    case Mpu6050RecordingSession::DriverKind::Bmi160:
        return "BMI160";
    case Mpu6050RecordingSession::DriverKind::Bmi270:
        return "BMI270";
    case Mpu6050RecordingSession::DriverKind::Icm42688p:
        return "ICM42688-P";
    case Mpu6050RecordingSession::DriverKind::Mpu6050:
    default:
        return "MPU6050";
    }
}

bool isActiveGyroConfigSection(const std::string &section, const std::string &driver)
{
    const std::string normalized = normalizeToken(section);
    if (normalized == "gyro")
        return true;

    // Driver fallback is valid for selection, not for unrelated config sections.
    const std::string activeDriver = canonicalDriverName(driver);
    if (activeDriver == "icm42688p") {
        return normalized == "icm42688p" || normalized == "icm42688_p" ||
               normalized == "icm_42688_p" || normalized == "icm_42688p";
    }
    return normalized == activeDriver;
}

std::filesystem::path defaultHardwareConfigPath()
{
    return "/home/pi/apertar-hardware.conf";
}

std::vector<std::filesystem::path> hardwareConfigCandidatePaths()
{
    std::vector<std::filesystem::path> candidates;
    if (const char *envPath = std::getenv("APERTAR_HARDWARE_CONFIG"); envPath && *envPath)
        candidates.emplace_back(envPath);

    candidates.push_back(defaultHardwareConfigPath());
    candidates.emplace_back("/boot/firmware/apertar-hardware.conf");
    candidates.emplace_back("/boot/apertar-hardware.conf");

    std::vector<std::filesystem::path> unique;
    for (const auto &path : candidates) {
        if (!path.empty() && std::find(unique.begin(), unique.end(), path) == unique.end())
            unique.push_back(path);
    }
    return unique;
}

std::string stripInlineComment(std::string value)
{
    const size_t hash = value.find('#');
    const size_t semi = value.find(';');
    size_t pos = std::string::npos;
    if (hash != std::string::npos)
        pos = hash;
    if (semi != std::string::npos)
        pos = pos == std::string::npos ? semi : std::min(pos, semi);
    if (pos != std::string::npos)
        value.resize(pos);
    return trimAscii(std::move(value));
}

std::optional<int> parseConfigInteger(const std::string &text)
{
    const std::string trimmed = stripInlineComment(text);
    if (trimmed.empty())
        return std::nullopt;

    char *end = nullptr;
    errno = 0;
    const long value = std::strtol(trimmed.c_str(), &end, 0);
    if (errno != 0 || end == trimmed.c_str())
        return std::nullopt;

    while (*end != '\0') {
        if (!std::isspace(static_cast<unsigned char>(*end)))
            return std::nullopt;
        ++end;
    }
    return static_cast<int>(value);
}

std::optional<double> parseConfigDouble(const std::string &text)
{
    const std::string trimmed = stripInlineComment(text);
    if (trimmed.empty())
        return std::nullopt;

    char *end = nullptr;
    errno = 0;
    const double value = std::strtod(trimmed.c_str(), &end);
    if (errno != 0 || end == trimmed.c_str())
        return std::nullopt;

    while (*end != '\0') {
        if (!std::isspace(static_cast<unsigned char>(*end)))
            return std::nullopt;
        ++end;
    }
    return value;
}

bool parseConfigBool(const std::string &text, bool fallback)
{
    const std::string normalized = normalizeToken(stripInlineComment(text));
    if (normalized == "true" || normalized == "yes" || normalized == "on" || normalized == "1")
        return true;
    if (normalized == "false" || normalized == "no" || normalized == "off" || normalized == "0")
        return false;
    return fallback;
}

std::optional<std::string> normalizeOrientationValue(const std::string &text)
{
    std::string value = stripInlineComment(text);
    value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char ch) {
        return std::isspace(ch);
    }), value.end());

    if (value.size() != 3)
        return std::nullopt;

    std::array<bool, 3> used{ false, false, false };
    for (char ch : value) {
        const char lower = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        int index = -1;
        if (lower == 'x')
            index = 0;
        else if (lower == 'y')
            index = 1;
        else if (lower == 'z')
            index = 2;
        else
            return std::nullopt;

        if (used[index])
            return std::nullopt;
        used[index] = true;
    }

    return value;
}

int clampInt(int value, int minValue, int maxValue)
{
    return std::max(minValue, std::min(maxValue, value));
}

uint8_t dlpfConfigForHz(int hz)
{
    if (hz <= 5)
        return 6;
    if (hz <= 10)
        return 5;
    if (hz <= 20)
        return 4;
    if (hz <= 42)
        return 3;
    if (hz <= 98)
        return 2;
    if (hz <= 188)
        return 1;
    return 0;
}

uint8_t gyroRangeBits(int rangeDps, double *scaleDpsPerLsb)
{
    if (rangeDps <= 250) {
        *scaleDpsPerLsb = 131.0;
        return 0;
    }
    if (rangeDps <= 500) {
        *scaleDpsPerLsb = 65.5;
        return 1;
    }
    if (rangeDps <= 1000) {
        *scaleDpsPerLsb = 32.8;
        return 2;
    }
    *scaleDpsPerLsb = 16.4;
    return 3;
}

uint8_t accelRangeBits(int rangeG, double *scaleLsbPerG)
{
    if (rangeG <= 2) {
        *scaleLsbPerG = 16384.0;
        return 0;
    }
    if (rangeG <= 4) {
        *scaleLsbPerG = 8192.0;
        return 1;
    }
    if (rangeG <= 8) {
        *scaleLsbPerG = 4096.0;
        return 2;
    }
    *scaleLsbPerG = 2048.0;
    return 3;
}

uint8_t bmiOdrCode(int hz)
{
    if (hz <= 25)
        return 0x06;
    if (hz <= 50)
        return 0x07;
    if (hz <= 100)
        return 0x08;
    if (hz <= 200)
        return 0x09;
    if (hz <= 400)
        return 0x0a;
    if (hz <= 800)
        return 0x0b;
    return 0x0c;
}

uint8_t bmi160AccelRangeValue(int rangeG, double *scaleLsbPerG)
{
    if (rangeG <= 2) {
        *scaleLsbPerG = 16384.0;
        return 0x03;
    }
    if (rangeG <= 4) {
        *scaleLsbPerG = 8192.0;
        return 0x05;
    }
    if (rangeG <= 8) {
        *scaleLsbPerG = 4096.0;
        return 0x08;
    }
    *scaleLsbPerG = 2048.0;
    return 0x0c;
}

uint8_t bmi270AccelRangeValue(int rangeG, double *scaleLsbPerG)
{
    if (rangeG <= 2) {
        *scaleLsbPerG = 16384.0;
        return 0x00;
    }
    if (rangeG <= 4) {
        *scaleLsbPerG = 8192.0;
        return 0x01;
    }
    if (rangeG <= 8) {
        *scaleLsbPerG = 4096.0;
        return 0x02;
    }
    *scaleLsbPerG = 2048.0;
    return 0x03;
}

uint8_t bmiGyroRangeValue(int rangeDps, double *scaleDpsPerLsb)
{
    if (rangeDps <= 250) {
        *scaleDpsPerLsb = 131.0;
        return 0x03;
    }
    if (rangeDps <= 500) {
        *scaleDpsPerLsb = 65.5;
        return 0x02;
    }
    if (rangeDps <= 1000) {
        *scaleDpsPerLsb = 32.8;
        return 0x01;
    }
    *scaleDpsPerLsb = 16.4;
    return 0x00;
}

uint8_t icm42688OdrCode(int hz)
{
    if (hz <= 25)
        return 0x0a;
    if (hz <= 50)
        return 0x09;
    if (hz <= 100)
        return 0x08;
    if (hz <= 200)
        return 0x07;
    if (hz <= 500)
        return 0x0f;
    return 0x06;
}

uint8_t icm42688GyroRangeBits(int rangeDps, double *scaleDpsPerLsb)
{
    if (rangeDps <= 250) {
        *scaleDpsPerLsb = 131.0;
        return 0x60;
    }
    if (rangeDps <= 500) {
        *scaleDpsPerLsb = 65.5;
        return 0x40;
    }
    if (rangeDps <= 1000) {
        *scaleDpsPerLsb = 32.8;
        return 0x20;
    }
    *scaleDpsPerLsb = 16.4;
    return 0x00;
}

uint8_t icm42688AccelRangeBits(int rangeG, double *scaleLsbPerG)
{
    if (rangeG <= 2) {
        *scaleLsbPerG = 16384.0;
        return 0x60;
    }
    if (rangeG <= 4) {
        *scaleLsbPerG = 8192.0;
        return 0x40;
    }
    if (rangeG <= 8) {
        *scaleLsbPerG = 4096.0;
        return 0x20;
    }
    *scaleLsbPerG = 2048.0;
    return 0x00;
}

void sensorScalesForConfig(Mpu6050RecordingSession::DriverKind kind,
                           int gyroRangeDps,
                           int accelRangeG,
                           double *gyroScaleDpsPerLsb,
                           double *accelScaleGPerLsb)
{
    double gyroScale = 32.8;
    double accelScale = 4096.0;
    switch (kind) {
    case Mpu6050RecordingSession::DriverKind::Bmi160:
        (void)bmiGyroRangeValue(gyroRangeDps, &gyroScale);
        (void)bmi160AccelRangeValue(accelRangeG, &accelScale);
        break;
    case Mpu6050RecordingSession::DriverKind::Bmi270:
        (void)bmiGyroRangeValue(gyroRangeDps, &gyroScale);
        (void)bmi270AccelRangeValue(accelRangeG, &accelScale);
        break;
    case Mpu6050RecordingSession::DriverKind::Icm42688p:
        (void)icm42688GyroRangeBits(gyroRangeDps, &gyroScale);
        (void)icm42688AccelRangeBits(accelRangeG, &accelScale);
        break;
    case Mpu6050RecordingSession::DriverKind::Mpu6050:
    default:
        (void)gyroRangeBits(gyroRangeDps, &gyroScale);
        (void)accelRangeBits(accelRangeG, &accelScale);
        break;
    }

    if (gyroScaleDpsPerLsb)
        *gyroScaleDpsPerLsb = gyroScale;
    if (accelScaleGPerLsb)
        *accelScaleGPerLsb = accelScale;
}

int16_t be16(const uint8_t *data)
{
    return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

int16_t le16(const uint8_t *data)
{
    return static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
}

int16_t clampToInt16(double value)
{
    return static_cast<int16_t>(std::clamp(
        std::llround(value),
        static_cast<long long>(std::numeric_limits<int16_t>::min()),
        static_cast<long long>(std::numeric_limits<int16_t>::max())));
}

uint64_t monotonicClockUs()
{
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }
    return (static_cast<uint64_t>(ts.tv_sec) * 1000000ull) +
           (static_cast<uint64_t>(ts.tv_nsec) / 1000ull);
}

std::string formatRawBias(double value)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << value;
    return out.str();
}

std::string formatConfigDouble(double value, int precision)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    std::string text = out.str();
    while (text.size() > 1 && text.back() == '0')
        text.pop_back();
    if (!text.empty() && text.back() == '.')
        text.pop_back();
    return text.empty() ? "0" : text;
}

std::string formatSyncOffsetText(double value)
{
    const double rounded = std::round(value);
    const int decimals = std::abs(value - rounded) < 0.05 ? 0 : 1;
    const std::string sign = value > 0.05 ? "+" : "";
    return sign + formatConfigDouble(value, decimals) + " ms";
}

std::string formatLevelReferenceText(double accelXG, double accelYG, double accelZG)
{
    return "X " + formatConfigDouble(accelXG, 2)
        + " / Y " + formatConfigDouble(accelYG, 2)
        + " / Z " + formatConfigDouble(accelZG, 2) + " g";
}

double mappedAccelForOrientation(const std::string &orientation,
                                 double accelXG,
                                 double accelYG,
                                 double accelZG,
                                 int index)
{
    const std::string normalized = normalizeOrientationValue(orientation).value_or("XYZ");
    if (index < 0 || index >= static_cast<int>(normalized.size()))
        return 0.0;

    const char token = normalized[static_cast<size_t>(index)];
    const char axis = static_cast<char>(std::toupper(static_cast<unsigned char>(token)));
    double value = 0.0;
    if (axis == 'X')
        value = accelXG;
    else if (axis == 'Y')
        value = accelYG;
    else if (axis == 'Z')
        value = accelZG;

    return token == axis ? value : -value;
}

bool writeHardwareConfigValue(const std::filesystem::path &configPath,
                              const std::string &sectionName,
                              const std::string &key,
                              const std::string &value,
                              std::string *error)
{
    const std::filesystem::path path = configPath.empty() ? defaultHardwareConfigPath() : configPath;
    std::vector<std::string> lines;

    {
        std::ifstream input(path);
        std::string line;
        while (std::getline(input, line)) {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            lines.push_back(line);
        }
    }

    if (lines.empty()) {
        lines.emplace_back("# Apertar hardware configuration");
        lines.emplace_back();
    }

    const std::string targetSection = normalizeToken(sectionName);
    const std::string targetKey = normalizeToken(key);
    bool foundSection = false;
    bool inTargetSection = false;
    bool wroteKey = false;

    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string trimmed = trimAscii(lines[i]);
        if (trimmed.size() >= 2 && trimmed.front() == '[' && trimmed.back() == ']') {
            if (inTargetSection && !wroteKey) {
                lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(i), targetKey + "=" + value);
                wroteKey = true;
                ++i;
            }

            inTargetSection = normalizeToken(trimmed.substr(1, trimmed.size() - 2)) == targetSection;
            foundSection = foundSection || inTargetSection;
            continue;
        }

        if (!inTargetSection)
            continue;

        const size_t eq = lines[i].find('=');
        if (eq == std::string::npos)
            continue;

        if (normalizeToken(lines[i].substr(0, eq)) == targetKey) {
            const size_t indentEnd = lines[i].find_first_not_of(" \t");
            const std::string indent = indentEnd == std::string::npos
                ? std::string()
                : lines[i].substr(0, indentEnd);
            lines[i] = indent + targetKey + "=" + value;
            wroteKey = true;
        }
    }

    if (inTargetSection && !wroteKey) {
        lines.push_back(targetKey + "=" + value);
        wroteKey = true;
    }

    if (!foundSection) {
        if (!lines.empty() && !trimAscii(lines.back()).empty())
            lines.emplace_back();
        lines.push_back("[" + targetSection + "]");
        lines.push_back(targetKey + "=" + value);
    }

    std::error_code ec;
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), ec);
        if (ec) {
            if (error)
                *error = "Could not create hardware config directory: " + ec.message();
            return false;
        }
    }

    const std::filesystem::path tmpPath = path.string() + ".tmp";
    {
        std::ofstream output(tmpPath, std::ios::out | std::ios::trunc);
        if (!output.is_open()) {
            if (error)
                *error = "Could not write hardware config " + path.string() + ".";
            return false;
        }

        for (const std::string &line : lines)
            output << line << '\n';
    }

    std::filesystem::rename(tmpPath, path, ec);
    if (ec) {
        std::filesystem::remove(path, ec);
        ec.clear();
        std::filesystem::rename(tmpPath, path, ec);
        if (ec) {
            if (error)
                *error = "Could not update hardware config " + path.string() + ": " + ec.message();
            return false;
        }
    }

    return true;
}

bool updateImuBiasConfig(const std::filesystem::path &configPath,
                         const std::string &sectionName,
                         double biasX,
                         double biasY,
                         double biasZ,
                         std::string *error)
{
    const std::filesystem::path path = configPath.empty() ? defaultHardwareConfigPath() : configPath;
    std::vector<std::string> lines;

    {
        std::ifstream input(path);
        std::string line;
        while (std::getline(input, line)) {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            lines.push_back(line);
        }
    }

    const std::array<std::pair<std::string, std::string>, 4> updates{{
        { "auto_calibrate_seconds", "0" },
        { "gyro_bias_x", formatRawBias(biasX) },
        { "gyro_bias_y", formatRawBias(biasY) },
        { "gyro_bias_z", formatRawBias(biasZ) },
    }};
    std::array<bool, updates.size()> written{ false, false, false, false };

    const auto insertMissingIntoSection = [&](size_t insertIndex) {
        std::vector<std::string> missing;
        for (size_t i = 0; i < updates.size(); ++i) {
            if (!written[i]) {
                missing.push_back(updates[i].first + "=" + updates[i].second);
                written[i] = true;
            }
        }
        lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(insertIndex),
                     missing.begin(),
                     missing.end());
        return missing.size();
    };

    const std::string targetSection = sectionName.empty() ? "mpu6050" : normalizeToken(sectionName);
    bool foundTargetSection = false;
    bool inTargetSection = false;
    for (size_t lineIndex = 0; lineIndex < lines.size(); ++lineIndex) {
        const std::string trimmed = trimAscii(lines[lineIndex]);
        if (trimmed.size() >= 2 && trimmed.front() == '[' && trimmed.back() == ']') {
            if (inTargetSection)
                lineIndex += insertMissingIntoSection(lineIndex);

            const std::string section = normalizeToken(trimmed.substr(1, trimmed.size() - 2));
            inTargetSection = section == targetSection;
            foundTargetSection = foundTargetSection || inTargetSection;
            continue;
        }

        if (!inTargetSection)
            continue;

        const size_t eq = lines[lineIndex].find('=');
        if (eq == std::string::npos)
            continue;

        const std::string key = normalizeToken(lines[lineIndex].substr(0, eq));
        for (size_t i = 0; i < updates.size(); ++i) {
            if (key == updates[i].first) {
                const size_t indentEnd = lines[lineIndex].find_first_not_of(" \t");
                const std::string indent = indentEnd == std::string::npos
                    ? std::string()
                    : lines[lineIndex].substr(0, indentEnd);
                lines[lineIndex] = indent + updates[i].first + "=" + updates[i].second;
                written[i] = true;
                break;
            }
        }
    }

    if (inTargetSection)
        insertMissingIntoSection(lines.size());

    if (!foundTargetSection) {
        if (!lines.empty() && !trimAscii(lines.back()).empty())
            lines.emplace_back();
        lines.push_back("[" + targetSection + "]");
        for (size_t i = 0; i < updates.size(); ++i) {
            lines.push_back(updates[i].first + "=" + updates[i].second);
            written[i] = true;
        }
    }

    std::error_code ec;
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), ec);
        if (ec) {
            if (error)
                *error = "Could not create hardware config directory: " + ec.message();
            return false;
        }
    }

    const std::filesystem::path tmpPath = path.string() + ".tmp";
    {
        std::ofstream output(tmpPath, std::ios::out | std::ios::trunc);
        if (!output.is_open()) {
            if (error)
                *error = "Could not write hardware config " + path.string() + ".";
            return false;
        }

        for (const std::string &line : lines)
            output << line << '\n';
    }

    std::filesystem::rename(tmpPath, path, ec);
    if (ec) {
        std::filesystem::remove(path, ec);
        ec.clear();
        std::filesystem::rename(tmpPath, path, ec);
        if (ec) {
            if (error)
                *error = "Could not update hardware config " + path.string() + ": " + ec.message();
            return false;
        }
    }

    return true;
}

} // namespace

Mpu6050RecordingSession::~Mpu6050RecordingSession()
{
    stop();
}

Mpu6050RecordingSession::Config Mpu6050RecordingSession::loadConfig()
{
    Config config;

    for (const auto &path : hardwareConfigCandidatePaths()) {
        std::ifstream input(path);
        if (!input.is_open())
            continue;

        config.configPath = path.string();
        std::vector<std::string> lines;
        std::string line;
        while (std::getline(input, line)) {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            lines.push_back(line);
        }

        bool syncOffsetFromImu = false;
        std::string section;
        for (const std::string &rawLine : lines) {
            const std::string trimmed = trimAscii(rawLine);
            if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';')
                continue;

            if (trimmed.front() == '[' && trimmed.back() == ']') {
                section = normalizeToken(trimmed.substr(1, trimmed.size() - 2));
                continue;
            }

            if (section != "imu")
                continue;

            const size_t eq = rawLine.find('=');
            if (eq == std::string::npos)
                continue;

            const std::string key = normalizeToken(rawLine.substr(0, eq));
            const std::string value = rawLine.substr(eq + 1);
            const std::string normalizedValue = normalizeToken(value);
            if (key == "driver") {
                if (!normalizedValue.empty())
                    config.driver = canonicalDriverName(normalizedValue);
            } else if (key == "sync_mode") {
                if (!normalizedValue.empty())
                    config.syncMode = normalizedValue;
            } else if (key == "enabled") {
                config.enabled = parseConfigBool(value, config.enabled);
            } else if (key == "sync_offset_ms" || key == "offset_ms" || key == "gyro_sync_offset_ms") {
                if (const auto parsed = parseConfigDouble(value)) {
                    config.syncOffsetMs = std::clamp(*parsed, -2000.0, 2000.0);
                    syncOffsetFromImu = true;
                }
            }
        }

        if (config.driver.empty() || config.driver == "auto")
            config.driver = "mpu6050";
        config.driver = canonicalDriverName(config.driver);
        if (config.syncMode.empty())
            config.syncMode = "free_run";

        section.clear();
        bool inImuSection = false;
        bool inDriverSection = false;
        for (const std::string &rawLine : lines) {
            const std::string trimmed = trimAscii(rawLine);
            if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';')
                continue;

            if (trimmed.front() == '[' && trimmed.back() == ']') {
                section = normalizeToken(trimmed.substr(1, trimmed.size() - 2));
                inImuSection = section == "imu";
                inDriverSection = isActiveGyroConfigSection(section, config.driver);
                continue;
            }

            if (!inDriverSection && !inImuSection)
                continue;

            const size_t eq = rawLine.find('=');
            if (eq == std::string::npos)
                continue;

            const std::string key = normalizeToken(rawLine.substr(0, eq));
            const std::string value = rawLine.substr(eq + 1);
            const std::string normalizedValue = normalizeToken(value);

            if (inImuSection) {
                if (key == "driver") {
                    if (!normalizedValue.empty())
                        config.driver = canonicalDriverName(normalizedValue);
                } else if (key == "sync_mode") {
                    if (!normalizedValue.empty())
                        config.syncMode = normalizedValue;
                } else if (key == "enabled") {
                    config.enabled = parseConfigBool(value, config.enabled);
                } else if (key == "sync_offset_ms" || key == "offset_ms" || key == "gyro_sync_offset_ms") {
                    if (const auto parsed = parseConfigDouble(value)) {
                        config.syncOffsetMs = std::clamp(*parsed, -2000.0, 2000.0);
                        syncOffsetFromImu = true;
                    }
                }
                continue;
            }

            if (key == "enabled")
                config.enabled = parseConfigBool(value, config.enabled);
            else if (key == "bus" || key == "bus_number") {
                if (const auto parsed = parseConfigInteger(value))
                    config.busNumber = clampInt(*parsed, 0, 32);
            } else if (key == "address" || key == "addr") {
                if (const auto parsed = parseConfigInteger(value))
                    config.address = clampInt(*parsed, 0x03, 0x77);
            } else if (key == "sample_rate_hz" || key == "sample_rate") {
                if (const auto parsed = parseConfigInteger(value))
                    config.sampleRateHz = clampInt(*parsed, 25, 1000);
            } else if (key == "gyro_range_dps" || key == "gyro_range") {
                if (const auto parsed = parseConfigInteger(value))
                    config.gyroRangeDps = clampInt(*parsed, 250, 2000);
            } else if (key == "accel_range_g" || key == "accel_range") {
                if (const auto parsed = parseConfigInteger(value))
                    config.accelRangeG = clampInt(*parsed, 2, 16);
            } else if (key == "dlpf_hz" || key == "filter_hz") {
                if (const auto parsed = parseConfigInteger(value))
                    config.dlpfHz = clampInt(*parsed, 5, 260);
            } else if (key == "use_fifo" || key == "fifo") {
                config.useFifo = parseConfigBool(value, config.useFifo);
            } else if (key == "fifo_batch_samples" || key == "batch_samples") {
                if (const auto parsed = parseConfigInteger(value))
                    config.fifoBatchSamples = clampInt(*parsed, 1, 64);
            } else if (key == "orientation" || key == "imu_orientation") {
                if (const auto parsed = normalizeOrientationValue(value))
                    config.orientation = *parsed;
            } else if (key == "auto_calibrate_seconds" || key == "gyro_auto_calibrate_seconds") {
                if (const auto parsed = parseConfigDouble(value))
                    config.autoCalibrateSeconds = std::clamp(*parsed, 0.0, 5.0);
            } else if (key == "gyro_bias_x" || key == "gyro_bias_x_raw") {
                if (const auto parsed = parseConfigDouble(value))
                    config.gyroBiasXRaw = *parsed;
            } else if (key == "gyro_bias_y" || key == "gyro_bias_y_raw") {
                if (const auto parsed = parseConfigDouble(value))
                    config.gyroBiasYRaw = *parsed;
            } else if (key == "gyro_bias_z" || key == "gyro_bias_z_raw") {
                if (const auto parsed = parseConfigDouble(value))
                    config.gyroBiasZRaw = *parsed;
            } else if (key == "level_reference_set" || key == "level_set" || key == "accel_level_set") {
                config.levelReferenceSet = parseConfigBool(value, config.levelReferenceSet);
            } else if (key == "level_reference_orientation" || key == "level_orientation") {
                if (const auto parsed = normalizeOrientationValue(value))
                    config.levelReferenceOrientation = *parsed;
            } else if (key == "level_accel_x" || key == "level_x_g" || key == "accel_zero_x") {
                if (const auto parsed = parseConfigDouble(value))
                    config.levelAccelXG = *parsed;
            } else if (key == "level_accel_y" || key == "level_y_g" || key == "accel_zero_y") {
                if (const auto parsed = parseConfigDouble(value))
                    config.levelAccelYG = *parsed;
            } else if (key == "level_accel_z" || key == "level_z_g" || key == "accel_zero_z") {
                if (const auto parsed = parseConfigDouble(value))
                    config.levelAccelZG = *parsed;
            } else if (key == "sync_offset_ms" || key == "offset_ms" || key == "gyro_sync_offset_ms") {
                if (!syncOffsetFromImu) {
                    if (const auto parsed = parseConfigDouble(value))
                        config.syncOffsetMs = std::clamp(*parsed, -2000.0, 2000.0);
                }
            } else if (key == "frame_readout_time_ms" || key == "frame_readout_time") {
                if (const auto parsed = parseConfigDouble(value))
                    config.frameReadoutTimeMs = std::clamp(*parsed, 0.0, 100.0);
            } else if (key == "frame_readout_direction" || key == "readout_direction") {
                if (const auto parsed = parseConfigInteger(value))
                    config.frameReadoutDirection = clampInt(*parsed, 0, 3);
            }
        }

        config.driver = canonicalDriverName(config.driver);
        if (config.syncMode.empty())
            config.syncMode = "free_run";
        return config;
    }

    config.driver = canonicalDriverName(config.driver);
    return config;
}

bool Mpu6050RecordingSession::calibrateConfiguredSensor(double seconds,
                                                        CalibrationResult *result,
                                                        std::string *error)
{
    Mpu6050RecordingSession session;
    session.config_ = loadConfig();
    if (!session.config_.enabled) {
        if (error)
            *error = "IMU recording is disabled in apertar-hardware.conf.";
        return false;
    }

    if (session.config_.configPath.empty())
        session.config_.configPath = defaultHardwareConfigPath().string();

    if (!session.openDevice(error))
        return false;
    if (!session.configureSensor(error)) {
        session.closeDevice();
        return false;
    }

    if (session.fifoEnabled_) {
        session.writeRegister(kRegFifoEnable, 0x00);
        session.writeRegister(kRegUserControl, 0x00);
        session.fifoEnabled_ = false;
    }

    const auto readGyroRawSample = [&session](int16_t *gx, int16_t *gy, int16_t *gz) {
        if (!gx || !gy || !gz)
            return false;

        if (session.driverKind_ == DriverKind::Bmi160) {
            uint8_t data[12]{};
            if (!session.readRegisters(kBmi160RegGyroData, data, sizeof(data)))
                return false;
            *gx = le16(data + 0);
            *gy = le16(data + 2);
            *gz = le16(data + 4);
            return true;
        }

        if (session.driverKind_ == DriverKind::Bmi270) {
            uint8_t data[12]{};
            if (!session.readRegisters(kBmi270RegAccelData, data, sizeof(data)))
                return false;
            *gx = le16(data + 6);
            *gy = le16(data + 8);
            *gz = le16(data + 10);
            return true;
        }

        if (session.driverKind_ == DriverKind::Icm42688p) {
            uint8_t data[12]{};
            if (!session.readRegisters(kIcm42688RegAccelData, data, sizeof(data)))
                return false;
            *gx = be16(data + 6);
            *gy = be16(data + 8);
            *gz = be16(data + 10);
            return true;
        }

        uint8_t data[14]{};
        if (!session.readRegisters(kRegAccelXoutH, data, sizeof(data)))
            return false;
        *gx = be16(data + 8);
        *gy = be16(data + 10);
        *gz = be16(data + 12);
        return true;
    };

    const double clampedSeconds = std::clamp(seconds, 1.0, 10.0);
    const uint64_t durationUs = static_cast<uint64_t>(std::llround(clampedSeconds * 1000000.0));
    const uint64_t startUs = monotonicClockUs();
    const uint64_t endUs = startUs + durationUs;
    const uint64_t settleEndUs = startUs + 150000ull;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;
    uint32_t sampleCount = 0;

    while (monotonicClockUs() < endUs) {
        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;
        if (!readGyroRawSample(&gx, &gy, &gz)) {
            session.closeDevice();
            if (error)
                *error = session.driverLabel_ + " sample read failed during calibration.";
            return false;
        }

        if (monotonicClockUs() >= settleEndUs) {
            sumX += gx;
            sumY += gy;
            sumZ += gz;
            ++sampleCount;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(std::max<uint32_t>(500, session.samplePeriodUs_)));
    }

    session.closeDevice();

    if (sampleCount < 50) {
        if (error)
            *error = "Not enough IMU samples for calibration.";
        return false;
    }

    const double biasX = sumX / static_cast<double>(sampleCount);
    const double biasY = sumY / static_cast<double>(sampleCount);
    const double biasZ = sumZ / static_cast<double>(sampleCount);
    if (!updateImuBiasConfig(session.config_.configPath,
                             driverConfigSection(session.config_.driver),
                             biasX,
                             biasY,
                             biasZ,
                             error))
        return false;

    if (result) {
        result->gyroBiasXRaw = biasX;
        result->gyroBiasYRaw = biasY;
        result->gyroBiasZRaw = biasZ;
        result->sampleCount = sampleCount;
        result->configPath = session.config_.configPath;
    }

    std::cerr << "ApertarCore gyro calibration saved: "
              << "bias_x=" << biasX
              << " bias_y=" << biasY
              << " bias_z=" << biasZ
              << " samples=" << sampleCount
              << " config=" << session.config_.configPath << "\n";
    return true;
}

Mpu6050RecordingSession::ConfigSnapshot Mpu6050RecordingSession::configuredSensorSnapshot(bool includeLiveSample)
{
    Mpu6050RecordingSession session;
    session.config_ = loadConfig();
    if (session.config_.configPath.empty())
        session.config_.configPath = defaultHardwareConfigPath().string();
    session.driverKind_ = driverKindFromName(session.config_.driver);
    session.driverLabel_ = driverLabel(session.driverKind_);

    ConfigSnapshot snapshot;
    snapshot.enabled = session.config_.enabled;
    snapshot.driver = session.config_.driver;
    snapshot.driverLabel = session.driverLabel_;
    snapshot.syncMode = session.config_.syncMode;
    snapshot.busNumber = session.config_.busNumber;
    snapshot.address = session.config_.address;
    snapshot.sampleRateHz = session.config_.sampleRateHz;
    snapshot.gyroRangeDps = session.config_.gyroRangeDps;
    snapshot.accelRangeG = session.config_.accelRangeG;
    snapshot.orientation = session.config_.orientation;
    snapshot.syncOffsetMs = session.config_.syncOffsetMs;
    snapshot.syncOffsetText = formatSyncOffsetText(session.config_.syncOffsetMs);
    snapshot.configPath = session.config_.configPath;

    const bool levelOrientationMatches =
        session.config_.levelReferenceOrientation.empty() ||
        session.config_.levelReferenceOrientation == session.config_.orientation;
    snapshot.levelReferenceSet = session.config_.levelReferenceSet && levelOrientationMatches;
    snapshot.levelAccelXG = session.config_.levelAccelXG;
    snapshot.levelAccelYG = session.config_.levelAccelYG;
    snapshot.levelAccelZG = session.config_.levelAccelZG;
    snapshot.levelReferenceText = snapshot.levelReferenceSet
        ? formatLevelReferenceText(snapshot.levelAccelXG, snapshot.levelAccelYG, snapshot.levelAccelZG)
        : "Not set";

    if (!includeLiveSample) {
        snapshot.liveStatusText = "Live gyro idle";
        return snapshot;
    }

    if (!snapshot.enabled) {
        snapshot.liveStatusText = "IMU disabled";
        return snapshot;
    }

    std::ostringstream liveKeyStream;
    liveKeyStream << session.config_.driver << ':'
                  << session.config_.busNumber << ':'
                  << session.config_.address << ':'
                  << session.config_.sampleRateHz << ':'
                  << session.config_.gyroRangeDps << ':'
                  << session.config_.accelRangeG << ':'
                  << session.config_.dlpfHz;
    const std::string liveConfigKey = liveKeyStream.str();

    static std::mutex liveConfigMutex;
    static std::string liveConfiguredKey;
    const auto markLiveConfigured = [&]() {
        std::lock_guard<std::mutex> lock(liveConfigMutex);
        liveConfiguredKey = liveConfigKey;
    };
    const auto clearLiveConfigured = [&]() {
        std::lock_guard<std::mutex> lock(liveConfigMutex);
        if (liveConfiguredKey == liveConfigKey)
            liveConfiguredKey.clear();
    };

    bool shouldConfigureLive = false;
    {
        std::lock_guard<std::mutex> lock(liveConfigMutex);
        shouldConfigureLive = liveConfiguredKey != liveConfigKey;
    }

    std::string liveError;
    LiveSample sample;
    bool liveOk = false;
    if (!session.openDevice(&liveError)) {
        snapshot.liveStatusText = liveError.empty() ? ("Waiting for " + snapshot.driverLabel) : liveError;
        session.closeDevice();
        return snapshot;
    }

    if (shouldConfigureLive) {
        session.config_.useFifo = false;
        liveError.clear();
        liveOk = session.configureSensor(&liveError) && session.readLiveSample(&sample);
        if (liveOk)
            markLiveConfigured();
        else
            clearLiveConfigured();
    } else if (session.ensureConfiguredDevice(&liveError)) {
        liveOk = session.readLiveSample(&sample);
        if (!liveOk) {
            session.config_.useFifo = false;
            liveError.clear();
            liveOk = session.configureSensor(&liveError) && session.readLiveSample(&sample);
            if (liveOk)
                markLiveConfigured();
            else
                clearLiveConfigured();
        }
    }

    if (liveOk) {
        snapshot.liveAvailable = true;
        snapshot.liveStatusText = sample.statusText;
        snapshot.busNumber = session.config_.busNumber;
        snapshot.address = session.config_.address;
        snapshot.accelXG = sample.accelXG;
        snapshot.accelYG = sample.accelYG;
        snapshot.accelZG = sample.accelZG;
        snapshot.gyroXDps = sample.gyroXDps;
        snapshot.gyroYDps = sample.gyroYDps;
        snapshot.gyroZDps = sample.gyroZDps;
    } else {
        snapshot.liveStatusText = liveError.empty() ? ("Waiting for " + snapshot.driverLabel) : liveError;
    }

    session.closeDevice();
    return snapshot;
}

bool Mpu6050RecordingSession::setConfiguredDriver(const std::string &driver, std::string *error)
{
    Config config = loadConfig();
    if (config.configPath.empty())
        config.configPath = defaultHardwareConfigPath().string();
    return writeHardwareConfigValue(config.configPath, "imu", "driver", canonicalDriverName(driver), error);
}

bool Mpu6050RecordingSession::setConfiguredOrientation(const std::string &orientation, std::string *error)
{
    const auto normalized = normalizeOrientationValue(orientation);
    if (!normalized) {
        if (error)
            *error = "Orientation must contain X, Y, and Z once.";
        return false;
    }

    Config config = loadConfig();
    if (config.configPath.empty())
        config.configPath = defaultHardwareConfigPath().string();
    return writeHardwareConfigValue(config.configPath,
                                    driverConfigSection(config.driver),
                                    "orientation",
                                    *normalized,
                                    error);
}

bool Mpu6050RecordingSession::setConfiguredSyncOffsetMs(double value, std::string *error)
{
    Config config = loadConfig();
    if (config.configPath.empty())
        config.configPath = defaultHardwareConfigPath().string();
    const double clamped = std::clamp(value, -2000.0, 2000.0);
    return writeHardwareConfigValue(config.configPath,
                                    "imu",
                                    "sync_offset_ms",
                                    formatConfigDouble(clamped, 3),
                                    error);
}

bool Mpu6050RecordingSession::setConfiguredLevelReference(std::string *error)
{
    ConfigSnapshot snapshot = configuredSensorSnapshot(true);
    if (!snapshot.liveAvailable) {
        if (error)
            *error = snapshot.liveStatusText.empty() ? "Gyro live sample is not available." : snapshot.liveStatusText;
        return false;
    }

    const double levelX = mappedAccelForOrientation(snapshot.orientation,
                                                   snapshot.accelXG,
                                                   snapshot.accelYG,
                                                   snapshot.accelZG,
                                                   0);
    const double levelY = mappedAccelForOrientation(snapshot.orientation,
                                                   snapshot.accelXG,
                                                   snapshot.accelYG,
                                                   snapshot.accelZG,
                                                   1);
    const double levelZ = mappedAccelForOrientation(snapshot.orientation,
                                                   snapshot.accelXG,
                                                   snapshot.accelYG,
                                                   snapshot.accelZG,
                                                   2);
    const std::string section = driverConfigSection(snapshot.driver);
    const std::filesystem::path configPath = snapshot.configPath.empty()
        ? defaultHardwareConfigPath()
        : std::filesystem::path(snapshot.configPath);
    return writeHardwareConfigValue(configPath, section, "level_reference_set", "true", error) &&
           writeHardwareConfigValue(configPath, section, "level_reference_orientation", snapshot.orientation, error) &&
           writeHardwareConfigValue(configPath, section, "level_accel_x", formatConfigDouble(levelX, 4), error) &&
           writeHardwareConfigValue(configPath, section, "level_accel_y", formatConfigDouble(levelY, 4), error) &&
           writeHardwareConfigValue(configPath, section, "level_accel_z", formatConfigDouble(levelZ, 4), error);
}

bool Mpu6050RecordingSession::clearConfiguredLevelReference(std::string *error)
{
    Config config = loadConfig();
    if (config.configPath.empty())
        config.configPath = defaultHardwareConfigPath().string();
    const std::string section = driverConfigSection(config.driver);
    return writeHardwareConfigValue(config.configPath, section, "level_reference_set", "false", error) &&
           writeHardwareConfigValue(config.configPath, section, "level_accel_x", "0", error) &&
           writeHardwareConfigValue(config.configPath, section, "level_accel_y", "0", error) &&
           writeHardwareConfigValue(config.configPath, section, "level_accel_z", "0", error);
}

bool Mpu6050RecordingSession::start(const std::string &clipFolder,
                                    const GyroRecordingOptions &options,
                                    std::string *error)
{
    stop();

    if (!options.enabled)
        return true;

    timelineAnchored_.store(false, std::memory_order_release);
    videoStartTimestampUs_.store(0, std::memory_order_release);
    videoStartFrameIndex_.store(std::numeric_limits<uint64_t>::max(), std::memory_order_release);
    fifoEnabled_ = false;

    config_ = loadConfig();
    if (!config_.enabled) {
        setLastError("IMU recording is disabled in apertar-hardware.conf.");
        if (error)
            *error = lastError();
        return false;
    }

    if (clipFolder.empty()) {
        setLastError("Gyro recording requires a valid clip folder.");
        if (error)
            *error = lastError();
        return false;
    }

    if (!openDevice(error))
        return false;
    if (!configureSensor(error)) {
        closeDevice();
        return false;
    }

    const std::filesystem::path folder(clipFolder);
    const std::string clipName = folder.filename().string();
    const std::filesystem::path outputPath = folder / (clipName + ".gcsv");
    outputPath_ = outputPath.string();

    running_.store(true, std::memory_order_release);
    thread_ = std::thread(&Mpu6050RecordingSession::run, this);

    std::cerr << "ApertarCore gyro recording: " << outputPath_
              << " driver=" << config_.driver
              << " sync_mode=" << config_.syncMode
              << " sync_offset_ms=" << config_.syncOffsetMs
              << " bus=" << config_.busNumber
              << " address=0x" << std::hex << config_.address << std::dec
              << " sample_rate=" << config_.sampleRateHz << "Hz\n";
    return true;
}

void Mpu6050RecordingSession::stop()
{
    running_.store(false, std::memory_order_release);
    if (thread_.joinable())
        thread_.join();

    if (fd_ >= 0 && fifoEnabled_) {
        writeRegister(kRegFifoEnable, 0x00);
        writeRegister(kRegUserControl, 0x00);
        fifoEnabled_ = false;
    }

    closeDevice();
}

void Mpu6050RecordingSession::setVideoStartTimestampUs(uint64_t videoStartTimestampUs,
                                                       uint64_t frameIndex)
{
    if (videoStartTimestampUs == 0)
        return;

    if (timelineAnchored_.load(std::memory_order_acquire))
        return;

    videoStartTimestampUs_.store(videoStartTimestampUs, std::memory_order_release);
    videoStartFrameIndex_.store(frameIndex, std::memory_order_release);
    timelineAnchored_.store(true, std::memory_order_release);
    std::cerr << "ApertarCore gyro timeline anchored: frame=" << frameIndex
              << " sensor_timestamp_us=" << videoStartTimestampUs << "\n";
}

bool Mpu6050RecordingSession::active() const
{
    return running_.load(std::memory_order_acquire);
}

std::string Mpu6050RecordingSession::lastError() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return lastError_;
}

bool Mpu6050RecordingSession::openDevice(std::string *error)
{
    const std::string devicePath = "/dev/i2c-" + std::to_string(config_.busNumber);
    driverKind_ = driverKindFromName(config_.driver);
    driverLabel_ = driverLabel(driverKind_);
    fd_ = ::open(devicePath.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
        if (probeConfiguredDevice())
            return true;
        setLastError("Could not open " + devicePath + ": " + std::strerror(errno));
        if (error)
            *error = lastError();
        return false;
    }

    if (::ioctl(fd_, I2C_SLAVE, config_.address) < 0) {
        if (probeConfiguredDevice())
            return true;
        setLastError("Could not select " + driverLabel_ + " address on " + devicePath + ": " + std::strerror(errno));
        if (error)
            *error = lastError();
        closeDevice();
        return false;
    }

    return true;
}

uint8_t Mpu6050RecordingSession::chipIdRegister() const
{
    switch (driverKind_) {
    case DriverKind::Bmi160:
        return kBmi160RegChipId;
    case DriverKind::Bmi270:
        return kBmi270RegChipId;
    case DriverKind::Icm42688p:
        return kIcm42688RegWhoAmI;
    case DriverKind::Mpu6050:
    default:
        return kRegWhoAmI;
    }
}

bool Mpu6050RecordingSession::chipIdMatches(uint8_t chipId) const
{
    switch (driverKind_) {
    case DriverKind::Bmi160:
        return chipId == kBmi160ChipId;
    case DriverKind::Bmi270:
        return chipId == kBmi270ChipId;
    case DriverKind::Icm42688p:
        return chipId == kIcm42688WhoAmI;
    case DriverKind::Mpu6050:
    default:
        return (chipId & 0x7e) == 0x68 || chipId == 0x70;
    }
}

bool Mpu6050RecordingSession::readChipId(uint8_t *chipId)
{
    if (!chipId)
        return false;
    return readRegisters(chipIdRegister(), chipId, 1);
}

bool Mpu6050RecordingSession::probeConfiguredDevice()
{
    const int originalBus = config_.busNumber;
    const int originalAddress = config_.address;
    closeDevice();

    std::vector<int> buses;
    const auto addBus = [&buses](int bus) {
        if (bus < 0 || bus > 32)
            return;
        if (std::find(buses.begin(), buses.end(), bus) == buses.end())
            buses.push_back(bus);
    };
    addBus(originalBus);
    for (int bus = 0; bus <= 32; ++bus) {
        if (std::filesystem::exists("/dev/i2c-" + std::to_string(bus)))
            addBus(bus);
    }

    std::vector<int> addresses;
    const auto addAddress = [&addresses](int address) {
        if (address < 0x03 || address > 0x77)
            return;
        if (std::find(addresses.begin(), addresses.end(), address) == addresses.end())
            addresses.push_back(address);
    };
    addAddress(originalAddress);
    addAddress(0x68);
    addAddress(0x69);

    for (const int bus : buses) {
        const std::string devicePath = "/dev/i2c-" + std::to_string(bus);
        for (const int address : addresses) {
            const int fd = ::open(devicePath.c_str(), O_RDWR | O_CLOEXEC);
            if (fd < 0)
                continue;
            if (::ioctl(fd, I2C_SLAVE, address) < 0) {
                ::close(fd);
                continue;
            }

            fd_ = fd;
            config_.busNumber = bus;
            config_.address = address;
            uint8_t chipId = 0;
            if (readChipId(&chipId) && chipIdMatches(chipId)) {
                logAutoProbeSuccessOnce(driverLabel_, bus, address, chipId);
                return true;
            }

            closeDevice();
        }
    }

    config_.busNumber = originalBus;
    config_.address = originalAddress;
    return false;
}

bool Mpu6050RecordingSession::ensureConfiguredDevice(std::string *error)
{
    uint8_t configuredChipId = 0;
    const bool configuredReadOk = readChipId(&configuredChipId);
    const int configuredReadError = errno;
    if (configuredReadOk && chipIdMatches(configuredChipId))
        return true;

    const int configuredBus = config_.busNumber;
    const int configuredAddress = config_.address;
    if (probeConfiguredDevice())
        return true;

    std::ostringstream message;
    message << driverLabel_ << " not found. Configured i2c-" << configuredBus
            << " address=0x" << std::hex << configuredAddress;
    if (configuredReadOk) {
        message << " returned chip_id=0x" << static_cast<int>(configuredChipId) << std::dec;
    } else {
        message << std::dec << " chip-ID read failed: " << std::strerror(configuredReadError);
    }
    message << ", and auto-probe did not find it at 0x68/0x69.";
    setLastError(message.str());
    if (error)
        *error = lastError();
    return false;
}

bool Mpu6050RecordingSession::configureSensor(std::string *error)
{
    driverKind_ = driverKindFromName(config_.driver);
    driverLabel_ = driverLabel(driverKind_);
    fifoEnabled_ = false;

    const auto fail = [this, error](const std::string &message) {
        setLastError(message);
        if (error)
            *error = lastError();
        return false;
    };

    if (!ensureConfiguredDevice(error))
        return false;

    if (driverKind_ == DriverKind::Bmi160) {
        uint8_t chipId = 0;
        if (!readRegisters(kBmi160RegChipId, &chipId, 1))
            return fail("BMI160 did not respond at configured I2C address.");
        if (chipId != kBmi160ChipId) {
            std::ostringstream message;
            message << "Unexpected BMI160 CHIP_ID value 0x"
                    << std::hex << static_cast<int>(chipId) << std::dec << ".";
            return fail(message.str());
        }

        if (!writeRegister(kBmi160RegCommand, 0x11))
            return fail("Could not enable BMI160 accelerometer.");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (!writeRegister(kBmi160RegCommand, 0x15))
            return fail("Could not enable BMI160 gyroscope.");
        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        const uint8_t odr = bmiOdrCode(config_.sampleRateHz);
        if (!writeRegister(kBmi160RegAccelConfig, static_cast<uint8_t>(0x20 | odr)) ||
            !writeRegister(kBmi160RegGyroConfig, static_cast<uint8_t>(0x20 | odr)))
            return fail("Could not configure BMI160 sample rate.");

        double gyroScale = 32.8;
        if (!writeRegister(kBmi160RegGyroRange, bmiGyroRangeValue(config_.gyroRangeDps, &gyroScale)))
            return fail("Could not configure BMI160 gyro range.");
        gyroScaleDpsPerLsb_ = gyroScale;

        double accelScale = 4096.0;
        if (!writeRegister(kBmi160RegAccelRange, bmi160AccelRangeValue(config_.accelRangeG, &accelScale)))
            return fail("Could not configure BMI160 accel range.");
        accelScaleGPerLsb_ = accelScale;

        samplePeriodUs_ = static_cast<uint32_t>(std::max<int64_t>(
            1,
            std::llround(1000000.0 / static_cast<double>(std::max(1, config_.sampleRateHz)))));
        return true;
    }

    if (driverKind_ == DriverKind::Bmi270) {
        uint8_t chipId = 0;
        if (!readRegisters(kBmi270RegChipId, &chipId, 1))
            return fail("BMI270 did not respond at configured I2C address.");
        if (chipId != kBmi270ChipId) {
            std::ostringstream message;
            message << "Unexpected BMI270 CHIP_ID value 0x"
                    << std::hex << static_cast<int>(chipId) << std::dec << ".";
            return fail(message.str());
        }

        writeRegister(kBmi270RegPowerConf, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (!writeRegister(kBmi270RegPowerCtrl, 0x0e))
            return fail("Could not enable BMI270 accelerometer/gyroscope.");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        const uint8_t odr = bmiOdrCode(config_.sampleRateHz);
        if (!writeRegister(kBmi270RegAccelConfig, static_cast<uint8_t>(0xa0 | odr)) ||
            !writeRegister(kBmi270RegGyroConfig, static_cast<uint8_t>(0xe0 | odr)))
            return fail("Could not configure BMI270 sample rate.");

        double gyroScale = 32.8;
        if (!writeRegister(kBmi270RegGyroRange, bmiGyroRangeValue(config_.gyroRangeDps, &gyroScale)))
            return fail("Could not configure BMI270 gyro range.");
        gyroScaleDpsPerLsb_ = gyroScale;

        double accelScale = 4096.0;
        if (!writeRegister(kBmi270RegAccelRange, bmi270AccelRangeValue(config_.accelRangeG, &accelScale)))
            return fail("Could not configure BMI270 accel range.");
        accelScaleGPerLsb_ = accelScale;

        uint8_t internalStatus = 0;
        if (readRegisters(kBmi270RegInternalStatus, &internalStatus, 1) && (internalStatus & 0x01) == 0) {
            std::cerr << "ApertarCore BMI270 warning: internal status 0x"
                      << std::hex << static_cast<int>(internalStatus) << std::dec
                      << " is not init-ok; this module may need the Bosch config blob loaded by its driver.\n";
        }

        samplePeriodUs_ = static_cast<uint32_t>(std::max<int64_t>(
            1,
            std::llround(1000000.0 / static_cast<double>(std::max(1, config_.sampleRateHz)))));
        return true;
    }

    if (driverKind_ == DriverKind::Icm42688p) {
        writeRegister(kIcm42688RegBankSelect, 0x00);
        uint8_t whoAmI = 0;
        if (!readRegisters(kIcm42688RegWhoAmI, &whoAmI, 1))
            return fail("ICM42688-P did not respond at configured I2C address.");
        if (whoAmI != kIcm42688WhoAmI) {
            std::ostringstream message;
            message << "Unexpected ICM42688-P WHO_AM_I value 0x"
                    << std::hex << static_cast<int>(whoAmI) << std::dec << ".";
            return fail(message.str());
        }

        if (!writeRegister(kIcm42688RegDeviceConfig, 0x01))
            return fail("Could not reset ICM42688-P.");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        writeRegister(kIcm42688RegBankSelect, 0x00);

        const uint8_t odr = icm42688OdrCode(config_.sampleRateHz);
        double gyroScale = 32.8;
        const uint8_t gyroConfig = static_cast<uint8_t>(
            icm42688GyroRangeBits(config_.gyroRangeDps, &gyroScale) | odr);
        if (!writeRegister(kIcm42688RegGyroConfig0, gyroConfig))
            return fail("Could not configure ICM42688-P gyro.");
        gyroScaleDpsPerLsb_ = gyroScale;

        double accelScale = 4096.0;
        const uint8_t accelConfig = static_cast<uint8_t>(
            icm42688AccelRangeBits(config_.accelRangeG, &accelScale) | odr);
        if (!writeRegister(kIcm42688RegAccelConfig0, accelConfig))
            return fail("Could not configure ICM42688-P accelerometer.");
        accelScaleGPerLsb_ = accelScale;

        if (!writeRegister(kIcm42688RegPowerMgmt0, 0x0f))
            return fail("Could not enable ICM42688-P accelerometer/gyroscope.");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        uint8_t status = 0;
        if (readRegisters(kIcm42688RegIntStatus, &status, 1) && (status & 0x08) == 0) {
            std::cerr << "ApertarCore ICM42688-P warning: data-ready bit has not asserted yet.\n";
        }

        samplePeriodUs_ = static_cast<uint32_t>(std::max<int64_t>(
            1,
            std::llround(1000000.0 / static_cast<double>(std::max(1, config_.sampleRateHz)))));
        return true;
    }

    uint8_t whoAmI = 0;
    if (!readRegisters(kRegWhoAmI, &whoAmI, 1)) {
        setLastError("MPU6050 did not respond at configured I2C address.");
        if (error)
            *error = lastError();
        return false;
    }

    if ((whoAmI & 0x7e) != 0x68 && whoAmI != 0x70) {
        std::ostringstream message;
        message << "Unexpected MPU6050 WHO_AM_I value 0x"
                << std::hex << static_cast<int>(whoAmI) << std::dec << ".";
        setLastError(message.str());
        if (error)
            *error = lastError();
        return false;
    }

    if (!writeRegister(kRegPwrMgmt1, 0x80)) {
        setLastError("Could not reset MPU6050.");
        if (error)
            *error = lastError();
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!writeRegister(kRegPwrMgmt1, 0x03)) {
        setLastError("Could not wake MPU6050.");
        if (error)
            *error = lastError();
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    const uint8_t dlpfConfig = dlpfConfigForHz(config_.dlpfHz);
    const int baseRateHz = dlpfConfig == 0 ? 8000 : 1000;
    const int requestedRateHz = std::max(1, config_.sampleRateHz);
    const int divisor = std::clamp((baseRateHz + (requestedRateHz / 2)) / requestedRateHz - 1, 0, 255);
    const int effectiveRateHz = std::max(1, baseRateHz / (divisor + 1));
    samplePeriodUs_ = static_cast<uint32_t>(std::max<int64_t>(
        1,
        std::llround(1000000.0 / static_cast<double>(effectiveRateHz))));

    if (!writeRegister(kRegConfig, dlpfConfig) ||
        !writeRegister(kRegSampleRateDivider, static_cast<uint8_t>(divisor))) {
        setLastError("Could not configure MPU6050 sample rate.");
        if (error)
            *error = lastError();
        return false;
    }

    double gyroScale = 32.8;
    const uint8_t gyroBits = gyroRangeBits(config_.gyroRangeDps, &gyroScale);
    if (!writeRegister(kRegGyroConfig, static_cast<uint8_t>(gyroBits << 3))) {
        setLastError("Could not configure MPU6050 gyro range.");
        if (error)
            *error = lastError();
        return false;
    }
    gyroScaleDpsPerLsb_ = gyroScale;

    double accelScale = 4096.0;
    const uint8_t accelBits = accelRangeBits(config_.accelRangeG, &accelScale);
    if (!writeRegister(kRegAccelConfig, static_cast<uint8_t>(accelBits << 3))) {
        setLastError("Could not configure MPU6050 accel range.");
        if (error)
            *error = lastError();
        return false;
    }
    accelScaleGPerLsb_ = accelScale;

    if (config_.useFifo) {
        if (resetFifo()) {
            fifoEnabled_ = true;
            std::cerr << "ApertarCore gyro FIFO enabled: effective_rate="
                      << effectiveRateHz << "Hz batch=" << config_.fifoBatchSamples << "\n";
        } else {
            std::cerr << "ApertarCore gyro FIFO unavailable; falling back to direct polling.\n";
            writeRegister(kRegFifoEnable, 0x00);
            writeRegister(kRegUserControl, 0x00);
        }
    }

    return true;
}

bool Mpu6050RecordingSession::writeRegister(uint8_t reg, uint8_t value)
{
    const uint8_t buffer[2] = { reg, value };
    return ::write(fd_, buffer, sizeof(buffer)) == static_cast<ssize_t>(sizeof(buffer));
}

bool Mpu6050RecordingSession::readRegisters(uint8_t reg, uint8_t *data, size_t length)
{
    if (!data || length == 0 || length > std::numeric_limits<uint16_t>::max()) {
        errno = EINVAL;
        return false;
    }

    // Keep register selection and reading atomic, including across other clients.
    i2c_msg messages[2]{};
    messages[0].addr = static_cast<uint16_t>(config_.address);
    messages[0].len = 1;
    messages[0].buf = &reg;
    messages[1].addr = messages[0].addr;
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<uint16_t>(length);
    messages[1].buf = data;
    i2c_rdwr_ioctl_data transaction{messages, 2};
    const int result = ::ioctl(fd_, I2C_RDWR, &transaction);
    if (result == 2)
        return true;
    if (result >= 0)
        errno = EIO;
    return false;
}

bool Mpu6050RecordingSession::readLiveSample(LiveSample *sample)
{
    if (!sample)
        return false;

    double gyroScale = 32.8;
    double accelScale = 4096.0;
    sensorScalesForConfig(driverKind_,
                          config_.gyroRangeDps,
                          config_.accelRangeG,
                          &gyroScale,
                          &accelScale);

    bool ok = false;
    if (driverKind_ == DriverKind::Bmi160) {
        uint8_t data[12]{};
        ok = readRegisters(kBmi160RegGyroData, data, sizeof(data));
        if (ok) {
            sample->gyroXDps = le16(data + 0) / gyroScale;
            sample->gyroYDps = le16(data + 2) / gyroScale;
            sample->gyroZDps = le16(data + 4) / gyroScale;
            sample->accelXG = le16(data + 6) / accelScale;
            sample->accelYG = le16(data + 8) / accelScale;
            sample->accelZG = le16(data + 10) / accelScale;
        }
    } else if (driverKind_ == DriverKind::Bmi270) {
        uint8_t data[12]{};
        ok = readRegisters(kBmi270RegAccelData, data, sizeof(data));
        if (ok) {
            sample->accelXG = le16(data + 0) / accelScale;
            sample->accelYG = le16(data + 2) / accelScale;
            sample->accelZG = le16(data + 4) / accelScale;
            sample->gyroXDps = le16(data + 6) / gyroScale;
            sample->gyroYDps = le16(data + 8) / gyroScale;
            sample->gyroZDps = le16(data + 10) / gyroScale;
        }
    } else if (driverKind_ == DriverKind::Icm42688p) {
        uint8_t data[12]{};
        ok = readRegisters(kIcm42688RegAccelData, data, sizeof(data));
        if (ok) {
            sample->accelXG = be16(data + 0) / accelScale;
            sample->accelYG = be16(data + 2) / accelScale;
            sample->accelZG = be16(data + 4) / accelScale;
            sample->gyroXDps = be16(data + 6) / gyroScale;
            sample->gyroYDps = be16(data + 8) / gyroScale;
            sample->gyroZDps = be16(data + 10) / gyroScale;
        }
    } else {
        uint8_t data[14]{};
        ok = readRegisters(kRegAccelXoutH, data, sizeof(data));
        if (ok) {
            sample->accelXG = be16(data + 0) / accelScale;
            sample->accelYG = be16(data + 2) / accelScale;
            sample->accelZG = be16(data + 4) / accelScale;
            sample->gyroXDps = be16(data + 8) / gyroScale;
            sample->gyroYDps = be16(data + 10) / gyroScale;
            sample->gyroZDps = be16(data + 12) / gyroScale;
        }
    }

    if (!ok)
        return false;

    sample->statusText = driverLabel_ + " live 0x" + [&] {
        std::ostringstream out;
        out << std::hex << config_.address;
        return out.str();
    }() + " on i2c-" + std::to_string(config_.busNumber);
    return true;
}

bool Mpu6050RecordingSession::resetFifo()
{
    if (!writeRegister(kRegFifoEnable, 0x00))
        return false;
    if (!writeRegister(kRegUserControl, kUserControlFifoReset))
        return false;

    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    if (!writeRegister(kRegUserControl, kUserControlFifoEnable))
        return false;
    return writeRegister(kRegFifoEnable, kFifoEnableAccelGyro);
}

void Mpu6050RecordingSession::run()
{
    struct Sample {
        uint64_t epochUs = 0;
        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;
        int16_t ax = 0;
        int16_t ay = 0;
        int16_t az = 0;
    };

    std::vector<Sample> recordedSamples;
    recordedSamples.reserve(static_cast<size_t>(std::max(1, config_.sampleRateHz)) * 60u);
    std::vector<uint8_t> fifoData;
    fifoData.resize(static_cast<size_t>(std::max(1, config_.fifoBatchSamples)) * kFifoPacketBytes);

    const size_t calibrationTarget = config_.autoCalibrateSeconds > 0.0
        ? static_cast<size_t>(std::max(1.0, std::round(config_.autoCalibrateSeconds * config_.sampleRateHz)))
        : 0u;
    std::vector<Sample> calibrationSamples;
    calibrationSamples.reserve(calibrationTarget);
    double biasX = config_.gyroBiasXRaw;
    double biasY = config_.gyroBiasYRaw;
    double biasZ = config_.gyroBiasZRaw;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;
    bool calibrated = calibrationTarget == 0u;

    const auto appendCorrectedSample = [&](const Sample &sample,
                                           double biasX,
                                           double biasY,
                                           double biasZ) {
        Sample corrected = sample;
        corrected.gx = clampToInt16(static_cast<double>(sample.gx) - biasX);
        corrected.gy = clampToInt16(static_cast<double>(sample.gy) - biasY);
        corrected.gz = clampToInt16(static_cast<double>(sample.gz) - biasZ);
        recordedSamples.push_back(corrected);
    };

    const auto processSample = [&](const Sample &sample) {
        if (!calibrated) {
            calibrationSamples.push_back(sample);
            sumX += sample.gx;
            sumY += sample.gy;
            sumZ += sample.gz;

            if (calibrationSamples.size() >= calibrationTarget) {
                biasX += sumX / static_cast<double>(calibrationSamples.size());
                biasY += sumY / static_cast<double>(calibrationSamples.size());
                biasZ += sumZ / static_cast<double>(calibrationSamples.size());
                calibrated = true;
                std::cerr << "ApertarCore gyro auto bias raw: "
                          << biasX << ", " << biasY << ", " << biasZ << "\n";
                for (const Sample &pending : calibrationSamples)
                    appendCorrectedSample(pending, biasX, biasY, biasZ);
                calibrationSamples.clear();
            }
        } else {
            appendCorrectedSample(sample, biasX, biasY, biasZ);
        }
    };

    bool fifoClockAnchored = false;
    uint64_t nextFifoSampleUs = 0;
    uint64_t lastSampleEpochUs = 0;
    uint64_t fifoOverflows = 0;
    uint64_t fifoMisalignments = 0;
    uint64_t fifoReadFailures = 0;

    while (running_.load(std::memory_order_acquire)) {
        if (fifoEnabled_) {
            uint8_t countBytes[2]{};
            if (!readRegisters(kRegFifoCountH, countBytes, sizeof(countBytes))) {
                setLastError("MPU6050 FIFO count read failed.");
                ++fifoReadFailures;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            const uint16_t fifoCount = static_cast<uint16_t>(
                (static_cast<uint16_t>(countBytes[0]) << 8) | countBytes[1]);
            if (fifoCount >= kFifoCapacityBytes - kFifoPacketBytes) {
                setLastError("MPU6050 FIFO overflow; samples were dropped.");
                ++fifoOverflows;
                std::cerr << "ApertarCore gyro FIFO overflow; resetting FIFO.\n";
                resetFifo();
                fifoClockAnchored = false;
                nextFifoSampleUs = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            const size_t partialFifoBytes = fifoCount % kFifoPacketBytes;
            if (partialFifoBytes != 0)
                ++fifoMisalignments;

            const size_t availablePackets = (fifoCount - partialFifoBytes) / kFifoPacketBytes;
            if (availablePackets == 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(std::max<uint32_t>(500, samplePeriodUs_ / 2)));
                continue;
            }

            const size_t packetsToRead = std::min<size_t>(
                availablePackets,
                static_cast<size_t>(std::max(1, config_.fifoBatchSamples)));
            if (!readRegisters(kRegFifoRw, fifoData.data(), packetsToRead * kFifoPacketBytes)) {
                setLastError("MPU6050 FIFO sample read failed.");
                ++fifoReadFailures;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            const uint64_t readCompleteUs = monotonicClockUs();

            if (!fifoClockAnchored) {
                const uint64_t fifoSpanUs = static_cast<uint64_t>(availablePackets - 1) * samplePeriodUs_;
                nextFifoSampleUs = readCompleteUs > fifoSpanUs
                    ? readCompleteUs - fifoSpanUs
                    : readCompleteUs;
                if (lastSampleEpochUs > 0 && nextFifoSampleUs <= lastSampleEpochUs)
                    nextFifoSampleUs = lastSampleEpochUs + samplePeriodUs_;
                fifoClockAnchored = true;
            }

            for (size_t packet = 0; packet < packetsToRead; ++packet) {
                const uint8_t *data = fifoData.data() + (packet * kFifoPacketBytes);
                Sample sample;
                sample.epochUs = nextFifoSampleUs;
                nextFifoSampleUs += samplePeriodUs_;
                if (lastSampleEpochUs > 0 && sample.epochUs <= lastSampleEpochUs) {
                    sample.epochUs = lastSampleEpochUs + samplePeriodUs_;
                    nextFifoSampleUs = sample.epochUs + samplePeriodUs_;
                }
                lastSampleEpochUs = sample.epochUs;
                sample.ax = be16(data + 0);
                sample.ay = be16(data + 2);
                sample.az = be16(data + 4);
                sample.gx = be16(data + 6);
                sample.gy = be16(data + 8);
                sample.gz = be16(data + 10);
                processSample(sample);
            }

            continue;
        }

        bool sampleRead = false;
        Sample sample;
        if (driverKind_ == DriverKind::Bmi160) {
            uint8_t data[12]{};
            sampleRead = readRegisters(kBmi160RegGyroData, data, sizeof(data));
            if (sampleRead) {
                sample.gx = le16(data + 0);
                sample.gy = le16(data + 2);
                sample.gz = le16(data + 4);
                sample.ax = le16(data + 6);
                sample.ay = le16(data + 8);
                sample.az = le16(data + 10);
            }
        } else if (driverKind_ == DriverKind::Bmi270) {
            uint8_t data[12]{};
            sampleRead = readRegisters(kBmi270RegAccelData, data, sizeof(data));
            if (sampleRead) {
                sample.ax = le16(data + 0);
                sample.ay = le16(data + 2);
                sample.az = le16(data + 4);
                sample.gx = le16(data + 6);
                sample.gy = le16(data + 8);
                sample.gz = le16(data + 10);
            }
        } else if (driverKind_ == DriverKind::Icm42688p) {
            uint8_t data[12]{};
            sampleRead = readRegisters(kIcm42688RegAccelData, data, sizeof(data));
            if (sampleRead) {
                sample.ax = be16(data + 0);
                sample.ay = be16(data + 2);
                sample.az = be16(data + 4);
                sample.gx = be16(data + 6);
                sample.gy = be16(data + 8);
                sample.gz = be16(data + 10);
            }
        } else {
            uint8_t data[14]{};
            sampleRead = readRegisters(kRegAccelXoutH, data, sizeof(data));
            if (sampleRead) {
                sample.ax = be16(data + 0);
                sample.ay = be16(data + 2);
                sample.az = be16(data + 4);
                sample.gx = be16(data + 8);
                sample.gy = be16(data + 10);
                sample.gz = be16(data + 12);
            }
        }

        if (sampleRead) {
            sample.epochUs = monotonicClockUs();
            if (lastSampleEpochUs > 0 && sample.epochUs <= lastSampleEpochUs)
                sample.epochUs = lastSampleEpochUs + samplePeriodUs_;
            lastSampleEpochUs = sample.epochUs;
            processSample(sample);
        } else {
            setLastError(driverLabel_ + " sample read failed.");
        }

        std::this_thread::sleep_for(std::chrono::microseconds(std::max<uint32_t>(500, samplePeriodUs_)));
    }

    if (!calibrated && !calibrationSamples.empty()) {
        biasX += sumX / static_cast<double>(calibrationSamples.size());
        biasY += sumY / static_cast<double>(calibrationSamples.size());
        biasZ += sumZ / static_cast<double>(calibrationSamples.size());
        for (const Sample &pending : calibrationSamples)
            appendCorrectedSample(pending, biasX, biasY, biasZ);
        calibrationSamples.clear();
    }

    if (recordedSamples.empty()) {
        std::ofstream emptyOutput(outputPath_, std::ios::out | std::ios::trunc);
        if (!emptyOutput.is_open()) {
            setLastError("Could not create empty gyro sidecar " + outputPath_);
            return;
        }
    } else if (!timelineAnchored_.load(std::memory_order_acquire)) {
        videoStartTimestampUs_.store(recordedSamples.front().epochUs, std::memory_order_release);
        videoStartFrameIndex_.store(0, std::memory_order_release);
        timelineAnchored_.store(true, std::memory_order_release);
        std::cerr << "ApertarCore gyro timeline fallback: no video frame anchor received.\n";
    }

    uint64_t anchorUs = videoStartTimestampUs_.load(std::memory_order_acquire);
    if (!recordedSamples.empty()) {
        const uint64_t firstSampleUs = recordedSamples.front().epochUs;
        const uint64_t lastSampleUs = recordedSamples.back().epochUs;
        constexpr uint64_t kAnchorPadUs = 5000000;
        const bool anchorLooksValid =
            anchorUs > 0 &&
            anchorUs + kAnchorPadUs >= firstSampleUs &&
            anchorUs <= lastSampleUs + kAnchorPadUs;
        if (!anchorLooksValid) {
            std::cerr << "ApertarCore gyro timeline fallback: video anchor was outside gyro sample range.\n";
            anchorUs = firstSampleUs;
        }
    }

    const int64_t syncOffsetUs = static_cast<int64_t>(std::llround(config_.syncOffsetMs * 1000.0));
    int64_t exportAnchorUs = static_cast<int64_t>(std::min<uint64_t>(
        anchorUs,
        static_cast<uint64_t>(std::numeric_limits<int64_t>::max())));
    exportAnchorUs = std::max<int64_t>(0, exportAnchorUs + syncOffsetUs);

    const std::filesystem::path outputPath(outputPath_);
    const std::string clipName = outputPath.stem().string();
    std::ofstream output(outputPath, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        setLastError("Could not create gyro sidecar " + outputPath.string());
        return;
    }

    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double gyroScale = kDegToRad / gyroScaleDpsPerLsb_;
    const double accelScale = 1.0 / accelScaleGPerLsb_;

    output << "GYROFLOW IMU LOG\n"
           << "version,1.3\n"
           << "id,Apertar " << driverLabel_ << "\n"
           << "orientation," << config_.orientation << "\n"
           << "note," << driverLabel_ << " sidecar recorded by ApertarCore; sync_mode="
           << config_.syncMode << "; sync_offset_ms="
           << std::fixed << std::setprecision(3) << config_.syncOffsetMs << "\n"
           << "videofilename," << clipName << "\n";
    if (config_.frameReadoutTimeMs > 0.0)
        output << "frame_readout_time," << std::fixed << std::setprecision(3)
               << config_.frameReadoutTimeMs << "\n";
    output << "frame_readout_direction," << config_.frameReadoutDirection << "\n"
           << "tscale,0.000001\n"
           << "gscale," << std::setprecision(12) << gyroScale << "\n"
           << "ascale," << std::setprecision(12) << accelScale << "\n"
           << "t,gx,gy,gz,ax,ay,az\n";

    uint64_t exportedSamples = 0;
    for (const Sample &sample : recordedSamples) {
        const int64_t relativeUs = static_cast<int64_t>(std::min<uint64_t>(
            sample.epochUs,
            static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))) - exportAnchorUs;
        if (relativeUs < 0)
            continue;
        output << relativeUs << ','
               << sample.gx << ','
               << sample.gy << ','
               << sample.gz << ','
               << sample.ax << ','
               << sample.ay << ','
               << sample.az << '\n';
        ++exportedSamples;
    }
    output.flush();

    std::cerr << "ApertarCore gyro sidecar exported: " << outputPath_
              << " samples=" << exportedSamples
              << " anchor_us=" << anchorUs
              << " export_anchor_us=" << exportAnchorUs
              << " sync_offset_ms=" << config_.syncOffsetMs
              << " fifo_overflows=" << fifoOverflows
              << " fifo_misalignments=" << fifoMisalignments
              << " fifo_read_failures=" << fifoReadFailures << "\n";
}

void Mpu6050RecordingSession::closeDevice()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void Mpu6050RecordingSession::setLastError(const std::string &error)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    lastError_ = error;
}

} // namespace apertar
