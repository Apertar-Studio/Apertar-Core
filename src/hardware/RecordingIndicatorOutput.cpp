#include "hardware/RecordingIndicatorOutput.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <vector>

#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace apertar {
namespace {

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
    value = lowerAscii(trimAscii(value));
    for (char &ch : value) {
        if (ch == '-' || ch == ' ' || ch == '.')
            ch = '_';
    }
    return value;
}

std::string stripInlineComment(std::string value)
{
    for (size_t i = 0; i < value.size(); ++i) {
        if ((value[i] == '#' || value[i] == ';') &&
            (i == 0 || std::isspace(static_cast<unsigned char>(value[i - 1])))) {
            value.resize(i);
            break;
        }
    }
    return trimAscii(std::move(value));
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

std::optional<int> parseConfigInteger(const std::string &text)
{
    const std::string trimmed = trimAscii(text);
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

bool parseConfigBool(const std::string &text, bool fallback)
{
    const std::string normalized = normalizeToken(text);
    if (normalized == "true" || normalized == "yes" || normalized == "on" || normalized == "1")
        return true;
    if (normalized == "false" || normalized == "no" || normalized == "off" || normalized == "0")
        return false;
    return fallback;
}

bool isRecordingIndicatorAction(const std::string &text)
{
    const std::string action = normalizeToken(text);
    return action == "recindicatorled" ||
           action == "rec_indicator_led" ||
           action == "recording_indicator" ||
           action == "recording_indicator_led" ||
           action == "record_led" ||
           action == "rec_led" ||
           action == "tally" ||
           action == "tally_led";
}

std::optional<int> pcfPinIndexForKey(std::string key)
{
    key = normalizeToken(std::move(key));
    if (key.size() > 7 && key.substr(key.size() - 7) == "_action")
        key.resize(key.size() - 7);
    if (key.size() > 9 && key.substr(key.size() - 9) == "_function")
        key.resize(key.size() - 9);

    std::string digits;
    if (key.rfind("pin_", 0) == 0)
        digits = key.substr(4);
    else if (key.rfind("pin", 0) == 0)
        digits = key.substr(3);
    else if (key.rfind("gpio_", 0) == 0)
        digits = key.substr(5);
    else if (key.rfind("gpio", 0) == 0)
        digits = key.substr(4);
    else if (key.size() == 2 && key[0] == 'p' && std::isdigit(static_cast<unsigned char>(key[1])))
        digits = key.substr(1);

    if (digits.empty() || !std::all_of(digits.begin(), digits.end(), [](unsigned char ch) {
            return std::isdigit(ch);
        })) {
        return std::nullopt;
    }

    const int pin = std::stoi(digits);
    if (pin < 0 || pin > 7)
        return std::nullopt;
    return pin;
}

std::optional<int> gpioFromDigitalKey(std::string key)
{
    key = normalizeToken(std::move(key));
    if (key.size() > 7 && key.substr(key.size() - 7) == "_action")
        key.resize(key.size() - 7);
    if (key.size() > 9 && key.substr(key.size() - 9) == "_function")
        key.resize(key.size() - 9);

    std::string digits;
    for (const std::string prefix : { "gpio_", "gpio", "pin_", "pin", "bcm_", "bcm", "d_", "d",
                                     "grove_d_", "grove_d" }) {
        if (key.rfind(prefix, 0) == 0) {
            digits = key.substr(prefix.size());
            break;
        }
    }

    if (digits.empty() || !std::all_of(digits.begin(), digits.end(), [](unsigned char ch) {
            return std::isdigit(ch);
        })) {
        return std::nullopt;
    }

    const int gpio = std::stoi(digits);
    if (gpio < 0 || gpio > 63)
        return std::nullopt;
    return gpio;
}

bool gpioChipPathIsAuto(const std::string &path)
{
    const std::string normalized = normalizeToken(path);
    return normalized.empty() || normalized == "auto";
}

int gpioChipScore(const std::string &chipPath)
{
    const int fd = ::open(chipPath.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        return -1;

    gpiochip_info info{};
    int score = 0;
    if (::ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &info) == 0) {
        const std::string name = normalizeToken(info.name);
        const std::string label = normalizeToken(info.label);
        const std::string combined = name + " " + label;
        if (combined.find("rp1") != std::string::npos)
            score += 100;
        if (combined.find("pinctrl_bcm") != std::string::npos ||
            combined.find("bcm2711") != std::string::npos ||
            combined.find("bcm2835") != std::string::npos) {
            score += 90;
        }
        if (combined.find("raspberrypi_exp") != std::string::npos)
            score -= 50;
        if (info.lines >= 28)
            score += 10;
    }

    ::close(fd);
    return score;
}

int outputPhysicalValue(bool active, bool activeHigh)
{
    return active == activeHigh ? 1 : 0;
}

void closeFd(int *fd)
{
    if (fd && *fd >= 0) {
        ::close(*fd);
        *fd = -1;
    }
}

} // namespace

RecordingIndicatorOutput::~RecordingIndicatorOutput()
{
    stop();
}

RecordingIndicatorOutput::Config RecordingIndicatorOutput::loadConfig()
{
    Config config;
    std::error_code ec;

    for (const auto &path : hardwareConfigCandidatePaths()) {
        if (!std::filesystem::exists(path, ec) || !std::filesystem::is_regular_file(path, ec))
            continue;

        std::ifstream input(path);
        if (!input.is_open())
            continue;

        config.configPath = path.string();
        std::string section;
        bool hasGenericPin = false;
        int genericPin = -1;
        std::optional<Config> actionConfig;

        std::string line;
        while (std::getline(input, line)) {
            std::string trimmed = trimAscii(line);
            if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';')
                continue;

            if (trimmed.front() == '[' && trimmed.back() == ']') {
                section = normalizeToken(trimmed.substr(1, trimmed.size() - 2));
                if (section == "rec_indicator" || section == "record_led" ||
                    section == "recording_led" || section == "tally_led") {
                    section = "recording_indicator";
                }
                if (section == "recording_indicator") {
                    config.configured = true;
                    hasGenericPin = false;
                    genericPin = -1;
                }
                continue;
            }

            const size_t equals = trimmed.find('=');
            if (equals == std::string::npos)
                continue;

            const std::string key = normalizeToken(trimmed.substr(0, equals));
            const std::string value = stripInlineComment(trimmed.substr(equals + 1));

            if (section == "pcf8574" || section == "pcf8574_buttons") {
                if (key == "bus" || key == "channel" || key == "i2c_bus") {
                    if (auto parsed = parseConfigInteger(value))
                        config.pcfBusNumber = *parsed;
                    if (actionConfig)
                        actionConfig->pcfBusNumber = config.pcfBusNumber;
                } else if (key == "address" || key == "addr") {
                    if (auto parsed = parseConfigInteger(value))
                        config.pcfAddress = *parsed;
                    if (actionConfig)
                        actionConfig->pcfAddress = config.pcfAddress;
                } else if (const auto pin = pcfPinIndexForKey(key);
                           pin && isRecordingIndicatorAction(value) && !actionConfig) {
                    Config detected = config;
                    detected.configured = true;
                    detected.enabled = true;
                    detected.target = Target::Pcf8574;
                    detected.pcfPin = *pin;
                    detected.source = "pcf8574 pin" + std::to_string(*pin);
                    actionConfig = detected;
                }
                continue;
            }

            if (section == "gpio_buttons" || section == "gpio" ||
                section == "grove_digital" || section == "grove_buttons") {
                if (key == "chip" || key == "gpiochip" || key == "gpio_chip") {
                    config.gpioChipPath = value.empty() ? "auto" : value;
                    if (actionConfig && actionConfig->target == Target::Gpio)
                        actionConfig->gpioChipPath = config.gpioChipPath;
                } else if (const auto gpio = gpioFromDigitalKey(key);
                           gpio && isRecordingIndicatorAction(value) && !actionConfig) {
                    Config detected = config;
                    detected.configured = true;
                    detected.enabled = true;
                    detected.target = Target::Gpio;
                    detected.gpioLine = *gpio;
                    detected.source = "GPIO " + std::to_string(*gpio);
                    actionConfig = detected;
                }
                continue;
            }

            if (section != "recording_indicator")
                continue;

            if (key == "enabled") {
                config.enabled = parseConfigBool(value, config.enabled);
            } else if (key == "target" || key == "type" || key == "backend" || key == "output") {
                const std::string target = normalizeToken(value);
                if (target == "pcf" || target == "pcf8574" || target == "i2c")
                    config.target = Target::Pcf8574;
                else if (target == "gpio" || target == "pin" || target == "raspberry_pi_gpio")
                    config.target = Target::Gpio;
            } else if (key == "chip" || key == "gpiochip" || key == "gpio_chip") {
                config.gpioChipPath = value.empty() ? "auto" : value;
            } else if (key == "line" || key == "gpio" || key == "gpio_line") {
                if (auto parsed = parseConfigInteger(value))
                    config.gpioLine = *parsed;
            } else if (key == "pin") {
                if (auto parsed = parseConfigInteger(value)) {
                    genericPin = *parsed;
                    hasGenericPin = true;
                }
            } else if (key == "open_drain" || key == "gpio_open_drain") {
                config.gpioOpenDrain = parseConfigBool(value, config.gpioOpenDrain);
            } else if (key == "bus" || key == "pcf_bus") {
                if (auto parsed = parseConfigInteger(value))
                    config.pcfBusNumber = *parsed;
            } else if (key == "address" || key == "pcf_address") {
                if (auto parsed = parseConfigInteger(value))
                    config.pcfAddress = *parsed;
            } else if (key == "pcf_pin" || key == "pcf8574_pin") {
                if (auto parsed = parseConfigInteger(value))
                    config.pcfPin = *parsed;
            } else if (key == "active_high") {
                config.activeHigh = parseConfigBool(value, config.activeHigh);
            } else if (key == "active_low") {
                config.activeHigh = !parseConfigBool(value, !config.activeHigh);
            }
        }

        if (hasGenericPin) {
            if (config.target == Target::Pcf8574)
                config.pcfPin = genericPin;
            else
                config.gpioLine = genericPin;
        }

        if (config.configured && config.enabled) {
            config.source = config.target == Target::Pcf8574
                ? "recording_indicator pcf8574 pin" + std::to_string(config.pcfPin)
                : "recording_indicator GPIO " + std::to_string(config.gpioLine);
            return config;
        }

        if (actionConfig)
            return *actionConfig;

        if (config.configured)
            return config;

        return config;
    }

    return config;
}

bool RecordingIndicatorOutput::start()
{
    stop();

    const Config config = loadConfig();
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        configPath_ = config.configPath;
        lastError_.clear();
    }

    if (!config.configured || !config.enabled)
        return true;

    std::string error;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        target_ = config.target;
        gpioChipPath_ = config.gpioChipPath;
        gpioLine_ = config.gpioLine;
        gpioOpenDrain_ = config.gpioOpenDrain;
        pcfBusNumber_ = config.pcfBusNumber;
        pcfAddress_ = config.pcfAddress;
        pcfPin_ = config.pcfPin;
        activeHigh_ = config.activeHigh;
        source_ = config.source;
        active_ = false;

        if (!openOutputLocked(&error) || !setOutputActiveLocked(false, &error)) {
            closeOutputLocked();
            enabled_ = false;
            setLastErrorLocked(error);
            return false;
        }
        enabled_ = true;
        running_ = true;
        try {
            worker_ = std::thread(&RecordingIndicatorOutput::workerLoop, this);
        } catch (const std::exception &e) {
            running_ = false;
            enabled_ = false;
            closeOutputLocked();
            setLastErrorLocked(std::string("Could not start recording indicator timer: ") + e.what());
            return false;
        }
    }

    const char *targetText = config.target == Target::Pcf8574 ? "pcf8574" : "gpio";
    std::cerr << "ApertarCore recording indicator LED enabled: target=" << targetText
              << " active_high=" << (config.activeHigh ? "true" : "false");
    if (config.target == Target::Pcf8574) {
        std::cerr << " pcf=i2c-" << config.pcfBusNumber
                  << "/0x" << std::hex << config.pcfAddress << std::dec
                  << " pin=" << config.pcfPin;
    } else {
        std::string openedGpioChipPath;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            openedGpioChipPath = gpioChipPath_;
        }
        std::cerr << " gpio=" << openedGpioChipPath
                  << " line=" << config.gpioLine
                  << " open_drain=" << (config.gpioOpenDrain ? "true" : "false");
    }
    std::cerr << "\n";
    return true;
}

void RecordingIndicatorOutput::stop()
{
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        running_ = false;
    }
    cv_.notify_all();
    if (worker_.joinable())
        worker_.join();

    std::lock_guard<std::mutex> lock(stateMutex_);
    std::string ignored;
    setOutputActiveLocked(false, &ignored);
    closeOutputLocked();
    enabled_ = false;
    active_ = false;
    outputDirty_ = false;
    recordingActive_ = false;
    stillExposureActive_ = false;
    capturePulseUntil_ = {};
}

void RecordingIndicatorOutput::setRecordingActive(bool active)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (!enabled_ || !running_)
        return;
    recordingActive_ = active;
    outputDirty_ = true;
    cv_.notify_one();
}

void RecordingIndicatorOutput::setStillExposureActive(bool active)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (!enabled_ || !running_)
        return;
    stillExposureActive_ = active;
    if (active) {
        capturePulseUntil_ = {};
        cv_.notify_one();
    }
    outputDirty_ = true;
    cv_.notify_one();
}

void RecordingIndicatorOutput::pulseCapture()
{
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (!enabled_ || !running_ || stillExposureActive_)
            return;
        // Match the photo/timelapse screen flash without blocking capture.
        capturePulseUntil_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(320);
        outputDirty_ = true;
    }
    cv_.notify_one();
}

void RecordingIndicatorOutput::workerLoop()
{
    std::unique_lock<std::mutex> lock(stateMutex_);
    while (running_) {
        if (outputDirty_) {
            outputDirty_ = false;
            const bool active = recordingActive_ || stillExposureActive_ ||
                std::chrono::steady_clock::now() < capturePulseUntil_;
            if (active != active_) {
                std::string error;
                // Only this worker uses the output while running. In particular,
                // a slow I2C transfer must not hold a capture callback's state lock.
                lock.unlock();
                const bool written = setOutputActiveLocked(active, &error);
                lock.lock();
                if (written) active_ = active;
                else setLastErrorLocked(error);
            }
            continue;
        }
        if (capturePulseUntil_ == std::chrono::steady_clock::time_point{}) {
            cv_.wait(lock, [this] { return !running_ || outputDirty_; });
            continue;
        }
        const auto deadline = capturePulseUntil_;
        if (cv_.wait_until(lock, deadline, [this, deadline] {
                return !running_ || outputDirty_ || capturePulseUntil_ != deadline;
            }))
            continue;
        capturePulseUntil_ = {};
        outputDirty_ = true;
    }
}

bool RecordingIndicatorOutput::enabled() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return enabled_;
}

std::string RecordingIndicatorOutput::configPath() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return configPath_;
}

std::string RecordingIndicatorOutput::lastError() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return lastError_;
}

bool RecordingIndicatorOutput::openOutputLocked(std::string *errorMessage)
{
    if (target_ == Target::Pcf8574)
        return openPcf8574Locked(errorMessage);
    return openGpioLocked(errorMessage);
}

bool RecordingIndicatorOutput::openGpioLocked(std::string *errorMessage)
{
    if (gpioLine_ < 0) {
        if (errorMessage)
            *errorMessage = "Recording indicator GPIO line must be >= 0.";
        return false;
    }

    const std::string configuredChipPath = trimAscii(gpioChipPath_);
    if (!gpioChipPathIsAuto(configuredChipPath))
        return requestGpioLineFromChipLocked(configuredChipPath, errorMessage);

    std::string lastError;
    std::vector<std::pair<int, std::string>> candidates;
    for (int chip = 0; chip <= 9; ++chip) {
        const std::string path = "/dev/gpiochip" + std::to_string(chip);
        std::error_code ec;
        if (!std::filesystem::exists(path, ec))
            continue;
        const int score = gpioChipScore(path);
        if (score >= 0)
            candidates.emplace_back(score, path);
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto &left, const auto &right) {
        return left.first > right.first;
    });

    for (const auto &candidate : candidates) {
        std::string error;
        if (requestGpioLineFromChipLocked(candidate.second, &error))
            return true;
        if (!error.empty())
            lastError = error;
    }

    if (errorMessage)
        *errorMessage = lastError.empty()
            ? "No usable /dev/gpiochip found for recording indicator GPIO line"
            : lastError;
    return false;
}

bool RecordingIndicatorOutput::requestGpioLineFromChipLocked(const std::string &chipPath,
                                                             std::string *errorMessage)
{
    const int chipFd = ::open(chipPath.c_str(), O_RDONLY | O_CLOEXEC);
    if (chipFd < 0) {
        if (errorMessage)
            *errorMessage = "Could not open " + chipPath + ": " + std::strerror(errno);
        return false;
    }

    gpio_v2_line_request request{};
    request.offsets[0] = static_cast<__u32>(gpioLine_);
    request.num_lines = 1;
    std::snprintf(request.consumer, sizeof(request.consumer), "apertar-rec-led");
    request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    if (gpioOpenDrain_)
        request.config.flags |= GPIO_V2_LINE_FLAG_OPEN_DRAIN;
    request.config.num_attrs = 1;
    request.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    request.config.attrs[0].attr.values = static_cast<__u64>(outputPhysicalValue(false, activeHigh_));
    request.config.attrs[0].mask = 1;

    if (::ioctl(chipFd, GPIO_V2_GET_LINE_IOCTL, &request) < 0) {
        if (errorMessage)
            *errorMessage = "Could not request recording indicator GPIO line " +
                            std::to_string(gpioLine_) + " on " + chipPath + ": " +
                            std::strerror(errno);
        ::close(chipFd);
        return false;
    }

    ::close(chipFd);
    gpioLineFd_ = request.fd;
    gpioChipPath_ = chipPath;
    return true;
}

bool RecordingIndicatorOutput::openPcf8574Locked(std::string *errorMessage)
{
    if (pcfBusNumber_ < 0 || pcfAddress_ < 0 || pcfPin_ < 0 || pcfPin_ > 7) {
        if (errorMessage)
            *errorMessage = "Recording indicator PCF8574 bus/address/pin is invalid.";
        return false;
    }

    const std::string devicePath = "/dev/i2c-" + std::to_string(pcfBusNumber_);
    const int fd = ::open(devicePath.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        if (errorMessage)
            *errorMessage = "Could not open " + devicePath + ": " + std::strerror(errno);
        return false;
    }

    if (::ioctl(fd, I2C_SLAVE, pcfAddress_) < 0) {
        if (errorMessage)
            *errorMessage = "Could not select PCF8574 address 0x" + [&]() {
                std::ostringstream out;
                out << std::hex << pcfAddress_;
                return out.str();
            }() + " on " + devicePath + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }

    // The PCF8574 may also be used for button inputs. Keep every non-LED pin
    // released high so output writes never latch a pressed input low.
    pcfState_ = 0xff;

    pcfFd_ = fd;
    return true;
}

bool RecordingIndicatorOutput::setOutputActiveLocked(bool active, std::string *errorMessage)
{
    const int value = outputPhysicalValue(active, activeHigh_);
    if (target_ == Target::Pcf8574) {
        if (pcfFd_ < 0)
            return true;

        const uint8_t mask = static_cast<uint8_t>(1u << static_cast<unsigned int>(pcfPin_));
        if (value)
            pcfState_ = static_cast<uint8_t>(pcfState_ | mask);
        else
            pcfState_ = static_cast<uint8_t>(pcfState_ & ~mask);

        if (::write(pcfFd_, &pcfState_, 1) != 1) {
            if (errorMessage)
                *errorMessage = "Could not write PCF8574 recording indicator output: " +
                                std::string(std::strerror(errno));
            return false;
        }
        return true;
    }

    if (gpioLineFd_ < 0)
        return true;

    gpio_v2_line_values data{};
    data.mask = 1;
    data.bits = static_cast<__u64>(value);
    if (::ioctl(gpioLineFd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &data) < 0) {
        if (errorMessage)
            *errorMessage = "Could not set recording indicator GPIO line: " +
                            std::string(std::strerror(errno));
        return false;
    }
    return true;
}

void RecordingIndicatorOutput::closeOutputLocked()
{
    closeFd(&gpioLineFd_);
    closeFd(&pcfFd_);
}

void RecordingIndicatorOutput::setLastErrorLocked(const std::string &error)
{
    lastError_ = error;
    if (!error.empty())
        std::cerr << "ApertarCore recording indicator LED: " << error << "\n";
}

} // namespace apertar
