#include "hardware/TriggerOutput.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
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

constexpr uint32_t kMaxQueuedPulses = 4;

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

std::set<TriggerOutput::Event> defaultTriggerEvents()
{
    return {
        TriggerOutput::Event::RecordingStart,
        TriggerOutput::Event::Still,
        TriggerOutput::Event::TimelapseFrame,
    };
}

std::optional<TriggerOutput::Event> eventFromToken(std::string token)
{
    token = normalizeToken(std::move(token));
    if (token == "record" || token == "record_start" || token == "recording" ||
        token == "recording_start" || token == "rec_start") {
        return TriggerOutput::Event::RecordingStart;
    }
    if (token == "still" || token == "photo" || token == "photo_capture" ||
        token == "capture_photo" || token == "still_capture") {
        return TriggerOutput::Event::Still;
    }
    if (token == "timelapse" || token == "timelapse_start" || token == "time_lapse_start") {
        return TriggerOutput::Event::TimelapseStart;
    }
    if (token == "timelapse_frame" || token == "timelapse_shot" ||
        token == "timelapse_still" || token == "time_lapse_frame") {
        return TriggerOutput::Event::TimelapseFrame;
    }
    return std::nullopt;
}

std::set<TriggerOutput::Event> parseEventList(std::string text)
{
    for (char &ch : text) {
        if (ch == ',' || ch == ';' || ch == '|' || ch == '+')
            ch = ' ';
    }

    std::set<TriggerOutput::Event> events;
    std::istringstream stream(text);
    std::string token;
    while (stream >> token) {
        const std::string normalized = normalizeToken(token);
        if (normalized == "all") {
            events.insert(TriggerOutput::Event::RecordingStart);
            events.insert(TriggerOutput::Event::Still);
            events.insert(TriggerOutput::Event::TimelapseStart);
            events.insert(TriggerOutput::Event::TimelapseFrame);
            continue;
        }

        if (auto event = eventFromToken(normalized))
            events.insert(*event);
    }
    return events;
}

std::string eventsSummary(const std::set<TriggerOutput::Event> &events)
{
    std::ostringstream out;
    bool first = true;
    for (TriggerOutput::Event event : events) {
        if (!first)
            out << ',';
        first = false;
        switch (event) {
        case TriggerOutput::Event::RecordingStart: out << "recording_start"; break;
        case TriggerOutput::Event::Still: out << "still"; break;
        case TriggerOutput::Event::TimelapseStart: out << "timelapse_start"; break;
        case TriggerOutput::Event::TimelapseFrame: out << "timelapse_frame"; break;
        }
    }
    return out.str();
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

TriggerOutput::~TriggerOutput()
{
    stop();
}

TriggerOutput::Config TriggerOutput::loadConfig()
{
    Config config;
    config.events = defaultTriggerEvents();

    std::error_code ec;
    for (const auto &path : hardwareConfigCandidatePaths()) {
        if (!std::filesystem::exists(path, ec) || !std::filesystem::is_regular_file(path, ec))
            continue;

        std::ifstream input(path);
        if (!input.is_open())
            continue;

        config.configPath = path.string();
        std::string section;
        std::optional<int> genericPin;
        bool eventsConfigured = false;
        std::string line;
        while (std::getline(input, line)) {
            std::string trimmed = trimAscii(line);
            if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';')
                continue;

            if (trimmed.front() == '[' && trimmed.back() == ']') {
                section = normalizeToken(trimmed.substr(1, trimmed.size() - 2));
                if (section == "trigger" || section == "capture_trigger" || section == "hotshoe")
                    section = "trigger_output";
                if (section == "trigger_output")
                    config.configured = true;
                continue;
            }

            const size_t equals = trimmed.find('=');
            if (equals == std::string::npos)
                continue;

            const std::string key = normalizeToken(trimmed.substr(0, equals));
            const std::string value = stripInlineComment(trimmed.substr(equals + 1));
            if (section == "pcf8574") {
                if (key == "bus") {
                    if (auto parsed = parseConfigInteger(value))
                        config.pcfBusNumber = *parsed;
                } else if (key == "address") {
                    if (auto parsed = parseConfigInteger(value))
                        config.pcfAddress = *parsed;
                }
                continue;
            }

            if (section != "trigger_output")
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
                config.gpioChipPath = value;
            } else if (key == "line" || key == "gpio" || key == "gpio_line") {
                if (auto parsed = parseConfigInteger(value))
                    config.gpioLine = *parsed;
            } else if (key == "pin") {
                genericPin = parseConfigInteger(value);
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
            } else if (key == "pulse_ms" || key == "duration_ms") {
                if (auto parsed = parseConfigInteger(value)) {
                    config.pulseMs = static_cast<uint32_t>(std::clamp(*parsed, 1, 5000));
                }
            } else if (key == "events" || key == "event") {
                const std::set<Event> parsed = parseEventList(value);
                if (!parsed.empty()) {
                    config.events = parsed;
                    eventsConfigured = true;
                }
            }
        }

        if (genericPin) {
            if (config.target == Target::Pcf8574)
                config.pcfPin = *genericPin;
            else
                config.gpioLine = *genericPin;
        }

        if (!eventsConfigured && config.events.empty())
            config.events = defaultTriggerEvents();
        return config;
    }

    config.events = defaultTriggerEvents();
    return config;
}

bool TriggerOutput::start()
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
        pulseMs_ = config.pulseMs;
        events_ = config.events;
        pendingPulses_ = 0;

        if (!openOutputLocked(&error) || !setOutputActiveLocked(false, &error)) {
            closeOutputLocked();
            enabled_ = false;
            running_ = false;
            setLastErrorLocked(error);
            return false;
        }

        enabled_ = true;
        running_ = true;
    }

    worker_ = std::thread(&TriggerOutput::workerLoop, this);

    std::string openedGpioChipPath;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        openedGpioChipPath = gpioChipPath_;
    }

    const char *targetText = config.target == Target::Pcf8574 ? "pcf8574" : "gpio";
    std::cerr << "ApertarCore trigger output enabled: target=" << targetText
              << " events=" << eventsSummary(config.events)
              << " pulse_ms=" << config.pulseMs
              << " active_high=" << (config.activeHigh ? "true" : "false");
    if (config.target == Target::Pcf8574) {
        std::cerr << " pcf=i2c-" << config.pcfBusNumber
                  << "/0x" << std::hex << config.pcfAddress << std::dec
                  << " pin=" << config.pcfPin;
    } else {
        std::cerr << " gpio=" << openedGpioChipPath
                  << " line=" << config.gpioLine
                  << " open_drain=" << (config.gpioOpenDrain ? "true" : "false");
    }
    std::cerr << "\n";
    return true;
}

void TriggerOutput::stop()
{
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        running_ = false;
        pendingPulses_ = 0;
    }
    cv_.notify_all();

    if (worker_.joinable())
        worker_.join();

    std::lock_guard<std::mutex> lock(stateMutex_);
    std::string ignored;
    setOutputActiveLocked(false, &ignored);
    closeOutputLocked();
    enabled_ = false;
    running_ = false;
    pendingPulses_ = 0;
}

void TriggerOutput::pulse(Event event)
{
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (!enabled_ || !running_ || !eventEnabledLocked(event))
            return;
        pendingPulses_ = std::min<uint32_t>(kMaxQueuedPulses, pendingPulses_ + 1);
    }
    cv_.notify_one();
}

bool TriggerOutput::enabled() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return enabled_;
}

std::string TriggerOutput::configPath() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return configPath_;
}

std::string TriggerOutput::lastError() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return lastError_;
}

void TriggerOutput::workerLoop()
{
    while (true) {
        uint32_t pulseMs = 0;
        {
            std::unique_lock<std::mutex> lock(stateMutex_);
            cv_.wait(lock, [this]() {
                return !running_ || pendingPulses_ > 0;
            });

            if (!running_ && pendingPulses_ == 0)
                break;

            if (pendingPulses_ > 0)
                --pendingPulses_;
            pulseMs = pulseMs_;

            std::string error;
            lock.unlock();
            const bool written = setOutputActiveLocked(true, &error);
            lock.lock();
            if (!written)
                setLastErrorLocked(error);
        }

        {
            std::unique_lock<std::mutex> lock(stateMutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(pulseMs), [this]() {
                return !running_;
            });

            std::string error;
            lock.unlock();
            const bool written = setOutputActiveLocked(false, &error);
            lock.lock();
            if (!written)
                setLastErrorLocked(error);

            if (!running_ && pendingPulses_ == 0)
                break;
        }
    }
}

bool TriggerOutput::eventEnabledLocked(Event event) const
{
    return events_.find(event) != events_.end();
}

bool TriggerOutput::openOutputLocked(std::string *errorMessage)
{
    if (target_ == Target::Pcf8574)
        return openPcf8574Locked(errorMessage);
    return openGpioLocked(errorMessage);
}

bool TriggerOutput::openGpioLocked(std::string *errorMessage)
{
    if (gpioLine_ < 0) {
        if (errorMessage)
            *errorMessage = "Trigger GPIO line must be >= 0.";
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
            ? "No usable /dev/gpiochip found for trigger GPIO line"
            : lastError;
    return false;
}

bool TriggerOutput::requestGpioLineFromChipLocked(const std::string &chipPath,
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
    std::snprintf(request.consumer, sizeof(request.consumer), "apertar-trigger");
    request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    if (gpioOpenDrain_)
        request.config.flags |= GPIO_V2_LINE_FLAG_OPEN_DRAIN;
    request.config.num_attrs = 1;
    request.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    request.config.attrs[0].attr.values = static_cast<__u64>(outputPhysicalValue(false, activeHigh_));
    request.config.attrs[0].mask = 1;

    if (::ioctl(chipFd, GPIO_V2_GET_LINE_IOCTL, &request) < 0) {
        if (errorMessage)
            *errorMessage = "Could not request GPIO line " + std::to_string(gpioLine_) +
                            " on " + chipPath + ": " + std::strerror(errno);
        ::close(chipFd);
        return false;
    }

    ::close(chipFd);
    gpioLineFd_ = request.fd;
    gpioChipPath_ = chipPath;
    return true;
}

bool TriggerOutput::openPcf8574Locked(std::string *errorMessage)
{
    if (pcfBusNumber_ < 0 || pcfAddress_ < 0 || pcfPin_ < 0 || pcfPin_ > 7) {
        if (errorMessage)
            *errorMessage = "Trigger PCF8574 bus/address/pin is invalid.";
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

    uint8_t current = 0xff;
    if (::read(fd, &current, 1) == 1)
        pcfState_ = current;
    else
        pcfState_ = 0xff;

    pcfFd_ = fd;
    return true;
}

bool TriggerOutput::setOutputActiveLocked(bool active, std::string *errorMessage)
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
                *errorMessage = "Could not write PCF8574 trigger output: " + std::string(std::strerror(errno));
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
            *errorMessage = "Could not set trigger GPIO line: " + std::string(std::strerror(errno));
        return false;
    }
    return true;
}

void TriggerOutput::closeOutputLocked()
{
    closeFd(&gpioLineFd_);
    closeFd(&pcfFd_);
}

void TriggerOutput::setLastErrorLocked(const std::string &error)
{
    lastError_ = error;
    if (!error.empty())
        std::cerr << "ApertarCore trigger output: " << error << "\n";
}

} // namespace apertar
