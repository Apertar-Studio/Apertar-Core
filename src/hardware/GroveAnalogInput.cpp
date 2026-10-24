#include "hardware/GroveAnalogInput.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>

#include <fcntl.h>
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

std::optional<double> parseConfigDouble(const std::string &text)
{
    const std::string trimmed = trimAscii(text);
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
    const std::string normalized = normalizeToken(text);
    if (normalized == "true" || normalized == "yes" || normalized == "on" || normalized == "1")
        return true;
    if (normalized == "false" || normalized == "no" || normalized == "off" || normalized == "0")
        return false;
    return fallback;
}

std::string normalizeTarget(std::string target)
{
    target = normalizeToken(std::move(target));
    if (target.empty() || target == "none" || target == "off" || target == "disabled")
        return {};
    if (target == "white_balance" || target == "whitebalance")
        return "wb";
    if (target == "focus_peaking" || target == "focus_peaking_value" || target == "focus")
        return "focus_peaking_threshold";
    if (target == "zebra" || target == "zebras")
        return "zebra_threshold";
    if (target == "volume" || target == "input_gain")
        return "input_volume";
    if (target == "headphone" || target == "headphones")
        return "headphone_volume";
    if (target == "opacity" || target == "controls_opacity")
        return "camera_controls_opacity";
    if (target == "res")
        return "resolution";
    return target;
}

std::optional<int> channelIndexFromKeyPrefix(const std::string &key)
{
    for (const std::string prefix : { "channel", "chan", "ch", "analog", "a" }) {
        if (key.rfind(prefix, 0) != 0)
            continue;
        std::string rest = key.substr(prefix.size());
        if (!rest.empty() && rest.front() == '_')
            rest.erase(rest.begin());
        if (rest.empty() || !std::isdigit(static_cast<unsigned char>(rest.front())))
            continue;
        const int channel = rest.front() - '0';
        if (channel < 0 || channel > 7)
            return std::nullopt;
        return channel;
    }
    return std::nullopt;
}

std::string channelFieldFromKey(std::string key)
{
    key = normalizeToken(std::move(key));
    for (const std::string prefix : { "channel", "chan", "ch", "analog", "a" }) {
        if (key.rfind(prefix, 0) != 0)
            continue;
        std::string rest = key.substr(prefix.size());
        if (!rest.empty() && rest.front() == '_')
            rest.erase(rest.begin());
        if (rest.empty() || !std::isdigit(static_cast<unsigned char>(rest.front())))
            continue;
        rest.erase(rest.begin());
        if (!rest.empty() && rest.front() == '_')
            rest.erase(rest.begin());
        return rest.empty() ? "target" : rest;
    }
    return {};
}

std::string i2cDevicePath(int busNumber)
{
    return "/dev/i2c-" + std::to_string(busNumber);
}

} // namespace

GroveAnalogInput::GroveAnalogInput(AnalogCallback analogCallback)
    : analogCallback_(std::move(analogCallback))
{
}

GroveAnalogInput::~GroveAnalogInput()
{
    stop();
}

bool GroveAnalogInput::start()
{
    stop();
    config_ = loadConfig();
    if (!config_.configured || !config_.enabled)
        return false;

    if (config_.channels.empty()) {
        std::cerr << "Grove analog config in " << config_.configPath
                  << " has no channel mappings; input disabled.\n";
        return false;
    }

    running_ = true;
    thread_ = std::thread(&GroveAnalogInput::run, this);
    std::cerr << "Grove analog input enabled: " << i2cDevicePath(config_.busNumber)
              << " address=0x" << std::hex << config_.address << std::dec
              << " config=" << config_.configPath << "\n";
    return true;
}

void GroveAnalogInput::stop()
{
    if (!running_.exchange(false))
        return;
    if (thread_.joinable())
        thread_.join();
}

bool GroveAnalogInput::enabled() const
{
    return config_.configured && config_.enabled;
}

std::string GroveAnalogInput::configPath() const
{
    return config_.configPath;
}

std::string GroveAnalogInput::lastError() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return lastError_;
}

GroveAnalogInput::Config GroveAnalogInput::loadConfig()
{
    Config config;
    std::error_code ec;

    for (const auto &path : hardwareConfigCandidatePaths()) {
        if (!std::filesystem::exists(path, ec) || !std::filesystem::is_regular_file(path, ec))
            continue;

        std::ifstream input(path);
        if (!input.is_open())
            continue;

        bool inAnalogSection = false;
        std::array<ChannelMapping, 8> channelDrafts{};
        std::array<bool, 8> channelSeen{};
        std::string line;
        while (std::getline(input, line)) {
            std::string trimmed = trimAscii(line);
            if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';')
                continue;

            const auto comment = trimmed.find_first_of("#;");
            if (comment != std::string::npos)
                trimmed = trimAscii(trimmed.substr(0, comment));
            if (trimmed.empty())
                continue;

            if (trimmed.front() == '[' && trimmed.back() == ']') {
                const std::string section = normalizeToken(trimmed.substr(1, trimmed.size() - 2));
                inAnalogSection = section == "grove_analog" || section == "grove_adc" ||
                                  section == "analog_controls";
                if (inAnalogSection) {
                    config.configured = true;
                    config.enabled = true;
                    config.configPath = path.string();
                }
                continue;
            }

            if (!inAnalogSection)
                continue;

            const std::size_t eq = trimmed.find('=');
            if (eq == std::string::npos)
                continue;

            const std::string key = normalizeToken(trimmed.substr(0, eq));
            const std::string value = trimAscii(trimmed.substr(eq + 1));
            if (key == "enabled") {
                config.enabled = parseConfigBool(value, config.enabled);
            } else if (key == "bus" || key == "channel" || key == "i2c_bus") {
                if (const auto parsed = parseConfigInteger(value); parsed && *parsed >= 0 && *parsed <= 255)
                    config.busNumber = *parsed;
            } else if (key == "address" || key == "addr") {
                if (const auto parsed = parseConfigInteger(value); parsed && *parsed >= 0x03 && *parsed <= 0x77)
                    config.address = *parsed;
            } else if (key == "poll_ms" || key == "poll_interval_ms") {
                if (const auto parsed = parseConfigInteger(value))
                    config.pollMs = std::clamp(*parsed, 20, 2000);
            } else if (key == "deadband" || key == "deadband_raw") {
                if (const auto parsed = parseConfigInteger(value))
                    config.deadband = std::clamp(*parsed, 0, 2048);
            } else if (key == "emit_on_start" || key == "sync_on_start") {
                config.emitOnStart = parseConfigBool(value, config.emitOnStart);
            } else if (const auto channel = channelIndexFromKeyPrefix(key)) {
                const std::string field = channelFieldFromKey(key);
                ChannelMapping &mapping = channelDrafts[*channel];
                mapping.channel = *channel;
                mapping.deadband = config.deadband;
                channelSeen[*channel] = true;

                if (field == "target" || field == "control") {
                    mapping.target = normalizeTarget(value);
                } else if (field == "min" || field == "minimum") {
                    if (const auto parsed = parseConfigDouble(value))
                        mapping.minValue = *parsed;
                } else if (field == "max" || field == "maximum") {
                    if (const auto parsed = parseConfigDouble(value))
                        mapping.maxValue = *parsed;
                } else if (field == "step") {
                    if (const auto parsed = parseConfigDouble(value))
                        mapping.step = *parsed;
                } else if (field == "deadband" || field == "deadband_raw") {
                    if (const auto parsed = parseConfigInteger(value))
                        mapping.deadband = std::clamp(*parsed, 0, 2048);
                } else if (field == "invert" || field == "reverse") {
                    mapping.invert = parseConfigBool(value, mapping.invert);
                } else if (field == "mode") {
                    const std::string mode = normalizeToken(value);
                    mapping.usePresets = mode == "preset" || mode == "presets" || mode == "stepped";
                } else if (field == "presets" || field == "preset") {
                    mapping.usePresets = parseConfigBool(value, mapping.usePresets);
                }
            }
        }

        for (size_t i = 0; i < channelDrafts.size(); ++i) {
            ChannelMapping mapping = channelDrafts[i];
            if (!channelSeen[i] || mapping.channel < 0 || mapping.target.empty())
                continue;
            config.channels.push_back(std::move(mapping));
        }

        if (config.configured)
            return config;
    }

    return config;
}

bool GroveAnalogInput::openDevice(int *fd, std::string *errorMessage) const
{
    if (!fd)
        return false;

    const std::string path = i2cDevicePath(config_.busNumber);
    *fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (*fd < 0) {
        if (errorMessage)
            *errorMessage = path + ": " + std::strerror(errno);
        return false;
    }

    if (::ioctl(*fd, I2C_SLAVE, config_.address) < 0) {
        if (errorMessage) {
            std::ostringstream out;
            out << "Grove analog address 0x" << std::hex << config_.address << std::dec
                << ": " << std::strerror(errno);
            *errorMessage = out.str();
        }
        closeDevice(fd);
        return false;
    }

    return true;
}

bool GroveAnalogInput::readRaw(int fd, int channel, int *raw) const
{
    if (fd < 0 || !raw || channel < 0 || channel > 7)
        return false;

    const uint8_t reg = static_cast<uint8_t>(0x10 + channel);
    if (::write(fd, &reg, 1) != 1)
        return false;

    uint8_t bytes[2]{};
    if (::read(fd, bytes, sizeof(bytes)) != static_cast<ssize_t>(sizeof(bytes)))
        return false;

    const int value = static_cast<int>(bytes[0]) | (static_cast<int>(bytes[1]) << 8);
    *raw = std::clamp(value, 0, 4095);
    return true;
}

void GroveAnalogInput::closeDevice(int *fd) const
{
    if (fd && *fd >= 0) {
        ::close(*fd);
        *fd = -1;
    }
}

void GroveAnalogInput::emitAnalog(const ChannelMapping &channel, int raw)
{
    if (!analogCallback_ || channel.target.empty())
        return;

    double normalized = std::clamp(static_cast<double>(raw) / 4095.0, 0.0, 1.0);
    if (channel.invert)
        normalized = 1.0 - normalized;

    AnalogEvent event;
    event.target = channel.target;
    event.normalized = normalized;
    event.raw = raw;
    event.minValue = channel.minValue;
    event.maxValue = channel.maxValue;
    event.step = channel.step;
    event.usePresets = channel.usePresets;
    analogCallback_(event);
}

void GroveAnalogInput::setLastError(const std::string &error)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (lastError_ == error)
        return;
    lastError_ = error;
    if (!error.empty())
        std::cerr << "Grove analog input: " << error << "\n";
}

void GroveAnalogInput::run()
{
    int fd = -1;

    while (running_.load(std::memory_order_relaxed)) {
        if (fd < 0) {
            std::string errorMessage;
            if (!openDevice(&fd, &errorMessage)) {
                setLastError(errorMessage);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
            setLastError({});
            for (ChannelMapping &channel : config_.channels)
                channel.haveLastRaw = false;
        }

        for (ChannelMapping &channel : config_.channels) {
            int raw = 0;
            if (!readRaw(fd, channel.channel, &raw)) {
                setLastError("channel read failed: " + std::string(std::strerror(errno)));
                closeDevice(&fd);
                break;
            }

            if (!channel.haveLastRaw) {
                channel.haveLastRaw = true;
                channel.lastRaw = raw;
                if (config_.emitOnStart)
                    emitAnalog(channel, raw);
                continue;
            }

            if (std::abs(raw - channel.lastRaw) < std::max(0, channel.deadband))
                continue;

            channel.lastRaw = raw;
            emitAnalog(channel, raw);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config_.pollMs));
    }

    closeDevice(&fd);
}

} // namespace apertar
