#include "hardware/NeoRotary4Input.hpp"
#include "hardware/ButtonActionList.hpp"

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
#include <system_error>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace apertar {
namespace {

constexpr uint8_t kStatusBase = 0x00;
constexpr uint8_t kStatusHardwareId = 0x01;
constexpr uint8_t kStatusSoftwareReset = 0x7f;
constexpr uint8_t kGpioBase = 0x01;
constexpr uint8_t kGpioDirClrBulk = 0x03;
constexpr uint8_t kGpioBulk = 0x04;
constexpr uint8_t kGpioBulkSet = 0x05;
constexpr uint8_t kGpioPullEnableSet = 0x0b;
constexpr uint8_t kEncoderBase = 0x11;
constexpr uint8_t kEncoderDelta = 0x40;
constexpr uint8_t kExpectedHardwareId = 0x87; // ATTiny817 seesaw, used by NeoRotary 4.
constexpr std::array<uint8_t, 4> kButtonPins{{12, 14, 17, 9}};

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

bool parseConfigBool(const std::string &text, bool fallback)
{
    const std::string normalized = normalizeToken(text);
    if (normalized == "true" || normalized == "yes" || normalized == "on" || normalized == "1")
        return true;
    if (normalized == "false" || normalized == "no" || normalized == "off" || normalized == "0")
        return false;
    return fallback;
}

std::string normalizeSingleAction(std::string action)
{
    action = normalizeToken(std::move(action));
    if (action.empty() || action == "none" || action == "off" || action == "disabled")
        return {};
    if (action == "recindicatorled" || action == "rec_indicator_led" ||
        action == "recording_indicator" || action == "recording_indicator_led" ||
        action == "record_led" || action == "rec_led" || action == "tally" ||
        action == "tally_led")
        return {};

    if (action == "record" || action == "capture" || action == "camera_button" || action == "take")
        return "capture";
    if (action == "record_toggle" || action == "toggle_record" || action == "rec")
        return "record_toggle";
    if (action == "still" || action == "photo" || action == "take_photo")
        return "photo";
    if (action == "timelapse" || action == "time_lapse")
        return "timelapse";
    if (action == "eject" || action == "unmount" || action == "eject_media" || action == "unmount_media")
        return "eject";
    if (action == "still_mode" || action == "photo_mode")
        return "still_mode";
    if (action == "settings" || action == "settings_page")
        return "settings";
    if (action == "clip_browser" || action == "browser" || action == "playback")
        return "clip_browser";
    if (action == "quick_menu" || action == "quickmenu")
        return "quick_menu";
    if (action == "standby" || action == "standby_toggle")
        return "standby";
    if (action == "zoom" || action == "zoom_toggle" || action == "preview_zoom")
        return "zoom_toggle";
    if (const auto script = hardware::normalizeScriptAction(action); !script.empty())
        return script;
    if (action == "focus_peaking" || action == "peaking")
        return "focus_peaking";
    if (action == "zebra" || action == "zebras")
        return "zebra";
    if (action == "false_color" || action == "falsecolour")
        return "false_color";
    if (action == "guides")
        return "guides";
    if (action == "center_marker" || action == "centre_marker")
        return "center_marker";
    if (action == "grayscale" || action == "greyscale")
        return "grayscale";
    if (action == "smpte" || action == "smpte_bars" || action == "color_bars" || action == "colour_bars")
        return "smpte";
    if (action == "external_monitor" || action == "external_monitoring" || action == "hdmi")
        return "external_monitor";
    if (action == "date_time" || action == "date_time_overlay")
        return "date_time_overlay";
    if (action == "audio_meter" || action == "vu_meter" || action == "vumeter")
        return "audio_meter";
    if (action == "fps" || action == "fps_cycle")
        return "fps";
    if (action == "fps_up" || action == "fps_next" || action == "fps_plus")
        return "fps_up";
    if (action == "fps_down" || action == "fps_prev" || action == "fps_previous" || action == "fps_minus")
        return "fps_down";
    if (action == "resolution" || action == "resolution_cycle" || action == "res" || action == "res_cycle")
        return "resolution";
    if (action == "resolution_up" || action == "resolution_next" || action == "resolution_plus" ||
        action == "res_up" || action == "res_next" || action == "res_plus")
        return "resolution_up";
    if (action == "resolution_down" || action == "resolution_prev" || action == "resolution_previous" ||
        action == "resolution_minus" || action == "res_down" || action == "res_prev" ||
        action == "res_previous" || action == "res_minus")
        return "resolution_down";
    if (action == "iso" || action == "iso_cycle")
        return "iso";
    if (action == "iso_up" || action == "iso_next" || action == "iso_plus")
        return "iso_up";
    if (action == "iso_down" || action == "iso_prev" || action == "iso_previous" || action == "iso_minus")
        return "iso_down";
    if (action == "shutter" || action == "shutter_cycle")
        return "shutter";
    if (action == "shutter_up" || action == "shutter_next" || action == "shutter_plus")
        return "shutter_up";
    if (action == "shutter_down" || action == "shutter_prev" || action == "shutter_previous" ||
        action == "shutter_minus")
        return "shutter_down";
    if (action == "wb" || action == "white_balance" || action == "whitebalance" ||
        action == "wb_cycle" || action == "white_balance_cycle")
        return "wb";
    if (action == "wb_up" || action == "white_balance_up" || action == "whitebalance_up" ||
        action == "wb_next" || action == "wb_plus")
        return "wb_up";
    if (action == "wb_down" || action == "white_balance_down" || action == "whitebalance_down" ||
        action == "wb_prev" || action == "wb_previous" || action == "wb_minus")
        return "wb_down";

    return action;
}

std::string normalizeAction(std::string action)
{
    return hardware::normalizeButtonMapping(action, normalizeSingleAction);
}

std::optional<int> encoderIndexFromKeyPrefix(const std::string &key)
{
    for (const std::string prefix : { "encoder", "enc", "rotary" }) {
        if (key.rfind(prefix, 0) != 0)
            continue;
        std::string rest = key.substr(prefix.size());
        if (!rest.empty() && rest.front() == '_')
            rest.erase(rest.begin());
        if (rest.empty() || !std::isdigit(static_cast<unsigned char>(rest.front())))
            continue;
        const int encoder = rest.front() - '0';
        if (encoder < 0 || encoder > 3)
            return std::nullopt;
        return encoder;
    }
    return std::nullopt;
}

std::string encoderFieldFromKey(std::string key)
{
    key = normalizeToken(std::move(key));
    for (const std::string prefix : { "encoder", "enc", "rotary" }) {
        if (key.rfind(prefix, 0) != 0)
            continue;
        std::string rest = key.substr(prefix.size());
        if (!rest.empty() && rest.front() == '_')
            rest.erase(rest.begin());
        if (rest.size() < 2 || !std::isdigit(static_cast<unsigned char>(rest.front())))
            continue;
        rest.erase(rest.begin());
        if (!rest.empty() && rest.front() == '_')
            rest.erase(rest.begin());
        return rest;
    }
    return {};
}

bool hasMappedAction(const std::array<NeoRotary4Input::EncoderActions, 4> &encoders)
{
    return std::any_of(encoders.begin(), encoders.end(), [](const auto &actions) {
        return !actions.clockwise.empty() || !actions.counterClockwise.empty() || !actions.press.empty();
    });
}

std::string i2cDevicePath(int busNumber)
{
    return "/dev/i2c-" + std::to_string(busNumber);
}

void writeBigEndian32(uint32_t value, uint8_t out[4])
{
    out[0] = static_cast<uint8_t>((value >> 24) & 0xff);
    out[1] = static_cast<uint8_t>((value >> 16) & 0xff);
    out[2] = static_cast<uint8_t>((value >> 8) & 0xff);
    out[3] = static_cast<uint8_t>(value & 0xff);
}

int32_t readBigEndianI32(const uint8_t in[4])
{
    const uint32_t value =
        (static_cast<uint32_t>(in[0]) << 24) |
        (static_cast<uint32_t>(in[1]) << 16) |
        (static_cast<uint32_t>(in[2]) << 8) |
        static_cast<uint32_t>(in[3]);
    return static_cast<int32_t>(value);
}

uint32_t readBigEndianU32(const uint8_t in[4])
{
    return (static_cast<uint32_t>(in[0]) << 24) |
           (static_cast<uint32_t>(in[1]) << 16) |
           (static_cast<uint32_t>(in[2]) << 8) |
           static_cast<uint32_t>(in[3]);
}

} // namespace

NeoRotary4Input::NeoRotary4Input(ActionCallback actionCallback, ActionCallback rotationCallback)
    : actionCallback_(std::move(actionCallback))
    , rotationCallback_(rotationCallback ? std::move(rotationCallback) : actionCallback_)
{
}

NeoRotary4Input::~NeoRotary4Input()
{
    stop();
}

bool NeoRotary4Input::start()
{
    stop();
    config_ = loadConfig();
    if (!config_.configured || !config_.enabled)
        return false;

    if (!hasMappedAction(config_.encoders)) {
        std::cerr << "NeoRotary4 config in " << config_.configPath
                  << " has no encoder actions; input disabled.\n";
        return false;
    }

    running_ = true;
    thread_ = std::thread(&NeoRotary4Input::run, this);
    std::cerr << "NeoRotary4 input enabled: " << i2cDevicePath(config_.busNumber)
              << " address=0x" << std::hex << config_.address << std::dec
              << " config=" << config_.configPath << "\n";
    return true;
}

void NeoRotary4Input::stop()
{
    if (!running_.exchange(false))
        return;
    if (thread_.joinable())
        thread_.join();
}

bool NeoRotary4Input::enabled() const
{
    return config_.configured && config_.enabled;
}

std::string NeoRotary4Input::configPath() const
{
    return config_.configPath;
}

std::string NeoRotary4Input::lastError() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return lastError_;
}

NeoRotary4Input::Config NeoRotary4Input::loadConfig()
{
    Config config;
    std::error_code ec;

    for (const auto &path : hardwareConfigCandidatePaths()) {
        if (!std::filesystem::exists(path, ec) || !std::filesystem::is_regular_file(path, ec))
            continue;

        std::ifstream input(path);
        if (!input.is_open())
            continue;

        bool inNeoRotarySection = false;
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
                inNeoRotarySection = section == "neorotary4" ||
                                     section == "neo_rotary4" ||
                                     section == "neo_rotary_4";
                if (inNeoRotarySection) {
                    config.configured = true;
                    config.enabled = true;
                    config.configPath = path.string();
                }
                continue;
            }

            if (!inNeoRotarySection)
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
                    config.pollMs = std::clamp(*parsed, 10, 1000);
            } else if (key == "debounce_ms") {
                if (const auto parsed = parseConfigInteger(value))
                    config.debounceMs = std::clamp(*parsed, 0, 1000);
            } else if (key == "invert_direction" || key == "invert" || key == "reverse_direction") {
                config.invertDirection = parseConfigBool(value, config.invertDirection);
            } else if (key == "reset_on_start") {
                config.resetOnStart = parseConfigBool(value, config.resetOnStart);
            } else if (key == "max_events_per_poll") {
                if (const auto parsed = parseConfigInteger(value))
                    config.maxEventsPerPoll = std::clamp(*parsed, 1, 32);
            } else if (const auto encoder = encoderIndexFromKeyPrefix(key)) {
                const std::string field = encoderFieldFromKey(key);
                const std::string normalized = normalizeAction(value);
                if (field == "cw" || field == "clockwise" || field == "right" || field == "up" ||
                    field == "plus") {
                    config.encoders[*encoder].clockwise = normalized;
                } else if (field == "ccw" || field == "counter_clockwise" || field == "counterclockwise" ||
                           field == "left" || field == "down" || field == "minus") {
                    config.encoders[*encoder].counterClockwise = normalized;
                } else if (field == "press" || field == "push" || field == "button" || field == "click") {
                    config.encoders[*encoder].press = normalized;
                }
            }
        }

        if (config.configured)
            return config;
    }

    return config;
}

bool NeoRotary4Input::openDevice(int *fd, std::string *errorMessage) const
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
            out << "NeoRotary4 address 0x" << std::hex << config_.address << std::dec
                << ": " << std::strerror(errno);
            *errorMessage = out.str();
        }
        closeDevice(fd);
        return false;
    }

    if (config_.resetOnStart) {
        const uint8_t resetValue = 0xff;
        if (!writeRegister(*fd, kStatusBase, kStatusSoftwareReset, &resetValue, 1)) {
            if (errorMessage)
                *errorMessage = "NeoRotary4 software reset failed: " + std::string(std::strerror(errno));
            closeDevice(fd);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (!verifyDevice(*fd, errorMessage)) {
        closeDevice(fd);
        return false;
    }

    if (!configureButtons(*fd, errorMessage)) {
        closeDevice(fd);
        return false;
    }

    return true;
}

bool NeoRotary4Input::verifyDevice(int fd, std::string *errorMessage) const
{
    uint8_t hardwareId = 0;
    if (!readRegister(fd, kStatusBase, kStatusHardwareId, &hardwareId, 1)) {
        if (errorMessage)
            *errorMessage = "NeoRotary4 hardware ID read failed: " + std::string(std::strerror(errno));
        return false;
    }

    if (hardwareId != kExpectedHardwareId) {
        if (errorMessage) {
            std::ostringstream out;
            out << "NeoRotary4 unexpected hardware ID 0x" << std::hex << static_cast<int>(hardwareId)
                << " at address 0x" << config_.address << std::dec;
            *errorMessage = out.str();
        }
        return false;
    }

    return true;
}

bool NeoRotary4Input::configureButtons(int fd, std::string *errorMessage) const
{
    uint32_t mask = 0;
    for (uint8_t pin : kButtonPins)
        mask |= (1u << pin);

    uint8_t bytes[4]{};
    writeBigEndian32(mask, bytes);

    if (!writeRegister(fd, kGpioBase, kGpioDirClrBulk, bytes, sizeof(bytes))) {
        if (errorMessage)
            *errorMessage = "NeoRotary4 button input configuration failed: " + std::string(std::strerror(errno));
        return false;
    }
    if (!writeRegister(fd, kGpioBase, kGpioPullEnableSet, bytes, sizeof(bytes))) {
        if (errorMessage)
            *errorMessage = "NeoRotary4 button pull-up configuration failed: " + std::string(std::strerror(errno));
        return false;
    }
    if (!writeRegister(fd, kGpioBase, kGpioBulkSet, bytes, sizeof(bytes))) {
        if (errorMessage)
            *errorMessage = "NeoRotary4 button pull-up latch failed: " + std::string(std::strerror(errno));
        return false;
    }

    return true;
}

bool NeoRotary4Input::writeRegister(int fd, uint8_t base, uint8_t reg, const uint8_t *data, size_t size) const
{
    std::array<uint8_t, 34> buffer{};
    if (size + 2 > buffer.size()) {
        errno = EINVAL;
        return false;
    }

    buffer[0] = base;
    buffer[1] = reg;
    if (data && size > 0)
        std::copy(data, data + size, buffer.begin() + 2);
    return ::write(fd, buffer.data(), size + 2) == static_cast<ssize_t>(size + 2);
}

bool NeoRotary4Input::readRegister(int fd, uint8_t base, uint8_t reg, uint8_t *data, size_t size, int delayUs) const
{
    if (!data || size == 0)
        return false;

    const uint8_t selector[2] = { base, reg };
    if (::write(fd, selector, sizeof(selector)) != static_cast<ssize_t>(sizeof(selector)))
        return false;
    if (delayUs > 0)
        ::usleep(static_cast<useconds_t>(delayUs));
    return ::read(fd, data, size) == static_cast<ssize_t>(size);
}

bool NeoRotary4Input::readEncoderDelta(int fd, int encoder, int32_t *delta) const
{
    if (!delta || encoder < 0 || encoder > 3)
        return false;

    uint8_t bytes[4]{};
    if (!readRegister(fd, kEncoderBase, static_cast<uint8_t>(kEncoderDelta | encoder), bytes, sizeof(bytes)))
        return false;

    *delta = readBigEndianI32(bytes);
    if (config_.invertDirection)
        *delta = -*delta;
    return true;
}

bool NeoRotary4Input::readButtonMask(int fd, uint32_t *mask) const
{
    if (!mask)
        return false;

    uint8_t bytes[4]{};
    if (!readRegister(fd, kGpioBase, kGpioBulk, bytes, sizeof(bytes)))
        return false;
    *mask = readBigEndianU32(bytes);
    return true;
}

void NeoRotary4Input::closeDevice(int *fd) const
{
    if (fd && *fd >= 0) {
        ::close(*fd);
        *fd = -1;
    }
}

void NeoRotary4Input::emitAction(const std::string &action)
{
    if (action.empty() || !actionCallback_)
        return;
    actionCallback_(action);
}

void NeoRotary4Input::setLastError(const std::string &error)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (lastError_ == error)
        return;
    lastError_ = error;
    if (!error.empty())
        std::cerr << "NeoRotary4 input: " << error << "\n";
}

void NeoRotary4Input::run()
{
    int fd = -1;
    uint8_t observedPressedMask = 0;
    uint8_t stablePressedMask = 0;
    auto observedChangedAt = std::chrono::steady_clock::now();
    bool haveButtonSample = false;

    while (running_.load(std::memory_order_relaxed)) {
        if (fd < 0) {
            std::string errorMessage;
            if (!openDevice(&fd, &errorMessage)) {
                setLastError(errorMessage);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
            setLastError({});
            haveButtonSample = false;

            // Clear any movement accumulated while ApertarCore was not polling.
            for (int encoder = 0; encoder < 4; ++encoder) {
                int32_t ignored = 0;
                (void)readEncoderDelta(fd, encoder, &ignored);
            }
        }

        for (int encoder = 0; encoder < 4; ++encoder) {
            int32_t delta = 0;
            if (!readEncoderDelta(fd, encoder, &delta)) {
                setLastError("encoder delta read failed: " + std::string(std::strerror(errno)));
                closeDevice(&fd);
                break;
            }

            const std::string &action = delta > 0
                ? config_.encoders[encoder].clockwise
                : config_.encoders[encoder].counterClockwise;
            const int count = std::min<int>(std::abs(delta), config_.maxEventsPerPoll);
            for (int i = 0; i < count; ++i)
                if (!action.empty() && rotationCallback_)
                    rotationCallback_(action);
        }

        if (fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
        }

        uint32_t gpioMask = 0;
        if (!readButtonMask(fd, &gpioMask)) {
            setLastError("button state read failed: " + std::string(std::strerror(errno)));
            closeDevice(&fd);
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
        }

        uint8_t pressedMask = 0;
        for (int encoder = 0; encoder < 4; ++encoder) {
            const uint32_t bit = (1u << kButtonPins[encoder]);
            if ((gpioMask & bit) == 0)
                pressedMask |= static_cast<uint8_t>(1u << encoder);
        }

        const auto now = std::chrono::steady_clock::now();
        if (!haveButtonSample) {
            observedPressedMask = pressedMask;
            stablePressedMask = pressedMask;
            observedChangedAt = now;
            haveButtonSample = true;
        } else if (pressedMask != observedPressedMask) {
            observedPressedMask = pressedMask;
            observedChangedAt = now;
        } else {
            const auto stableForMs =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - observedChangedAt).count();
            if (stableForMs >= config_.debounceMs && observedPressedMask != stablePressedMask) {
                const uint8_t changedMask = static_cast<uint8_t>(observedPressedMask ^ stablePressedMask);
                stablePressedMask = observedPressedMask;
                for (int encoder = 0; encoder < 4; ++encoder) {
                    const uint8_t bit = static_cast<uint8_t>(1u << encoder);
                    if ((changedMask & bit) == 0 || (stablePressedMask & bit) == 0)
                        continue;
                    emitAction(config_.encoders[encoder].press);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config_.pollMs));
    }

    closeDevice(&fd);
}

} // namespace apertar
