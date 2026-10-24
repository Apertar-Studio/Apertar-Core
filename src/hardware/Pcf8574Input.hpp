#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace apertar {

class Pcf8574Input {
public:
    using ActionCallback = std::function<void(const std::string &)>;

    explicit Pcf8574Input(ActionCallback actionCallback);
    ~Pcf8574Input();

    bool start();
    void stop();

    bool enabled() const;
    std::string configPath() const;
    std::string lastError() const;

private:
    friend struct HardwareInputTestAccess;
    struct Config {
        bool configured = false;
        bool enabled = false;
        int busNumber = 1;
        int address = 0x20;
        int pollMs = 40;
        int debounceMs = 60;
        bool activeLow = true;
        std::array<std::string, 8> actions{};
        std::string configPath;
    };

    static Config loadConfig();
    void run();
    bool openDevice(int *fd, std::string *errorMessage) const;
    bool configureAsInputs(int fd) const;
    bool readByte(int fd, uint8_t *value) const;
    void closeDevice(int *fd) const;
    void emitAction(const std::string &action);
    void setLastError(const std::string &error);

    ActionCallback actionCallback_;
    Config config_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    mutable std::mutex stateMutex_;
    std::string lastError_;
};

} // namespace apertar
