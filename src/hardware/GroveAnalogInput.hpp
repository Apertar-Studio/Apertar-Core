#pragma once

#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace apertar {

class GroveAnalogInput {
public:
    struct AnalogEvent {
        std::string target;
        double normalized = 0.0;
        int raw = 0;
        double minValue = std::numeric_limits<double>::quiet_NaN();
        double maxValue = std::numeric_limits<double>::quiet_NaN();
        double step = std::numeric_limits<double>::quiet_NaN();
        bool usePresets = false;
    };

    using AnalogCallback = std::function<void(const AnalogEvent &)>;

    explicit GroveAnalogInput(AnalogCallback analogCallback);
    ~GroveAnalogInput();

    bool start();
    void stop();

    bool enabled() const;
    std::string configPath() const;
    std::string lastError() const;

private:
    struct ChannelMapping {
        int channel = -1;
        std::string target;
        bool invert = false;
        int deadband = 32;
        double minValue = std::numeric_limits<double>::quiet_NaN();
        double maxValue = std::numeric_limits<double>::quiet_NaN();
        double step = std::numeric_limits<double>::quiet_NaN();
        bool usePresets = false;
        bool haveLastRaw = false;
        int lastRaw = 0;
    };

    struct Config {
        bool configured = false;
        bool enabled = false;
        int busNumber = 1;
        int address = 0x04;
        int pollMs = 80;
        int deadband = 32;
        bool emitOnStart = false;
        std::vector<ChannelMapping> channels;
        std::string configPath;
    };

    static Config loadConfig();
    void run();
    bool openDevice(int *fd, std::string *errorMessage) const;
    bool readRaw(int fd, int channel, int *raw) const;
    void closeDevice(int *fd) const;
    void emitAnalog(const ChannelMapping &channel, int raw);
    void setLastError(const std::string &error);

    AnalogCallback analogCallback_;
    Config config_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    mutable std::mutex stateMutex_;
    std::string lastError_;
};

} // namespace apertar
