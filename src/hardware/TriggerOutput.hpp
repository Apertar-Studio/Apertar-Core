#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <set>
#include <string>
#include <thread>

namespace apertar {

class TriggerOutput {
public:
    enum class Event {
        RecordingStart,
        Still,
        TimelapseStart,
        TimelapseFrame,
    };

    TriggerOutput() = default;
    ~TriggerOutput();

    TriggerOutput(const TriggerOutput &) = delete;
    TriggerOutput &operator=(const TriggerOutput &) = delete;

    bool start();
    void stop();
    void pulse(Event event);

    bool enabled() const;
    std::string configPath() const;
    std::string lastError() const;

private:
    enum class Target {
        Gpio,
        Pcf8574,
    };

    struct Config {
        bool configured = false;
        bool enabled = false;
        Target target = Target::Gpio;
        std::string gpioChipPath = "auto";
        int gpioLine = 17;
        bool gpioOpenDrain = false;
        int pcfBusNumber = 1;
        int pcfAddress = 0x20;
        int pcfPin = 0;
        bool activeHigh = false;
        uint32_t pulseMs = 80;
        std::set<Event> events;
        std::string configPath;
    };

    static Config loadConfig();

    void workerLoop();
    bool eventEnabledLocked(Event event) const;
    bool openOutputLocked(std::string *errorMessage);
    bool openGpioLocked(std::string *errorMessage);
    bool requestGpioLineFromChipLocked(const std::string &chipPath, std::string *errorMessage);
    bool openPcf8574Locked(std::string *errorMessage);
    bool setOutputActiveLocked(bool active, std::string *errorMessage);
    void closeOutputLocked();
    void setLastErrorLocked(const std::string &error);

    Target target_ = Target::Gpio;
    std::string gpioChipPath_ = "auto";
    int gpioLine_ = 17;
    bool gpioOpenDrain_ = false;
    int pcfBusNumber_ = 1;
    int pcfAddress_ = 0x20;
    int pcfPin_ = 0;
    uint8_t pcfState_ = 0xff;
    bool activeHigh_ = false;
    uint32_t pulseMs_ = 80;
    std::set<Event> events_;
    std::string configPath_;

    int gpioLineFd_ = -1;
    int pcfFd_ = -1;

    mutable std::mutex stateMutex_;
    std::condition_variable cv_;
    std::thread worker_;
    bool enabled_ = false;
    bool running_ = false;
    uint32_t pendingPulses_ = 0;
    std::string lastError_;
};

} // namespace apertar
