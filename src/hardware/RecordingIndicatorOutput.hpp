#pragma once

#include <cstdint>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace apertar {

class RecordingIndicatorOutput {
public:
    RecordingIndicatorOutput() = default;
    ~RecordingIndicatorOutput();

    RecordingIndicatorOutput(const RecordingIndicatorOutput &) = delete;
    RecordingIndicatorOutput &operator=(const RecordingIndicatorOutput &) = delete;

    bool start();
    void stop();
    void setRecordingActive(bool active);
    void setStillExposureActive(bool active);
    void pulseCapture();

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
        int gpioLine = -1;
        bool gpioOpenDrain = false;
        int pcfBusNumber = 1;
        int pcfAddress = 0x20;
        int pcfPin = -1;
        bool activeHigh = false;
        std::string configPath;
        std::string source;
    };

    static Config loadConfig();

    void workerLoop();
    bool openOutputLocked(std::string *errorMessage);
    bool openGpioLocked(std::string *errorMessage);
    bool requestGpioLineFromChipLocked(const std::string &chipPath, std::string *errorMessage);
    bool openPcf8574Locked(std::string *errorMessage);
    bool setOutputActiveLocked(bool active, std::string *errorMessage);
    void closeOutputLocked();
    void setLastErrorLocked(const std::string &error);

    Target target_ = Target::Gpio;
    std::string gpioChipPath_ = "auto";
    int gpioLine_ = -1;
    bool gpioOpenDrain_ = false;
    int pcfBusNumber_ = 1;
    int pcfAddress_ = 0x20;
    int pcfPin_ = -1;
    uint8_t pcfState_ = 0xff;
    bool activeHigh_ = false;
    std::string configPath_;
    std::string source_;

    int gpioLineFd_ = -1;
    int pcfFd_ = -1;

    mutable std::mutex stateMutex_;
    std::condition_variable cv_;
    std::thread worker_;
    bool running_ = false;
    bool enabled_ = false;
    bool active_ = false;
    bool outputDirty_ = false;
    bool recordingActive_ = false;
    bool stillExposureActive_ = false;
    std::chrono::steady_clock::time_point capturePulseUntil_{};
    std::string lastError_;
};

} // namespace apertar
