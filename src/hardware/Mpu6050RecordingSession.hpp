#pragma once

#include <apertar/core/Types.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

namespace apertar {

class Mpu6050RecordingSession {
public:
    Mpu6050RecordingSession() = default;
    ~Mpu6050RecordingSession();

    bool start(const std::string &clipFolder,
               const GyroRecordingOptions &options,
               std::string *error);
    void stop();
    void setVideoStartTimestampUs(uint64_t videoStartTimestampUs, uint64_t frameIndex);

    struct CalibrationResult {
        double gyroBiasXRaw = 0.0;
        double gyroBiasYRaw = 0.0;
        double gyroBiasZRaw = 0.0;
        uint32_t sampleCount = 0;
        std::string configPath;
    };

    struct ConfigSnapshot {
        bool enabled = true;
        std::string driver = "mpu6050";
        std::string driverLabel = "MPU6050";
        std::string syncMode = "free_run";
        int busNumber = 1;
        int address = 0x68;
        int sampleRateHz = 500;
        int gyroRangeDps = 1000;
        int accelRangeG = 8;
        std::string orientation = "XYZ";
        double syncOffsetMs = 0.0;
        std::string syncOffsetText = "0 ms";
        bool levelReferenceSet = false;
        std::string levelReferenceText = "Not set";
        double levelAccelXG = 0.0;
        double levelAccelYG = 0.0;
        double levelAccelZG = 0.0;
        bool liveAvailable = false;
        std::string liveStatusText = "Live gyro idle";
        double accelXG = 0.0;
        double accelYG = 0.0;
        double accelZG = 0.0;
        double gyroXDps = 0.0;
        double gyroYDps = 0.0;
        double gyroZDps = 0.0;
        std::string configPath;
    };

    struct LiveSample {
        double accelXG = 0.0;
        double accelYG = 0.0;
        double accelZG = 0.0;
        double gyroXDps = 0.0;
        double gyroYDps = 0.0;
        double gyroZDps = 0.0;
        std::string statusText;
    };

    static bool calibrateConfiguredSensor(double seconds,
                                          CalibrationResult *result,
                                          std::string *error);
    static ConfigSnapshot configuredSensorSnapshot(bool includeLiveSample);
    static bool setConfiguredDriver(const std::string &driver, std::string *error);
    static bool setConfiguredOrientation(const std::string &orientation, std::string *error);
    static bool setConfiguredSyncOffsetMs(double value, std::string *error);
    static bool setConfiguredLevelReference(std::string *error);
    static bool clearConfiguredLevelReference(std::string *error);

    bool active() const;
    std::string lastError() const;

    enum class DriverKind {
        Mpu6050,
        Bmi160,
        Bmi270,
        Icm42688p
    };

private:
    struct Config {
        bool enabled = true;
        std::string driver = "mpu6050";
        std::string syncMode = "free_run";
        int busNumber = 1;
        int address = 0x68;
        int sampleRateHz = 500;
        int gyroRangeDps = 1000;
        int accelRangeG = 8;
        int dlpfHz = 98;
        bool useFifo = true;
        int fifoBatchSamples = 24;
        std::string orientation = "XYZ";
        double autoCalibrateSeconds = 0.0;
        double gyroBiasXRaw = 0.0;
        double gyroBiasYRaw = 0.0;
        double gyroBiasZRaw = 0.0;
        double syncOffsetMs = 0.0;
        bool levelReferenceSet = false;
        std::string levelReferenceOrientation;
        double levelAccelXG = 0.0;
        double levelAccelYG = 0.0;
        double levelAccelZG = 0.0;
        double frameReadoutTimeMs = 0.0;
        int frameReadoutDirection = 0;
        std::string configPath;
    };

    static Config loadConfig();
    bool openDevice(std::string *error);
    bool configureSensor(std::string *error);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t *data, size_t length);
    uint8_t chipIdRegister() const;
    bool chipIdMatches(uint8_t chipId) const;
    bool readChipId(uint8_t *chipId);
    bool probeConfiguredDevice();
    bool ensureConfiguredDevice(std::string *error);
    bool readLiveSample(LiveSample *sample);
    bool resetFifo();
    void run();
    void closeDevice();
    void setLastError(const std::string &error);

    Config config_{};
    std::atomic<bool> running_{false};
    std::thread thread_;
    mutable std::mutex stateMutex_;
    std::ofstream output_;
    int fd_ = -1;
    DriverKind driverKind_ = DriverKind::Mpu6050;
    std::string driverLabel_ = "MPU6050";
    double gyroScaleDpsPerLsb_ = 32.8;
    double accelScaleGPerLsb_ = 4096.0;
    bool fifoEnabled_ = false;
    uint32_t samplePeriodUs_ = 5000;
    std::atomic<bool> timelineAnchored_{false};
    std::atomic<uint64_t> videoStartTimestampUs_{0};
    std::atomic<uint64_t> videoStartFrameIndex_{std::numeric_limits<uint64_t>::max()};
    std::string outputPath_;
    std::string lastError_;
};

} // namespace apertar
