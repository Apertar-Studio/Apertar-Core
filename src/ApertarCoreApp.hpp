#pragma once

#include "camera/CameraController.hpp"
#include "audio/AudioRecordingSession.hpp"
#include "hardware/GpioInput.hpp"
#include "hardware/StandbyPowerButtonInput.hpp"
#include "hardware/HardwareScriptRunner.hpp"
#include "hardware/GroveAnalogInput.hpp"
#include "hardware/NeoRotary4Input.hpp"
#include "hardware/Pcf8574Input.hpp"
#include "hardware/RecordingIndicatorOutput.hpp"
#include "hardware/TriggerOutput.hpp"
#include "ipc/ControlProtocol.hpp"
#include "ipc/ControlServer.hpp"
#include "web/WebServer.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace apertar {

class ApertarCoreApp {
public:
    struct Options {
        std::string socketPath = "/tmp/apertar-core.sock";
        std::string mediaRoot = "/media/RAW";
        std::string webBindAddress = "0.0.0.0";
        uint16_t webPort = 8000;
        bool simulateCamera = false;
    };

    explicit ApertarCoreApp(Options options);
    ~ApertarCoreApp();

    bool start();
    void stop();
    void wait();
    void requestShutdown();

private:
    friend struct HardwareInputTestAccess;
    struct WebPreviewSlotMapping {
        int fd = -1;
        void *mapping = nullptr;
        size_t mappingLength = 0;
        uint8_t *data = nullptr;
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t stride = 0;
        int planeCount = 0;
        uint32_t planeOffsets[3] = { 0, 0, 0 };
        uint32_t planePitches[3] = { 0, 0, 0 };
        uint32_t planeLengths[3] = { 0, 0, 0 };
    };

    struct PendingWebPreviewFrame {
        uint32_t width = 0;
        uint32_t height = 0;
        uint64_t frameId = 0;
        int64_t timestampUs = 0;
        uint32_t planeOffsets[3] = { 0, 0, 0 };
        uint32_t planeStrides[3] = { 0, 0, 0 };
        std::vector<uint8_t> yuv;
    };

    std::string handleRequest(const std::string &line);
    std::string handleWebCommand(const std::string &payload);
    std::string handleCommand(const ControlCommand &command);
    WebServer::Response handleWebRequest(const WebServer::Request &request);
    std::string buildWebStateJson() const;
    std::string buildPresetLibraryJson() const;
    std::string buildMediaLibraryJson() const;
    void enqueuePreviewFrame(PreviewFrameDescriptor frame);
    void enqueueCurrentPreviewRegistrations();
    void updateWebPreviewRegistration(const PreviewFrameDescriptor &frame);
    void captureWebPreviewFrame(const PreviewFrameDescriptor &frame);
    void publishEvent(const std::string &eventJson);
    void handleTriggerEvent(const std::string &eventJson);
    void publishPreviewFrame(const PreviewFrameDescriptor &frame);
    void runPreviewPublisher();
    void stopPreviewPublisher();
    void runMjpegEncoder();
    void stopMjpegEncoder();
    void streamMjpeg(int clientFd);
    void syncRecordingTracking(bool active);
    double recordingElapsedSeconds() const;
    bool startSoundRecording(AudioRecordingOptions audioOptions, std::string *errorMessage);
    void stopSoundRecording();
    bool soundRecordingActive() const;
    bool startMediaSpeedTest(std::string *errorMessage);
    void runMediaSpeedTest(uint64_t testMiB, std::string mountPath);
    void joinMediaSpeedTest();
    void handleHardwareInputAction(const std::string &action, bool buttonPress = true);
    bool handlePowerButtonPress(bool doublePress);
    void handleSingleHardwareInputAction(const std::string &action);
    void handleHardwareAnalogInput(const GroveAnalogInput::AnalogEvent &event);
    std::string dispatchHardwareInputCommand(ControlCommand command);
    std::string toggleSharedUiBoolAction(const std::string &action,
                                         const std::string &commandName,
                                         bool currentValue);

    Options options_;
    CameraController camera_;
    ControlServer controlServer_;
    WebServer webServer_;
    GpioInput gpioInput_;
    StandbyPowerButtonInput standbyPowerButton_;
    GroveAnalogInput groveAnalogInput_;
    NeoRotary4Input neoRotary4Input_;
    Pcf8574Input pcf8574Input_;
    HardwareScriptRunner hardwareScriptRunner_;
    RecordingIndicatorOutput recordingIndicatorOutput_;
    TriggerOutput triggerOutput_;
    AudioRecordingSession soundRecorder_;
    std::atomic<bool> soundRecordingActive_{false};
    std::string soundRecordingFolder_;

    std::atomic<bool> running_{false};
    std::atomic<bool> shutdownRequested_{false};
    std::mutex waitMutex_;
    std::condition_variable waitCv_;
    std::mutex hardwareActionMutex_;
    std::mutex hardwareInputSequenceMutex_;
    std::mutex commandMutex_;
    std::atomic<int64_t> lastUserActivityMs_{0};
    std::atomic<bool> standbyUsbPoweredOff_{false};

    std::mutex eventMutex_;
    bool collectingCommandEvents_ = false;
    std::vector<std::string> pendingCommandEvents_;

    std::mutex previewMutex_;
    std::condition_variable previewCv_;
    std::deque<PreviewFrameDescriptor> pendingPreviewRegistrations_;
    std::optional<PreviewFrameDescriptor> pendingPreviewFrame_;
    std::optional<PreviewFrameDescriptor> lastPreviewFrame_;
    std::unordered_map<uint32_t, PreviewFrameDescriptor> registeredPreviewFrames_;
    bool previewPublisherRunning_ = false;
    std::thread previewThread_;

    std::mutex mjpegMutex_;
    std::condition_variable mjpegCv_;
    std::unordered_map<uint32_t, WebPreviewSlotMapping> webPreviewSlots_;
    std::optional<PendingWebPreviewFrame> pendingMjpegFrame_;
    std::vector<uint8_t> recycledMjpegYuv_;
    std::vector<uint8_t> latestMjpegFrame_;
    uint64_t latestMjpegGeneration_ = 0;
    bool mjpegEncoderRunning_ = false;
    std::thread mjpegThread_;
    std::atomic<uint32_t> mjpegClientCount_{0};
    std::chrono::steady_clock::time_point lastWebPreviewCaptureAt_{};

    mutable std::mutex telemetryMutex_;
    mutable bool cpuSampleValid_ = false;
    mutable uint64_t lastCpuIdleTicks_ = 0;
    mutable uint64_t lastCpuTotalTicks_ = 0;
    bool recordingStartKnown_ = false;
    std::chrono::steady_clock::time_point recordingStartedAt_{};

    mutable std::mutex mediaSpeedMutex_;
    bool mediaSpeedTestRunning_ = false;
    bool mediaSpeedCurrentUsedFio_ = false;
    bool mediaSpeedLastUsedFio_ = false;
    uint64_t mediaSpeedCurrentBytes_ = 0;
    uint64_t mediaSpeedLastBytes_ = 0;
    double mediaSpeedLastWriteSpeedMBps_ = 0.0;
    std::string mediaSpeedError_;
    std::thread mediaSpeedThread_;
};

} // namespace apertar
