#pragma once

#include <apertar/core/Types.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace apertar {

class CameraController {
public:
    using EventCallback = std::function<void(const std::string &)>;
    using PreviewCallback = std::function<void(const PreviewFrameDescriptor &)>;

    CameraController(EventCallback eventCallback, PreviewCallback previewCallback);
    ~CameraController();

    bool start(const std::string &mediaRoot, bool simulateCamera);
    void stop();

    bool setResolution(Resolution resolution);
    bool setFps(double fps);
    bool setIso(uint32_t iso);
    bool setIsoAuto();
    bool setShutterUs(uint32_t shutterUs);
    bool setShutterAuto();
    bool setWhiteBalanceKelvin(uint32_t kelvin);

    bool capturePhoto();
    bool startRecording();
    bool stopRecording();

    CameraState state() const;
    CaptureSettings settings() const;
    bool isRecording() const;
    std::string lastError() const;

private:
    class CameraSession;

    bool applySettingsLocked(const CaptureSettings &nextSettings, bool requiresReconfigure);
    bool validateResolution(Resolution resolution);
    void setErrorLocked(std::string error);
    void emitSettingsEventLocked();
    void emitRecordingEventLocked();

    mutable std::mutex mutex_;
    EventCallback eventCallback_;
    PreviewCallback previewCallback_;
    CameraState state_ = CameraState::Offline;
    CaptureSettings settings_{};
    std::string mediaRoot_;
    std::string lastError_;
    bool simulateCamera_ = false;
    bool recording_ = false;
    std::unique_ptr<CameraSession> session_;
};

} // namespace apertar
