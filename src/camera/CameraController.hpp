#pragma once

#include <apertar/core/Types.hpp>
#include "util/RecordingDropWarning.hpp"

#include <condition_variable>
#include <functional>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace apertar {

class CameraController {
public:
    using EventCallback = std::function<void(const std::string &)>;
    using PreviewCallback = std::function<void(PreviewFrameDescriptor)>;
    using AudioLevelCallback = std::function<void(int)>;
    struct StillExposureStatus {
        bool active = false;
        double remainingSeconds = 0;
    };
    struct MediaWriteStatus {
        bool pending = false;
        uint64_t revision = 0;
    };

    CameraController(EventCallback eventCallback, PreviewCallback previewCallback);
    ~CameraController();

    bool start(const std::string &mediaRoot, bool simulateCamera);
    void stop();
    bool enterStandby();
    bool wakeFromStandby();

    bool setResolution(Resolution resolution);
    bool setFps(double fps);
    bool setIso(uint32_t iso);
    bool setIsoAuto();
    bool setShutterUs(uint32_t shutterUs);
    bool setShutterAuto();
    bool setExposureCompensation(double ev);
    bool setExposureMetering(const std::string &mode);
    bool setStillExposureMode(bool photoMode, bool longExposure, double timelapseIntervalSeconds = 0);
    bool setWhiteBalanceKelvin(uint32_t kelvin);
    bool setWhiteBalanceAuto();
    bool setRecordingFormat(RecordingFormat format);
    bool setCdngCompression(CdngCompression compression);
    bool setLossyDngRatio(LossyDngRatio ratio);
    bool setProresProfile(ProresProfile profile);
    bool setProresColorProfile(ProresColorProfile profile);
    bool setCdngPreviewColorProfile(ProresColorProfile profile);
    bool setTuningFile(const std::string &path);
    bool setRawBitDepth(uint32_t bitDepth);
    bool setH264Resolution(Resolution resolution);
    bool setTimecodeMode(TimecodeMode mode);
    bool setAutofocusMode(AutofocusMode mode);
    bool setAutofocusRange(AutofocusRange range);
    bool setAutofocusSpeed(AutofocusSpeed speed);
    bool setAutofocusWindow(double x, double y, double width, double height);
    bool applyCapturePreset(const CaptureSettings &settings, const std::function<bool()> &commitUi = {});
    bool checkRecordingFailure();
    bool mediaWritesPending() const;
    MediaWriteStatus mediaWriteStatus() const;
    StillExposureStatus stillExposureStatus() const;
    std::string photoWriteError() const;

    bool capturePhoto(PhotoCaptureFormat format = PhotoCaptureFormat::Dng, unsigned count = 1);
    bool startTimelapse(PhotoCaptureFormat format, double intervalSeconds);
    bool stopTimelapse();
    bool timelapseActive() const;
    double timelapseIntervalSeconds() const;
    uint64_t timelapseShotCount() const;
    bool startRecording(const AudioRecordingOptions &audioOptions = {},
                        const GyroRecordingOptions &gyroOptions = {});
    bool stopRecording();

    CameraState state() const;
    CaptureSettings settings() const;
    std::string sensorName() const;
    bool isRecording() const;
    std::string lastError() const;
    std::string recordingError() const;
    bool recordingHasDroppedFrames() const { return recordingDropWarning_.dropped(); }

private:
    class CameraSession;

    bool applySettingsLocked(CaptureSettings nextSettings, bool requiresReconfigure, bool commit = true);
    void persistSettingsLocked();
    void restoreModeGeometryLocked(CaptureSettings &next) const;
    bool validateResolution(Resolution resolution);
    void setErrorLocked(std::string error);
    void emitSettingsEventLocked();
    void emitRecordingEventLocked();
    bool queuePhotoCaptureLocked(PhotoCaptureFormat format, const std::string &basePath,
                                 std::function<void()> onSaved = {}, unsigned count = 1);
    void completeTimelapseShot(uint64_t sequence);
    void emitTimelapseProgressLocked();
    void timelapseLoop();
    void emitTimelapseEvent(bool active, double intervalSeconds, const std::string &folderPath);
    void emitTimelapseFrameEvent(uint64_t frameIndex);

    mutable std::mutex mutex_;
    mutable std::mutex timelapseMutex_;
    std::condition_variable timelapseCv_;
    std::thread timelapseThread_;
    EventCallback eventCallback_;
    PreviewCallback previewCallback_;
    RecordingDropWarning recordingDropWarning_;
    CameraState state_ = CameraState::Offline;
    CaptureSettings settings_{};
    std::map<RecordingFormat, CaptureSettings> formatSettings_;
    std::optional<CaptureSettings> stillSettings_;
    std::string mediaRoot_;
    std::string lastError_;
    bool simulateCamera_ = false;
    bool recording_ = false;
    bool recordingFailureReported_ = false;
    std::string recordingFailureMessage_;
    bool timelapseStopRequested_ = false;
    bool timelapseActive_ = false;
    double timelapseIntervalSeconds_ = 0;
    uint64_t timelapseFrameIndex_ = 1;
    uint64_t timelapseShotCount_ = 0;
    uint64_t timelapseSequence_ = 0;
    PhotoCaptureFormat timelapseFormat_ = PhotoCaptureFormat::Dng;
    std::string timelapseFolderPath_;
    std::unique_ptr<CameraSession> session_;
};

} // namespace apertar
