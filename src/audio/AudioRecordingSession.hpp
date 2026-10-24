#pragma once

#include <apertar/core/Types.hpp>
#include "RecordingClock.hpp"
#include "TimestampedAudioCapture.hpp"
#include "util/RecordingFailure.hpp"

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <fstream>
#include <mutex>
#include <string>
#include <sys/types.h>
#include <thread>

namespace apertar {

class AudioRecordingSession {
public:
    using AudioLevelCallback = std::function<void(int)>;

    AudioRecordingSession() = default;
    ~AudioRecordingSession();

    bool start(const std::filesystem::path &clipFolder,
               const AudioRecordingOptions &options,
               AudioLevelCallback levelCallback,
               std::string *error,
               const RecordingClock *clock = nullptr);
    void observeVideoFrame(uint64_t sensorBootUs, int64_t frameDurationUs);
    void stop();

    bool active() const;
    std::string error() const { return failure_.message(); }

private:
    void readerLoop();
    void finalizeWav();
    void resetState();
    void emitLevel(int level);
    static void writeWavHeader(std::ofstream &stream,
                               uint32_t sampleRate,
                               uint16_t channels,
                               uint16_t bitsPerSample,
                               bool floatingPoint,
                               uint64_t dataSize,
                               uint16_t firstChannel,
                               uint64_t clipStartWallClockUs,
                               const std::string &originatorReference,
                               TimecodeMode timecodeMode,
                               double fps);

    AudioRecordingOptions options_{};
    TimestampedAudioCapture capture_;
    RecordingClock recordingClock_;
    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> sessionActive_{false};
    RecordingFailure failure_;
    std::filesystem::path clipFolder_;
    std::filesystem::path rawPath_;
    std::filesystem::path wavPath_;
    std::ofstream rawStream_;
    AudioLevelCallback levelCallback_;

    std::thread readerThread_;
    std::atomic<bool> running_{false};
    std::atomic<pid_t> playbackPid_{-1};
    int playbackWriteFd_ = -1;
    int playbackStderrFd_ = -1;

    mutable std::mutex syncMutex_;
    uint64_t audioStartBootUs_ = 0;
    uint64_t videoFirstBootUs_ = 0;
    uint64_t videoEndBootUs_ = 0;
    uint64_t capturedBytes_ = 0;
    uint64_t audioEndBootUs_ = 0;
};

} // namespace apertar
