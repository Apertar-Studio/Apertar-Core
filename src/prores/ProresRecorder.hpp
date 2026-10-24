#pragma once
#include "ProresPluginApi.hpp"
#include "util/RecordingFailure.hpp"
#include "util/RecordingDropWarning.hpp"
#include <apertar/core/Types.hpp>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace apertar {
class ProresRecorder {
public:
    explicit ProresRecorder(RecordingDropWarning *warning = nullptr) : dropWarning_(warning) {}
    ~ProresRecorder();
    bool startAtFolder(const std::filesystem::path &, const CaptureSettings &, uint32_t, uint32_t, std::string *, size_t cameraBuffers = 12);
    bool enqueueFrame(const uint8_t *, size_t, uint32_t, uint32_t, uint32_t, int64_t, std::function<void()>);
    void stop();
    bool active() const { return accepting_.load(std::memory_order_acquire); }
    std::string error() const { return failure_.message(); }
private:
    struct Job {
        ApertarProresFrame frame{};
        std::function<void()> release;
    };
    bool load(std::string *);
    void run();
    void *library_ = nullptr;
    void *encoder_ = nullptr;
    ApertarProresCreate create_ = nullptr;
    ApertarProresEncode encode_ = nullptr;
    ApertarProresFinish finish_ = nullptr;
    ApertarProresDestroy destroy_ = nullptr;
    std::atomic<bool> accepting_{false};
    RecordingFailure failure_;
    RecordingDropWarning *dropWarning_ = nullptr;
    bool stopping_ = false;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Job> queue_;
    std::thread worker_;
    uint64_t submitted_ = 0, written_ = 0, dropped_ = 0, failures_ = 0;
    size_t peak_ = 0;
    size_t queueCapacity_ = 2;
};
}
