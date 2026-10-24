#pragma once
#include <atomic>
#include <mutex>
#include <string>

namespace apertar {
// Worker threads report failures; the controller stops them from its supervisor.
class RecordingFailure {
public:
    void reset() { std::lock_guard lock(mutex_); message_.clear(); failed_.store(false); }
    void set(std::string message) {
        std::lock_guard lock(mutex_);
        if (failed_.load(std::memory_order_relaxed)) return;
        message_ = message.empty() ? "Recording write failed." : std::move(message);
        failed_.store(true, std::memory_order_release);
    }
    std::string message() const {
        if (!failed_.load(std::memory_order_acquire)) return {};
        std::lock_guard lock(mutex_); return message_;
    }
private:
    mutable std::mutex mutex_;
    std::atomic<bool> failed_{false};
    std::string message_;
};
}
