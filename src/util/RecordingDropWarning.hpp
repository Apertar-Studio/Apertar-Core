#pragma once

#include <atomic>
#include <functional>
#include <utility>

namespace apertar {

// A clip-scoped latch: notify once, not once per frame or per dropped frame.
class RecordingDropWarning {
public:
    explicit RecordingDropWarning(std::function<void(bool)> changed)
        : changed_(std::move(changed)) {}

    bool dropped() const { return dropped_.load(std::memory_order_acquire); }
    void markDropped() {
        if (!dropped_.exchange(true, std::memory_order_acq_rel) && changed_)
            changed_(true);
    }
    void reset() {
        if (dropped_.exchange(false, std::memory_order_acq_rel) && changed_)
            changed_(false);
    }

private:
    std::atomic<bool> dropped_{false};
    const std::function<void(bool)> changed_;
};

} // namespace apertar
