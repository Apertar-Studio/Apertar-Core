#pragma once
#include <cstdint>

namespace apertar {
// Use evdev timestamps so delayed/batched reads preserve press spacing.
class PowerButtonGesture {
public:
    enum class Action { None, Press, DoublePress };
    Action event(int value, int64_t milliseconds)
    {
        if (value == 0 && down_) {
            down_ = false;
            released_ = candidate_ && milliseconds >= firstPress_ &&
                        milliseconds - firstPress_ <= 500;
            if (!released_) candidate_ = false;
        }
        if (value != 1 || down_) return Action::None;
        down_ = true;
        const auto spacing = milliseconds - firstPress_;
        if (candidate_ && released_ && spacing >= 80 && spacing <= 500) {
            consume();
            return Action::DoublePress;
        }
        if (candidate_ && spacing >= 0 && spacing < 80)
            return Action::None; // Contact bounce, not a second human press.
        candidate_ = true;
        released_ = false;
        firstPress_ = milliseconds;
        return Action::Press;
    }
    void consume() { candidate_ = false; released_ = false; }
    void reset() { consume(); down_ = false; }
private:
    bool down_ = false;
    bool candidate_ = false;
    bool released_ = false;
    int64_t firstPress_ = 0;
};
}
