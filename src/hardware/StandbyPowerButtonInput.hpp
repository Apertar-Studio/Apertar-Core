#pragma once
#include <atomic>
#include <functional>
#include <thread>
namespace apertar {
class StandbyPowerButtonInput {
public:
    explicit StandbyPowerButtonInput(std::function<bool(bool)> pressed) : pressed_(std::move(pressed)) {}
    ~StandbyPowerButtonInput() { stop(); }
    void start();
    void stop();
private:
    void run();
    std::function<bool(bool)> pressed_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    int stopFd_ = -1;
};
}
