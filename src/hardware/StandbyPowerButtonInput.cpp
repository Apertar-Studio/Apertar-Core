#include "StandbyPowerButtonInput.hpp"
#include "PowerButtonGesture.hpp"
#include <filesystem>
#include <fstream>
#include <string>
#include <chrono>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <unistd.h>
#include <sys/eventfd.h>
namespace apertar {
void StandbyPowerButtonInput::start()
{
    if (!running_.exchange(true)) {
        stopFd_ = ::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
        if (stopFd_ < 0) { running_.store(false); return; }
        thread_ = std::thread(&StandbyPowerButtonInput::run, this);
    }
}
void StandbyPowerButtonInput::stop()
{
    running_.store(false);
    if (stopFd_ >= 0) {
        const uint64_t signal = 1;
        ::write(stopFd_, &signal, sizeof(signal));
    }
    if (thread_.joinable()) thread_.join();
    if (stopFd_ >= 0) { ::close(stopFd_); stopFd_ = -1; }
}
void StandbyPowerButtonInput::run()
{
    int fd = -1;
    PowerButtonGesture gesture;
    while (running_.load()) {
        if (fd < 0) {
            std::error_code error;
            for (const auto &entry : std::filesystem::directory_iterator("/sys/class/input", error)) {
                const std::string name = entry.path().filename();
                if (name.rfind("event", 0) != 0) continue;
                std::ifstream description(entry.path() / "device/name");
                std::string deviceName;
                std::getline(description, deviceName);
                if (deviceName == "pwr_button") {
                    fd = ::open(("/dev/input/" + name).c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
                    break;
                }
            }
            if (fd < 0) {
                pollfd stop{stopFd_, POLLIN, 0};
                ::poll(&stop, 1, 1500);
                continue;
            }
        }
        pollfd descriptors[] = {{fd, POLLIN, 0}, {stopFd_, POLLIN, 0}};
        if (::poll(descriptors, 2, -1) <= 0) continue;
        if (descriptors[1].revents) break;
        if (descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            ::close(fd); fd = -1; gesture.reset(); continue;
        }
        input_event events[16];
        const ssize_t bytes = ::read(fd, events, sizeof(events));
        if (bytes == 0) { ::close(fd); fd = -1; gesture.reset(); continue; }
        for (ssize_t index = 0; index < bytes / ssize_t(sizeof(input_event)); ++index) {
            const auto &event = events[index];
            if (event.type != EV_KEY || event.code != KEY_POWER) continue;
            const auto action = gesture.event(event.value,
                int64_t(event.time.tv_sec) * 1000 + event.time.tv_usec / 1000);
            if (action != PowerButtonGesture::Action::None && pressed_ &&
                pressed_(action == PowerButtonGesture::Action::DoublePress))
                gesture.consume(); // A wake press cannot seed another sleep.
        }
    }
    if (fd >= 0) ::close(fd);
}
}
