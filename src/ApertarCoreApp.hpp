#pragma once

#include "camera/CameraController.hpp"
#include "ipc/ControlProtocol.hpp"
#include "ipc/ControlServer.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

namespace apertar {

class ApertarCoreApp {
public:
    struct Options {
        std::string socketPath = "/tmp/apertar-core.sock";
        std::string mediaRoot = "/media/RAW";
        bool simulateCamera = false;
    };

    explicit ApertarCoreApp(Options options);
    ~ApertarCoreApp();

    bool start();
    void stop();
    void wait();

private:
    std::string handleRequest(const std::string &line);
    std::string handleCommand(const ControlCommand &command);
    void publishEvent(const std::string &eventJson);
    void publishPreviewFrame(const PreviewFrameDescriptor &frame);
    void requestShutdown();

    Options options_;
    CameraController camera_;
    ControlServer controlServer_;

    std::atomic<bool> running_{false};
    std::mutex waitMutex_;
    std::condition_variable waitCv_;

    std::mutex eventMutex_;
    bool collectingCommandEvents_ = false;
    std::vector<std::string> pendingCommandEvents_;
};

} // namespace apertar
