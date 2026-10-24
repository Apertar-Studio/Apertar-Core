#pragma once

#include <condition_variable>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>

#include <sys/types.h>

namespace apertar {

class HardwareScriptRunner {
public:
    ~HardwareScriptRunner();

    bool request(const std::string &configPath, unsigned slot = 0);
    bool busy() const;
    void stop();

private:
    void run();
    void execute(const std::string &configPath, unsigned slot);
    void executeScript(const std::filesystem::path &script);

    struct Request { std::string config; unsigned slot; };

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::thread worker_;
    std::deque<Request> pending_;
    std::string activeConfig_;
    int activeSlot_ = -1;
    bool busy_ = false;
    bool stopping_ = false;
    pid_t pid_ = -1;
};

} // namespace apertar
