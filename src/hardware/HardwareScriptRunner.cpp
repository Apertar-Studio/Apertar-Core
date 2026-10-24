#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "hardware/HardwareScriptRunner.hpp"
#include "hardware/ButtonActionList.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <system_error>

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

namespace apertar {
namespace {

std::string trim(std::string value)
{
    const auto notSpace = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
    return value;
}

std::vector<std::filesystem::path> readScriptPaths(const std::string &configPath, unsigned slot)
{
    if (std::filesystem::file_size(configPath) > 256 * 1024)
        throw std::runtime_error("hardware config exceeds 256 KiB");
    std::ifstream input(configPath);
    if (!input) throw std::runtime_error("cannot read hardware config " + configPath);
    bool section = false;
    bool explicitFirst = false;
    std::string line;
    std::array<std::string, hardware::maxHardwareScripts> paths;
    while (std::getline(input, line)) {
        line = trim(std::move(line));
        if (line.empty() || line.front() == '#' || line.front() == ';') continue;
        if (line.front() == '[') {
            section = line == "[run_script]";
            continue;
        }
        if (!section) continue;
        const auto equal = line.find('=');
        if (equal == std::string::npos) continue;
        const auto key = trim(line.substr(0, equal));
        if (key == "path" && explicitFirst) continue;
        if (key == "path1") explicitFirst = true;
        unsigned index = 0;
        if (key != "path") {
            if (key.size() != 5 || key.substr(0, 4) != "path" || key[4] < '1' || key[4] > '5') {
                if (key.rfind("path", 0) == 0) throw std::runtime_error("only path/path1 through path5 are supported");
                continue;
            }
            index = unsigned(key[4] - '1');
        }
        auto path = trim(line.substr(equal + 1));
        if (!path.empty() && (path.front() == '\'' || path.front() == '"')) {
            const auto end = path.find(path.front(), 1);
            if (end == std::string::npos) throw std::runtime_error("unterminated script path quote");
            const auto suffix = trim(path.substr(end + 1));
            if (!suffix.empty() && suffix.front() != '#' && suffix.front() != ';')
                throw std::runtime_error("unexpected text after script path");
            path = path.substr(1, end - 1);
        } else {
            path = trim(path.substr(0, path.find_first_of("#;")));
        }
        paths[index] = std::move(path);
    }
    std::vector<std::filesystem::path> scripts;
    for (unsigned index = 0; index < paths.size(); ++index) {
        if (slot && slot != index + 1) continue;
        const auto &path = paths[index];
        if (path.empty()) continue;
        const std::filesystem::path script(path);
        if (!script.is_absolute()) throw std::runtime_error("script path must be absolute");
        if (!std::filesystem::is_regular_file(script) || ::access(script.c_str(), R_OK) != 0)
            throw std::runtime_error("script is missing or unreadable: " + path);
        scripts.push_back(script);
    }
    if (scripts.empty()) throw std::runtime_error(slot ? "no script configured for slot " + std::to_string(slot)
        : "set [run_script] path (or path1) through path5 to Bash scripts");
    return scripts;
}

void checkSpawn(int error)
{
    if (error != 0) throw std::system_error(error, std::generic_category(), "script launch");
}

struct SpawnActions {
    posix_spawn_file_actions_t value;
    SpawnActions() { checkSpawn(::posix_spawn_file_actions_init(&value)); }
    ~SpawnActions() { ::posix_spawn_file_actions_destroy(&value); }
};

struct SpawnAttributes {
    posix_spawnattr_t value;
    SpawnAttributes() { checkSpawn(::posix_spawnattr_init(&value)); }
    ~SpawnAttributes() { ::posix_spawnattr_destroy(&value); }
};

} // namespace

HardwareScriptRunner::~HardwareScriptRunner()
{
    stop();
}

bool HardwareScriptRunner::request(const std::string &configPath, unsigned slot)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (configPath.empty() || slot > hardware::maxHardwareScripts) return false;
    const auto overlaps = [&](const Request &job) { return job.config == configPath && (!slot || !job.slot || job.slot == slot); };
    if (stopping_ || pending_.size() + (activeSlot_ >= 0 ? 1 : 0) >= hardware::maxHardwareScripts ||
        (activeSlot_ >= 0 && overlaps({activeConfig_, unsigned(activeSlot_)})) ||
        std::any_of(pending_.begin(), pending_.end(), overlaps)) {
        std::cerr << "ApertarCore run_script ignored: script already pending, queue full or stopping.\n";
        return false;
    }
    try {
        if (!worker_.joinable()) worker_ = std::thread(&HardwareScriptRunner::run, this);
    } catch (const std::exception &error) {
        std::cerr << "ApertarCore run_script: " << error.what() << '\n';
        return false;
    }
    pending_.push_back({configPath, slot});
    busy_ = true;
    cv_.notify_all();
    return true;
}

bool HardwareScriptRunner::busy() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return busy_;
}

void HardwareScriptRunner::stop()
{
    {
        std::unique_lock<std::mutex> lock(mutex_);
        stopping_ = true;
        pending_.clear();
        if (pid_ > 0) ::kill(-pid_, SIGTERM);
        cv_.notify_all();
        if (!cv_.wait_for(lock, std::chrono::seconds(2), [this] { return !busy_; }) && pid_ > 0)
            ::kill(-pid_, SIGKILL);
    }
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = false;
    stopping_ = false;
}

void HardwareScriptRunner::run()
{
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        cv_.wait(lock, [this] { return stopping_ || !pending_.empty(); });
        if (stopping_) break;
        const auto job = std::move(pending_.front());
        pending_.pop_front();
        activeConfig_ = job.config;
        activeSlot_ = int(job.slot);
        lock.unlock();
        try {
            execute(job.config, job.slot);
        } catch (const std::exception &error) {
            std::cerr << "ApertarCore run_script: " << error.what() << '\n';
        }
        lock.lock();
        activeConfig_.clear();
        activeSlot_ = -1;
        busy_ = !pending_.empty();
        pid_ = -1;
        cv_.notify_all();
    }
    busy_ = false;
    cv_.notify_all();
}

void HardwareScriptRunner::execute(const std::string &configPath, unsigned slot)
{
    for (const auto &script : readScriptPaths(configPath, slot)) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_) return;
        }
        try { executeScript(script); }
        catch (const std::exception &error) { std::cerr << "ApertarCore run_script: " << error.what() << '\n'; }
    }
}

void HardwareScriptRunner::executeScript(const std::filesystem::path &script)
{
    SpawnActions actions;
    SpawnAttributes attributes;
    checkSpawn(::posix_spawn_file_actions_addopen(&actions.value, STDIN_FILENO, "/dev/null", O_RDONLY, 0));
    checkSpawn(::posix_spawn_file_actions_addchdir_np(&actions.value, script.parent_path().c_str()));
    // Keep camera buffers, sockets and device handles out of user scripts.
#if defined(__GLIBC__) && __GLIBC_PREREQ(2, 34)
    checkSpawn(::posix_spawn_file_actions_addclosefrom_np(&actions.value, 3));
#else
    for (const auto &entry : std::filesystem::directory_iterator("/proc/self/fd")) {
        const int fd = std::stoi(entry.path().filename().string());
        if (fd >= 3) checkSpawn(::posix_spawn_file_actions_addclose(&actions.value, fd));
    }
#endif
    sigset_t empty, defaults;
    ::sigemptyset(&empty);
    ::sigemptyset(&defaults);
    for (int signal : {SIGTERM, SIGINT, SIGHUP, SIGPIPE}) ::sigaddset(&defaults, signal);
    checkSpawn(::posix_spawnattr_setsigmask(&attributes.value, &empty));
    checkSpawn(::posix_spawnattr_setsigdefault(&attributes.value, &defaults));
    checkSpawn(::posix_spawnattr_setpgroup(&attributes.value, 0));
    checkSpawn(::posix_spawnattr_setflags(&attributes.value,
        POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF));
    std::string path = script.string();
    char bash[] = "bash", separator[] = "--";
    char *argv[] = {bash, separator, path.data(), nullptr};
    pid_t child;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_) return;
        checkSpawn(::posix_spawn(&child, "/bin/bash", &actions.value, &attributes.value, argv, environ));
        pid_ = child;
    }
    std::cerr << "ApertarCore run_script started: pid=" << child << " path=" << path << '\n';
    int status = 0;
    pid_t result;
    do { result = ::waitpid(child, &status, 0); } while (result < 0 && errno == EINTR);
    // A script that is being cancelled must not leave background children behind.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_) ::kill(-child, SIGKILL);
        pid_ = -1;
    }
    if (result < 0) throw std::system_error(errno, std::generic_category(), "script wait");
    if (WIFEXITED(status))
        std::cerr << "ApertarCore run_script finished: pid=" << child << " exit=" << WEXITSTATUS(status) << '\n';
    else if (WIFSIGNALED(status))
        std::cerr << "ApertarCore run_script finished: pid=" << child << " signal=" << WTERMSIG(status) << '\n';
}

} // namespace apertar
