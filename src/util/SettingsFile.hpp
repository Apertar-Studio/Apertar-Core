#pragma once
#include <chrono>
#include <cerrno>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <fcntl.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace apertar::settingsFile {
inline std::string line(const std::filesystem::path &path)
{
    std::ifstream input(path);
    std::string value;
    std::getline(input, value);
    return value;
}
inline bool writeAll(int fd, const std::string &data)
{
    size_t offset = 0;
    while (offset < data.size()) {
        const auto bytes = ::write(fd, data.data() + offset, data.size() - offset);
        if (bytes < 0 && errno == EINTR) continue;
        if (bytes <= 0) return false;
        offset += size_t(bytes);
    }
    return true;
}
inline bool atomicWrite(const std::filesystem::path &path, const std::string &data)
{
    std::error_code error;
    std::filesystem::create_directories(path.parent_path(), error);
    if (error) return false;
    std::string temporary = path.string() + ".XXXXXX";
    const int fd = ::mkstemp(temporary.data());
    if (fd < 0) return false;
    struct stat previous {};
    bool ok = ::stat(path.c_str(), &previous) != 0 ||
              ::fchmod(fd, previous.st_mode & 0777) == 0;
    ok = ok && writeAll(fd, data) && ::fsync(fd) == 0;
    if (::close(fd) != 0) ok = false;
    if (ok) ok = ::rename(temporary.c_str(), path.c_str()) == 0;
    if (!ok) { ::unlink(temporary.c_str()); return false; }
    const int directory = ::open(path.parent_path().c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (directory < 0) return false;
    ok = ::fsync(directory) == 0;
    ::close(directory);
    return ok;
}

// QSettings uses QLockFile on <file>.lock. Cooperate without linking Qt into Core.
class QtIniLock {
public:
    explicit QtIniLock(const std::filesystem::path &file) : path_(file.string() + ".lock")
    {
        char host[256] {};
        if (::gethostname(host, sizeof(host)-1) != 0) return;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        do {
            fd_ = ::open(path_.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
            if (fd_ >= 0) {
                struct flock native {};
                native.l_type = F_WRLCK; native.l_whence = SEEK_SET;
                std::error_code error;
                const auto executable = std::filesystem::read_symlink("/proc/self/exe", error).filename().string();
                const auto contents = std::to_string(::getpid()) + "\n" + executable + "\n" + host + "\n" +
                    line("/etc/machine-id") + "\n" + line("/proc/sys/kernel/random/boot_id") + "\n";
                if (::flock(fd_, LOCK_EX | LOCK_NB) == 0 && ::fcntl(fd_, F_SETLK, &native) == 0 &&
                    writeAll(fd_, contents)) return;
                ::unlink(path_.c_str()); ::close(fd_); fd_ = -1;
                return;
            }
            if (errno != EEXIST) return;
            removeDeadOwner(host);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while (std::chrono::steady_clock::now() < deadline);
    }
    ~QtIniLock() { if (fd_ >= 0) { ::unlink(path_.c_str()); ::close(fd_); } }
    QtIniLock(const QtIniLock &) = delete;
    QtIniLock &operator=(const QtIniLock &) = delete;
    explicit operator bool() const { return fd_ >= 0; }
private:
    void removeDeadOwner(const std::string &host)
    {
        const int fd = ::open(path_.c_str(), O_RDWR | O_CLOEXEC | O_NOFOLLOW);
        if (fd < 0) return;
        struct flock native {};
        native.l_type = F_WRLCK; native.l_whence = SEEK_SET;
        if (::flock(fd, LOCK_EX | LOCK_NB) == 0 && ::fcntl(fd, F_SETLK, &native) == 0) {
            std::ifstream input(path_);
            std::string pid, executable, ownerHost, machine, boot;
            std::getline(input, pid); std::getline(input, executable); std::getline(input, ownerHost);
            std::getline(input, machine); std::getline(input, boot);
            bool dead = false;
            try {
                const auto owner = std::stol(pid);
                const bool sameHost = machine.empty() ? ownerHost == host : machine == line("/etc/machine-id");
                dead = sameHost && owner > 0 && owner <= INT32_MAX &&
                    ((!boot.empty() && boot != line("/proc/sys/kernel/random/boot_id")) ||
                     (::kill(pid_t(owner), 0) != 0 && errno == ESRCH));
            } catch (...) {}
            struct stat held {}, current {};
            if (dead && ::fstat(fd, &held) == 0 && ::lstat(path_.c_str(), &current) == 0 &&
                held.st_dev == current.st_dev && held.st_ino == current.st_ino)
                ::unlink(path_.c_str());
        }
        ::close(fd);
    }
    std::filesystem::path path_;
    int fd_ = -1;
};
}
