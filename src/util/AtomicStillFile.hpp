#pragma once

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <stdexcept>
#include <system_error>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace apertar {

inline void syncStillDirectory(const std::filesystem::path &path)
{
    const int fd = ::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (fd < 0)
        throw std::system_error(errno, std::generic_category(), "Could not open still directory");
    int result;
    do { result = ::fsync(fd); } while (result < 0 && errno == EINTR);
    const int error = errno;
    ::close(fd);
    if (result < 0)
        throw std::system_error(error, std::generic_category(), "Could not sync still directory");
}

// Stills are published only after their data is durable. Keep this out of the
// continuous recording path: syncing each video frame would destroy throughput.
class AtomicStillFile {
public:
    explicit AtomicStillFile(const std::filesystem::path &finalPath)
        : finalPath_(finalPath), temporary_(finalPath.string() + ".writing-XXXXXX")
    {
        fd_ = ::mkostemp(temporary_.data(), O_CLOEXEC);
        if (fd_ < 0)
            throw std::system_error(errno, std::generic_category(), "Could not create temporary still");
    }
    ~AtomicStillFile()
    {
        if (fd_ >= 0) ::close(fd_);
        if (!temporary_.empty()) ::unlink(temporary_.c_str());
    }
    AtomicStillFile(const AtomicStillFile &) = delete;
    AtomicStillFile &operator=(const AtomicStillFile &) = delete;

    int fd() const { return fd_; }
    const std::string &path() const { return temporary_; }

    void write(const void *bytes, size_t size)
    {
        const auto *data = static_cast<const char *>(bytes);
        size_t offset = 0;
        while (offset < size) {
            const auto written = ::write(fd_, data + offset, std::min<size_t>(size - offset, 1024 * 1024));
            if (written < 0 && errno == EINTR) continue;
            if (written <= 0)
                throw std::system_error(written < 0 ? errno : EIO, std::generic_category(), "Could not write still");
            offset += static_cast<size_t>(written);
        }
    }

    void commit(uint64_t minimumBytes = 1)
    {
        struct stat info{};
        if (::fstat(fd_, &info) < 0)
            throw std::system_error(errno, std::generic_category(), "Could not check completed still");
        if (minimumBytes == 0 || info.st_size < 0 || static_cast<uint64_t>(info.st_size) < minimumBytes)
            throw std::runtime_error("Still output is empty or incomplete; it was not saved.");
        // FAT/exFAT mount masks may reject POSIX permission changes.
        if (::fchmod(fd_, 0644) < 0 && errno != EPERM && errno != EOPNOTSUPP)
            throw std::system_error(errno, std::generic_category(), "Could not set still permissions");
        int result;
        do { result = ::fsync(fd_); } while (result < 0 && errno == EINTR);
        if (result < 0)
            throw std::system_error(errno, std::generic_category(), "Could not sync still image");
        const int closed = ::close(fd_);
        fd_ = -1;
        if (closed < 0)
            throw std::system_error(errno, std::generic_category(), "Could not close still image");

        // Do not replace a previous photo, even if another process creates its
        // name after allocation or the camera restarts with an old temp file.
        if (::renameat2(AT_FDCWD, temporary_.c_str(), AT_FDCWD, finalPath_.c_str(), RENAME_NOREPLACE) < 0)
            throw std::system_error(errno, std::generic_category(), "Could not finalize still image");
        temporary_.clear();
        syncStillDirectory(finalPath_.parent_path().empty() ? "." : finalPath_.parent_path());
    }

private:
    std::filesystem::path finalPath_;
    std::string temporary_;
    int fd_ = -1;
};
}
