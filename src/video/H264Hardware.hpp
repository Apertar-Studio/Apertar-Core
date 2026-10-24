#pragma once

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace apertar {

inline bool raspberryPiH264Encoder(const v4l2_capability &cap)
{
    const auto caps = cap.capabilities & V4L2_CAP_DEVICE_CAPS ? cap.device_caps : cap.capabilities;
    return std::strncmp(reinterpret_cast<const char *>(cap.driver), "bcm2835-codec", sizeof(cap.driver)) == 0 &&
        (caps & V4L2_CAP_VIDEO_M2M_MPLANE) && (caps & V4L2_CAP_STREAMING);
}

inline bool hasV4l2Format(int fd, v4l2_buf_type type, uint32_t format)
{
    for (unsigned int index = 0; index < 128; ++index) {
        v4l2_fmtdesc desc {};
        desc.type = type;
        desc.index = index;
        int result;
        do { result = ::ioctl(fd, VIDIOC_ENUM_FMT, &desc); } while (result < 0 && errno == EINTR);
        if (result < 0) return false;
        if (desc.pixelformat == format) return true;
    }
    return false;
}

// Probe nodes rather than assuming the encoder is always /dev/video11.
inline std::string findRaspberryPiH264Encoder(std::string *error, const std::filesystem::path &root = "/dev")
{
    std::vector<std::filesystem::path> nodes;
    std::error_code ec;
    for (const auto &entry : std::filesystem::directory_iterator(root, ec)) {
        const auto name = entry.path().filename().string();
        if (name.starts_with("video") && name.size() > 5 &&
            name.find_first_not_of("0123456789", 5) == std::string::npos)
            nodes.push_back(entry.path());
    }
    std::sort(nodes.begin(), nodes.end());
    bool permissionDenied = false;
    for (const auto &node : nodes) {
        const int fd = ::open(node.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) { permissionDenied |= errno == EACCES || errno == EPERM; continue; }
        v4l2_capability cap {};
        int result;
        do { result = ::ioctl(fd, VIDIOC_QUERYCAP, &cap); } while (result < 0 && errno == EINTR);
        const bool encoder = result == 0 && raspberryPiH264Encoder(cap) &&
            hasV4l2Format(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_PIX_FMT_YUV420) &&
            hasV4l2Format(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_PIX_FMT_H264);
        ::close(fd);
        if (encoder) return node.string();
    }
    if (error) *error = permissionDenied
        ? "Cannot access the Pi H.264 hardware encoder. Check video-group permissions and the bcm2835-codec driver."
        : "Pi H.264 hardware encoder unavailable or busy. Check the bcm2835-codec driver and /dev/video* devices. CPU encoding was not enabled.";
    return {};
}

} // namespace apertar
