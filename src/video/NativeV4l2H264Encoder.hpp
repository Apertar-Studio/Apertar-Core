#pragma once

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <utility>
#include <vector>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace apertar {

struct NativeH264System {
    static int open(const char *path, int flags) { return ::open(path, flags); }
    static int close(int fd) { return ::close(fd); }
    static int ioctl(int fd, unsigned long request, void *arg) { return ::ioctl(fd, request, arg); }
    static void *map(size_t size, int fd, off_t offset)
    { return ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset); }
    static void unmap(void *data, size_t size) { ::munmap(data, size); }
    static int poll(pollfd *fd, int timeoutMs) { return ::poll(fd, 1, timeoutMs); }
};

// Direct BCM2835 encoder transport. libav is used only by the caller's MP4 muxer.
template<class System = NativeH264System>
class NativeV4l2H264Encoder {
public:
    using PacketSink = std::function<bool(const uint8_t *, size_t, int64_t, bool, std::string *)>;

    NativeV4l2H264Encoder() = default;
    NativeV4l2H264Encoder(const NativeV4l2H264Encoder &) = delete;
    NativeV4l2H264Encoder &operator=(const NativeV4l2H264Encoder &) = delete;
    ~NativeV4l2H264Encoder() { close(); }

    bool open(const std::string &device, uint32_t width, uint32_t height,
              uint32_t rateNumerator, uint32_t rateDenominator, int bitrate,
              PacketSink sink, std::string *error)
    {
        close();
        if (width < 2 || width > 1920 || height < 2 || height > 1080 ||
            width % 2 || height % 2 || !rateNumerator || !rateDenominator ||
            double(rateNumerator) / rateDenominator > 30.0005 ||
            bitrate < 25'000 || bitrate > 25'000'000 || !sink)
            return fail(error, "Invalid native Pi H.264 configuration.");
        width_ = width;
        height_ = height;
        sink_ = std::move(sink);
        fd_ = System::open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) return systemError(error, "open encoder " + device);

        if (!control(V4L2_CID_MPEG_VIDEO_BITRATE, bitrate, error) ||
            !control(V4L2_CID_MPEG_VIDEO_H264_PROFILE, V4L2_MPEG_VIDEO_H264_PROFILE_HIGH, error) ||
            !control(V4L2_CID_MPEG_VIDEO_H264_LEVEL, V4L2_MPEG_VIDEO_H264_LEVEL_4_0, error) ||
            !control(V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
                     std::max(1, int(std::lround(double(rateNumerator) / rateDenominator))), error) ||
            !control(V4L2_CID_MPEG_VIDEO_REPEAT_SEQ_HEADER, 1, error)) return false;

        v4l2_format format{};
        format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        format.fmt.pix_mp.width = width;
        format.fmt.pix_mp.height = height;
        format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420;
        format.fmt.pix_mp.field = V4L2_FIELD_NONE;
        format.fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
        format.fmt.pix_mp.quantization = V4L2_QUANTIZATION_LIM_RANGE;
        format.fmt.pix_mp.num_planes = 1;
        format.fmt.pix_mp.plane_fmt[0].bytesperline = width;
        if (call(VIDIOC_S_FMT, &format) < 0) return systemError(error, "set YUV420 input format");
        const auto &raw = format.fmt.pix_mp;
        stride_ = raw.plane_fmt[0].bytesperline;
        storageHeight_ = raw.height;
        inputBytes_ = raw.plane_fmt[0].sizeimage;
        if (raw.pixelformat != V4L2_PIX_FMT_YUV420 || raw.num_planes != 1 ||
            raw.width != width || storageHeight_ < height || storageHeight_ > 1088 ||
            stride_ < width || stride_ > 4096 || stride_ % 2 || storageHeight_ % 2 ||
            inputBytes_ < size_t(stride_) * storageHeight_ * 3 / 2 || inputBytes_ > 8 * 1024 * 1024)
            return fail(error, "Native Pi H.264 returned an unsupported YUV420 layout.");

        format = {};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        format.fmt.pix_mp.width = width;
        format.fmt.pix_mp.height = height;
        format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
        format.fmt.pix_mp.field = V4L2_FIELD_NONE;
        format.fmt.pix_mp.num_planes = 1;
        format.fmt.pix_mp.plane_fmt[0].sizeimage = 512 * 1024;
        if (call(VIDIOC_S_FMT, &format) < 0) return systemError(error, "set H.264 capture format");
        if (format.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_H264 || format.fmt.pix_mp.num_planes != 1)
            return fail(error, "Native Pi H.264 returned an unsupported capture format.");

        v4l2_streamparm timing{};
        timing.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        timing.parm.output.timeperframe = {rateDenominator, rateNumerator};
        if (call(VIDIOC_S_PARM, &timing) < 0) return systemError(error, "set encoder frame rate");

        for (const char *path : {"/dev/dma_heap/vidbuf_cached", "/dev/dma_heap/linux,cma", "/dev/dma_heap/reserved"}) {
            heap_ = System::open(path, O_RDWR | O_CLOEXEC);
            if (heap_ >= 0) break;
        }
        if (heap_ < 0) return systemError(error, "open CMA DMA heap (check video-group permissions)");

        v4l2_requestbuffers request{};
        request.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        request.memory = V4L2_MEMORY_DMABUF;
        request.count = 4;
        if (call(VIDIOC_REQBUFS, &request) < 0) return systemError(error, "request DMA input slots");
        if (!request.count || request.count > 16) return fail(error, "Invalid native H.264 input buffer count.");
        input_.resize(request.count);
        for (Buffer &buffer : input_) {
            dma_heap_allocation_data allocation{};
            allocation.len = inputBytes_;
            allocation.fd_flags = O_RDWR | O_CLOEXEC;
            if (systemCall(heap_, DMA_HEAP_IOCTL_ALLOC, &allocation) < 0)
                return systemError(error, "allocate CMA input buffer");
            buffer.fd = static_cast<int>(allocation.fd);
            buffer.size = inputBytes_;
            buffer.data = System::map(buffer.size, buffer.fd, 0);
            if (buffer.data == MAP_FAILED) return systemError(error, "map DMA input buffer");
            if (!sync(buffer, DMA_BUF_SYNC_START, error)) return false;
            std::memset(buffer.data, 0, buffer.size);
            // Neutral padded chroma prevents coloured borders on aligned crop modes.
            std::memset(static_cast<uint8_t *>(buffer.data) + size_t(stride_) * storageHeight_, 128,
                        size_t(stride_) * storageHeight_ / 2);
            if (!sync(buffer, DMA_BUF_SYNC_END, error)) return false;
        }
        System::close(heap_);
        heap_ = -1;

        request = {};
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        request.memory = V4L2_MEMORY_MMAP;
        request.count = 8;
        if (call(VIDIOC_REQBUFS, &request) < 0) return systemError(error, "request encoded output buffers");
        if (!request.count || request.count > 32) return fail(error, "Invalid native H.264 capture buffer count.");
        capture_.resize(request.count);
        for (uint32_t i = 0; i < capture_.size(); ++i) {
            v4l2_plane plane{};
            v4l2_buffer buffer = makeBuffer(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, i, plane);
            if (call(VIDIOC_QUERYBUF, &buffer) < 0) return systemError(error, "query capture buffer");
            if (!plane.length || plane.length > 8 * 1024 * 1024)
                return fail(error, "Invalid native H.264 capture buffer size.");
            capture_[i].size = plane.length;
            capture_[i].data = System::map(plane.length, fd_, plane.m.mem_offset);
            if (capture_[i].data == MAP_FAILED) return systemError(error, "map capture buffer");
            if (!queueCapture(i, error)) return false;
        }

        // Like rpicam-vid: DMABUF input, and STREAMON before the first input QBUF.
        // Do not use FFmpeg's queue-first MMAP-input startup on BCM2835.
        auto type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        if (call(VIDIOC_STREAMON, &type) < 0) return systemError(error, "start input streaming");
        inputStreaming_ = true;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        if (call(VIDIOC_STREAMON, &type) < 0) return systemError(error, "start capture streaming");
        captureStreaming_ = true;
        return true;
    }

    bool encode(const uint8_t *packed, size_t size, int64_t timestampUs, std::string *error)
    {
        if (!captureStreaming_ || finishing_ || !packed || timestampUs < 0 ||
            size < size_t(width_) * height_ * 3 / 2)
            return fail(error, "Invalid native H.264 input frame.");
        size_t index;
        if (!availableInput(index, error)) return false;
        Buffer &buffer = input_[index];
        if (!sync(buffer, DMA_BUF_SYNC_START, error)) return false;
        auto *destination = static_cast<uint8_t *>(buffer.data);
        const uint8_t *source = packed;
        for (unsigned int p = 0; p < 3; ++p) {
            const uint32_t rows = p == 0 ? height_ : height_ / 2;
            const uint32_t rowBytes = p == 0 ? width_ : width_ / 2;
            const uint32_t pitch = p == 0 ? stride_ : stride_ / 2;
            for (uint32_t y = 0; y < rows; ++y) {
                std::memcpy(destination + size_t(y) * pitch, source, rowBytes);
                source += rowBytes;
            }
            destination += size_t(pitch) * (p == 0 ? storageHeight_ : storageHeight_ / 2);
        }
        if (!sync(buffer, DMA_BUF_SYNC_END, error)) return false;
        if (!queueInput(index, timestampUs, error)) return false;
        ++submitted_;
        return drain(error);
    }

    bool finish(std::string *error)
    {
        if (!captureStreaming_ || ended_ || !submitted_) return true;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        if (!finishing_) {
            // Wait until the submitted inputs reach the VPU before sending its EOS.
            while (std::any_of(input_.begin(), input_.end(), [](const Buffer &buffer) { return buffer.queued; })) {
                if (!drain(error)) return false;
                if (std::any_of(input_.begin(), input_.end(), [](const Buffer &buffer) { return buffer.queued; }) &&
                    !wait(deadline, error)) return false;
            }
            v4l2_encoder_cmd command{};
            command.cmd = V4L2_ENC_CMD_STOP;
            if (call(VIDIOC_ENCODER_CMD, &command) < 0) return systemError(error, "drain encoder EOS");
            finishing_ = true;
        }
        while (!ended_) {
            if (!drain(error)) return false;
            if (!ended_ && !wait(deadline, error)) return false;
        }
        return true;
    }

    void close()
    {
        if (fd_ >= 0) {
            if (inputStreaming_) {
                auto type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
                call(VIDIOC_STREAMOFF, &type);
            }
            if (captureStreaming_) {
                auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                call(VIDIOC_STREAMOFF, &type);
            }
            System::close(fd_);
            fd_ = -1;
        }
        for (auto *buffers : {&input_, &capture_}) {
            for (Buffer &buffer : *buffers) {
                if (buffer.data != MAP_FAILED) System::unmap(buffer.data, buffer.size);
                if (buffer.fd >= 0) System::close(buffer.fd);
            }
            buffers->clear();
        }
        if (heap_ >= 0) System::close(heap_);
        heap_ = -1;
        sink_ = {};
        inputStreaming_ = captureStreaming_ = finishing_ = ended_ = false;
        submitted_ = 0;
    }

private:
    struct Buffer {
        void *data = MAP_FAILED;
        size_t size = 0;
        int fd = -1;
        bool queued = false;
    };
    static int systemCall(int fd, unsigned long request, void *arg)
    {
        int result;
        do { result = System::ioctl(fd, request, arg); } while (result < 0 && errno == EINTR);
        return result;
    }
    int call(unsigned long request, void *arg) { return systemCall(fd_, request, arg); }
    static bool fail(std::string *error, const std::string &message)
    { if (error) *error = message; return false; }
    static bool systemError(std::string *error, const std::string &operation)
    { return fail(error, "Native Pi H.264: " + operation + ": " + std::strerror(errno)); }
    bool control(uint32_t id, int value, std::string *error)
    {
        v4l2_control ctrl{};
        ctrl.id = id;
        ctrl.value = value;
        return call(VIDIOC_S_CTRL, &ctrl) == 0 || systemError(error, "set encoder control " + std::to_string(id));
    }
    static v4l2_buffer makeBuffer(v4l2_buf_type type, v4l2_memory memory, uint32_t index, v4l2_plane &plane)
    {
        v4l2_buffer buffer{};
        buffer.type = type;
        buffer.memory = memory;
        buffer.index = index;
        buffer.field = V4L2_FIELD_NONE;
        buffer.length = 1;
        buffer.m.planes = &plane;
        return buffer;
    }
    static bool sync(Buffer &buffer, uint64_t phase, std::string *error)
    {
        dma_buf_sync access{};
        access.flags = DMA_BUF_SYNC_WRITE | phase;
        return systemCall(buffer.fd, DMA_BUF_IOCTL_SYNC, &access) == 0 || systemError(error, "synchronize DMA input");
    }
    bool queueCapture(uint32_t index, std::string *error)
    {
        v4l2_plane plane{};
        auto buffer = makeBuffer(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, index, plane);
        plane.length = capture_[index].size;
        return call(VIDIOC_QBUF, &buffer) == 0 || systemError(error, "queue encoded output buffer");
    }
    bool queueInput(size_t index, int64_t timestampUs, std::string *error)
    {
        v4l2_plane plane{};
        auto buffer = makeBuffer(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_DMABUF, index, plane);
        plane.m.fd = input_[index].fd;
        plane.length = input_[index].size;
        plane.bytesused = inputBytes_;
        buffer.timestamp.tv_sec = timestampUs / 1000000;
        buffer.timestamp.tv_usec = timestampUs % 1000000;
        if (call(VIDIOC_QBUF, &buffer) < 0) return systemError(error, "queue DMA input frame");
        input_[index].queued = true;
        return true;
    }
    bool drain(std::string *error)
    {
        for (unsigned int i = 0; i < 64; ++i) {
            v4l2_plane plane{};
            auto buffer = makeBuffer(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_DMABUF, 0, plane);
            if (call(VIDIOC_DQBUF, &buffer) < 0) {
                if (errno == EAGAIN) break;
                return systemError(error, "dequeue DMA input");
            }
            if (buffer.index >= input_.size() || !input_[buffer.index].queued)
                return fail(error, "Native Pi H.264 returned an invalid input buffer index.");
            input_[buffer.index].queued = false;
            if (buffer.flags & V4L2_BUF_FLAG_ERROR) return fail(error, "Native Pi H.264 rejected an input frame.");
        }
        for (unsigned int i = 0; i < 64 && !ended_; ++i) {
            v4l2_plane plane{};
            auto buffer = makeBuffer(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, 0, plane);
            if (call(VIDIOC_DQBUF, &buffer) < 0) {
                if (errno == EAGAIN) break;
                return systemError(error, "dequeue encoded output");
            }
            if (buffer.index >= capture_.size() || plane.data_offset > plane.bytesused ||
                plane.bytesused > capture_[buffer.index].size)
                return fail(error, "Native Pi H.264 returned an invalid encoded buffer.");
            if (buffer.flags & V4L2_BUF_FLAG_ERROR) return fail(error, "Native Pi H.264 returned a corrupt frame.");
            const auto bytes = plane.bytesused - plane.data_offset;
            if (bytes && !sink_(static_cast<uint8_t *>(capture_[buffer.index].data) + plane.data_offset, bytes,
                                int64_t(buffer.timestamp.tv_sec) * 1000000 + buffer.timestamp.tv_usec,
                                (buffer.flags & V4L2_BUF_FLAG_KEYFRAME) != 0, error)) return false;
            ended_ = (buffer.flags & V4L2_BUF_FLAG_LAST) != 0;
            if (!ended_ && !queueCapture(buffer.index, error)) return false;
        }
        return true;
    }
    bool wait(std::chrono::steady_clock::time_point deadline, std::string *error)
    {
        if (std::chrono::steady_clock::now() >= deadline)
            return fail(error, "Native Pi H.264 timed out waiting for the hardware encoder.");
        pollfd descriptor{fd_, POLLIN, 0};
        const int result = System::poll(&descriptor, 50);
        if (result < 0 && errno != EINTR) return systemError(error, "poll encoder");
        if (result > 0 && (descriptor.revents & (POLLHUP | POLLNVAL)))
            return fail(error, "Native Pi H.264 encoder disconnected.");
        if (result > 0 && !(descriptor.revents & POLLIN)) {
            // Some M2M drivers report POLLERR while one queue is temporarily empty.
            // Do not busy-spin during a drain or a failed-device timeout.
            pollfd idle{-1, 0, 0};
            System::poll(&idle, 10);
        }
        return true;
    }
    bool availableInput(size_t &index, std::string *error)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (true) {
            if (!drain(error)) return false;
            for (size_t i = 0; i < input_.size(); ++i) {
                if (!input_[i].queued) { index = i; return true; }
            }
            if (!wait(deadline, error)) return false;
        }
    }

    int fd_ = -1;
    int heap_ = -1;
    uint32_t width_ = 0, height_ = 0, stride_ = 0, storageHeight_ = 0;
    uint32_t inputBytes_ = 0;
    uint64_t submitted_ = 0;
    std::vector<Buffer> input_, capture_;
    PacketSink sink_;
    bool inputStreaming_ = false, captureStreaming_ = false, finishing_ = false, ended_ = false;
};

} // namespace apertar
