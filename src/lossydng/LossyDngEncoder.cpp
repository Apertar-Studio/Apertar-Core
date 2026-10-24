#include <algorithm>
#include "lossydng/LossyDngEncoder.hpp"

#include <array>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <system_error>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>

namespace apertar {
namespace {


// Square-root companding keeps more highlight precision than a steep log
// curve at the small code ranges useful to predictive JPEG compression.
constexpr unsigned kWorkingMax = 4094;

void buildCompanding(int black, int white, bool gradation,
                     std::vector<uint16_t> &forward, std::vector<uint16_t> &inverse)
{
    forward.resize(size_t(white) + 1u);
    for (size_t code = 0; code < forward.size(); ++code) {
        const double signal = std::max(0.0, double(code) - black);
        const double linear = !gradation ? signal / (white - black)
            : (signal <= 500.0 ? signal
               : signal <= 3250.0 ? 500.0 + (signal - 500.0) * 4.0
                                   : 11500.0 + (signal - 3250.0) * 64.0) / 65535.0;
        forward[code] = uint16_t(std::lround(std::sqrt(std::clamp(linear, 0.0, 1.0)) * kWorkingMax));
    }
    inverse.resize(4096);
    for (unsigned code = 0; code < inverse.size(); ++code) {
        const double value = std::min(code, kWorkingMax) / double(kWorkingMax);
        inverse[code] = uint16_t(std::lround(value * value * 65535.0));
    }
}


uint16_t readLe16(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}


bool writeAll(int fd, const uint8_t *data, size_t size)
{
    size_t off = 0;
    while (off < size) {
        const ssize_t n = ::write(fd, data + off, size - off);
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            return false;
        off += static_cast<size_t>(n);
    }
    return true;
}

bool writeFile(const std::string &path, const uint8_t *data, size_t size)
{
    int flags = O_WRONLY | O_CREAT | O_TRUNC;
#ifdef O_CLOEXEC
    flags |= O_CLOEXEC;
#endif
    const int fd = ::open(path.c_str(), flags, 0644);
    if (fd < 0)
        return false;
    ::posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);
    bool ok = writeAll(fd, data, size);
    const int writeErrno = errno;
    ::posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
    if (::close(fd) != 0 && ok)
        ok = false;
    if (!ok) {
        errno = writeErrno;
        ::unlink(path.c_str());
    }
    return ok;
}

size_t roundUp4096(size_t value)
{
    return (value + 4095u) & ~size_t(4095u);
}

size_t configuredWriterByteCap()
{
    static const size_t value = [] {
        const char *env = std::getenv("APERTAR_LOSSYDNG_WRITER_BYTES");
        if (!env || !*env)
            return size_t(512) * 1024u * 1024u;
        char *end = nullptr;
        const unsigned long long parsed = std::strtoull(env, &end, 10);
        if (end == env || parsed < 16ull * 1024ull * 1024ull)
            return size_t(512) * 1024u * 1024u;
        return static_cast<size_t>(parsed);
    }();
    return value;
}

class LossyDngDiskWriter {
public:
    static LossyDngDiskWriter &instance()
    {
        static LossyDngDiskWriter writer;
        return writer;
    }

    uint8_t *take(size_t need, size_t *capacity)
    {
        const size_t wanted = roundUp4096(std::max<size_t>(need, 4096u));
        {
            std::lock_guard<std::mutex> lock(poolMutex_);
            auto best = pool_.end();
            for (auto it = pool_.begin(); it != pool_.end(); ++it) {
                if (it->capacity >= wanted && (best == pool_.end() || it->capacity < best->capacity))
                    best = it;
            }
            if (best != pool_.end()) {
                uint8_t *data = best->data;
                if (capacity)
                    *capacity = best->capacity;
                pool_.erase(best);
                return data;
            }
        }

        void *ptr = nullptr;
        if (::posix_memalign(&ptr, 4096, wanted) != 0)
            return nullptr;
        if (capacity)
            *capacity = wanted;
        return static_cast<uint8_t *>(ptr);
    }

    void give(uint8_t *data, size_t capacity)
    {
        if (!data)
            return;
        std::lock_guard<std::mutex> lock(poolMutex_);
        size_t pooled = 0;
        for (const PoolBuf &buf : pool_)
            pooled += buf.capacity;
        if (pooled + capacity > 256u * 1024u * 1024u) {
            std::free(data);
            return;
        }
        pool_.push_back({ data, capacity });
    }

    void submit(std::string path, uint8_t *data, size_t size, size_t capacity)
    {
        if (!data)
            return;
        std::unique_lock<std::mutex> lock(mutex_);
        const size_t cap = configuredWriterByteCap();
        drainCv_.wait(lock, [&] {
            return queuedBytes_ + size <= cap;
        });
        queuedBytes_ += size;
        queue_.push_back({ std::move(path), data, size, capacity });
        queueCv_.notify_one();
    }

    void drain()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        drainCv_.wait(lock, [&] {
            return queue_.empty() && !writing_;
        });
    }

private:
    struct Item {
        std::string path;
        uint8_t *data = nullptr;
        size_t size = 0;
        size_t capacity = 0;
    };
    struct PoolBuf {
        uint8_t *data = nullptr;
        size_t capacity = 0;
    };

    LossyDngDiskWriter()
        : thread_([this] { threadMain(); })
    {
    }

    ~LossyDngDiskWriter()
    {
        drain();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        queueCv_.notify_all();
        thread_.join();
        for (const auto &buffer : pool_)
            std::free(buffer.data);
    }

    void threadMain()
    {
        for (;;) {
            Item item;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                queueCv_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
                if (queue_.empty())
                    return;
                item = std::move(queue_.front());
                queue_.pop_front();
                queuedBytes_ -= std::min(queuedBytes_, item.size);
                writing_ = true;
                drainCv_.notify_all();
            }

            if (!writeFile(item.path, item.data, item.size)) {
                std::cerr << "ApertarCore LossyDng write failed for "
                          << item.path << ": " << std::strerror(errno) << "\n";
            }
            give(item.data, item.capacity);

            {
                std::lock_guard<std::mutex> lock(mutex_);
                writing_ = false;
                if (queue_.empty())
                    drainCv_.notify_all();
            }
        }
    }

    std::mutex mutex_;
    std::condition_variable queueCv_;
    std::condition_variable drainCv_;
    std::deque<Item> queue_;
    size_t queuedBytes_ = 0;
    bool writing_ = false;
    bool stopping_ = false;
    std::thread thread_;

    std::mutex poolMutex_;
    std::vector<PoolBuf> pool_;
};


void decodeCsi2Raw10ToMsb16(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    uint32_t x = 0;
    for (; x + 3u < width; x += 4, src += 5) {
        dst[x + 0] = static_cast<uint16_t>(((uint16_t(src[0]) << 2) | ((src[4] >> 0) & 0x3)) << 6);
        dst[x + 1] = static_cast<uint16_t>(((uint16_t(src[1]) << 2) | ((src[4] >> 2) & 0x3)) << 6);
        dst[x + 2] = static_cast<uint16_t>(((uint16_t(src[2]) << 2) | ((src[4] >> 4) & 0x3)) << 6);
        dst[x + 3] = static_cast<uint16_t>(((uint16_t(src[3]) << 2) | ((src[4] >> 6) & 0x3)) << 6);
    }
    if (x < width) {
        const uint8_t tail = src[4];
        for (uint32_t i = 0; x < width; ++x, ++i)
            dst[x] = static_cast<uint16_t>(((uint16_t(src[i]) << 2) | ((tail >> (i * 2)) & 0x3)) << 6);
    }
}

void decodeCsi2Raw12ToMsb16(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    uint32_t x = 0;
    for (; x + 1u < width; x += 2, src += 3) {
        dst[x + 0] = static_cast<uint16_t>(((uint16_t(src[0]) << 4) | ((src[2] >> 0) & 0xF)) << 4);
        dst[x + 1] = static_cast<uint16_t>(((uint16_t(src[1]) << 4) | ((src[2] >> 4) & 0xF)) << 4);
    }
    if (x < width)
        dst[x] = static_cast<uint16_t>(((uint16_t(src[0]) << 4) | ((src[2] >> 0) & 0xF)) << 4);
}

uint8_t unpackedBytesPerSampleForFrame(const RawFrameView &frame)
{
    if (frame.unpackedBytesPerSample == 1u || frame.unpackedBytesPerSample == 2u)
        return frame.unpackedBytesPerSample;
    return frame.bitDepth == RawBitDepth::Bits8 ? 1u : 2u;
}

void decodeRowToMsb16(const RawFrameView &frame, const uint8_t *src, uint16_t *dst)
{
    if (frame.packing == PixelPacking::PispCompressed1) {
        CdngEncoder::decodePispRowToMsb16(src, dst, frame.width);
        return;
    }
    if (frame.packing == PixelPacking::Csi2Packed) {
        if (frame.bitDepth == RawBitDepth::Bits10)
            decodeCsi2Raw10ToMsb16(src, dst, frame.width);
        else
            decodeCsi2Raw12ToMsb16(src, dst, frame.width);
        return;
    }

    const uint8_t bytes = unpackedBytesPerSampleForFrame(frame);
    if (bytes == 1u) {
        for (uint32_t x = 0; x < frame.width; ++x)
            dst[x] = static_cast<uint16_t>(src[x] << 8);
        return;
    }
    for (uint32_t x = 0; x < frame.width; ++x)
        dst[x] = readLe16(src + size_t(x) * 2u);
}


}  // namespace

LossyDngEncoder::LossyDngEncoder(LossyDngRatio ratio) : ratio_(ratio)
{
    /* Knob, not a hardcode: the inline path is what every number before
     * 2026-09-15 was measured on, so an A/B has to stay reachable without a
     * rebuild. 0 restores the old open/write/close on the encode worker. */
    const char *env = std::getenv("APERTAR_LOSSYDNG_ASYNCWRITE");
    if (!env || !*env)
        env = std::getenv("QRAW_LOSSYDNG_ASYNCWRITE");
    asyncWrite_ = !(env && env[0] == '0');
}

LossyDngEncoder::~LossyDngEncoder()
{
    /* Only a buffer we still own; after submit() it belongs to the writer. */
    if (buf_)
        LossyDngDiskWriter::instance().give(buf_, bufCap_);
    if (memoryFd_ >= 0)
        ::close(memoryFd_);
}

void LossyDngEncoder::ensureBuf(size_t need, size_t used)
{
    if (buf_ && bufCap_ >= need)
        return;
    size_t cap = 0;
    uint8_t *nb = LossyDngDiskWriter::instance().take(need, &cap);
    if (!nb || cap < need)
        throw std::runtime_error("LossyDng: cannot allocate container buffer");
    if (buf_) {
        if (used) std::memcpy(nb, buf_, used);
        LossyDngDiskWriter::instance().give(buf_, bufCap_);
    }
    buf_ = nb;
    bufCap_ = cap;
}

void LossyDngEncoder::setRatio(LossyDngRatio ratio)
{
    if (ratio == ratio_) return;
    ratio_ = ratio;
    scale_ = 0;          /* re-seed the rate loop; the old scale is for the old budget */
    invalidateTables();          /* a different quantiser makes old tables wrong */
}

void LossyDngEncoder::invalidateTables()
{
    scale_ = 0;
}

void LossyDngEncoder::drainWrites()
{
    LossyDngDiskWriter::instance().drain();
}

void LossyDngEncoder::writeDngFile(const std::string &path, const RawFrameView &frame,
                                   const CdngEncoder::Metadata &metadata)
{
    if (!frame.data || frame.width < 16 || !frame.height ||
        (frame.width & 1u) || (frame.height & 1u))
        throw std::runtime_error("LossyDng: invalid Bayer geometry");
    const size_t rowBytes = frame.packing == PixelPacking::PispCompressed1
        ? ((size_t(frame.width) + 7u) & ~size_t{7})
        : frame.packing == PixelPacking::Csi2Packed
            ? (frame.bitDepth == RawBitDepth::Bits10 ? ((size_t(frame.width) + 3u) / 4u) * 5u
                                                    : ((size_t(frame.width) + 1u) / 2u) * 3u)
            : size_t(frame.width) * unpackedBytesPerSampleForFrame(frame);
    if (frame.stride < rowBytes || frame.size < size_t(frame.height - 1u) * frame.stride + rowBytes)
        throw std::runtime_error("LossyDng: source buffer is smaller than its geometry");

    const int srcBits = frame.sampleBits ? int(frame.sampleBits) : int(frame.bitDepth);
    if (srcBits < 8 || srcBits > 16)
        throw std::runtime_error("LossyDng: unsupported sample precision");
    const unsigned outputBits = srcBits <= 8 ? 8u : (srcBits <= 10 ? 10u : 12u);
    const unsigned outputCodes = 1u << outputBits;
    const int lutBits = frame.gradationCompanded ? 12 : std::max(srcBits, 12);
    const double blackBase = metadata.hasBlackLevels ? metadata.blackLevels[0] : 200.0;
    const int black = int(std::lround(blackBase * (metadata.blackLevelsOnContainerScale
        ? 1.0 : std::pow(2.0, lutBits - 12.0))));
    const int white = (1 << lutBits) - 1;
    if (black < 0 || black >= white)
        throw std::runtime_error("LossyDng: invalid black level");
    if (cachedBlack_ != black || cachedLutBits_ != lutBits ||
        cachedGradation_ != frame.gradationCompanded) {
        buildCompanding(black, white, frame.gradationCompanded, fwd_, linTable_);
        cachedBlack_ = black;
        cachedLutBits_ = lutBits;
        cachedGradation_ = frame.gradationCompanded;
        cachedQuantStep_ = 0;
    }

    // Quality-bounded rate control. The ratios are targets, not permission
    // to discard unlimited highlight precision on a noisy scene.
    const int maxStep = ratio_ == LossyDngRatio::R5 ? 128 : 64;
    // Keep every quantized code representable in the selected JPEG precision.
    const int minStep = int((4094u + outputCodes - 2u) / (outputCodes - 1u));
    if (scale_ <= 0)
        scale_ = ratio_ == LossyDngRatio::R5 ? 96 : 32;
    const unsigned step = unsigned(std::clamp(scale_, minStep, maxStep));
    if (cachedQuantStep_ != step || cachedOutputBits_ != outputBits) {
        quantizedForward_.resize(fwd_.size());
        for (size_t code = 0; code < fwd_.size(); ++code)
            quantizedForward_[code] = uint16_t((unsigned(fwd_[code]) + step / 2u) / step);
        inverseQuantized_.resize(outputCodes);
        for (unsigned code = 0; code < outputCodes; ++code) {
            const unsigned linear = linTable_[std::min(4094u, code * step)];
            // Preserve the proven 12-bit curve; lower-depth modes reconstruct
            // to their native range rather than advertising 16-bit linear RAW.
            inverseQuantized_[code] = outputBits == 12 ? uint16_t(linear)
                : uint16_t((linear * (outputCodes - 1u) + 32767u) / 65535u);
        }
        cachedQuantStep_ = step;
        cachedOutputBits_ = outputBits;
    }
    const unsigned shift = 16u - unsigned(lutBits);
    const bool streamPisp = frame.packing == PixelPacking::PispCompressed1;
    const bool byteOutput = outputBits == 8;
    if (!streamPisp) rowScratch_.resize(frame.width);
    if (!streamPisp && byteOutput) {
        unpacked8_.resize(size_t(frame.width) * frame.height);
        std::vector<uint16_t>().swap(unpacked_);
        for (uint32_t y = 0; y < frame.height; ++y) {
            decodeRowToMsb16(frame, frame.data + size_t(y) * frame.stride, rowScratch_.data());
            uint8_t *target = unpacked8_.data() + size_t(y) * frame.width;
            for (uint32_t x = 0; x < frame.width; ++x)
                target[x] = uint8_t(quantizedForward_[rowScratch_[x] >> shift]);
        }
    } else if (!streamPisp) {
        unpacked_.resize(size_t(frame.width) * frame.height);
        std::vector<uint8_t>().swap(unpacked8_);
        for (uint32_t y = 0; y < frame.height; ++y) {
            decodeRowToMsb16(frame, frame.data + size_t(y) * frame.stride, rowScratch_.data());
            uint16_t *target = unpacked_.data() + size_t(y) * frame.width;
            for (uint32_t x = 0; x < frame.width; ++x)
                target[x] = quantizedForward_[rowScratch_[x] >> shift];
        }
    }

    RawFrameView encoded = frame;
    if (!streamPisp) {
        encoded.unpackedBytesPerSample = byteOutput ? 1 : 2;
        encoded.data = byteOutput ? unpacked8_.data() : reinterpret_cast<const uint8_t *>(unpacked_.data());
        encoded.size = size_t(frame.width) * frame.height * encoded.unpackedBytesPerSample;
        encoded.stride = frame.width * encoded.unpackedBytesPerSample;
        encoded.packing = PixelPacking::Unpacked;
    }
    encoded.bitDepth = static_cast<RawBitDepth>(outputBits);
    encoded.sampleBits = outputBits;
    encoded.gradationCompanded = false;
    encoded.cropX = encoded.cropY = 0;
    CdngEncoder::Metadata outputMetadata = metadata;
    outputMetadata.compression = CdngCompression::LosslessJpeg;
    if (streamPisp) {
        // Quantize each decoded JPEG row while it is still in cache, instead
        // of materializing and rereading a full 16-bit intermediate image.
        outputMetadata.jpegSampleLookup = quantizedForward_;
        outputMetadata.jpegSampleShift = shift;
    }
    outputMetadata.linearizationTable = inverseQuantized_;
    outputMetadata.compactJpegCodes = true;
    outputMetadata.hasBlackLevels = true;
    outputMetadata.blackLevelsOnContainerScale = true;
    std::fill(std::begin(outputMetadata.blackLevels), std::end(outputMetadata.blackLevels), 0.0f);

    if (memoryFd_ < 0) {
        memoryFd_ = ::memfd_create("apertar-lossy-dng", MFD_CLOEXEC);
        if (memoryFd_ < 0)
            throw std::system_error(errno, std::generic_category(), "LossyDng: memory destination failed");
    }
    standardEncoder_.writeDngFile(path, encoded, outputMetadata, memoryFd_);
    const off_t bytes = ::lseek(memoryFd_, 0, SEEK_END);
    if (bytes <= 0)
        throw std::runtime_error("LossyDng: empty encoded frame");
    ensureBuf(size_t(bytes), 0);
    size_t offset = 0;
    while (offset < size_t(bytes)) {
        const ssize_t n = ::pread(memoryFd_, buf_ + offset, size_t(bytes) - offset, off_t(offset));
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            throw std::system_error(errno ? errno : EIO, std::generic_category(), "LossyDng: memory read failed");
        offset += size_t(n);
    }
    lastBytes_ = size_t(bytes);
    const double budget = double(frame.width) * frame.height * (outputBits / 8.0) /
        (ratio_ == LossyDngRatio::R5 ? 5.0 : 3.0);
    const double correction = std::clamp(std::pow(double(bytes) / budget, 0.5), 0.8, 1.25);
    scale_ = std::clamp(int(std::lround(step * correction)), minStep, maxStep);

    if (asyncWrite_) {
        LossyDngDiskWriter::instance().submit(path, buf_, size_t(bytes), bufCap_);
        buf_ = nullptr;
        bufCap_ = 0;
    } else if (!writeFile(path, buf_, size_t(bytes))) {
        ::unlink(path.c_str());
        throw std::system_error(errno ? errno : EIO, std::generic_category(), "LossyDng: write failed");
    }
}
}  // namespace apertar
