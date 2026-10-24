#include "ProresRecorder.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <dlfcn.h>
#include <iostream>
#include <memory>
#include <vector>
#include <ctime>
#include <cstdio>

namespace apertar {
namespace {
void setError(std::string *error, const std::string &text) { if (error) *error = text; }
void releaseFrame(void *token) {
    std::unique_ptr<std::function<void()>> release(static_cast<std::function<void()> *>(token));
    (*release)();
}
}
ProresRecorder::~ProresRecorder() {
    stop();
    if (library_) ::dlclose(library_);
}
bool ProresRecorder::load(std::string *error) {
    if (library_) return true;
    std::vector<std::filesystem::path> paths;
    if (const char *p = std::getenv("APERTAR_PRORES_PLUGIN"); p && *p) paths.emplace_back(p);
    std::error_code ec;
    const auto exe = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (!ec) paths.push_back(exe.parent_path() / "apertar-prores-libav.so");
    paths.emplace_back("/home/pi/ApertarCore-build/apertar-prores-libav.so");
    for (const auto &path : paths) {
        void *module = ::dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!module) continue;
        const auto abi = reinterpret_cast<unsigned (*)()>(::dlsym(module, "apertar_prores_abi"));
        create_ = reinterpret_cast<ApertarProresCreate>(::dlsym(module, "apertar_prores_create"));
        encode_ = reinterpret_cast<ApertarProresEncode>(::dlsym(module, "apertar_prores_encode"));
        finish_ = reinterpret_cast<ApertarProresFinish>(::dlsym(module, "apertar_prores_finish"));
        destroy_ = reinterpret_cast<ApertarProresDestroy>(::dlsym(module, "apertar_prores_destroy"));
        if (abi && abi() == kApertarProresAbi && create_ && encode_ && finish_ && destroy_) {
            library_ = module;
            std::cerr << "ApertarCore ProRes lazy-loaded: " << path << '\n';
            return true;
        }
        ::dlclose(module);
    }
    setError(error, "ProRes requires apertar-prores-libav.so. Build Core with libavcodec/libavformat development libraries.");
    return false;
}
bool ProresRecorder::startAtFolder(const std::filesystem::path &folder, const CaptureSettings &settings,
                                  uint32_t width, uint32_t height, std::string *error, size_t cameraBuffers) {
    stop();
    if (!load(error)) return false;
    failure_.reset();
    std::error_code ec;
    std::filesystem::create_directories(folder, ec);
    if (ec) { setError(error, ec.message()); return false; }
    const auto path = (folder / (folder.filename().string() + ".mov")).string();
    unsigned threads = 4;
    if (const char *p = std::getenv("APERTAR_PRORES_THREADS"); p && *p)
        threads = unsigned(std::clamp(std::atoi(p), 1, 4));
    // Absorb short stalls without borrowing the camera's pipeline headroom.
    const size_t reserve = size_t(threads) + 4;
    queueCapacity_ = cameraBuffers > reserve ? std::min<size_t>(4, cameraBuffers - reserve) : 1;
    unsigned profile = 2;
    switch (settings.proresProfile) {
    case ProresProfile::Proxy: profile = 0; break;
    case ProresProfile::LT: profile = 1; break;
    case ProresProfile::Standard: profile = 2; break;
    case ProresProfile::HQ: profile = 3; break;
    }
    // Rec-run starts at zero; free-run starts at the current local wall clock.
    char tc[16] = "00:00:00:00";
    if (settings.timecodeMode == TimecodeMode::FreeRun) {
        std::snprintf(tc, sizeof(tc), "@capture");
    }
    ApertarProresConfig cfg{path.c_str(), tc, width, height, settings.fps, profile, threads,
                           static_cast<unsigned>(settings.proresColorProfile)};
    std::array<char, 512> why{};
    encoder_ = create_(&cfg, why.data(), why.size());
    if (!encoder_) { setError(error, why.data()); return false; }
    {
        std::lock_guard lock(mutex_);
        stopping_ = false;
        submitted_ = written_ = dropped_ = failures_ = peak_ = 0;
        accepting_.store(true, std::memory_order_release);
    }
    worker_ = std::thread(&ProresRecorder::run, this);
    std::cerr << "ApertarCore ProRes recording clip: " << path << " profile=" << toString(settings.proresProfile)
              << " source=RGB48 output=yuv422p10le threads=" << threads << " queue=" << queueCapacity_ << '\n';
    return true;
}
bool ProresRecorder::enqueueFrame(const uint8_t *data, size_t size, uint32_t width, uint32_t height,
                                 uint32_t stride, int64_t timestamp, std::function<void()> release) {
    std::lock_guard lock(mutex_);
    if (!accepting_.load(std::memory_order_relaxed) || stopping_) return false;
    if (queue_.size() >= queueCapacity_) {
        if (dropWarning_) dropWarning_->markDropped();
        if (++dropped_ == 1 || dropped_ % 30 == 0)
            std::cerr << "ApertarCore ProRes drop[queue-full]: total=" << dropped_ << '\n';
        return false;
    }
    queue_.push_back({{data, size, width, height, stride, timestamp, nullptr, nullptr}, std::move(release)});
    ++submitted_;
    peak_ = std::max(peak_, queue_.size());
    cv_.notify_one();
    return true;
}
void ProresRecorder::run() {
    for (;;) {
        Job job;
        {
            std::unique_lock lock(mutex_);
            cv_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
            if (queue_.empty()) break;
            job = std::move(queue_.front()); queue_.pop_front();
        }
        job.frame.release = releaseFrame;
        job.frame.token = new std::function<void()>(std::move(job.release));
        std::array<char, 512> why{};
        if (encode_(encoder_, &job.frame, why.data(), why.size())) ++written_;
        else {
            ++failures_;
            failure_.set(why.data());
            accepting_.store(false, std::memory_order_release);
            std::deque<Job> cancelled;
            {
                std::lock_guard lock(mutex_);
                stopping_ = true;
                cancelled.swap(queue_);
            }
            for (auto &pending : cancelled) if (pending.release) pending.release();
            std::cerr << "ApertarCore ProRes encode failure: " << why.data() << '\n';
            break;
        }
    }
    std::array<char, 512> why{};
    if (!finish_(encoder_, why.data(), why.size())) {
        ++failures_;
        failure_.set(why.data());
        std::cerr << "ApertarCore ProRes finalize failure: " << why.data() << '\n';
    }
}
void ProresRecorder::stop() {
    accepting_.store(false, std::memory_order_release);
    {
        std::lock_guard lock(mutex_);
        stopping_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (encoder_) {
        destroy_(encoder_); encoder_ = nullptr;
        std::cerr << "ApertarCore ProRes recording stats: queued=" << submitted_ << " submitted=" << written_
                  << " dropped_queue_full=" << dropped_ << " failures=" << failures_ << " peak_queue=" << peak_ << '/' << queueCapacity_ << '\n';
    }
}
}
