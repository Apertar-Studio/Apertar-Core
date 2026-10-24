#include "audio/AudioRecordingSession.hpp"
#include "audio/AudioSample.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace apertar {
namespace {

constexpr uint32_t kBextChunkSize = 602;

void ignoreSigpipeProcessWide()
{
    static std::once_flag once;
    std::call_once(once, []() {
        struct sigaction action {};
        action.sa_handler = SIG_IGN;
        ::sigemptyset(&action.sa_mask);
        action.sa_flags = 0;
        ::sigaction(SIGPIPE, &action, nullptr);
    });
}

double inputGainFactor(uint8_t inputVolume)
{
    const int clamped = std::clamp(static_cast<int>(inputVolume), 0, 100);
    if (clamped <= 0)
        return 0.0;

    constexpr double kUnityReference = 60.0;
    return std::clamp(static_cast<double>(clamped) / kUnityReference, 0.0, 2.0);
}

int applyGainAndMeasure(char *data, size_t sizeBytes, const std::string &format, double gain, bool measurePeak)
{
    if (gain <= 0.0) {
        std::fill(data, data + sizeBytes, 0);
        return 0;
    }

    const bool unityGain = std::fabs(gain - 1.0) < 0.0001;
    if (unityGain && !measurePeak)
        return 0;

    double peak = 0;
    const unsigned bytes = audioSampleBytes(format);
    for (size_t offset = 0; offset + bytes <= sizeBytes; offset += bytes) {
        const double sample = readAudioSample(data + offset, format) * gain;
        if (!unityGain) writeAudioSample(data + offset, format, sample);
        if (measurePeak) peak = std::max(peak, std::abs(sample));
    }

    if (!measurePeak || peak <= 0)
        return 0;

    const double normalized = std::clamp(peak, 0.0, 1.0);
    const double db = 20.0 * std::log10(std::max(normalized, 1.0e-6));
    const double scaled = std::clamp((db + 50.0) / 50.0, 0.0, 1.0);
    return static_cast<int>(std::lround(scaled * 100.0));
}

void writeUint16(std::ofstream &stream, uint16_t value)
{
    stream.write(reinterpret_cast<const char *>(&value), sizeof(value));
}

void writeUint32(std::ofstream &stream, uint32_t value)
{
    stream.write(reinterpret_cast<const char *>(&value), sizeof(value));
}

void writeUint64(std::ofstream &stream, uint64_t value)
{
    stream.write(reinterpret_cast<const char *>(&value), sizeof(value));
}

std::tm localTimeForTimestampUs(uint64_t timestampUs)
{
    const std::time_t seconds = static_cast<std::time_t>(timestampUs / 1000000ULL);
    std::tm localTime{};
#if defined(_WIN32)
    localtime_s(&localTime, &seconds);
#else
    localtime_r(&seconds, &localTime);
#endif
    return localTime;
}

std::string frameRateString(double fps)
{
    if (std::fabs(fps - 23.976) < 0.01)
        return "24000/1001";
    if (std::fabs(fps - 29.97) < 0.01)
        return "30000/1001";
    if (std::fabs(fps - 59.94) < 0.01)
        return "60000/1001";
    return std::to_string(std::max(1, static_cast<int>(std::llround(fps)))) + "/1";
}

std::string timecodeString(TimecodeMode mode, uint64_t timestampUs, uint64_t frameIndex, double fps)
{
    const int nominalFps = std::max(1, static_cast<int>(std::llround(fps > 0.0 ? fps : 24.0)));
    int hours = 0;
    int minutes = 0;
    int seconds = 0;
    int frames = 0;

    if (mode == TimecodeMode::RecRun) {
        const uint64_t totalFrames = frameIndex;
        const uint64_t totalSeconds = totalFrames / static_cast<uint64_t>(nominalFps);
        frames = static_cast<int>(totalFrames % static_cast<uint64_t>(nominalFps));
        hours = static_cast<int>((totalSeconds / 3600ULL) % 24ULL);
        minutes = static_cast<int>((totalSeconds / 60ULL) % 60ULL);
        seconds = static_cast<int>(totalSeconds % 60ULL);
    } else {
        const std::tm localTime = localTimeForTimestampUs(timestampUs);
        hours = localTime.tm_hour;
        minutes = localTime.tm_min;
        seconds = localTime.tm_sec;
        frames = static_cast<int>(((timestampUs % 1000000ULL) * (fps > 0.0 ? fps : 24.0)) / 1000000.0);
    }

    std::ostringstream out;
    out << std::setw(2) << std::setfill('0') << hours << ":"
        << std::setw(2) << std::setfill('0') << minutes << ":"
        << std::setw(2) << std::setfill('0') << seconds << ":"
        << std::setw(2) << std::setfill('0') << frames;
    return out.str();
}

uint64_t framesForDurationUs(uint64_t durationUs, uint32_t sampleRate)
{
    if (durationUs == 0 || sampleRate == 0)
        return 0;
    return (durationUs * sampleRate + 500000ULL) / 1000000ULL;
}

uint64_t framesForDurationUsCeil(uint64_t durationUs, uint32_t sampleRate)
{
    if (durationUs == 0 || sampleRate == 0)
        return 0;
    return (durationUs * sampleRate + 999999ULL) / 1000000ULL;
}

void writeFixedAscii(std::ofstream &stream, const std::string &value, size_t fieldSize)
{
    const size_t bytesToWrite = std::min(fieldSize, value.size());
    stream.write(value.data(), static_cast<std::streamsize>(bytesToWrite));
    if (fieldSize > bytesToWrite) {
        std::array<char, 256> zeros{};
        size_t remaining = fieldSize - bytesToWrite;
        while (remaining > 0) {
            const size_t chunk = std::min(remaining, zeros.size());
            stream.write(zeros.data(), static_cast<std::streamsize>(chunk));
            remaining -= chunk;
        }
    }
}

std::string readAvailableFd(int fd)
{
    if (fd < 0)
        return {};

    std::string output;
    std::array<char, 1024> buffer{};
    while (true) {
        const ssize_t bytesRead = ::read(fd, buffer.data(), buffer.size());
        if (bytesRead > 0) {
            output.append(buffer.data(), static_cast<size_t>(bytesRead));
            continue;
        }

        if (bytesRead < 0 && errno == EINTR)
            continue;
        break;
    }

    while (!output.empty() && (output.back() == '\n' || output.back() == '\r'))
        output.pop_back();
    return output;
}

} // namespace

AudioRecordingSession::~AudioRecordingSession()
{
    stop();
}

bool AudioRecordingSession::start(const std::filesystem::path &clipFolder,
                                  const AudioRecordingOptions &options,
                                  AudioLevelCallback levelCallback,
                                  std::string *error,
                                  const RecordingClock *clock)
{
    stop();
    failure_.reset();
    ignoreSigpipeProcessWide();

    if (!options.enabled)
        return true;

    if (clipFolder.empty()) {
        if (error)
            *error = "Audio recording requires a clip folder.";
        return false;
    }

    options_ = options;
    options_.fps = recordingFps(options_.fps);
    recordingClock_ = clock ? *clock : RecordingClock();
    levelCallback_ = std::move(levelCallback);
    if (options_.inputDevice.empty())
        options_.inputDevice = "default";

    clipFolder_ = clipFolder;
    rawPath_ = clipFolder_ / "audio_capture.raw";
    const std::string clipName = clipFolder_.filename().string();
    wavPath_ = clipFolder_ / ((clipName.empty() ? std::string("audio") : clipName) + ".wav");

    rawStream_.open(rawPath_, std::ios::binary | std::ios::trunc);
    if (!rawStream_) {
        if (error)
            *error = "Failed to open temporary audio capture file.";
        resetState();
        return false;
    }

    if (!validAudioFormat(options_.sampleFormat)) {
        if (error) *error = "Invalid audio sample format.";
        resetState();
        return false;
    }
    AudioCaptureMode mode{options_.sampleFormat, options_.sampleRate, options_.channels};
    if (!capture_.open(options_.inputDevice, mode, options_.firstChannel, error)) {
        capture_.close();
        resetState();
        return false;
    }
    options_.sampleFormat = mode.format;
    options_.sampleRate = mode.rate;
    options_.channels = mode.channels;
    std::cerr << "ApertarCore audio capture: format=" << mode.format << " rate=" << mode.rate
              << " channels=" << mode.channels << " first_channel=" << options_.firstChannel << "\n";

    if (options_.liveMonitoringEnabled && !options_.outputDevice.empty() && options_.outputVolume > 0) {
        int playbackPipeFds[2] = { -1, -1 };
        if (::pipe(playbackPipeFds) == 0) {
            int playbackStderrPipeFds[2] = { -1, -1 };
            if (::pipe(playbackStderrPipeFds) == 0) {
                const pid_t playbackPid = ::fork();
                if (playbackPid == 0) {
                    ::close(playbackPipeFds[1]);
                    ::close(playbackStderrPipeFds[0]);
                    ::dup2(playbackPipeFds[0], STDIN_FILENO);
                    ::dup2(playbackStderrPipeFds[1], STDERR_FILENO);

                    const std::string sampleRate = std::to_string(options_.sampleRate);
                    const std::string channels = std::to_string(options_.channels);
                    const char *argv[] = {
                        "aplay",
                        "-q",
                        "-D", options_.outputDevice.c_str(),
                        "-f", "S16_LE",
                        "-c", channels.c_str(),
                        "-r", sampleRate.c_str(),
                        "-t", "raw",
                        nullptr
                    };

                    ::execvp("aplay", const_cast<char * const *>(argv));
                    _exit(127);
                }

                if (playbackPid > 0) {
                    ::close(playbackPipeFds[0]);
                    ::close(playbackStderrPipeFds[1]);
                    playbackWriteFd_ = playbackPipeFds[1];
                    playbackStderrFd_ = playbackStderrPipeFds[0];
                    ::fcntl(playbackWriteFd_, F_SETFL, ::fcntl(playbackWriteFd_, F_GETFL, 0) | O_NONBLOCK);
                    ::fcntl(playbackStderrFd_, F_SETFL, ::fcntl(playbackStderrFd_, F_GETFL, 0) | O_NONBLOCK);
                    playbackPid_.store(playbackPid, std::memory_order_relaxed);

                    std::this_thread::sleep_for(std::chrono::milliseconds(120));
                    int playbackStatus = 0;
                    const pid_t playbackWaited = ::waitpid(playbackPid, &playbackStatus, WNOHANG);
                    if (playbackWaited == playbackPid) {
                        const std::string playbackStderrText = readAvailableFd(playbackStderrFd_);
                        if (!playbackStderrText.empty()) {
                            std::cerr << "ApertarCore audio monitoring disabled: "
                                      << playbackStderrText << "\n";
                        } else {
                            std::cerr << "ApertarCore audio monitoring disabled: aplay exited before monitoring started.\n";
                        }

                        if (playbackWriteFd_ >= 0) {
                            ::close(playbackWriteFd_);
                            playbackWriteFd_ = -1;
                        }
                        if (playbackStderrFd_ >= 0) {
                            ::close(playbackStderrFd_);
                            playbackStderrFd_ = -1;
                        }
                        playbackPid_.store(-1, std::memory_order_relaxed);
                    }
                } else {
                    ::close(playbackPipeFds[0]);
                    ::close(playbackPipeFds[1]);
                    ::close(playbackStderrPipeFds[0]);
                    ::close(playbackStderrPipeFds[1]);
                }
            } else {
                ::close(playbackPipeFds[0]);
                ::close(playbackPipeFds[1]);
            }
        }
    }

    stopRequested_.store(false, std::memory_order_relaxed);
    running_.store(true, std::memory_order_relaxed);
    sessionActive_.store(true, std::memory_order_relaxed);
    readerThread_ = std::thread(&AudioRecordingSession::readerLoop, this);
    return true;
}

void AudioRecordingSession::observeVideoFrame(uint64_t sensorBootUs, int64_t frameDurationUs)
{
    if (!sessionActive_.load(std::memory_order_relaxed))
        return;

    std::lock_guard<std::mutex> lock(syncMutex_);
    if (videoFirstBootUs_ == 0)
        videoFirstBootUs_ = sensorBootUs;

    const uint64_t frameEndUs = options_.fps > 0
        ? videoFirstBootUs_ + uint64_t(std::llround((std::llround(
            double(int64_t(sensorBootUs) - int64_t(videoFirstBootUs_)) * options_.fps / 1000000) + 1) * 1000000 / options_.fps))
        : sensorBootUs + uint64_t(std::max<int64_t>(0, frameDurationUs));
    videoEndBootUs_ = std::max(videoEndBootUs_, frameEndUs);
}

void AudioRecordingSession::stop()
{
    stopRequested_.store(true, std::memory_order_relaxed);
    if (readerThread_.joinable())
        readerThread_.join();
    capture_.close();

    if (rawStream_.is_open())
        rawStream_.close();

    if (capturedBytes_ > 0)
        finalizeWav();
    else if (!clipFolder_.empty())
        std::cerr << "ApertarCore audio capture produced no samples.\n";

    if (!rawPath_.empty()) {
        std::error_code ec;
        std::filesystem::remove(rawPath_, ec);
    }

    if (playbackWriteFd_ >= 0) {
        ::close(playbackWriteFd_);
        playbackWriteFd_ = -1;
    }
    const pid_t playbackPid = playbackPid_.load(std::memory_order_relaxed);
    if (playbackPid > 0)
        ::kill(playbackPid, SIGINT);
    if (playbackPid > 0) {
        int status = 0;
        ::waitpid(playbackPid, &status, 0);
        playbackPid_.store(-1, std::memory_order_relaxed);
    }
    if (playbackStderrFd_ >= 0) {
        ::close(playbackStderrFd_);
        playbackStderrFd_ = -1;
    }

    emitLevel(0);
    resetState();
}

bool AudioRecordingSession::active() const
{
    return running_.load(std::memory_order_relaxed);
}

void AudioRecordingSession::readerLoop()
{
    const unsigned sampleBytes = audioSampleBytes(options_.sampleFormat);
    const size_t blockAlign = static_cast<size_t>(options_.channels) * sampleBytes;
    std::vector<char> buffer(1024 * blockAlign);
    std::vector<char> monitorBuffer(1024 * options_.channels * 2);
    const double gain = inputGainFactor(options_.inputVolume);
    const bool shouldMeasureLevel = static_cast<bool>(levelCallback_);

    while (!stopRequested_.load(std::memory_order_relaxed)) {
        int64_t sampleBootUs = 0;
        std::string captureError;
        const int framesRead = capture_.read(buffer.data(), 1024, &sampleBootUs, &captureError);
        if (framesRead < 0) {
            failure_.set(captureError);
            std::cerr << "ApertarCore audio capture failed: " << captureError << "\n";
            break;
        }
        if (framesRead == 0) continue;
        const ssize_t bytesRead = ssize_t(framesRead * blockAlign);
        if (bytesRead > 0) {
            {
                std::lock_guard<std::mutex> lock(syncMutex_);
                if (audioStartBootUs_ == 0)
                    audioStartBootUs_ = uint64_t(std::max<int64_t>(0, sampleBootUs));
                // Preserve discontinuities (e.g. a recovered USB overrun) as silence,
                // rather than closing the gap and shifting the remaining take.
                if (audioEndBootUs_ && sampleBootUs > int64_t(audioEndBootUs_) + 10000) {
                    const uint64_t gapBytes = framesForDurationUs(sampleBootUs - audioEndBootUs_, options_.sampleRate) * blockAlign;
                    std::array<char,4096> zeros{};
                    for (uint64_t left = gapBytes; left;) {
                        const auto size = std::min<uint64_t>(left, zeros.size());
                        rawStream_.write(zeros.data(), size);left -= size;
                    }
                    capturedBytes_ += gapBytes;
                    std::cerr << "ApertarCore audio discontinuity: padded_us=" << sampleBootUs - audioEndBootUs_ << "\n";
                }
                audioEndBootUs_ = sampleBootUs + uint64_t(framesRead) * 1000000 / options_.sampleRate;
            }

            const int meterLevel = applyGainAndMeasure(buffer.data(),
                                                            static_cast<size_t>(bytesRead),
                                                            options_.sampleFormat,
                                                            gain,
                                                            shouldMeasureLevel);
            if (playbackWriteFd_ >= 0) {
                for (size_t i = 0; i < size_t(framesRead) * options_.channels; ++i)
                    writeAudioSample(monitorBuffer.data() + i * 2, "s16", readAudioSample(buffer.data() + i * sampleBytes, options_.sampleFormat));
                const char *playbackData = monitorBuffer.data();
                ssize_t remaining = framesRead * options_.channels * 2;
                while (remaining > 0) {
                    const ssize_t written = ::write(playbackWriteFd_, playbackData, static_cast<size_t>(remaining));
                    if (written > 0) {
                        playbackData += written;
                        remaining -= written;
                        continue;
                    }

                    if (written < 0 && errno == EINTR)
                        continue;
                    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                        break;

                    ::close(playbackWriteFd_);
                    playbackWriteFd_ = -1;
                    break;
                }
            }
            rawStream_.write(buffer.data(), bytesRead);
            if (!rawStream_) { failure_.set("Audio capture write failed.");std::cerr << "ApertarCore audio capture write failed.\n"; break; }
            capturedBytes_ += static_cast<uint64_t>(bytesRead);
            if (shouldMeasureLevel)
                emitLevel(meterLevel);
            continue;
        }

    }

    if (playbackWriteFd_ >= 0) {
        ::close(playbackWriteFd_);
        playbackWriteFd_ = -1;
    }
    if (playbackStderrFd_ >= 0) {
        ::close(playbackStderrFd_);
        playbackStderrFd_ = -1;
    }

    const pid_t playbackPid = playbackPid_.exchange(-1, std::memory_order_relaxed);
    if (playbackPid > 0) {
        int status = 0;
        ::waitpid(playbackPid, &status, 0);
    }

    emitLevel(0);
    running_.store(false, std::memory_order_relaxed);
}

void AudioRecordingSession::finalizeWav()
{
    if (capturedBytes_ == 0)
        return;

    const uint16_t channels = options_.channels > 0 ? options_.channels : 2;
    const uint32_t sampleRate = options_.sampleRate > 0 ? options_.sampleRate : 48000;
    const unsigned sampleBytes = audioSampleBytes(options_.sampleFormat);
    const uint32_t blockAlign = static_cast<uint32_t>(channels) * sampleBytes;
    if (blockAlign == 0)
        return;

    uint64_t audioStartUs = 0;
    uint64_t videoFirstUs = 0;
    uint64_t videoEndUs = 0;
    {
        std::lock_guard<std::mutex> lock(syncMutex_);
        audioStartUs = audioStartBootUs_;
        videoFirstUs = videoFirstBootUs_;
        videoEndUs = videoEndBootUs_;
    }

    const uint64_t availableFrames = capturedBytes_ / blockAlign;
    double sourceRate = sampleRate;
    const uint64_t capturedDurationUs = audioEndBootUs_ > audioStartUs ? audioEndBootUs_ - audioStartUs : 0;
    if (videoFirstUs > 0 && capturedDurationUs >= 10000000) {
        const double measuredRate = double(availableFrames) * 1000000 / capturedDurationUs;
        const double drift = measuredRate / sampleRate - 1;
        if (std::abs(drift) > 0.00002 && std::abs(drift) < 0.002)
            sourceRate = measuredRate;
        std::cerr << "ApertarCore audio clock: measured_drift_ppm=" << drift * 1000000
                  << " correction=" << (sourceRate != sampleRate ? "resample" : "none") << "\n";
    }
    uint64_t trimFrames = 0;
    uint64_t padFrames = 0;
    if (audioStartUs > 0 && videoFirstUs > 0) {
        if (videoFirstUs >= audioStartUs) {
            trimFrames = uint64_t(std::llround(double(videoFirstUs - audioStartUs) * sourceRate / 1000000));
        } else {
            padFrames = framesForDurationUsCeil(audioStartUs - videoFirstUs, sampleRate);
        }
    }

    const uint64_t sourceFramesAfterTrim = trimFrames >= availableFrames
        ? 0
        : (availableFrames - trimFrames);

    uint64_t desiredFrames = 0;
    if (videoEndUs > videoFirstUs)
        desiredFrames = framesForDurationUs(videoEndUs - videoFirstUs, sampleRate);
    if (desiredFrames == 0)
        desiredFrames = padFrames + sourceFramesAfterTrim;

    padFrames = std::min(padFrames, desiredFrames);
    const uint64_t copyFrames = desiredFrames > padFrames
        ? std::min<uint64_t>(uint64_t(sourceFramesAfterTrim * sampleRate / sourceRate), desiredFrames - padFrames)
        : 0;
    const uint64_t outputFrames = desiredFrames;
    const uint64_t outputBytes64 = outputFrames * blockAlign;
    std::cerr << "ApertarCore audio sync: video_boot_us=" << videoFirstUs
              << " audio_boot_us=" << audioStartUs << " trim_samples=" << trimFrames
              << " pad_samples=" << padFrames << " output_samples=" << outputFrames
              << " sample_rate=" << sampleRate << "\n";

    std::ifstream input(rawPath_, std::ios::binary);
    if (!input) {
        std::cerr << "ApertarCore audio finalize failed: could not reopen " << rawPath_ << "\n";
        return;
    }

    std::ofstream output(wavPath_, std::ios::binary | std::ios::trunc);
    if (!output) {
        std::cerr << "ApertarCore audio finalize failed: could not create " << wavPath_ << "\n";
        return;
    }

    const uint64_t clipStartUs = recordingClock_.wallTime(videoFirstUs > 0
        ? videoFirstUs : (audioStartUs > 0 ? audioStartUs : clockMicroseconds(CLOCK_BOOTTIME)));
    writeWavHeader(output,
                   sampleRate,
                   channels,
                   sampleBytes * 8,
                   options_.sampleFormat == "float32",
                   outputBytes64,
                   options_.firstChannel,
                   clipStartUs,
                   clipFolder_.filename().string(),
                   options_.timecodeMode,
                   options_.fps);

    std::array<char, 8192> zeroBlock{};
    uint64_t remainingPadBytes = padFrames * blockAlign;
    while (remainingPadBytes > 0) {
        const uint64_t chunk = std::min<uint64_t>(remainingPadBytes, zeroBlock.size());
        output.write(zeroBlock.data(), static_cast<std::streamsize>(chunk));
        remainingPadBytes -= chunk;
    }

    if (trimFrames > 0)
        input.seekg(static_cast<std::streamoff>(trimFrames * blockAlign), std::ios::beg);

    std::array<char, 32768> copyBuffer{};
    uint64_t remainingCopyBytes = copyFrames * blockAlign;
    if (sourceRate != sampleRate && copyFrames) {
        // Drift correction is performed once, in bounded blocks, outside capture.
        std::vector<char> samples(4096 * blockAlign);
        uint64_t cacheStart = availableFrames;
        size_t cacheFrames = 0;
        auto sampleAt = [&](uint64_t frame, unsigned channel) -> double {
            frame = std::min(frame, availableFrames - 1);
            if (frame < cacheStart || frame >= cacheStart + cacheFrames) {
                input.clear();input.seekg(std::streamoff(frame * blockAlign));
                input.read(samples.data(), samples.size());
                cacheFrames = size_t(input.gcount()) / blockAlign;cacheStart = frame;
            }
            return frame < cacheStart + cacheFrames ? readAudioSample(samples.data() + ((frame - cacheStart) * channels + channel) * sampleBytes, options_.sampleFormat) : 0;
        };
        std::vector<char> resampled(4096 * blockAlign);
        for (uint64_t first = 0; first < copyFrames;) {
            const size_t count = size_t(std::min<uint64_t>(4096, copyFrames - first));
            for (size_t i = 0; i < count; ++i) {
                const double position = trimFrames + (first + i) * sourceRate / sampleRate;
                const uint64_t frame = uint64_t(position);
                const double fraction = position - frame;
                for (unsigned channel = 0; channel < channels; ++channel) {
                    const double a = sampleAt(frame, channel);
                    const double b = sampleAt(frame + 1, channel);
                    writeAudioSample(resampled.data() + (i * channels + channel) * sampleBytes, options_.sampleFormat, a + fraction * (b - a));
                }
            }
            output.write(resampled.data(), count * blockAlign);
            first += count;
        }
        remainingCopyBytes = 0;
    }
    while (remainingCopyBytes > 0 && input.good()) {
        const size_t request = static_cast<size_t>(std::min<uint64_t>(remainingCopyBytes, copyBuffer.size()));
        input.read(copyBuffer.data(), static_cast<std::streamsize>(request));
        const std::streamsize got = input.gcount();
        if (got <= 0)
            break;
        output.write(copyBuffer.data(), got);
        remainingCopyBytes -= static_cast<uint64_t>(got);
    }

    uint64_t writtenFrames = padFrames + copyFrames;
    if (outputFrames > writtenFrames) {
        uint64_t trailingBytes = (outputFrames - writtenFrames) * blockAlign;
        while (trailingBytes > 0) {
            const uint64_t chunk = std::min<uint64_t>(trailingBytes, zeroBlock.size());
            output.write(zeroBlock.data(), static_cast<std::streamsize>(chunk));
            trailingBytes -= chunk;
        }
    }
    if (outputBytes64 & 1u) output.put('\0');
}

void AudioRecordingSession::resetState()
{
    if (playbackWriteFd_ >= 0) {
        ::close(playbackWriteFd_);
        playbackWriteFd_ = -1;
    }
    if (playbackStderrFd_ >= 0) {
        ::close(playbackStderrFd_);
        playbackStderrFd_ = -1;
    }

    if (rawStream_.is_open())
        rawStream_.close();

    if (!rawPath_.empty()) {
        std::error_code ec;
        std::filesystem::remove(rawPath_, ec);
    }

    options_ = AudioRecordingOptions{};
    levelCallback_ = {};
    clipFolder_.clear();
    rawPath_.clear();
    wavPath_.clear();
    running_.store(false, std::memory_order_relaxed);
    playbackPid_.store(-1, std::memory_order_relaxed);
    sessionActive_.store(false, std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(syncMutex_);
    audioStartBootUs_ = 0;
    videoFirstBootUs_ = 0;
    videoEndBootUs_ = 0;
    capturedBytes_ = 0;
    audioEndBootUs_ = 0;
}

void AudioRecordingSession::emitLevel(int level)
{
    if (levelCallback_)
        levelCallback_(std::clamp(level, 0, 100));
}

void AudioRecordingSession::writeWavHeader(std::ofstream &stream,
                                           uint32_t sampleRate,
                                           uint16_t channels,
                                           uint16_t bitsPerSample,
                                           bool floatingPoint,
                                           uint64_t dataSize,
                                           uint16_t firstChannel,
                                           uint64_t clipStartWallClockUs,
                                           const std::string &originatorReference,
                                           TimecodeMode timecodeMode,
                                           double fps)
{
    const uint16_t blockAlign = static_cast<uint16_t>(channels * (bitsPerSample / 8));
    const uint32_t byteRate = sampleRate * blockAlign;
    const std::tm localTime = localTimeForTimestampUs(clipStartWallClockUs);
    char dateBuffer[11]{};
    char timeBuffer[9]{};
    std::strftime(dateBuffer, sizeof(dateBuffer), "%Y-%m-%d", &localTime);
    std::strftime(timeBuffer, sizeof(timeBuffer), "%H:%M:%S", &localTime);

    uint64_t timeReferenceSamples = 0;
    if (timecodeMode == TimecodeMode::FreeRun) {
        const uint64_t secondsSinceMidnight =
            static_cast<uint64_t>(localTime.tm_hour) * 3600ULL +
            static_cast<uint64_t>(localTime.tm_min) * 60ULL +
            static_cast<uint64_t>(localTime.tm_sec);
        const int nominalFps = std::max(1, int(std::llround(fps)));
        const uint64_t frame = std::min<uint64_t>(nominalFps - 1,
            uint64_t((clipStartWallClockUs % 1000000ULL) * fps / 1000000));
        const uint64_t tcFrames = secondsSinceMidnight * nominalFps + frame;
        // Place sample zero on the same coded frame boundary as picture. The
        // samples themselves already start at the sensor's real exposure time.
        timeReferenceSamples = uint64_t(std::llround(double(tcFrames) * sampleRate / fps));
    }

    const std::string rateString = frameRateString(fps);
    const std::string tcString = timecodeString(timecodeMode, clipStartWallClockUs, 0, fps);
    std::ostringstream ixml;
    ixml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
         << "<BWFXML>\n"
         << "  <IXML_VERSION>1.5</IXML_VERSION>\n"
         << "  <PROJECT>Apertar</PROJECT>\n"
         << "  <NOTE>ApertarCore synchronized clip audio</NOTE>\n"
         << "  <TIMECODE>" << tcString << "</TIMECODE>\n"
         << "  <SPEED>\n"
         << "    <MASTER_SPEED>" << rateString << "</MASTER_SPEED>\n"
         << "    <CURRENT_SPEED>" << rateString << "</CURRENT_SPEED>\n"
         << "    <TIMECODE_FLAG>NDF</TIMECODE_FLAG>\n"
         << "    <TIMECODE_RATE>" << rateString << "</TIMECODE_RATE>\n"
         << "    <FILE_SAMPLE_RATE>" << sampleRate << "</FILE_SAMPLE_RATE>\n"
         << "    <AUDIO_BIT_DEPTH>" << bitsPerSample << "</AUDIO_BIT_DEPTH>\n"
         << "    <TIMESTAMP_SAMPLES_SINCE_MIDNIGHT_HI>" << (timeReferenceSamples >> 32) << "</TIMESTAMP_SAMPLES_SINCE_MIDNIGHT_HI>\n"
         << "    <TIMESTAMP_SAMPLES_SINCE_MIDNIGHT_LO>" << (timeReferenceSamples & 0xffffffffULL) << "</TIMESTAMP_SAMPLES_SINCE_MIDNIGHT_LO>\n"
         << "    <TIMESTAMP_SAMPLE_RATE>" << sampleRate << "</TIMESTAMP_SAMPLE_RATE>\n"
         << "  </SPEED>\n"
         << "  <TRACK_LIST><TRACK_COUNT>" << channels << "</TRACK_COUNT>\n";
    for (unsigned channel = 0; channel < channels; ++channel)
        ixml << "<TRACK><CHANNEL_INDEX>" << channel + 1 << "</CHANNEL_INDEX><INTERLEAVE_INDEX>" << channel + 1
             << "</INTERLEAVE_INDEX><NAME>Input " << firstChannel + channel << "</NAME></TRACK>\n";
    ixml << "</TRACK_LIST>\n</BWFXML>\n";
    const std::string ixmlText = ixml.str();
    const uint32_t ixmlChunkSize = static_cast<uint32_t>(ixmlText.size());
    const uint32_t ixmlChunkPaddedSize = ixmlChunkSize + (ixmlChunkSize & 1u);
    const bool extensible = channels > 2;
    const uint32_t fmtSize = extensible ? 40 : floatingPoint ? 18 : 16;
    const uint64_t riffChunkSize = 4u
        + 8u + fmtSize
        + (floatingPoint ? 12u : 0u)
        + 8u + kBextChunkSize
        + 8u + ixmlChunkPaddedSize
        + 8u + dataSize + (dataSize & 1u);

    const bool rf64 = riffChunkSize > 0xffffffffULL;
    stream.write(rf64 ? "RF64" : "RIFF", 4);
    writeUint32(stream, rf64 ? 0xffffffffu : uint32_t(riffChunkSize));
    stream.write("WAVE", 4);
    if (rf64) {
        stream.write("ds64", 4); writeUint32(stream, 28);
        writeUint64(stream, riffChunkSize + 36);
        writeUint64(stream, dataSize);
        writeUint64(stream, dataSize / blockAlign);
        writeUint32(stream, 0);
    }

    stream.write("fmt ", 4);
    writeUint32(stream, fmtSize);
    writeUint16(stream, extensible ? 0xfffe : floatingPoint ? 3 : 1);
    writeUint16(stream, channels);
    writeUint32(stream, sampleRate);
    writeUint32(stream, byteRate);
    writeUint16(stream, blockAlign);
    writeUint16(stream, bitsPerSample);
    if (extensible) {
        writeUint16(stream, 22);
        writeUint16(stream, bitsPerSample);
        writeUint32(stream, 0); // Discrete microphone channels, not surround speakers.
        writeUint32(stream, floatingPoint ? 3 : 1);
        const unsigned char guidTail[] = {0,0,0x10,0,0x80,0,0,0xaa,0,0x38,0x9b,0x71};
        stream.write(reinterpret_cast<const char *>(guidTail), sizeof(guidTail));
    } else if (floatingPoint) writeUint16(stream, 0);
    if (floatingPoint) {
        stream.write("fact", 4); writeUint32(stream, 4); writeUint32(stream, uint32_t(std::min<uint64_t>(0xffffffffu, dataSize / blockAlign)));
    }

    stream.write("bext", 4);
    writeUint32(stream, kBextChunkSize);
    writeFixedAscii(stream, "ApertarCore synchronized clip audio", 256);
    writeFixedAscii(stream, "ApertarCore", 32);
    writeFixedAscii(stream, originatorReference, 32);
    writeFixedAscii(stream, dateBuffer, 10);
    writeFixedAscii(stream, timeBuffer, 8);
    writeUint64(stream, timeReferenceSamples);
    writeUint16(stream, 1);

    std::array<char, 64> umid{};
    stream.write(umid.data(), static_cast<std::streamsize>(umid.size()));
    std::array<char, 190> reserved{};
    stream.write(reserved.data(), static_cast<std::streamsize>(reserved.size()));

    stream.write("iXML", 4);
    writeUint32(stream, ixmlChunkSize);
    stream.write(ixmlText.data(), static_cast<std::streamsize>(ixmlText.size()));
    if ((ixmlText.size() & 1u) != 0)
        stream.put('\0');

    stream.write("data", 4);
    writeUint32(stream, rf64 ? 0xffffffffu : uint32_t(dataSize));
}

} // namespace apertar
