#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace apertar {

struct AudioCaptureMode {
    std::string format;
    unsigned rate = 0;
    unsigned channels = 0;
    int nativeFormat = 0;
};

// ALSA is loaded only when recording audio; preview/boot has no new dependency load.
class TimestampedAudioCapture {
public:
    TimestampedAudioCapture();
    ~TimestampedAudioCapture();
    bool open(const std::string &device, unsigned rate, unsigned channels, std::string *error);
    bool open(const std::string &device, AudioCaptureMode &mode, unsigned firstChannel, std::string *error);
    static std::vector<AudioCaptureMode> probe(const std::string &device, std::string *error);
    int read(void *data, size_t frames, int64_t *firstSampleBootUs, std::string *error);
    void close();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace apertar
