#include "TimestampedAudioCapture.hpp"
#include "RecordingClock.hpp"
#include "AudioSample.hpp"
#include <alsa/asoundlib.h>
#include <algorithm>
#include <dlfcn.h>
#include <cstring>
#include <fstream>
#include <regex>
#include <sstream>

namespace apertar {

#define ALSA_FUNCTIONS(X) \
    X(snd_pcm_open) X(snd_pcm_close) X(snd_pcm_set_params) \
    X(snd_pcm_sw_params_malloc) X(snd_pcm_sw_params_free) X(snd_pcm_sw_params_current) \
    X(snd_pcm_sw_params_set_tstamp_mode) X(snd_pcm_sw_params_set_tstamp_type) \
    X(snd_pcm_sw_params_set_avail_min) X(snd_pcm_sw_params) X(snd_pcm_start) \
    X(snd_pcm_wait) X(snd_pcm_htimestamp) X(snd_pcm_readi) X(snd_pcm_recover) X(snd_strerror) \
    X(snd_pcm_hw_params_malloc) X(snd_pcm_hw_params_free) X(snd_pcm_hw_params_any) \
    X(snd_pcm_hw_params_set_access) X(snd_pcm_hw_params_set_format) \
    X(snd_pcm_hw_params_set_rate) X(snd_pcm_hw_params_test_channels) X(snd_pcm_hw_params_set_channels)

struct TimestampedAudioCapture::Impl {
    void *library = nullptr;
    snd_pcm_t *pcm = nullptr;
    unsigned rate = 0;
    unsigned nativeChannels = 0, outputChannels = 0, firstChannel = 0;
    unsigned nativeBytes = 0, outputBytes = 0;
    std::vector<char> scratch;
    bool started = false;
#define DECLARE(name) decltype(&::name) name = nullptr;
    ALSA_FUNCTIONS(DECLARE)
#undef DECLARE
    bool result(int code, std::string *error)
    {
        if (code >= 0) return true;
        if (error) *error = std::string("ALSA capture: ") + snd_strerror(code);
        return false;
    }
    bool load(std::string *error)
    {
        library = ::dlopen("libasound.so.2", RTLD_NOW | RTLD_LOCAL);
        if (!library) { if (error) *error = "Could not load libasound.so.2."; return false; }
#define LOAD(name) name = reinterpret_cast<decltype(name)>(::dlsym(library, #name)); if (!name) { if (error) *error = "Missing ALSA function " #name; return false; }
        ALSA_FUNCTIONS(LOAD)
#undef LOAD
        return true;
    }
    ~Impl()
    {
        if (pcm) snd_pcm_close(pcm);
        if (library) ::dlclose(library);
    }
};

TimestampedAudioCapture::TimestampedAudioCapture() : impl_(std::make_unique<Impl>()) {}
TimestampedAudioCapture::~TimestampedAudioCapture() = default;

bool TimestampedAudioCapture::open(const std::string &device, unsigned rate, unsigned channels, std::string *error)
{
    AudioCaptureMode mode{"s16", rate, channels};
    return open(device, mode, 1, error);
}

namespace {
std::string hardwareDevice(std::string device)
{
    if (device.rfind("plughw:", 0) == 0) device.erase(0, 4);
    return device;
}

std::vector<AudioCaptureMode> usbDescriptorModes(const std::string &device)
{
    std::smatch match;
    if (!std::regex_match(device, match, std::regex("hw:([0-9]+),([0-9]+)"))) return {};
    std::ifstream input("/proc/asound/card" + match[1].str() + "/stream" + match[2].str());
    std::vector<AudioCaptureMode> modes;
    bool capture = false;
    std::string line, format;
    int native = -1;
    unsigned channels = 0;
    while (std::getline(input, line)) {
        if (line == "Capture:") { capture = true; continue; }
        if (line == "Playback:") { capture = false; continue; }
        if (!capture) continue;
        const auto first = line.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        line.erase(0, first);
        if (line.rfind("Altset", 0) == 0) { format.clear(); native = -1; channels = 0; }
        if (line.rfind("Format: ", 0) == 0) {
            const auto f = line.substr(8);
            if (f == "S16_LE") { format = "s16"; native = SND_PCM_FORMAT_S16_LE; }
            else if (f == "S24_3LE") { format = "s24"; native = SND_PCM_FORMAT_S24_3LE; }
            else if (f == "S24_LE") { format = "s24"; native = SND_PCM_FORMAT_S24_LE; }
            else if (f == "S32_LE") { format = "s32"; native = SND_PCM_FORMAT_S32_LE; }
            else if (f == "FLOAT_LE") { format = "float32"; native = SND_PCM_FORMAT_FLOAT_LE; }
        }
        if (line.rfind("Channels: ", 0) == 0) { std::istringstream values(line.substr(10)); values >> channels; }
        if (line.rfind("Rates: ", 0) == 0 && native >= 0 && channels > 0 && channels <= 32) {
            const std::regex numbers("[0-9]+");
            std::vector<unsigned> rates;
            for (auto it = std::sregex_iterator(line.begin(), line.end(), numbers); it != std::sregex_iterator(); ++it)
                rates.push_back(unsigned(std::stoul(it->str())));
            if (line.find("continuous") != std::string::npos && rates.size() >= 2) {
                const auto min = rates[0], max = rates[1]; rates.clear();
                for (unsigned rate : {8000u, 16000u, 22050u, 24000u, 32000u, 44100u, 48000u, 88200u, 96000u, 176400u, 192000u})
                    if (rate >= min && rate <= max) rates.push_back(rate);
            }
            for (unsigned rate : rates) if (rate <= 192000) modes.push_back({format, rate, channels, native});
        }
    }
    return modes;
}
}

std::vector<AudioCaptureMode> TimestampedAudioCapture::probe(const std::string &device, std::string *error)
{
    Impl s;
    std::vector<AudioCaptureMode> modes;
    if (!s.load(error)) return modes;
    const auto hardware = hardwareDevice(device);
    const int opened = s.snd_pcm_open(&s.pcm, hardware.c_str(), SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK);
    if (opened < 0) {
        // USB descriptors expose complete native altsettings even while the
        // idle VU meter owns the PCM. Never infer capabilities from plughw.
        if (opened == -EBUSY) modes = usbDescriptorModes(hardware);
        if (modes.empty()) s.result(opened, error);
        return modes;
    }
    snd_pcm_hw_params_t *params = nullptr;
    if (!s.result(s.snd_pcm_hw_params_malloc(&params), error)) return modes;
    const std::pair<const char *, snd_pcm_format_t> formats[] = {
        {"s16", SND_PCM_FORMAT_S16_LE}, {"s24", SND_PCM_FORMAT_S24_3LE},
        {"s24", SND_PCM_FORMAT_S24_LE}, {"s32", SND_PCM_FORMAT_S32_LE},
        {"float32", SND_PCM_FORMAT_FLOAT_LE}
    };
    // Refine the combined constraint space, not independent lists that could
    // advertise an unsupported rate/format/channel combination.
    for (const auto &format : formats) for (unsigned rate : {48000u, 44100u, 96000u, 88200u, 192000u, 176400u, 32000u, 24000u, 22050u, 16000u, 8000u}) {
        if (s.snd_pcm_hw_params_any(s.pcm, params) < 0 ||
            s.snd_pcm_hw_params_set_access(s.pcm, params, SND_PCM_ACCESS_RW_INTERLEAVED) < 0 ||
            s.snd_pcm_hw_params_set_format(s.pcm, params, format.second) < 0 ||
            s.snd_pcm_hw_params_set_rate(s.pcm, params, rate, 0) < 0) continue;
        for (unsigned channels = 1; channels <= 32; ++channels) {
            if (s.snd_pcm_hw_params_test_channels(s.pcm, params, channels) >= 0)
                modes.push_back({format.first, rate, channels, int(format.second)});
        }
    }
    s.snd_pcm_hw_params_free(params);
    if (modes.empty() && error) *error = "No supported native PCM capture mode (16/24/32-bit or float, up to 32 channels).";
    return modes;
}

bool TimestampedAudioCapture::open(const std::string &device, AudioCaptureMode &mode, unsigned firstChannel, std::string *error)
{
    close();
    const auto modes = probe(device, error);
    const auto requested = mode;
    auto selected = modes.end();
    unsigned bestScore = ~0u;
    for (auto it = modes.begin(); it != modes.end(); ++it) {
        if ((!requested.format.empty() && requested.format != "auto" && requested.format != it->format) ||
            (requested.rate && requested.rate != it->rate)) continue;
        const unsigned outputChannels = requested.channels == 65535 ? it->channels
            : requested.channels ? requested.channels : std::min(2u, it->channels);
        if (!firstChannel || firstChannel - 1 + outputChannels > it->channels) continue;
        const unsigned score = (it->rate == 48000 ? 0 : 1000000) +
            (it->format == "s16" ? 0 : 100000) +
            (requested.channels == 65535 ? (32 - it->channels) * 10000000 : (2 - std::min(2u, outputChannels)) * 10000 + it->channels * 100);
        if (score < bestScore) { selected = it; bestScore = score; }
    }
    if (selected == modes.end()) {
        if (error && !modes.empty()) *error = "Selected audio format, rate or input channels are not supported by this device. Choose Auto or refresh Audio settings.";
        return false;
    }
    mode = *selected;
    mode.channels = requested.channels == 65535 ? selected->channels
        : requested.channels ? requested.channels : std::min(2u, selected->channels);
    auto &s = *impl_;
    if (!s.load(error)) return false;
    s.rate = mode.rate;
    s.nativeChannels = selected->channels;
    s.outputChannels = mode.channels;
    s.firstChannel = firstChannel - 1;
    s.outputBytes = audioSampleBytes(mode.format);
    s.nativeBytes = selected->nativeFormat == SND_PCM_FORMAT_S24_LE ? 4 : s.outputBytes;
    if (!s.result(s.snd_pcm_open(&s.pcm, hardwareDevice(device).c_str(), SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK), error) ||
        !s.result(s.snd_pcm_set_params(s.pcm, snd_pcm_format_t(mode.nativeFormat), SND_PCM_ACCESS_RW_INTERLEAVED,
                                    s.nativeChannels, s.rate, 0, 100000), error)) return false;
    snd_pcm_sw_params_t *sw = nullptr;
    if (!s.result(s.snd_pcm_sw_params_malloc(&sw), error)) return false;
    const bool configured = s.result(s.snd_pcm_sw_params_current(s.pcm, sw), error) &&
        s.result(s.snd_pcm_sw_params_set_tstamp_mode(s.pcm, sw, SND_PCM_TSTAMP_ENABLE), error) &&
        s.result(s.snd_pcm_sw_params_set_tstamp_type(s.pcm, sw, SND_PCM_TSTAMP_TYPE_MONOTONIC), error) &&
        s.result(s.snd_pcm_sw_params_set_avail_min(s.pcm, sw, std::max(1u, s.rate / 100)), error) &&
        s.result(s.snd_pcm_sw_params(s.pcm, sw), error);
    s.snd_pcm_sw_params_free(sw);
    return configured;
}

int TimestampedAudioCapture::read(void *data, size_t frames, int64_t *firstSampleBootUs, std::string *error)
{
    auto &s = *impl_;
    if (!s.started) {
        if (!s.result(s.snd_pcm_start(s.pcm), error)) return -1;
        s.started = true;
    }
    int result = s.snd_pcm_wait(s.pcm, 50);
    if (result == 0) return 0;
    if (result < 0) {
        if (!s.result(s.snd_pcm_recover(s.pcm, result, 1), error)) return -1;
        s.started = false;
        return 0;
    }
    snd_pcm_uframes_t available = 0;
    snd_htimestamp_t stamp{};
    result = s.snd_pcm_htimestamp(s.pcm, &available, &stamp);
    if (result < 0) {
        if (!s.result(s.snd_pcm_recover(s.pcm, result, 1), error)) return -1;
        s.started = false;
        return 0;
    }
    if (!available) return 0;
    const int64_t stampUs = int64_t(stamp.tv_sec) * 1000000 + stamp.tv_nsec / 1000;
    if (stampUs <= 0) { if (error) *error = "Audio device did not provide a capture timestamp."; return -1; }
    const int64_t bootOffset = clockMicroseconds(CLOCK_BOOTTIME) - clockMicroseconds(CLOCK_MONOTONIC);
    *firstSampleBootUs = stampUs + bootOffset - int64_t(available * 1000000ULL / s.rate);
    const bool remap = s.nativeChannels != s.outputChannels || s.firstChannel || s.nativeBytes != s.outputBytes;
    if (remap) s.scratch.resize(frames * s.nativeChannels * s.nativeBytes);
    const auto got = s.snd_pcm_readi(s.pcm, remap ? s.scratch.data() : data, std::min<size_t>(frames, available));
    if (got == -EAGAIN) return 0;
    if (got < 0) {
        if (!s.result(s.snd_pcm_recover(s.pcm, int(got), 1), error)) return -1;
        s.started = false;
        return 0;
    }
    if (remap) for (size_t frame = 0; frame < size_t(got); ++frame) for (unsigned ch = 0; ch < s.outputChannels; ++ch)
        std::memcpy(static_cast<char *>(data) + (frame * s.outputChannels + ch) * s.outputBytes,
                    s.scratch.data() + (frame * s.nativeChannels + s.firstChannel + ch) * s.nativeBytes, s.outputBytes);
    return int(got);
}

void TimestampedAudioCapture::close() { impl_ = std::make_unique<Impl>(); }
#undef ALSA_FUNCTIONS
} // namespace apertar
