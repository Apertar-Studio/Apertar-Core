#include "video/LibavH264PluginApi.hpp"
#include "video/H264Hardware.hpp"
#include "video/NativeV4l2H264Encoder.hpp"
#include "audio/RecordingClock.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/bsf.h>
#include <libavformat/avformat.h>
#include <libavutil/dict.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
#include <libavutil/rational.h>
}

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <string>

namespace {

void setError(char *buffer, size_t size, const std::string &message)
{
    if (!buffer || size == 0)
        return;
    std::snprintf(buffer, size, "%s", message.c_str());
}

std::string avErrorString(int code)
{
    char message[AV_ERROR_MAX_STRING_SIZE] = {};
    if (av_strerror(code, message, sizeof(message)) == 0)
        return message;
    return "unknown FFmpeg error";
}

int64_t parseBitrate(const char *value)
{
    if (!value || !*value)
        return 0;

    char *end = nullptr;
    const double numeric = std::strtod(value, &end);
    if (end == value || !std::isfinite(numeric) || numeric <= 0.0)
        return 0;
    while (*end == ' ' || *end == '\t')
        ++end;

    double multiplier = 1.0;
    if (*end == 'k' || *end == 'K') { multiplier = 1000.0; ++end; }
    else if (*end == 'm' || *end == 'M') { multiplier = 1000.0 * 1000.0; ++end; }
    else if (*end == 'g' || *end == 'G') { multiplier = 1000.0 * 1000.0 * 1000.0; ++end; }
    while (*end == ' ' || *end == '\t') ++end;
    const double result = numeric * multiplier;
    if (*end || !std::isfinite(result) || result >= static_cast<double>(std::numeric_limits<int64_t>::max()))
        return 0;
    return static_cast<int64_t>(result);
}

bool isX264Encoder(const char *encoder)
{
    return encoder && (std::strcmp(encoder, "libx264") == 0
                       || std::strcmp(encoder, "libx264rgb") == 0);
}

bool hasAnnexBVcl(const AVPacket *packet)
{
    for (int i = 0; i + 3 < packet->size; ++i) {
        if (packet->data[i] || packet->data[i + 1]) continue;
        int header = -1;
        if (packet->data[i + 2] == 1) header = i + 3;
        else if (i + 4 < packet->size && packet->data[i + 2] == 0 && packet->data[i + 3] == 1)
            header = i + 4;
        if (header >= 0) {
            const unsigned int type = packet->data[header] & 31;
            if (type >= 1 && type <= 5) return true;
        }
    }
    return false;
}

class Encoder {
public:
    ~Encoder()
    {
        close();
    }

    bool open(const ApertarLibavH264Config &config, std::string *error)
    {
        freeRunTimecode_ = config.freeRunTimecode;
        fps_ = apertar::recordingFps(config.fps);
        if (config.abiVersion != kApertarLibavH264PluginAbi
            || !config.outputPath || !config.encoder
            || config.width < 2 || config.height < 2 || config.width % 2 || config.height % 2
            || !std::isfinite(config.fps) || !(config.fps > 0.0)) {
            *error = "Invalid lazy libav H.264 encoder configuration.";
            return false;
        }

        hardware_ = std::strcmp(config.encoder, "h264_v4l2m2m") == 0;
        const AVCodec *codec = hardware_ ? nullptr : avcodec_find_encoder_by_name(config.encoder);
        if (!hardware_ && !codec) {
            *error = "FFmpeg/libav encoder not found: " + std::string(config.encoder);
            return false;
        }
        if (hardware_ && (config.width > 1920 || config.height > 1080 || config.fps > 30.0005)) {
            *error = "Pi hardware H.264 supports at most 1920x1080 at 30 FPS.";
            return false;
        }

        int ret = avformat_alloc_output_context2(&formatContext_, nullptr, "mp4", config.outputPath);
        if (ret < 0 || !formatContext_) {
            *error = "Could not create MP4 muxer: " + avErrorString(ret);
            return false;
        }

        videoStream_ = avformat_new_stream(formatContext_, nullptr);
        codecContext_ = avcodec_alloc_context3(codec);
        if (!videoStream_ || !codecContext_) {
            *error = "Could not allocate the H.264 stream or encoder context.";
            return false;
        }

        const auto standardRate = apertar::recordingFrameRate(config.fps);
        const AVRational frameRate{standardRate.first, standardRate.second};
        codecContext_->codec_id = hardware_ ? AV_CODEC_ID_H264 : codec->id;
        codecContext_->codec_type = AVMEDIA_TYPE_VIDEO;
        codecContext_->width = static_cast<int>(config.width);
        codecContext_->height = static_cast<int>(config.height);
        codecContext_->pix_fmt = AV_PIX_FMT_YUV420P;
        codecContext_->time_base = av_inv_q(frameRate);
        codecContext_->framerate = frameRate;
        codecContext_->gop_size = std::max(1, static_cast<int>(std::lround(config.fps)));
        codecContext_->max_b_frames = 0;
        if (hardware_) {
            codecContext_->color_range = AVCOL_RANGE_MPEG;
            codecContext_->color_primaries = AVCOL_PRI_BT709;
            codecContext_->color_trc = AVCOL_TRC_BT709;
            codecContext_->colorspace = AVCOL_SPC_BT709;
            codecContext_->profile = AV_PROFILE_H264_HIGH;
            codecContext_->level = 40;
        }
        width_ = config.width;
        height_ = config.height;
        frameBytes_ = static_cast<size_t>(width_) * height_ * 3u / 2u;

        const int64_t bitrate = parseBitrate(config.bitrate);
        const int64_t maxrate = parseBitrate(config.maxrate);
        const int64_t bufsize = parseBitrate(config.bufsize);
        if (hardware_ && ((config.bitrate && *config.bitrate && bitrate < 25'000) || bitrate > 25'000'000)) {
            *error = "Pi hardware H.264 bitrate must be between 25 kbit/s and 25 Mbit/s.";
            return false;
        }
        if (bitrate > 0)
            codecContext_->bit_rate = bitrate;
        if (maxrate > 0)
            codecContext_->rc_max_rate = maxrate;
        if (bufsize > 0) {
            codecContext_->rc_buffer_size = static_cast<int>(
                std::min<int64_t>(bufsize, std::numeric_limits<int>::max()));
        }

        if (config.threads && *config.threads) {
            char *end = nullptr;
            const unsigned long threads = std::strtoul(config.threads, &end, 10);
            if (end && *end == '\0' && threads > 0)
                codecContext_->thread_count = static_cast<int>(std::min<unsigned long>(threads, 16ul));
        }
        if (formatContext_->oformat && (formatContext_->oformat->flags & AVFMT_GLOBALHEADER))
            codecContext_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

        AVDictionary *options = nullptr;
        if (isX264Encoder(config.encoder)) {
            av_dict_set(&options, "preset", config.preset ? config.preset : "ultrafast", 0);
            if (config.tune && *config.tune)
                av_dict_set(&options, "tune", config.tune, 0);
            if (config.qp && *config.qp)
                av_dict_set(&options, "qp", config.qp, 0);
            else if (bitrate <= 0)
                av_dict_set(&options, "crf", config.crf ? config.crf : "22", 0);
            if (config.x264Params && *config.x264Params)
                av_dict_set(&options, "x264-params", config.x264Params, 0);
        } else if (bitrate <= 0) {
            codecContext_->bit_rate = hardware_ ? 20'000'000 : 80'000'000;
        }
        // BCM2835 uses native DMA-BUF transport, not FFmpeg's V4L2 encoder wrapper.
        ret = hardware_ ? 0 : avcodec_open2(codecContext_, codec, &options);
        av_dict_free(&options);
        if (ret < 0) {
            *error = "Could not open H.264 encoder: " + avErrorString(ret);
            return false;
        }

        ret = avcodec_parameters_from_context(videoStream_->codecpar, codecContext_);
        if (ret < 0) {
            *error = "Could not copy H.264 encoder parameters: " + avErrorString(ret);
            return false;
        }
        videoStream_->time_base = codecContext_->time_base;
        videoStream_->avg_frame_rate = videoStream_->r_frame_rate = frameRate;

        if (hardware_) {
            const auto *filter = av_bsf_get_by_name("extract_extradata");
            ret = filter ? av_bsf_alloc(filter, &extract_) : AVERROR_BSF_NOT_FOUND;
            if (ret >= 0) ret = avcodec_parameters_copy(extract_->par_in, videoStream_->codecpar);
            if (ret >= 0) {
                extract_->time_base_in = codecContext_->time_base;
                // Separate V4L2 SPS/PPS buffers belong in avcC, not an extra video sample.
                ret = av_opt_set_int(extract_->priv_data, "remove", 1, 0);
                if (ret >= 0) ret = av_bsf_init(extract_);
            }
            if (ret < 0) { *error = "Could not initialize hardware H.264 header extraction: " + avErrorString(ret); return false; }
        }

        ret = avio_open(&formatContext_->pb, config.outputPath, AVIO_FLAG_WRITE);
        if (ret < 0) {
            *error = "Could not open H.264 output file: " + avErrorString(ret);
            return false;
        }

        frame_ = av_frame_alloc();
        packet_ = av_packet_alloc();
        if (!frame_ || !packet_) {
            *error = "Could not allocate H.264 frame or packet buffers.";
            return false;
        }
        frame_->format = codecContext_->pix_fmt;
        frame_->width = codecContext_->width;
        frame_->height = codecContext_->height;
        if (hardware_) {
            const auto device = apertar::findRaspberryPiH264Encoder(error);
            if (device.empty()) return false;
            if (!native_.open(device, width_, height_, frameRate.num, frameRate.den,
                              static_cast<int>(codecContext_->bit_rate),
                              [this](const uint8_t *data, size_t bytes, int64_t pts, bool key, std::string *message) {
                                  return nativePacket(data, bytes, pts, key, message);
                              }, error)) return false;
            std::fprintf(stderr, "ApertarCore H.264 native V4L2/DMA-BUF encoder: %s %ux%u; libav MP4 mux only\n",
                         device.c_str(), width_, height_);
        }
        return true;
    }

    bool encode(const uint8_t *data, size_t size, int64_t timestampUs, std::string *error)
    {
        if (!codecContext_ || !formatContext_ || !videoStream_ || !frame_ || size < frameBytes_) {
            *error = "The lazy libav H.264 encoder received an invalid frame.";
            return false;
        }

        const size_t yBytes = static_cast<size_t>(width_) * height_;
        const size_t uvBytes = static_cast<size_t>(width_ / 2u) * (height_ / 2u);
        frame_->data[0] = const_cast<uint8_t *>(data);
        frame_->data[1] = const_cast<uint8_t *>(data + yBytes);
        frame_->data[2] = const_cast<uint8_t *>(data + yBytes + uvBytes);
        frame_->linesize[0] = static_cast<int>(width_);
        frame_->linesize[1] = static_cast<int>(width_ / 2u);
        frame_->linesize[2] = static_cast<int>(width_ / 2u);
        if (firstTimestampUs_ < 0) {
            firstTimestampUs_ = timestampUs;
            const auto tc = apertar::recordingTimecode(freeRunTimecode_, clock_.wallTime(timestampUs), fps_);
            av_dict_set(&videoStream_->metadata, "timecode", tc.c_str(), 0);
        }
        frame_->pts = std::max(nextPts_, av_rescale_q(std::max<int64_t>(0, timestampUs - firstTimestampUs_),
            AVRational{1,1000000}, codecContext_->time_base));
        nextPts_ = frame_->pts + 1;

        if (hardware_) {
            const int64_t ptsUs = av_rescale_q(frame_->pts, codecContext_->time_base, AVRational{1,1000000});
            if (!native_.encode(data, size, ptsUs, error)) return false;
            ++nativeSubmittedFrames_;
            return true;
        }

        int ret = avcodec_send_frame(codecContext_, frame_);
        if (ret == AVERROR(EAGAIN)) {
            if (!drain(error))
                return false;
            ret = avcodec_send_frame(codecContext_, frame_);
        }
        if (ret < 0) {
            *error = "Could not submit H.264 frame: " + avErrorString(ret);
            return false;
        }
        return drain(error);
    }

    bool finish(std::string *error)
    {
        if (!codecContext_ || !formatContext_)
            return true;

        int ret;
        if (hardware_) {
            if (!native_.finish(error)) return false;
            native_.close();
        } else {
            ret = avcodec_send_frame(codecContext_, nullptr);
            if (ret < 0 && ret != AVERROR_EOF) {
                *error = "Could not flush H.264 encoder: " + avErrorString(ret);
                return false;
            }
            if (!drain(error)) return false;
        }
        if (extract_) {
            ret = av_bsf_send_packet(extract_, nullptr);
            if (ret < 0 && ret != AVERROR_EOF) { *error = avErrorString(ret); return false; }
            if (!drainExtracted(error)) return false;
        }
        if (hardware_ && nativeSubmittedFrames_ != nativeWrittenFrames_) {
            *error = "Native H.264 frame count mismatch: submitted " + std::to_string(nativeSubmittedFrames_)
                + ", written " + std::to_string(nativeWrittenFrames_) + ".";
            return false;
        }
        if (headerWritten_) {
            ret = av_write_trailer(formatContext_);
            headerWritten_ = false;
            if (ret < 0) {
                *error = "Could not finalize MP4 file: " + avErrorString(ret);
                return false;
            }
        }
        return true;
    }

private:
    bool nativePacket(const uint8_t *data, size_t size, int64_t ptsUs, bool key, std::string *error)
    {
        if (size > static_cast<size_t>(std::numeric_limits<int>::max())) {
            *error = "Native H.264 packet is too large.";
            return false;
        }
        av_packet_unref(packet_);
        int ret = av_new_packet(packet_, static_cast<int>(size));
        if (ret < 0) { *error = "Could not allocate native H.264 packet: " + avErrorString(ret); return false; }
        std::memcpy(packet_->data, data, size);
        packet_->pts = packet_->dts = av_rescale_q(ptsUs, AVRational{1,1000000}, codecContext_->time_base);
        packet_->duration = 1;
        if (key) packet_->flags |= AV_PKT_FLAG_KEY;
        ret = av_bsf_send_packet(extract_, packet_);
        if (ret < 0) { *error = "Could not extract native H.264 headers: " + avErrorString(ret); return false; }
        const bool result = drainExtracted(error);
        av_packet_unref(packet_);
        return result;
    }

    bool writePacket(std::string *error)
    {
        const bool picture = !hardware_ || hasAnnexBVcl(packet_);
        if (!headerWritten_) {
            size_t bytes = 0;
            const auto *extra = av_packet_get_side_data(packet_, AV_PKT_DATA_NEW_EXTRADATA, &bytes);
            if (extra && bytes && bytes <= 1024 * 1024) {
                auto *parameters = videoStream_->codecpar;
                av_freep(&parameters->extradata);
                parameters->extradata = static_cast<uint8_t *>(av_mallocz(bytes + AV_INPUT_BUFFER_PADDING_SIZE));
                if (!parameters->extradata) { *error = "Could not allocate H.264 headers."; return false; }
                std::memcpy(parameters->extradata, extra, bytes);
                parameters->extradata_size = static_cast<int>(bytes);
            }
            if (packet_->size == 0 || !picture) return true;
            // V4L2 supplies SPS/PPS with its first packet, not at avcodec_open2.
            if (hardware_ && videoStream_->codecpar->extradata_size == 0) {
                *error = "Hardware H.264 encoder supplied no SPS/PPS headers.";
                return false;
            }
            const int ret = avformat_write_header(formatContext_, nullptr);
            if (ret < 0) { *error = "Could not write hardware H.264 MP4 header: " + avErrorString(ret); return false; }
            headerWritten_ = true;
        }
        // SPS/PPS, AUD and SEI-only buffers are not additional video samples.
        if (packet_->size == 0 || !picture) return true;
        if (packet_->duration <= 0) packet_->duration = 1;
        av_packet_rescale_ts(packet_, codecContext_->time_base, videoStream_->time_base);
        packet_->stream_index = videoStream_->index;
        const int ret = av_interleaved_write_frame(formatContext_, packet_);
        if (ret < 0) { *error = "Could not write H.264 packet: " + avErrorString(ret); return false; }
        if (hardware_) ++nativeWrittenFrames_;
        return true;
    }

    bool drainExtracted(std::string *error)
    {
        while (true) {
            const int ret = av_bsf_receive_packet(extract_, packet_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) return true;
            if (ret < 0) { *error = "Could not extract H.264 headers: " + avErrorString(ret); return false; }
            const bool written = writePacket(error);
            av_packet_unref(packet_);
            if (!written) return false;
        }
    }

    bool drain(std::string *error)
    {
        while (true) {
            const int ret = avcodec_receive_packet(codecContext_, packet_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                return true;
            if (ret < 0) {
                *error = "Could not receive H.264 packet: " + avErrorString(ret);
                return false;
            }

            bool written;
            if (extract_) {
                const int sent = av_bsf_send_packet(extract_, packet_);
                if (sent < 0) { *error = "Could not extract H.264 headers: " + avErrorString(sent); return false; }
                written = drainExtracted(error);
            } else written = writePacket(error);
            av_packet_unref(packet_);
            if (!written) return false;
        }
    }

    void close()
    {
        native_.close();
        av_bsf_free(&extract_);
        if (packet_)
            av_packet_free(&packet_);
        if (frame_)
            av_frame_free(&frame_);
        if (codecContext_)
            avcodec_free_context(&codecContext_);
        if (formatContext_) {
            if (formatContext_->pb)
                avio_closep(&formatContext_->pb);
            avformat_free_context(formatContext_);
        }
        formatContext_ = nullptr;
        videoStream_ = nullptr;
    }

    AVFormatContext *formatContext_ = nullptr;
    AVCodecContext *codecContext_ = nullptr;
    AVStream *videoStream_ = nullptr;
    AVFrame *frame_ = nullptr;
    AVPacket *packet_ = nullptr;
    AVBSFContext *extract_ = nullptr;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    size_t frameBytes_ = 0;
    int64_t nextPts_ = 0;
    int64_t firstTimestampUs_ = -1;
    apertar::RecordingClock clock_;
    bool freeRunTimecode_ = false;
    double fps_ = 24;
    bool headerWritten_ = false;
    bool hardware_ = false;
    uint64_t nativeSubmittedFrames_ = 0, nativeWrittenFrames_ = 0;
    apertar::NativeV4l2H264Encoder<> native_;
};

} // namespace

extern "C" uint32_t apertar_libav_h264_abi()
{
    return kApertarLibavH264PluginAbi;
}

extern "C" ApertarLibavH264Handle apertar_libav_h264_create(
    const ApertarLibavH264Config *config, char *error, size_t errorSize)
{
    if (!config) {
        setError(error, errorSize, "Missing lazy libav H.264 configuration.");
        return nullptr;
    }

    try {
        auto encoder = std::make_unique<Encoder>();
        std::string message;
        if (!encoder->open(*config, &message)) {
            setError(error, errorSize, message);
            return nullptr;
        }
        return encoder.release();
    } catch (const std::exception &exception) {
        setError(error, errorSize, exception.what());
    } catch (...) {
        setError(error, errorSize, "Unknown lazy libav H.264 initialization failure.");
    }
    return nullptr;
}

extern "C" int apertar_libav_h264_encode(
    ApertarLibavH264Handle handle,
    const uint8_t *data,
    size_t size,
    int64_t timestampUs,
    char *error,
    size_t errorSize)
{
    if (!handle)
        return 0;
    std::string message;
    const bool success = static_cast<Encoder *>(handle)->encode(data, size, timestampUs, &message);
    if (!success)
        setError(error, errorSize, message);
    return success ? 1 : 0;
}

extern "C" int apertar_libav_h264_finish(
    ApertarLibavH264Handle handle, char *error, size_t errorSize)
{
    if (!handle)
        return 1;
    std::string message;
    const bool success = static_cast<Encoder *>(handle)->finish(&message);
    if (!success)
        setError(error, errorSize, message);
    return success ? 1 : 0;
}

extern "C" void apertar_libav_h264_destroy(ApertarLibavH264Handle handle)
{
    delete static_cast<Encoder *>(handle);
}

