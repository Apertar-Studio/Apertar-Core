#include "ProresRepair.hpp"
#include <apertar/core/ProresColorProfile.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
}

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>

namespace apertar {
namespace prores_repair {
namespace {

std::string avErr(int e)
{
    char b[AV_ERROR_MAX_STRING_SIZE] = {0};
    av_strerror(e, b, sizeof(b));
    return b;
}

uint32_t be32(const uint8_t *p)
{
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

uint64_t be64(const uint8_t *p)
{
    return (uint64_t(be32(p)) << 32) | uint64_t(be32(p + 4));
}

struct Atoms {
    bool haveMoov = false;
    uint64_t mdatPayload = 0;   /* first byte of frame data */
    uint64_t mdatEnd = 0;       /* one past the last byte the header claims */
    bool mdatSizeKnown = false;
};

/* Top-level atom walk. Tolerates the truncated file this exists for: a size of
 * 0 means "to end of file", and a header that runs off the end simply ends the
 * walk rather than being an error. */
Atoms scanAtoms(std::ifstream &f, uint64_t fileSize)
{
    Atoms a;
    uint64_t off = 0;
    while (off + 8 <= fileSize) {
        uint8_t hdr[16];
        f.seekg(std::streamoff(off));
        f.read(reinterpret_cast<char *>(hdr), 8);
        if (!f) break;
        uint64_t size = be32(hdr);
        const uint32_t type = be32(hdr + 4);
        uint64_t headerLen = 8;
        if (size == 1) {
            f.read(reinterpret_cast<char *>(hdr + 8), 8);
            if (!f) break;
            size = be64(hdr + 8);
            headerLen = 16;
        } else if (size == 0) {
            size = fileSize - off;      /* "runs to end of file" */
        }
        if (size < headerLen) break;

        if (type == 0x6D6F6F76u /* moov */) a.haveMoov = true;
        if (type == 0x6D646174u /* mdat */) {
            a.mdatPayload = off + headerLen;
            a.mdatEnd = off + size > fileSize ? fileSize : off + size;
            a.mdatSizeKnown = true;
        }
        off += size;
    }
    return a;
}

struct SidecarInfo {
    int width = 0;
    int height = 0;
    double fps = 0.0;
    std::string fourcc;
    ProresColorProfile colorProfile = ProresColorProfile::Standard;
};

/* Deliberately a scanner rather than a JSON parser: this file is written by
 * ProresEncoder::openClip a few lines long, and a repair path should not fail
 * because a dependency was unavailable. */
bool readSidecar(const std::string &moviePath, SidecarInfo *out)
{
    std::ifstream in(recoverySidecarPath(moviePath));
    if (!in) return false;
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string text = ss.str();
    auto num = [&](const char *key, double *dst) {
        const size_t k = text.find(key);
        if (k == std::string::npos) return false;
        const size_t c = text.find(':', k);
        if (c == std::string::npos) return false;
        *dst = std::atof(text.c_str() + c + 1);
        return true;
    };
    double v = 0;
    if (num("\"width\"", &v)) out->width = int(v);
    if (num("\"height\"", &v)) out->height = int(v);
    if (num("\"fps\"", &v)) out->fps = v;
    const size_t colorKey = text.find("\"color_profile\"");
    if (colorKey != std::string::npos) {
        const size_t colon = text.find(':', colorKey);
        const size_t first = text.find('"', colon);
        const size_t last = first == std::string::npos ? first : text.find('"', first + 1);
        if (first != std::string::npos && last != std::string::npos)
            parseProresColorProfile(text.substr(first + 1, last - first - 1), out->colorProfile);
    }
    const size_t p = text.find("\"profile\"");
    if (p != std::string::npos) {
        const size_t q1 = text.find('"', text.find(':', p));
        if (q1 != std::string::npos && q1 + 5 <= text.size())
            out->fourcc = text.substr(q1 + 1, 4);
    }
    return out->width > 0 && out->height > 0;
}

struct Frame { uint64_t offset; uint32_t size; };

/* Walk the mdat by frame headers. Stops at the first thing that is not a
 * complete 'icpf' sample, which is exactly where the interruption happened. */
std::vector<Frame> scanFrames(std::ifstream &f, uint64_t begin, uint64_t end)
{
    std::vector<Frame> frames;
    uint64_t off = begin;
    // The MOV muxer can put its single four-byte tmcd sample before video.
    // Recognize only that prefix, not an arbitrary search through damaged data.
    if (off + 12 <= end) {
        uint8_t prefix[12];
        f.seekg(std::streamoff(off));
        f.read(reinterpret_cast<char *>(prefix), sizeof(prefix));
        if (f && be32(prefix + 4) != 0x69637066u && be32(prefix + 8) == 0x69637066u &&
            be32(prefix + 4) >= 28 && off + 4 + be32(prefix + 4) <= end)
            off += 4;
    }
    while (off + 8 <= end) {
        uint8_t hdr[8];
        f.seekg(std::streamoff(off));
        f.read(reinterpret_cast<char *>(hdr), 8);
        if (!f) break;
        const uint32_t size = be32(hdr);
        if (be32(hdr + 4) != 0x69637066u /* icpf */) break;
        if (size < 8) break;
        /* A frame whose tail was never written is NOT recovered: a half sample
         * decodes to garbage, and the point of this is a clip that plays. */
        if (off + size > end) break;
        frames.push_back(Frame{off, size});
        off += size;
    }
    return frames;
}

}  // namespace

std::string recoverySidecarPath(const std::string &moviePath)
{
    const size_t dot = moviePath.find_last_of('.');
    const size_t slash = moviePath.find_last_of('/');
    const bool hasExt = dot != std::string::npos && (slash == std::string::npos || dot > slash);
    return (hasExt ? moviePath.substr(0, dot) : moviePath) + "_recovery.json";
}

bool needsRepair(const std::string &path)
{
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) return false;
    const uint64_t size = std::filesystem::file_size(path, ec);
    if (ec || size < 16) return false;
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    const Atoms a = scanAtoms(f, size);
    return !a.haveMoov && a.mdatSizeKnown;
}

Result repair(const std::string &path)
{
    Result r;
    if (!needsRepair(path)) return r;
    r.attempted = true;

    std::error_code ec;
    const uint64_t fileSize = std::filesystem::file_size(path, ec);
    if (ec) { r.message = "cannot stat " + path; return r; }

    std::ifstream in(path, std::ios::binary);
    if (!in) { r.message = "cannot open " + path; return r; }

    const Atoms a = scanAtoms(in, fileSize);
    const std::vector<Frame> frames = scanFrames(in, a.mdatPayload, a.mdatEnd);
    r.bytesScanned = a.mdatEnd - a.mdatPayload;
    if (frames.empty()) {
        r.message = "no complete ProRes frames in the mdat -- nothing to recover";
        return r;
    }

    SidecarInfo info;
    const bool haveSidecar = readSidecar(path, &info);
    if (!haveSidecar) {
        /* The bitstream carries its own geometry, so a lost sidecar costs the
         * frame rate and the profile, not the clip. */
        std::vector<uint8_t> hdr(24);
        in.seekg(std::streamoff(frames.front().offset));
        in.read(reinterpret_cast<char *>(hdr.data()), std::streamsize(hdr.size()));
        info.width = (hdr[16] << 8) | hdr[17];
        info.height = (hdr[18] << 8) | hdr[19];

        /* THE PROFILE IS INFERRED FROM THE DATA RATE, because it is nowhere in
         * the bitstream. A ProRes frame header carries geometry and chroma
         * format but not the profile -- the profile IS the stsd fourcc, which
         * is precisely what was lost with the moov. So this measures bytes per
         * pixel and picks the nearest of Apple's published rates.
         *
         * Apple's UHD figures work out at roughly, in bytes per pixel:
         *     Proxy 0.061   LT 0.142   422 0.206   HQ 0.309
         * and the boundaries below sit between them. Getting it wrong is
         * cosmetic -- every decoder reads the frames from the bitstream, not
         * from the tag -- but a clip that reports the wrong profile invites
         * someone to distrust it, so it is worth doing properly.
         *
         * Only ever reached when the sidecar is missing. A clip recorded by a
         * build that writes the sidecar carries its exact profile. */
        double bpp = 0.0;
        if (info.width > 0 && info.height > 0 && !frames.empty()) {
            uint64_t total = 0;
            const size_t sample = std::min<size_t>(frames.size(), 64);
            for (size_t i = 0; i < sample; ++i) total += frames[i].size;
            bpp = double(total) / double(sample)
                / (double(info.width) * double(info.height));
        }
        if (bpp < 0.100)      info.fourcc = "apco";   /* Proxy */
        else if (bpp < 0.175) info.fourcc = "apcs";   /* LT */
        else if (bpp < 0.260) info.fourcc = "apcn";   /* 422 */
        else                  info.fourcc = "apch";   /* HQ */
        std::cerr << "ApertarCore ProRes repair: no sidecar for this clip; "
                     "geometry " << info.width << "x" << info.height
                  << " read from the bitstream and profile inferred as "
                  << info.fourcc << " from " << bpp << " bytes/pixel\n";
    }
    if (info.fps <= 0.0) info.fps = 30.0;
    if (info.fourcc.size() != 4) info.fourcc = "apcn";
    if (info.width <= 0 || info.height <= 0) {
        r.message = "could not determine the clip's geometry";
        return r;
    }

    const std::string tmp = path + ".repair-tmp.mov";
    AVFormatContext *out = nullptr;
    int ret = avformat_alloc_output_context2(&out, nullptr, "mov", tmp.c_str());
    if (ret < 0) { r.message = "mov muxer: " + avErr(ret); return r; }

    AVStream *st = avformat_new_stream(out, nullptr);
    if (!st) { avformat_free_context(out); r.message = "no stream"; return r; }

    const int den = 1000;
    const int num = int(lround(info.fps * den));
    st->time_base = AVRational{den, num};
    st->avg_frame_rate = AVRational{num, den};
    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id = AV_CODEC_ID_PRORES;
    st->codecpar->codec_tag = MKTAG(info.fourcc[0], info.fourcc[1], info.fourcc[2], info.fourcc[3]);
    st->codecpar->width = info.width;
    st->codecpar->height = info.height;
    st->codecpar->format = AV_PIX_FMT_YUV422P10LE;
    // Preserve the recorder's transfer tag, including explicitly linear ProRes.
    st->codecpar->color_trc = info.colorProfile == ProresColorProfile::P3D60Linear ? AVCOL_TRC_LINEAR : AVCOL_TRC_UNSPECIFIED;
    st->codecpar->color_primaries = info.colorProfile == ProresColorProfile::Standard ? AVCOL_PRI_BT709 : AVCOL_PRI_UNSPECIFIED;
    st->codecpar->color_space = AVCOL_SPC_BT709;
    st->codecpar->color_range = AVCOL_RANGE_MPEG;
    av_dict_set(&out->metadata, "apertar_color_profile", toString(info.colorProfile), 0);
    av_dict_set(&out->metadata, "apertar_input_color_space", proresColorSpace(info.colorProfile), 0);
    av_dict_set(&out->metadata, "apertar_input_gamma", proresGamma(info.colorProfile), 0);
    av_dict_set(&out->metadata, "apertar_data_levels", "Video", 0);

    if ((ret = avio_open(&out->pb, tmp.c_str(), AVIO_FLAG_WRITE)) < 0) {
        avformat_free_context(out);
        r.message = "cannot write " + tmp + ": " + avErr(ret);
        return r;
    }
    AVDictionary *options = nullptr;
    av_dict_set(&options, "movflags", "+use_metadata_tags+write_colr", 0);
    ret = avformat_write_header(out, &options);
    av_dict_free(&options);
    if (ret < 0) {
        avio_closep(&out->pb);
        avformat_free_context(out);
        std::remove(tmp.c_str());
        r.message = "write_header: " + avErr(ret);
        return r;
    }

    AVPacket *pkt = av_packet_alloc();
    std::vector<uint8_t> buf;
    bool wrote = true;
    for (size_t i = 0; i < frames.size(); ++i) {
        buf.resize(frames[i].size);
        in.seekg(std::streamoff(frames[i].offset));
        in.read(reinterpret_cast<char *>(buf.data()), std::streamsize(buf.size()));
        if (!in) { wrote = false; r.message = "read failed at frame " + std::to_string(i); break; }

        av_packet_unref(pkt);
        if (av_new_packet(pkt, int(buf.size())) < 0) { wrote = false; r.message = "out of memory"; break; }
        std::memcpy(pkt->data, buf.data(), buf.size());
        pkt->stream_index = 0;
        pkt->pts = pkt->dts = int64_t(i);
        pkt->duration = 1;
        pkt->flags |= AV_PKT_FLAG_KEY;   /* ProRes is all-intra */
        av_packet_rescale_ts(pkt, AVRational{1, int(lround(info.fps))}, st->time_base);
        if ((ret = av_interleaved_write_frame(out, pkt)) < 0) {
            wrote = false;
            r.message = "write_frame: " + avErr(ret);
            break;
        }
    }
    av_packet_free(&pkt);

    if (wrote && (ret = av_write_trailer(out)) < 0) {
        wrote = false;
        r.message = "write_trailer: " + avErr(ret);
    }
    avio_closep(&out->pb);
    avformat_free_context(out);
    in.close();

    if (!wrote) { std::remove(tmp.c_str()); return r; }

    /* ATOMIC. The original is kept until the replacement is complete and named,
     * so an interruption during a repair cannot lose what the repair was for. */
    const std::string keep = path + ".unrepaired";
    if (std::rename(path.c_str(), keep.c_str()) != 0) {
        std::remove(tmp.c_str());
        r.message = std::string("cannot set aside the original: ") + std::strerror(errno);
        return r;
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        std::rename(keep.c_str(), path.c_str());   /* put it back */
        std::remove(tmp.c_str());
        r.message = std::string("cannot install the repaired clip: ") + std::strerror(errno);
        return r;
    }
    std::remove(keep.c_str());

    r.ok = true;
    r.framesRecovered = frames.size();
    return r;
}

unsigned repairAll(const std::string &mediaRoot)
{
    std::error_code ec;
    if (!std::filesystem::is_directory(mediaRoot, ec)) return 0;

    unsigned repaired = 0;
    for (const auto &entry : std::filesystem::directory_iterator(mediaRoot, ec)) {
        if (ec) break;
        if (!entry.is_directory(ec)) continue;
        for (const auto &file : std::filesystem::directory_iterator(entry.path(), ec)) {
            if (ec) break;
            if (!file.is_regular_file(ec)) continue;
            const std::string p = file.path().string();
            if (p.size() < 4 || p.compare(p.size() - 4, 4, ".mov") != 0) continue;
            if (!needsRepair(p)) continue;

            std::cerr << "ApertarCore ProRes: " << p << " has no moov atom -- it was "
                         "interrupted. Rebuilding its index.\n";
            const Result res = repair(p);
            if (res.ok) {
                ++repaired;
                std::cerr << "ApertarCore ProRes: recovered " << res.framesRecovered
                          << " frames into " << p << "\n";
            } else {
                std::cerr << "ApertarCore ProRes: could NOT repair " << p << ": "
                          << res.message << "\n";
            }
        }
    }
    return repaired;
}

}  // namespace prores_repair
}  // namespace apertar

