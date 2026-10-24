#include "ProresPluginApi.hpp"
#include "ProresEncoder.hpp"
#include <stdexcept>
#include <cstdio>
#include <exception>
#include <memory>

namespace {
void message(char *out, size_t size, const std::string &text) {
    if (out && size) std::snprintf(out, size, "%s", text.c_str());
}
}
extern "C" unsigned apertar_prores_abi() { return kApertarProresAbi; }
extern "C" void *apertar_prores_create(const ApertarProresConfig *cfg, char *error, size_t size) {
    try {
        if (!cfg || !cfg->path || cfg->width < 2 || cfg->height < 2 ||
            cfg->width % 2 || cfg->height % 2 || !(cfg->fps > 0) || cfg->profile > 3 ||
            cfg->colorProfile > static_cast<unsigned>(apertar::ProresColorProfile::P3D60Linear))
            throw std::runtime_error("Invalid ProRes configuration");
        apertar::ProresEncoder::Config config;
        config.width = cfg->width; config.height = cfg->height; config.fps = cfg->fps;
        config.threads = cfg->threads;
        config.profile = static_cast<apertar::ProresEncoder::Profile>(cfg->profile);
        config.transfer = apertar::ProresTransfer::Display;
        config.colorProfile = static_cast<apertar::ProresColorProfile>(cfg->colorProfile);
        config.timecode = cfg->timecode ? cfg->timecode : "";
        auto encoder = std::make_unique<apertar::ProresEncoder>(config);
        std::string why;
        if (!encoder->openClip(cfg->path, &why)) throw std::runtime_error(why);
        return encoder.release();
    } catch (const std::exception &e) { message(error, size, e.what()); return nullptr; }
}
extern "C" int apertar_prores_encode(void *handle, const ApertarProresFrame *frame, char *error, size_t size) {
    // writeFrame owns release on every return path, including validation errors.
    apertar::IspFrameView view;
    view.data = frame->data; view.size = frame->size;
    view.width = frame->width; view.height = frame->height; view.stride = frame->stride;
    view.timestampUs = frame->timestampUs; view.release = frame->release; view.token = frame->token;
    try {
        std::string why;
        const bool ok = static_cast<apertar::ProresEncoder *>(handle)->writeFrame(view, &why);
        if (!ok) message(error, size, why);
        return ok;
    } catch (const std::exception &e) { message(error, size, e.what()); return 0; }
}
extern "C" int apertar_prores_finish(void *handle, char *error, size_t size) {
    try {
        std::string why;
        const bool ok = static_cast<apertar::ProresEncoder *>(handle)->closeClip(&why);
        if (!ok) message(error, size, why);
        return ok;
    } catch (const std::exception &e) { message(error, size, e.what()); return 0; }
}
extern "C" void apertar_prores_destroy(void *handle) {
    delete static_cast<apertar::ProresEncoder *>(handle);
}
