#include "video/LibavH264EncoderClient.hpp"

#include <dlfcn.h>

#include <array>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace apertar {
namespace {

constexpr const char *kPluginName = "apertar-h264-libav.so";

std::vector<std::filesystem::path> pluginCandidates()
{
    std::vector<std::filesystem::path> paths;
    if (const char *configured = std::getenv("APERTAR_H264_LIBAV_PLUGIN"); configured && *configured)
        paths.emplace_back(configured);

    std::error_code ec;
    const std::filesystem::path executable = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (!ec && !executable.empty())
        paths.push_back(executable.parent_path() / kPluginName);

    paths.emplace_back("/home/pi/ApertarCore-build/apertar-h264-libav.so");
    paths.emplace_back(kPluginName);
    return paths;
}

template<typename Function>
Function loadSymbol(void *library, const char *name)
{
    return reinterpret_cast<Function>(::dlsym(library, name));
}

} // namespace

LibavH264EncoderClient::~LibavH264EncoderClient()
{
    closeSession();
    unload();
}

bool LibavH264EncoderClient::open(const ApertarLibavH264Config &config, std::string *error)
{
    closeSession();
    if (!load(error))
        return false;

    std::array<char, 512> message{};
    encoder_ = create_(&config, message.data(), message.size());
    if (!encoder_) {
        setError(error, message[0] ? message.data() : "Could not create the lazy libav H.264 encoder.");
        return false;
    }
    return true;
}

bool LibavH264EncoderClient::encode(const uint8_t *data, size_t size, int64_t timestampUs, std::string *error)
{
    if (!encoder_ || !encode_) {
        setError(error, "The libav H.264 encoder is not open.");
        return false;
    }

    std::array<char, 512> message{};
    if (encode_(encoder_, data, size, timestampUs, message.data(), message.size()) != 0)
        return true;

    setError(error, message[0] ? message.data() : "libav failed to encode an H.264 frame.");
    return false;
}

bool LibavH264EncoderClient::finish(std::string *error)
{
    if (!encoder_ || !finish_)
        return true;

    std::array<char, 512> message{};
    if (finish_(encoder_, message.data(), message.size()) != 0)
        return true;

    setError(error, message[0] ? message.data() : "libav failed to finalize the H.264 file.");
    return false;
}

void LibavH264EncoderClient::closeSession()
{
    if (encoder_ && destroy_)
        destroy_(encoder_);
    encoder_ = nullptr;
}

bool LibavH264EncoderClient::load(std::string *error)
{
    if (library_)
        return true;

    std::string lastError;
    for (const std::filesystem::path &candidate : pluginCandidates()) {
        ::dlerror();
        library_ = ::dlopen(candidate.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!library_) {
            if (const char *message = ::dlerror())
                lastError = message;
            continue;
        }

        const auto abi = loadSymbol<ApertarLibavH264AbiFn>(library_, "apertar_libav_h264_abi");
        create_ = loadSymbol<ApertarLibavH264CreateFn>(library_, "apertar_libav_h264_create");
        encode_ = loadSymbol<ApertarLibavH264EncodeFn>(library_, "apertar_libav_h264_encode");
        finish_ = loadSymbol<ApertarLibavH264FinishFn>(library_, "apertar_libav_h264_finish");
        destroy_ = loadSymbol<ApertarLibavH264DestroyFn>(library_, "apertar_libav_h264_destroy");
        if (abi && create_ && encode_ && finish_ && destroy_ && abi() == kApertarLibavH264PluginAbi) {
            std::cerr << "ApertarCore H.264 lazy-loaded libav plugin: "
                      << candidate.string() << "\n";
            return true;
        }

        lastError = "incompatible libav H.264 plugin ABI";
        unload();
    }

    setError(error,
             "Could not load " + std::string(kPluginName)
                 + (lastError.empty() ? std::string() : ": " + lastError));
    return false;
}

void LibavH264EncoderClient::unload()
{
    create_ = nullptr;
    encode_ = nullptr;
    finish_ = nullptr;
    destroy_ = nullptr;
    if (library_)
        ::dlclose(library_);
    library_ = nullptr;
}

void LibavH264EncoderClient::setError(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
}

} // namespace apertar

