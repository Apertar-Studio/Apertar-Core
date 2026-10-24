#pragma once

#include "video/LibavH264PluginApi.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace apertar {

class LibavH264EncoderClient {
public:
    LibavH264EncoderClient() = default;
    ~LibavH264EncoderClient();

    LibavH264EncoderClient(const LibavH264EncoderClient &) = delete;
    LibavH264EncoderClient &operator=(const LibavH264EncoderClient &) = delete;

    bool open(const ApertarLibavH264Config &config, std::string *error);
    bool encode(const uint8_t *data, size_t size, int64_t timestampUs, std::string *error = nullptr);
    bool finish(std::string *error = nullptr);
    void closeSession();
    bool isOpen() const { return encoder_ != nullptr; }

private:
    bool load(std::string *error);
    void unload();
    static void setError(std::string *error, const std::string &message);

    void *library_ = nullptr;
    ApertarLibavH264Handle encoder_ = nullptr;
    ApertarLibavH264CreateFn create_ = nullptr;
    ApertarLibavH264EncodeFn encode_ = nullptr;
    ApertarLibavH264FinishFn finish_ = nullptr;
    ApertarLibavH264DestroyFn destroy_ = nullptr;
};

} // namespace apertar

