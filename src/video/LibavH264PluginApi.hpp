#pragma once

#include <cstddef>
#include <cstdint>

constexpr uint32_t kApertarLibavH264PluginAbi = 2;

struct ApertarLibavH264Config {
    uint32_t abiVersion = kApertarLibavH264PluginAbi;
    const char *outputPath = nullptr;
    const char *encoder = nullptr;
    const char *preset = nullptr;
    const char *tune = nullptr;
    const char *crf = nullptr;
    const char *qp = nullptr;
    const char *threads = nullptr;
    const char *bitrate = nullptr;
    const char *maxrate = nullptr;
    const char *bufsize = nullptr;
    const char *x264Params = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    double fps = 0.0;
    bool freeRunTimecode = false;
};

using ApertarLibavH264Handle = void *;
using ApertarLibavH264AbiFn = uint32_t (*)();
using ApertarLibavH264CreateFn = ApertarLibavH264Handle (*)(
    const ApertarLibavH264Config *, char *, size_t);
using ApertarLibavH264EncodeFn = int (*)(
    ApertarLibavH264Handle, const uint8_t *, size_t, int64_t, char *, size_t);
using ApertarLibavH264FinishFn = int (*)(
    ApertarLibavH264Handle, char *, size_t);
using ApertarLibavH264DestroyFn = void (*)(ApertarLibavH264Handle);

