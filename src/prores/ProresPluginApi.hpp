#pragma once
#include <cstddef>
#include <cstdint>

inline constexpr unsigned kApertarProresAbi = 2;
struct ApertarProresConfig {
    const char *path;
    const char *timecode;
    uint32_t width, height;
    double fps;
    unsigned profile, threads;
    unsigned colorProfile;
};
struct ApertarProresFrame {
    const uint8_t *data;
    size_t size;
    uint32_t width, height, stride;
    int64_t timestampUs;
    void (*release)(void *);
    void *token;
};
using ApertarProresCreate = void *(*)(const ApertarProresConfig *, char *, size_t);
using ApertarProresEncode = int (*)(void *, const ApertarProresFrame *, char *, size_t);
using ApertarProresFinish = int (*)(void *, char *, size_t);
using ApertarProresDestroy = void (*)(void *);
