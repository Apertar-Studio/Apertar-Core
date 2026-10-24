#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>
#include <utility>

namespace apertar {

inline std::pair<int,int> recordingFrameRate(double fps)
{
    for (int numerator : {24000,30000,60000,120000})
        if (std::abs(fps - double(numerator) / 1001) < 0.001)
            return {numerator,1001};
    return {int(std::llround(std::max(1.0, fps) * 1000)),1000};
}

inline double recordingFps(double fps)
{
    const auto rate = recordingFrameRate(fps);
    return double(rate.first) / rate.second;
}

inline int64_t clockMicroseconds(clockid_t clock)
{
    timespec time{};
    return ::clock_gettime(clock, &time) == 0
        ? int64_t(time.tv_sec) * 1000000 + time.tv_nsec / 1000 : 0;
}

class RecordingClock {
public:
    RecordingClock()
    {
        const auto before = clockMicroseconds(CLOCK_BOOTTIME);
        const auto wall = clockMicroseconds(CLOCK_REALTIME);
        offsetUs_ = wall - (before + clockMicroseconds(CLOCK_BOOTTIME)) / 2;
    }
    uint64_t wallTime(int64_t bootUs) const
    {
        return uint64_t(std::max<int64_t>(0, bootUs + offsetUs_));
    }
private:
    int64_t offsetUs_ = 0;
};

inline std::string recordingTimecode(bool freeRun, uint64_t epochUs, double fps)
{
    const int nominal = std::max(1, int(std::llround(fps)));
    if (!freeRun) return "00:00:00:00";
    const time_t seconds = time_t(epochUs / 1000000);
    tm local{};
    localtime_r(&seconds, &local);
    const int frame = std::min(nominal - 1, int((epochUs % 1000000) * fps / 1000000));
    char value[32];
    std::snprintf(value, sizeof(value), "%02d:%02d:%02d:%02d", local.tm_hour, local.tm_min, local.tm_sec, frame);
    return value;
}

} // namespace apertar
