#pragma once
#include <apertar/core/ProresColorProfile.hpp>
#include <string>
#include <apertar/core/PreviewExposureReference.hpp>

namespace apertar {
// Writes an internal derived tuning, never the user's selected source file.
bool makeProresIspTuning(const std::string &source, ProresColorProfile profile,
                         bool monochrome, std::string &destination, std::string &error,
                         PreviewExposureReference *reference = nullptr);
}
