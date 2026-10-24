#pragma once
#include <array>
#include <vector>
#include <sstream>
#include <iomanip>
#include <locale>

namespace apertar {
struct PreviewExposureReference {
    std::vector<std::array<double, 2>> decodeCurve;
    std::vector<std::array<double, 2>> displayCurve;
    std::array<double, 9> toRec709{};
};
inline std::string previewExposureReferenceJson(const PreviewExposureReference *reference)
{
    if (!reference) return "null";
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(9);
    const auto curve = [&](const auto &points) {
        out << '[';
        for (size_t i=0;i<points.size();++i) out << (i ? "," : "") << '[' << points[i][0] << ',' << points[i][1] << ']';
        out << ']';
    };
    out << "{\"decode_curve\":"; curve(reference->decodeCurve);
    out << ",\"display_curve\":"; curve(reference->displayCurve);
    out << ",\"to_rec709\":[";
    for (size_t i=0;i<9;++i) out << (i ? "," : "") << reference->toRec709[i];
    return out.str()+"]}";
}
}
