#include "color/ColorProfileConfig.hpp"
#include "camera/ExposureMetering.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
#include <yaml.h>

namespace apertar {
namespace {

struct Matrix {
    float m[9]{};

    Matrix() = default;
    explicit Matrix(const std::array<float, 9> &values)
    {
        std::copy(values.begin(), values.end(), m);
    }
    Matrix(float m0, float m1, float m2,
           float m3, float m4, float m5,
           float m6, float m7, float m8)
    {
        m[0] = m0; m[1] = m1; m[2] = m2;
        m[3] = m3; m[4] = m4; m[5] = m5;
        m[6] = m6; m[7] = m7; m[8] = m8;
    }

    Matrix transpose() const
    {
        return Matrix(m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]);
    }

    Matrix cofactors() const
    {
        return Matrix(
            m[4] * m[8] - m[5] * m[7],
           -(m[3] * m[8] - m[5] * m[6]),
            m[3] * m[7] - m[4] * m[6],
           -(m[1] * m[8] - m[2] * m[7]),
            m[0] * m[8] - m[2] * m[6],
           -(m[0] * m[7] - m[1] * m[6]),
            m[1] * m[5] - m[2] * m[4],
           -(m[0] * m[5] - m[2] * m[3]),
            m[0] * m[4] - m[1] * m[3]);
    }

    float determinant() const
    {
        return m[0] * (m[4] * m[8] - m[5] * m[7]) -
               m[1] * (m[3] * m[8] - m[5] * m[6]) +
               m[2] * (m[3] * m[7] - m[4] * m[6]);
    }

    Matrix inverse() const
    {
        const float det = determinant();
        if (!std::isfinite(det) || std::fabs(det) < 1e-9f)
            throw std::runtime_error("Singular DNG colour transform");

        Matrix adj = cofactors().transpose();
        for (float &value : adj.m)
            value /= det;
        return adj;
    }

    Matrix operator*(const Matrix &other) const
    {
        Matrix result;
        for (int y = 0; y < 3; ++y) {
            for (int x = 0; x < 3; ++x) {
                result.m[y * 3 + x] =
                    m[y * 3 + 0] * other.m[x + 0] +
                    m[y * 3 + 1] * other.m[x + 3] +
                    m[y * 3 + 2] * other.m[x + 6];
            }
        }
        return result;
    }

    std::array<float, 9> toArray() const
    {
        std::array<float, 9> out{};
        std::copy(std::begin(m), std::end(m), out.begin());
        return out;
    }
};

struct RawProfile {
    std::string name;
    std::vector<std::string> matches;

    bool hasColorMatrix1 = false;
    bool hasColorMatrix2 = false;
    std::array<float, 9> colorMatrix1{};
    std::array<float, 9> colorMatrix2{};

    bool hasForwardMatrix1 = false;
    bool hasForwardMatrix2 = false;
    std::array<float, 9> forwardMatrix1{};
    std::array<float, 9> forwardMatrix2{};

    bool hasCcm = false;
    bool hasCcm1 = false;
    bool hasCcm2 = false;
    std::array<float, 9> ccm{};
    std::array<float, 9> ccm1{};
    std::array<float, 9> ccm2{};
    uint32_t ccm1Kelvin = 0;
    uint32_t ccm2Kelvin = 0;

    uint16_t illuminant1 = 21;
    uint16_t illuminant2 = 21;
};

struct ProfileStore {
    std::filesystem::file_time_type mtime{};
    bool loaded = false;
    bool fileExists = false;
    std::vector<RawProfile> profiles;
};

std::mutex &profileMutex()
{
    static std::mutex mutex;
    return mutex;
}

ProfileStore &profileStore()
{
    static ProfileStore store;
    return store;
}

std::string lowerCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string trim(std::string_view view)
{
    while (!view.empty() && std::isspace(static_cast<unsigned char>(view.front())))
        view.remove_prefix(1);
    while (!view.empty() && std::isspace(static_cast<unsigned char>(view.back())))
        view.remove_suffix(1);
    return std::string(view);
}

std::vector<std::string> splitList(const std::string &value)
{
    std::string normalized = value;
    for (char &c : normalized) {
        if (c == ',' || c == ';')
            c = ' ';
    }

    std::istringstream stream(normalized);
    std::vector<std::string> out;
    std::string token;
    while (stream >> token)
        out.push_back(token);
    return out;
}

bool parseFloatArray(const std::string &value, std::array<float, 9> *out)
{
    if (!out)
        return false;
    const std::vector<std::string> parts = splitList(value);
    if (parts.size() != 9)
        return false;

    std::array<float, 9> parsed{};
    for (size_t i = 0; i < parsed.size(); ++i) {
        char *end = nullptr;
        errno = 0;
        const float value = std::strtof(parts[i].c_str(), &end);
        if (errno != 0 || end == parts[i].c_str() || *end != '\0' || !std::isfinite(value))
            return false;
        parsed[i] = value;
    }
    *out = parsed;
    return true;
}

bool parseUint16(const std::string &value, uint16_t *out)
{
    if (!out)
        return false;
    unsigned long parsed = 0;
    char *end = nullptr;
    errno = 0;
    parsed = std::strtoul(value.c_str(), &end, 10);
    if (errno != 0 || end == value.c_str() || *end != '\0' || parsed > 0xffffUL)
        return false;
    *out = static_cast<uint16_t>(parsed);
    return true;
}

bool parseUint32(const std::string &value, uint32_t *out)
{
    if (!out)
        return false;
    unsigned long parsed = 0;
    char *end = nullptr;
    errno = 0;
    parsed = std::strtoul(value.c_str(), &end, 10);
    if (errno != 0 || end == value.c_str() || *end != '\0')
        return false;
    *out = static_cast<uint32_t>(parsed);
    return true;
}

std::filesystem::path configPath()
{
    if (const char *env = std::getenv("APERTAR_COLOR_PROFILE_CONFIG"); env && *env)
        return env;
    return "/home/pi/apertar-color-profiles.conf";
}

RawProfile imx585FallbackProfile()
{
    RawProfile profile;
    profile.name = "imx585-built-in";
    profile.matches = { "imx585" };
    profile.illuminant1 = 17;
    profile.illuminant2 = 20;
    profile.hasColorMatrix1 = true;
    profile.hasColorMatrix2 = true;
    profile.hasForwardMatrix1 = true;
    profile.hasForwardMatrix2 = true;
    profile.colorMatrix1 = {
         3.429791f, -2.015503f, -1.251441f,
         0.061013f,  1.077063f, -0.183289f,
        -0.110771f,  0.164758f,  0.463407f,
    };
    profile.colorMatrix2 = {
         1.727238f, -0.641794f, -0.438629f,
        -0.222387f,  1.340926f, -0.133617f,
        -0.291897f,  0.132585f,  0.732832f,
    };
    profile.forwardMatrix1 = {
         0.335526f,  0.334787f,  0.293906f,
         0.173946f,  0.592194f,  0.233861f,
         0.292602f, -0.524238f,  1.056837f,
    };
    profile.forwardMatrix2 = {
         0.330185f,  0.541575f,  0.092459f,
         0.149706f,  0.730722f,  0.119572f,
         0.180432f, -0.015315f,  0.660084f,
    };
    return profile;
}

void finalizeProfile(RawProfile &profile)
{
    if (profile.matches.empty())
        profile.matches.push_back(profile.name);
}

void setArrayKey(RawProfile &profile, const std::string &key, const std::string &value, int line)
{
    std::array<float, 9> parsed{};
    if (!parseFloatArray(value, &parsed)) {
        std::cerr << "ApertarCore color profile config: invalid 3x3 matrix at line "
                  << line << " key=" << key << "\n";
        return;
    }

    if (key == "color_matrix1" || key == "colour_matrix1" || key == "dng_color_matrix1") {
        profile.colorMatrix1 = parsed;
        profile.hasColorMatrix1 = true;
    } else if (key == "color_matrix2" || key == "colour_matrix2" || key == "dng_color_matrix2") {
        profile.colorMatrix2 = parsed;
        profile.hasColorMatrix2 = true;
    } else if (key == "forward_matrix1" || key == "ff_matrix1") {
        profile.forwardMatrix1 = parsed;
        profile.hasForwardMatrix1 = true;
    } else if (key == "forward_matrix2" || key == "ff_matrix2") {
        profile.forwardMatrix2 = parsed;
        profile.hasForwardMatrix2 = true;
    } else if (key == "ccm" || key == "colour_correction_matrix" || key == "color_correction_matrix") {
        profile.ccm = parsed;
        profile.hasCcm = true;
    } else if (key == "ccm1") {
        profile.ccm1 = parsed;
        profile.hasCcm1 = true;
    } else if (key == "ccm2") {
        profile.ccm2 = parsed;
        profile.hasCcm2 = true;
    }
}

std::vector<RawProfile> loadProfilesFromFile(const std::filesystem::path &path)
{
    std::ifstream file(path);
    std::vector<RawProfile> profiles;
    if (!file)
        return profiles;

    RawProfile current;
    bool haveSection = false;
    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line)) {
        ++lineNumber;
        const size_t comment = line.find_first_of("#;");
        if (comment != std::string::npos)
            line.erase(comment);
        line = trim(line);
        if (line.empty())
            continue;

        if (line.front() == '[' && line.back() == ']') {
            if (haveSection) {
                finalizeProfile(current);
                profiles.push_back(current);
            }
            current = RawProfile{};
            current.name = lowerCopy(trim(std::string_view(line).substr(1, line.size() - 2)));
            haveSection = true;
            continue;
        }

        if (!haveSection)
            continue;

        const size_t equals = line.find('=');
        if (equals == std::string::npos)
            continue;

        const std::string key = lowerCopy(trim(std::string_view(line).substr(0, equals)));
        const std::string value = trim(std::string_view(line).substr(equals + 1));
        if (key == "match" || key == "matches" || key == "sensor") {
            current.matches.clear();
            for (std::string token : splitList(value))
                current.matches.push_back(lowerCopy(token));
        } else if (key == "illuminant1") {
            parseUint16(value, &current.illuminant1);
        } else if (key == "illuminant2") {
            parseUint16(value, &current.illuminant2);
        } else if (key == "ccm1_kelvin") {
            parseUint32(value, &current.ccm1Kelvin);
        } else if (key == "ccm2_kelvin") {
            parseUint32(value, &current.ccm2Kelvin);
        } else {
            setArrayKey(current, key, value, lineNumber);
        }
    }

    if (haveSection) {
        finalizeProfile(current);
        profiles.push_back(current);
    }

    return profiles;
}

void reloadStoreIfNeeded()
{
    ProfileStore &store = profileStore();
    const std::filesystem::path path = configPath();
    std::error_code ec;
    const bool exists = std::filesystem::exists(path, ec);
    const std::filesystem::file_time_type mtime =
        exists ? std::filesystem::last_write_time(path, ec) : std::filesystem::file_time_type{};

    if (store.loaded && store.fileExists == exists && (!exists || store.mtime == mtime))
        return;

    store.loaded = true;
    store.fileExists = exists;
    store.mtime = mtime;
    store.profiles.clear();
    if (exists) {
        store.profiles = loadProfilesFromFile(path);
        std::cerr << "ApertarCore color profile config: loaded "
                  << store.profiles.size() << " profile(s) from "
                  << path.string() << "\n";
    } else {
        std::cerr << "ApertarCore color profile config: "
                  << path.string()
                  << " not found; using built-in IMX585 profile and runtime CCM fallback.\n";
    }

    store.profiles.push_back(imx585FallbackProfile());
}

int matchScore(const RawProfile &profile, const std::string &sensorLower)
{
    int best = -1;
    for (const std::string &match : profile.matches) {
        const std::string lowerMatch = lowerCopy(match);
        if (lowerMatch == "*" || lowerMatch == "default") {
            best = std::max(best, 1);
        } else if (!lowerMatch.empty() && sensorLower.find(lowerMatch) != std::string::npos) {
            best = std::max(best, static_cast<int>(lowerMatch.size()) + 10);
        }
    }
    return best;
}

std::array<float, 9> chooseCcm(const RawProfile &profile, uint32_t whiteBalanceKelvin, bool *ok)
{
    if (ok)
        *ok = false;
    if (profile.hasCcm) {
        if (ok)
            *ok = true;
        return profile.ccm;
    }
    if (profile.hasCcm1 && profile.hasCcm2) {
        const uint32_t wb = whiteBalanceKelvin > 0 ? whiteBalanceKelvin : 5600u;
        const uint32_t k1 = profile.ccm1Kelvin > 0 ? profile.ccm1Kelvin : 2850u;
        const uint32_t k2 = profile.ccm2Kelvin > 0 ? profile.ccm2Kelvin : 5600u;
        if (ok)
            *ok = true;
        if (k1 == k2)
            return profile.ccm1;

        const uint32_t lowKelvin = std::min(k1, k2);
        const uint32_t highKelvin = std::max(k1, k2);
        const float clampedWb = static_cast<float>(std::clamp(wb, lowKelvin, highKelvin));
        float t = (clampedWb - static_cast<float>(k1)) / (static_cast<float>(k2) - static_cast<float>(k1));
        t = std::clamp(t, 0.0f, 1.0f);

        std::array<float, 9> blended{};
        for (size_t i = 0; i < blended.size(); ++i)
            blended[i] = profile.ccm1[i] + (profile.ccm2[i] - profile.ccm1[i]) * t;
        return blended;
    }
    if (profile.hasCcm1) {
        if (ok)
            *ok = true;
        return profile.ccm1;
    }
    if (profile.hasCcm2) {
        if (ok)
            *ok = true;
        return profile.ccm2;
    }
    return {};
}

struct TuningCalibration {
    std::string path;
    std::filesystem::file_time_type mtime{};
    bool loaded = false;
    bool calibratedAwb = false;
    uint8_t exposureMeteringModes = 0;
    std::optional<std::array<float, 4>> blackLevels;
    std::vector<std::array<float, 3>> curve;
    std::vector<std::array<float, 2>> gammaCurve;
    std::vector<std::pair<uint32_t, std::array<float, 9>>> ccms;
};

yaml_node_t *member(yaml_document_t &doc, yaml_node_t *node, const char *key)
{
    if (!node || node->type != YAML_MAPPING_NODE) return nullptr;
    for (auto pair = node->data.mapping.pairs.start; pair != node->data.mapping.pairs.top; ++pair) {
        auto name = yaml_document_get_node(&doc, pair->key);
        if (name && name->type == YAML_SCALAR_NODE &&
            std::string_view(reinterpret_cast<char *>(name->data.scalar.value), name->data.scalar.length) == key)
            return yaml_document_get_node(&doc, pair->value);
    }
    return nullptr;
}

float scalar(yaml_node_t *node)
{
    if (!node || node->type != YAML_SCALAR_NODE) throw std::runtime_error("Expected tuning number");
    const std::string text(reinterpret_cast<char *>(node->data.scalar.value), node->data.scalar.length);
    size_t consumed = 0;
    const float value = std::stof(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value)) throw std::runtime_error("Invalid tuning number");
    return value;
}

std::vector<float> numbers(yaml_document_t &doc, yaml_node_t *node)
{
    if (!node || node->type != YAML_SEQUENCE_NODE) throw std::runtime_error("Expected tuning array");
    std::vector<float> values;
    for (auto item = node->data.sequence.items.start; item != node->data.sequence.items.top; ++item)
        values.push_back(scalar(yaml_document_get_node(&doc, *item)));
    return values;
}

uint8_t readMeteringModes(yaml_document_t &doc, yaml_node_t *channel)
{
    const auto modes = member(doc, channel, "metering_modes");
    uint8_t supported = 0;
    for (const char *name : {"centre-weighted", "spot", "matrix"}) {
        const auto mode = member(doc, modes, name);
        if (!mode) continue;
        try {
            const auto weights = numbers(doc, member(doc, mode, "weights"));
            if (!weights.empty() && std::all_of(weights.begin(), weights.end(), [](float v) { return v >= 0; }) &&
                std::any_of(weights.begin(), weights.end(), [](float v) { return v > 0; }))
                supported |= exposureMeteringBit(name);
        } catch (const std::exception &) {
            // Invalid AGC weights must not discard unrelated RAW/WB calibration.
        }
    }
    return supported;
}

void readCalibration(yaml_document_t &doc, yaml_node_t *algorithm, TuningCalibration &out)
{
    if (auto agc = member(doc, algorithm, "rpi.agc")) {
        const auto channels = member(doc, agc, "channels");
        if (channels && channels->type == YAML_SEQUENCE_NODE) {
            uint8_t supported = 7;
            bool found = false;
            for (auto item = channels->data.sequence.items.start; item != channels->data.sequence.items.top; ++item) {
                supported &= readMeteringModes(doc, yaml_document_get_node(&doc, *item));
                found = true;
            }
            out.exposureMeteringModes = found ? supported : 0;
        } else {
            out.exposureMeteringModes = readMeteringModes(doc, agc);
        }
    }
    if (auto contrast = member(doc, algorithm, "rpi.contrast")) {
        try {
            const auto node = member(doc, contrast, "gamma_curve");
            if (node) {
                const auto values = numbers(doc, node);
                if (values.size() < 4 || values.size() % 2)
                    throw std::runtime_error("Incomplete tuning gamma curve");
                for (size_t i = 0; i < values.size(); i += 2) {
                    if (values[i] < 0 || values[i] > 65535 || values[i + 1] < 0 || values[i + 1] > 65535 ||
                        (!out.gammaCurve.empty() && values[i] <= out.gammaCurve.back()[0]))
                        throw std::runtime_error("Invalid tuning gamma curve");
                    out.gammaCurve.push_back({values[i], values[i + 1]});
                }
            }
        } catch (const std::exception &error) {
            // A display-curve fallback must not invalidate RAW/WB calibration.
            out.gammaCurve.clear();
            std::cerr << "ApertarCore still tone curve: " << error.what() << '\n';
        }
    }
    if (auto black = member(doc, algorithm, "rpi.black_level")) {
        const auto readBlack = [&](const char *key, float fallback) {
            auto node = member(doc, black, key);
            const float value = node ? scalar(node) : fallback;
            if (value < 0 || value > 65535)
                throw std::runtime_error("Invalid tuning black level");
            return value;
        };
        const float base = readBlack("black_level", 4096.0f);
        const float red = readBlack("black_level_r", base) / 16.0f;
        const float green = readBlack("black_level_g", base) / 16.0f;
        const float blue = readBlack("black_level_b", base) / 16.0f;
        out.blackLevels = std::array<float, 4>{red, green, green, blue};
    }
    if (auto awb = member(doc, algorithm, "rpi.awb")) {
        const auto bayes = member(doc, awb, "bayes");
        const auto modes = member(doc, awb, "modes");
        const auto priors = member(doc, awb, "priors");
        out.calibratedAwb = (!bayes || scalar(bayes) != 0) &&
            modes && modes->type == YAML_MAPPING_NODE && modes->data.mapping.pairs.start != modes->data.mapping.pairs.top &&
            priors && priors->type == YAML_SEQUENCE_NODE && priors->data.sequence.items.start != priors->data.sequence.items.top;
        const auto curve = member(doc, awb, "ct_curve");
        auto values = curve ? numbers(doc, curve) : std::vector<float>{};
        if (!curve) out.calibratedAwb = false;
        if (curve && (values.size() < 6 || values.size() % 3)) throw std::runtime_error("Incomplete CT curve");
        for (size_t i = 0; i < values.size(); i += 3) {
            if (values[i] < 1000 || values[i] > 30000 || values[i + 1] <= 0 || values[i + 2] <= 0 ||
                (!out.curve.empty() && values[i] <= out.curve.back()[0]))
                throw std::runtime_error("Invalid CT curve");
            out.curve.push_back({values[i], values[i + 1], values[i + 2]});
        }
    }
    if (auto ccm = member(doc, algorithm, "rpi.ccm")) {
        auto list = member(doc, ccm, "ccms");
        if (!list || list->type != YAML_SEQUENCE_NODE) return;
        for (auto item = list->data.sequence.items.start; item != list->data.sequence.items.top; ++item) {
            auto entry = yaml_document_get_node(&doc, *item);
            const float temperature = scalar(member(doc, entry, "ct"));
            auto values = numbers(doc, member(doc, entry, "ccm"));
            if (values.size() != 9 || temperature < 1000 || temperature > 30000)
                throw std::runtime_error("Invalid tuning CCM");
            std::array<float, 9> matrix{};
            std::copy(values.begin(), values.end(), matrix.begin());
            out.ccms.emplace_back(static_cast<uint32_t>(temperature), matrix);
        }
        std::sort(out.ccms.begin(), out.ccms.end());
        for (size_t i = 1; i < out.ccms.size(); ++i)
            if (out.ccms[i - 1].first == out.ccms[i].first) throw std::runtime_error("Duplicate tuning CCM temperature");
    }
}

// JSON tuning is also valid YAML. Parse once per path/mtime, not per frame.
const TuningCalibration &tuningCalibration()
{
    static TuningCalibration cached;
    const char *env = std::getenv("APERTAR_SENSOR_TUNING_FILE");
    if (!env || !*env) env = std::getenv("LIBCAMERA_RPI_TUNING_FILE");
    const std::string path = env ? env : "";
    std::error_code ec;
    const auto mtime = std::filesystem::last_write_time(path, ec);
    if (cached.loaded && cached.path == path && cached.mtime == mtime) return cached;
    cached = {};
    cached.loaded = true;
    cached.path = path;
    cached.mtime = mtime;
    if (path.empty() || ec) return cached;
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input || input.tellg() < 0 || input.tellg() > 4 * 1024 * 1024) return cached;
    const size_t size = static_cast<size_t>(input.tellg());
    std::string text(size, '\0');
    input.seekg(0);
    if (!input.read(text.data(), size)) return cached;
    yaml_parser_t parser;
    if (!yaml_parser_initialize(&parser)) return cached;
    yaml_parser_set_input_string(&parser, reinterpret_cast<const unsigned char *>(text.data()), text.size());
    yaml_document_t doc;
    const bool parsed = yaml_parser_load(&parser, &doc);
    yaml_parser_delete(&parser);
    if (!parsed) return cached;
    try {
        auto root = yaml_document_get_root_node(&doc);
        auto algorithms = member(doc, root, "algorithms");
        if (algorithms && algorithms->type == YAML_SEQUENCE_NODE) {
            for (auto item = algorithms->data.sequence.items.start; item != algorithms->data.sequence.items.top; ++item)
                readCalibration(doc, yaml_document_get_node(&doc, *item), cached);
        } else {
            readCalibration(doc, root, cached); // Version 1 tuning layout.
        }
    } catch (const std::exception &error) {
        cached.curve.clear();
        cached.ccms.clear();
        cached.gammaCurve.clear();
        std::cerr << "ApertarCore DNG calibration: " << path << ": " << error.what() << '\n';
    }
    yaml_document_delete(&doc);
    return cached;
}

std::array<float, 2> calibratedGains(const TuningCalibration &calibration, float kelvin)
{
    if (calibration.curve.empty()) throw std::runtime_error("No calibrated tuning WB curve");
    auto point = calibration.curve.front();
    if (kelvin >= calibration.curve.back()[0]) point = calibration.curve.back();
    else for (size_t i = 1; i < calibration.curve.size(); ++i) {
        const auto &next = calibration.curve[i];
        if (kelvin <= next[0]) {
            const float t = std::clamp((kelvin - point[0]) / (next[0] - point[0]), 0.0f, 1.0f);
            point[1] += t * (next[1] - point[1]);
            point[2] += t * (next[2] - point[2]);
            break;
        }
        point = next;
    }
    return {1.0f / point[1], 1.0f / point[2]};
}

std::array<float, 9> calibratedCcm(const RawProfile &raw, const TuningCalibration &calibration, uint32_t kelvin)
{
    bool configured = false;
    auto result = chooseCcm(raw, kelvin, &configured);
    if (configured) return result;
    if (calibration.ccms.empty()) throw std::runtime_error("No tuning CCM");
    auto point = calibration.ccms.front();
    if (kelvin >= calibration.ccms.back().first) return calibration.ccms.back().second;
    for (size_t i = 1; i < calibration.ccms.size(); ++i) {
        const auto &next = calibration.ccms[i];
        if (kelvin <= next.first) {
            const float t = std::clamp((float(kelvin) - point.first) / (next.first - point.first), 0.0f, 1.0f);
            for (size_t n = 0; n < 9; ++n) point.second[n] += t * (next.second[n] - point.second[n]);
            return point.second;
        }
        point = next;
    }
    return point.second;
}

struct ReferenceWhite {
    uint32_t kelvin;
    std::array<float, 3> xyz;
};

ReferenceWhite referenceWhite(uint16_t illuminant)
{
    switch (illuminant) {
    case 17: return {2856, {1.09850f, 1.0f, 0.35585f}}; // CIE A
    case 20: return {5503, {0.95682f, 1.0f, 0.92149f}}; // D55
    case 21: return {6504, {0.95047f, 1.0f, 1.08883f}}; // D65
    case 22: return {7504, {0.94972f, 1.0f, 1.22638f}}; // D75
    case 23: return {5003, {0.96422f, 1.0f, 0.82521f}}; // D50
    default: throw std::runtime_error("CCM-derived DNG profile requires A/D50/D55/D65/D75 illuminants");
    }
}

Matrix rgbToXyz()
{
    return Matrix(0.4124564f, 0.3575761f, 0.1804375f,
                  0.2126729f, 0.7151522f, 0.0721750f,
                  0.0193339f, 0.1191920f, 0.9503041f);
}

Matrix adaptation(const std::array<float, 3> &from, const std::array<float, 3> &to)
{
    const Matrix bradford(0.8951f, 0.2664f, -0.1614f, -0.7502f, 1.7135f, 0.0367f, 0.0389f, -0.0685f, 1.0296f);
    Matrix scale;
    for (size_t row = 0; row < 3; ++row) {
        float a = 0, b = 0;
        for (size_t col = 0; col < 3; ++col) {
            a += bradford.m[3 * row + col] * from[col];
            b += bradford.m[3 * row + col] * to[col];
        }
        scale.m[4 * row] = b / a;
    }
    return bradford.inverse() * scale * bradford;
}

void appendMatrixKey(std::ostringstream &out, const char *name, const std::array<float, 9> &matrix)
{
    out << ';' << name << '=';
    for (float value : matrix)
        out << std::setprecision(7) << value << ',';
}

EncoderColorProfile makeEncoderProfile(const RawProfile &raw, uint32_t whiteBalanceKelvin)
{
    EncoderColorProfile out;
    out.name = raw.name;
    out.illuminant1 = raw.illuminant1;
    out.illuminant2 = raw.illuminant2;

    out.hasColorMatrix1 = raw.hasColorMatrix1;
    out.hasColorMatrix2 = raw.hasColorMatrix2;
    out.colorMatrix1 = raw.colorMatrix1;
    out.colorMatrix2 = raw.colorMatrix2;

    out.hasForwardMatrix1 = raw.hasForwardMatrix1;
    out.hasForwardMatrix2 = raw.hasForwardMatrix2;
    out.forwardMatrix1 = raw.forwardMatrix1;
    out.forwardMatrix2 = raw.forwardMatrix2;

    const auto &calibration = tuningCalibration();
    auto derive = [&](uint16_t illuminant, std::array<float, 9> &cm, bool &hasCm,
                      std::array<float, 9> &fm, bool &hasFm) {
        if (hasCm || calibration.curve.empty()) return;
        try {
            const auto reference = referenceWhite(illuminant);
            const auto ccm = calibratedCcm(raw, calibration, reference.kelvin);
            cm = dngColorMatrixFromCcm(ccm, calibratedGains(calibration, reference.kelvin), illuminant);
            hasCm = true;
            if (!hasFm) {
                fm = dngForwardMatrixFromCcm(ccm);
                hasFm = true;
            }
        } catch (const std::exception &) {
            // The writer can still use the actual frame CCM/gains as a single profile.
        }
    };
    derive(out.illuminant1, out.colorMatrix1, out.hasColorMatrix1, out.forwardMatrix1, out.hasForwardMatrix1);
    derive(out.illuminant2, out.colorMatrix2, out.hasColorMatrix2, out.forwardMatrix2, out.hasForwardMatrix2);

    bool hasCcm = false;
    out.ccm = chooseCcm(raw, whiteBalanceKelvin, &hasCcm);
    out.hasCcm = hasCcm;

    std::ostringstream key;
    key << out.name
        << ";i1=" << out.illuminant1
        << ";i2=" << out.illuminant2
        << ";cm1=" << out.hasColorMatrix1
        << ";cm2=" << out.hasColorMatrix2
        << ";fm1=" << out.hasForwardMatrix1
        << ";fm2=" << out.hasForwardMatrix2
        << ";ccm=" << out.hasCcm;
    if (out.hasColorMatrix1) appendMatrixKey(key, "cm1v", out.colorMatrix1);
    if (out.hasColorMatrix2) appendMatrixKey(key, "cm2v", out.colorMatrix2);
    if (out.hasForwardMatrix1) appendMatrixKey(key, "fm1v", out.forwardMatrix1);
    if (out.hasForwardMatrix2) appendMatrixKey(key, "fm2v", out.forwardMatrix2);
    if (out.hasCcm) appendMatrixKey(key, "ccmv", out.ccm);
    out.cacheKey = key.str();
    return out;
}

} // namespace

std::array<float, 9> dngColorMatrixFromCcm(const std::array<float, 9> &ccm,
                                        const std::array<float, 2> &gains, uint16_t illuminant)
{
    for (float gain : gains)
        if (!std::isfinite(gain) || gain <= 0) throw std::runtime_error("Invalid DNG calibration WB gain");
    const Matrix cameraCcm(ccm);
    const Matrix wb(gains[0], 0, 0, 0, 1, 0, 0, 0, gains[1]);
    // IPA CCM maps balanced camera RGB to linear sRGB (D65). Undo that
    // adaptation for the declared calibration light before mapping XYZ to RAW.
    return ((rgbToXyz() * cameraCcm * wb).inverse() *
            adaptation(referenceWhite(illuminant).xyz, referenceWhite(21).xyz)).toArray();
}

std::array<float, 9> dngForwardMatrixFromCcm(const std::array<float, 9> &ccm)
{
    Matrix result = adaptation(referenceWhite(21).xyz, referenceWhite(23).xyz) * rgbToXyz() * Matrix(ccm);
    const auto white = referenceWhite(23).xyz;
    // The DNG forward transform must map balanced (1,1,1) to XYZ D50.
    for (size_t row = 0; row < 3; ++row) {
        const float sum = result.m[3 * row] + result.m[3 * row + 1] + result.m[3 * row + 2];
        if (!std::isfinite(sum) || sum <= 0) throw std::runtime_error("Invalid DNG forward transform");
        for (size_t col = 0; col < 3; ++col) result.m[3 * row + col] *= white[row] / sum;
    }
    return result.toArray();
}

std::optional<EncoderColorProfile> encoderColorProfileForSensor(const std::string &sensorModel,
                                                                uint32_t whiteBalanceKelvin)
{
    std::lock_guard<std::mutex> lock(profileMutex());
    reloadStoreIfNeeded();

    const std::string sensorLower = lowerCopy(sensorModel);
    const RawProfile *best = nullptr;
    int bestScore = -1;
    for (const RawProfile &profile : profileStore().profiles) {
        const int score = matchScore(profile, sensorLower);
        if (score > bestScore) {
            best = &profile;
            bestScore = score;
        }
    }

    if (!best || bestScore < 0) {
        if (tuningCalibration().curve.empty() || tuningCalibration().ccms.empty()) return std::nullopt;
        RawProfile tuningProfile;
        tuningProfile.name = "active-tuning";
        tuningProfile.illuminant1 = 17;
        tuningProfile.illuminant2 = 21;
        return makeEncoderProfile(tuningProfile, whiteBalanceKelvin);
    }
    return makeEncoderProfile(*best, whiteBalanceKelvin);
}

std::optional<std::array<float, 9>> encoderDaylightColorMatrixForSensor(const std::string &sensorModel)
{
    const auto profile = encoderColorProfileForSensor(sensorModel, 6504);
    if (!profile || (!profile->hasColorMatrix1 && !profile->hasColorMatrix2))
        return std::nullopt;

    std::optional<std::array<float, 9>> result;
    float closest = std::numeric_limits<float>::max();
    auto consider = [&](bool present, const std::array<float, 9> &values, uint16_t illuminant) {
        if (!present)
            return;
        try {
            const auto reference = referenceWhite(illuminant);
            const float distance = std::abs(1.0f / reference.kelvin - 1.0f / 6504.0f);
            if (distance >= closest)
                return;
            // Preserve native channel sensitivities, including WB scaling.
            result = illuminant == 21 ? values
                : (Matrix(values) * adaptation(referenceWhite(21).xyz, reference.xyz)).toArray();
            closest = distance;
        } catch (const std::exception &) {
            // An unsupported calibration light can still use the runtime CCM.
        }
    };
    consider(profile->hasColorMatrix1, profile->colorMatrix1, profile->illuminant1);
    consider(profile->hasColorMatrix2, profile->colorMatrix2, profile->illuminant2);
    return result;
}

bool activeTuningHasCalibratedWhiteBalance()
{
    std::lock_guard<std::mutex> lock(profileMutex());
    const auto &calibration = tuningCalibration();
    return calibration.calibratedAwb && calibration.curve.size() >= 2;
}

uint8_t activeTuningExposureMeteringModes()
{
    std::lock_guard<std::mutex> lock(profileMutex());
    return tuningCalibration().exposureMeteringModes;
}

std::optional<std::array<float, 4>> activeTuningBlackLevels()
{
    std::lock_guard<std::mutex> lock(profileMutex());
    return tuningCalibration().blackLevels;
}

std::vector<std::array<float, 2>> activeTuningGammaCurve()
{
    std::lock_guard<std::mutex> lock(profileMutex());
    return tuningCalibration().gammaCurve;
}

} // namespace apertar
