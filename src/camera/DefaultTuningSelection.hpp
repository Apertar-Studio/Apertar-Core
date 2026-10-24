#pragma once

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>
#include <yaml.h>

namespace apertar::tuning {

enum class Pipeline { Unknown, PiSP, Vc4 };

inline Pipeline pipelineFromCompatible(const std::string &compatible)
{
    for (size_t start = 0; start < compatible.size();) {
        const auto end = compatible.find('\0', start);
        const auto soc = compatible.substr(start, end == std::string::npos ? end : end - start);
        if (soc == "brcm,bcm2712")
            return Pipeline::PiSP;
        if (soc == "brcm,bcm2711" || soc == "brcm,bcm2837" || soc == "brcm,bcm2836" || soc == "brcm,bcm2835")
            return Pipeline::Vc4;
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return Pipeline::Unknown;
}

inline Pipeline detectPipeline()
{
    for (const char *path : {"/sys/firmware/devicetree/base/compatible", "/proc/device-tree/compatible"}) {
        std::ifstream input(path, std::ios::binary);
        const std::string compatible((std::istreambuf_iterator<char>(input)), {});
        const auto pipeline = pipelineFromCompatible(compatible);
        if (pipeline != Pipeline::Unknown)
            return pipeline;
    }
    return Pipeline::Unknown;
}

inline Pipeline nativePipeline()
{
    static const auto pipeline = detectPipeline();
    return pipeline;
}

inline const char *pipelineName(Pipeline pipeline)
{
    return pipeline == Pipeline::PiSP ? "pisp" : pipeline == Pipeline::Vc4 ? "vc4" : "unknown";
}

inline std::filesystem::path defaultsDirectory(Pipeline pipeline = nativePipeline())
{
    const char *overridePath = std::getenv("APERTAR_DEFAULT_TUNING_DIR");
    if (overridePath && *overridePath)
        return overridePath;
    return std::filesystem::path("/home/pi/Tuning-Files") /
        (pipeline == Pipeline::PiSP ? "defaults-pisp" : "defaults-vc4");
}

inline std::vector<std::filesystem::path> defaultDirectories(Pipeline pipeline = nativePipeline())
{
    // Do not guess a Pi pipeline on another platform. Leave selection to libcamera.
    if (pipeline == Pipeline::Unknown)
        return {};
    return {
        defaultsDirectory(pipeline),
        std::filesystem::path("/home/pi/Tuning-Files") /
            (pipeline == Pipeline::PiSP ? "defaults-pisp" : "defaults-vc4"),
        "/home/pi/Tuning-Files/defaults", // Existing images remain usable during migration.
        std::filesystem::path("/usr/local/share/libcamera/ipa/rpi") / pipelineName(pipeline),
        std::filesystem::path("/usr/share/libcamera/ipa/rpi") / pipelineName(pipeline),
    };
}

inline bool compatibleFile(const std::filesystem::path &path, Pipeline pipeline,
                           std::string *error = nullptr)
{
    const auto fail = [&](const std::string &message) {
        if (error) *error = message + ": " + path.string();
        return false;
    };
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input || input.tellg() <= 0 || input.tellg() > 4 * 1024 * 1024)
        return fail("Tuning file is unreadable, empty or too large");
    std::string text(static_cast<size_t>(input.tellg()), '\0');
    input.seekg(0);
    if (!input.read(text.data(), text.size()))
        return fail("Could not read tuning file");
    yaml_parser_t parser;
    if (!yaml_parser_initialize(&parser))
        return fail("Could not initialize tuning parser");
    yaml_parser_set_input_string(&parser, reinterpret_cast<const unsigned char *>(text.data()), text.size());
    yaml_document_t document;
    const bool parsed = yaml_parser_load(&parser, &document);
    yaml_parser_delete(&parser);
    if (!parsed)
        return fail("Invalid tuning JSON/YAML");
    auto root = yaml_document_get_root_node(&document);
    bool valid = root && root->type == YAML_MAPPING_NODE;
    // Libcamera defaults legacy files without a target to bcm2835, not PiSP.
    std::string target = "bcm2835";
    bool foundTarget = false;
    if (valid) {
        for (auto pair = root->data.mapping.pairs.start; pair != root->data.mapping.pairs.top; ++pair) {
            const auto key = yaml_document_get_node(&document, pair->key);
            if (!key || key->type != YAML_SCALAR_NODE ||
                std::string(reinterpret_cast<const char *>(key->data.scalar.value), key->data.scalar.length) != "target")
                continue;
            const auto value = yaml_document_get_node(&document, pair->value);
            if (foundTarget || !value || value->type != YAML_SCALAR_NODE) { valid = false; break; }
            foundTarget = true;
            target.assign(reinterpret_cast<const char *>(value->data.scalar.value), value->data.scalar.length);
        }
    }
    yaml_document_delete(&document);
    if (!valid || (target != "pisp" && target != "bcm2835"))
        return fail("Invalid tuning target");
    const std::string expected = pipeline == Pipeline::PiSP ? "pisp" : "bcm2835";
    if (pipeline != Pipeline::Unknown && target != expected)
        return fail("Tuning target '" + target + "' is incompatible with " + pipelineName(pipeline) +
                    " (expected '" + expected + "')");
    return true;
}

inline std::string sensorName(std::string model)
{
    const auto end = model.find_first_of(" \t\r\n");
    model.resize(end == std::string::npos ? model.size() : end);
    if (model.empty() || !std::all_of(model.begin(), model.end(), [](unsigned char ch) {
            return std::isalnum(ch) || ch == '_' || ch == '-';
        }))
        return {};
    std::transform(model.begin(), model.end(), model.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return model;
}

inline std::string fileForSensor(const std::string &model, const std::filesystem::path &directory,
                                 Pipeline pipeline = nativePipeline())
{
    const std::string name = sensorName(model);
    if (name.empty())
        return {};
    const auto path = directory / (name + ".json");
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec) || ec || !compatibleFile(path, pipeline))
        return {};
    const auto canonical = std::filesystem::weakly_canonical(path, ec);
    return ec ? path.string() : canonical.string();
}

inline std::string defaultForSensor(const std::string &model,
                                   const std::vector<std::filesystem::path> &directories,
                                   Pipeline pipeline, bool monochrome = false)
{
    // Never automatically substitute a colour calibration for a mono sensor.
    if (monochrome) {
        const std::string name = sensorName(model);
        if (name.empty())
            return {};
        const std::string monoModel = name.ends_with("_mono") ? name : name + "_mono";
        for (const auto &directory : directories) {
            auto path = fileForSensor(monoModel, directory, pipeline);
            if (!path.empty())
                return path;
        }
        return {};
    }
    for (const auto &directory : directories) {
        auto path = fileForSensor(model, directory, pipeline);
        if (!path.empty())
            return path;
    }
    return {};
}

// Subdevice names are available before CameraManager loads its IPA tuning file.
// Only preselect a unique candidate; verify against the selected camera afterward.
inline std::string preselectDefault(const std::vector<std::filesystem::path> &directories,
                                   Pipeline pipeline,
                                   const std::filesystem::path &videoDevices = "/sys/class/video4linux")
{
    std::set<std::string> candidates;
    std::error_code ec;
    std::filesystem::directory_iterator entries(videoDevices, ec);
    for (auto end = std::filesystem::directory_iterator{}; !ec && entries != end; entries.increment(ec)) {
        if (entries->path().filename().string().rfind("v4l-subdev", 0) != 0)
            continue;
        std::ifstream input(entries->path() / "name");
        std::string model;
        std::getline(input, model);
        const bool monochrome = std::filesystem::exists(entries->path() / "device/of_node/mono-mode", ec);
        ec.clear();
        const auto path = defaultForSensor(model, directories, pipeline, monochrome);
        if (!path.empty())
            candidates.insert(path);
    }
    return candidates.size() == 1 ? *candidates.begin() : std::string{};
}

} // namespace apertar::tuning
