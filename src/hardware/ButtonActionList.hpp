#pragma once

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace apertar::hardware {

inline constexpr size_t maxButtonActions = 6;
inline constexpr unsigned maxHardwareScripts = 5;

inline int scriptActionSlot(std::string_view action)
{
    for (const std::string_view prefix : {"run_script", "script"}) {
        if (action == prefix) return 0;
        if (action.substr(0, prefix.size()) != prefix) continue;
        auto suffix = action.substr(prefix.size());
        if (!suffix.empty() && suffix.front() == '_') suffix.remove_prefix(1);
        if (suffix.size() == 1 && suffix.front() >= '1' && suffix.front() <= '5') return suffix.front() - '0';
    }
    return -1;
}

inline std::string normalizeScriptAction(std::string_view action)
{
    const int slot = scriptActionSlot(action);
    return slot < 0 ? std::string() : slot == 0 ? "run_script" : "run_script" + std::to_string(slot);
}

inline std::string actionToken(std::string_view text)
{
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) return {};
    const auto last = text.find_last_not_of(" \t\r\n");
    std::string token(text.substr(first, last - first + 1));
    std::transform(token.begin(), token.end(), token.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    // Preserve the sign of EV shortcuts before generic punctuation normalization.
    if (token.size() > 3 && token.substr(0, 3) == "ev-" &&
        (std::isdigit(static_cast<unsigned char>(token[3])) || token[3] == '.'))
        token.replace(2, 1, "m");
    else if (token.size() > 3 && token.substr(0, 3) == "ev+" &&
             (std::isdigit(static_cast<unsigned char>(token[3])) || token[3] == '.'))
        token.erase(2, 1);
    for (char &ch : token) {
        if (ch == '-' || ch == ' ' || ch == '.') ch = '_';
    }
    return token;
}

inline bool isOutputAction(const std::string &action)
{
    return action == "recindicatorled" || action == "rec_indicator_led" ||
        action == "recording_indicator" || action == "recording_indicator_led" ||
        action == "record_led" || action == "rec_led" || action == "tally" || action == "tally_led";
}

struct ButtonActionList {
    std::vector<std::string> actions;
    std::string error;

    std::string joined() const
    {
        std::string result;
        for (const auto &action : actions) {
            if (!result.empty()) result += ',';
            result += action;
        }
        return result;
    }
};

template<class Normalizer>
ButtonActionList parseButtonActions(std::string_view text, Normalizer normalize)
{
    ButtonActionList result;
    size_t start = 0;
    size_t count = 0;
    do {
        const auto end = text.find(',', start);
        const auto token = actionToken(text.substr(start, end == std::string_view::npos ? end : end - start));
        if (++count > maxButtonActions) {
            result.error = "at most six actions are allowed per mapping";
            break;
        }
        if (token.empty() && text.find(',') != std::string_view::npos) {
            result.error = "empty action in comma-separated mapping";
            break;
        }
        if (isOutputAction(token)) {
            if (text.find(',') != std::string_view::npos)
                result.error = "recording LED output must be mapped alone, not combined with button actions";
            break;
        }
        auto action = normalize(token);
        if (action == "none" || action == "off" || action == "disabled") action.clear();
        if (!action.empty() && std::find(result.actions.begin(), result.actions.end(), action) == result.actions.end())
            result.actions.push_back(std::move(action));
        if (end == std::string_view::npos) break;
        start = end + 1;
    } while (true);
    if (!result.error.empty()) result.actions.clear();
    return result;
}

template<class Normalizer>
std::string normalizeButtonMapping(const std::string &text, Normalizer normalize)
{
    const auto result = parseButtonActions(text, normalize);
    if (!result.error.empty())
        std::cerr << "Hardware input mapping ignored: " << result.error << ": '" << text << "'.\n";
    return result.joined();
}

} // namespace apertar::hardware
