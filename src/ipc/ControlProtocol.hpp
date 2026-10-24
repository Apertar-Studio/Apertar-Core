#pragma once

#include <map>
#include <optional>
#include <string>

namespace apertar {

struct ControlCommand {
    int id = -1;
    std::string name;
    std::map<std::string, std::string> fields;

    std::optional<double> number(const std::string &key) const;
    std::optional<std::string> string(const std::string &key) const;
};

struct ParseResult {
    bool ok = false;
    ControlCommand command;
    std::string error;
};

ParseResult parseControlCommand(const std::string &line);
std::string escapeJson(const std::string &text);

} // namespace apertar
