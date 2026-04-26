#include "ipc/ControlProtocol.hpp"

#include <cctype>
#include <cstdlib>
#include <stdexcept>

namespace apertar {
namespace {

void skipSpace(const std::string &text, size_t &pos)
{
    while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos])))
        ++pos;
}

bool consume(const std::string &text, size_t &pos, char expected)
{
    skipSpace(text, pos);
    if (pos >= text.size() || text[pos] != expected)
        return false;
    ++pos;
    return true;
}

std::optional<std::string> parseStringLiteral(const std::string &text, size_t &pos)
{
    skipSpace(text, pos);
    if (pos >= text.size() || text[pos] != '"')
        return std::nullopt;
    ++pos;

    std::string value;
    while (pos < text.size()) {
        const char ch = text[pos++];
        if (ch == '"')
            return value;

        if (ch == '\\') {
            if (pos >= text.size())
                return std::nullopt;

            const char escaped = text[pos++];
            switch (escaped) {
            case '"': value.push_back('"'); break;
            case '\\': value.push_back('\\'); break;
            case '/': value.push_back('/'); break;
            case 'b': value.push_back('\b'); break;
            case 'f': value.push_back('\f'); break;
            case 'n': value.push_back('\n'); break;
            case 'r': value.push_back('\r'); break;
            case 't': value.push_back('\t'); break;
            default: return std::nullopt;
            }
        } else {
            value.push_back(ch);
        }
    }

    return std::nullopt;
}

std::optional<std::string> parseBareValue(const std::string &text, size_t &pos)
{
    skipSpace(text, pos);
    const size_t start = pos;
    while (pos < text.size() && text[pos] != ',' && text[pos] != '}')
        ++pos;

    if (pos == start)
        return std::nullopt;

    size_t end = pos;
    while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1])))
        --end;

    return text.substr(start, end - start);
}

} // namespace

std::optional<double> ControlCommand::number(const std::string &key) const
{
    const auto it = fields.find(key);
    if (it == fields.end())
        return std::nullopt;

    try {
        size_t parsed = 0;
        const double value = std::stod(it->second, &parsed);
        if (parsed != it->second.size())
            return std::nullopt;
        return value;
    } catch (const std::exception &) {
        return std::nullopt;
    }
}

std::optional<std::string> ControlCommand::string(const std::string &key) const
{
    const auto it = fields.find(key);
    if (it == fields.end())
        return std::nullopt;
    return it->second;
}

ParseResult parseControlCommand(const std::string &line)
{
    ParseResult result;
    size_t pos = 0;

    if (!consume(line, pos, '{')) {
        result.error = "Expected JSON object.";
        return result;
    }

    while (true) {
        skipSpace(line, pos);
        if (pos < line.size() && line[pos] == '}') {
            ++pos;
            break;
        }

        auto key = parseStringLiteral(line, pos);
        if (!key) {
            result.error = "Expected object key.";
            return result;
        }

        if (!consume(line, pos, ':')) {
            result.error = "Expected ':' after object key.";
            return result;
        }

        std::optional<std::string> value;
        skipSpace(line, pos);
        if (pos < line.size() && line[pos] == '"')
            value = parseStringLiteral(line, pos);
        else
            value = parseBareValue(line, pos);

        if (!value) {
            result.error = "Expected value for key '" + *key + "'.";
            return result;
        }

        result.command.fields[*key] = *value;

        skipSpace(line, pos);
        if (pos < line.size() && line[pos] == ',') {
            ++pos;
            continue;
        }
        if (pos < line.size() && line[pos] == '}') {
            ++pos;
            break;
        }

        result.error = "Expected ',' or '}'.";
        return result;
    }

    if (auto cmd = result.command.string("cmd"))
        result.command.name = *cmd;

    if (result.command.name.empty()) {
        result.error = "Missing command name.";
        return result;
    }

    if (const auto id = result.command.number("id"))
        result.command.id = static_cast<int>(*id);

    result.ok = true;
    return result;
}

std::string escapeJson(const std::string &text)
{
    std::string escaped;
    escaped.reserve(text.size());
    for (const char ch : text) {
        switch (ch) {
        case '"': escaped += "\\\""; break;
        case '\\': escaped += "\\\\"; break;
        case '\b': escaped += "\\b"; break;
        case '\f': escaped += "\\f"; break;
        case '\n': escaped += "\\n"; break;
        case '\r': escaped += "\\r"; break;
        case '\t': escaped += "\\t"; break;
        default: escaped.push_back(ch); break;
        }
    }
    return escaped;
}

} // namespace apertar
