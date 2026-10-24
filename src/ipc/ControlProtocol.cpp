#include "ipc/ControlProtocol.hpp"

#include <cctype>
#include <cstdint>
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

std::optional<uint32_t> parseHexQuad(const std::string &text, size_t &pos)
{
    if (text.size() - pos < 4)
        return std::nullopt;
    uint32_t value = 0;
    for (int i = 0; i < 4; ++i) {
        const char ch = text[pos++];
        unsigned digit;
        if (ch >= '0' && ch <= '9') digit = ch - '0';
        else if (ch >= 'a' && ch <= 'f') digit = ch - 'a' + 10;
        else if (ch >= 'A' && ch <= 'F') digit = ch - 'A' + 10;
        else return std::nullopt;
        value = (value << 4) | digit;
    }
    return value;
}

void appendUtf8(std::string &value, uint32_t codepoint)
{
    if (codepoint <= 0x7f) {
        value.push_back(static_cast<char>(codepoint));
    } else if (codepoint <= 0x7ff) {
        value.push_back(static_cast<char>(0xc0 | (codepoint >> 6)));
        value.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    } else if (codepoint <= 0xffff) {
        value.push_back(static_cast<char>(0xe0 | (codepoint >> 12)));
        value.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
        value.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    } else {
        value.push_back(static_cast<char>(0xf0 | (codepoint >> 18)));
        value.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3f)));
        value.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
        value.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    }
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
            case 'u': {
                auto codepoint = parseHexQuad(text, pos);
                if (!codepoint)
                    return std::nullopt;
                // JSON represents supplementary characters as UTF-16 surrogate pairs.
                if (*codepoint >= 0xd800 && *codepoint <= 0xdbff) {
                    if (text.size() - pos < 6 || text[pos] != '\\' || text[pos + 1] != 'u')
                        return std::nullopt;
                    pos += 2;
                    const auto low = parseHexQuad(text, pos);
                    if (!low || *low < 0xdc00 || *low > 0xdfff)
                        return std::nullopt;
                    *codepoint = 0x10000 + ((*codepoint - 0xd800) << 10) + (*low - 0xdc00);
                } else if (*codepoint >= 0xdc00 && *codepoint <= 0xdfff) {
                    return std::nullopt;
                }
                appendUtf8(value, *codepoint);
                break;
            }
            default: return std::nullopt;
            }
        } else {
            if (static_cast<unsigned char>(ch) < 0x20)
                return std::nullopt;
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
