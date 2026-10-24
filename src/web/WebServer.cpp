#include "web/WebServer.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string_view>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace apertar {
namespace {

constexpr size_t kMaxRequestSize = 1024 * 1024;

std::string trim(std::string value)
{
    auto notSpace = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(),
                std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(),
                value.end());
    return value;
}

std::string toLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

bool sendAll(int fd, const std::string &payload)
{
    const char *data = payload.data();
    size_t remaining = payload.size();
    while (remaining > 0) {
        const ssize_t written = ::send(fd, data, remaining, MSG_NOSIGNAL);
        if (written < 0 && errno == EINTR)
            continue;
        if (written <= 0)
            return false;
        data += written;
        remaining -= static_cast<size_t>(written);
    }
    return true;
}

const char *reasonPhrase(int status)
{
    switch (status) {
    case 200: return "OK";
    case 204: return "No Content";
    case 400: return "Bad Request";
    case 403: return "Forbidden";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 413: return "Payload Too Large";
    case 500: return "Internal Server Error";
    case 503: return "Service Unavailable";
    default: return "OK";
    }
}

bool parseRequest(const std::string &raw, WebServer::Request *request)
{
    if (!request)
        return false;

    const size_t headerEnd = raw.find("\r\n\r\n");
    if (headerEnd == std::string::npos)
        return false;

    std::istringstream stream(raw.substr(0, headerEnd));
    std::string requestLine;
    if (!std::getline(stream, requestLine))
        return false;
    if (!requestLine.empty() && requestLine.back() == '\r')
        requestLine.pop_back();

    std::istringstream requestLineStream(requestLine);
    if (!(requestLineStream >> request->method >> request->target))
        return false;

    request->headers.clear();
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.empty())
            break;

        const size_t colon = line.find(':');
        if (colon == std::string::npos)
            return false;

        std::string key = toLower(trim(line.substr(0, colon)));
        std::string value = trim(line.substr(colon + 1));
        if (key.empty() || ((key == "content-length" || key == "host" || key == "origin") && request->headers.count(key)))
            return false;
        request->headers[key] = value;
    }

    const size_t queryPos = request->target.find('?');
    request->path = queryPos == std::string::npos
        ? request->target
        : request->target.substr(0, queryPos);
    request->query = queryPos == std::string::npos
        ? std::string()
        : request->target.substr(queryPos + 1);

    const size_t bodyStart = headerEnd + 4;
    request->body = bodyStart <= raw.size() ? raw.substr(bodyStart) : std::string();
    return true;
}

std::string buildHttpResponse(const WebServer::Response &response)
{
    std::ostringstream out;
    out << "HTTP/1.1 " << response.status << ' ' << reasonPhrase(response.status) << "\r\n";
    out << "Content-Type: " << response.contentType << "\r\n";
    out << "Content-Length: " << response.body.size() << "\r\n";
    out << "Connection: close\r\n";
    out << "Cache-Control: no-store\r\n";
    for (const auto &[key, value] : response.headers)
        out << key << ": " << value << "\r\n";
    out << "\r\n";
    out << response.body;
    return out.str();
}

std::string buildStreamingHttpHeader(const WebServer::Response &response)
{
    std::ostringstream out;
    out << "HTTP/1.1 " << response.status << ' ' << reasonPhrase(response.status) << "\r\n";
    out << "Content-Type: " << response.contentType << "\r\n";
    out << "Connection: close\r\n";
    out << "Cache-Control: no-store\r\n";
    out << "Pragma: no-cache\r\n";
    for (const auto &[key, value] : response.headers)
        out << key << ": " << value << "\r\n";
    out << "\r\n";
    return out.str();
}

bool allowedBrowserOrigin(const WebServer::Request &request)
{
    const auto site = request.headers.find("sec-fetch-site");
    if (site != request.headers.end() && toLower(site->second) == "cross-site") return false;
    const auto origin = request.headers.find("origin");
    if (origin == request.headers.end()) return true; // Native clients/CLI have no Origin.
    const auto host = request.headers.find("host");
    if (host == request.headers.end()) return false;
    auto authority = toLower(host->second);
    if (authority.find_first_of("/@\\ \t\r\n?#") != std::string::npos || authority.empty()) return false;
    if (authority.ends_with(":80")) authority.resize(authority.size()-3);
    auto expected = toLower(origin->second);
    if (expected.ends_with(":80")) expected.resize(expected.size()-3);
    return expected == "http://" + authority;
}

} // namespace

WebServer::ClientConnection::~ClientConnection()
{
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
}

WebServer::~WebServer()
{
    stop();
}

bool WebServer::start(const std::string &bindAddress, uint16_t port, Handler handler)
{
    if (running_)
        return true;

    bindAddress_ = bindAddress.empty() ? "0.0.0.0" : bindAddress;
    port_ = port;
    handler_ = std::move(handler);

    listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd_ < 0) {
        std::cerr << "ApertarCore Web: socket() failed: " << std::strerror(errno) << "\n";
        return false;
    }

    const int reuse = 1;
    ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port_);
    if (::inet_pton(AF_INET, bindAddress_.c_str(), &address.sin_addr) != 1) {
        std::cerr << "ApertarCore Web: invalid bind address " << bindAddress_ << "\n";
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    if (::bind(listenFd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
        std::cerr << "ApertarCore Web: bind() failed: " << std::strerror(errno) << "\n";
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    if (::listen(listenFd_, 16) < 0) {
        std::cerr << "ApertarCore Web: listen() failed: " << std::strerror(errno) << "\n";
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    running_ = true;
    acceptThread_ = std::thread(&WebServer::acceptLoop, this);
    return true;
}

void WebServer::stop()
{
    if (!running_.exchange(false))
        return;

    if (listenFd_ >= 0) {
        ::shutdown(listenFd_, SHUT_RDWR);
    }

    if (acceptThread_.joinable())
        acceptThread_.join();
    if (listenFd_ >= 0) {
        ::close(listenFd_);
        listenFd_ = -1;
    }
    std::unique_lock<std::mutex> lock(clientsMutex_);
    for (const auto &client : clients_)
        ::shutdown(client->fd, SHUT_RDWR);
    clientsDone_.wait(lock, [this] { return clients_.empty(); });
}

void WebServer::acceptLoop()
{
    while (running_) {
        const int clientFd = ::accept(listenFd_, nullptr, nullptr);
        if (clientFd < 0) {
            if (running_)
                std::cerr << "ApertarCore Web: accept() failed: " << std::strerror(errno) << "\n";
            continue;
        }

        // A Wi-Fi/hotspot disconnect must not leave a preview sender blocked
        // indefinitely, still counted as a live viewer.
        const timeval timeout{5, 0};
        ::setsockopt(clientFd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        ::setsockopt(clientFd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        auto client = std::make_shared<ClientConnection>(clientFd);
        {
            std::lock_guard<std::mutex> lock(clientsMutex_);
            if (!running_ || clients_.size() >= 32)
                continue;
            clients_.push_back(client);
        }
        try {
            std::thread([this, client] {
                try {
                    clientLoop(client->fd);
                } catch (const std::exception &error) {
                    std::cerr << "ApertarCore Web: client handler failed: " << error.what() << '\n';
                } catch (...) {
                    std::cerr << "ApertarCore Web: client handler failed\n";
                }
                std::lock_guard<std::mutex> lock(clientsMutex_);
                clients_.erase(std::remove(clients_.begin(), clients_.end(), client), clients_.end());
                clientsDone_.notify_all();
            }).detach();
        } catch (...) {
            std::lock_guard<std::mutex> lock(clientsMutex_);
            clients_.erase(std::remove(clients_.begin(), clients_.end(), client), clients_.end());
        }
    }
}

void WebServer::clientLoop(int clientFd)
{
    std::string raw;
    std::array<char, 4096> buffer{};
    size_t expectedBodyBytes = 0;
    bool parsedHeaders = false;
    size_t bodyStart = 0;
    int requestError = 0;
    WebServer::Request request;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);

    while (running_ && raw.size() < kMaxRequestSize) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0)
            break;
        pollfd incoming{clientFd, POLLIN, 0};
        const int polled = ::poll(&incoming, 1, static_cast<int>(remaining));
        if (polled < 0 && errno == EINTR)
            continue;
        if (polled <= 0)
            break;
        const ssize_t received = ::recv(clientFd, buffer.data(), buffer.size(), 0);
        if (received < 0 && errno == EINTR)
            continue;
        if (received <= 0)
            break;

        raw.append(buffer.data(), static_cast<size_t>(received));

        if (!parsedHeaders) {
            const size_t headerEnd = raw.find("\r\n\r\n");
            if (headerEnd == std::string::npos)
                continue;

            parsedHeaders = true;
            bodyStart = headerEnd + 4;
            if (!parseRequest(raw, &request) || request.headers.count("transfer-encoding")) {
                requestError = 400;
                break;
            }
            const auto length = request.headers.find("content-length");
            if (length != request.headers.end()) {
                const auto &value = length->second;
                const auto parsed = std::from_chars(value.data(), value.data() + value.size(), expectedBodyBytes);
                if (value.empty() || parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
                    requestError = 400;
                    break;
                }
            }
            if (bodyStart >= kMaxRequestSize || expectedBodyBytes >= kMaxRequestSize - bodyStart) {
                requestError = 413;
                break;
            }
        }

        if (raw.size() - bodyStart >= expectedBodyBytes) {
            break;
        }
    }

    if (!running_)
        return;

    WebServer::Response response;
    if (requestError == 413 || raw.size() >= kMaxRequestSize) {
        response.status = 413;
        response.body = "{\"ok\":false,\"error\":\"Request too large.\"}";
        response.contentType = "application/json; charset=utf-8";
    } else {
        // Never pass an interrupted config upload to a handler that saves it.
        if (requestError || !parsedHeaders || raw.size() - bodyStart < expectedBodyBytes) {
            response.status = 400;
            response.body = "{\"ok\":false,\"error\":\"Malformed HTTP request.\"}";
            response.contentType = "application/json; charset=utf-8";
        } else if (!allowedBrowserOrigin(request)) {
            response.status = 403;
            response.body = "{\"ok\":false,\"error\":\"Cross-origin camera control is not allowed. Open the camera WebUI directly.\"}";
            response.contentType = "application/json; charset=utf-8";
        } else if (!handler_) {
            response.status = 503;
            response.body = "{\"ok\":false,\"error\":\"Web handler unavailable.\"}";
            response.contentType = "application/json; charset=utf-8";
        } else {
            request.body = raw.substr(bodyStart, expectedBodyBytes);
            response = handler_(request);
        }
    }

    if (response.streaming && response.streamWriter) {
        const std::string header = buildStreamingHttpHeader(response);
        if (sendAll(clientFd, header))
            response.streamWriter(clientFd);
    } else {
        const std::string payload = buildHttpResponse(response);
        sendAll(clientFd, payload);
    }
}

} // namespace apertar
