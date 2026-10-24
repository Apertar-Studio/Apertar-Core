#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace apertar {

class WebServer {
public:
    struct Request {
        std::string method;
        std::string target;
        std::string path;
        std::string query;
        std::string body;
        std::map<std::string, std::string> headers;
    };

    struct Response {
        int status = 200;
        std::string contentType = "text/plain; charset=utf-8";
        std::string body;
        std::vector<std::pair<std::string, std::string>> headers;
        bool streaming = false;
        std::function<void(int)> streamWriter;
    };

    using Handler = std::function<Response(const Request &)>;

    WebServer() = default;
    ~WebServer();

    WebServer(const WebServer &) = delete;
    WebServer &operator=(const WebServer &) = delete;

    bool start(const std::string &bindAddress, uint16_t port, Handler handler);
    void stop();

private:
    struct ClientConnection {
        explicit ClientConnection(int socketFd) : fd(socketFd) {}
        ~ClientConnection();
        const int fd;
    };
    void acceptLoop();
    void clientLoop(int clientFd);

    std::string bindAddress_;
    uint16_t port_ = 0;
    Handler handler_;
    int listenFd_ = -1;
    std::atomic<bool> running_{false};
    std::thread acceptThread_;
    std::mutex clientsMutex_;
    std::condition_variable clientsDone_;
    std::vector<std::shared_ptr<ClientConnection>> clients_;
};

} // namespace apertar
