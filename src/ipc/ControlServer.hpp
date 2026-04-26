#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace apertar {

class ControlServer {
public:
    using Handler = std::function<std::string(const std::string &)>;

    ControlServer() = default;
    ~ControlServer();

    ControlServer(const ControlServer &) = delete;
    ControlServer &operator=(const ControlServer &) = delete;

    bool start(const std::string &socketPath, Handler handler);
    void stop();
    void broadcast(const std::string &line);
    void broadcastWithFd(const std::string &line, int fd);
    void broadcastWithFds(const std::string &line, const std::vector<int> &fds);

private:
    struct ClientConnection {
        explicit ClientConnection(int socketFd) : fd(socketFd) {}
        int fd = -1;
        std::mutex writeMutex;
    };

    void acceptLoop();
    void clientLoop(std::shared_ptr<ClientConnection> client);
    void closeClients();
    void removeClient(const std::shared_ptr<ClientConnection> &client);
    std::vector<std::shared_ptr<ClientConnection>> clientsSnapshot();

    std::string socketPath_;
    Handler handler_;
    int listenFd_ = -1;
    std::atomic<bool> running_{false};
    std::thread acceptThread_;

    std::mutex clientsMutex_;
    std::vector<std::shared_ptr<ClientConnection>> clients_;
};

} // namespace apertar
