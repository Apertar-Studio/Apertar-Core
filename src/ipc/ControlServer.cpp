#include "ipc/ControlServer.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace apertar {
namespace {

bool sendLine(int fd, const std::string &line)
{
    std::string payload = line;
    if (payload.empty() || payload.back() != '\n')
        payload.push_back('\n');

    const char *data = payload.data();
    size_t remaining = payload.size();
    while (remaining > 0) {
        const ssize_t written = ::send(fd, data, remaining, MSG_NOSIGNAL);
        if (written <= 0)
            return false;
        data += written;
        remaining -= static_cast<size_t>(written);
    }

    return true;
}

bool sendLineWithFds(int socketFd, const std::string &line, const std::vector<int> &attachedFds)
{
    std::vector<int> validFds;
    validFds.reserve(attachedFds.size());
    for (int fd : attachedFds) {
        if (fd >= 0)
            validFds.push_back(fd);
    }

    if (validFds.empty())
        return sendLine(socketFd, line);

    std::string payload = line;
    if (payload.empty() || payload.back() != '\n')
        payload.push_back('\n');

    iovec iov{};
    iov.iov_base = payload.data();
    iov.iov_len = payload.size();

    std::vector<char> control(CMSG_SPACE(sizeof(int) * validFds.size()));
    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control.data();
    msg.msg_controllen = control.size();

    cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(int) * validFds.size());
    std::memcpy(CMSG_DATA(cmsg), validFds.data(), sizeof(int) * validFds.size());
    msg.msg_controllen = control.size();

    const ssize_t written = ::sendmsg(socketFd, &msg, MSG_NOSIGNAL | MSG_DONTWAIT);
    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        return true;

    return written == static_cast<ssize_t>(payload.size());
}

} // namespace

ControlServer::~ControlServer()
{
    stop();
}

bool ControlServer::start(const std::string &socketPath, Handler handler)
{
    if (running_)
        return true;

    socketPath_ = socketPath;
    handler_ = std::move(handler);

    listenFd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (listenFd_ < 0) {
        std::cerr << "ApertarCore: socket() failed: " << std::strerror(errno) << "\n";
        return false;
    }

    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (socketPath_.size() >= sizeof(address.sun_path)) {
        std::cerr << "ApertarCore: socket path is too long: " << socketPath_ << "\n";
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    std::strncpy(address.sun_path, socketPath_.c_str(), sizeof(address.sun_path) - 1);
    ::unlink(socketPath_.c_str());

    if (::bind(listenFd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
        std::cerr << "ApertarCore: bind() failed: " << std::strerror(errno) << "\n";
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    if (::listen(listenFd_, 8) < 0) {
        std::cerr << "ApertarCore: listen() failed: " << std::strerror(errno) << "\n";
        ::close(listenFd_);
        listenFd_ = -1;
        ::unlink(socketPath_.c_str());
        return false;
    }

    running_ = true;
    acceptThread_ = std::thread(&ControlServer::acceptLoop, this);
    return true;
}

void ControlServer::stop()
{
    if (!running_.exchange(false))
        return;

    if (listenFd_ >= 0) {
        ::shutdown(listenFd_, SHUT_RDWR);
        ::close(listenFd_);
        listenFd_ = -1;
    }

    if (acceptThread_.joinable())
        acceptThread_.join();

    closeClients();

    if (!socketPath_.empty()) {
        ::unlink(socketPath_.c_str());
        socketPath_.clear();
    }
}

void ControlServer::broadcast(const std::string &line)
{
    for (const auto &client : clientsSnapshot()) {
        std::lock_guard<std::mutex> writeLock(client->writeMutex);
        if (!sendLine(client->fd, line))
            removeClient(client);
    }
}

void ControlServer::broadcastWithFd(const std::string &line, int fd)
{
    broadcastWithFds(line, std::vector<int>{ fd });
}

void ControlServer::broadcastWithFds(const std::string &line, const std::vector<int> &fds)
{
    for (const auto &client : clientsSnapshot()) {
        std::lock_guard<std::mutex> writeLock(client->writeMutex);
        if (!sendLineWithFds(client->fd, line, fds))
            removeClient(client);
    }
}

void ControlServer::acceptLoop()
{
    while (running_) {
        const int clientFd = ::accept(listenFd_, nullptr, nullptr);
        if (clientFd < 0) {
            if (running_)
                std::cerr << "ApertarCore: accept() failed: " << std::strerror(errno) << "\n";
            continue;
        }

        auto client = std::make_shared<ClientConnection>(clientFd);
        {
            std::lock_guard<std::mutex> lock(clientsMutex_);
            clients_.push_back(client);
        }

        std::thread(&ControlServer::clientLoop, this, client).detach();
    }
}

void ControlServer::clientLoop(std::shared_ptr<ClientConnection> client)
{
    std::string line;
    char buffer[512]{};

    while (running_) {
        const ssize_t received = ::recv(client->fd, buffer, sizeof(buffer), 0);
        if (received <= 0)
            break;

        for (ssize_t i = 0; i < received; ++i) {
            const char ch = buffer[i];
            if (ch == '\n') {
                if (!line.empty() && handler_) {
                    const std::string reply = handler_(line);
                    std::lock_guard<std::mutex> writeLock(client->writeMutex);
                    if (!sendLine(client->fd, reply)) {
                        removeClient(client);
                        return;
                    }
                }
                line.clear();
            } else if (ch != '\r') {
                line.push_back(ch);
            }
        }
    }

    removeClient(client);
}

void ControlServer::closeClients()
{
    std::lock_guard<std::mutex> lock(clientsMutex_);
    for (const auto &client : clients_) {
        if (client->fd >= 0) {
            ::shutdown(client->fd, SHUT_RDWR);
            ::close(client->fd);
            client->fd = -1;
        }
    }
    clients_.clear();
}

void ControlServer::removeClient(const std::shared_ptr<ClientConnection> &client)
{
    if (!client)
        return;

    std::lock_guard<std::mutex> lock(clientsMutex_);
    const auto it = std::find(clients_.begin(), clients_.end(), client);
    if (it != clients_.end())
        clients_.erase(it);
    if (client->fd >= 0) {
        ::close(client->fd);
        client->fd = -1;
    }
}

std::vector<std::shared_ptr<ControlServer::ClientConnection>> ControlServer::clientsSnapshot()
{
    std::lock_guard<std::mutex> lock(clientsMutex_);
    return clients_;
}

} // namespace apertar
