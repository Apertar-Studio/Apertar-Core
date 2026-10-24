#include "ApertarCoreApp.hpp"

#include <csignal>
#include <cerrno>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

namespace {

volatile sig_atomic_t shutdownSignalFd = -1;

void handleSignal(int)
{
    const int savedErrno = errno;
    const int fd = shutdownSignalFd;
    if (fd >= 0) {
        const char wake = 1;
        const auto ignored = ::write(fd, &wake, 1);
        (void)ignored;
    }
    errno = savedErrno;
}

void printUsage(const char *argv0)
{
    std::cout
        << "Usage: " << argv0 << " [options]\n\n"
        << "Options:\n"
        << "  --socket PATH       Control socket path (default: /tmp/apertar-core.sock)\n"
        << "  --media PATH        Recording media root (default: /media/RAW)\n"
        << "  --web-bind ADDR     Web control bind address (default: 0.0.0.0)\n"
        << "  --web-port PORT     Web control port, 0 disables it (default: 8000)\n"
        << "  --simulate          Run without opening the camera yet\n"
        << "  --help              Show this help\n";
}

} // namespace

int main(int argc, char **argv)
{
    apertar::ApertarCoreApp::Options options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        if (arg == "--simulate") {
            options.simulateCamera = true;
            continue;
        }
        if (arg == "--socket" && i + 1 < argc) {
            options.socketPath = argv[++i];
            continue;
        }
        if (arg == "--media" && i + 1 < argc) {
            options.mediaRoot = argv[++i];
            continue;
        }
        if (arg == "--web-bind" && i + 1 < argc) {
            options.webBindAddress = argv[++i];
            continue;
        }
        if (arg == "--web-port" && i + 1 < argc) {
            const std::string value = argv[++i];
            try {
                const unsigned long parsed = std::stoul(value);
                if (parsed > 65535) {
                    std::cerr << "Invalid web port: " << value << "\n";
                    return 2;
                }
                options.webPort = static_cast<uint16_t>(parsed);
            } catch (...) {
                std::cerr << "Invalid web port: " << value << "\n";
                return 2;
            }
            continue;
        }

        std::cerr << "Unknown or incomplete option: " << arg << "\n";
        printUsage(argv[0]);
        return 2;
    }

    // Camera shutdown takes locks and joins threads: never do it inside a
    // signal handler, which may interrupt a thread already holding those locks.
    int signalPipe[2];
    if (::pipe2(signalPipe, O_CLOEXEC) != 0 ||
        ::fcntl(signalPipe[1], F_SETFL, O_NONBLOCK) < 0) {
        std::cerr << "ApertarCore could not configure shutdown signals.\n";
        return 1;
    }

    shutdownSignalFd = signalPipe[1];
    struct sigaction action{};
    action.sa_handler = handleSignal;
    ::sigemptyset(&action.sa_mask);
    action.sa_flags = SA_RESTART;
    ::sigaction(SIGINT, &action, nullptr);
    ::sigaction(SIGTERM, &action, nullptr);

    apertar::ApertarCoreApp app(options);

    if (!app.start()) {
        std::cerr << "ApertarCore failed to start.\n";
        return 1;
    }

    std::thread signalThread([&] {
        char wake;
        ssize_t result;
        do {
            result = ::read(signalPipe[0], &wake, 1);
        } while (result < 0 && errno == EINTR);
        app.requestShutdown();
    });

    std::cout << "ApertarCore running on " << options.socketPath << "\n";
    if (options.webPort != 0) {
        std::cout << "ApertarCore web control available on http://"
                  << options.webBindAddress
                  << ":" << options.webPort
                  << "\n";
    }
    app.wait();
    const char wake = 1;
    const auto ignored = ::write(signalPipe[1], &wake, 1);
    (void)ignored;
    signalThread.join();
    app.stop();
    shutdownSignalFd = -1;
    ::close(signalPipe[0]);
    ::close(signalPipe[1]);

    std::cout << "ApertarCore stopped.\n";
    return 0;
}
