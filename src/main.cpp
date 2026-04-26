#include "ApertarCoreApp.hpp"

#include <atomic>
#include <csignal>
#include <iostream>
#include <string>

namespace {

std::atomic<apertar::ApertarCoreApp *> g_app{nullptr};

void handleSignal(int)
{
    if (auto *app = g_app.load())
        app->stop();
}

void printUsage(const char *argv0)
{
    std::cout
        << "Usage: " << argv0 << " [options]\n\n"
        << "Options:\n"
        << "  --socket PATH       Control socket path (default: /tmp/apertar-core.sock)\n"
        << "  --media PATH        Recording media root (default: /media/RAW)\n"
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

        std::cerr << "Unknown or incomplete option: " << arg << "\n";
        printUsage(argv[0]);
        return 2;
    }

    apertar::ApertarCoreApp app(options);
    g_app = &app;

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    if (!app.start()) {
        std::cerr << "ApertarCore failed to start.\n";
        return 1;
    }

    std::cout << "ApertarCore running on " << options.socketPath << "\n";
    app.wait();
    g_app = nullptr;

    std::cout << "ApertarCore stopped.\n";
    return 0;
}
