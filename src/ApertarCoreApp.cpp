#include "ApertarCoreApp.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <vector>

namespace apertar {
namespace {

std::string okReply(int id, const std::string &body = {})
{
    std::ostringstream out;
    out << "{\"ok\":true";
    if (id >= 0)
        out << ",\"id\":" << id;
    if (!body.empty())
        out << "," << body;
    out << "}";
    return out.str();
}

std::string errorReply(int id, const std::string &message)
{
    std::ostringstream out;
    out << "{\"ok\":false";
    if (id >= 0)
        out << ",\"id\":" << id;
    out << ",\"error\":\"" << escapeJson(message) << "\"}";
    return out.str();
}

std::string settingsBody(const CaptureSettings &settings)
{
    std::ostringstream out;
    out << "\"resolution\":\"" << settings.resolution.toString() << "\""
        << ",\"width\":" << settings.resolution.width
        << ",\"height\":" << settings.resolution.height
        << ",\"fps\":" << std::fixed << std::setprecision(3) << settings.fps
        << ",\"iso\":" << settings.iso
        << ",\"iso_auto\":" << (settings.isoAuto ? "true" : "false")
        << ",\"shutter_us\":" << settings.shutterUs
        << ",\"shutter_auto\":" << (settings.shutterAuto ? "true" : "false")
        << ",\"wb_kelvin\":" << settings.whiteBalanceKelvin;
    return out.str();
}

} // namespace

ApertarCoreApp::ApertarCoreApp(Options options)
    : options_(std::move(options))
    , camera_(
          [this](const std::string &eventJson) { publishEvent(eventJson); },
          [this](const PreviewFrameDescriptor &frame) { publishPreviewFrame(frame); })
{
}

ApertarCoreApp::~ApertarCoreApp()
{
    stop();
}

bool ApertarCoreApp::start()
{
    if (running_.exchange(true))
        return true;

    if (!camera_.start(options_.mediaRoot, options_.simulateCamera)) {
        running_ = false;
        return false;
    }

    if (!controlServer_.start(options_.socketPath, [this](const std::string &line) {
            return handleRequest(line);
        })) {
        camera_.stop();
        running_ = false;
        return false;
    }

    publishEvent("{\"event\":\"camera_state\",\"state\":\"ready\"}");
    return true;
}

void ApertarCoreApp::stop()
{
    if (!running_.exchange(false))
        return;

    publishEvent("{\"event\":\"camera_state\",\"state\":\"stopping\"}");
    controlServer_.stop();
    camera_.stop();
    waitCv_.notify_all();
}

void ApertarCoreApp::wait()
{
    std::unique_lock<std::mutex> lock(waitMutex_);
    waitCv_.wait(lock, [this]() { return !running_.load(); });
}

std::string ApertarCoreApp::handleRequest(const std::string &line)
{
    const ParseResult parsed = parseControlCommand(line);
    if (!parsed.ok)
        return errorReply(-1, parsed.error);

    {
        std::lock_guard<std::mutex> lock(eventMutex_);
        collectingCommandEvents_ = true;
        pendingCommandEvents_.clear();
    }

    std::string reply = handleCommand(parsed.command);

    std::vector<std::string> events;
    {
        std::lock_guard<std::mutex> lock(eventMutex_);
        events = std::move(pendingCommandEvents_);
        pendingCommandEvents_.clear();
        collectingCommandEvents_ = false;
    }

    for (const std::string &event : events) {
        reply.push_back('\n');
        reply += event;
    }

    return reply;
}

std::string ApertarCoreApp::handleCommand(const ControlCommand &command)
{
    if (command.name == "ping")
        return okReply(command.id, "\"pong\":true");

    if (command.name == "help")
        return okReply(command.id, "\"commands\":[\"get_state\",\"set_iso\",\"set_iso_auto\",\"set_fps\",\"set_shutter_us\",\"set_shutter_auto\",\"set_wb\",\"set_resolution\",\"capture_photo\",\"record_start\",\"record_stop\",\"shutdown\"]");

    if (command.name == "get_state") {
        std::ostringstream body;
        body << "\"state\":\"" << toString(camera_.state()) << "\""
             << ",\"recording\":" << (camera_.isRecording() ? "true" : "false")
             << "," << settingsBody(camera_.settings());
        return okReply(command.id, body.str());
    }

    if (command.name == "set_iso") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_iso requires value.");
        return camera_.setIso(static_cast<uint32_t>(*value))
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_iso_auto") {
        return camera_.setIsoAuto()
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_fps") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_fps requires value.");
        return camera_.setFps(*value)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_shutter_us") {
        const auto value = command.number("value");
        if (!value)
            return errorReply(command.id, "set_shutter_us requires value.");
        return camera_.setShutterUs(static_cast<uint32_t>(*value))
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_shutter_auto") {
        return camera_.setShutterAuto()
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_wb") {
        const auto kelvin = command.number("kelvin");
        if (!kelvin)
            return errorReply(command.id, "set_wb requires kelvin.");
        return camera_.setWhiteBalanceKelvin(static_cast<uint32_t>(*kelvin))
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "set_resolution") {
        const auto width = command.number("width");
        const auto height = command.number("height");
        if (!width || !height)
            return errorReply(command.id, "set_resolution requires width and height.");

        Resolution resolution{static_cast<uint32_t>(*width), static_cast<uint32_t>(*height)};
        return camera_.setResolution(resolution)
            ? okReply(command.id, settingsBody(camera_.settings()))
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "record_start") {
        return camera_.startRecording()
            ? okReply(command.id, "\"recording\":true")
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "capture_photo") {
        return camera_.capturePhoto()
            ? okReply(command.id, "\"photo_capture\":true")
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "record_stop") {
        return camera_.stopRecording()
            ? okReply(command.id, "\"recording\":false")
            : errorReply(command.id, camera_.lastError());
    }

    if (command.name == "shutdown") {
        requestShutdown();
        return okReply(command.id, "\"shutting_down\":true");
    }

    return errorReply(command.id, "Unknown command: " + command.name);
}

void ApertarCoreApp::publishEvent(const std::string &eventJson)
{
    {
        std::lock_guard<std::mutex> lock(eventMutex_);
        if (collectingCommandEvents_) {
            pendingCommandEvents_.push_back(eventJson);
            return;
        }
    }

    controlServer_.broadcast(eventJson);
}

void ApertarCoreApp::publishPreviewFrame(const PreviewFrameDescriptor &frame)
{
    if (frame.fd < 0)
        return;

    std::ostringstream out;
    out << "{\"event\":\"preview_frame\""
        << ",\"frame_id\":" << frame.frameId
        << ",\"width\":" << frame.width
        << ",\"height\":" << frame.height
        << ",\"stride\":" << frame.stride
        << ",\"capture_width\":" << frame.captureWidth
        << ",\"capture_height\":" << frame.captureHeight
        << ",\"size\":" << frame.size
        << ",\"fourcc\":\"" << escapeJson(frame.fourcc) << "\""
        << ",\"timestamp_us\":" << frame.timestampUs;

    const int planeCount = std::max(0, std::min(frame.planeCount, 3));
    if (planeCount > 0) {
        out << ",\"plane_count\":1"
            << ",\"layout_plane_count\":" << planeCount;
        for (int i = 0; i < planeCount; ++i) {
            out << ",\"plane" << i << "_offset\":" << frame.planeOffsets[i]
                << ",\"plane" << i << "_pitch\":" << frame.planePitches[i]
                << ",\"plane" << i << "_length\":" << frame.planeLengths[i];
        }
    } else {
        out << ",\"plane_count\":1";
    }

    out << ",\"fd_attached\":true"
        << "}";

    controlServer_.broadcastWithFd(out.str(), frame.fd);
}

void ApertarCoreApp::requestShutdown()
{
    running_ = false;
    waitCv_.notify_all();
}

} // namespace apertar
