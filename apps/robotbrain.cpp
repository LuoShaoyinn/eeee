// Decision-to-actuator bridge for Cubie replay and supervised field tests.
// Input is one frame per line:
//   LOCALIZATION_OK COLLECTION_SENSOR [CLASS CONFIDENCE CENTER_X BOTTOM_Y]...
// Example: 1 0 yellow .91 .52 .63 other_robot .88 .20 .45
// The process must receive frames at least four times per second while live,
// because robotd deliberately stops a stale twist after 250 ms.

#include <atomic>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "robot/mission.hpp"

namespace {

std::atomic_bool g_running = true;

void handle_signal(int) { g_running = false; }

class RobotdClient {
public:
    explicit RobotdClient(std::string socket_path) : socket_path_(std::move(socket_path)) {}

    void request(const std::string& command) const {
        const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if (fd < 0) throw std::runtime_error("cannot create robotd socket");
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        if (socket_path_.size() >= sizeof(address.sun_path)) {
            close(fd);
            throw std::runtime_error("robotd socket path is too long");
        }
        std::strncpy(address.sun_path, socket_path_.c_str(), sizeof(address.sun_path) - 1);
        if (connect(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
            close(fd);
            throw std::runtime_error("cannot connect to robotd at " + socket_path_);
        }
        const std::string wire = command + "\n";
        if (write(fd, wire.data(), wire.size()) != static_cast<ssize_t>(wire.size())) {
            close(fd);
            throw std::runtime_error("cannot write robot command");
        }
        char reply[256]{};
        const ssize_t count = read(fd, reply, sizeof(reply) - 1);
        close(fd);
        if (count <= 0 || std::string(reply, static_cast<size_t>(count)).rfind("error:", 0) == 0) {
            throw std::runtime_error("robotd rejected: " + command);
        }
    }

    void send(const robot::MissionOutput& output) const {
        if (output.emergency_stop) {
            request("ga25 0");
            request("stop");
            return;
        }
        std::ostringstream twist;
        twist << std::fixed << std::setprecision(3) << "twist " << output.forward_mps << ' '
              << output.left_mps << ' ' << output.yaw_radps;
        request(twist.str());
        request("ga25 " + std::to_string(output.collector_percent));
        if (output.servo_pulse_us) request("s3 pulse " + std::to_string(*output.servo_pulse_us));
    }

    void safe_stop() const {
        try { request("ga25 0"); } catch (const std::exception&) {}
        try { request("stop"); } catch (const std::exception&) {}
    }

private:
    std::string socket_path_;
};

robot::ObjectClass parse_class(const std::string& value) {
    if (value == "yellow") return robot::ObjectClass::yellow;
    if (value == "red") return robot::ObjectClass::red;
    if (value == "other_robot") return robot::ObjectClass::other_robot;
    if (value == "home") return robot::ObjectClass::home;
    throw std::runtime_error("unknown detection class: " + value);
}

robot::MissionInput parse_frame(const std::string& line) {
    std::istringstream input(line);
    int localization = 0;
    int collection = 0;
    if (!(input >> localization >> collection)) throw std::runtime_error("expected LOCALIZATION_OK COLLECTION_SENSOR");
    robot::MissionInput frame;
    frame.localization_valid = localization != 0;
    frame.collection_sensor_triggered = collection != 0;
    std::string object_name;
    while (input >> object_name) {
        robot::Detection detection{.object_class = parse_class(object_name)};
        if (!(input >> detection.confidence >> detection.center_x >> detection.bottom_y)) {
            throw std::runtime_error("incomplete detection: " + object_name);
        }
        frame.detections.push_back(detection);
    }
    return frame;
}

void print_output(const robot::MissionOutput& output) {
    std::cout << "state=" << robot::to_string(output.state) << " twist=" << std::fixed << std::setprecision(3)
              << output.forward_mps << ',' << output.left_mps << ',' << output.yaw_radps
              << " ga25=" << output.collector_percent;
    if (output.servo_pulse_us) std::cout << " s3=" << *output.servo_pulse_us;
    if (output.emergency_stop) std::cout << " EMERGENCY_STOP";
    std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    bool live = false;
    std::string socket = "/tmp/robotd.sock";
    std::optional<int> dump_pulse;
    robot::MissionConfig config;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        const auto value = [&](const char* name) -> const char* {
            if (index + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + name);
            return argv[++index];
        };
        if (argument == "--live") live = true;
        else if (argument == "--socket") socket = value("--socket");
        else if (argument == "--expected-objects") config.expected_collectibles = std::stoi(value("--expected-objects"));
        else if (argument == "--dump-pulse") {
            config.dump_servo_pulse_us = std::stoi(value("--dump-pulse"));
            dump_pulse = config.dump_servo_pulse_us;
        }
        else if (argument == "--help") {
            std::cout << "robotbrain [--live] [--socket PATH] --expected-objects N [--dump-pulse US]\n"
                         "Read YOLO/localization frames from stdin. --live is required to command robotd.\n";
            return 0;
        } else throw std::runtime_error("unknown option: " + argument);
    }
    try {
        robot::MissionController mission(config);
        RobotdClient robotd(socket);
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
        std::string line;
        while (g_running && std::getline(std::cin, line)) {
            if (line.empty() || line.starts_with('#')) continue;
            robot::MissionOutput output = mission.update(parse_frame(line));
            // Dump direction is mechanical configuration.  Do not move the
            // rear servo unless an operator explicitly supplied its pulse.
            if (!dump_pulse) output.servo_pulse_us.reset();
            print_output(output);
            if (live) robotd.send(output);
            if (output.state == robot::MissionState::done || output.state == robot::MissionState::fault) break;
        }
        if (live) robotd.safe_stop();
        return mission.state() == robot::MissionState::fault ? 1 : 0;
    } catch (const std::exception& error) {
        std::cerr << "robotbrain: " << error.what() << '\n';
        return 1;
    }
}
