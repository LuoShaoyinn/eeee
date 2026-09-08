#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "robot/hardware/esp32_client.hpp"

namespace {

constexpr std::chrono::milliseconds kHeartbeatPeriod{40};
constexpr std::chrono::milliseconds kClientCommandTimeout{250};
std::atomic_bool g_running = true;

void handle_signal(int) { g_running = false; }

int make_server(const std::string& socket_path) {
    const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) throw std::runtime_error("cannot create Unix socket");
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (socket_path.size() >= sizeof(address.sun_path)) throw std::runtime_error("socket path too long");
    std::strncpy(address.sun_path, socket_path.c_str(), sizeof(address.sun_path) - 1);
    unlink(socket_path.c_str());
    if (bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0 || listen(fd, 4) != 0) {
        close(fd);
        throw std::runtime_error("cannot bind Unix socket");
    }
    return fd;
}

std::string read_command(int client) {
    char buffer[256]{};
    const ssize_t count = read(client, buffer, sizeof(buffer) - 1);
    if (count <= 0) return {};
    std::string command(buffer, static_cast<size_t>(count));
    const size_t newline = command.find_first_of("\r\n");
    if (newline != std::string::npos) command.erase(newline);
    return command;
}

bool parse_twist(const std::string& command, double& vx, double& vy, double& wz) {
    return std::sscanf(command.c_str(), "twist %lf %lf %lf", &vx, &vy, &wz) == 3;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string serial_device = argc > 1 ? argv[1] : "/dev/ttyAS2";
    const std::string socket_path = argc > 2 ? argv[2] : "/tmp/robotd.sock";
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    try {
        robot::Esp32Client esp;
        esp.open(serial_device);
        const int server = make_server(socket_path);
        std::cerr << "robotd: UART " << serial_device << ", socket " << socket_path << '\n';
        bool twist_active = false;
        double vx = 0, vy = 0, wz = 0;
        auto last_twist = std::chrono::steady_clock::now();

        while (g_running) {
            fd_set readable;
            FD_ZERO(&readable);
            FD_SET(server, &readable);
            timeval wait{.tv_sec = 0, .tv_usec = static_cast<suseconds_t>(kHeartbeatPeriod.count() * 1000)};
            const int ready = select(server + 1, &readable, nullptr, nullptr, &wait);
            if (ready > 0 && FD_ISSET(server, &readable)) {
                const int client = accept4(server, nullptr, nullptr, SOCK_CLOEXEC);
                if (client >= 0) {
                    std::string reply;
                    try {
                        const std::string command = read_command(client);
                        double requested_vx, requested_vy, requested_wz;
                        if (parse_twist(command, requested_vx, requested_vy, requested_wz)) {
                            vx = requested_vx; vy = requested_vy; wz = requested_wz;
                            twist_active = true;
                            last_twist = std::chrono::steady_clock::now();
                            reply = esp.set_twist(vx, vy, wz);
                        } else if (command == "stop") {
                            twist_active = false;
                            reply = esp.stop();
                        } else if (!command.empty()) {
                            reply = esp.request(command);
                        } else {
                            reply = "error: empty command";
                        }
                    } catch (const std::exception& error) {
                        reply = std::string("error: ") + error.what();
                    }
                    reply += '\n';
                    (void)write(client, reply.data(), reply.size());
                    close(client);
                }
            }
            if (!twist_active) continue;
            const auto now = std::chrono::steady_clock::now();
            try {
                if (now - last_twist > kClientCommandTimeout) {
                    esp.stop();
                    twist_active = false;
                } else {
                    esp.set_twist(vx, vy, wz);
                }
            } catch (const std::exception& error) {
                std::cerr << "robotd UART error: " << error.what() << '\n';
                twist_active = false;
            }
        }
        try { esp.stop(); } catch (const std::exception&) {}
        close(server);
        unlink(socket_path.c_str());
    } catch (const std::exception& error) {
        std::cerr << "robotd: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
