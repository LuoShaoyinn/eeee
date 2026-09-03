#include "robot/hardware/esp32_client.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace robot {

void Esp32Client::open(const std::string& device, int baud_rate) {
    std::scoped_lock lock(mutex_);
    port_.open(device, baud_rate);
}

std::string Esp32Client::request(const std::string& command, std::chrono::milliseconds timeout) {
    std::scoped_lock lock(mutex_);
    port_.write_all(command + "\n");
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto now = std::chrono::steady_clock::now();
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        const std::string line = port_.read_line(remaining);
        if (line.rfind("@ ", 0) == 0) return line.substr(2);
    }
    throw std::runtime_error("ESP32 response timeout");
}

std::string Esp32Client::set_twist(double vx_mps, double vy_mps, double yaw_radps) {
    std::ostringstream command;
    command << std::fixed << std::setprecision(3)
            << "twist " << vx_mps << ' ' << vy_mps << ' ' << yaw_radps;
    return request(command.str());
}

std::string Esp32Client::stop() { return request("stop"); }

}  // namespace robot
