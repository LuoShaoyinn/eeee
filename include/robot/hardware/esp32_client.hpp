#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "robot/hardware/serial_port.hpp"

namespace robot {

class Esp32Client {
public:
    void open(const std::string& device = "/dev/ttyAS2", int baud_rate = 115200);
    std::string request(const std::string& command,
                        std::chrono::milliseconds timeout = std::chrono::milliseconds(500));
    std::string set_twist(double vx_mps, double vy_mps, double yaw_radps);
    std::string stop();

private:
    SerialPort port_;
    std::mutex mutex_;
};

}  // namespace robot
