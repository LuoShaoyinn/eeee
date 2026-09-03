#include <iostream>
#include <string>

#include "robot/config/runtime_config.hpp"

int main() {
    try {
        const auto config = robot::load_runtime_config(
            std::string(ROBOT_SOURCE_DIR) + "/config/robot.yaml");
        if (config.servo_operational_min_pulse_us != 1600 ||
            config.servo_operational_max_pulse_us != 2000 ||
            config.servo_firmware_min_pulse_us != 1550 ||
            config.servo_firmware_max_pulse_us != 2125) {
            std::cerr << "unexpected servo safety envelope\n";
            return 1;
        }
        if (config.telemetry_hz != 25 || config.capture_fps != 30) {
            std::cerr << "unexpected production sampling rates\n";
            return 1;
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
