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
        if (config.telemetry_hz != 30 || config.capture_fps != 30 ||
            config.detector_inference_hz != 30 || config.visual_geometry_hz != 1) {
            std::cerr << "unexpected production sampling rates\n";
            return 1;
        }
        if (!config.debug_broadcast_enabled || config.debug_broadcast_port != 3335) {
            std::cerr << "unexpected debug UDP configuration\n";
            return 1;
        }
        if (config.fence_hsv_h_min != 96 || config.fence_hsv_h_max != 121 ||
            config.fence_hsv_s_min != 128 || config.fence_hsv_v_min != 82) {
            std::cerr << "unexpected blue-fence HSV profile\n";
            return 1;
        }
        if (config.visual_max_correction_m != .04 ||
            config.visual_certain_max_correction_m != .15 ||
            config.visual_min_lower_edge_points != 60 ||
            config.visual_yaw_reset_max_error_deg != 15.0) {
            std::cerr << "unexpected visual correction limits\n";
            return 1;
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
