#pragma once

#include <cstddef>
#include <string>

namespace robot {

struct RuntimeConfig {
    std::string socket_path = "/tmp/robotd.sock";
    std::string serial_device = "/dev/ttyAS2";
    int baud_rate = 115200;
    double telemetry_hz = 25;
    int command_timeout_ms = 250;

    std::string camera_device = "/dev/video0";
    std::string camera_calibration = "config/camera_fisheye_1280x720.yaml";
    int capture_width = 1280;
    int capture_height = 720;
    double capture_fps = 30;
    int visual_width = 320;
    int visual_height = 180;
    double record_fps = 10;

    std::size_t particles = 600;
    double camera_height_m = .1291;
    double camera_pitch_deg = 30.0296;
    double camera_roll_deg = .2071;
    double initial_x_m = .1;
    double initial_y_m = .1;
    double initial_yaw_deg = 0;

    double arena_length_m = 3;
    double arena_width_m = 1.985;
    double fence_height_m = .254;

    int servo_operational_min_pulse_us = 1600;
    int servo_operational_max_pulse_us = 2000;
    int servo_firmware_min_pulse_us = 1550;
    int servo_firmware_max_pulse_us = 2125;

    double max_linear_mps = .45;
    double max_yaw_radps = 2;
    double max_linear_accel_mps2 = .6;
    double max_yaw_accel_radps2 = 2.5;

    std::string detector_backend = "npu";
    std::string detector_model = "models/yolo26n.rknn";
    double detector_confidence = .35;
    double detector_nms = .45;
};

RuntimeConfig load_runtime_config(const std::string& path);

}  // namespace robot
