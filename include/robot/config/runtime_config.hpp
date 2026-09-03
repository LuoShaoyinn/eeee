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
    double visual_pull_gain = .12;
    double visual_max_correction_m = .04;
    double visual_max_correction_deg = 1.5;
    double visual_certain_pull_gain = .50;
    double visual_certain_max_correction_m = .15;
    double visual_certain_max_correction_deg = 6.0;
    double visual_certain_confidence = .65;
    double visual_precise_residual_m = .010;
    double visual_dominant_residual_m = .030;
    double visual_certain_margin_m = .015;
    int visual_min_lower_edge_points = 60;
    double visual_yaw_reset_max_error_deg = 15.0;

    double arena_length_m = 3;
    double arena_width_m = 1.985;
    double fence_height_m = .254;
    int fence_hsv_h_min = 96;
    int fence_hsv_s_min = 128;
    int fence_hsv_v_min = 82;
    int fence_hsv_h_max = 121;
    int fence_hsv_s_max = 255;
    int fence_hsv_v_max = 255;

    int servo_operational_min_pulse_us = 1600;
    int servo_operational_max_pulse_us = 2000;
    int servo_firmware_min_pulse_us = 1550;
    int servo_firmware_max_pulse_us = 2125;

    double max_linear_mps = .45;
    double max_yaw_radps = 2;
    double max_linear_accel_mps2 = .6;
    double max_yaw_accel_radps2 = 2.5;

    std::string detector_backend = "vip_lite";
    std::string detector_model = "models/official_yolo26n_split_pcq_a733.nb";
    double detector_confidence = .35;
    double detector_nms = .45;

    bool debug_broadcast_enabled = true;
    std::string debug_broadcast_address = "192.168.19.218";
    int debug_broadcast_port = 3335;
};

RuntimeConfig load_runtime_config(const std::string& path);

}  // namespace robot
