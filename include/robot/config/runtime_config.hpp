#pragma once

#include <cstddef>
#include <string>

namespace robot {

struct RuntimeConfig {
    std::string socket_path = "/tmp/robotd.sock";
    std::string serial_device = "/dev/ttyAS2";
    int baud_rate = 115200;
    double telemetry_hz = 30;
    int command_timeout_ms = 250;

    std::string camera_device = "/dev/video0";
    std::string camera_calibration = "config/camera_fisheye_1280x720.yaml";
    int capture_width = 1280;
    int capture_height = 720;
    double capture_fps = 30;
    int visual_width = 320;
    int visual_height = 180;
    bool record_enabled = true;
    double record_fps = 10;
    double visual_geometry_hz = 1;

    std::size_t particles = 600;
    double camera_height_m = .1291;
    double camera_pitch_deg = 30.0296;
    double camera_roll_deg = .2071;
    double initial_x_m = .1;
    double initial_y_m = .1;
    double initial_yaw_deg = 0;
    double visual_pull_gain = .12;
    double visual_precise_residual_m = .010;
    double visual_residual_limit_m = .045;
    double visual_yaw_reset_max_error_deg = 15.0;
    double visual_axis_certainty_min = .05;
    double visual_axis_sigma_m = .15;
    double visual_yaw_sigma_deg = 5.0;
    double visual_axis_max_correction_m = 1.0;
    double visual_axis_max_correction_deg = 30.0;
    double visual_axis_max_pull_gain = .80;
    // The home detector observes the interior of the home rectangle, while
    // navigation docks at its outer field-facing corner.
    double home_landmark_x_m = .10;
    double home_landmark_y_m = .15;
    double home_landmark_sigma_m = .20;
    double home_landmark_maximum_error_m = 1.20;

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
    double minimum_moving_linear_mps = .10;
    double minimum_moving_left_mps = 0;
    double minimum_moving_yaw_radps = .10;

    double approach_translation_kp = .75;
    double approach_translation_ki = .04;
    double approach_translation_kd = .06;
    double approach_lateral_kp = .75;
    double approach_lateral_ki = .04;
    double approach_lateral_kd = .06;
    double approach_yaw_kp = 1.4;
    double approach_yaw_kd = .08;
    double approach_alignment_yaw_kp = 1.4;
    double approach_alignment_yaw_kd = .08;
    double approach_maximum_linear_mps = .28;
    double approach_maximum_yaw_radps = .8;
    double approach_maximum_linear_accel_mps2 = .5;
    double approach_maximum_yaw_accel_radps2 = 1.5;
    double approach_target_forward_m = .20;
    double approach_target_left_m = .01;
    double approach_target_tolerance_m = .06;
    double approach_alignment_enter_yaw_deg = 8;
    double approach_alignment_exit_yaw_deg = 15;
    double approach_forward_command_deadband_mps = .04;
    double approach_left_command_deadband_mps = .04;
    double approach_yaw_command_deadband_radps = .05;
    double approach_capture_finish_distance_m = .30;
    double approach_capture_finish_speed_mps = .22;
    int approach_capture_finish_timeout_ms = 3000;
    int approach_alignment_settle_ms = 700;
    int approach_target_timeout_ms = 3000;
    double approach_target_measurement_gain = .60;

    double search_local_rotate_seconds = 5;
    double search_center_rotate_seconds = 5;
    double search_center_x_m = 1.5;
    double search_center_y_m = .9925;
    double search_center_entry_radius_m = .25;
    double search_center_exit_radius_m = .35;
    double search_home_x_m = .20;
    double search_home_y_m = .30;
    double search_home_stop_radius_m = .3;
    double search_rotation_speed_radps = .35;
    double search_maximum_linear_mps = .20;
    double search_maximum_yaw_radps = .6;
    double search_post_home_moonwalk_x_m = .55;
    double search_post_home_moonwalk_y_m = .50;
    double search_post_home_moonwalk_translation_kp = .55;
    double search_post_home_moonwalk_yaw_deg = -145;
    double search_post_home_moonwalk_yaw_tolerance_deg = 10;
    double search_post_home_moonwalk_yaw_kp = 1.5;
    double search_post_home_moonwalk_timeout_seconds = 10;
    double search_post_home_turn_yaw_kp = 1.5;
    double search_post_home_turn_yaw_tolerance_deg = 5;
    double search_post_home_reverse_mps = .30;
    double search_post_home_reverse_seconds = 1;

    std::string detector_backend = "vip_lite";
    std::string detector_model = "models/official_yolo26n_640x384_rgbfix_rebuild_a733.nb";
    double detector_inference_hz = 30;
    double detector_confidence = .35;
    double detector_nms = .45;

    bool debug_broadcast_enabled = true;
    std::string debug_broadcast_address = "192.168.19.218";
    int debug_broadcast_port = 3335;
};

RuntimeConfig load_runtime_config(const std::string& path);

}  // namespace robot
