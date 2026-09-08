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
            config.servo_firmware_max_pulse_us != 2125 ||
            config.servo_unload_pulse_us != std::vector<int>({1600, 2000, 1600, 2000}) ||
            config.servo_unload_duration_ms != std::vector<int>({0, 1000, 0, 1000})) {
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
        if (config.minimum_moving_linear_mps != .10 ||
            config.minimum_moving_left_mps != 0 ||
            config.minimum_moving_yaw_radps != 0) {
            std::cerr << "unexpected minimum motion commands\n";
            return 1;
        }
        if (config.fence_hsv_h_min != 96 || config.fence_hsv_h_max != 121 ||
            config.fence_hsv_s_min != 128 || config.fence_hsv_v_min != 82) {
            std::cerr << "unexpected blue-fence HSV profile\n";
            return 1;
        }
        if (config.visual_yaw_reset_max_error_deg != 15.0 ||
            config.visual_axis_certainty_min != .05 ||
            config.visual_axis_max_correction_m != 1.0 ||
            config.visual_axis_max_correction_deg != 30.0 ||
            config.home_landmark_x_m != .10 || config.home_landmark_y_m != .15 ||
            config.home_landmark_sigma_m != .20 ||
            config.home_landmark_maximum_error_m != 1.20) {
            std::cerr << "unexpected visual correction limits\n";
            return 1;
        }
        if (config.approach_translation_kp != 4. || config.approach_lateral_kp != 2.3 ||
            config.approach_yaw_kp != 1.3 || config.approach_alignment_yaw_kp != 1. ||
            config.approach_alignment_yaw_kd != 2. ||
            config.approach_alignment_settle_ms != 0 ||
            config.approach_alignment_enter_yaw_deg != 45 ||
            config.approach_alignment_exit_yaw_deg != 60 ||
            config.approach_target_forward_m != .20 ||
            config.approach_target_left_m != -.03 || config.approach_target_tolerance_m != .04 ||
            config.approach_maximum_linear_mps != .8 ||
            config.approach_capture_finish_distance_m != .30 ||
            config.approach_target_timeout_ms != 3000 ||
            config.approach_target_measurement_gain != .60 ||
            config.approach_target_confirmation_ms != 0 ||
            config.approach_target_confirmation_gap_ms != 150 ||
            config.approach_target_minimum_observations != 1 ||
            config.approach_collection_suppression_ms != 8000) {
            std::cerr << "unexpected approach controller configuration\n";
            return 1;
        }
        if (config.search_local_rotate_seconds != 10.0 ||
            config.search_center_rotate_seconds != 10.0 ||
            config.search_target_reset_seconds != 2.0 ||
            config.search_center_x_m != 1.5 || config.search_center_y_m != .9925 ||
            config.search_center_entry_radius_m != .25 ||
            config.search_center_exit_radius_m != .35 ||
            config.search_home_x_m != .25 || config.search_home_y_m != .35 ||
            config.search_home_stop_radius_m != .05 ||
            config.search_rotation_speed_radps != 1.80 ||
            config.search_go_to_pos_translation_kp != 2. ||
            config.search_go_to_pos_translation_ki != 0. ||
            config.search_go_to_pos_translation_kd != 1.3 ||
            config.search_go_to_pos_yaw_kp != 2.0 ||
            config.search_go_to_pos_yaw_ki != 0. ||
            config.search_go_to_pos_yaw_kd != 1. ||
            config.search_maximum_yaw_radps != 2.0 ||
            config.search_post_home_moonwalk_yaw_deg != -120. ||
            config.search_post_home_moonwalk_yaw_tolerance_deg != 10. ||
            config.search_post_home_turn_yaw_deg != 45. ||
            config.search_post_home_reverse_mps != .20 ||
            config.search_post_home_reverse_seconds != 2. ||
            config.search_cargo_calibration_x_m != 1.0 ||
            config.search_cargo_calibration_y_m != 1.0 ||
            config.search_cargo_calibration_stop_radius_m != .15 ||
            config.search_cargo_calibration_yaw_deg != 0. ||
            config.search_cargo_calibration_steps.size() != 3 ||
            config.search_cargo_calibration_steps[0].forward_mps != -.5 ||
            config.search_cargo_calibration_steps[0].timeout_s != 10. ||
            !config.search_cargo_calibration_steps[0].until_imu_detect ||
            config.search_cargo_calibration_steps[1].forward_mps != .30 ||
            config.search_cargo_calibration_steps[1].timeout_s != 2. ||
            config.search_cargo_calibration_steps[1].until_imu_detect ||
            config.search_cargo_calibration_steps[2].forward_mps != -.5 ||
            config.search_cargo_calibration_steps[2].timeout_s != 2.5 ||
            !config.search_cargo_calibration_steps[2].until_imu_detect ||
            !config.search_cargo_wall_hit_enabled ||
            config.search_cargo_wall_hit_accel_threshold_g != .25 ||
            config.search_cargo_wall_hit_arm_ms != 300 ||
            config.search_cargo_wall_hit_max_imu_age_ms != 150) {
            std::cerr << "unexpected search controller configuration\n";
            return 1;
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
