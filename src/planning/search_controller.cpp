#include "robot/planning/search_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace robot {
namespace {
double wrap(double angle) { return std::remainder(angle, 2.0 * std::numbers::pi); }
double slew(double value, double previous, double limit) {
    return previous + std::clamp(value - previous, -limit, limit);
}
}  // namespace

SearchController::SearchController(SearchConfig config) : config_(config) {}

SearchResult SearchController::update(const Pose2& pose, bool target_visible,
                                      Timestamp now, double dt_s, bool navigation_allowed) {
    if (target_visible && !direct_return_home_ &&
        cargo_calibration_phase_ == CargoCalibrationPhase::none) {
        if (target_visible_since_ == Timestamp{}) target_visible_since_ = now;
        if (now - target_visible_since_ >=
            std::chrono::duration<double>(config_.target_reset_seconds)) {
            reset();
        }
        return {};
    }
    target_visible_since_ = {};
    if (complete_) return {.command = {}, .phase = SearchPhase::complete, .lost_seconds = 0};
    if (cargo_calibration_phase_ != CargoCalibrationPhase::none) {
        SearchResult result;
        if (!navigation_allowed) {
            result.phase = SearchPhase::hold_for_localization;
        } else {
            switch (cargo_calibration_phase_) {
            case CargoCalibrationPhase::navigate:
                result.phase = SearchPhase::navigate_cargo_calibration;
                result.command = go_to_position(pose, config_.cargo_calibration_x_m,
                                                config_.cargo_calibration_y_m,
                                                config_.cargo_calibration_stop_radius_m, dt_s);
                if (std::hypot(pose.x_m - config_.cargo_calibration_x_m,
                               pose.y_m - config_.cargo_calibration_y_m) <=
                    config_.cargo_calibration_stop_radius_m) {
                    cargo_calibration_phase_ = CargoCalibrationPhase::align;
                    reset_go_to_pos_pid();
                    result.command = {};
                }
                break;
            case CargoCalibrationPhase::align: {
                result.phase = SearchPhase::cargo_calibration_align;
                const double yaw_error = wrap(config_.cargo_calibration_yaw_deg *
                                                  std::numbers::pi / 180.0 - pose.yaw_rad);
                if (std::abs(yaw_error) <= config_.cargo_calibration_yaw_tolerance_deg *
                                               std::numbers::pi / 180.0) {
                    cargo_calibration_phase_ = CargoCalibrationPhase::motion;
                    cargo_calibration_step_ = 0;
                    cargo_calibration_phase_started_ = now;
                } else {
                    result.command.yaw_radps = std::clamp(
                        config_.cargo_calibration_yaw_kp * yaw_error,
                        -config_.maximum_yaw_radps, config_.maximum_yaw_radps);
                }
                break;
            }
            case CargoCalibrationPhase::motion: {
                const CargoCalibrationStep& step =
                    config_.cargo_calibration_steps[cargo_calibration_step_];
                result.phase = cargo_calibration_step_ == 0
                    ? SearchPhase::cargo_calibration_reverse_fast
                    : cargo_calibration_step_ == 1
                        ? SearchPhase::cargo_calibration_forward_slow
                        : cargo_calibration_step_ == 2
                            ? SearchPhase::cargo_calibration_reverse_final
                            : SearchPhase::cargo_calibration_motion;
                result.command.forward_mps = step.forward_mps;
                if (now - cargo_calibration_phase_started_ >=
                    std::chrono::duration<double>(step.timeout_s)) {
                    ++cargo_calibration_step_;
                    cargo_calibration_phase_started_ = now;
                    if (cargo_calibration_step_ == config_.cargo_calibration_steps.size()) {
                        cargo_calibration_phase_ = CargoCalibrationPhase::return_center;
                        reset_go_to_pos_pid();
                        result.command = {};
                    }
                }
                break;
            }
            case CargoCalibrationPhase::return_center:
                result.phase = SearchPhase::navigate_center_after_cargo;
                result.command = go_to_position(pose, config_.center_x_m, config_.center_y_m,
                                                config_.center_entry_radius_m, dt_s);
                if (std::hypot(pose.x_m - config_.center_x_m,
                               pose.y_m - config_.center_y_m) <= config_.center_entry_radius_m) {
                    cargo_calibration_phase_ = CargoCalibrationPhase::none;
                    direct_return_home_ = true;
                    center_reached_ = true;
                    reset_go_to_pos_pid();
                    result.command = {};
                }
                break;
            case CargoCalibrationPhase::none:
                break;
            }
        }
        const double linear_step = config_.cargo_calibration_linear_accel_mps2 *
                                   std::clamp(dt_s, 0.0, .2);
        const double yaw_step = 2.5 * std::clamp(dt_s, 0.0, .2);
        result.command.forward_mps = slew(result.command.forward_mps, previous_command_.forward_mps,
                                          linear_step);
        result.command.left_mps = slew(result.command.left_mps, previous_command_.left_mps,
                                       linear_step);
        result.command.yaw_radps = slew(result.command.yaw_radps, previous_command_.yaw_radps,
                                        yaw_step);
        previous_command_ = result.command;
        return result;
    }
    // The post-home trajectory runs after cargo calibration is complete.
    if (direct_return_home_) {
        SearchResult result;
        if (!navigation_allowed) {
            result.phase = SearchPhase::hold_for_localization;
        } else {
            if (!center_reached_ &&
                std::hypot(pose.x_m - config_.center_x_m, pose.y_m - config_.center_y_m) <=
                    config_.center_entry_radius_m) {
                center_reached_ = true;
            }
            if (!center_reached_) {
                result.phase = SearchPhase::navigate_center;
                result.command = go_to_position(pose, config_.center_x_m, config_.center_y_m,
                                                config_.center_entry_radius_m, dt_s);
            } else if (post_home_phase_ == PostHomePhase::none) {
                result.phase = SearchPhase::return_home;
                result.command = go_to_position(pose, config_.home_x_m, config_.home_y_m,
                                                config_.home_stop_radius_m, dt_s);
                if (std::hypot(pose.x_m - config_.home_x_m, pose.y_m - config_.home_y_m) <=
                    config_.home_stop_radius_m) {
                    post_home_phase_ = PostHomePhase::moonwalk;
                    post_home_phase_started_ = now;
                    result = {.command = {}, .phase = SearchPhase::post_home_moonwalk,
                              .lost_seconds = 0};
                }
            } else if (post_home_phase_ == PostHomePhase::moonwalk) {
                result.phase = SearchPhase::post_home_moonwalk;
                const double target_yaw = config_.post_home_moonwalk_yaw_deg *
                                          std::numbers::pi / 180.0;
                const double yaw_error = wrap(target_yaw - pose.yaw_rad);
                if (std::abs(yaw_error) <= config_.post_home_moonwalk_yaw_tolerance_deg *
                                               std::numbers::pi / 180.0) {
                    post_home_phase_ = PostHomePhase::turn;
                    post_home_phase_started_ = now;
                    post_home_turn_target_yaw_rad_ = wrap(
                        pose.yaw_rad + config_.post_home_turn_degrees * std::numbers::pi / 180.0);
                    result = {.command = {}, .phase = SearchPhase::post_home_turn,
                              .lost_seconds = 0};
                } else if (now - post_home_phase_started_ >=
                           std::chrono::duration<double>(config_.post_home_moonwalk_timeout_seconds)) {
                    result = {.command = {}, .phase = SearchPhase::complete, .lost_seconds = 0};
                    complete_ = true;
                } else {
                    result.command.yaw_radps = std::clamp(
                        config_.post_home_moonwalk_yaw_kp * yaw_error,
                        -config_.maximum_yaw_radps, config_.maximum_yaw_radps);
                }
            } else if (post_home_phase_ == PostHomePhase::turn) {
                result.phase = SearchPhase::post_home_turn;
                const double yaw_error = wrap(post_home_turn_target_yaw_rad_ - pose.yaw_rad);
                if (std::abs(yaw_error) <= config_.post_home_turn_yaw_tolerance_deg *
                                               std::numbers::pi / 180.0) {
                    post_home_phase_ = PostHomePhase::reverse;
                    post_home_phase_started_ = now;
                    result = {.command = {}, .phase = SearchPhase::post_home_reverse,
                              .lost_seconds = 0};
                } else {
                    result.command.yaw_radps = std::clamp(
                        config_.post_home_turn_yaw_kp * yaw_error,
                        -config_.maximum_yaw_radps, config_.maximum_yaw_radps);
                }
            } else {
                result.phase = SearchPhase::post_home_reverse;
                if (now - post_home_phase_started_ >=
                    std::chrono::duration<double>(config_.post_home_reverse_seconds)) {
                    result = {.command = {}, .phase = SearchPhase::complete, .lost_seconds = 0};
                    complete_ = true;
                } else {
                    result.command.forward_mps = -config_.post_home_reverse_mps;
                }
            }
        }
        const double linear_step = .4 * std::clamp(dt_s, 0.0, .2);
        const double yaw_step = 2.5 * std::clamp(dt_s, 0.0, .2);
        result.command.forward_mps = slew(result.command.forward_mps, previous_command_.forward_mps,
                                          linear_step);
        result.command.left_mps = slew(result.command.left_mps, previous_command_.left_mps,
                                       linear_step);
        result.command.yaw_radps = slew(result.command.yaw_radps, previous_command_.yaw_radps,
                                        yaw_step);
        previous_command_ = result.command;
        return result;
    }
    if (lost_since_ == Timestamp{}) lost_since_ = now;
    const double lost = std::chrono::duration<double>(now - lost_since_).count();
    SearchResult result{.command = {}, .phase = SearchPhase::tracking, .lost_seconds = lost};
    if (lost < config_.local_rotate_seconds) {
        result.phase = SearchPhase::rotate_local;
        result.command.yaw_radps = config_.rotation_speed_radps;
    } else if (!center_search_complete_) {
        if (!navigation_allowed) {
            result.phase = SearchPhase::hold_for_localization;
            result.command.yaw_radps = config_.rotation_speed_radps;
        } else {
            const double center_distance = std::hypot(pose.x_m - config_.center_x_m,
                                                      pose.y_m - config_.center_y_m);
            if (center_reached_ && center_distance > config_.center_exit_radius_m) {
                center_reached_ = false;
                center_search_started_ = {};
            } else if (!center_reached_ && center_distance <= config_.center_entry_radius_m) {
                center_reached_ = true;
                center_search_started_ = now;
            }
            if (!center_reached_) {
                result.phase = SearchPhase::navigate_center;
                result.command = go_to_position(pose, config_.center_x_m, config_.center_y_m,
                                                config_.center_entry_radius_m, dt_s);
            } else {
                result.phase = SearchPhase::rotate_center;
                result.command.yaw_radps = config_.rotation_speed_radps;
                if (center_search_started_ == Timestamp{}) center_search_started_ = now;
                if (now - center_search_started_ >=
                    std::chrono::duration<double>(config_.center_search_seconds)) {
                    center_search_complete_ = true;
                    cargo_calibration_phase_ = CargoCalibrationPhase::navigate;
                    result.phase = SearchPhase::navigate_cargo_calibration;
                    result.command = {};
                }
            }
        }
    } else {
        cargo_calibration_phase_ = CargoCalibrationPhase::navigate;
        result.phase = SearchPhase::navigate_cargo_calibration;
    }
    const double linear_step = .4 * std::clamp(dt_s, 0.0, .2);
    // Search needs to acquire targets quickly; retain a bounded ramp so a
    // transition from translation to rotation cannot step the wheel command.
    const double yaw_step = 2.5 * std::clamp(dt_s, 0.0, .2);
    result.command.forward_mps = slew(result.command.forward_mps, previous_command_.forward_mps,
                                      linear_step);
    result.command.left_mps = slew(result.command.left_mps, previous_command_.left_mps,
                                   linear_step);
    result.command.yaw_radps = slew(result.command.yaw_radps, previous_command_.yaw_radps,
                                    yaw_step);
    previous_command_ = result.command;
    return result;
}

Twist2 SearchController::go_to_position(const Pose2& pose, double x_m, double y_m,
                                        double stop_radius_m, double dt_s) {
    const double dx = x_m - pose.x_m;
    const double dy = y_m - pose.y_m;
    const double distance = std::hypot(dx, dy);
    if (distance <= stop_radius_m) {
        reset_go_to_pos_pid();
        return {};
    }
    if (!go_to_pos_goal_valid_ || std::hypot(x_m - go_to_pos_goal_x_m_,
                                             y_m - go_to_pos_goal_y_m_) > 1e-6) {
        reset_go_to_pos_pid();
        go_to_pos_goal_valid_ = true;
        go_to_pos_goal_x_m_ = x_m;
        go_to_pos_goal_y_m_ = y_m;
    }
    const double cosine = std::cos(pose.yaw_rad);
    const double sine = std::sin(pose.yaw_rad);
    const double forward_error = cosine * dx + sine * dy;
    const double left_error = -sine * dx + cosine * dy;
    const double yaw_error = wrap(std::atan2(dy, dx) - pose.yaw_rad);
    const double bounded_dt = std::clamp(dt_s, 0.0, .2);
    go_to_pos_forward_integral_ = std::clamp(go_to_pos_forward_integral_ + forward_error * bounded_dt,
                                              -1.0, 1.0);
    go_to_pos_left_integral_ = std::clamp(go_to_pos_left_integral_ + left_error * bounded_dt,
                                           -1.0, 1.0);
    go_to_pos_yaw_integral_ = std::clamp(go_to_pos_yaw_integral_ + yaw_error * bounded_dt,
                                          -1.0, 1.0);
    const double inverse_dt = bounded_dt > 1e-6 ? 1.0 / bounded_dt : 0.0;
    const double forward_derivative = go_to_pos_previous_error_valid_ ?
        (forward_error - go_to_pos_previous_forward_error_) * inverse_dt : 0;
    const double left_derivative = go_to_pos_previous_error_valid_ ?
        (left_error - go_to_pos_previous_left_error_) * inverse_dt : 0;
    const double yaw_derivative = go_to_pos_previous_error_valid_ ?
        wrap(yaw_error - go_to_pos_previous_yaw_error_) * inverse_dt : 0;
    go_to_pos_previous_forward_error_ = forward_error;
    go_to_pos_previous_left_error_ = left_error;
    go_to_pos_previous_yaw_error_ = yaw_error;
    go_to_pos_previous_error_valid_ = true;
    Twist2 command{
        .forward_mps = config_.go_to_pos_translation_kp * forward_error +
            config_.go_to_pos_translation_ki * go_to_pos_forward_integral_ +
            config_.go_to_pos_translation_kd * forward_derivative,
        .left_mps = config_.go_to_pos_translation_kp * left_error +
            config_.go_to_pos_translation_ki * go_to_pos_left_integral_ +
            config_.go_to_pos_translation_kd * left_derivative,
        .yaw_radps = config_.go_to_pos_yaw_kp * yaw_error +
            config_.go_to_pos_yaw_ki * go_to_pos_yaw_integral_ +
            config_.go_to_pos_yaw_kd * yaw_derivative,
    };
    const double magnitude = std::hypot(command.forward_mps, command.left_mps);
    if (magnitude > config_.maximum_linear_mps) {
        command.forward_mps *= config_.maximum_linear_mps / magnitude;
        command.left_mps *= config_.maximum_linear_mps / magnitude;
    }
    command.yaw_radps = std::clamp(command.yaw_radps,
                                    -config_.maximum_yaw_radps, config_.maximum_yaw_radps);
    return command;
}

void SearchController::reset_go_to_pos_pid() {
    go_to_pos_goal_valid_ = false;
    go_to_pos_forward_integral_ = 0;
    go_to_pos_left_integral_ = 0;
    go_to_pos_yaw_integral_ = 0;
    go_to_pos_previous_error_valid_ = false;
}

void SearchController::begin_return_home() {
    reset();
    direct_return_home_ = true;
    cargo_calibration_phase_ = CargoCalibrationPhase::navigate;
}

bool SearchController::report_cargo_wall_hit(Timestamp now) {
    if (cargo_calibration_phase_ != CargoCalibrationPhase::motion ||
        cargo_calibration_step_ >= config_.cargo_calibration_steps.size() ||
        !config_.cargo_calibration_steps[cargo_calibration_step_].until_imu_detect) return false;
    ++cargo_calibration_step_;
    cargo_calibration_phase_started_ = now;
    if (cargo_calibration_step_ == config_.cargo_calibration_steps.size()) {
        cargo_calibration_phase_ = CargoCalibrationPhase::return_center;
        reset_go_to_pos_pid();
    }
    return true;
}

bool SearchController::cargo_wall_contact_armed() const {
    return cargo_calibration_phase_ == CargoCalibrationPhase::motion &&
           cargo_calibration_step_ < config_.cargo_calibration_steps.size() &&
           config_.cargo_calibration_steps[cargo_calibration_step_].until_imu_detect;
}

void SearchController::reset() {
    lost_since_ = {};
    target_visible_since_ = {};
    center_search_started_ = {};
    previous_command_ = {};
    center_reached_ = false;
    center_search_complete_ = false;
    direct_return_home_ = false;
    complete_ = false;
    post_home_phase_ = PostHomePhase::none;
    post_home_phase_started_ = {};
    post_home_turn_target_yaw_rad_ = 0;
    cargo_calibration_phase_ = CargoCalibrationPhase::none;
    cargo_calibration_phase_started_ = {};
    cargo_calibration_step_ = 0;
    reset_go_to_pos_pid();
}

const char* to_string(SearchPhase phase) {
    switch (phase) {
    case SearchPhase::tracking: return "tracking";
    case SearchPhase::rotate_local: return "rotate_local";
    case SearchPhase::navigate_center: return "navigate_center";
    case SearchPhase::rotate_center: return "rotate_center";
    case SearchPhase::hold_for_localization: return "hold_for_localization";
    case SearchPhase::return_home: return "return_home";
    case SearchPhase::post_home_moonwalk: return "post_home_moonwalk";
    case SearchPhase::post_home_turn: return "post_home_turn";
    case SearchPhase::post_home_reverse: return "post_home_reverse";
    case SearchPhase::navigate_cargo_calibration: return "navigate_cargo_calibration";
    case SearchPhase::cargo_calibration_align: return "cargo_calibration_align";
    case SearchPhase::cargo_calibration_reverse_fast: return "cargo_calibration_reverse_fast";
    case SearchPhase::cargo_calibration_forward_slow: return "cargo_calibration_forward_slow";
    case SearchPhase::cargo_calibration_reverse_final: return "cargo_calibration_reverse_final";
    case SearchPhase::cargo_calibration_motion: return "cargo_calibration_motion";
    case SearchPhase::navigate_center_after_cargo: return "navigate_center_after_cargo";
    case SearchPhase::complete: return "complete";
    }
    return "unknown";
}

}  // namespace robot
