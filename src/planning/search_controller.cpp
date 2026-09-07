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
    if (target_visible && !direct_return_home_) {
        reset();
        return {};
    }
    if (complete_) return {.command = {}, .phase = SearchPhase::complete, .lost_seconds = 0};
    // The explicit operator command and the automatic lost-target recovery
    // share the same post-home trajectory once recovery has committed to the
    // home leg.
    if (direct_return_home_ || center_search_complete_) {
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
                result.command = navigate(pose, config_.center_x_m, config_.center_y_m,
                                          config_.center_entry_radius_m, dt_s);
            } else if (post_home_phase_ == PostHomePhase::none) {
                result.phase = SearchPhase::return_home;
                result.command = navigate(pose, config_.home_x_m, config_.home_y_m,
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
                    post_home_turn_target_yaw_rad_ = wrap(pose.yaw_rad + std::numbers::pi);
                    result = {.command = {}, .phase = SearchPhase::post_home_turn,
                              .lost_seconds = 0};
                } else if (now - post_home_phase_started_ >=
                           std::chrono::duration<double>(config_.post_home_moonwalk_timeout_seconds)) {
                    result = {.command = {}, .phase = SearchPhase::complete, .lost_seconds = 0};
                    complete_ = true;
                } else {
                    // Reuse the field-frame position controller, but command
                    // the requested heading instead of facing the waypoint.
                    result.command = navigate(pose, config_.post_home_moonwalk_x_m,
                                              config_.post_home_moonwalk_y_m, 0, dt_s);
                    result.command.forward_mps *=
                        config_.post_home_moonwalk_translation_kp /
                        config_.navigate_translation_kp;
                    result.command.left_mps *=
                        config_.post_home_moonwalk_translation_kp /
                        config_.navigate_translation_kp;
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
                result.command = navigate(pose, config_.center_x_m, config_.center_y_m,
                                          config_.center_entry_radius_m, dt_s);
            } else {
                result.phase = SearchPhase::rotate_center;
                result.command.yaw_radps = config_.rotation_speed_radps;
                if (center_search_started_ == Timestamp{}) center_search_started_ = now;
                if (now - center_search_started_ >=
                    std::chrono::duration<double>(config_.center_search_seconds)) {
                    center_search_complete_ = true;
                    result.phase = SearchPhase::return_home;
                    result.command = navigate(pose, config_.home_x_m, config_.home_y_m,
                                              config_.home_stop_radius_m, dt_s);
                }
            }
        }
    } else {
        if (!navigation_allowed) {
            result.phase = SearchPhase::hold_for_localization;
            result.command = {};
        } else {
            result.phase = SearchPhase::return_home;
            result.command = navigate(pose, config_.home_x_m, config_.home_y_m,
                                      config_.home_stop_radius_m, dt_s);
            if (std::hypot(pose.x_m - config_.home_x_m, pose.y_m - config_.home_y_m) <=
                config_.home_stop_radius_m) {
                result = {.command = {}, .phase = SearchPhase::complete, .lost_seconds = lost};
                complete_ = true;
            }
        }
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

Twist2 SearchController::navigate(const Pose2& pose, double x_m, double y_m,
                                  double stop_radius_m, double) {
    const double dx = x_m - pose.x_m;
    const double dy = y_m - pose.y_m;
    const double distance = std::hypot(dx, dy);
    if (distance <= stop_radius_m) return {};
    const double cosine = std::cos(pose.yaw_rad);
    const double sine = std::sin(pose.yaw_rad);
    Twist2 command{.forward_mps = config_.navigate_translation_kp * (cosine * dx + sine * dy),
                   .left_mps = config_.navigate_translation_kp * (-sine * dx + cosine * dy),
                   .yaw_radps = config_.navigate_yaw_kp * wrap(std::atan2(dy, dx) - pose.yaw_rad)};
    const double magnitude = std::hypot(command.forward_mps, command.left_mps);
    if (magnitude > config_.maximum_linear_mps) {
        command.forward_mps *= config_.maximum_linear_mps / magnitude;
        command.left_mps *= config_.maximum_linear_mps / magnitude;
    }
    command.yaw_radps = std::clamp(command.yaw_radps,
                                    -config_.maximum_yaw_radps, config_.maximum_yaw_radps);
    return command;
}

void SearchController::begin_return_home() {
    reset();
    direct_return_home_ = true;
}

void SearchController::reset() {
    lost_since_ = {};
    center_search_started_ = {};
    previous_command_ = {};
    center_reached_ = false;
    center_search_complete_ = false;
    direct_return_home_ = false;
    complete_ = false;
    post_home_phase_ = PostHomePhase::none;
    post_home_phase_started_ = {};
    post_home_turn_target_yaw_rad_ = 0;
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
    case SearchPhase::complete: return "complete";
    }
    return "unknown";
}

}  // namespace robot
