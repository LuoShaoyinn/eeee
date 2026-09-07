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
    if (target_visible) {
        reset();
        return {};
    }
    if (complete_) return {.command = {}, .phase = SearchPhase::complete, .lost_seconds = 0};
    if (lost_since_ == Timestamp{}) lost_since_ = now;
    const double lost = std::chrono::duration<double>(now - lost_since_).count();
    SearchResult result{.command = {}, .phase = SearchPhase::tracking, .lost_seconds = lost};
    if (lost < config_.local_rotate_seconds) {
        result.phase = SearchPhase::rotate_local;
        result.command.yaw_radps = config_.rotation_speed_radps;
    } else if (lost < config_.local_rotate_seconds + config_.center_search_seconds) {
        if (!navigation_allowed) {
            result.phase = SearchPhase::hold_for_localization;
            result.command.yaw_radps = config_.rotation_speed_radps;
        } else {
            const double center_distance = std::hypot(pose.x_m - config_.center_x_m,
                                                      pose.y_m - config_.center_y_m);
            if (center_reached_ && center_distance > config_.center_exit_radius_m) {
                center_reached_ = false;
            } else if (!center_reached_ && center_distance <= config_.center_entry_radius_m) {
                center_reached_ = true;
            }
            if (!center_reached_) {
            result.phase = SearchPhase::navigate_center;
                result.command = navigate(pose, config_.center_x_m, config_.center_y_m,
                                          config_.center_entry_radius_m, dt_s);
            } else {
                result.phase = SearchPhase::rotate_center;
                result.command.yaw_radps = config_.rotation_speed_radps;
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
    const double yaw_step = 1.2 * std::clamp(dt_s, 0.0, .2);
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

void SearchController::reset() {
    lost_since_ = {};
    previous_command_ = {};
    center_reached_ = false;
    complete_ = false;
}

const char* to_string(SearchPhase phase) {
    switch (phase) {
    case SearchPhase::tracking: return "tracking";
    case SearchPhase::rotate_local: return "rotate_local";
    case SearchPhase::navigate_center: return "navigate_center";
    case SearchPhase::rotate_center: return "rotate_center";
    case SearchPhase::hold_for_localization: return "hold_for_localization";
    case SearchPhase::return_home: return "return_home";
    case SearchPhase::complete: return "complete";
    }
    return "unknown";
}

}  // namespace robot
