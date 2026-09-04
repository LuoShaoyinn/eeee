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

SearchResult SearchController::update(const Pose2& pose, bool target_visible,
                                      Timestamp now, double dt_s) {
    if (complete_) return {.command = {}, .phase = SearchPhase::complete, .lost_seconds = 0};
    if (target_visible) {
        reset();
        return {};
    }
    if (lost_since_ == Timestamp{}) lost_since_ = now;
    const double lost = std::chrono::duration<double>(now - lost_since_).count();
    SearchResult result{.command = {}, .phase = SearchPhase::tracking, .lost_seconds = lost};
    if (lost < 5.0) {
        result.phase = SearchPhase::rotate_local;
        result.command.yaw_radps = .45;
    } else if (lost < 10.0) {
        const double center_distance = std::hypot(pose.x_m - 1.5, pose.y_m - .9925);
        if (center_distance > .25) {
            result.phase = SearchPhase::navigate_center;
            result.command = navigate(pose, 1.5, .9925, .25, dt_s);
        } else {
            result.phase = SearchPhase::rotate_center;
            result.command.yaw_radps = .45;
        }
    } else {
        result.phase = SearchPhase::return_home;
        result.command = navigate(pose, .1, .15, .3, dt_s);
        if (std::hypot(pose.x_m - .1, pose.y_m - .15) <= .3) {
            result = {.command = {}, .phase = SearchPhase::complete, .lost_seconds = lost};
            complete_ = true;
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
    Twist2 command{.forward_mps = .55 * (cosine * dx + sine * dy),
                   .left_mps = .55 * (-sine * dx + cosine * dy),
                   .yaw_radps = 1.0 * wrap(std::atan2(dy, dx) - pose.yaw_rad)};
    const double magnitude = std::hypot(command.forward_mps, command.left_mps);
    if (magnitude > .20) {
        command.forward_mps *= .20 / magnitude;
        command.left_mps *= .20 / magnitude;
    }
    command.yaw_radps = std::clamp(command.yaw_radps, -.6, .6);
    return command;
}

void SearchController::reset() {
    lost_since_ = {};
    previous_command_ = {};
    complete_ = false;
}

const char* to_string(SearchPhase phase) {
    switch (phase) {
    case SearchPhase::tracking: return "tracking";
    case SearchPhase::rotate_local: return "rotate_local";
    case SearchPhase::navigate_center: return "navigate_center";
    case SearchPhase::rotate_center: return "rotate_center";
    case SearchPhase::return_home: return "return_home";
    case SearchPhase::complete: return "complete";
    }
    return "unknown";
}

}  // namespace robot
