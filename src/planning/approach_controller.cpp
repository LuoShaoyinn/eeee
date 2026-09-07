#include "robot/planning/approach_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace robot {
namespace {

double wrap_angle(double value) {
    return std::remainder(value, 2.0 * std::numbers::pi);
}

double slew(double requested, double previous, double maximum_delta) {
    return previous + std::clamp(requested - previous, -maximum_delta, maximum_delta);
}

}  // namespace

ApproachController::ApproachController(ApproachControllerConfig config) : config_(config) {}

ApproachResult ApproachController::update(const Pose2& pose, const TrackedObject& target,
                                          Timestamp now, double dt_s) {
    ApproachResult result;
    if (target.last_seen == Timestamp{} || now < target.last_seen ||
        now - target.last_seen > config_.target_timeout || dt_s <= 0 || dt_s > .25) {
        return continue_capture(pose, now, dt_s);
    }

    const double dx = target.x_m - pose.x_m;
    const double dy = target.y_m - pose.y_m;
    result.distance_m = std::hypot(dx, dy);
    result.target_valid = true;
    if (result.distance_m <= config_.stopping_distance_m) {
        // The cylinder may remain visible until it is physically under the
        // collector.  Keep a straight capture-entry advance until detection
        // disappears; only then begin the measured 0.3 m finish pass.
        capture_finish_pending_ = true;
        capture_finish_active_ = false;
        forward_integral_ = left_integral_ = 0;
        previous_forward_error_ = previous_left_error_ = previous_yaw_error_ = 0;
        result.command.forward_mps = slew(config_.capture_finish_speed_mps,
                                          previous_command_.forward_mps,
                                          config_.maximum_linear_accel_mps2 * dt_s);
        previous_command_ = result.command;
        initialized_ = false;
        return result;
    }

    const double cosine = std::cos(pose.yaw_rad);
    const double sine = std::sin(pose.yaw_rad);
    const double forward_error = cosine * dx + sine * dy;
    const double left_error = -sine * dx + cosine * dy;
    const double yaw_error = wrap_angle(std::atan2(dy, dx) - pose.yaw_rad);
    if (!initialized_) {
        previous_forward_error_ = forward_error;
        previous_left_error_ = left_error;
        previous_yaw_error_ = yaw_error;
        initialized_ = true;
    }

    // A reacquired target on the other side of the intake must not inherit
    // accumulated lateral/forward correction from the old target lock.
    if (forward_error * previous_forward_error_ < 0) forward_integral_ = 0;
    if (left_error * previous_left_error_ < 0) left_integral_ = 0;

    forward_integral_ = std::clamp(forward_integral_ + forward_error * dt_s,
                                   -config_.integral_limit_m_s, config_.integral_limit_m_s);
    left_integral_ = std::clamp(left_integral_ + left_error * dt_s,
                                -config_.integral_limit_m_s, config_.integral_limit_m_s);
    const double forward_derivative = (forward_error - previous_forward_error_) / dt_s;
    const double left_derivative = (left_error - previous_left_error_) / dt_s;
    const double yaw_derivative = wrap_angle(yaw_error - previous_yaw_error_) / dt_s;

    Twist2 requested{
        .forward_mps = config_.translation_kp * forward_error +
                       config_.translation_ki * forward_integral_ +
                       config_.translation_kd * forward_derivative,
        .left_mps = config_.translation_kp * left_error +
                    config_.translation_ki * left_integral_ +
                    config_.translation_kd * left_derivative,
        .yaw_radps = config_.yaw_kp * yaw_error + config_.yaw_kd * yaw_derivative,
    };
    const double magnitude = std::hypot(requested.forward_mps, requested.left_mps);
    if (magnitude > config_.maximum_linear_mps) {
        requested.forward_mps *= config_.maximum_linear_mps / magnitude;
        requested.left_mps *= config_.maximum_linear_mps / magnitude;
    }
    requested.yaw_radps = std::clamp(requested.yaw_radps,
                                     -config_.maximum_yaw_radps, config_.maximum_yaw_radps);
    result.command.forward_mps = slew(requested.forward_mps, previous_command_.forward_mps,
                                      config_.maximum_linear_accel_mps2 * dt_s);
    result.command.left_mps = slew(requested.left_mps, previous_command_.left_mps,
                                   config_.maximum_linear_accel_mps2 * dt_s);
    result.command.yaw_radps = slew(requested.yaw_radps, previous_command_.yaw_radps,
                                    config_.maximum_yaw_accel_radps2 * dt_s);
    previous_forward_error_ = forward_error;
    previous_left_error_ = left_error;
    previous_yaw_error_ = yaw_error;
    previous_command_ = result.command;
    return result;
}

ApproachResult ApproachController::continue_capture(const Pose2& pose, Timestamp now, double dt_s) {
    ApproachResult result;
    if (!capture_finish_pending_ || dt_s <= 0 || dt_s > .25) return result;
    if (!capture_finish_active_) {
        capture_finish_active_ = true;
        capture_finish_origin_ = pose;
        capture_finish_started_ = now;
    }
    result.target_valid = true;
    result.distance_m = std::hypot(pose.x_m - capture_finish_origin_.x_m,
                                   pose.y_m - capture_finish_origin_.y_m);
    if (result.distance_m >= config_.capture_finish_distance_m ||
        now - capture_finish_started_ >= config_.capture_finish_timeout) {
        result.target_reached = true;
        reset();
        return result;
    }
    result.command.forward_mps = slew(config_.capture_finish_speed_mps,
                                      previous_command_.forward_mps,
                                      config_.maximum_linear_accel_mps2 * dt_s);
    previous_command_ = result.command;
    return result;
}

void ApproachController::reset() {
    forward_integral_ = 0;
    left_integral_ = 0;
    previous_forward_error_ = 0;
    previous_left_error_ = 0;
    previous_yaw_error_ = 0;
    previous_command_ = {};
    initialized_ = false;
    capture_finish_pending_ = false;
    capture_finish_active_ = false;
    capture_finish_origin_ = {};
    capture_finish_started_ = {};
}

}  // namespace robot
