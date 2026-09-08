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

Twist2 constrain_twist(Twist2 command, const ApproachControllerConfig& config) {
    const double linear_fraction =
        std::hypot(command.forward_mps, command.left_mps) / config.maximum_linear_mps;
    const double yaw_fraction = std::abs(command.yaw_radps) / config.maximum_yaw_radps;
    // Translation and yaw share the available traction budget. A hard turn
    // deliberately reduces translation rather than asking mecanum wheels to
    // deliver an infeasible combination.
    const double demand = std::hypot(linear_fraction, yaw_fraction);
    if (demand > 1.0) {
        command.forward_mps /= demand;
        command.left_mps /= demand;
        command.yaw_radps /= demand;
    }
    return command;
}

}  // namespace

ApproachController::ApproachController(ApproachControllerConfig config) : config_(config) {}

ApproachResult ApproachController::update(const Pose2& pose, const TrackedObject& target,
                                          Timestamp now, double dt_s) {
    if (capture_finish_active_) return continue_capture(pose, now, dt_s);
    ApproachResult result;
    if (target.last_seen == Timestamp{} || now < target.last_seen ||
        now - target.last_seen > config_.target_timeout || dt_s <= 0 || dt_s > .25) {
        return continue_capture(pose, now, dt_s);
    }

    const double dx = target.x_m - pose.x_m;
    const double dy = target.y_m - pose.y_m;
    const double cosine = std::cos(pose.yaw_rad);
    const double sine = std::sin(pose.yaw_rad);
    const double forward_error = cosine * dx + sine * dy;
    const double left_error = -sine * dx + cosine * dy - config_.target_left_m;
    const double forward_control_error = forward_error - config_.target_forward_m;
    result.distance_m = std::hypot(forward_control_error, left_error);
    result.target_valid = true;
    // The intake's lateral deadband also defines when bearing correction is
    // useful. Without this, a close object produces a large bearing from a
    // harmless lateral residual and can consume the shared traction budget.
    const bool lateral_in_yaw_deadband =
        std::abs(left_error) <= config_.left_command_deadband_mps;
    const double yaw_error = lateral_in_yaw_deadband ? 0 : std::atan2(left_error, forward_error);
    // Capture only once the target has crossed the intake plane while staying
    // laterally inside the collector. Do not use radial proximity here: it can
    // start a dash before a close off-axis target has actually reached intake.
    const bool capture_gate =
        forward_error > 0 && forward_error < config_.target_forward_m &&
        std::abs(left_error) <= config_.target_tolerance_m;
    result.capture_eligible = capture_gate;

    // Once the object is inside the intake gate, go straight into its dash.
    // A tracked target may still have a substantial bearing near the camera;
    // requiring it to finish a rotate-only phase here made capture wait for a
    // full yaw stop before the intake could advance.
    if (capture_gate) {
        capture_finish_pending_ = true;
        capture_finish_active_ = true;
        capture_finish_origin_ = pose;
        capture_finish_started_ = now;
        forward_integral_ = left_integral_ = 0;
        previous_forward_error_ = previous_left_error_ = previous_yaw_error_ = 0;
        initialized_ = false;
        alignment_active_ = false;
        alignment_settle_required_ = false;
        capture_alignment_settling_ = false;
        capture_alignment_settled_at_ = {};
        return continue_capture(pose, now, dt_s);
    }

    if (!initialized_) {
        previous_forward_error_ = forward_control_error;
        previous_left_error_ = left_error;
        previous_yaw_error_ = yaw_error;
        // A newly acquired target must not immediately mix a search rotation
        // with lateral translation. Align first when its bearing is material.
        alignment_active_ = std::abs(yaw_error) > config_.alignment_enter_yaw_rad;
        initialized_ = true;
    } else if (alignment_active_ && !capture_alignment_settling_ &&
               std::abs(yaw_error) <= config_.alignment_enter_yaw_rad) {
        alignment_active_ = false;
        alignment_settle_required_ =
            std::abs(previous_command_.yaw_radps) > config_.yaw_command_deadband_radps;
        forward_integral_ = left_integral_ = 0;
    } else if (!alignment_active_ &&
               std::abs(yaw_error) >= config_.alignment_exit_yaw_rad) {
        alignment_active_ = true;
        forward_integral_ = left_integral_ = 0;
    }

    if (alignment_active_) {
        const double yaw_derivative = lateral_in_yaw_deadband
            ? 0 : wrap_angle(yaw_error - previous_yaw_error_) / dt_s;
        double requested_yaw = lateral_in_yaw_deadband || (capture_alignment_settling_ &&
                std::abs(yaw_error) <= config_.alignment_enter_yaw_rad
            ) ? 0 : config_.alignment_yaw_kp * yaw_error +
                       config_.alignment_yaw_kd * yaw_derivative;
        requested_yaw = std::clamp(requested_yaw,
                                   -config_.maximum_yaw_radps, config_.maximum_yaw_radps);
        if (std::abs(requested_yaw) < config_.yaw_command_deadband_radps) requested_yaw = 0;
        result.aligning = true;
        // Decelerate any previous translation first; this avoids commanding a
        // sudden direction reversal when a target moves outside hysteresis.
        result.command.forward_mps = slew(0, previous_command_.forward_mps,
                                          config_.maximum_linear_accel_mps2 * dt_s);
        result.command.left_mps = slew(0, previous_command_.left_mps,
                                       config_.maximum_linear_accel_mps2 * dt_s);
        result.command.yaw_radps = slew(requested_yaw, previous_command_.yaw_radps,
                                        config_.maximum_yaw_accel_radps2 * dt_s);
        previous_forward_error_ = forward_control_error;
        previous_left_error_ = left_error;
        previous_yaw_error_ = yaw_error;
        previous_command_ = result.command;
        if (capture_alignment_settling_ &&
            std::abs(yaw_error) <= config_.alignment_enter_yaw_rad &&
            std::abs(result.command.yaw_radps) <= config_.yaw_command_deadband_radps) {
            if (capture_alignment_settled_at_ == Timestamp{}) {
                capture_alignment_settled_at_ = now;
            }
            if (now - capture_alignment_settled_at_ < config_.alignment_settle) {
                return result;
            }
            alignment_active_ = false;
            alignment_settle_required_ = false;
            capture_alignment_settling_ = false;
            capture_alignment_settled_at_ = {};
        } else {
            capture_alignment_settled_at_ = {};
            return result;
        }
    }

    // A reacquired target on the other side of the intake must not inherit
    // accumulated lateral/forward correction from the old target lock.
    if (forward_control_error * previous_forward_error_ < 0) forward_integral_ = 0;
    if (left_error * previous_left_error_ < 0) left_integral_ = 0;

    forward_integral_ = std::clamp(forward_integral_ + forward_control_error * dt_s,
                                   -config_.integral_limit_m_s, config_.integral_limit_m_s);
    left_integral_ = std::clamp(left_integral_ + left_error * dt_s,
                                -config_.integral_limit_m_s, config_.integral_limit_m_s);
    const double forward_derivative = (forward_control_error - previous_forward_error_) / dt_s;
    const double left_derivative = (left_error - previous_left_error_) / dt_s;
    const double yaw_derivative = lateral_in_yaw_deadband
        ? 0 : wrap_angle(yaw_error - previous_yaw_error_) / dt_s;

    Twist2 requested{
        .forward_mps = config_.translation_kp * forward_control_error +
                       config_.translation_ki * forward_integral_ +
                       config_.translation_kd * forward_derivative,
        .left_mps = config_.lateral_kp * left_error +
                    config_.lateral_ki * left_integral_ +
                    config_.lateral_kd * left_derivative,
        .yaw_radps = lateral_in_yaw_deadband
            ? 0 : config_.yaw_kp * yaw_error + config_.yaw_kd * yaw_derivative,
    };
    requested = constrain_twist(requested, config_);
    if (std::abs(requested.forward_mps) < config_.forward_command_deadband_mps) {
        requested.forward_mps = 0;
    }
    if (std::abs(requested.left_mps) < config_.left_command_deadband_mps) {
        requested.left_mps = 0;
    }
    if (std::abs(requested.yaw_radps) < config_.yaw_command_deadband_radps) {
        requested.yaw_radps = 0;
    }
    result.command.forward_mps = slew(requested.forward_mps, previous_command_.forward_mps,
                                      config_.maximum_linear_accel_mps2 * dt_s);
    result.command.left_mps = slew(requested.left_mps, previous_command_.left_mps,
                                   config_.maximum_linear_accel_mps2 * dt_s);
    result.command.yaw_radps = slew(requested.yaw_radps, previous_command_.yaw_radps,
                                    config_.maximum_yaw_accel_radps2 * dt_s);
    result.command = constrain_twist(result.command, config_);
    previous_forward_error_ = forward_control_error;
    previous_left_error_ = left_error;
    previous_yaw_error_ = yaw_error;
    previous_command_ = result.command;
    if (!alignment_active_ && !capture_alignment_settling_ &&
        std::abs(previous_command_.yaw_radps) <= config_.yaw_command_deadband_radps) {
        alignment_settle_required_ = false;
    }
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
    result.capturing = true;
    result.distance_m = std::hypot(pose.x_m - capture_finish_origin_.x_m,
                                   pose.y_m - capture_finish_origin_.y_m);
    if (result.distance_m >= config_.capture_finish_distance_m ||
        now - capture_finish_started_ >= config_.capture_finish_timeout) {
        result.target_reached = true;
        result.capturing = false;
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
    alignment_active_ = false;
    alignment_settle_required_ = false;
    capture_alignment_settling_ = false;
    capture_alignment_settled_at_ = {};
    capture_finish_pending_ = false;
    capture_finish_active_ = false;
    capture_finish_origin_ = {};
    capture_finish_started_ = {};
}

}  // namespace robot
