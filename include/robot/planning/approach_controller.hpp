#pragma once

#include <chrono>

#include "robot/core/types.hpp"
#include "robot/planning/world_model.hpp"

namespace robot {

struct ApproachControllerConfig {
    double translation_kp = .75;
    double translation_ki = .04;
    double translation_kd = .06;
    double lateral_kp = .75;
    double lateral_ki = .04;
    double lateral_kd = .06;
    // Normal yaw tracking while translating toward the target.
    double yaw_kp = 1.4;
    double yaw_kd = .08;
    // Gains used only while rotating to put the target on the intake axis.
    double alignment_yaw_kp = 1.4;
    double alignment_yaw_kd = .08;
    double maximum_linear_mps = .28;
    double maximum_yaw_radps = .8;
    double maximum_linear_accel_mps2 = .5;
    double maximum_yaw_accel_radps2 = 1.5;
    // Desired object contact position in robot coordinates before the intake
    // dash: forward, then left.
    double target_forward_m = .20;
    double target_left_m = .01;
    double target_tolerance_m = .06;
    // Align before translating after acquiring a target. Hysteresis prevents
    // alternating between alignment and approach near the threshold.
    double alignment_enter_yaw_rad = .1396263402;  // 8 degrees
    double alignment_exit_yaw_rad = .2617993878;   // 15 degrees
    // Suppress detector-scale output noise before drivetrain minimum-command
    // handling. Values are controller outputs, not target-position errors.
    double forward_command_deadband_mps = .04;
    double left_command_deadband_mps = .04;
    double yaw_command_deadband_radps = .05;
    double capture_finish_distance_m = .30;
    double capture_finish_speed_mps = .22;
    std::chrono::milliseconds capture_finish_timeout{3000};
    // ESP32 reverses a wheel safely: it brakes first, then changes direction.
    // Keep all wheel targets at zero after yaw alignment before an intake dash.
    std::chrono::milliseconds alignment_settle{700};
    double integral_limit_m_s = .25;
    std::chrono::milliseconds target_timeout{300};
};

struct ApproachResult {
    Twist2 command;
    bool target_valid = false;
    bool target_reached = false;
    bool aligning = false;
    bool capturing = false;
    bool capture_eligible = false;
    double distance_m = 0;
};

class ApproachController {
public:
    explicit ApproachController(ApproachControllerConfig config = {});
    ApproachResult update(const Pose2& pose, const TrackedObject& target,
                          Timestamp now, double dt_s);
    ApproachResult continue_capture(const Pose2& pose, Timestamp now, double dt_s);
    void reset();

private:
    ApproachControllerConfig config_;
    double forward_integral_ = 0;
    double left_integral_ = 0;
    double previous_forward_error_ = 0;
    double previous_left_error_ = 0;
    double previous_yaw_error_ = 0;
    Twist2 previous_command_;
    bool initialized_ = false;
    bool alignment_active_ = false;
    bool alignment_settle_required_ = false;
    bool capture_alignment_settling_ = false;
    Timestamp capture_alignment_settled_at_{};
    bool capture_finish_pending_ = false;
    bool capture_finish_active_ = false;
    Pose2 capture_finish_origin_;
    Timestamp capture_finish_started_{};
};

}  // namespace robot
