#pragma once

#include <chrono>

#include "robot/core/types.hpp"
#include "robot/planning/world_model.hpp"

namespace robot {

struct ApproachControllerConfig {
    double translation_kp = .75;
    double translation_ki = .04;
    double translation_kd = .06;
    double yaw_kp = 1.4;
    double yaw_kd = .08;
    double maximum_linear_mps = .28;
    double maximum_yaw_radps = .8;
    double maximum_linear_accel_mps2 = .5;
    double maximum_yaw_accel_radps2 = 1.5;
    double stopping_distance_m = .18;
    double integral_limit_m_s = .25;
    std::chrono::milliseconds target_timeout{300};
};

struct ApproachResult {
    Twist2 command;
    bool target_valid = false;
    bool target_reached = false;
    double distance_m = 0;
};

class ApproachController {
public:
    explicit ApproachController(ApproachControllerConfig config = {});
    ApproachResult update(const Pose2& pose, const TrackedObject& target,
                          Timestamp now, double dt_s);
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
};

}  // namespace robot
