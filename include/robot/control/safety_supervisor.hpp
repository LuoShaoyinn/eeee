#pragma once

#include <chrono>

#include "robot/core/types.hpp"

namespace robot {

struct SafetyLimits {
    double max_linear_mps = .45;
    double max_yaw_radps = 2.0;
    double uncertain_position_sigma_m = .25;
    double uncertain_speed_scale = .25;
    std::chrono::milliseconds localization_timeout{300};
};

struct SafetyResult {
    Twist2 command;
    bool stopped = false;
    bool limited = false;
};

class SafetySupervisor {
public:
    explicit SafetySupervisor(SafetyLimits limits = {});
    SafetyResult evaluate(const Twist2& requested, const LocalizationState& localization,
                          Timestamp now, bool fault) const;

private:
    SafetyLimits limits_;
};

}  // namespace robot
