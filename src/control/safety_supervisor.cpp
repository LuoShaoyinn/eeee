#include "robot/control/safety_supervisor.hpp"

#include <algorithm>
#include <cmath>

namespace robot {

SafetySupervisor::SafetySupervisor(SafetyLimits limits) : limits_(limits) {}

SafetyResult SafetySupervisor::evaluate(const Twist2& requested,
                                        const LocalizationState& localization,
                                        Timestamp now, bool fault) const {
    SafetyResult result;
    if (fault || !localization.globally_localized || localization.timestamp == Timestamp{} ||
        now - localization.timestamp > limits_.localization_timeout) {
        result.stopped = true;
        result.limited = requested.forward_mps != 0 || requested.left_mps != 0 || requested.yaw_radps != 0;
        return result;
    }

    double scale = 1.0;
    if (localization.position_sigma_m > limits_.uncertain_position_sigma_m) {
        scale = limits_.uncertain_speed_scale;
        result.limited = true;
    }
    result.command.forward_mps = requested.forward_mps * scale;
    result.command.left_mps = requested.left_mps * scale;
    result.command.yaw_radps = requested.yaw_radps * scale;
    const double magnitude = std::hypot(result.command.forward_mps, result.command.left_mps);
    if (magnitude > limits_.max_linear_mps) {
        const double clamp_scale = limits_.max_linear_mps / magnitude;
        result.command.forward_mps *= clamp_scale;
        result.command.left_mps *= clamp_scale;
        result.limited = true;
    }
    const double yaw = std::clamp(result.command.yaw_radps,
                                  -limits_.max_yaw_radps, limits_.max_yaw_radps);
    result.limited = result.limited || yaw != result.command.yaw_radps;
    result.command.yaw_radps = yaw;
    return result;
}

}  // namespace robot
