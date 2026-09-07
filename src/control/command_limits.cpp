#include "robot/control/command_limits.hpp"

#include <algorithm>
#include <cmath>

namespace robot {

Twist2 clamp_twist(const Twist2& command, double maximum_linear_mps,
                   double maximum_yaw_radps) {
    if (!std::isfinite(command.forward_mps) || !std::isfinite(command.left_mps) ||
        !std::isfinite(command.yaw_radps)) {
        return {};
    }
    Twist2 limited = command;
    const double magnitude = std::hypot(limited.forward_mps, limited.left_mps);
    if (magnitude > maximum_linear_mps && magnitude > 0) {
        const double scale = maximum_linear_mps / magnitude;
        limited.forward_mps *= scale;
        limited.left_mps *= scale;
    }
    limited.yaw_radps = std::clamp(limited.yaw_radps,
                                   -maximum_yaw_radps, maximum_yaw_radps);
    return limited;
}

}  // namespace robot
