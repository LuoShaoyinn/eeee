#pragma once

#include "robot/core/types.hpp"

namespace robot {

// Preserve a planar command's direction while keeping it within the calibrated
// transport envelope accepted by the ESP32 firmware.
Twist2 clamp_twist(const Twist2& command, double maximum_linear_mps,
                   double maximum_yaw_radps);

}  // namespace robot
