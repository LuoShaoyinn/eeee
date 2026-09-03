#pragma once

#include <chrono>
#include <cstdint>

namespace robot {

using MonotonicClock = std::chrono::steady_clock;
using Timestamp = MonotonicClock::time_point;

struct Pose2 {
    double x_m = 0;
    double y_m = 0;
    double yaw_rad = 0;
};

struct Twist2 {
    double forward_mps = 0;
    double left_mps = 0;
    double yaw_radps = 0;
};

struct TimedTwist {
    Timestamp timestamp{};
    std::uint64_t sequence = 0;
    Twist2 value;
};

struct LocalizationState {
    Timestamp timestamp{};
    Pose2 pose;
    double position_sigma_m = 1;
    double yaw_sigma_rad = 3.141592653589793;
    bool globally_localized = false;
};

}  // namespace robot
