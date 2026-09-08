#pragma once

#include <chrono>

#include "robot/core/types.hpp"

namespace robot {

// Detect a wall impact from the IMU's forward-axis acceleration. The IMU is
// mounted with +X forward, so braking a reverse motion produces a +X spike.
struct WallContactDetectorConfig {
    bool enabled = true;
    double impact_threshold_g = .35;
    std::chrono::milliseconds reverse_arm_time{300};
    std::chrono::milliseconds maximum_imu_age{150};
};

class WallContactDetector {
public:
    explicit WallContactDetector(WallContactDetectorConfig config = {});

    // `reverse_commanded` must describe the command currently sent to the
    // chassis. Returns true once per reverse leg after its arming interval.
    bool update(bool reverse_commanded, double accel_x_g,
                std::chrono::milliseconds imu_age, Timestamp now);
    void reset();
    double residual_g() const;

private:
    WallContactDetectorConfig config_;
    bool baseline_valid_ = false;
    double baseline_x_g_ = 0;
    double residual_g_ = 0;
    Timestamp reverse_started_{};
    bool hit_latched_ = false;
};

}  // namespace robot
