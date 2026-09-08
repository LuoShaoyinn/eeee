#include "robot/planning/wall_contact_detector.hpp"

#include <algorithm>

namespace robot {

WallContactDetector::WallContactDetector(WallContactDetectorConfig config) : config_(config) {}

bool WallContactDetector::update(bool reverse_commanded, double accel_x_g,
                                 std::chrono::milliseconds imu_age, Timestamp now) {
    const bool valid = imu_age <= config_.maximum_imu_age;
    if (!reverse_commanded || !valid) {
        reverse_started_ = {};
        hit_latched_ = false;
        residual_g_ = 0;
        if (valid) {
            baseline_x_g_ = baseline_valid_ ? .95 * baseline_x_g_ + .05 * accel_x_g : accel_x_g;
            baseline_valid_ = true;
        }
        return false;
    }
    if (!baseline_valid_) {
        baseline_x_g_ = accel_x_g;
        baseline_valid_ = true;
    }
    if (reverse_started_ == Timestamp{}) reverse_started_ = now;
    residual_g_ = accel_x_g - baseline_x_g_;
    if (!config_.enabled || hit_latched_ || now < reverse_started_ ||
        now - reverse_started_ < config_.reverse_arm_time ||
        residual_g_ < config_.impact_threshold_g) {
        return false;
    }
    hit_latched_ = true;
    return true;
}

void WallContactDetector::reset() {
    baseline_valid_ = false;
    baseline_x_g_ = 0;
    residual_g_ = 0;
    reverse_started_ = {};
    hit_latched_ = false;
}

double WallContactDetector::residual_g() const { return residual_g_; }

}  // namespace robot
