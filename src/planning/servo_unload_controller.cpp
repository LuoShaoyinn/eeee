#include "robot/planning/servo_unload_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace robot {

ServoUnloadController::ServoUnloadController(ServoUnloadConfig config) : config_(std::move(config)) {
    reset();
}

std::optional<int> ServoUnloadController::update(Timestamp now) {
    if (complete_) return std::nullopt;
    if (!started_) {
        started_ = true;
        segment_started_ = now;
        segment_start_pulse_us_ = config_.pulse_us.back();
    }

    int output = segment_start_pulse_us_;
    while (segment_ < config_.pulse_us.size()) {
        const int target = config_.pulse_us[segment_];
        const auto duration = std::chrono::milliseconds(config_.duration_ms[segment_]);
        const auto elapsed = now - segment_started_;
        if (duration.count() > 0 && elapsed < duration) {
            const double progress = std::clamp(
                std::chrono::duration<double>(elapsed).count() /
                    std::chrono::duration<double>(duration).count(),
                0.0, 1.0);
            output = static_cast<int>(std::lround(
                segment_start_pulse_us_ + progress * (target - segment_start_pulse_us_)));
            break;
        }
        output = target;
        segment_started_ += duration;
        segment_start_pulse_us_ = target;
        ++segment_;
    }
    if (segment_ == config_.pulse_us.size()) complete_ = true;
    if (!last_output_pulse_us_ || *last_output_pulse_us_ != output) {
        last_output_pulse_us_ = output;
        return output;
    }
    return std::nullopt;
}

bool ServoUnloadController::complete() const { return complete_; }

void ServoUnloadController::reset() {
    segment_ = 0;
    segment_start_pulse_us_ = config_.pulse_us.empty() ? 0 : config_.pulse_us.back();
    last_output_pulse_us_.reset();
    segment_started_ = {};
    started_ = false;
    complete_ = false;
}

}  // namespace robot
