#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <vector>
#include <cmath>
#include <stdexcept>

#include "robot/core/types.hpp"

namespace robot {

struct ServoUnloadConfig {
    std::vector<int> pulse_us;
    std::vector<int> duration_ms;
};

// Each pulse/duration pair is a linear segment from the prior target. The
// resting pulse before index zero is the last pulse in the sequence.
class ServoUnloadController {
public:
    explicit ServoUnloadController(ServoUnloadConfig config);
    std::optional<int> update(Timestamp now);
    bool complete() const;
    void reset();

private:
    ServoUnloadConfig config_;
    std::size_t segment_ = 0;
    int segment_start_pulse_us_ = 0;
    std::optional<int> last_output_pulse_us_;
    Timestamp segment_started_{};
    bool started_ = false;
    bool complete_ = false;
};

}  // namespace robot

namespace robot {
// Five two-cycle batches: reverse 5 cm once, then advance 5 cm three times.
class StagedUnloadController {
public:
    explicit StagedUnloadController(ServoUnloadConfig config) : servo_(two_cycles(config)) {}
    void reset() { servo_.reset(); phase_ = 0; batches_completed_ = 0; command_ = {}; }
    bool complete() const { return phase_ == 4; }
    Twist2 command() const { return command_; }
    std::optional<int> update(Timestamp now, const Pose2& odometry) {
        command_ = {};
        if (phase_ == 0 || phase_ == 3) {
            auto pulse = servo_.update(now);
            if (servo_.complete()) {
                if (++batches_completed_ == 5) phase_ = 4;
                else { origin_ = odometry; started_ = now; phase_ = 1; }
            }
            return pulse;
        }
        if (phase_ == 1) {
            const double direction = batches_completed_ == 1 ? -1.0 : 1.0;
            const double advance = std::cos(origin_.yaw_rad) * (odometry.x_m - origin_.x_m) +
                                   std::sin(origin_.yaw_rad) * (odometry.y_m - origin_.y_m);
            if (!std::isfinite(advance)) throw std::runtime_error("unload advance: invalid odometry");
            if (direction * advance >= .05) { phase_ = 2; started_ = now; }
            else {
                if (now - started_ > std::chrono::seconds(3))
                    throw std::runtime_error("unload advance: odometry timeout");
                command_.forward_mps = direction * .05;
            }
        } else if (phase_ == 2 && now - started_ >= std::chrono::milliseconds(500)) {
            servo_.reset(); phase_ = 3;
        }
        return std::nullopt;
    }
private:
    static ServoUnloadConfig two_cycles(ServoUnloadConfig config) {
        if (config.pulse_us.size() < 4 || config.duration_ms.size() < 4)
            throw std::runtime_error("staged unload requires two pulse pairs");
        config.pulse_us.resize(4); config.duration_ms.resize(4);
        return config;
    }
    ServoUnloadController servo_;
    int phase_ = 0;
    int batches_completed_ = 0;
    Pose2 origin_{};
    Timestamp started_{};
    Twist2 command_{};
};
}
