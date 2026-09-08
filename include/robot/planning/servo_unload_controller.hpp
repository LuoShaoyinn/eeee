#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <vector>

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
