#pragma once

#include <string_view>

namespace robot {

enum class MissionState {
    inactive,
    search_target,
    align_target,
    approach_target,
    capture_target,
    safe_stop,
};

struct MissionInputs {
    bool target_active = false;
    bool aligning = false;
    bool capturing = false;
    bool capture_complete = false;
    bool search_complete = false;
    bool fault = false;
};

class SoloMission {
public:
    MissionState update(const MissionInputs& inputs);
    MissionState state() const;
    void start();
    void stop();
    void reset();

private:
    MissionState state_ = MissionState::inactive;
};

std::string_view to_string(MissionState state);

}  // namespace robot
