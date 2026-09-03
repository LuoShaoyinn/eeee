#pragma once

#include <string_view>

namespace robot {

enum class MissionState {
    boot,
    self_test,
    localize,
    search_target,
    approach_target,
    acquire_target,
    navigate_home,
    deposit,
    recover_localization,
    safe_stop,
};

struct MissionInputs {
    bool hardware_ready = false;
    bool localized = false;
    bool target_visible = false;
    bool target_reached = false;
    bool target_acquired = false;
    bool home_reached = false;
    bool deposit_complete = false;
    bool fault = false;
};

class SoloMission {
public:
    MissionState update(const MissionInputs& inputs);
    MissionState state() const;

private:
    MissionState state_ = MissionState::boot;
};

std::string_view to_string(MissionState state);

}  // namespace robot
