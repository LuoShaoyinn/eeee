#include "robot/planning/mission.hpp"

namespace robot {

MissionState SoloMission::update(const MissionInputs& inputs) {
    if (inputs.fault) return state_ = MissionState::safe_stop;
    if (state_ != MissionState::boot && state_ != MissionState::self_test &&
        state_ != MissionState::safe_stop && !inputs.localized) {
        return state_ = MissionState::recover_localization;
    }
    switch (state_) {
    case MissionState::boot: if (inputs.hardware_ready) state_ = MissionState::self_test; break;
    case MissionState::self_test: if (inputs.localized) state_ = MissionState::search_target; else state_ = MissionState::localize; break;
    case MissionState::localize: if (inputs.localized) state_ = MissionState::search_target; break;
    case MissionState::search_target: if (inputs.target_visible) state_ = MissionState::approach_target; break;
    case MissionState::approach_target: if (inputs.target_reached) state_ = MissionState::acquire_target; break;
    case MissionState::acquire_target: if (inputs.target_acquired) state_ = MissionState::navigate_home; break;
    case MissionState::navigate_home: if (inputs.home_reached) state_ = MissionState::deposit; break;
    case MissionState::deposit: if (inputs.deposit_complete) state_ = MissionState::search_target; break;
    case MissionState::recover_localization: if (inputs.localized) state_ = MissionState::search_target; break;
    case MissionState::safe_stop: break;
    }
    return state_;
}

MissionState SoloMission::state() const { return state_; }

std::string_view to_string(MissionState state) {
    switch (state) {
    case MissionState::boot: return "boot";
    case MissionState::self_test: return "self_test";
    case MissionState::localize: return "localize";
    case MissionState::search_target: return "search_target";
    case MissionState::approach_target: return "approach_target";
    case MissionState::acquire_target: return "acquire_target";
    case MissionState::navigate_home: return "navigate_home";
    case MissionState::deposit: return "deposit";
    case MissionState::recover_localization: return "recover_localization";
    case MissionState::safe_stop: return "safe_stop";
    }
    return "unknown";
}

}  // namespace robot
