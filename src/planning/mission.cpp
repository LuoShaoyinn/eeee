#include "robot/planning/mission.hpp"

namespace robot {

MissionState SoloMission::update(const MissionInputs& inputs) {
    if (inputs.fault) return state_ = MissionState::safe_stop;
    switch (state_) {
    case MissionState::inactive: break;
    case MissionState::search_target:
        if (inputs.search_complete) state_ = MissionState::unload;
        else if (inputs.target_active) {
            state_ = inputs.aligning ? MissionState::align_target : MissionState::approach_target;
        }
        break;
    case MissionState::align_target:
        if (!inputs.target_active) state_ = MissionState::search_target;
        else if (inputs.capturing) state_ = MissionState::capture_target;
        else if (!inputs.aligning) state_ = MissionState::approach_target;
        break;
    case MissionState::approach_target:
        if (!inputs.target_active) state_ = MissionState::search_target;
        else if (inputs.capturing) state_ = MissionState::capture_target;
        else if (inputs.aligning) state_ = MissionState::align_target;
        break;
    case MissionState::capture_target:
        if (inputs.capture_complete) state_ = MissionState::search_target;
        else if (!inputs.capturing) state_ = MissionState::search_target;
        break;
    case MissionState::unload:
        if (inputs.unload_complete) state_ = MissionState::search_target;
        break;
    case MissionState::safe_stop: break;
    }
    return state_;
}

MissionState SoloMission::state() const { return state_; }

void SoloMission::start() {
    if (state_ != MissionState::safe_stop) state_ = MissionState::search_target;
}

void SoloMission::stop() { state_ = MissionState::inactive; }

void SoloMission::reset() { state_ = MissionState::inactive; }

std::string_view to_string(MissionState state) {
    switch (state) {
    case MissionState::inactive: return "inactive";
    case MissionState::search_target: return "search_target";
    case MissionState::align_target: return "align_target";
    case MissionState::approach_target: return "approach_target";
    case MissionState::capture_target: return "capture_target";
    case MissionState::unload: return "unload";
    case MissionState::safe_stop: return "safe_stop";
    }
    return "unknown";
}

}  // namespace robot
