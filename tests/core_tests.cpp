#include <chrono>
#include <iostream>

#include "robot/control/safety_supervisor.hpp"
#include "robot/planning/mission.hpp"

namespace {

bool require(bool condition, const char* message) {
    if (!condition) std::cerr << "failed: " << message << '\n';
    return condition;
}

}  // namespace

int main() {
    using namespace std::chrono_literals;
    const auto now = robot::MonotonicClock::now();
    robot::LocalizationState localization{.timestamp = now, .pose = {}, .position_sigma_m = .05,
                                          .globally_localized = true};
    robot::SafetySupervisor safety;
    auto result = safety.evaluate({1, 0, 4}, localization, now, false);
    if (!require(result.limited, "overspeed command is limited") ||
        !require(result.command.forward_mps == .45, "linear speed clamp") ||
        !require(result.command.yaw_radps == 2.0, "yaw speed clamp")) return 1;
    result = safety.evaluate({.1, 0, 0}, localization, now + 301ms, false);
    if (!require(result.stopped, "stale localization stops motion")) return 1;

    robot::SoloMission mission;
    if (!require(mission.update({.hardware_ready = true}) == robot::MissionState::self_test,
                 "boot to self-test") ||
        !require(mission.update({.localized = true}) == robot::MissionState::search_target,
                 "self-test to search") ||
        !require(mission.update({.localized = true, .target_visible = true}) ==
                     robot::MissionState::approach_target,
                 "search to approach") ||
        !require(mission.update({.fault = true}) == robot::MissionState::safe_stop,
                 "fault to safe stop")) return 1;
}
