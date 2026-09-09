#include <chrono>
#include <iostream>
#include <stdexcept>
#include "robot/planning/mission.hpp"
#include "robot/planning/search_controller.hpp"
#include "robot/planning/servo_unload_controller.hpp"

int main() {
    using namespace std::chrono_literals;
    auto now = robot::MonotonicClock::now();
    robot::SoloMission mission;
    robot::SearchController search;
    robot::StagedUnloadController unload({{1600,2000,1600,2000}, {0,1000,0,1000}});
    auto check = [](bool ok) { if (!ok) throw std::runtime_error("unload round regression"); };
    mission.start();
    for (int round = 0; round < 2; ++round) {
        search.begin_return_home();
        check(!search.collectibles_allowed());
        check(mission.update({.search_complete=true}) == robot::MissionState::unload);
        const robot::Pose2 origin{.x_m=1, .y_m=1, .yaw_rad=0};
        (void)unload.update(now, origin);
        (void)unload.update(now+2s, origin);
        (void)unload.update(now+2100ms, origin);
        check(unload.command().forward_mps > 0 && !unload.complete());
        check(mission.update({.unload_complete=unload.complete()}) == robot::MissionState::unload);
        auto arrived = origin; arrived.x_m += .031;
        (void)unload.update(now+2200ms, arrived);
        check(unload.command().forward_mps == 0 && !unload.complete());
        (void)unload.update(now+2700ms, arrived);
        (void)unload.update(now+2800ms, arrived);
        (void)unload.update(now+4800ms, arrived);
        check(unload.complete());
        check(mission.update({.unload_complete=unload.complete()}) == robot::MissionState::search_target);
        search.reset(); unload.reset();
        check(search.collectibles_allowed() && !unload.complete());
        check(mission.update({.target_active=true}) == robot::MissionState::approach_target);
        check(mission.update({}) == robot::MissionState::search_target);
        now += 10s;
    }
    mission.update({.search_complete=true});
    check(mission.update({.unload_complete=true, .fault=true}) == robot::MissionState::safe_stop);
    mission.reset(); mission.start(); mission.update({.search_complete=true}); mission.stop();
    check(mission.update({.unload_complete=true}) == robot::MissionState::inactive);
    std::cout << "PASS: repeated unload/search rounds, fresh target acquisition, fault and manual stop\n";
}
