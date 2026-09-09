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
        check(unload.command().forward_mps < 0 && !unload.complete());
        check(mission.update({.unload_complete=unload.complete()}) == robot::MissionState::unload);
        auto wrong_direction = origin; wrong_direction.x_m += .06;
        (void)unload.update(now+2125ms, wrong_direction);
        check(unload.command().forward_mps < 0 && !unload.complete());
        auto short_advance = origin; short_advance.x_m -= .031;
        (void)unload.update(now+2150ms, short_advance);
        check(unload.command().forward_mps < 0);
        auto arrived = origin; arrived.x_m -= .051;
        (void)unload.update(now+2200ms, arrived);
        check(unload.command().forward_mps == 0 && !unload.complete());
        (void)unload.update(now+2700ms, arrived);
        (void)unload.update(now+2800ms, arrived);
        (void)unload.update(now+4800ms, arrived);
        check(!unload.complete());
        check(mission.update({.unload_complete=unload.complete()}) == robot::MissionState::unload);
        (void)unload.update(now+4900ms, arrived);
        check(unload.command().forward_mps > 0);
        auto final_position = arrived; final_position.x_m += .051;
        (void)unload.update(now+5s, final_position);
        check(unload.command().forward_mps == 0 && !unload.complete());
        (void)unload.update(now+5500ms, final_position);
        (void)unload.update(now+5600ms, final_position);
        (void)unload.update(now+7600ms, final_position);
        check(!unload.complete());
        check(mission.update({.unload_complete=unload.complete()}) == robot::MissionState::unload);
        for (int leg = 0; leg < 2; ++leg) {
            const auto leg_time = now + 7700ms + leg * 2800ms;
            (void)unload.update(leg_time, final_position);
            check(unload.command().forward_mps > 0);
            auto short_move = final_position; short_move.x_m += .03;
            (void)unload.update(leg_time+50ms, short_move);
            check(unload.command().forward_mps > 0);
            final_position.x_m += .051;
            (void)unload.update(leg_time+100ms, final_position);
            check(unload.command().forward_mps == 0 && !unload.complete());
            (void)unload.update(leg_time+600ms, final_position);
            (void)unload.update(leg_time+700ms, final_position);
            (void)unload.update(leg_time+2700ms, final_position);
            check(unload.complete() == (leg == 1));
        }
        check(unload.complete());
        check(mission.update({.unload_complete=unload.complete()}) == robot::MissionState::search_target);
        search.reset(); unload.reset();
        check(search.collectibles_allowed() && !unload.complete());
        check(mission.update({.target_active=true}) == robot::MissionState::approach_target);
        check(mission.update({}) == robot::MissionState::search_target);
        now += 20s;
    }
    mission.update({.search_complete=true});
    check(mission.update({.unload_complete=true, .fault=true}) == robot::MissionState::safe_stop);
    mission.reset(); mission.start(); mission.update({.search_complete=true}); mission.stop();
    check(mission.update({.unload_complete=true}) == robot::MissionState::inactive);
    std::cout << "PASS: repeated unload/search rounds, fresh target acquisition, fault and manual stop\n";
}
