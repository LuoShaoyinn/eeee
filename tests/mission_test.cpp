#include <cassert>

#include "robot/mission.hpp"

namespace {

robot::Detection object(robot::ObjectClass type, double bottom, double centre = .51) {
    return {.object_class = type, .confidence = .9, .center_x = centre, .bottom_y = bottom};
}

}  // namespace

int main() {
    robot::MissionConfig config;
    config.expected_collectibles = 1;
    config.frames_to_confirm_collection = 1;
    config.frames_to_confirm_dock = 1;
    robot::MissionController mission(config);

    assert(mission.update({.localization_valid = true, .detections = {}}).state == robot::MissionState::searching);
    auto output = mission.update({.localization_valid = true,
                                  .detections = {object(robot::ObjectClass::yellow, .9)}});
    assert(output.state == robot::MissionState::approaching_target);
    assert(output.collector_percent == -100);
    assert(output.forward_mps == .18);

    output = mission.update({.localization_valid = true,
                             .detections = {object(robot::ObjectClass::yellow, .7)}});
    assert(output.forward_mps == .35);
    output = mission.update({.localization_valid = true,
                             .detections = {object(robot::ObjectClass::yellow, .7, .20)}});
    assert(output.forward_mps == 0.0);

    output = mission.update({.localization_valid = true,
                             .detections = {object(robot::ObjectClass::other_robot, .7)}});
    assert(output.state == robot::MissionState::avoiding_robot);
    assert(output.forward_mps == 0.0);

    output = mission.update({.localization_valid = true, .detections = {}});
    assert(output.state == robot::MissionState::returning_home);
    output = mission.update({.localization_valid = true,
                             .detections = {object(robot::ObjectClass::home, .9)}});
    assert(output.state == robot::MissionState::dumping);
    assert(output.servo_pulse_us == config.dump_servo_pulse_us);
    output = mission.update({.localization_valid = true, .detections = {}});
    assert(output.state == robot::MissionState::done);
    assert(output.collector_percent == 0);

    robot::MissionConfig distinct_config;
    distinct_config.expected_collectibles = 2;
    distinct_config.frames_to_confirm_collection = 1;
    robot::MissionController distinct_mission(distinct_config);
    (void)distinct_mission.update({.localization_valid = true, .detections = {}});
    (void)distinct_mission.update({.localization_valid = true,
                                   .detections = {object(robot::ObjectClass::yellow, .9)}});
    (void)distinct_mission.update({.localization_valid = true, .detections = {}});
    output = distinct_mission.update({.localization_valid = true,
                                      .detections = {object(robot::ObjectClass::yellow, .9)}});
    assert(output.state == robot::MissionState::searching);
    (void)distinct_mission.update({.localization_valid = true,
                                   .detections = {object(robot::ObjectClass::red, .9)}});
    output = distinct_mission.update({.localization_valid = true, .detections = {}});
    assert(output.state == robot::MissionState::returning_home);

    robot::MissionController fault_mission(config);
    (void)fault_mission.update({.localization_valid = true, .detections = {}});
    output = fault_mission.update({.localization_valid = false, .detections = {}});
    assert(output.state == robot::MissionState::fault && output.emergency_stop);
}
