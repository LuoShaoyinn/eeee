#include <cassert>

#include "robot/mission.hpp"

namespace {

robot::Detection object(robot::ObjectClass type, double bottom, double centre = .51) {
    return {.object_class = type, .confidence = .9, .center_x = centre, .bottom_y = bottom};
}

robot::Detection ground_object(robot::ObjectClass type, double forward, double left) {
    return {.object_class = type, .confidence = .9, .center_x = .51, .bottom_y = .5,
            .ground_valid = true, .ground_forward_m = forward, .ground_left_m = left};
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

    // A calibrated ground point expresses the target relative to the intake,
    // not relative to the (left-mounted) image centre.
    robot::MissionController ground_mission(config);
    (void)ground_mission.update({.localization_valid = true, .detections = {}});
    output = ground_mission.update({.localization_valid = true,
                                    .detections = {ground_object(robot::ObjectClass::yellow, .50, .20)}});
    assert(output.state == robot::MissionState::approaching_target);
    assert(output.forward_mps == 0.0);
    output = ground_mission.update({.localization_valid = true,
                                    .detections = {ground_object(robot::ObjectClass::yellow, .12, .01)}});
    assert(output.forward_mps == .18);

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

    // Lock the acquired object: a nearer object of the same colour and an
    // object of the other colour must not repeatedly steal the pursuit.
    robot::MissionConfig lock_config;
    lock_config.expected_collectibles = 0;
    robot::MissionController lock_mission(lock_config);
    (void)lock_mission.update({.localization_valid = true, .detections = {}});
    output = lock_mission.update({.localization_valid = true,
                                  .detections = {object(robot::ObjectClass::yellow, .70, .72),
                                                 object(robot::ObjectClass::red, .65, .30)}});
    assert(output.yaw_radps < 0.0);
    output = lock_mission.update({.localization_valid = true,
                                  .detections = {object(robot::ObjectClass::yellow, .50, .72),
                                                 object(robot::ObjectClass::yellow, .92, .30),
                                                 object(robot::ObjectClass::red, .95, .30)}});
    assert(output.state == robot::MissionState::approaching_target);
    assert(output.yaw_radps < 0.0);

    robot::MissionController fault_mission(config);
    (void)fault_mission.update({.localization_valid = true, .detections = {}});
    output = fault_mission.update({.localization_valid = false, .detections = {}});
    assert(output.state == robot::MissionState::fault && output.emergency_stop);
}
