#include <cassert>
#include <cmath>
#include <limits>

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
    // Global docking uses the camera pose even with no Home detection.
    robot::MissionConfig global;
    global.global_home = true;
    global.expected_collectibles = 1;
    global.frames_to_confirm_collection = 1;
    global.frames_to_confirm_dock = 3;
    global.dump_duration_s = .1;
    robot::MissionInput frame;
    frame.localization_valid = true;
    frame.pose_valid = true;
    frame.camera_x_m = 1.0;
    frame.camera_y_m = .8;
    frame.chassis_yaw_rad = 1.57;
    const auto collect = [&](robot::MissionController& controller) {
        (void)controller.update(frame);
        frame.detections = {object(robot::ObjectClass::yellow, .9)};
        (void)controller.update(frame);
        frame.detections.clear();
        return controller.update(frame);
    };
    robot::MissionController global_mission(global);
    auto global_output = collect(global_mission);
    assert(global_output.yaw_radps < 0 && global_output.forward_mps == 0 && global_output.left_mps == 0);
    frame.chassis_yaw_rad = 0;
    global_output = global_mission.update(frame);
    assert(global_output.forward_mps < 0 && global_output.left_mps < 0);
    assert(std::hypot(global_output.forward_mps, global_output.left_mps) <= .100001);
    assert(global_output.collector_percent == 0 && !global_output.servo_pulse_us);
    frame.camera_x_m = .28;
    frame.camera_y_m = .12;
    assert(!global_mission.update(frame).servo_pulse_us);
    assert(!global_mission.update(frame).servo_pulse_us);
    frame.detections = {object(robot::ObjectClass::other_robot, .9)};
    global_output = global_mission.update(frame);
    assert(global_output.forward_mps == 0 && global_output.left_mps == 0 && !global_output.servo_pulse_us);
    frame.detections.clear();
    assert(!global_mission.update(frame).servo_pulse_us);
    frame.camera_x_m = .4;  // A pose excursion resets consecutive docking confirmation.
    assert(!global_mission.update(frame).servo_pulse_us);
    frame.camera_x_m = .28;
    assert(!global_mission.update(frame).servo_pulse_us);
    assert(!global_mission.update(frame).servo_pulse_us);
    global_output = global_mission.update(frame);
    assert(global_output.state == robot::MissionState::dumping && global_output.servo_pulse_us == 2000);
    assert(global_output.forward_mps == 0 && global_output.left_mps == 0 && global_output.yaw_radps == 0);
    global_output = global_mission.update(frame);
    assert(global_output.state == robot::MissionState::done && global_output.servo_pulse_us == 1600);
    robot::MissionController missing_pose(global);
    frame.pose_valid = false;
    assert(missing_pose.update(frame).state == robot::MissionState::searching);
    frame.pose_valid = true;
    robot::MissionController nan_pose(global);
    frame.camera_x_m = std::numeric_limits<double>::quiet_NaN();
    assert(nan_pose.update(frame).state == robot::MissionState::searching);
    frame.camera_x_m = 1.0;
    global.home_timeout_s = .15;
    robot::MissionController timeout(global);
    (void)collect(timeout);
    (void)timeout.update(frame);
    assert(timeout.update(frame).emergency_stop);

    robot::MissionConfig config;
    config.expected_collectibles = 1;
    config.frames_to_confirm_collection = 1;
    config.frames_to_confirm_dock = 1;
    config.dump_duration_s = .05;
    robot::MissionController mission(config);

    assert(mission.update({.localization_valid = true, .detections = {}}).state == robot::MissionState::searching);
    auto output = mission.update({.localization_valid = true,
                                  .detections = {object(robot::ObjectClass::yellow, .9)}});
    assert(output.state == robot::MissionState::approaching_target);
    assert(output.collector_percent == -100);
    assert(output.forward_mps == .18);

    output = mission.update({.localization_valid = true,
                             .detections = {object(robot::ObjectClass::yellow, .7)}});
    // Once the intake-zone confirmation begins, the collector keeps running
    // but the chassis must not rotate or drive the object back out.
    assert(output.forward_mps == 0.0 && output.left_mps == 0.0 && output.yaw_radps == 0.0);
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
                                    .odometry_valid = true, .odometry_forward_m = 1.0,
                                    .detections = {ground_object(robot::ObjectClass::yellow, .025, .003)}});
    assert(output.state == robot::MissionState::intake_run && output.forward_mps == .15);

    // Mecanum visual servo: a lateral miss closes the forward gate while the
    // robot strafes and yaws; a centred object then releases forward PID.
    robot::MissionConfig servo_config;
    servo_config.expected_collectibles = 0;
    robot::MissionController servo_mission(servo_config);
    (void)servo_mission.update({.localization_valid = true, .detections = {}});
    output = servo_mission.update({.localization_valid = true, .control_dt_s = .10,
                                   .detections = {ground_object(robot::ObjectClass::yellow, .60, .16)}});
    assert(output.forward_mps == 0.0);
    assert(output.left_mps > 0.0);
    assert(output.yaw_radps > 0.0);
    output = servo_mission.update({.localization_valid = true, .control_dt_s = .10,
                                   .detections = {ground_object(robot::ObjectClass::yellow, .60, .01)}});
    assert(output.forward_mps > .30);
    assert(output.left_mps == 0.0);
    assert(output.yaw_radps == 0.0);

    // Being close in range alone must never be treated as a collected object:
    // the intake must also be centred before the collection-confirmation wait.
    robot::MissionController missed_intake_mission(config);
    (void)missed_intake_mission.update({.localization_valid = true, .detections = {}});
    output = missed_intake_mission.update({.localization_valid = true,
                                           .detections = {ground_object(robot::ObjectClass::yellow, .14, .09)}});
    assert(output.forward_mps == 0.0 && output.left_mps > 0.0);
    output = missed_intake_mission.update({.localization_valid = true, .detections = {}});
    assert(output.state == robot::MissionState::approaching_target);

    robot::MissionController aligned_intake_mission(config);
    (void)aligned_intake_mission.update({.localization_valid = true, .detections = {}});
    (void)aligned_intake_mission.update({.localization_valid = true,
                                         .odometry_valid = true, .odometry_forward_m = 1.0,
                                         .detections = {ground_object(robot::ObjectClass::yellow, .025, .003)}});
    (void)aligned_intake_mission.update({.localization_valid = true,
                                         .odometry_valid = true, .odometry_forward_m = 1.25, .detections = {}});
    output = aligned_intake_mission.update({.localization_valid = true,
                                            .odometry_valid = true, .odometry_forward_m = 1.25, .detections = {}});
    assert(output.state == robot::MissionState::returning_home);

    // The supervised object-servo test intentionally freezes global
    // localization dependencies: it collects one centred object and stops.
    robot::MissionConfig object_test_config;
    object_test_config.object_servo_test = true;
    object_test_config.frames_to_confirm_collection = 1;
    robot::MissionController object_test_mission(object_test_config);
    output = object_test_mission.update({.localization_valid = false, .detections = {}});
    assert(output.state == robot::MissionState::searching);
    (void)object_test_mission.update({.localization_valid = false,
                                     .odometry_valid = true, .odometry_forward_m = 1.0,
                                     .detections = {ground_object(robot::ObjectClass::yellow, .025, .003)}});
    (void)object_test_mission.update({.localization_valid = false,
                                      .odometry_valid = true, .odometry_forward_m = 1.25, .detections = {}});
    output = object_test_mission.update({.localization_valid = false,
                                         .odometry_valid = true, .odometry_forward_m = 1.25, .detections = {}});
    assert(output.state == robot::MissionState::done && output.collector_percent == 0);

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
    assert(output.state == robot::MissionState::searching && !output.emergency_stop);
}
