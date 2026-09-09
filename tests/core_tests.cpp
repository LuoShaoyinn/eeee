#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>

#include "robot/control/safety_supervisor.hpp"
#include "robot/control/command_limits.hpp"
#include "robot/perception/object_projection.hpp"
#include "robot/planning/approach_controller.hpp"
#include "robot/planning/local_target_tracker.hpp"
#include "robot/planning/mission.hpp"
#include "robot/planning/search_controller.hpp"
#include "robot/planning/servo_unload_controller.hpp"
#include "robot/planning/world_model.hpp"
#include "robot/planning/vehicle_geometry.hpp"
#include "robot/planning/wall_contact_detector.hpp"

namespace {

bool require(bool condition, const char* message) {
    if (!condition) std::cerr << "failed: " << message << '\n';
    return condition;
}

}  // namespace

int main() {
    using namespace std::chrono_literals;
    const auto now = robot::MonotonicClock::now();
    {
        robot::StagedUnloadController staged({{1600,2000,1600,2000}, {0,1000,0,1000}});
        const robot::Pose2 origin{.x_m=1, .y_m=1, .yaw_rad=1.5707963267948966};
        (void)staged.update(now, origin);
        (void)staged.update(now+2s, origin);
        (void)staged.update(now+2100ms, origin);
        if (!require(staged.command().forward_mps == -.05 && !staged.complete(), "reverse after two unload cycles")) return 1;
        auto lateral = origin; lateral.x_m += .04;
        (void)staged.update(now+2200ms, lateral);
        if (!require(staged.command().forward_mps < 0, "lateral displacement must not complete reverse")) return 1;
        auto arrived = origin; arrived.y_m -= .051;
        (void)staged.update(now+2300ms, arrived);
        if (!require(staged.command().forward_mps == 0 && !staged.complete(), "stop before second unload batch")) return 1;
        (void)staged.update(now+2800ms, arrived);
        (void)staged.update(now+2900ms, arrived);
        (void)staged.update(now+4900ms, arrived);
        if (!require(!staged.complete(), "second unload batch must not finish unloading")) return 1;
        (void)staged.update(now+5s, arrived);
        if (!require(staged.command().forward_mps > 0, "advance again after second batch")) return 1;
        auto final_position = arrived; final_position.y_m += .051;
        (void)staged.update(now+5100ms, final_position);
        if (!require(staged.command().forward_mps == 0, "stop before third unload batch")) return 1;
        (void)staged.update(now+5600ms, final_position);
        (void)staged.update(now+5700ms, final_position);
        (void)staged.update(now+7700ms, final_position);
        if (!require(!staged.complete(), "third unload batch must not finish unloading")) return 1;
        for (int leg = 0; leg < 2; ++leg) {
            const auto leg_time = now + 7800ms + leg * 2800ms;
            (void)staged.update(leg_time, final_position);
            if (!require(staged.command().forward_mps > 0, "remaining legs move forward")) return 1;
            final_position.y_m += .051;
            (void)staged.update(leg_time+100ms, final_position);
            (void)staged.update(leg_time+600ms, final_position);
            (void)staged.update(leg_time+700ms, final_position);
            (void)staged.update(leg_time+2700ms, final_position);
        }
        if (!require(staged.complete(), "fifth unload batch completes")) return 1;
        staged.reset(); (void)staged.update(now, origin); (void)staged.update(now+2s, origin);
        bool timed_out = false;
        try { (void)staged.update(now+6s, origin); } catch (const std::runtime_error&) { timed_out = true; }
        if (!require(timed_out && staged.command().forward_mps == 0, "stalled advance aborts")) return 1;
        robot::SearchController search;
        if (!require(search.collectibles_allowed(), "initial collection enabled")) return 1;
        search.begin_return_home();
        if (!require(!search.collectibles_allowed(), "return sequence blocks collection")) return 1;
        search.reset();
        if (!require(search.collectibles_allowed(), "new mission restores collection")) return 1;
    }
    robot::LocalizationState localization{.timestamp = now, .pose = {}, .position_sigma_m = .05,
                                          .globally_localized = true};
    robot::SafetySupervisor safety;
    auto result = safety.evaluate({1, 0, 4}, localization, now, false);
    if (!require(result.limited, "overspeed command is limited") ||
        !require(result.command.forward_mps == .45, "linear speed clamp") ||
        !require(result.command.yaw_radps == 2.0, "yaw speed clamp")) return 1;
    const robot::Twist2 transport_limited = robot::clamp_twist({.forward_mps = .8,
                                                                  .left_mps = -.6,
                                                                  .yaw_radps = 3},
                                                                 .45, 2);
    if (!require(std::abs(std::hypot(transport_limited.forward_mps,
                                     transport_limited.left_mps) - .45) < 1e-9,
                 "transport clamp preserves the calibrated linear envelope") ||
        !require(transport_limited.yaw_radps == 2,
                 "transport clamp preserves the calibrated yaw envelope")) return 1;
    const robot::Twist2 invalid_transport = robot::clamp_twist(
        {.forward_mps = std::numeric_limits<double>::quiet_NaN(), .left_mps = 0,
         .yaw_radps = 0}, .45, 2);
    if (!require(invalid_transport.forward_mps == 0 && invalid_transport.left_mps == 0 &&
                     invalid_transport.yaw_radps == 0,
                 "non-finite transport command becomes a safe stop")) return 1;
    result = safety.evaluate({.1, 0, 0}, localization, now + 301ms, false);
    if (!require(result.stopped, "stale localization stops motion")) return 1;

    robot::ApproachController approach;
    robot::TrackedObject target{.id = 1, .object_class = robot::ObjectClass::red_cube,
                                .x_m = 1, .y_m = 0, .last_seen = now};
    auto approach_result = approach.update({}, target, now, .1);
    if (!require(approach_result.target_valid, "fresh approach target accepted") ||
        !require(approach_result.command.forward_mps > 0,
                 "target ahead commands forward motion") ||
        !require(approach_result.command.left_mps == 0,
                 "one-centimeter intake offset remains inside lateral PID deadband")) return 1;
    robot::ApproachController constrained_approach({
        .translation_kp = 4.0,
        .lateral_kp = 4.0,
        .yaw_kp = 4.0,
        .maximum_linear_mps = .8,
        .maximum_yaw_radps = .8,
        .maximum_linear_accel_mps2 = 100.0,
        .maximum_yaw_accel_radps2 = 100.0,
        .alignment_enter_yaw_rad = 10.0,
        .alignment_exit_yaw_rad = 10.0,
    });
    robot::TrackedObject constrained_target{.id = 7,
                                             .object_class = robot::ObjectClass::yellow_cylinder,
                                             .x_m = 1.0,
                                             .y_m = .5,
                                             .last_seen = now};
    const auto constrained_result = constrained_approach.update({}, constrained_target, now, .1);
    const double constrained_demand = std::hypot(
        std::hypot(constrained_result.command.forward_mps, constrained_result.command.left_mps) / .8,
        std::abs(constrained_result.command.yaw_radps) / .8);
    if (!require(constrained_demand <= 1.0 + 1e-9 &&
                     constrained_result.command.forward_mps < .8 &&
                     std::abs(constrained_result.command.yaw_radps) < .8,
                 "approach shares the motion envelope between translation and yaw")) return 1;
    approach.reset();
    target.x_m = 0;
    target.y_m = 1;
    approach_result = approach.update({}, target, now, .1);
    if (!require(approach_result.aligning && approach_result.command.forward_mps == 0 &&
                     approach_result.command.left_mps == 0,
                 "target left first enters yaw alignment") ||
        !require(approach_result.command.yaw_radps > 0,
                 "target left commands counterclockwise alignment rotation")) return 1;
    approach_result = approach.update({}, target, now + 301ms, .1);
    if (!require(!approach_result.target_valid &&
                     approach_result.command.forward_mps == 0 &&
                     approach_result.command.left_mps == 0,
                 "stale target stops approach")) return 1;
    target.x_m = .19;
    target.y_m = .01;
    target.last_seen = now;
    approach_result = approach.update({}, target, now, .1);
    if (!require(approach_result.target_valid && !approach_result.target_reached &&
                 approach_result.command.forward_mps > 0 &&
                 approach_result.command.left_mps == 0,
                 "target crossing intake plane starts the dash without yaw settling")) return 1;
    robot::ApproachController lateral_deadband_approach({
        .maximum_linear_accel_mps2 = 100.0,
        .maximum_yaw_accel_radps2 = 100.0,
    });
    robot::TrackedObject lateral_deadband_target{.id = 10,
                                                  .object_class = robot::ObjectClass::red_cube,
                                                  .x_m = .5,
                                                  .y_m = .04,
                                                  .last_seen = now};
    const auto lateral_deadband_result = lateral_deadband_approach.update(
        {}, lateral_deadband_target, now, .1);
    if (!require(!lateral_deadband_result.aligning &&
                 lateral_deadband_result.command.yaw_radps == 0,
                 "lateral intake deadband suppresses yaw correction")) return 1;
    target.last_seen = now + 100ms;
    approach_result = approach.update({}, target, now + 100ms, .1);
    if (!require(approach_result.target_valid && approach_result.command.forward_mps > 0 &&
                 approach_result.command.left_mps == 0,
                 "capture dash continues without a yaw-settle pause")) return 1;
    target.last_seen = now + 800ms;
    approach_result = approach.update({}, target, now + 800ms, .1);
    if (!require(approach_result.target_valid && approach_result.command.forward_mps > 0,
                 "capture dash remains active while the target is visible")) return 1;
    approach_result = approach.continue_capture({}, now + 1001ms, .1);
    if (!require(approach_result.target_valid && approach_result.command.forward_mps > 0,
                 "lost close target drives capture finish")) return 1;
    approach_result = approach.continue_capture({.x_m = .29}, now + 1400ms, .1);
    if (!require(approach_result.target_valid && !approach_result.target_reached,
                 "capture finish remains active below 0.3m")) return 1;
    approach_result = approach.continue_capture({.x_m = .31}, now + 2200ms, .1);
    if (!require(approach_result.target_reached && approach_result.command.forward_mps == 0,
                 "capture finish stops after 0.3m")) return 1;

    robot::ApproachController close_approach;
    robot::TrackedObject close_target{.id = 9, .object_class = robot::ObjectClass::yellow_cylinder,
                                      .x_m = .15, .y_m = .01, .last_seen = now};
    const auto close_result = close_approach.update({}, close_target, now, .1);
    if (!require(close_result.capturing && close_result.command.forward_mps > 0,
                 "a laterally aligned target inside 0.2m enters capture")) return 1;

    robot::WallContactDetector wall_contact({.impact_threshold_g = .35,
                                             .reverse_arm_time = 300ms,
                                             .maximum_imu_age = 150ms});
    if (!require(!wall_contact.update(false, 0, 10ms, now) &&
                     !wall_contact.update(true, -.1, 10ms, now + 100ms) &&
                     !wall_contact.update(true, .5, 10ms, now + 350ms) &&
                     wall_contact.update(true, .5, 10ms, now + 450ms) &&
                     !wall_contact.update(true, .5, 10ms, now + 500ms),
                 "IMU impact is armed only after sustained reverse motion")) return 1;
    if (!require(!wall_contact.update(false, 0, 10ms, now + 600ms) &&
                     !wall_contact.update(true, .5, 200ms, now + 1100ms),
                 "stale IMU sample cannot report wall contact")) return 1;

    robot::WorldModel world;
    world.replace_objects({
        {.id = 2, .object_class = robot::ObjectClass::yellow_cylinder,
         .x_m = 1.0, .y_m = 0.0},
        {.id = 3, .object_class = robot::ObjectClass::red_cube,
         .x_m = .4, .y_m = 0.0},
        {.id = 4, .object_class = robot::ObjectClass::opponent_robot,
         .x_m = .1, .y_m = 0.0},
    });
    const auto nearest = world.nearest_collectible({});
    if (!require(nearest && nearest->id == 3,
                 "nearest collectible ignores opponent") ||
        !require(world.collectible_by_id(3).has_value() && !world.collectible_by_id(4).has_value(),
                 "only collectable IDs can be held as an approach target")) return 1;

    const auto corners = robot::footprint_corners({.x_m = .38, .y_m = .25});
    if (!require(std::abs(corners[0].x_m - .10) < 1e-9 &&
                     std::abs(corners[0].y_m - .10) < 1e-9 &&
                     std::abs(corners[2].x_m - .40) < 1e-9 &&
                     std::abs(corners[2].y_m - .30) < 1e-9,
                 "camera-centered footprint preserves physical corners") ||
        !require(robot::footprint_inside_arena({.x_m = .38, .y_m = .25}),
                 "footprint with margin is accepted") ||
        !require(!robot::footprint_inside_arena({.x_m = .20, .y_m = .25}),
                 "rear corner crossing margin is rejected")) return 1;
    const robot::Twist2 camera_twist = robot::camera_origin_twist(
        {.forward_mps = 0, .left_mps = 0, .yaw_radps = 1});
    if (!require(std::abs(camera_twist.forward_mps + .05) < 1e-9 &&
                     std::abs(camera_twist.left_mps - .13) < 1e-9,
                 "turning applies camera lever-arm velocity")) return 1;

    robot::WorldModel tracked_world;
    auto tracked_target = target;
    tracked_target.last_seen = now;
    tracked_world.update_objects({tracked_target}, now);
    tracked_world.update_objects({}, now + 100ms);
    if (!require(tracked_world.nearest_collectible({}).has_value(),
                 "one missing detector frame keeps collectible track")) return 1;
    tracked_world.update_objects({}, now + 600ms);
    if (!require(!tracked_world.nearest_collectible({}).has_value(),
                 "stale collectible track expires")) return 1;

    robot::LocalTargetTracker local_tracker({.memory = 3s, .acquisition_confirmation = 0ms,
                                             .minimum_observations = 1,
                                             .collection_suppression = 3s,
                                             .association_gate_m = .75,
                                             .measurement_gain = .55});
    robot::TrackedObject local_observation{
        .object_class = robot::ObjectClass::yellow_cylinder,
        .camera_forward_m = 1.0,
        .camera_left_m = .2,
        .confidence = .9F,
        .last_seen = now,
    };
    local_tracker.observe({local_observation}, {}, now);
    robot::TrackedObject second_local_observation{
        .object_class = robot::ObjectClass::red_cube,
        .camera_forward_m = 1.4,
        .camera_left_m = -.2,
        .confidence = .8F,
        .last_seen = now,
    };
    local_tracker.observe({second_local_observation}, {}, now);
    if (!require(local_tracker.tracks().size() == 2,
                 "local tracker retains more than the active collectible")) return 1;
    const auto selected_local = local_tracker.acquire_nearest({}, now);
    if (!require(selected_local && selected_local->object_class == robot::ObjectClass::yellow_cylinder,
                 "local tracker selects nearest fresh collectible")) return 1;
    const auto propagated = local_tracker.target({.x_m = .3}, now + 1s);
    if (!require(propagated && std::abs(propagated->camera_forward_m - .7) < 1e-9 &&
                     std::abs(propagated->camera_left_m - .2) < 1e-9,
                 "local target propagates a static object through odometry")) return 1;
    local_tracker.mark_collected(now + 100ms);
    if (!require(local_tracker.tracks().size() == 1 &&
                     local_tracker.acquire_nearest({}, now + 100ms) &&
                     local_tracker.acquire_nearest({}, now + 100ms)->object_class ==
                         robot::ObjectClass::red_cube,
                 "collecting one target retains the other local track")) return 1;
    const robot::TrackedObject outlier{
        .object_class = robot::ObjectClass::yellow_cylinder,
        .camera_forward_m = 4.0,
        .camera_left_m = .2,
        .confidence = .9F,
        .last_seen = now + 1s,
    };
    local_tracker.observe({outlier}, {.x_m = .3}, now + 1s);
    if (!require(local_tracker.target({.x_m = .3}, now + 3001ms) == std::nullopt,
                 "local target expires after three seconds without a detection")) return 1;

    robot::LocalTargetTracker expired_tracker({.memory = 3s, .acquisition_confirmation = 0ms,
                                               .minimum_observations = 1,
                                               .association_gate_m = .75,
                                               .measurement_gain = .55});
    expired_tracker.observe({local_observation}, {}, now);
    robot::TrackedObject red_observation{
        .object_class = robot::ObjectClass::red_cube,
        .camera_forward_m = .8,
        .camera_left_m = -.1,
        .confidence = .9F,
        .last_seen = now + 4s,
    };
    expired_tracker.observe({red_observation}, {}, now + 4s);
    const auto reacquired = expired_tracker.acquire_nearest({}, now + 4s);
    if (!require(reacquired && reacquired->object_class == robot::ObjectClass::red_cube,
                 "expired target does not reject a newly visible collectible class")) return 1;

    local_tracker.observe({local_observation}, {}, now + 4s);
    local_tracker.mark_collected(now + 4s);
    local_tracker.observe({local_observation}, {}, now + 5s);
    if (!require(!local_tracker.target({}, now + 5s),
                 "recently collected target is suppressed through stale detections")) return 1;
    local_tracker.observe({local_observation}, {}, now + 8s);
    if (!require(local_tracker.acquire_nearest({}, now + 8s).has_value(),
                 "suppression expires after local target memory interval")) return 1;

    robot::LocalTargetTracker guarded_tracker({.memory = 3s, .acquisition_confirmation = 200ms,
                                               .confirmation_gap = 150ms,
                                               .minimum_observations = 3,
                                               .collection_suppression = 8s,
                                               .association_gate_m = .75,
                                               .measurement_gain = .55});
    guarded_tracker.observe({local_observation}, {}, now);
    // One or two nearby false boxes must not stop the no-object search.
    guarded_tracker.observe({local_observation}, {}, now + 100ms);
    if (!require(!guarded_tracker.acquire_nearest({}, now + 100ms),
                 "short detector flash cannot acquire a target")) return 1;
    guarded_tracker.observe({local_observation}, {}, now + 220ms);
    if (!require(guarded_tracker.acquire_nearest({}, now + 220ms).has_value(),
                 "persistent detector evidence acquires a target")) return 1;
    guarded_tracker.mark_collected(now + 220ms);
    guarded_tracker.observe({local_observation}, {}, now + 400ms);
    guarded_tracker.observe({local_observation}, {}, now + 620ms);
    guarded_tracker.observe({local_observation}, {}, now + 840ms);
    if (!require(!guarded_tracker.acquire_nearest({}, now + 840ms),
                 "completed capture suppresses stale rediscovery")) return 1;
    guarded_tracker.observe({local_observation}, {}, now + 9s);
    guarded_tracker.observe({local_observation}, {}, now + 9100ms);
    guarded_tracker.observe({local_observation}, {}, now + 9200ms);
    if (!require(guarded_tracker.acquire_nearest({}, now + 9200ms).has_value(),
                 "collection suppression eventually expires")) return 1;

    const cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    const robot::GroundProjector projector(camera_matrix, 1.0, 45.0);
    robot::DetectionFrame detections{
        .timestamp = now,
        .frame_sequence = 7,
        .detections = {{.object_class = robot::ObjectClass::yellow_cylinder,
                        .confidence = .9F,
                        .box = {.left = -.1F, .top = -.2F, .right = .1F, .bottom = 0}}},
    };
    const auto projected = robot::project_collectibles(
        detections, projector, {.x_m = .5, .y_m = .5, .yaw_rad = CV_PI / 2});
    if (!require(projected.size() == 1, "collectible projects onto arena") ||
        !require(std::abs(projected[0].x_m - .5) < 1e-6 &&
                     std::abs(projected[0].y_m - 1.5) < 1e-6,
                 "camera-relative target transforms to arena frame") ||
        !require(std::abs(projected[0].camera_forward_m - 1.0) < 1e-6 &&
                     std::abs(projected[0].camera_left_m) < 1e-6,
                 "collectible retains direct camera-relative control vector")) return 1;
    const auto beyond_navigation_envelope = robot::project_collectibles(
        detections, projector, {.x_m = 20.0, .y_m = -20.0, .yaw_rad = 0});
    if (!require(beyond_navigation_envelope.size() == 1,
                 "collectible outside arena and navigation envelope remains accepted")) return 1;
    detections.detections.push_back({.object_class = robot::ObjectClass::opponent_robot,
                                     .confidence = .9F,
                                     .box = {.left = -.05F, .top = -.1F,
                                             .right = .1F, .bottom = .1F}});
    if (!require(robot::project_collectibles(
                     detections, projector, {.x_m = .5, .y_m = .5, .yaw_rad = CV_PI / 2}).empty(),
                 "collectible overlapping opponent is not a drive target")) return 1;

    const auto deduplicated = robot::deduplicate_same_class_detections({
        {.object_class = robot::ObjectClass::yellow_cylinder, .confidence = .91F,
         .box = {.left = 10, .top = 10, .right = 30, .bottom = 30}},
        {.object_class = robot::ObjectClass::yellow_cylinder, .confidence = .70F,
         .box = {.left = 12, .top = 12, .right = 32, .bottom = 32}},
        {.object_class = robot::ObjectClass::yellow_cylinder, .confidence = .80F,
         .box = {.left = 50, .top = 10, .right = 70, .bottom = 30}},
        {.object_class = robot::ObjectClass::red_cube, .confidence = .60F,
         .box = {.left = 12, .top = 12, .right = 32, .bottom = 32}},
    });
    if (!require(deduplicated.size() == 3 && deduplicated[0].confidence == .91F,
                 "same-class overlapping detections collapse to highest confidence") ||
        !require(std::any_of(deduplicated.begin(), deduplicated.end(),
                             [](const robot::Detection& detection) {
                                 return detection.object_class == robot::ObjectClass::red_cube;
                             }),
                 "different classes remain available for overlap safety filtering")) return 1;

    robot::SearchController search;
    auto search_result = search.update({.x_m = .2, .y_m = .2}, false, now, .1);
    if (!require(search_result.phase == robot::SearchPhase::rotate_local &&
                     search_result.command.yaw_radps > 0,
                 "lost target starts local rotation")) return 1;
    search_result = search.update({.x_m = .2, .y_m = .2}, true, now + 1s, .1);
    search_result = search.update({.x_m = .2, .y_m = .2}, false, now + 1100ms, .1);
    if (!require(search_result.phase == robot::SearchPhase::rotate_local &&
                 search_result.lost_seconds >= 1.0,
                 "brief target track does not reset search timeout")) return 1;
    search_result = search.update({.x_m = .2, .y_m = .2}, true, now + 2s, .1);
    search_result = search.update({.x_m = .2, .y_m = .2}, true, now + 4100ms, .1);
    search_result = search.update({.x_m = .2, .y_m = .2}, false, now + 4200ms, .1);
    if (!require(search_result.phase == robot::SearchPhase::rotate_local &&
                 search_result.lost_seconds < .01,
                 "two-second target track resets search timeout")) return 1;
    search_result = search.update({.x_m = .2, .y_m = .2}, false, now + 10s, .1);
    if (!require(search_result.phase == robot::SearchPhase::navigate_center &&
                     std::hypot(search_result.command.forward_mps,
                                search_result.command.left_mps) > 0,
                 "five-second loss navigates to center")) return 1;
    search_result = search.update({.x_m = 1.5, .y_m = .9925}, false, now + 11s, .1);
    if (!require(search_result.phase == robot::SearchPhase::rotate_center,
                 "search rotates after reaching center")) return 1;
    search_result = search.update({.x_m = 1.76, .y_m = .9925}, false, now + 12s, .1);
    if (!require(search_result.phase == robot::SearchPhase::rotate_center,
                 "center exit hysteresis tolerates localization noise")) return 1;
    search_result = search.update({.x_m = 1.86, .y_m = .9925}, false, now + 13s, .1);
    if (!require(search_result.phase == robot::SearchPhase::navigate_center,
                 "center search resumes translation outside exit tolerance")) return 1;
    search_result = search.update({.x_m = .2, .y_m = .2}, false, now + 14s, .1, false);
    if (!require(search_result.phase == robot::SearchPhase::hold_for_localization &&
                 search_result.command.forward_mps == 0 && search_result.command.left_mps == 0,
                 "uncertain localization never permits center translation")) return 1;
    search_result = search.update({.x_m = 1.5, .y_m = .9925}, false, now + 15s, .1);
    if (!require(search_result.phase == robot::SearchPhase::rotate_center,
                 "center dwell begins after returning to the center")) return 1;
    search_result = search.update({.x_m = 1.5, .y_m = .9925}, false, now + 19s, .1);
    if (!require(search_result.phase == robot::SearchPhase::rotate_center,
                 "center travel does not consume the center-search dwell")) return 1;
    search_result = search.update({.x_m = 1.5, .y_m = .9925}, false, now + 20s, .1);
    if (!require(search_result.phase == robot::SearchPhase::navigate_cargo_calibration,
                 "completed center search enters cargo calibration before home")) return 1;

    robot::ServoUnloadController unload({.pulse_us = {1600, 1600, 1800, 2000},
                                         .duration_ms = {3000, 1000, 500, 0}});
    if (!require(unload.update(now) == std::optional<int>(2000),
                 "unload begins from the configured resting close pulse") ||
        !require(unload.update(now + 1500ms) == std::optional<int>(1800),
                 "unload linearly opens across its first segment") ||
        !require(unload.update(now + 3000ms) == std::optional<int>(1600),
                 "unload reaches the fully open pulse") ||
        !require(!unload.update(now + 3999ms),
                 "repeated open target holds without redundant UART commands") ||
        !require(unload.update(now + 4250ms) == std::optional<int>(1700),
                 "unload interpolates the quick intermediate segment") ||
        !require(unload.update(now + 4500ms) == std::optional<int>(2000) && unload.complete(),
                 "unload closes at the final sequence target")) return 1;

    robot::SoloMission terminal_mission;
    terminal_mission.start();
    if (!require(terminal_mission.update({.search_complete = true}) == robot::MissionState::unload &&
                     terminal_mission.update({.unload_complete = true}) == robot::MissionState::search_target,
                 "completed return-home unloads before restarting search")) return 1;

    robot::SearchController return_home({.center_x_m = 1.5, .center_y_m = .9925,
                                         .center_entry_radius_m = .25,
                                         .home_x_m = .10, .home_y_m = .15,
                                         .home_stop_radius_m = .20,
                                         .cargo_calibration_x_m = 1.0,
                                         .cargo_calibration_y_m = 1.0,
                                         .cargo_calibration_stop_radius_m = .15,
                                         .cargo_calibration_steps = {
                                             {.forward_mps = -1.0, .timeout_s = 1.5,
                                              .until_imu_detect = true},
                                             {.forward_mps = .30, .timeout_s = 2.0},
                                             {.forward_mps = -1.0, .timeout_s = 1.0,
                                              .until_imu_detect = true}}});
    return_home.begin_return_home();
    search_result = return_home.update({.x_m = .3, .y_m = 1.0}, true, now, .1);
    if (!require(search_result.phase == robot::SearchPhase::navigate_cargo_calibration &&
                     std::hypot(search_result.command.forward_mps,
                                search_result.command.left_mps) > 0,
                 "Home + Exit begins at the cargo calibration waypoint")) return 1;
    search_result = return_home.update({.x_m = 1.0, .y_m = 1.0, .yaw_rad = .2}, true, now + 1s, .1);
    search_result = return_home.update({.x_m = 1.0, .y_m = 1.0, .yaw_rad = .2}, true, now + 1100ms, .1);
    if (!require(search_result.phase == robot::SearchPhase::cargo_calibration_align &&
                     search_result.command.yaw_radps < 0,
                 "cargo calibration aligns yaw at its waypoint")) return 1;
    search_result = return_home.update({.x_m = 1.0, .y_m = 1.0, .yaw_rad = 0}, true, now + 2s, .1);
    search_result = return_home.update({.x_m = 1.0, .y_m = 1.0, .yaw_rad = 0}, true, now + 2100ms, .1);
    if (!require(search_result.phase == robot::SearchPhase::cargo_calibration_reverse_fast &&
                     search_result.command.forward_mps < 0,
                 "cargo calibration starts its fast reverse leg")) return 1;
    if (!require(return_home.report_cargo_wall_hit(now + 2200ms),
                 "IMU wall contact advances a tagged cargo step")) return 1;
    search_result = return_home.update({.x_m = 1.0, .y_m = 1.0, .yaw_rad = 0}, true, now + 2300ms, .1);
    if (!require(search_result.phase == robot::SearchPhase::cargo_calibration_forward_slow,
                 "cargo calibration enters its slow forward leg after reverse")) return 1;
    search_result = return_home.update({.x_m = 1.0, .y_m = 1.0, .yaw_rad = 0}, true, now + 4400ms, .1);
    search_result = return_home.update({.x_m = 1.0, .y_m = 1.0, .yaw_rad = 0}, true, now + 4500ms, .1);
    if (!require(search_result.phase == robot::SearchPhase::cargo_calibration_reverse_final,
                 "cargo calibration enters its final reverse leg")) return 1;
    if (!require(return_home.report_cargo_wall_hit(now + 4600ms),
                 "IMU wall contact completes the final cargo reverse leg")) return 1;
    search_result = return_home.update({.x_m = 1.5, .y_m = .9925, .yaw_rad = 0}, true, now + 4700ms, .1);
    search_result = return_home.update({.x_m = .10, .y_m = .15, .yaw_rad = 0}, true, now + 4800ms, .1);
    if (!require(search_result.phase == robot::SearchPhase::post_home_moonwalk,
                 "cargo calibration returns through center and enters home moonwalk")) return 1;
    search_result = return_home.update({.x_m = .10, .y_m = .15,
                                         .yaw_rad = -145. * std::numbers::pi / 180.},
                                       true, now + 4900ms, .1);
    if (!require(search_result.phase == robot::SearchPhase::post_home_turn,
                 "moonwalk transitions to the final absolute yaw turn")) return 1;
    search_result = return_home.update({.x_m = .10, .y_m = .15,
                                         .yaw_rad = 145. * std::numbers::pi / 180.},
                                       true, now + 5s, .1);
    if (!require(search_result.phase == robot::SearchPhase::post_home_reverse,
                 "final turn accepts the configured positive 145-degree yaw")) return 1;
    search_result = return_home.update({.x_m = .10, .y_m = .15,
                                         .yaw_rad = 145. * std::numbers::pi / 180.},
                                       true, now + 7100ms, .1);
    if (!require(search_result.phase == robot::SearchPhase::complete,
                 "home and exit path completes after its configured reverse leg")) return 1;

    robot::DetectionFrame home_frame{
        .timestamp = now,
        .frame_sequence = 8,
        .detections = {{.object_class = robot::ObjectClass::home,
                        .confidence = .9F,
                        .box = {.left = -.1F, .top = -.2F, .right = .1F, .bottom = 0}}},
    };
    const robot::GroundProjector near_projector(camera_matrix, .1, 45.0);
    const auto home = robot::check_home_box(home_frame, near_projector, {});
    if (!require(home.detected && home.consistent,
                 "boxed home agrees with localized home rectangle")) return 1;

    robot::SoloMission mission;
    mission.start();
    if (!require(mission.state() == robot::MissionState::search_target,
                 "start enters target search") ||
        !require(mission.update({.target_active = true, .aligning = true}) ==
                     robot::MissionState::align_target,
                 "search to target alignment") ||
        !require(mission.update({.target_active = true}) == robot::MissionState::approach_target,
                 "alignment to approach") ||
        !require(mission.update({.target_active = true, .capturing = true}) ==
                     robot::MissionState::capture_target,
                 "approach to capture") ||
        !require(mission.update({.capture_complete = true}) == robot::MissionState::search_target,
                 "capture returns to a fresh search") ||
        !require(mission.update({.fault = true}) == robot::MissionState::safe_stop,
                 "fault to safe stop")) return 1;
}
