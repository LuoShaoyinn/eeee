#include <chrono>
#include <iostream>

#include "robot/control/safety_supervisor.hpp"
#include "robot/perception/object_projection.hpp"
#include "robot/planning/approach_controller.hpp"
#include "robot/planning/mission.hpp"
#include "robot/planning/search_controller.hpp"
#include "robot/planning/world_model.hpp"
#include "robot/planning/vehicle_geometry.hpp"

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

    robot::ApproachController approach;
    robot::TrackedObject target{.id = 1, .object_class = robot::ObjectClass::red_cube,
                                .x_m = 1, .y_m = 0, .last_seen = now};
    auto approach_result = approach.update({}, target, now, .1);
    if (!require(approach_result.target_valid, "fresh approach target accepted") ||
        !require(approach_result.command.forward_mps > 0,
                 "target ahead commands forward motion") ||
        !require(approach_result.command.left_mps == 0,
                 "target ahead does not command strafe")) return 1;
    approach.reset();
    target.x_m = 0;
    target.y_m = 1;
    approach_result = approach.update({}, target, now, .1);
    if (!require(approach_result.command.left_mps > 0,
                 "target left commands left strafe") ||
        !require(approach_result.command.yaw_radps > 0,
                 "target left commands counterclockwise rotation")) return 1;
    approach_result = approach.update({}, target, now + 301ms, .1);
    if (!require(!approach_result.target_valid &&
                     approach_result.command.forward_mps == 0 &&
                     approach_result.command.left_mps == 0,
                 "stale target stops approach")) return 1;
    target.x_m = .1;
    target.y_m = 0;
    target.last_seen = now;
    approach_result = approach.update({}, target, now, .1);
    if (!require(approach_result.target_reached,
                 "pickup-distance target stops approach")) return 1;

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
                 "nearest collectible ignores opponent")) return 1;

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
    tracked_world.update_objects({target}, now);
    tracked_world.update_objects({}, now + 100ms);
    if (!require(tracked_world.nearest_collectible({}).has_value(),
                 "one missing detector frame keeps collectible track")) return 1;
    tracked_world.update_objects({}, now + 600ms);
    if (!require(!tracked_world.nearest_collectible({}).has_value(),
                 "stale collectible track expires")) return 1;

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
                 "camera-relative target transforms to arena frame")) return 1;

    robot::SearchController search;
    auto search_result = search.update({.x_m = .2, .y_m = .2}, false, now, .1);
    if (!require(search_result.phase == robot::SearchPhase::rotate_local &&
                     search_result.command.yaw_radps > 0,
                 "lost target starts local rotation")) return 1;
    search_result = search.update({.x_m = .2, .y_m = .2}, false, now + 6s, .1);
    if (!require(search_result.phase == robot::SearchPhase::navigate_center &&
                     std::hypot(search_result.command.forward_mps,
                                search_result.command.left_mps) > 0,
                 "five-second loss navigates to center")) return 1;
    search_result = search.update({.x_m = 1.5, .y_m = .9925}, false, now + 7s, .1);
    if (!require(search_result.phase == robot::SearchPhase::rotate_center,
                 "search rotates after reaching center")) return 1;
    search_result = search.update({.x_m = 1.5, .y_m = .9925}, false, now + 11s, .1);
    if (!require(search_result.phase == robot::SearchPhase::return_home,
                 "ten-second loss returns home")) return 1;
    search_result = search.update({.x_m = .25, .y_m = .2}, false, now + 12s, .1);
    if (!require(search_result.phase == robot::SearchPhase::complete &&
                     search_result.command.forward_mps == 0,
                 "home radius permanently stops search")) return 1;
    search_result = search.update({.x_m = .25, .y_m = .2}, true, now + 13s, .1);
    if (!require(search_result.phase == robot::SearchPhase::complete &&
                     search_result.command.forward_mps == 0,
                 "completed search remains stopped after a detection")) return 1;

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
