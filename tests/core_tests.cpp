#include <chrono>
#include <iostream>

#include "robot/control/safety_supervisor.hpp"
#include "robot/perception/object_projection.hpp"
#include "robot/planning/approach_controller.hpp"
#include "robot/planning/mission.hpp"
#include "robot/planning/world_model.hpp"

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
