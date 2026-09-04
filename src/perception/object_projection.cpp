#include "robot/perception/object_projection.hpp"

#include <algorithm>
#include <cmath>

namespace robot {
namespace {

bool project_box_center(const Detection& detection, const GroundProjector& projector,
                        const Pose2& pose, cv::Point2d& arena) {
    if (detection.box.right <= detection.box.left || detection.box.bottom <= detection.box.top) {
        return false;
    }
    cv::Point2d relative;
    if (!projector.project({.5F * (detection.box.left + detection.box.right),
                            .5F * (detection.box.top + detection.box.bottom)}, relative)) return false;
    const double cosine = std::cos(pose.yaw_rad);
    const double sine = std::sin(pose.yaw_rad);
    arena = {pose.x_m + cosine * relative.x - sine * relative.y,
             pose.y_m + sine * relative.x + cosine * relative.y};
    return true;
}

double rectangle_distance(double x, double y) {
    const double dx = std::max({0.0, -x, x - .2});
    const double dy = std::max({0.0, -y, y - .3});
    return std::hypot(dx, dy);
}

}  // namespace

std::vector<TrackedObject> project_collectibles(
    const DetectionFrame& frame, const GroundProjector& projector,
    const Pose2& robot_pose, ObjectProjectionLimits limits) {
    std::vector<TrackedObject> objects;
    for (std::size_t index = 0; index < frame.detections.size(); ++index) {
        const Detection& detection = frame.detections[index];
        if ((detection.object_class != ObjectClass::yellow_cylinder &&
             detection.object_class != ObjectClass::red_cube) ||
            detection.confidence < limits.minimum_confidence) continue;

        const cv::Point2f contact_pixel{
            .5F * (detection.box.left + detection.box.right), detection.box.bottom};
        cv::Point2d relative;
        if (!projector.project(contact_pixel, relative)) continue;
        const double range = std::hypot(relative.x, relative.y);
        if (range < limits.minimum_range_m || range > limits.maximum_range_m) continue;

        const double cosine = std::cos(robot_pose.yaw_rad);
        const double sine = std::sin(robot_pose.yaw_rad);
        const double x = robot_pose.x_m + cosine * relative.x - sine * relative.y;
        const double y = robot_pose.y_m + sine * relative.x + cosine * relative.y;
        if (x < limits.arena_margin_m || x > limits.arena_length_m - limits.arena_margin_m ||
            y < limits.arena_margin_m || y > limits.arena_width_m - limits.arena_margin_m) continue;

        objects.push_back({
            .id = (frame.frame_sequence << 16U) | static_cast<std::uint64_t>(index),
            .object_class = detection.object_class,
            .x_m = x,
            .y_m = y,
            .uncertainty_m = std::clamp(.02 + .04 * range, .02, .20),
            .confidence = detection.confidence,
            .last_seen = frame.timestamp,
        });
    }
    return objects;
}

HomeObservation check_home_box(const DetectionFrame& frame,
                               const GroundProjector& projector,
                               const Pose2& robot_pose, float minimum_confidence,
                               double tolerance_m) {
    HomeObservation best;
    for (const Detection& detection : frame.detections) {
        if (detection.object_class != ObjectClass::home ||
            detection.confidence < minimum_confidence || detection.confidence < best.confidence) continue;
        cv::Point2d arena;
        if (!project_box_center(detection, projector, robot_pose, arena)) continue;
        best.detected = true;
        best.x_m = arena.x;
        best.y_m = arena.y;
        best.distance_to_home_m = rectangle_distance(arena.x, arena.y);
        best.consistent = best.distance_to_home_m <= tolerance_m;
        best.confidence = detection.confidence;
    }
    return best;
}

}  // namespace robot
