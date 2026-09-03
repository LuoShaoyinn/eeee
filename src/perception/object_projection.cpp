#include "robot/perception/object_projection.hpp"

#include <algorithm>
#include <cmath>

namespace robot {

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

}  // namespace robot
