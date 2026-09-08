#include "robot/planning/vehicle_geometry.hpp"

#include <cmath>

namespace robot {

std::array<Pose2, 4> footprint_corners(const Pose2& camera_pose,
                                       const VehicleGeometry& geometry) {
    const double cosine = std::cos(camera_pose.yaw_rad);
    const double sine = std::sin(camera_pose.yaw_rad);
    const auto transform = [&](double x, double y) {
        return Pose2{.x_m = camera_pose.x_m + cosine * x - sine * y,
                     .y_m = camera_pose.y_m + sine * x + cosine * y,
                     .yaw_rad = camera_pose.yaw_rad};
    };
    return {transform(geometry.rear_x_m, geometry.right_y_m),
            transform(geometry.front_x_m, geometry.right_y_m),
            transform(geometry.front_x_m, geometry.left_y_m),
            transform(geometry.rear_x_m, geometry.left_y_m)};
}

bool footprint_inside_arena(const Pose2& camera_pose,
                            const VehicleGeometry& vehicle,
                            const ArenaGeometry& arena) {
    constexpr double kGeometryEpsilonM = 1e-9;
    for (const Pose2& corner : footprint_corners(camera_pose, vehicle)) {
        if (corner.x_m < arena.fence_margin_m - kGeometryEpsilonM ||
            corner.x_m > arena.length_m - arena.fence_margin_m + kGeometryEpsilonM ||
            corner.y_m < arena.fence_margin_m - kGeometryEpsilonM ||
            corner.y_m > arena.width_m - arena.fence_margin_m + kGeometryEpsilonM) return false;
    }
    return true;
}

Twist2 camera_origin_twist(const Twist2& center_twist,
                           const VehicleGeometry& geometry) {
    const double camera_x_from_center = -geometry.center_x_m;
    const double camera_y_from_center = -geometry.center_y_m;
    return {.forward_mps = center_twist.forward_mps -
                           center_twist.yaw_radps * camera_y_from_center,
            .left_mps = center_twist.left_mps +
                        center_twist.yaw_radps * camera_x_from_center,
            .yaw_radps = center_twist.yaw_radps};
}

}  // namespace robot
