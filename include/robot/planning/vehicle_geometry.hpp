#pragma once

#include <array>

#include "robot/core/types.hpp"

namespace robot {

struct VehicleGeometry {
    // Body coordinates use +x forward and +y left. The pose origin is the
    // camera optical-axis projection on the ground.
    double rear_x_m = -.28;
    double front_x_m = .02;
    double right_y_m = -.15;
    double left_y_m = .05;
    double center_x_m = -.13;
    double center_y_m = -.05;
};

struct ArenaGeometry {
    double length_m = 3.0;
    double width_m = 1.985;
    double fence_margin_m = .10;
};

std::array<Pose2, 4> footprint_corners(const Pose2& camera_pose,
                                       const VehicleGeometry& geometry = {});
bool footprint_inside_arena(const Pose2& camera_pose,
                            const VehicleGeometry& vehicle = {},
                            const ArenaGeometry& arena = {});

// Convert velocity measured at the chassis center to velocity of the
// camera-centered pose used by localization and planning.
Twist2 camera_origin_twist(const Twist2& center_twist,
                           const VehicleGeometry& geometry = {});

}  // namespace robot
