#pragma once

#include <vector>

#include "robot/localization/location.hpp"
#include "robot/planning/world_model.hpp"

namespace robot {

struct ObjectProjectionLimits {
    float minimum_confidence = .35F;
};

struct HomeObservation {
    bool detected = false;
    bool consistent = false;
    double x_m = 0;
    double y_m = 0;
    double distance_to_home_m = 0;
    float confidence = 0;
};

std::vector<TrackedObject> project_collectibles(
    const DetectionFrame& frame, const GroundProjector& projector,
    const Pose2& robot_pose, ObjectProjectionLimits limits = {});

HomeObservation check_home_box(const DetectionFrame& frame,
                               const GroundProjector& projector,
                               const Pose2& robot_pose,
                               float minimum_confidence = .35F,
                               double tolerance_m = .20);

}  // namespace robot
