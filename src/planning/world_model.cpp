#include "robot/planning/world_model.hpp"

#include <cmath>
#include <limits>

namespace robot {

void WorldModel::replace_objects(std::vector<TrackedObject> objects) {
    objects_ = std::move(objects);
}

const std::vector<TrackedObject>& WorldModel::objects() const { return objects_; }

std::optional<TrackedObject> WorldModel::nearest_collectible(const Pose2& pose) const {
    const TrackedObject* best = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto& object : objects_) {
        if (object.object_class != ObjectClass::yellow_cylinder &&
            object.object_class != ObjectClass::red_cube) continue;
        const double distance = std::hypot(object.x_m - pose.x_m, object.y_m - pose.y_m);
        if (distance < best_distance) {
            best = &object;
            best_distance = distance;
        }
    }
    return best == nullptr ? std::nullopt : std::optional<TrackedObject>(*best);
}

}  // namespace robot
