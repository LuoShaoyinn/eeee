#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "robot/core/types.hpp"
#include "robot/perception/detections.hpp"

namespace robot {

struct TrackedObject {
    std::uint64_t id = 0;
    ObjectClass object_class{};
    double x_m = 0;
    double y_m = 0;
    double uncertainty_m = 1;
    float confidence = 0;
    Timestamp last_seen{};
};

class WorldModel {
public:
    void replace_objects(std::vector<TrackedObject> objects);
    const std::vector<TrackedObject>& objects() const;
    std::optional<TrackedObject> nearest_collectible(const Pose2& pose) const;

private:
    std::vector<TrackedObject> objects_;
};

}  // namespace robot
