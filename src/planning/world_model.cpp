#include "robot/planning/world_model.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace robot {

void WorldModel::replace_objects(std::vector<TrackedObject> objects) {
    objects_ = std::move(objects);
}

void WorldModel::update_objects(std::vector<TrackedObject> observations, Timestamp now) {
    constexpr double kAssociationDistanceM = .25;
    constexpr auto kTrackLifetime = std::chrono::milliseconds(500);
    std::vector<bool> matched(objects_.size(), false);
    for (TrackedObject& observation : observations) {
        std::size_t best_index = objects_.size();
        double best_distance = kAssociationDistanceM;
        for (std::size_t index = 0; index < objects_.size(); ++index) {
            if (matched[index] || objects_[index].object_class != observation.object_class) continue;
            const double distance = std::hypot(objects_[index].x_m - observation.x_m,
                                               objects_[index].y_m - observation.y_m);
            if (distance < best_distance) {
                best_distance = distance;
                best_index = index;
            }
        }
        if (best_index == objects_.size()) {
            observation.id = next_track_id_++;
            objects_.push_back(observation);
            matched.push_back(true);
            continue;
        }
        TrackedObject& track = objects_[best_index];
        const double alpha = std::clamp(.25 + .50 * observation.confidence, .25, .75);
        track.x_m += alpha * (observation.x_m - track.x_m);
        track.y_m += alpha * (observation.y_m - track.y_m);
        track.uncertainty_m = std::max(observation.uncertainty_m,
                                       (1.0 - alpha) * track.uncertainty_m);
        track.confidence = observation.confidence;
        track.last_seen = observation.last_seen;
        matched[best_index] = true;
    }
    std::erase_if(objects_, [&](const TrackedObject& track) {
        return track.last_seen == Timestamp{} || now < track.last_seen ||
               now - track.last_seen > kTrackLifetime;
    });
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
