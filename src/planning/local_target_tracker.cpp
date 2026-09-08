#include "robot/planning/local_target_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot {
namespace {

bool collectable(ObjectClass object_class) {
    return object_class == ObjectClass::yellow_cylinder ||
           object_class == ObjectClass::red_cube;
}

}  // namespace

LocalTargetTracker::LocalTargetTracker(LocalTargetTrackerConfig config) : config_(config) {}

std::optional<TrackedObject> LocalTargetTracker::predicted_track(
    const TrackedObject& track, const Pose2& odometry_pose) const {
    TrackedObject predicted = track;
    const double dx = predicted.x_m - odometry_pose.x_m;
    const double dy = predicted.y_m - odometry_pose.y_m;
    const double cosine = std::cos(odometry_pose.yaw_rad);
    const double sine = std::sin(odometry_pose.yaw_rad);
    predicted.camera_forward_m = cosine * dx + sine * dy;
    predicted.camera_left_m = -sine * dx + cosine * dy;
    return predicted;
}

void LocalTargetTracker::discard_expired(Timestamp now) {
    std::erase_if(tracks_, [&](const TrackedObject& track) {
        return track.last_seen == Timestamp{} || now < track.last_seen ||
               now - track.last_seen > config_.memory;
    });
    if (active_track_id_ && std::none_of(tracks_.begin(), tracks_.end(),
                                         [&](const TrackedObject& track) {
                                             return track.id == *active_track_id_;
                                         })) {
        active_track_id_.reset();
    }
}

void LocalTargetTracker::observe(const std::vector<TrackedObject>& observations,
                                 const Pose2& odometry_pose, Timestamp now) {
    discard_expired(now);
    for (const TrackedObject& observation : observations) {
        if (!collectable(observation.object_class)) continue;
        const double cosine = std::cos(odometry_pose.yaw_rad);
        const double sine = std::sin(odometry_pose.yaw_rad);
        const double measured_x = odometry_pose.x_m + cosine * observation.camera_forward_m -
                                  sine * observation.camera_left_m;
        const double measured_y = odometry_pose.y_m + sine * observation.camera_forward_m +
                                  cosine * observation.camera_left_m;
        if (suppressed_track_ && now < suppress_until_ &&
            observation.object_class == suppressed_track_->object_class &&
            std::hypot(measured_x - suppressed_track_->x_m,
                       measured_y - suppressed_track_->y_m) <= config_.association_gate_m) {
            continue;
        }
        std::size_t match = tracks_.size();
        double best_score = config_.association_gate_m;
        for (std::size_t index = 0; index < tracks_.size(); ++index) {
            const TrackedObject& track = tracks_[index];
            if (track.object_class != observation.object_class) continue;
            const double score = std::hypot(measured_x - track.x_m, measured_y - track.y_m);
            if (score < best_score) {
                match = index;
                best_score = score;
            }
        }
        if (match == tracks_.size()) {
            TrackedObject track = observation;
            track.id = next_track_id_++;
            track.x_m = measured_x;
            track.y_m = measured_y;
            track.last_seen = now;
            tracks_.push_back(track);
        } else {
            TrackedObject& track = tracks_[match];
            track.x_m += config_.measurement_gain * (measured_x - track.x_m);
            track.y_m += config_.measurement_gain * (measured_y - track.y_m);
            track.camera_forward_m = observation.camera_forward_m;
            track.camera_left_m = observation.camera_left_m;
            track.confidence = observation.confidence;
            track.uncertainty_m = observation.uncertainty_m;
            track.last_seen = now;
        }
    }
}

std::optional<TrackedObject> LocalTargetTracker::acquire_nearest(
    const Pose2& odometry_pose, Timestamp now) {
    discard_expired(now);
    if (active_track_id_) return target(odometry_pose, now);
    const TrackedObject* best = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const TrackedObject& track : tracks_) {
        if (now - track.last_seen > config_.acquisition_freshness) continue;
        const auto predicted = predicted_track(track, odometry_pose);
        const double distance = std::hypot(predicted->camera_forward_m, predicted->camera_left_m);
        if (distance < best_distance) {
            best = &track;
            best_distance = distance;
        }
    }
    if (!best) return std::nullopt;
    active_track_id_ = best->id;
    return predicted_track(*best, odometry_pose);
}

std::optional<TrackedObject> LocalTargetTracker::target(const Pose2& odometry_pose,
                                                         Timestamp now) const {
    if (!active_track_id_) return std::nullopt;
    for (const TrackedObject& track : tracks_) {
        if (track.id != *active_track_id_) continue;
        if (now < track.last_seen || now - track.last_seen > config_.memory) return std::nullopt;
        auto predicted = predicted_track(track, odometry_pose);
        predicted->last_seen = now;
        return predicted;
    }
    return std::nullopt;
}

void LocalTargetTracker::reset() {
    tracks_.clear();
    active_track_id_.reset();
    suppressed_track_.reset();
    suppress_until_ = {};
}

void LocalTargetTracker::mark_collected(Timestamp now) {
    if (active_track_id_) {
        const auto it = std::find_if(tracks_.begin(), tracks_.end(), [&](const TrackedObject& track) {
            return track.id == *active_track_id_;
        });
        if (it == tracks_.end()) return;
        suppressed_track_ = *it;
        suppress_until_ = now + config_.memory;
        tracks_.erase(it);
    }
    active_track_id_.reset();
}

void LocalTargetTracker::release_target() { active_track_id_.reset(); }

const std::vector<TrackedObject>& LocalTargetTracker::tracks() const { return tracks_; }

}  // namespace robot
