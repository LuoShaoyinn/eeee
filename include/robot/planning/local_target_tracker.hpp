#pragma once

#include <chrono>
#include <optional>
#include <vector>

#include "robot/planning/world_model.hpp"

namespace robot {

struct LocalTargetTrackerConfig {
    // Detection outages while the chassis turns are common. This bounds how
    // long odometry alone may propagate an otherwise static collectable.
    std::chrono::milliseconds memory{3000};
    std::chrono::milliseconds acquisition_freshness{250};
    // A new candidate must persist across several detector outputs before it
    // can interrupt search. This prevents a one-frame false positive from
    // restarting the no-object timer after a successful collection pass.
    std::chrono::milliseconds acquisition_confirmation{200};
    std::chrono::milliseconds confirmation_gap{150};
    unsigned minimum_observations = 3;
    // Once the intake dash has completed, ignore boxes close to that object
    // longer than normal target memory. The dash is the collection-success
    // signal; a stale detector box must not revive the completed target.
    std::chrono::milliseconds collection_suppression{8000};
    double association_gate_m = .75;
    // Detector ground projection is visibly noisier than wheel/IMU odometry
    // over one inference interval. Use measurements to correct the local
    // static-object track gradually so the approach PID does not chase box
    // jitter with alternating wheel reversals.
    double measurement_gain = .20;
};

// Tracks one collectable in the wheel/IMU odometry frame. Particle-filter
// corrections deliberately do not move this local frame during an approach.
class LocalTargetTracker {
public:
    explicit LocalTargetTracker(LocalTargetTrackerConfig config = {});

    void observe(const std::vector<TrackedObject>& observations, const Pose2& odometry_pose,
                 Timestamp now);
    // Choose and lock the nearest recently observed collectible. All other
    // tracks remain available for telemetry and later selection.
    std::optional<TrackedObject> acquire_nearest(const Pose2& odometry_pose, Timestamp now);
    std::optional<TrackedObject> target(const Pose2& odometry_pose, Timestamp now) const;
    // The collector has just passed over this local target. Ignore stale model
    // boxes at the same place while it clears the camera view.
    void mark_collected(Timestamp now);
    void release_target();
    void reset();
    const std::vector<TrackedObject>& tracks() const;

private:
    std::optional<TrackedObject> predicted_track(const TrackedObject& track,
                                                  const Pose2& odometry_pose) const;
    bool confirmed(const TrackedObject& track, Timestamp now) const;
    void discard_expired(Timestamp now);
    LocalTargetTrackerConfig config_;
    std::vector<TrackedObject> tracks_;
    std::optional<std::uint64_t> active_track_id_;
    std::optional<TrackedObject> suppressed_track_;
    Timestamp suppress_until_{};
    std::uint64_t next_track_id_ = 1;
};

}  // namespace robot
