#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <vector>

#include <opencv2/core.hpp>

#include "robot/core/types.hpp"

namespace robot {

using BodyVelocity = Twist2;

struct VisualMotion {
    bool valid = false;
    int tracked_features = 0;
    BodyVelocity velocity;
};

struct PoseEstimate {
    double x_m = 1.5;
    double y_m = .9925;
    double yaw_rad = 0;
    double position_sigma_m = 0;
    double yaw_sigma_rad = 0;
    double effective_particles = 0;
};

struct VisualPoseCandidate {
    Pose2 pose;
    double wall_residual_m = 0;
};

struct VisualGeometryEstimate {
    std::vector<VisualPoseCandidate> candidates;
    double confidence = 0;
    double alternative_margin_m = 0;
    double sigma_major_m = 0;
    double sigma_minor_m = 0;
    double major_axis_rad = 0;
    bool valid = false;
};

class GroundProjector {
public:
    GroundProjector(cv::Mat camera_matrix, double camera_height_m, double pitch_down_deg,
                    double roll_deg = 0);
    bool project(const cv::Point2f& pixel, cv::Point2d& ground) const;

private:
    cv::Matx33d camera_inverse_;
    cv::Matx33d rotation_car_from_camera_;
    double camera_height_m_;
};

class VisualOdometry {
public:
    VisualMotion update(const cv::Mat& gray, const GroundProjector& projector, double dt_s);

private:
    cv::Mat previous_gray_;
    std::vector<cv::Point2f> previous_pixels_;
};

class FenceParticleFilter {
public:
    FenceParticleFilter(size_t count, double initial_x_m, double initial_y_m,
                        double initial_yaw_rad, bool global_initialize = false,
                        std::uint32_t seed = 1);
    void predict(const BodyVelocity& body_velocity, double dt_s);
    void update(const std::vector<cv::Point2d>& lower_fence_points);
    PoseEstimate estimate() const;

private:
    struct Particle { double x, y, yaw, weight; };
    std::vector<Particle> particles_;
    std::mt19937 generator_;
    void resample();
};

BodyVelocity wheel_body_velocity(const std::array<double, 4>& rpm,
                                 const std::array<double, 4>& targets);

std::vector<VisualPoseCandidate> match_fence_geometry(
    const std::vector<cv::Point2d>& observations, double yaw_prior_rad,
    std::size_t maximum_candidates = 4);
VisualGeometryEstimate estimate_fence_geometry(
    const std::vector<cv::Point2d>& observations, double yaw_prior_rad,
    std::size_t maximum_candidates = 4);

}  // namespace robot
