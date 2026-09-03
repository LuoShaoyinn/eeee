#include "robot/localization/location.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/features.hpp>
#endif
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace robot {
namespace {
constexpr double kWheelRadiusM = .023;
constexpr double kMecanumRadiusM = .190;
constexpr double kFieldLengthM = 3.0;
constexpr double kFieldWidthM = 1.985;

double wrap_angle(double value) {
    while (value > CV_PI) value -= 2.0 * CV_PI;
    while (value <= -CV_PI) value += 2.0 * CV_PI;
    return value;
}

double segment_distance(double x, double y, double ax, double ay, double bx, double by) {
    const double dx = bx - ax;
    const double dy = by - ay;
    const double length2 = dx * dx + dy * dy;
    const double t = length2 == 0 ? 0 : std::clamp(((x - ax) * dx + (y - ay) * dy) / length2, 0.0, 1.0);
    return std::hypot(x - (ax + t * dx), y - (ay + t * dy));
}

double field_wall_distance(double x, double y) {
    return std::min({segment_distance(x, y, 0, 0, kFieldLengthM, 0),
                     segment_distance(x, y, kFieldLengthM, 0, kFieldLengthM, kFieldWidthM),
                     segment_distance(x, y, kFieldLengthM, kFieldWidthM, 0, kFieldWidthM),
                     segment_distance(x, y, 0, kFieldWidthM, 0, 0)});
}
}  // namespace

GroundProjector::GroundProjector(cv::Mat camera_matrix, double camera_height_m,
                                 double pitch_down_deg, double roll_deg)
    : camera_height_m_(camera_height_m) {
    cv::Mat matrix64;
    camera_matrix.convertTo(matrix64, CV_64F);
    const cv::Mat inverse = matrix64.inv();
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) camera_inverse_(row, column) = inverse.at<double>(row, column);
    }
    const double pitch = pitch_down_deg * CV_PI / 180.0;
    const double roll = roll_deg * CV_PI / 180.0;
    const cv::Matx33d pitch_rotation(0, -std::sin(pitch), std::cos(pitch),
                                     -1, 0, 0,
                                     0, -std::cos(pitch), -std::sin(pitch));
    const cv::Matx33d roll_rotation(std::cos(roll), -std::sin(roll), 0,
                                    std::sin(roll), std::cos(roll), 0,
                                    0, 0, 1);
    rotation_car_from_camera_ = pitch_rotation * roll_rotation;
}

bool GroundProjector::project(const cv::Point2f& pixel, cv::Point2d& ground) const {
    return project_to_height(pixel, 0, ground);
}

bool GroundProjector::project_to_height(const cv::Point2f& pixel, double height_m,
                                        cv::Point2d& ground) const {
    const cv::Vec3d ray = rotation_car_from_camera_ * (camera_inverse_ * cv::Vec3d(pixel.x, pixel.y, 1));
    if (std::abs(ray[2]) < 1e-6) return false;
    const double scale = (height_m - camera_height_m_) / ray[2];
    if (scale <= 0) return false;
    ground = {scale * ray[0], scale * ray[1]};
    return std::isfinite(ground.x) && std::isfinite(ground.y) && ground.x > -.5 && ground.x < 6.0 &&
           std::abs(ground.y) < 4.0;
}

VisualMotion VisualOdometry::update(const cv::Mat& gray, const GroundProjector& projector, double dt_s) {
    VisualMotion output;
    if (dt_s <= .001 || previous_gray_.empty()) {
        previous_gray_ = gray.clone();
        cv::goodFeaturesToTrack(gray, previous_pixels_, 250, .01, 8.0);
        return output;
    }
    std::vector<cv::Point2f> current_pixels;
    std::vector<unsigned char> status;
    std::vector<float> errors;
    cv::calcOpticalFlowPyrLK(previous_gray_, gray, previous_pixels_, current_pixels, status, errors);
    std::vector<cv::Point2f> previous_ground_pixels, current_ground_pixels;
    for (size_t index = 0; index < current_pixels.size(); ++index) {
        if (!status[index] || errors[index] > 20 || previous_pixels_[index].y < gray.rows * .52F ||
            current_pixels[index].y < gray.rows * .52F) continue;
        cv::Point2d before, after;
        if (!projector.project(previous_pixels_[index], before) || !projector.project(current_pixels[index], after)) continue;
        previous_ground_pixels.emplace_back(static_cast<float>(before.x), static_cast<float>(before.y));
        current_ground_pixels.emplace_back(static_cast<float>(after.x), static_cast<float>(after.y));
    }
    if (previous_ground_pixels.size() >= 12) {
        cv::Mat inliers;
        cv::Mat affine = cv::estimateAffinePartial2D(previous_ground_pixels, current_ground_pixels,
                                                      inliers, cv::RANSAC, .04);
        if (!affine.empty()) {
            affine.convertTo(affine, CV_64F);
            const double a = affine.at<double>(0, 0);
            const double b = affine.at<double>(1, 0);
            const double yaw = -std::atan2(b, a);
            const double tx = affine.at<double>(0, 2);
            const double ty = affine.at<double>(1, 2);
            output.valid = true;
            output.tracked_features = cv::countNonZero(inliers);
            output.velocity.forward_mps = -(std::cos(yaw) * tx - std::sin(yaw) * ty) / dt_s;
            output.velocity.left_mps = -(std::sin(yaw) * tx + std::cos(yaw) * ty) / dt_s;
            output.velocity.yaw_radps = yaw / dt_s;
        }
    }
    previous_gray_ = gray.clone();
    cv::goodFeaturesToTrack(gray, previous_pixels_, 250, .01, 8.0);
    return output;
}

FenceParticleFilter::FenceParticleFilter(size_t count, double initial_x_m, double initial_y_m,
                                         double initial_yaw_rad, bool global_initialize, std::uint32_t seed)
    : generator_(seed) {
    std::uniform_real_distribution<double> global_x(0, kFieldLengthM), global_y(0, kFieldWidthM), global_yaw(-CV_PI, CV_PI);
    std::normal_distribution<double> local_x(initial_x_m, .06), local_y(initial_y_m, .06), local_yaw(initial_yaw_rad, 8.0 * CV_PI / 180.0);
    particles_.reserve(count);
    for (size_t index = 0; index < count; ++index) {
        const double x = global_initialize ? global_x(generator_) : std::clamp(local_x(generator_), 0.0, kFieldLengthM);
        const double y = global_initialize ? global_y(generator_) : std::clamp(local_y(generator_), 0.0, kFieldWidthM);
        const double yaw = global_initialize ? global_yaw(generator_) : wrap_angle(local_yaw(generator_));
        particles_.push_back({x, y, yaw, 1.0 / count});
    }
}

void FenceParticleFilter::predict(const BodyVelocity& velocity, double dt_s) {
    const double forward_displacement = velocity.forward_mps * dt_s;
    const double left_displacement = velocity.left_mps * dt_s;
    const double yaw_displacement = velocity.yaw_radps * dt_s;
    const double translation_sigma = .00015 + .035 * std::hypot(forward_displacement, left_displacement);
    const double yaw_sigma = .0002 + .035 * std::abs(yaw_displacement);
    std::normal_distribution<double> translation_noise(0, translation_sigma);
    std::normal_distribution<double> yaw_noise(0, yaw_sigma);
    for (auto& particle : particles_) {
        const double forward = forward_displacement + translation_noise(generator_);
        const double left = left_displacement + translation_noise(generator_);
        particle.x = std::clamp(particle.x + std::cos(particle.yaw) * forward - std::sin(particle.yaw) * left, 0.0, kFieldLengthM);
        particle.y = std::clamp(particle.y + std::sin(particle.yaw) * forward + std::cos(particle.yaw) * left, 0.0, kFieldWidthM);
        particle.yaw = wrap_angle(particle.yaw + yaw_displacement + yaw_noise(generator_));
    }
}

void FenceParticleFilter::update(const std::vector<cv::Point2d>& observations) {
    if (observations.size() < 20) return;
    constexpr double sigma_m = .07;
    for (auto& particle : particles_) {
        std::vector<std::pair<double, double>> distances;
        distances.reserve(observations.size());
        const double cosine = std::cos(particle.yaw);
        const double sine = std::sin(particle.yaw);
        for (const auto& point : observations) {
            const double x = particle.x + cosine * point.x - sine * point.y;
            const double y = particle.y + sine * point.x + cosine * point.y;
            const double range = std::hypot(point.x, point.y);
            distances.push_back({field_wall_distance(x, y),
                                 1.0 / ((.20 + range) * (.20 + range))});
        }
        std::sort(distances.begin(), distances.end(), [](const auto& left, const auto& right) {
            return left.first < right.first;
        });
        const size_t keep = std::max<size_t>(10, distances.size() * 2 / 3);
        double weighted_error = 0;
        double total_weight = 0;
        for (size_t index = 0; index < keep; ++index) {
            weighted_error += distances[index].first * distances[index].second;
            total_weight += distances[index].second;
        }
        const double mean = weighted_error / total_weight;
        particle.weight *= std::exp(-.5 * mean * mean / (sigma_m * sigma_m));
    }
    const double total = std::accumulate(particles_.begin(), particles_.end(), 0.0,
                                         [](double sum, const Particle& item) { return sum + item.weight; });
    if (total <= 1e-20) {
        const double weight = 1.0 / particles_.size();
        for (auto& particle : particles_) particle.weight = weight;
        return;
    }
    for (auto& particle : particles_) particle.weight /= total;
    const double ess_inverse = std::accumulate(particles_.begin(), particles_.end(), 0.0,
                                                [](double sum, const Particle& item) { return sum + item.weight * item.weight; });
    if (1.0 / ess_inverse < particles_.size() * .55) resample();
}

void FenceParticleFilter::correct_toward(const Pose2& target, double gain,
                                         double max_distance_m, double max_yaw_rad,
                                         double major_axis_rad, double major_axis_gain) {
    if (particles_.empty() || gain <= 0 || max_distance_m <= 0 || max_yaw_rad <= 0) return;
    const PoseEstimate current = estimate();
    double dx = target.x_m - current.x_m;
    double dy = target.y_m - current.y_m;
    major_axis_gain = std::clamp(major_axis_gain, 0.0, 1.0);
    const double axis_x = std::cos(major_axis_rad);
    const double axis_y = std::sin(major_axis_rad);
    const double along = dx * axis_x + dy * axis_y;
    dx += (major_axis_gain - 1.0) * along * axis_x;
    dy += (major_axis_gain - 1.0) * along * axis_y;
    const double distance = std::hypot(dx, dy);
    const double step = std::min(max_distance_m, gain * distance);
    if (distance > 1e-9) {
        dx *= step / distance;
        dy *= step / distance;
    }
    const double yaw_step = std::clamp(gain * wrap_angle(target.yaw_rad - current.yaw_rad),
                                       -max_yaw_rad, max_yaw_rad);
    for (auto& particle : particles_) {
        particle.x = std::clamp(particle.x + dx, 0.0, kFieldLengthM);
        particle.y = std::clamp(particle.y + dy, 0.0, kFieldWidthM);
        particle.yaw = wrap_angle(particle.yaw + yaw_step);
    }
}

void FenceParticleFilter::resample() {
    std::vector<Particle> output;
    output.reserve(particles_.size());
    std::uniform_real_distribution<double> offset(0, 1.0 / particles_.size());
    double cursor = offset(generator_);
    double cumulative = particles_.front().weight;
    size_t source = 0;
    for (size_t index = 0; index < particles_.size(); ++index) {
        while (cursor > cumulative && source + 1 < particles_.size()) cumulative += particles_[++source].weight;
        output.push_back(particles_[source]);
        output.back().weight = 1.0 / particles_.size();
        cursor += 1.0 / particles_.size();
    }
    particles_ = std::move(output);
}

PoseEstimate FenceParticleFilter::estimate() const {
    PoseEstimate output;
    double sine = 0, cosine = 0, squared_weight = 0;
    output.x_m = output.y_m = 0;
    for (const auto& particle : particles_) {
        output.x_m += particle.weight * particle.x;
        output.y_m += particle.weight * particle.y;
        sine += particle.weight * std::sin(particle.yaw);
        cosine += particle.weight * std::cos(particle.yaw);
        squared_weight += particle.weight * particle.weight;
    }
    output.yaw_rad = std::atan2(sine, cosine);
    double position_variance = 0;
    double yaw_variance = 0;
    for (const auto& particle : particles_) {
        const double dx = particle.x - output.x_m;
        const double dy = particle.y - output.y_m;
        const double dyaw = wrap_angle(particle.yaw - output.yaw_rad);
        position_variance += particle.weight * (dx * dx + dy * dy);
        yaw_variance += particle.weight * dyaw * dyaw;
    }
    output.position_sigma_m = std::sqrt(position_variance);
    output.yaw_sigma_rad = std::sqrt(yaw_variance);
    output.effective_particles = squared_weight > 0 ? 1.0 / squared_weight : 0;
    return output;
}

BodyVelocity wheel_body_velocity(const std::array<double, 4>& rpm, const std::array<double, 4>& targets) {
    std::array<double, 4> wheel{};
    for (size_t index = 0; index < wheel.size(); ++index) {
        const double sign = targets[index] > .03 ? 1.0 : targets[index] < -.03 ? -1.0 : 0.0;
        wheel[index] = sign * rpm[index] * 2.0 * CV_PI * kWheelRadiusM / 60.0;
    }
    return {(wheel[0] + wheel[1] + wheel[2] + wheel[3]) / 4.0,
            (-wheel[0] + wheel[1] + wheel[2] - wheel[3]) / 4.0,
            (-wheel[0] + wheel[1] - wheel[2] + wheel[3]) / (4.0 * kMecanumRadiusM)};
}

VisualGeometryEstimate estimate_fence_geometry(
    const std::vector<cv::Point2d>& observations, double yaw_prior_rad,
    std::size_t maximum_candidates) {
    VisualGeometryEstimate result;
    std::vector<VisualPoseCandidate> evaluated;
    if (observations.size() < 20 || maximum_candidates == 0) return result;
    constexpr double kPositionStepM = .05;
    constexpr double kYawSpanRad = 25.0 * CV_PI / 180.0;
    constexpr double kYawStepRad = 5.0 * CV_PI / 180.0;
    std::vector<std::pair<double, double>> weighted_distances(observations.size());
    for (double yaw = yaw_prior_rad - kYawSpanRad; yaw <= yaw_prior_rad + kYawSpanRad + 1e-9;
         yaw += kYawStepRad) {
        const double cosine = std::cos(yaw);
        const double sine = std::sin(yaw);
        for (double x = 0; x <= kFieldLengthM + 1e-9; x += kPositionStepM) {
            for (double y = 0; y <= kFieldWidthM + 1e-9; y += kPositionStepM) {
                for (std::size_t index = 0; index < observations.size(); ++index) {
                    const auto& point = observations[index];
                    const double range = std::hypot(point.x, point.y);
                    weighted_distances[index] = {field_wall_distance(
                        x + cosine * point.x - sine * point.y,
                        y + sine * point.x + cosine * point.y),
                        1.0 / ((.20 + range) * (.20 + range))};
                }
                const std::size_t keep = std::max<std::size_t>(10, weighted_distances.size() * 2 / 3);
                std::nth_element(weighted_distances.begin(), weighted_distances.begin() +
                                 static_cast<std::ptrdiff_t>(keep), weighted_distances.end(),
                                 [](const auto& left, const auto& right) {
                                     return left.first < right.first;
                                 });
                double weighted_error = 0;
                double total_range_weight = 0;
                for (std::size_t index = 0; index < keep; ++index) {
                    weighted_error += weighted_distances[index].first * weighted_distances[index].second;
                    total_range_weight += weighted_distances[index].second;
                }
                const double residual = weighted_error / total_range_weight;
                evaluated.push_back({{x, y, wrap_angle(yaw)}, residual});
            }
        }
    }
    std::sort(evaluated.begin(), evaluated.end(), [](const auto& left, const auto& right) {
        return left.wall_residual_m < right.wall_residual_m;
    });
    std::vector<VisualPoseCandidate> output;
    for (const auto& candidate : evaluated) {
        const bool distinct = std::all_of(output.begin(), output.end(), [&](const auto& accepted) {
            return std::hypot(candidate.pose.x_m - accepted.pose.x_m,
                              candidate.pose.y_m - accepted.pose.y_m) >= .30;
        });
        if (distinct) output.push_back(candidate);
        if (output.size() == maximum_candidates) break;
    }
    result.candidates = std::move(output);
    if (result.candidates.empty()) return result;

    const auto& best = result.candidates.front();
    constexpr double kLocalRadiusM = .75;
    constexpr double kLocalScoreWindowM = .02;
    constexpr double kWeightScaleM = .01;
    double total_weight = 0;
    double mean_x = 0;
    double mean_y = 0;
    double mean_yaw_sine = 0;
    double mean_yaw_cosine = 0;
    double alternative_score = std::numeric_limits<double>::infinity();
    for (const auto& candidate : evaluated) {
        const double separation = std::hypot(candidate.pose.x_m - best.pose.x_m,
                                             candidate.pose.y_m - best.pose.y_m);
        const double yaw_difference = std::abs(wrap_angle(candidate.pose.yaw_rad - best.pose.yaw_rad));
        if (separation >= kLocalRadiusM || yaw_difference > 10.0 * CV_PI / 180.0) {
            alternative_score = std::min(alternative_score, candidate.wall_residual_m);
            continue;
        }
        if (candidate.wall_residual_m > best.wall_residual_m + kLocalScoreWindowM) continue;
        const double weight = std::exp(-(candidate.wall_residual_m - best.wall_residual_m) /
                                       kWeightScaleM);
        total_weight += weight;
        mean_x += weight * candidate.pose.x_m;
        mean_y += weight * candidate.pose.y_m;
        mean_yaw_sine += weight * std::sin(candidate.pose.yaw_rad);
        mean_yaw_cosine += weight * std::cos(candidate.pose.yaw_rad);
    }
    if (total_weight <= 0) return result;
    mean_x /= total_weight;
    mean_y /= total_weight;
    const double mean_yaw = std::atan2(mean_yaw_sine, mean_yaw_cosine);
    double covariance_xx = 0;
    double covariance_xy = 0;
    double covariance_yy = 0;
    double yaw_variance = 0;
    for (const auto& candidate : evaluated) {
        const double separation = std::hypot(candidate.pose.x_m - best.pose.x_m,
                                             candidate.pose.y_m - best.pose.y_m);
        const double yaw_difference = std::abs(wrap_angle(candidate.pose.yaw_rad - best.pose.yaw_rad));
        if (separation >= kLocalRadiusM || yaw_difference > 10.0 * CV_PI / 180.0 ||
            candidate.wall_residual_m > best.wall_residual_m + kLocalScoreWindowM) continue;
        const double weight = std::exp(-(candidate.wall_residual_m - best.wall_residual_m) /
                                       kWeightScaleM);
        const double dx = candidate.pose.x_m - mean_x;
        const double dy = candidate.pose.y_m - mean_y;
        covariance_xx += weight * dx * dx;
        covariance_xy += weight * dx * dy;
        covariance_yy += weight * dy * dy;
        const double dyaw = wrap_angle(candidate.pose.yaw_rad - mean_yaw);
        yaw_variance += weight * dyaw * dyaw;
    }
    covariance_xx /= total_weight;
    covariance_xy /= total_weight;
    covariance_yy /= total_weight;
    result.yaw_sigma_rad = std::sqrt(yaw_variance / total_weight);
    const double trace = covariance_xx + covariance_yy;
    const double discriminant = std::sqrt(std::max(0.0,
        (covariance_xx - covariance_yy) * (covariance_xx - covariance_yy) +
        4.0 * covariance_xy * covariance_xy));
    result.sigma_major_m = std::sqrt(std::max(0.0, .5 * (trace + discriminant)));
    result.sigma_minor_m = std::sqrt(std::max(0.0, .5 * (trace - discriminant)));
    result.major_axis_rad = .5 * std::atan2(2.0 * covariance_xy,
                                            covariance_xx - covariance_yy);
    result.alternative_margin_m = std::isfinite(alternative_score)
        ? std::max(0.0, alternative_score - best.wall_residual_m) : 0;
    const double residual_quality = std::clamp(1.0 - best.wall_residual_m / .06, 0.0, 1.0);
    const double margin_quality = std::clamp(result.alternative_margin_m / .03, 0.0, 1.0);
    const double shape_quality = std::clamp(1.0 - result.sigma_major_m / .50, 0.0, 1.0);
    result.confidence = residual_quality * (.3 + .7 * margin_quality) *
                        (.4 + .6 * shape_quality);
    result.valid = best.wall_residual_m < .06;
    return result;
}

std::vector<VisualPoseCandidate> match_fence_geometry(
    const std::vector<cv::Point2d>& observations, double yaw_prior_rad,
    std::size_t maximum_candidates) {
    return estimate_fence_geometry(observations, yaw_prior_rad, maximum_candidates).candidates;
}

}  // namespace robot
