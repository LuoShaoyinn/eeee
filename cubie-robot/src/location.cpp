#include "robot/location.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <opencv2/calib3d.hpp>
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
    const cv::Vec3d ray = rotation_car_from_camera_ * (camera_inverse_ * cv::Vec3d(pixel.x, pixel.y, 1));
    if (ray[2] >= -1e-6) return false;
    const double scale = -camera_height_m_ / ray[2];
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

FenceParticleFilter::FenceParticleFilter(size_t count, std::uint32_t seed) : generator_(seed) {
    std::uniform_real_distribution<double> x(0, kFieldLengthM), y(0, kFieldWidthM), yaw(-CV_PI, CV_PI);
    particles_.reserve(count);
    for (size_t index = 0; index < count; ++index) particles_.push_back({x(generator_), y(generator_), yaw(generator_), 1.0 / count});
}

void FenceParticleFilter::predict(const BodyVelocity& velocity, double dt_s) {
    std::normal_distribution<double> translation_noise(0, .008 + .05 * dt_s);
    std::normal_distribution<double> yaw_noise(0, .01 + .04 * dt_s);
    for (auto& particle : particles_) {
        const double forward = velocity.forward_mps * dt_s + translation_noise(generator_);
        const double left = velocity.left_mps * dt_s + translation_noise(generator_);
        particle.x += std::cos(particle.yaw) * forward - std::sin(particle.yaw) * left;
        particle.y += std::sin(particle.yaw) * forward + std::cos(particle.yaw) * left;
        particle.yaw = wrap_angle(particle.yaw + velocity.yaw_radps * dt_s + yaw_noise(generator_));
    }
}

void FenceParticleFilter::update(const std::vector<cv::Point2d>& observations) {
    if (observations.size() < 20) return;
    constexpr double sigma_m = .07;
    for (auto& particle : particles_) {
        std::vector<double> distances;
        distances.reserve(observations.size());
        const double cosine = std::cos(particle.yaw);
        const double sine = std::sin(particle.yaw);
        for (const auto& point : observations) {
            const double x = particle.x + cosine * point.x - sine * point.y;
            const double y = particle.y + sine * point.x + cosine * point.y;
            distances.push_back(field_wall_distance(x, y));
        }
        std::sort(distances.begin(), distances.end());
        const size_t keep = std::max<size_t>(10, distances.size() * 2 / 3);
        const double mean = std::accumulate(distances.begin(), distances.begin() + keep, 0.0) / keep;
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

}  // namespace robot
