#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

#include "robot/localization/location.hpp"

int main() {
    robot::FenceParticleFilter filter(1000, .1, .1, 0, false, 4);
    const auto before = filter.estimate();
    filter.correct_toward({2.0, 1.0, .5}, .5, .04, .02);
    const auto after = filter.estimate();
    const double correction = std::hypot(after.x_m - before.x_m,
                                         after.y_m - before.y_m);
    if (correction > .0401 || correction < .039 ||
        std::abs(after.yaw_rad - before.yaw_rad) > .0201) {
        std::cerr << "bounded visual correction exceeded its limit\n";
        return 1;
    }
    robot::FenceParticleFilter axis_filter(1000, .5, .5, 0, false, 5);
    const auto axis_before = axis_filter.estimate();
    axis_filter.correct_toward_axes({1.5, 1.5, 1.0}, {0, .5, 1}, .12, .8,
                                    1.0, 30.0 * CV_PI / 180.0);
    const auto axis_after = axis_filter.estimate();
    if (std::abs(axis_after.x_m - axis_before.x_m) > 1e-9 ||
        axis_after.y_m <= axis_before.y_m ||
        axis_after.y_m - axis_before.y_m > .5001 ||
        std::abs(axis_after.yaw_rad - axis_before.yaw_rad) >
            30.01 * CV_PI / 180.0) {
        std::cerr << "axis certainty did not independently bound correction\n";
        return 1;
    }
    std::vector<cv::Point2d> wall;
    for (double x = -.8; x <= .8; x += .04) wall.emplace_back(x, -.5);
    const auto estimate = robot::estimate_fence_geometry(wall, 0, 4);
    const auto& candidates = estimate.candidates;
    if (candidates.empty()) {
        std::cerr << "geometry matcher returned no candidates\n";
        return 1;
    }
    if (candidates.front().wall_residual_m > .001 ||
        std::abs(candidates.front().pose.y_m - .5) > .051 ||
        std::abs(candidates.front().pose.yaw_rad) > .001) {
        std::cerr << "geometry matcher did not recover the synthetic wall offset\n";
        return 1;
    }
    if (!estimate.valid || estimate.sigma_major_m <= estimate.sigma_minor_m) {
        std::cerr << "single-wall uncertainty is not anisotropic\n";
        return 1;
    }
    if (estimate.point_support <= 0 || estimate.point_support > 1 ||
        estimate.axis_sigma[0] < 0 || estimate.axis_sigma[1] < 0 ||
        estimate.axis_sigma[2] < 0) {
        std::cerr << "geometry uncertainty metadata is invalid\n";
        return 1;
    }
}
