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
}
