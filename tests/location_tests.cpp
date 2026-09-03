#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

#include "robot/localization/location.hpp"

int main() {
    std::vector<cv::Point2d> wall;
    for (double x = -.8; x <= .8; x += .04) wall.emplace_back(x, -.5);
    const auto candidates = robot::match_fence_geometry(wall, 0, 4);
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
}
