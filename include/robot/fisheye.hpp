#pragma once

#include <string>

#include <opencv2/core.hpp>

namespace robot {

// Owns the versioned fisheye calibration and lazily caches remap tables for
// the current camera resolution.  The rectified image is the canonical image
// space for localization and vision consumers.
class FisheyeRectifier {
public:
    explicit FisheyeRectifier(const std::string& calibration_path);

    [[nodiscard]] cv::Mat rectify(const cv::Mat& raw);
    [[nodiscard]] cv::Mat rectified_camera_matrix(int width, int height) const;

private:
    cv::Mat camera_matrix_;
    cv::Mat distortion_;
    cv::Mat rectified_matrix_;
    cv::Size map_size_;
    cv::Mat map_x_;
    cv::Mat map_y_;
};

}  // namespace robot
