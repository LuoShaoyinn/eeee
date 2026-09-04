#include "robot/fisheye.hpp"

#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace robot {

FisheyeRectifier::FisheyeRectifier(const std::string& calibration_path) {
    cv::FileStorage calibration(calibration_path, cv::FileStorage::READ);
    if (!calibration.isOpened()) throw std::runtime_error("cannot open calibration: " + calibration_path);
    calibration["K"] >> camera_matrix_;
    calibration["D"] >> distortion_;
    calibration["rectified_K"] >> rectified_matrix_;
    if (camera_matrix_.empty() || distortion_.empty() || rectified_matrix_.empty()) {
        throw std::runtime_error("calibration lacks K, D, or rectified_K");
    }
    camera_matrix_.convertTo(camera_matrix_, CV_64F);
    distortion_.convertTo(distortion_, CV_64F);
    rectified_matrix_.convertTo(rectified_matrix_, CV_64F);
}

cv::Mat FisheyeRectifier::rectified_camera_matrix(int width, int height) const {
    if (width <= 0 || height <= 0) throw std::runtime_error("invalid rectified image size");
    cv::Mat result = rectified_matrix_.clone();
    const double scale_x = static_cast<double>(width) / 1280.0;
    const double scale_y = static_cast<double>(height) / 720.0;
    result.at<double>(0, 0) *= scale_x;
    result.at<double>(0, 2) *= scale_x;
    result.at<double>(1, 1) *= scale_y;
    result.at<double>(1, 2) *= scale_y;
    return result;
}

cv::Mat FisheyeRectifier::rectify(const cv::Mat& raw) {
    if (raw.empty()) throw std::runtime_error("cannot rectify an empty frame");
    if (raw.size() != map_size_) {
        const double scale_x = static_cast<double>(raw.cols) / 1280.0;
        const double scale_y = static_cast<double>(raw.rows) / 720.0;
        cv::Mat source_matrix = camera_matrix_.clone();
        source_matrix.at<double>(0, 0) *= scale_x;
        source_matrix.at<double>(0, 2) *= scale_x;
        source_matrix.at<double>(1, 1) *= scale_y;
        source_matrix.at<double>(1, 2) *= scale_y;
        cv::fisheye::initUndistortRectifyMap(source_matrix, distortion_, cv::Mat::eye(3, 3, CV_64F),
                                             rectified_camera_matrix(raw.cols, raw.rows), raw.size(), CV_16SC2,
                                             map_x_, map_y_);
        map_size_ = raw.size();
    }
    cv::Mat rectified;
    cv::remap(raw, rectified, map_x_, map_y_, cv::INTER_LINEAR);
    return rectified;
}

}  // namespace robot
