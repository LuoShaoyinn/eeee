#pragma once

#include <opencv2/core.hpp>

#include "robot/perception/detections.hpp"

namespace robot {

class Detector {
public:
    virtual ~Detector() = default;
    virtual DetectionFrame detect(const cv::Mat& image, Timestamp timestamp,
                                  std::uint64_t frame_sequence) = 0;
};

}  // namespace robot
