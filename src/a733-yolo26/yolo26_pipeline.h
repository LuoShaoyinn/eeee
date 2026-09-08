#pragma once

#include <opencv2/core/core.hpp>

#include <string>
#include <vector>

struct Yolo26Detection {
    cv::Rect box;
    int class_id;
    float score;
};

int yolo26_preprocess_frame(const cv::Mat& image, void* buffer, unsigned int buffer_size);
std::vector<Yolo26Detection> yolo26_decode_frame(const cv::Size& image_size,
                                                  float** output);
