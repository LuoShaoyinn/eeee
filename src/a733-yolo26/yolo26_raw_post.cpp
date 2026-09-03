// Decoder for the four-class YOLO26 A733 model.
#include <opencv2/core/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "model_config.h"

namespace {

constexpr int kCandidates = 8400;

struct Object {
    cv::Rect rect;
    int label;
    float score;
};

void decode(const cv::Mat& image, const float* boxes_data, const float* scores_data,
            std::vector<Object>* objects) {
    const float scale = std::min(
        static_cast<float>(LETTERBOX_COLS) / image.cols,
        static_cast<float>(LETTERBOX_ROWS) / image.rows);
    const float resized_w = std::round(image.cols * scale);
    const float resized_h = std::round(image.rows * scale);
    const float pad_x = (LETTERBOX_COLS - resized_w) * 0.5f;
    const float pad_y = (LETTERBOX_ROWS - resized_h) * 0.5f;

    std::vector<cv::Rect> boxes[CLASS_NUM];
    std::vector<float> scores[CLASS_NUM];

    for (int class_id = 0; class_id < CLASS_NUM; ++class_id) {
        const float* class_scores = scores_data + class_id * kCandidates;
        float low = class_scores[0];
        float high = class_scores[0];
        for (int candidate = 1; candidate < kCandidates; ++candidate) {
            low = std::min(low, class_scores[candidate]);
            high = std::max(high, class_scores[candidate]);
        }
        std::printf("class %d score range %.5f..%.5f\n", class_id, low, high);
    }

    // Each NBG output is planar [4, 8400]. Keeping boxes and scores separate
    // prevents coordinate values from collapsing the class-score INT8 scale.
    for (int candidate = 0; candidate < kCandidates; ++candidate) {
        int label = 0;
        float score = scores_data[candidate];
        for (int class_id = 1; class_id < CLASS_NUM; ++class_id) {
            const float candidate_score = scores_data[class_id * kCandidates + candidate];
            if (candidate_score > score) {
                label = class_id;
                score = candidate_score;
            }
        }
        if (score < SCORE_THRESHOLD) {
            continue;
        }

        const float center_x = boxes_data[candidate];
        const float center_y = boxes_data[kCandidates + candidate];
        const float width = boxes_data[2 * kCandidates + candidate];
        const float height = boxes_data[3 * kCandidates + candidate];
        if (!(width > 0.0f && height > 0.0f)) {
            continue;
        }

        const float left = (center_x - width * 0.5f - pad_x) / scale;
        const float top = (center_y - height * 0.5f - pad_y) / scale;
        const float right = (center_x + width * 0.5f - pad_x) / scale;
        const float bottom = (center_y + height * 0.5f - pad_y) / scale;
        const int x = std::max(0, std::min(static_cast<int>(std::floor(left)), image.cols - 1));
        const int y = std::max(0, std::min(static_cast<int>(std::floor(top)), image.rows - 1));
        const int x2 = std::max(0, std::min(static_cast<int>(std::ceil(right)), image.cols));
        const int y2 = std::max(0, std::min(static_cast<int>(std::ceil(bottom)), image.rows));
        if (x2 <= x || y2 <= y) {
            continue;
        }
        boxes[label].emplace_back(x, y, x2 - x, y2 - y);
        scores[label].push_back(score);
    }

    for (int class_id = 0; class_id < CLASS_NUM; ++class_id) {
        std::vector<int> kept;
        cv::dnn::NMSBoxes(boxes[class_id], scores[class_id], SCORE_THRESHOLD, NMS_THRESHOLD, kept);
        for (int index : kept) {
            objects->push_back({boxes[class_id][index], class_id, scores[class_id][index]});
        }
    }
}

void draw(const cv::Mat& image, const std::vector<Object>& objects) {
    cv::Mat annotated = image.clone();
    for (const Object& object : objects) {
        cv::rectangle(annotated, object.rect, cv::Scalar(255, 0, 0), 2);
        const std::string text = g_classes_name[object.label] + " " +
            cv::format("%.0f%%", object.score * 100.0f);
        cv::putText(annotated, text, object.rect.tl() + cv::Point(0, -4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 0), 2);
        std::printf("%s %.1f%% [%d, %d, %d, %d]\n", g_classes_name[object.label].c_str(),
                    object.score * 100.0f, object.rect.x, object.rect.y,
                    object.rect.x + object.rect.width, object.rect.y + object.rect.height);
    }
    cv::imwrite("out_yolo26.png", annotated);
}

}  // namespace

int yolo26_postprocess(const char* imagepath, float** output) {
    cv::Mat image = cv::imread(imagepath, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::fprintf(stderr, "could not read %s\n", imagepath);
        return -1;
    }
    std::vector<Object> objects;
    decode(image, output[0], output[1], &objects);
    std::printf("detection num: %zu\n", objects.size());
    draw(image, objects);
    return 0;
}
