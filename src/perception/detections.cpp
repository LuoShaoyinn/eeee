#include "robot/perception/detections.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace robot {
namespace {

float area(const ImageBox& box) {
    return std::max(0.0F, box.right - box.left) * std::max(0.0F, box.bottom - box.top);
}

float overlap_over_smaller_box(const ImageBox& first, const ImageBox& second) {
    const float left = std::max(first.left, second.left);
    const float top = std::max(first.top, second.top);
    const float right = std::min(first.right, second.right);
    const float bottom = std::min(first.bottom, second.bottom);
    const float intersection = std::max(0.0F, right - left) * std::max(0.0F, bottom - top);
    const float smaller_area = std::min(area(first), area(second));
    return smaller_area > 0.0F ? intersection / smaller_area : 0.0F;
}

}  // namespace

std::vector<Detection> deduplicate_same_class_detections(
    std::vector<Detection> detections, float minimum_overlap) {
    minimum_overlap = std::clamp(minimum_overlap, 0.0F, 1.0F);
    std::stable_sort(detections.begin(), detections.end(),
                     [](const Detection& left, const Detection& right) {
                         return left.confidence > right.confidence;
                     });

    std::vector<Detection> kept;
    kept.reserve(detections.size());
    for (const Detection& candidate : detections) {
        const bool duplicate = std::any_of(kept.begin(), kept.end(), [&](const Detection& prior) {
            return prior.object_class == candidate.object_class &&
                   overlap_over_smaller_box(prior.box, candidate.box) >= minimum_overlap;
        });
        if (!duplicate) kept.push_back(candidate);
    }
    return kept;
}

}  // namespace robot
