#pragma once

#include <cstdint>
#include <vector>

#include "robot/core/types.hpp"

namespace robot {

enum class ObjectClass : std::uint8_t {
    yellow_cylinder,
    red_cube,
    home,
    opponent_robot,
};

struct ImageBox {
    float left = 0;
    float top = 0;
    float right = 0;
    float bottom = 0;
};

struct Detection {
    ObjectClass object_class{};
    float confidence = 0;
    ImageBox box;
};

struct DetectionFrame {
    Timestamp timestamp{};
    std::uint64_t frame_sequence = 0;
    std::vector<Detection> detections;
};

// Keep the most confident member of each set of substantially overlapping
// boxes from the same class. This is deliberately class-aware: a collectible
// inside an opponent-robot box must remain visible to the later safety filter.
std::vector<Detection> deduplicate_same_class_detections(
    std::vector<Detection> detections, float minimum_overlap = .50F);

}  // namespace robot
