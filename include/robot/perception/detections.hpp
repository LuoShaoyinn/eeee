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

}  // namespace robot
