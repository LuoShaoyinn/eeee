#pragma once

#include <string>
#include <vector>

#define CLASS_NUM 4
#define LETTERBOX_ROWS 640
#define LETTERBOX_COLS 640
#define SCORE_THRESHOLD 0.35f
#define NMS_THRESHOLD 0.45f

const std::vector<std::string> g_classes_name{
    "other_robot",
    "red_cube",
    "yellow_cylinder",
    "home",
};
