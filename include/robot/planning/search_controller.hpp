#pragma once

#include "robot/core/types.hpp"

namespace robot {

enum class SearchPhase { tracking, rotate_local, navigate_center, rotate_center, return_home, complete };

struct SearchResult {
    Twist2 command;
    SearchPhase phase = SearchPhase::tracking;
    double lost_seconds = 0;
};

class SearchController {
public:
    SearchResult update(const Pose2& pose, bool target_visible, Timestamp now, double dt_s);
    void reset();

private:
    Twist2 navigate(const Pose2& pose, double x_m, double y_m, double stop_radius_m,
                    double dt_s);
    Timestamp lost_since_{};
    Twist2 previous_command_;
    bool complete_ = false;
};

const char* to_string(SearchPhase phase);

}  // namespace robot
