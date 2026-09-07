#pragma once

#include "robot/core/types.hpp"

namespace robot {

struct SearchConfig {
    double local_rotate_seconds = 5.0;
    double center_search_seconds = 5.0;
    double center_x_m = 1.5;
    double center_y_m = .9925;
    double center_entry_radius_m = .25;
    double center_exit_radius_m = .35;
    double home_x_m = .1;
    double home_y_m = .15;
    double home_stop_radius_m = .3;
    double rotation_speed_radps = .35;
    double navigate_translation_kp = .55;
    double navigate_yaw_kp = 1.0;
    double maximum_linear_mps = .20;
    double maximum_yaw_radps = .6;
};

enum class SearchPhase {
    tracking,
    rotate_local,
    navigate_center,
    rotate_center,
    hold_for_localization,
    return_home,
    complete,
};

struct SearchResult {
    Twist2 command;
    SearchPhase phase = SearchPhase::tracking;
    double lost_seconds = 0;
};

class SearchController {
public:
    explicit SearchController(SearchConfig config = {});
    SearchResult update(const Pose2& pose, bool target_visible, Timestamp now, double dt_s,
                        bool navigation_allowed = true);
    void reset();

private:
    Twist2 navigate(const Pose2& pose, double x_m, double y_m, double stop_radius_m,
                    double dt_s);
    SearchConfig config_;
    Timestamp lost_since_{};
    // The center-search dwell begins only after the chassis reaches the
    // center region. Travel time and localization holds do not consume it.
    Timestamp center_search_started_{};
    Twist2 previous_command_;
    bool center_reached_ = false;
    bool center_search_complete_ = false;
    bool complete_ = false;
};

const char* to_string(SearchPhase phase);

}  // namespace robot
