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
    double home_stop_radius_m = .2;
    double rotation_speed_radps = .35;
    double navigate_translation_kp = .55;
    double navigate_yaw_kp = 1.0;
    double maximum_linear_mps = .20;
    double maximum_yaw_radps = .6;
    // Fixed recovery trajectory after reaching the home docking corner.
    // After returning to the home-center radius, align yaw in place.
    double post_home_moonwalk_yaw_deg = -145.0;
    double post_home_moonwalk_yaw_tolerance_deg = 10.0;
    double post_home_moonwalk_yaw_kp = 1.5;
    double post_home_moonwalk_timeout_seconds = 10.0;
    double post_home_turn_degrees = 180.0;
    double post_home_turn_yaw_kp = 1.5;
    double post_home_turn_yaw_tolerance_deg = 5.0;
    double post_home_reverse_mps = .30;
    double post_home_reverse_seconds = 2.0;
};

enum class SearchPhase {
    tracking,
    rotate_local,
    navigate_center,
    rotate_center,
    hold_for_localization,
    return_home,
    post_home_moonwalk,
    post_home_turn,
    post_home_reverse,
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
    // Explicit recovery path for operator-triggered return-home testing. It
    // visits the arena center and then the physical home corner without
    // searching for targets or running the collector.
    void begin_return_home();
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
    bool direct_return_home_ = false;
    bool complete_ = false;
    enum class PostHomePhase { none, moonwalk, turn, reverse };
    PostHomePhase post_home_phase_ = PostHomePhase::none;
    Timestamp post_home_phase_started_{};
    double post_home_turn_target_yaw_rad_ = 0;
};

const char* to_string(SearchPhase phase);

}  // namespace robot
