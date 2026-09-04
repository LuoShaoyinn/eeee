#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace robot {

enum class ObjectClass { yellow, red, other_robot, home };

struct Detection {
    ObjectClass object_class;
    double confidence = 0.0;
    // Normalized image coordinates.  x is 0 at the left edge, y is 0 at the
    // top edge. bottom is the normalized bottom edge of the detection box.
    double center_x = 0.5;
    double bottom_y = 0.0;
};

enum class MissionState {
    initializing,
    searching,
    approaching_target,
    avoiding_robot,
    returning_home,
    docking_home,
    dumping,
    done,
    fault,
};

struct MissionConfig {
    int expected_collectibles = 0;
    int frames_to_confirm_collection = 4;
    int frames_to_confirm_dock = 8;
    int max_lost_target_frames = 8;
    double min_confidence = 0.55;
    double obstacle_bottom_y = 0.36;
    double collect_bottom_y = 0.82;
    double home_dock_bottom_y = 0.86;
    double search_yaw_radps = 0.45;
    double cruise_mps = 0.30;
    double final_approach_mps = 0.15;
    double avoid_left_mps = 0.16;
    double avoid_yaw_radps = 0.70;
    double steering_gain = 1.35;
    // The installed front GA25 motor is wired so negative power rotates in
    // the collecting direction.
    int collector_percent = -100;
    int dump_servo_pulse_us = 2000;
    int stow_servo_pulse_us = 1600;
};

struct MissionInput {
    bool localization_valid = false;
    bool collection_sensor_triggered = false;
    std::vector<Detection> detections;
};

struct MissionOutput {
    MissionState state = MissionState::initializing;
    double forward_mps = 0.0;
    double left_mps = 0.0;
    double yaw_radps = 0.0;
    int collector_percent = 0;
    std::optional<int> servo_pulse_us;
    bool emergency_stop = false;
};

// High-level policy only: camera/NPU and ESP32 transport remain outside this
// class so the safety-critical decisions can be replayed and tested offline.
class MissionController {
public:
    explicit MissionController(MissionConfig config = {});

    MissionOutput update(const MissionInput& input);
    [[nodiscard]] MissionState state() const { return state_; }
    [[nodiscard]] int collected_count() const { return collected_count_; }

private:
    [[nodiscard]] std::optional<Detection> best_detection(
        const std::vector<Detection>& detections, ObjectClass object_class) const;
    [[nodiscard]] std::optional<Detection> best_collectible(
        const std::vector<Detection>& detections) const;
    [[nodiscard]] MissionOutput drive_to(const Detection& detection, bool home) const;
    [[nodiscard]] MissionOutput output_for_state() const;
    void begin_collection_wait();

    MissionConfig config_;
    MissionState state_ = MissionState::initializing;
    std::optional<ObjectClass> active_target_;
    bool awaiting_collection_ = false;
    int collected_count_ = 0;
    int missing_target_frames_ = 0;
    int dock_frames_ = 0;
};

const char* to_string(MissionState state);

}  // namespace robot
