#include "robot/mission.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot {
namespace {

bool is_collectible(ObjectClass object_class) {
    return object_class == ObjectClass::yellow || object_class == ObjectClass::red;
}

}  // namespace

MissionController::MissionController(MissionConfig config) : config_(config) {
    if (config_.expected_collectibles < 0 || config_.frames_to_confirm_collection < 1 ||
        config_.frames_to_confirm_dock < 1 || config_.max_lost_target_frames < 1 ||
        config_.collector_percent < -100 || config_.collector_percent > 100 ||
        config_.target_center_x < 0 || config_.target_center_x > 1 ||
        config_.alignment_deadband < 0 || config_.target_filter_alpha <= 0 || config_.target_filter_alpha > 1 ||
        config_.target_jump_threshold <= 0 ||
        config_.steering_gain <= 0 || config_.max_yaw_radps <= 0 || config_.turn_in_place_error <= 0) {
        throw std::invalid_argument("invalid mission configuration");
    }
}

std::optional<Detection> MissionController::best_detection(
    const std::vector<Detection>& detections, ObjectClass object_class) const {
    std::optional<Detection> best;
    for (const Detection& detection : detections) {
        if (detection.object_class != object_class || detection.confidence < config_.min_confidence) continue;
        if (!best || detection.confidence > best->confidence ||
            (detection.confidence == best->confidence && detection.bottom_y > best->bottom_y)) {
            best = detection;
        }
    }
    return best;
}

std::optional<Detection> MissionController::best_collectible(const std::vector<Detection>& detections) const {
    std::optional<Detection> best;
    for (const Detection& detection : detections) {
        if (!is_collectible(detection.object_class) || already_collected(detection.object_class) ||
            detection.confidence < config_.min_confidence) continue;
        // Prefer closer objects; confidence breaks ties.
        if (!best || detection.bottom_y > best->bottom_y ||
            (detection.bottom_y == best->bottom_y && detection.confidence > best->confidence)) {
            best = detection;
        }
    }
    return best;
}

bool MissionController::already_collected(ObjectClass object_class) const {
    if (object_class == ObjectClass::yellow) return collected_types_[0];
    if (object_class == ObjectClass::red) return collected_types_[1];
    return false;
}

Detection MissionController::stabilize_target(const Detection& detection) {
    if (!filtered_target_ || filtered_target_->object_class != detection.object_class) {
        filtered_target_ = detection;
        return detection;
    }
    if (std::abs(detection.center_x - filtered_target_->center_x) >= config_.target_jump_threshold) {
        filtered_target_ = detection;
        return detection;
    }
    const double alpha = config_.target_filter_alpha;
    filtered_target_->confidence = detection.confidence;
    filtered_target_->center_x = (1.0 - alpha) * filtered_target_->center_x + alpha * detection.center_x;
    filtered_target_->bottom_y = (1.0 - alpha) * filtered_target_->bottom_y + alpha * detection.bottom_y;
    return *filtered_target_;
}

MissionOutput MissionController::drive_to(const Detection& detection, bool home) const {
    MissionOutput output = output_for_state();
    double horizontal_error = std::clamp(detection.center_x, 0.0, 1.0) - config_.target_center_x;
    if (std::abs(horizontal_error) <= config_.alignment_deadband) horizontal_error = 0.0;
    output.yaw_radps = std::clamp(-config_.steering_gain * horizontal_error,
                                  -config_.max_yaw_radps, config_.max_yaw_radps);
    const double nominal_speed = detection.bottom_y >= (home ? config_.home_dock_bottom_y : config_.collect_bottom_y)
                                     ? config_.final_approach_mps
                                     : config_.cruise_mps;
    // Prevent the camera's lateral offset from producing a pass-by: rotate
    // first for a large error, then progressively release forward motion.
    const double alignment = std::abs(horizontal_error);
    output.forward_mps = alignment >= config_.turn_in_place_error
                             ? 0.0
                             : nominal_speed * std::clamp(1.0 - alignment / config_.turn_in_place_error, .20, 1.0);
    return output;
}

MissionOutput MissionController::output_for_state() const {
    MissionOutput output;
    output.state = state_;
    output.collector_percent = state_ == MissionState::dumping || state_ == MissionState::done ||
                                       state_ == MissionState::fault
                                   ? 0
                                   : config_.collector_percent;
    return output;
}

void MissionController::begin_collection_wait() {
    awaiting_collection_ = true;
    missing_target_frames_ = 0;
}

MissionOutput MissionController::update(const MissionInput& input) {
    if (!input.localization_valid && state_ != MissionState::initializing && state_ != MissionState::fault) {
        state_ = MissionState::fault;
    }
    if (state_ == MissionState::initializing) {
        if (input.localization_valid) state_ = MissionState::searching;
        return output_for_state();
    }
    if (state_ == MissionState::fault) {
        MissionOutput output = output_for_state();
        output.emergency_stop = true;
        return output;
    }
    if (state_ == MissionState::done) return output_for_state();

    const auto obstacle = best_detection(input.detections, ObjectClass::other_robot);
    // A nearby robot always overrides pursuit, except once the vehicle is
    // stationary in the dumping state.
    if (obstacle && obstacle->bottom_y >= config_.obstacle_bottom_y && state_ != MissionState::dumping) {
        state_ = MissionState::avoiding_robot;
        MissionOutput output = output_for_state();
        output.left_mps = obstacle->center_x < .5 ? config_.avoid_left_mps : -config_.avoid_left_mps;
        output.yaw_radps = obstacle->center_x < .5 ? config_.avoid_yaw_radps : -config_.avoid_yaw_radps;
        return output;
    }
    if (state_ == MissionState::avoiding_robot) state_ = MissionState::searching;

    if (state_ == MissionState::searching || state_ == MissionState::approaching_target) {
        const auto target = best_collectible(input.detections);
        if (target) {
            if (active_target_ != target->object_class) filtered_target_.reset();
            active_target_ = target->object_class;
            state_ = MissionState::approaching_target;
            const Detection stabilized = stabilize_target(*target);
            if (stabilized.bottom_y >= config_.collect_bottom_y) begin_collection_wait();
            return drive_to(stabilized, false);
        }

        if (awaiting_collection_) {
            ++missing_target_frames_;
            if (input.collection_sensor_triggered || missing_target_frames_ >= config_.frames_to_confirm_collection) {
                if (active_target_ && !already_collected(*active_target_)) {
                    collected_types_[*active_target_ == ObjectClass::yellow ? 0 : 1] = true;
                    ++collected_count_;
                }
                awaiting_collection_ = false;
                active_target_.reset();
                filtered_target_.reset();
                missing_target_frames_ = 0;
            }
        }
        if (config_.expected_collectibles > 0 && collected_count_ >= config_.expected_collectibles) {
            state_ = MissionState::returning_home;
        } else {
            state_ = MissionState::searching;
            MissionOutput output = output_for_state();
            output.yaw_radps = config_.search_yaw_radps;
            return output;
        }
    }

    if (state_ == MissionState::returning_home || state_ == MissionState::docking_home) {
        const auto home = best_detection(input.detections, ObjectClass::home);
        if (!home) {
            state_ = MissionState::returning_home;
            MissionOutput output = output_for_state();
            output.yaw_radps = config_.search_yaw_radps;
            return output;
        }
        if (!active_target_ || *active_target_ != ObjectClass::home) filtered_target_.reset();
        active_target_ = ObjectClass::home;
        const Detection stabilized_home = stabilize_target(*home);
        state_ = MissionState::docking_home;
        if (stabilized_home.bottom_y >= config_.home_dock_bottom_y) ++dock_frames_;
        else dock_frames_ = 0;
        if (dock_frames_ >= config_.frames_to_confirm_dock) {
            state_ = MissionState::dumping;
            MissionOutput output = output_for_state();
            output.servo_pulse_us = config_.dump_servo_pulse_us;
            return output;
        }
        return drive_to(stabilized_home, true);
    }

    if (state_ == MissionState::dumping) {
        state_ = MissionState::done;
        MissionOutput output = output_for_state();
        output.servo_pulse_us = config_.dump_servo_pulse_us;
        return output;
    }
    return output_for_state();
}

const char* to_string(MissionState state) {
    switch (state) {
        case MissionState::initializing: return "initializing";
        case MissionState::searching: return "searching";
        case MissionState::approaching_target: return "approaching_target";
        case MissionState::avoiding_robot: return "avoiding_robot";
        case MissionState::returning_home: return "returning_home";
        case MissionState::docking_home: return "docking_home";
        case MissionState::dumping: return "dumping";
        case MissionState::done: return "done";
        case MissionState::fault: return "fault";
    }
    return "unknown";
}

}  // namespace robot
