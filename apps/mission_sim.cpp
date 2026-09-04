#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot/mission.hpp"

namespace {

robot::Detection detection(robot::ObjectClass type, double x, double bottom) {
    return {.object_class = type, .confidence = .95, .center_x = x, .bottom_y = bottom};
}

void print_step(const char* label, robot::MissionController& controller,
                const robot::MissionInput& input) {
    const robot::MissionOutput output = controller.update(input);
    std::cout << label << " state=" << robot::to_string(output.state)
              << " twist=" << output.forward_mps << ',' << output.left_mps << ',' << output.yaw_radps
              << " collector=" << output.collector_percent;
    if (output.servo_pulse_us) std::cout << " s3=" << *output.servo_pulse_us;
    std::cout << '\n';
}

}  // namespace

int main() {
    try {
        robot::MissionConfig config;
        config.expected_collectibles = 2;
        config.frames_to_confirm_collection = 2;
        config.frames_to_confirm_dock = 2;
        robot::MissionController controller(config);
        print_step("init", controller, {.localization_valid = true, .detections = {}});
        print_step("yellow", controller, {.localization_valid = true,
                                             .detections = {detection(robot::ObjectClass::yellow, .60, .45)}});
        print_step("avoid", controller, {.localization_valid = true,
                                            .detections = {detection(robot::ObjectClass::other_robot, .30, .60)}});
        print_step("collect-yellow", controller, {.localization_valid = true,
                                                     .detections = {detection(robot::ObjectClass::yellow, .50, .90)}});
        print_step("yellow-gone-1", controller, {.localization_valid = true, .detections = {}});
        print_step("yellow-gone-2", controller, {.localization_valid = true, .detections = {}});
        print_step("red", controller, {.localization_valid = true,
                                          .detections = {detection(robot::ObjectClass::red, .42, .90)}});
        print_step("red-gone-1", controller, {.localization_valid = true, .detections = {}});
        print_step("red-gone-2", controller, {.localization_valid = true, .detections = {}});
        print_step("home-1", controller, {.localization_valid = true,
                                             .detections = {detection(robot::ObjectClass::home, .50, .90)}});
        print_step("home-2", controller, {.localization_valid = true,
                                             .detections = {detection(robot::ObjectClass::home, .50, .90)}});
        print_step("dump", controller, {.localization_valid = true,
                                           .detections = {detection(robot::ObjectClass::home, .50, .90)}});
        return controller.state() == robot::MissionState::done ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "mission_sim: " << error.what() << '\n';
        return 1;
    }
}
