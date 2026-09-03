#include "robot/config/runtime_config.hpp"

#include <stdexcept>

#include <opencv2/core.hpp>

namespace robot {
namespace {

template <typename T>
void read(const cv::FileNode& parent, const char* key, T& value) {
    const cv::FileNode node = parent[key];
    if (!node.empty()) node >> value;
}

void validate(const RuntimeConfig& config) {
    if (config.telemetry_hz <= 0 || config.capture_width <= 0 || config.capture_height <= 0 ||
        config.capture_fps <= 0 || config.visual_width <= 0 || config.visual_height <= 0 ||
        config.particles < 20) {
        throw std::runtime_error("robot configuration has invalid rates, dimensions, or particle count");
    }
    if (config.servo_firmware_min_pulse_us > config.servo_operational_min_pulse_us ||
        config.servo_operational_min_pulse_us > config.servo_operational_max_pulse_us ||
        config.servo_operational_max_pulse_us > config.servo_firmware_max_pulse_us) {
        throw std::runtime_error("operational servo range must remain inside the firmware range");
    }
    if (config.initial_x_m < 0 || config.initial_x_m > config.arena_length_m ||
        config.initial_y_m < 0 || config.initial_y_m > config.arena_width_m) {
        throw std::runtime_error("configured initial pose lies outside the arena");
    }
}

}  // namespace

RuntimeConfig load_runtime_config(const std::string& path) {
    cv::FileStorage file(path, cv::FileStorage::READ);
    if (!file.isOpened()) throw std::runtime_error("cannot open robot configuration: " + path);
    RuntimeConfig config;
    const cv::FileNode transport = file["transport"];
    read(transport, "socket", config.socket_path);
    read(transport, "serial_device", config.serial_device);
    read(transport, "baud_rate", config.baud_rate);
    read(transport, "telemetry_hz", config.telemetry_hz);
    read(transport, "command_timeout_ms", config.command_timeout_ms);
    const cv::FileNode camera = file["camera"];
    read(camera, "device", config.camera_device);
    read(camera, "calibration", config.camera_calibration);
    read(camera, "capture_width", config.capture_width);
    read(camera, "capture_height", config.capture_height);
    read(camera, "capture_fps", config.capture_fps);
    read(camera, "visual_width", config.visual_width);
    read(camera, "visual_height", config.visual_height);
    read(camera, "record_fps", config.record_fps);
    const cv::FileNode localization = file["localization"];
    int particles = static_cast<int>(config.particles);
    read(localization, "particles", particles);
    config.particles = static_cast<std::size_t>(particles);
    read(localization, "camera_height_m", config.camera_height_m);
    read(localization, "camera_pitch_deg", config.camera_pitch_deg);
    read(localization, "camera_roll_deg", config.camera_roll_deg);
    read(localization, "initial_x_m", config.initial_x_m);
    read(localization, "initial_y_m", config.initial_y_m);
    read(localization, "initial_yaw_deg", config.initial_yaw_deg);
    const cv::FileNode arena = file["arena"];
    read(arena, "length_m", config.arena_length_m);
    read(arena, "width_m", config.arena_width_m);
    read(arena, "fence_height_m", config.fence_height_m);
    const cv::FileNode servo = file["servo"];
    read(servo, "operational_min_pulse_us", config.servo_operational_min_pulse_us);
    read(servo, "operational_max_pulse_us", config.servo_operational_max_pulse_us);
    read(servo, "firmware_min_pulse_us", config.servo_firmware_min_pulse_us);
    read(servo, "firmware_max_pulse_us", config.servo_firmware_max_pulse_us);
    const cv::FileNode control = file["control"];
    read(control, "max_linear_mps", config.max_linear_mps);
    read(control, "max_yaw_radps", config.max_yaw_radps);
    read(control, "max_linear_accel_mps2", config.max_linear_accel_mps2);
    read(control, "max_yaw_accel_radps2", config.max_yaw_accel_radps2);
    const cv::FileNode detector = file["detector"];
    read(detector, "backend", config.detector_backend);
    read(detector, "model", config.detector_model);
    read(detector, "confidence_threshold", config.detector_confidence);
    read(detector, "nms_threshold", config.detector_nms);
    const cv::FileNode debug = file["debug"];
    int broadcast_enabled = config.debug_broadcast_enabled ? 1 : 0;
    read(debug, "broadcast_enabled", broadcast_enabled);
    config.debug_broadcast_enabled = broadcast_enabled != 0;
    read(debug, "broadcast_address", config.debug_broadcast_address);
    read(debug, "broadcast_port", config.debug_broadcast_port);
    if (config.debug_broadcast_port <= 0 || config.debug_broadcast_port > 65535) {
        throw std::runtime_error("debug broadcast port must be in 1..65535");
    }
    validate(config);
    return config;
}

}  // namespace robot
