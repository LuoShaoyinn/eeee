#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "robot/localization/location.hpp"
#include "robot/config/runtime_config.hpp"
#include "robot/runtime/latest_value.hpp"
#include "robot/perception/object_projection.hpp"
#include "robot/planning/approach_controller.hpp"
#include "robot/planning/world_model.hpp"
#include "robot/planning/search_controller.hpp"
#include "robot/planning/vehicle_geometry.hpp"
#ifdef ROBOT_A733_NPU
#include "robot/perception/a733_detector.hpp"
#endif

namespace {
constexpr double kDegreesToRadians = CV_PI / 180.0;
volatile std::sig_atomic_t g_running = 1;

struct Options {
    std::string config = "config/robot.yaml";
    std::string camera = "/dev/video0";
    std::string socket = "/tmp/robotd.sock";
    std::string calibration = "config/camera_fisheye_1280x720.yaml";
    std::string log_path;
    std::string video_path;
    std::string raw_video_path;
    std::string telemetry_replay_path;
    bool record_video = true;
    bool record_raw_video = false;
    bool rectified_input = false;
    bool realtime_video = false;
    int visual_width = 320;
    int visual_height = 180;
    size_t particles = 600;
    int max_frames = 0;
    double height_m = .12910;
    double pitch_deg = 30.0296;
    double roll_deg = .2071;
    double initial_x_m = .10;
    double initial_y_m = .10;
    double initial_yaw_deg = 0;
    bool global_initialize = false;
    int capture_width = 1280;
    int capture_height = 720;
    double capture_fps = 30;
    double record_fps = 10;
    double visual_geometry_hz = 1;
    double telemetry_hz = 25;
    double visual_pull_gain = .12;
    double visual_precise_residual_m = .010;
    double visual_residual_limit_m = .045;
    double visual_yaw_reset_max_error_rad = 15.0 * kDegreesToRadians;
    double visual_axis_certainty_min = .05;
    double visual_axis_sigma_m = .15;
    double visual_yaw_sigma_rad = 5.0 * kDegreesToRadians;
    double visual_axis_max_correction_m = 1.0;
    double visual_axis_max_correction_rad = 30.0 * kDegreesToRadians;
    double visual_axis_max_pull_gain = .80;
    double fence_height_m = .254;
    cv::Scalar fence_hsv_lower{96, 128, 82};
    cv::Scalar fence_hsv_upper{121, 255, 255};
    bool stream_json = false;
    bool broadcast_enabled = true;
    std::string broadcast_address = "255.255.255.255";
    int broadcast_port = 3335;
    std::string detector_model;
    double detector_hz = 30;
    robot::ApproachControllerConfig approach;
};

struct EspState {
    std::uint64_t ms = 0;
    std::uint64_t imu_age_ms = 0;
    double gyro_z_degps = 0;
    std::array<double, 4> rpm{};
    std::array<double, 4> targets{};
};

struct TimedEspState {
    EspState state;
    std::chrono::steady_clock::time_point timestamp{};
    std::uint64_t sequence = 0;
};

struct ReplaySample {
    EspState state;
    std::uint64_t monotonic_ns = 0;
};

std::vector<ReplaySample> load_telemetry_replay(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot read telemetry replay: " + path);
    std::vector<ReplaySample> samples;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        cv::FileStorage json(line, cv::FileStorage::READ | cv::FileStorage::MEMORY |
                                      cv::FileStorage::FORMAT_JSON);
        ReplaySample sample;
        EspState& state = sample.state;
        double esp_ms = 0;
        double imu_age_ms = 0;
        json["esp_ms"] >> esp_ms;
        json["imu_age_ms"] >> imu_age_ms;
        state.ms = static_cast<std::uint64_t>(std::max(esp_ms, 0.0));
        state.imu_age_ms = static_cast<std::uint64_t>(std::max(imu_age_ms, 0.0));
        double monotonic_ns = 0;
        json["monotonic_ns"] >> monotonic_ns;
        sample.monotonic_ns = static_cast<std::uint64_t>(std::max(monotonic_ns, 0.0));
        json["gyro_z_degps"] >> state.gyro_z_degps;
        std::vector<double> rpm, targets;
        json["rpm"] >> rpm;
        json["targets"] >> targets;
        if (rpm.size() != state.rpm.size() || targets.size() != state.targets.size()) {
            throw std::runtime_error("invalid telemetry replay row " +
                                     std::to_string(samples.size() + 1));
        }
        std::copy(rpm.begin(), rpm.end(), state.rpm.begin());
        std::copy(targets.begin(), targets.end(), state.targets.begin());
        samples.push_back(sample);
    }
    if (samples.empty()) throw std::runtime_error("telemetry replay is empty: " + path);
    return samples;
}

void handle_signal(int) { g_running = 0; }

std::string request_robotd(const std::string& path, const std::string& request) {
    const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) throw std::runtime_error("cannot create robotd socket");
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (path.size() >= sizeof(address.sun_path)) {
        close(fd);
        throw std::runtime_error("robotd socket path is too long");
    }
    std::strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1);
    if (connect(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        close(fd);
        throw std::runtime_error("cannot connect to robotd at " + path);
    }
    const std::string command = request + "\n";
    if (write(fd, command.data(), command.size()) != static_cast<ssize_t>(command.size())) {
        close(fd);
        throw std::runtime_error("cannot send robotd request");
    }
    std::string reply;
    char buffer[512];
    for (;;) {
        const ssize_t count = read(fd, buffer, sizeof(buffer));
        if (count <= 0) break;
        reply.append(buffer, static_cast<size_t>(count));
    }
    close(fd);
    if (reply.rfind("error:", 0) == 0) throw std::runtime_error(reply);
    return reply;
}

bool parse_state(const std::string& reply, EspState& state) {
    std::istringstream input(reply);
    std::string key;
    if (!(input >> key) || key != "state") return false;
    while (input >> key) {
        if (key == "ms") input >> state.ms;
        else if (key == "imu_age") input >> state.imu_age_ms;
        else if (key == "gyro") { double unused; input >> unused >> unused >> state.gyro_z_degps; }
        else if (key == "angle") { double unused; input >> unused >> unused >> unused; }
        else if (key == "rpm") { for (double& value : state.rpm) input >> value; }
        else if (key == "fg") { unsigned unused; for (int index = 0; index < 4; ++index) input >> unused; }
        else if (key == "ga25") { long long unused; input >> unused >> unused >> unused >> unused; }
        else return false;
    }
    return true;
}

bool parse_telemetry_targets(const std::string& reply, std::array<double, 4>& targets) {
    std::string normalized = reply;
    std::replace(normalized.begin(), normalized.end(), ';', ' ');
    std::istringstream input(normalized);
    std::string key;
    while (input >> key) {
        if (key == "target") {
            for (double& value : targets) input >> value;
            return static_cast<bool>(input);
        }
        std::string ignored;
        std::getline(input, ignored);
    }
    return false;
}

class TelemetrySampler {
public:
    TelemetrySampler(std::string socket_path, double frequency_hz)
        : socket_path_(std::move(socket_path)), period_(std::chrono::duration<double>(1.0 / frequency_hz)) {}

    ~TelemetrySampler() { stop(); }

    void start() { thread_ = std::thread(&TelemetrySampler::run, this); }

    void stop() {
        stopping_ = true;
        if (thread_.joinable()) thread_.join();
    }

    std::optional<TimedEspState> latest() const {
        std::scoped_lock lock(mutex_);
        return latest_;
    }

private:
    void run() {
        auto next = std::chrono::steady_clock::now();
        auto next_error_log = next;
        while (!stopping_) {
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period_);
            try {
                TimedEspState sample;
                if (!parse_state(request_robotd(socket_path_, "state"), sample.state)) {
                    throw std::runtime_error("invalid ESP state reply");
                }
                if (!parse_telemetry_targets(request_robotd(socket_path_, "telemetry"), sample.state.targets)) {
                    throw std::runtime_error("invalid ESP telemetry reply");
                }
                sample.timestamp = std::chrono::steady_clock::now();
                sample.sequence = ++sequence_;
                std::scoped_lock lock(mutex_);
                latest_ = sample;
            } catch (const std::exception& error) {
                const auto now = std::chrono::steady_clock::now();
                if (now >= next_error_log) {
                    std::cerr << "robot-runtime: passive telemetry unavailable: " << error.what() << '\n';
                    next_error_log = now + std::chrono::seconds(2);
                }
            }
            std::this_thread::sleep_until(next);
        }
    }

    std::string socket_path_;
    std::chrono::duration<double> period_;
    mutable std::mutex mutex_;
    std::optional<TimedEspState> latest_;
    std::thread thread_;
    std::atomic_bool stopping_{false};
    std::uint64_t sequence_ = 0;
};

class DebugBroadcaster {
public:
    DebugBroadcaster(bool enabled, const std::string& address, int port) {
        if (!enabled) return;
        fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (fd_ < 0) throw std::runtime_error("cannot create debug broadcast socket");
        const int enabled_flag = 1;
        if (setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &enabled_flag, sizeof(enabled_flag)) != 0) {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error("cannot enable UDP broadcast");
        }
        destination_.sin_family = AF_INET;
        destination_.sin_port = htons(static_cast<std::uint16_t>(port));
        if (inet_pton(AF_INET, address.c_str(), &destination_.sin_addr) != 1) {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error("invalid debug broadcast address: " + address);
        }
    }

    ~DebugBroadcaster() { if (fd_ >= 0) close(fd_); }

    void send(const std::string& record) const {
        if (fd_ < 0) return;
        (void)sendto(fd_, record.data(), record.size(), MSG_DONTWAIT,
                     reinterpret_cast<const sockaddr*>(&destination_), sizeof(destination_));
    }

private:
    int fd_ = -1;
    sockaddr_in destination_{};
};

cv::Mat scaled_camera_matrix(const cv::Mat& matrix, int width, int height) {
    cv::Mat result;
    matrix.convertTo(result, CV_64F);
    const double sx = static_cast<double>(width) / 1280.0;
    const double sy = static_cast<double>(height) / 720.0;
    result.at<double>(0, 0) *= sx;
    result.at<double>(0, 2) *= sx;
    result.at<double>(1, 1) *= sy;
    result.at<double>(1, 2) *= sy;
    return result;
}

struct FenceEdges {
    std::vector<cv::Point2d> lower;
    std::vector<cv::Point2d> upper;

    std::vector<cv::Point2d> combined() const {
        std::vector<cv::Point2d> result = lower;
        result.insert(result.end(), upper.begin(), upper.end());
        return result;
    }
};

struct FenceRequest {
    cv::Mat frame;
    robot::Timestamp timestamp;
    std::uint64_t sequence = 0;
    double yaw_prior_rad = 0;
    robot::Pose2 odometry_pose;
};

struct FenceResult {
    robot::Timestamp timestamp;
    std::uint64_t sequence = 0;
    FenceEdges edges;
    std::vector<cv::Point2d> observations;
    robot::VisualGeometryEstimate geometry;
    robot::Pose2 odometry_pose;
};

FenceEdges fence_edge_points(const cv::Mat& rectified,
                             const robot::GroundProjector& projector,
                             const cv::Scalar& hsv_lower,
                             const cv::Scalar& hsv_upper,
                             double fence_height_m) {
    cv::Mat hsv, mask;
    cv::cvtColor(rectified, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsv_lower, hsv_upper, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 3)));
    FenceEdges points;
    for (int x = 0; x < mask.cols; x += 4) {
        int upper_y = -1;
        int lower_y = -1;
        for (int row = 0; row < mask.rows; ++row) {
            if (mask.at<unsigned char>(row, x) == 0) continue;
            if (upper_y < 0) upper_y = row;
            lower_y = row;
        }
        cv::Point2d ground;
        if (lower_y - upper_y < 3) continue;
        // A mask contour coincident with the image border is clipping, not an
        // observed fence edge. Side-border intersections remain valid samples.
        if (lower_y < mask.rows - 2 &&
            projector.project(cv::Point2f(static_cast<float>(x), static_cast<float>(lower_y)), ground)) {
            points.lower.push_back(ground);
        }
        if (upper_y > 1 && projector.project_to_height(
                cv::Point2f(static_cast<float>(x), static_cast<float>(upper_y)),
                fence_height_m, ground)) {
            points.upper.push_back(ground);
        }
    }
    return points;
}

class FenceWorker {
public:
    FenceWorker(const robot::GroundProjector& projector, cv::Scalar hsv_lower,
                cv::Scalar hsv_upper, double fence_height_m, double frequency_hz)
        : projector_(projector), hsv_lower_(hsv_lower), hsv_upper_(hsv_upper),
          fence_height_m_(fence_height_m),
          period_(std::chrono::duration<double>(1.0 / frequency_hz)) {}

    ~FenceWorker() { stop(); }

    void start() { thread_ = std::thread(&FenceWorker::run, this); }
    void submit(FenceRequest request) { requests_.publish(std::move(request)); }
    std::optional<FenceResult> take() { return results_.take(); }

    void stop() {
        stopping_ = true;
        if (thread_.joinable()) thread_.join();
    }

private:
    void run() {
        auto next = std::chrono::steady_clock::now();
        while (!stopping_) {
            if (const auto request = requests_.take()) {
                FenceResult result;
                result.timestamp = request->timestamp;
                result.sequence = request->sequence;
                result.odometry_pose = request->odometry_pose;
                result.edges = fence_edge_points(request->frame, projector_, hsv_lower_,
                                                  hsv_upper_, fence_height_m_);
                result.observations = result.edges.combined();
                result.geometry = robot::estimate_fence_geometry(
                    result.observations, request->yaw_prior_rad);
                results_.publish(std::move(result));
            }
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period_);
            std::this_thread::sleep_until(next);
        }
    }

    robot::GroundProjector projector_;
    cv::Scalar hsv_lower_;
    cv::Scalar hsv_upper_;
    double fence_height_m_;
    std::chrono::duration<double> period_;
    robot::LatestValue<FenceRequest> requests_;
    robot::LatestValue<FenceResult> results_;
    std::atomic_bool stopping_{false};
    std::thread thread_;
};

#ifdef ROBOT_A733_NPU
struct DetectorRequest { cv::Mat frame; robot::Timestamp timestamp; std::uint64_t sequence; };

class DetectorWorker {
public:
    DetectorWorker(std::unique_ptr<robot::Detector> detector, double frequency_hz)
        : detector_(std::move(detector)), period_(1.0 / frequency_hz) {}
    ~DetectorWorker() { stopping_ = true; if (thread_.joinable()) thread_.join(); }
    void start() { thread_ = std::thread(&DetectorWorker::run, this); }
    void submit(DetectorRequest request) { requests_.publish(std::move(request)); }
    std::optional<robot::DetectionFrame> take() { return results_.take(); }
private:
    void run() {
        auto next = std::chrono::steady_clock::now();
        while (!stopping_) {
            if (auto request = requests_.take()) {
                results_.publish(detector_->detect(request->frame, request->timestamp,
                                                   request->sequence));
            }
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period_);
            std::this_thread::sleep_until(next);
        }
    }
    std::unique_ptr<robot::Detector> detector_;
    std::chrono::duration<double> period_;
    robot::LatestValue<DetectorRequest> requests_;
    robot::LatestValue<robot::DetectionFrame> results_;
    std::atomic_bool stopping_{false};
    std::thread thread_;
};
#endif

std::string default_log_path() {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::ostringstream name;
    name << "run-log/location-" << std::put_time(&local, "%Y%m%d-%H%M%S") << ".jsonl";
    return name.str();
}

std::string default_video_path(const std::string& log_path) {
    std::filesystem::path path(log_path);
    path.replace_extension(".avi");
    return path.string();
}

Options parse_options(int argc, char** argv) {
    std::string config_path = "config/robot.yaml";
    for (int index = 1; index + 1 < argc; ++index) {
        if (std::string(argv[index]) == "--config") config_path = argv[index + 1];
    }
    const robot::RuntimeConfig config = robot::load_runtime_config(config_path);
    Options options;
    options.config = config_path;
    options.camera = config.camera_device;
    options.socket = config.socket_path;
    options.calibration = config.camera_calibration;
    options.visual_width = config.visual_width;
    options.visual_height = config.visual_height;
    options.particles = config.particles;
    options.height_m = config.camera_height_m;
    options.pitch_deg = config.camera_pitch_deg;
    options.roll_deg = config.camera_roll_deg;
    options.initial_x_m = config.initial_x_m;
    options.initial_y_m = config.initial_y_m;
    options.initial_yaw_deg = config.initial_yaw_deg;
    options.capture_width = config.capture_width;
    options.capture_height = config.capture_height;
    options.capture_fps = config.capture_fps;
    options.record_fps = config.record_fps;
    options.visual_geometry_hz = config.visual_geometry_hz;
    options.telemetry_hz = config.telemetry_hz;
    options.visual_pull_gain = config.visual_pull_gain;
    options.visual_precise_residual_m = config.visual_precise_residual_m;
    options.visual_residual_limit_m = config.visual_residual_limit_m;
    options.visual_yaw_reset_max_error_rad =
        config.visual_yaw_reset_max_error_deg * kDegreesToRadians;
    options.visual_axis_certainty_min = config.visual_axis_certainty_min;
    options.visual_axis_sigma_m = config.visual_axis_sigma_m;
    options.visual_yaw_sigma_rad = config.visual_yaw_sigma_deg * kDegreesToRadians;
    options.visual_axis_max_correction_m = config.visual_axis_max_correction_m;
    options.visual_axis_max_correction_rad =
        config.visual_axis_max_correction_deg * kDegreesToRadians;
    options.visual_axis_max_pull_gain = config.visual_axis_max_pull_gain;
    options.fence_height_m = config.fence_height_m;
    options.fence_hsv_lower = cv::Scalar(config.fence_hsv_h_min, config.fence_hsv_s_min,
                                         config.fence_hsv_v_min);
    options.fence_hsv_upper = cv::Scalar(config.fence_hsv_h_max, config.fence_hsv_s_max,
                                         config.fence_hsv_v_max);
    options.broadcast_enabled = config.debug_broadcast_enabled;
    options.broadcast_address = config.debug_broadcast_address;
    options.broadcast_port = config.debug_broadcast_port;
    options.detector_model = config.detector_model;
    options.detector_hz = config.detector_inference_hz;
    options.approach.translation_kp = config.approach_translation_kp;
    options.approach.translation_ki = config.approach_translation_ki;
    options.approach.translation_kd = config.approach_translation_kd;
    options.approach.yaw_kp = config.approach_yaw_kp;
    options.approach.yaw_kd = config.approach_yaw_kd;
    options.approach.maximum_linear_mps = config.approach_maximum_linear_mps;
    options.approach.maximum_yaw_radps = config.approach_maximum_yaw_radps;
    options.approach.maximum_linear_accel_mps2 = config.approach_maximum_linear_accel_mps2;
    options.approach.maximum_yaw_accel_radps2 = config.approach_maximum_yaw_accel_radps2;
    options.approach.stopping_distance_m = config.approach_stopping_distance_m;
    options.approach.target_timeout = std::chrono::milliseconds(config.approach_target_timeout_ms);
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        const auto value = [&](const char* name) -> const char* {
            if (index + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + name);
            return argv[++index];
        };
        if (argument == "--config") ++index;
        else if (argument == "--camera") options.camera = value("--camera");
        else if (argument == "--socket") options.socket = value("--socket");
        else if (argument == "--calibration") options.calibration = value("--calibration");
        else if (argument == "--log") options.log_path = value("--log");
        else if (argument == "--video") options.video_path = value("--video");
        else if (argument == "--raw-video") {
            options.raw_video_path = value("--raw-video");
            options.record_raw_video = true;
        }
        else if (argument == "--telemetry-replay") {
            options.telemetry_replay_path = value("--telemetry-replay");
        }
        else if (argument == "--realtime-video") options.realtime_video = true;
        else if (argument == "--no-video") options.record_video = false;
        else if (argument == "--rectified-input") options.rectified_input = true;
        else if (argument == "--visual-width") options.visual_width = std::stoi(value("--visual-width"));
        else if (argument == "--visual-height") options.visual_height = std::stoi(value("--visual-height"));
        else if (argument == "--particles") options.particles = std::stoul(value("--particles"));
        else if (argument == "--max-frames") options.max_frames = std::stoi(value("--max-frames"));
        else if (argument == "--height") options.height_m = std::stod(value("--height"));
        else if (argument == "--pitch") options.pitch_deg = std::stod(value("--pitch"));
        else if (argument == "--roll") options.roll_deg = std::stod(value("--roll"));
        else if (argument == "--initial-x") options.initial_x_m = std::stod(value("--initial-x"));
        else if (argument == "--initial-y") options.initial_y_m = std::stod(value("--initial-y"));
        else if (argument == "--initial-yaw") options.initial_yaw_deg = std::stod(value("--initial-yaw"));
        else if (argument == "--global-initialize") options.global_initialize = true;
        else if (argument == "--stdout-json") options.stream_json = true;
        else if (argument == "--no-broadcast") options.broadcast_enabled = false;
        else if (argument == "--help") {
            std::cout << "robot-runtime [--config FILE] [--camera PATH] [--socket PATH] [--calibration FILE] [--log FILE] [--video FILE] [--raw-video FILE] [--no-video] [--telemetry-replay FILE] [--realtime-video] "
                         "[--visual-width N] [--visual-height N] [--particles N] [--max-frames N] "
                         "[--height M] [--pitch DEG] [--roll DEG] [--initial-x M] [--initial-y M] "
                         "[--initial-yaw DEG] [--global-initialize] [--rectified-input] "
                         "[--stdout-json] [--no-broadcast]\n";
            std::exit(0);
        } else throw std::runtime_error("unknown option: " + argument);
    }
    if (options.visual_width <= 0 || options.visual_height <= 0 || options.particles < 20) {
        throw std::runtime_error("invalid image size or particle count");
    }
    if (options.initial_x_m < 0 || options.initial_x_m > 3.0 || options.initial_y_m < 0 || options.initial_y_m > 1.985) {
        throw std::runtime_error("initial pose must lie inside the field");
    }
    if (options.log_path.empty()) options.log_path = default_log_path();
    if (options.video_path.empty() && options.record_video) options.video_path = default_video_path(options.log_path);
    return options;
}

void write_record(std::ostream& log, int frame_index, std::uint64_t time_ns, const EspState& state,
                  bool telemetry_valid, std::uint64_t telemetry_sequence, double telemetry_age_ms,
                  const robot::BodyVelocity& wheel, const robot::VisualMotion& visual,
                  const robot::BodyVelocity& fused, const robot::Pose2& odometry_pose,
                  const robot::PoseEstimate& pose,
                  const robot::VisualGeometryEstimate& visual_geometry,
                  size_t lower_fence_count, size_t upper_fence_count,
                  bool visual_certain, bool visual_very_certain,
                  bool imu_yaw_reset, double gyro_bias_degps,
                  const std::vector<robot::TrackedObject>& objects,
                  const robot::ApproachResult& approach,
                  const robot::SearchResult& search,
                  const robot::HomeObservation& home) {
    log << std::fixed << std::setprecision(6)
        << "{\"frame_index\":" << frame_index << ",\"monotonic_ns\":" << time_ns
        << ",\"telemetry_valid\":" << (telemetry_valid ? "true" : "false")
        << ",\"telemetry_sequence\":" << telemetry_sequence
        << ",\"telemetry_age_ms\":" << telemetry_age_ms
        << ",\"esp_ms\":" << state.ms << ",\"imu_age_ms\":" << state.imu_age_ms
        << ",\"gyro_z_degps\":" << state.gyro_z_degps
        << ",\"rpm\":[" << state.rpm[0] << ',' << state.rpm[1] << ',' << state.rpm[2] << ',' << state.rpm[3] << ']'
        << ",\"targets\":[" << state.targets[0] << ',' << state.targets[1] << ',' << state.targets[2] << ',' << state.targets[3] << ']'
        << ",\"wheel\":[" << wheel.forward_mps << ',' << wheel.left_mps << ',' << wheel.yaw_radps << ']'
        << ",\"visual\":{\"valid\":" << (visual.valid ? "true" : "false")
        << ",\"features\":" << visual.tracked_features << ",\"velocity\":[" << visual.velocity.forward_mps
        << ',' << visual.velocity.left_mps << ',' << visual.velocity.yaw_radps << "]}"
        << ",\"fused\":[" << fused.forward_mps << ',' << fused.left_mps << ',' << fused.yaw_radps << ']'
        << ",\"odometry_pose\":[" << odometry_pose.x_m << ',' << odometry_pose.y_m << ','
        << odometry_pose.yaw_rad << ']'
        << ",\"pose\":[" << pose.x_m << ',' << pose.y_m << ',' << pose.yaw_rad << ']'
        << ",\"position_sigma_m\":" << pose.position_sigma_m
        << ",\"yaw_sigma_rad\":" << pose.yaw_sigma_rad
        << ",\"effective_particles\":" << pose.effective_particles
        << ",\"visual_geometry\":{\"valid\":" << (visual_geometry.valid ? "true" : "false")
        << ",\"confidence\":" << visual_geometry.confidence
        << ",\"alternative_margin_m\":" << visual_geometry.alternative_margin_m
        << ",\"sigma_major_m\":" << visual_geometry.sigma_major_m
        << ",\"sigma_minor_m\":" << visual_geometry.sigma_minor_m
        << ",\"major_axis_rad\":" << visual_geometry.major_axis_rad
        << ",\"point_support\":" << visual_geometry.point_support
        << ",\"axis_sigma\":[" << visual_geometry.axis_sigma[0] << ','
        << visual_geometry.axis_sigma[1] << ',' << visual_geometry.axis_sigma[2] << ']'
        << ",\"axis_certainty\":[" << visual_geometry.axis_certainty[0] << ','
        << visual_geometry.axis_certainty[1] << ','
        << visual_geometry.axis_certainty[2] << "]}"
        << ",\"visual_yaw_sigma_rad\":" << visual_geometry.yaw_sigma_rad
        << ",\"visual_certain\":" << (visual_certain ? "true" : "false")
        << ",\"visual_very_certain\":" << (visual_very_certain ? "true" : "false")
        << ",\"imu_yaw_reset\":" << (imu_yaw_reset ? "true" : "false")
        << ",\"gyro_bias_degps\":" << gyro_bias_degps
        << ",\"visual_geometry_candidates\":[";
    for (std::size_t index = 0; index < visual_geometry.candidates.size(); ++index) {
        if (index != 0) log << ',';
        const auto& candidate = visual_geometry.candidates[index];
        log << '[' << candidate.pose.x_m << ',' << candidate.pose.y_m << ','
            << candidate.pose.yaw_rad << ',' << candidate.wall_residual_m << ']';
    }
    log << "]"
        << ",\"lower_fence_points\":" << lower_fence_count
        << ",\"upper_fence_points\":" << upper_fence_count << ",\"objects\":[";
    for (std::size_t index = 0; index < objects.size(); ++index) {
        if (index) log << ',';
        log << "[" << static_cast<int>(objects[index].object_class) << ','
            << objects[index].x_m << ',' << objects[index].y_m << ','
            << objects[index].confidence << ']';
    }
    log << "],\"home_box\":{\"detected\":" << (home.detected ? "true" : "false")
        << ",\"consistent\":" << (home.consistent ? "true" : "false")
        << ",\"position\":[" << home.x_m << ',' << home.y_m << "]"
        << ",\"error_m\":" << home.distance_to_home_m
        << ",\"confidence\":" << home.confidence << '}'
        << ",\"auto_proposal\":{\"valid\":" << (approach.target_valid ? "true" : "false")
        << ",\"reached\":" << (approach.target_reached ? "true" : "false")
        << ",\"phase\":\"" << robot::to_string(search.phase) << "\""
        << ",\"lost_seconds\":" << search.lost_seconds
        << ",\"distance_m\":" << approach.distance_m << ",\"twist\":["
        << approach.command.forward_mps << ',' << approach.command.left_mps << ','
        << approach.command.yaw_radps << "]}}\n";
}
}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_options(argc, argv);
        cv::FileStorage calibration(options.calibration, cv::FileStorage::READ);
        if (!calibration.isOpened()) throw std::runtime_error("cannot open calibration: " + options.calibration);
        cv::Mat camera_matrix, distortion, rectified_matrix;
        calibration["K"] >> camera_matrix;
        calibration["D"] >> distortion;
        calibration["rectified_K"] >> rectified_matrix;
        if (camera_matrix.empty() || distortion.empty() || rectified_matrix.empty()) {
            throw std::runtime_error("calibration lacks K, D, or rectified_K");
        }
        const cv::Mat visual_matrix = scaled_camera_matrix(rectified_matrix, options.visual_width, options.visual_height);
        robot::GroundProjector projector(visual_matrix, options.height_m, options.pitch_deg, options.roll_deg);
        robot::GroundProjector object_projector(rectified_matrix, options.height_m,
                                                options.pitch_deg, options.roll_deg);
        cv::Mat map_x, map_y;
        cv::fisheye::initUndistortRectifyMap(camera_matrix, distortion, cv::Mat::eye(3, 3, CV_64F),
                                             rectified_matrix, cv::Size(1280, 720), CV_16SC2, map_x, map_y);
        cv::VideoCapture capture;
        if (std::filesystem::is_regular_file(options.camera)) capture.open(options.camera);
        else capture.open(options.camera, cv::CAP_V4L2);
        if (!capture.isOpened()) throw std::runtime_error("cannot open camera: " + options.camera);
        capture.set(cv::CAP_PROP_FRAME_WIDTH, options.capture_width);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, options.capture_height);
        capture.set(cv::CAP_PROP_FPS, options.capture_fps);
        const std::filesystem::path log_parent = std::filesystem::path(options.log_path).parent_path();
        if (!log_parent.empty()) std::filesystem::create_directories(log_parent);
        std::ofstream log(options.log_path);
        if (!log) throw std::runtime_error("cannot write log: " + options.log_path);
        cv::VideoWriter video;
        cv::VideoWriter raw_video;
        if (options.record_video) {
            const std::filesystem::path video_parent = std::filesystem::path(options.video_path).parent_path();
            if (!video_parent.empty()) std::filesystem::create_directories(video_parent);
            video.open(options.video_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), options.record_fps,
                       cv::Size(options.capture_width, options.capture_height));
            if (!video.isOpened()) throw std::runtime_error("cannot open video writer: " + options.video_path);
        }
        if (options.record_raw_video) {
            const std::filesystem::path raw_parent = std::filesystem::path(options.raw_video_path).parent_path();
            if (!raw_parent.empty()) std::filesystem::create_directories(raw_parent);
            raw_video.open(options.raw_video_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), options.record_fps,
                           cv::Size(options.capture_width, options.capture_height));
            if (!raw_video.isOpened()) throw std::runtime_error("cannot open raw video writer: " + options.raw_video_path);
        }
        std::cerr << "robot-runtime: PASSIVE config=" << options.config << " capture=" << options.camera
                  << " robotd=" << options.socket << " log=" << options.log_path;
        if (options.record_video) std::cerr << " video=" << options.video_path;
        if (options.record_raw_video) std::cerr << " raw_video=" << options.raw_video_path;
        std::cerr << '\n';
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
        robot::VisualOdometry visual_odometry;
        robot::FenceParticleFilter filter(options.particles, options.initial_x_m, options.initial_y_m,
                                          options.initial_yaw_deg * kDegreesToRadians, options.global_initialize);
        robot::Pose2 odometry_pose{options.initial_x_m, options.initial_y_m,
                                   options.initial_yaw_deg * kDegreesToRadians};
        std::unique_ptr<TelemetrySampler> telemetry;
        std::vector<ReplaySample> replay_telemetry;
        if (options.telemetry_replay_path.empty()) {
            telemetry = std::make_unique<TelemetrySampler>(options.socket, options.telemetry_hz);
            telemetry->start();
        } else {
            replay_telemetry = load_telemetry_replay(options.telemetry_replay_path);
        }
        DebugBroadcaster broadcaster(options.broadcast_enabled, options.broadcast_address,
                                     options.broadcast_port);
        auto previous_time = std::chrono::steady_clock::now();
        int frame_count = 0;
        robot::VisualGeometryEstimate visual_geometry;
        double gyro_bias_radps = 0;
        std::optional<double> bias_anchor_visual_yaw;
        double bias_anchor_elapsed_s = 0;
        double bias_anchor_imu_delta_rad = 0;
        FenceWorker fence_worker(projector, options.fence_hsv_lower,
                                 options.fence_hsv_upper, options.fence_height_m,
                                 options.visual_geometry_hz);
        fence_worker.start();
        std::uint64_t fence_request_sequence = 0;
        std::size_t lower_fence_count = 0;
        std::size_t upper_fence_count = 0;
        robot::WorldModel world;
        robot::ApproachController approach_controller(options.approach);
        robot::ApproachResult approach_result;
        robot::SearchController search_controller;
        robot::SearchResult search_result;
        robot::HomeObservation home_observation;
#ifdef ROBOT_A733_NPU
        DetectorWorker detector_worker(robot::make_a733_detector(options.detector_model),
                                       options.detector_hz);
        detector_worker.start();
#endif
        const auto replay_start = std::chrono::steady_clock::now();
        while (g_running && (options.max_frames == 0 || frame_count < options.max_frames)) {
            if (!replay_telemetry.empty() &&
                static_cast<std::size_t>(frame_count) >= replay_telemetry.size()) break;
            if (options.realtime_video && frame_count > 0) {
                const double elapsed_s = replay_telemetry.empty()
                    ? frame_count / options.capture_fps
                    : (replay_telemetry[frame_count].monotonic_ns -
                       replay_telemetry.front().monotonic_ns) / 1e9;
                const auto offset = std::chrono::duration<double>(elapsed_s);
                std::this_thread::sleep_until(
                    replay_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(offset));
            }
            cv::Mat raw, rectified, small, gray;
            if (!capture.read(raw) || raw.empty()) throw std::runtime_error("camera capture failed");
            const auto capture_time = std::chrono::steady_clock::now();
            const auto captured_telemetry = telemetry ? telemetry->latest()
                : std::optional<TimedEspState>{};
            if (options.rectified_input) rectified = raw;
            else cv::remap(raw, rectified, map_x, map_y, cv::INTER_LINEAR);
            if (options.record_raw_video) raw_video.write(raw);
            if (options.record_video) video.write(rectified);
#ifdef ROBOT_A733_NPU
            detector_worker.submit({rectified.clone(), capture_time,
                                    static_cast<std::uint64_t>(frame_count)});
#endif
            cv::resize(rectified, small, cv::Size(options.visual_width, options.visual_height), 0, 0, cv::INTER_AREA);
            cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
            const double dt_s = std::chrono::duration<double>(capture_time - previous_time).count();
            previous_time = capture_time;
            EspState state;
            bool telemetry_valid = false;
            std::uint64_t telemetry_sequence = 0;
            double telemetry_age_ms = -1;
            if (!replay_telemetry.empty()) {
                if (static_cast<std::size_t>(frame_count) >= replay_telemetry.size()) break;
                state = replay_telemetry[frame_count].state;
                telemetry_sequence = static_cast<std::uint64_t>(frame_count + 1);
                telemetry_age_ms = 0;
                telemetry_valid = true;
            } else if (const auto& sample = captured_telemetry) {
                state = sample->state;
                telemetry_sequence = sample->sequence;
                telemetry_age_ms = std::chrono::duration<double, std::milli>(capture_time - sample->timestamp).count();
                telemetry_valid = telemetry_age_ms >= 0 && telemetry_age_ms <= 250;
            }
            const robot::BodyVelocity wheel_center = robot::wheel_body_velocity(state.rpm, state.targets);
            const robot::Twist2 wheel_at_camera = robot::camera_origin_twist(
                {.forward_mps = wheel_center.forward_mps,
                 .left_mps = wheel_center.left_mps,
                 .yaw_radps = wheel_center.yaw_radps});
            const robot::BodyVelocity wheel{.forward_mps = wheel_at_camera.forward_mps,
                                            .left_mps = wheel_at_camera.left_mps,
                                            .yaw_radps = wheel_at_camera.yaw_radps};
            const robot::VisualMotion visual = visual_odometry.update(gray, projector, dt_s);
            robot::BodyVelocity fused = wheel;
            const bool plausible_visual = visual.valid &&
                                          std::hypot(visual.velocity.forward_mps, visual.velocity.left_mps) < .8 &&
                                          std::abs(visual.velocity.yaw_radps) < 4.0;
            if (plausible_visual) {
                fused.forward_mps = .7 * wheel.forward_mps + .3 * visual.velocity.forward_mps;
                fused.left_mps = .7 * wheel.left_mps + .3 * visual.velocity.left_mps;
            }
            // IMU axes are +x forward, +y right, +z down. Robot +z is up, so
            // use the negated gyro-z rate and never the IMU's absolute yaw.
            const double prediction_dt = std::min(dt_s, .2);
            const double measured_imu_yaw_rate = -state.gyro_z_degps * kDegreesToRadians;
            bias_anchor_elapsed_s += prediction_dt;
            bias_anchor_imu_delta_rad += measured_imu_yaw_rate * prediction_dt;
            const double imu_yaw_rate = measured_imu_yaw_rate - gyro_bias_radps;
            fused.yaw_radps = (plausible_visual
                               ? .8 * imu_yaw_rate + .2 * visual.velocity.yaw_radps
                               : imu_yaw_rate);
            robot::BodyVelocity prediction = wheel;
            prediction.yaw_radps = imu_yaw_rate;
            const double midpoint_yaw = odometry_pose.yaw_rad + .5 * prediction.yaw_radps * prediction_dt;
            odometry_pose.x_m += (std::cos(midpoint_yaw) * prediction.forward_mps -
                                  std::sin(midpoint_yaw) * prediction.left_mps) * prediction_dt;
            odometry_pose.y_m += (std::sin(midpoint_yaw) * prediction.forward_mps +
                                  std::cos(midpoint_yaw) * prediction.left_mps) * prediction_dt;
            odometry_pose.yaw_rad += prediction.yaw_radps * prediction_dt;
            filter.predict(prediction, prediction_dt);
            fence_worker.submit({small.clone(), capture_time, ++fence_request_sequence,
                                 odometry_pose.yaw_rad, odometry_pose});
            bool visual_certain = false;
            bool visual_very_certain = false;
            bool imu_yaw_reset = false;
            if (auto fence_result = fence_worker.take()) {
                visual_geometry = std::move(fence_result->geometry);
                lower_fence_count = fence_result->edges.lower.size();
                upper_fence_count = fence_result->edges.upper.size();
                if (visual_geometry.valid && !visual_geometry.candidates.empty()) {
                    const auto& candidate = visual_geometry.candidates.front();
                    robot::Pose2 current_candidate = candidate.pose;
                    current_candidate.x_m += odometry_pose.x_m - fence_result->odometry_pose.x_m;
                    current_candidate.y_m += odometry_pose.y_m - fence_result->odometry_pose.y_m;
                    current_candidate.yaw_rad += std::remainder(
                        odometry_pose.yaw_rad - fence_result->odometry_pose.yaw_rad,
                        2.0 * CV_PI);
                    const bool precise = candidate.wall_residual_m <=
                                         options.visual_precise_residual_m;
                    const double residual_quality = std::clamp(
                        (options.visual_residual_limit_m - candidate.wall_residual_m) /
                            (options.visual_residual_limit_m -
                             options.visual_precise_residual_m),
                        0.0, 1.0);
                    const std::array<double, 3> sigma_scale = {
                        options.visual_axis_sigma_m, options.visual_axis_sigma_m,
                        options.visual_yaw_sigma_rad};
                    for (std::size_t axis = 0; axis < 3; ++axis) {
                        const double ratio = visual_geometry.axis_sigma[axis] /
                                             sigma_scale[axis];
                        double certainty = residual_quality * visual_geometry.point_support *
                                           std::exp(-.5 * ratio * ratio);
                        if (certainty < options.visual_axis_certainty_min) certainty = 0;
                        visual_geometry.axis_certainty[axis] = certainty;
                    }
                    visual_certain =
                        std::max(visual_geometry.axis_certainty[0],
                                 visual_geometry.axis_certainty[1]) >= .65 &&
                        visual_geometry.axis_certainty[2] >= .50;
                    visual_very_certain =
                        std::min(visual_geometry.axis_certainty[0],
                                 visual_geometry.axis_certainty[1]) >= .80 &&
                        visual_geometry.axis_certainty[2] >= .85;
                    filter.correct_toward_axes(
                        current_candidate, visual_geometry.axis_certainty,
                        options.visual_pull_gain, options.visual_axis_max_pull_gain,
                        options.visual_axis_max_correction_m,
                        options.visual_axis_max_correction_rad);
                    const double yaw_error = std::remainder(
                        current_candidate.yaw_rad - odometry_pose.yaw_rad, 2.0 * CV_PI);
                    if (precise && visual_geometry.axis_certainty[2] >= .90 &&
                        std::abs(yaw_error) <= options.visual_yaw_reset_max_error_rad) {
                        // The IMU supplies only relative yaw rate; reset the
                        // integrated heading reference, not the sensor itself.
                        odometry_pose.yaw_rad = current_candidate.yaw_rad;
                        imu_yaw_reset = true;
                        if (!bias_anchor_visual_yaw) {
                            bias_anchor_visual_yaw = current_candidate.yaw_rad;
                            bias_anchor_elapsed_s = 0;
                            bias_anchor_imu_delta_rad = 0;
                        } else if (bias_anchor_elapsed_s >= 1.0) {
                            const double visual_delta = std::remainder(
                                current_candidate.yaw_rad - *bias_anchor_visual_yaw,
                                2.0 * CV_PI);
                            const double observed_bias = std::clamp(
                                (bias_anchor_imu_delta_rad - visual_delta) /
                                    bias_anchor_elapsed_s,
                                -5.0 * kDegreesToRadians, 5.0 * kDegreesToRadians);
                            gyro_bias_radps += .15 * (observed_bias - gyro_bias_radps);
                            bias_anchor_visual_yaw = current_candidate.yaw_rad;
                            bias_anchor_elapsed_s = 0;
                            bias_anchor_imu_delta_rad = 0;
                        }
                    }
                }
            }
            const robot::PoseEstimate pose = filter.estimate();
#ifdef ROBOT_A733_NPU
            if (auto detections = detector_worker.take()) {
                home_observation = robot::check_home_box(
                    *detections, object_projector,
                    {.x_m = pose.x_m, .y_m = pose.y_m, .yaw_rad = pose.yaw_rad});
                world.update_objects(robot::project_collectibles(
                    *detections, object_projector,
                    {.x_m = pose.x_m, .y_m = pose.y_m, .yaw_rad = pose.yaw_rad}),
                    capture_time);
            }
#endif
            auto target = world.nearest_collectible(
                {.x_m = pose.x_m, .y_m = pose.y_m, .yaw_rad = pose.yaw_rad});
            if (target && capture_time - target->last_seen > options.approach.target_timeout) {
                target.reset();
            }
            if (target) {
                search_result = search_controller.update(
                    {.x_m = pose.x_m, .y_m = pose.y_m, .yaw_rad = pose.yaw_rad},
                    true, capture_time, std::min(dt_s, .2));
                approach_result = approach_controller.update(
                    {.x_m = pose.x_m, .y_m = pose.y_m, .yaw_rad = pose.yaw_rad},
                    *target, capture_time, std::min(dt_s, .2));
            } else {
                approach_controller.reset();
                search_result = search_controller.update(
                    {.x_m = pose.x_m, .y_m = pose.y_m, .yaw_rad = pose.yaw_rad},
                    false, capture_time, std::min(dt_s, .2));
                approach_result = {.command = search_result.command,
                                   .target_valid = search_result.phase != robot::SearchPhase::complete,
                                   .target_reached = search_result.phase == robot::SearchPhase::complete};
            }
            const std::uint64_t time_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(capture_time.time_since_epoch()).count());
            std::ostringstream record;
            write_record(record, frame_count, time_ns, state, telemetry_valid, telemetry_sequence,
                         telemetry_age_ms, wheel, visual, fused, odometry_pose, pose,
                         visual_geometry, lower_fence_count, upper_fence_count,
                         visual_certain, visual_very_certain, imu_yaw_reset,
                         gyro_bias_radps / kDegreesToRadians, world.objects(), approach_result,
                         search_result, home_observation);
            log << record.str();
            if (options.stream_json) { std::cout << record.str(); std::cout.flush(); }
            broadcaster.send(record.str());
            if ((frame_count++ % 30) == 0) {
                std::cerr << "frame=" << frame_count << " fps_dt=" << dt_s << " wheel=" << wheel.forward_mps << ','
                          << wheel.left_mps << " visual=" << (visual.valid ? "ok" : "warmup")
                          << " telemetry=" << (telemetry_valid ? "ok" : "stale")
                          << " fence=" << lower_fence_count << '+' << upper_fence_count
                          << " pose=" << pose.x_m << ',' << pose.y_m << '\n';
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "robotloc: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
