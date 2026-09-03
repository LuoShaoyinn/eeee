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
    bool record_video = true;
    bool record_raw_video = false;
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
    double telemetry_hz = 25;
    bool stream_json = false;
    bool broadcast_enabled = true;
    std::string broadcast_address = "255.255.255.255";
    int broadcast_port = 3335;
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

std::vector<cv::Point2d> lower_fence_points(const cv::Mat& rectified, const robot::GroundProjector& projector) {
    cv::Mat hsv, mask;
    cv::cvtColor(rectified, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(92, 75, 45), cv::Scalar(135, 255, 255), mask);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 3)));
    std::vector<cv::Point2d> points;
    for (int x = 0; x < mask.cols; x += 4) {
        int y = -1;
        for (int row = mask.rows - 1; row >= mask.rows / 5; --row) {
            if (mask.at<unsigned char>(row, x) != 0) { y = row; break; }
        }
        cv::Point2d ground;
        if (y >= 0 && projector.project(cv::Point2f(static_cast<float>(x), static_cast<float>(y)), ground)) {
            points.push_back(ground);
        }
    }
    return points;
}

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
    options.telemetry_hz = config.telemetry_hz;
    options.broadcast_enabled = config.debug_broadcast_enabled;
    options.broadcast_address = config.debug_broadcast_address;
    options.broadcast_port = config.debug_broadcast_port;
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
        else if (argument == "--no-video") options.record_video = false;
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
            std::cout << "robot-runtime [--config FILE] [--camera PATH] [--socket PATH] [--calibration FILE] [--log FILE] [--video FILE] [--raw-video FILE] [--no-video] "
                         "[--visual-width N] [--visual-height N] [--particles N] [--max-frames N] "
                         "[--height M] [--pitch DEG] [--roll DEG] [--initial-x M] [--initial-y M] "
                         "[--initial-yaw DEG] [--global-initialize] [--stdout-json] [--no-broadcast]\n";
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
                  size_t fence_count) {
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
        << ",\"major_axis_rad\":" << visual_geometry.major_axis_rad << '}'
        << ",\"visual_geometry_candidates\":[";
    for (std::size_t index = 0; index < visual_geometry.candidates.size(); ++index) {
        if (index != 0) log << ',';
        const auto& candidate = visual_geometry.candidates[index];
        log << '[' << candidate.pose.x_m << ',' << candidate.pose.y_m << ','
            << candidate.pose.yaw_rad << ',' << candidate.wall_residual_m << ']';
    }
    log << "]"
        << ",\"lower_fence_points\":" << fence_count << "}\n";
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
        cv::Mat map_x, map_y;
        cv::fisheye::initUndistortRectifyMap(camera_matrix, distortion, cv::Mat::eye(3, 3, CV_64F),
                                             rectified_matrix, cv::Size(1280, 720), CV_16SC2, map_x, map_y);
        cv::VideoCapture capture(options.camera, cv::CAP_V4L2);
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
        TelemetrySampler telemetry(options.socket, options.telemetry_hz);
        telemetry.start();
        DebugBroadcaster broadcaster(options.broadcast_enabled, options.broadcast_address,
                                     options.broadcast_port);
        auto previous_time = std::chrono::steady_clock::now();
        int frame_count = 0;
        robot::VisualGeometryEstimate visual_geometry;
        while (g_running && (options.max_frames == 0 || frame_count < options.max_frames)) {
            cv::Mat raw, rectified, small, gray;
            if (!capture.read(raw) || raw.empty()) throw std::runtime_error("camera capture failed");
            const auto capture_time = std::chrono::steady_clock::now();
            const auto captured_telemetry = telemetry.latest();
            cv::remap(raw, rectified, map_x, map_y, cv::INTER_LINEAR);
            if (options.record_raw_video) raw_video.write(raw);
            if (options.record_video) video.write(rectified);
            cv::resize(rectified, small, cv::Size(options.visual_width, options.visual_height), 0, 0, cv::INTER_AREA);
            cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
            const double dt_s = std::chrono::duration<double>(capture_time - previous_time).count();
            previous_time = capture_time;
            EspState state;
            bool telemetry_valid = false;
            std::uint64_t telemetry_sequence = 0;
            double telemetry_age_ms = -1;
            if (const auto& sample = captured_telemetry) {
                state = sample->state;
                telemetry_sequence = sample->sequence;
                telemetry_age_ms = std::chrono::duration<double, std::milli>(capture_time - sample->timestamp).count();
                telemetry_valid = telemetry_age_ms >= 0 && telemetry_age_ms <= 250;
            }
            const robot::BodyVelocity wheel = robot::wheel_body_velocity(state.rpm, state.targets);
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
            const double imu_yaw_rate = -state.gyro_z_degps * kDegreesToRadians;
            fused.yaw_radps = (plausible_visual
                               ? .8 * imu_yaw_rate + .2 * visual.velocity.yaw_radps
                               : imu_yaw_rate);
            robot::BodyVelocity prediction = wheel;
            prediction.yaw_radps = imu_yaw_rate;
            const double prediction_dt = std::min(dt_s, .2);
            const double midpoint_yaw = odometry_pose.yaw_rad + .5 * prediction.yaw_radps * prediction_dt;
            odometry_pose.x_m += (std::cos(midpoint_yaw) * prediction.forward_mps -
                                  std::sin(midpoint_yaw) * prediction.left_mps) * prediction_dt;
            odometry_pose.y_m += (std::sin(midpoint_yaw) * prediction.forward_mps +
                                  std::cos(midpoint_yaw) * prediction.left_mps) * prediction_dt;
            odometry_pose.yaw_rad += prediction.yaw_radps * prediction_dt;
            filter.predict(prediction, prediction_dt);
            const std::vector<cv::Point2d> fence = lower_fence_points(small, projector);
            if (frame_count % 5 == 0) {
                visual_geometry = robot::estimate_fence_geometry(fence, odometry_pose.yaw_rad);
            }
            filter.update(fence);
            const robot::PoseEstimate pose = filter.estimate();
            const std::uint64_t time_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(capture_time.time_since_epoch()).count());
            std::ostringstream record;
            write_record(record, frame_count, time_ns, state, telemetry_valid, telemetry_sequence,
                         telemetry_age_ms, wheel, visual, fused, odometry_pose, pose,
                         visual_geometry, fence.size());
            log << record.str();
            if (options.stream_json) { std::cout << record.str(); std::cout.flush(); }
            broadcaster.send(record.str());
            if ((frame_count++ % 30) == 0) {
                std::cerr << "frame=" << frame_count << " fps_dt=" << dt_s << " wheel=" << wheel.forward_mps << ','
                          << wheel.left_mps << " visual=" << (visual.valid ? "ok" : "warmup")
                          << " telemetry=" << (telemetry_valid ? "ok" : "stale")
                          << " fence=" << fence.size() << " pose=" << pose.x_m << ',' << pose.y_m << '\n';
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "robotloc: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
