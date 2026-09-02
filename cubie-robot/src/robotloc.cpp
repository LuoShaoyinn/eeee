#include <algorithm>
#include <array>
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
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "robot/location.hpp"

namespace {
constexpr double kDegreesToRadians = CV_PI / 180.0;
volatile std::sig_atomic_t g_running = 1;

struct Options {
    std::string camera = "/dev/video0";
    std::string socket = "/tmp/robotd.sock";
    std::string calibration = "calibration/camera1_fisheye_1280x720_rectilinear_f400.yaml";
    std::string log_path;
    std::string video_path;
    bool record_video = true;
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
};

struct EspState {
    std::uint64_t ms = 0;
    std::uint64_t imu_age_ms = 0;
    double gyro_z_degps = 0;
    std::array<double, 4> rpm{};
    std::array<double, 4> targets{};
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
    name << "location-" << std::put_time(&local, "%Y%m%d-%H%M%S") << ".jsonl";
    return name.str();
}

std::string default_video_path(const std::string& log_path) {
    std::filesystem::path path(log_path);
    path.replace_extension(".avi");
    return path.string();
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        const auto value = [&](const char* name) -> const char* {
            if (index + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + name);
            return argv[++index];
        };
        if (argument == "--camera") options.camera = value("--camera");
        else if (argument == "--socket") options.socket = value("--socket");
        else if (argument == "--calibration") options.calibration = value("--calibration");
        else if (argument == "--log") options.log_path = value("--log");
        else if (argument == "--video") options.video_path = value("--video");
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
        else if (argument == "--help") {
            std::cout << "robotloc [--camera PATH] [--socket PATH] [--calibration FILE] [--log FILE] [--video FILE] [--no-video] "
                         "[--visual-width N] [--visual-height N] [--particles N] [--max-frames N] "
                         "[--height M] [--pitch DEG] [--roll DEG] [--initial-x M] [--initial-y M] "
                         "[--initial-yaw DEG] [--global-initialize]\n";
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

void write_record(std::ofstream& log, int frame_index, std::uint64_t time_ns, const EspState& state,
                  const robot::BodyVelocity& wheel, const robot::VisualMotion& visual,
                  const robot::BodyVelocity& fused, const robot::PoseEstimate& pose, size_t fence_count) {
    log << std::fixed << std::setprecision(6)
        << "{\"frame_index\":" << frame_index << ",\"monotonic_ns\":" << time_ns
        << ",\"esp_ms\":" << state.ms << ",\"imu_age_ms\":" << state.imu_age_ms
        << ",\"gyro_z_degps\":" << state.gyro_z_degps
        << ",\"rpm\":[" << state.rpm[0] << ',' << state.rpm[1] << ',' << state.rpm[2] << ',' << state.rpm[3] << ']'
        << ",\"targets\":[" << state.targets[0] << ',' << state.targets[1] << ',' << state.targets[2] << ',' << state.targets[3] << ']'
        << ",\"wheel\":[" << wheel.forward_mps << ',' << wheel.left_mps << ',' << wheel.yaw_radps << ']'
        << ",\"visual\":{\"valid\":" << (visual.valid ? "true" : "false")
        << ",\"features\":" << visual.tracked_features << ",\"velocity\":[" << visual.velocity.forward_mps
        << ',' << visual.velocity.left_mps << ',' << visual.velocity.yaw_radps << "]}"
        << ",\"fused\":[" << fused.forward_mps << ',' << fused.left_mps << ',' << fused.yaw_radps << ']'
        << ",\"pose\":[" << pose.x_m << ',' << pose.y_m << ',' << pose.yaw_rad << ']'
        << ",\"effective_particles\":" << pose.effective_particles
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
        capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        capture.set(cv::CAP_PROP_FPS, 30);
        std::ofstream log(options.log_path);
        if (!log) throw std::runtime_error("cannot write log: " + options.log_path);
        cv::VideoWriter video;
        if (options.record_video) {
            const std::filesystem::path video_parent = std::filesystem::path(options.video_path).parent_path();
            if (!video_parent.empty()) std::filesystem::create_directories(video_parent);
            video.open(options.video_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10.0, cv::Size(1280, 720));
            if (!video.isOpened()) throw std::runtime_error("cannot open video writer: " + options.video_path);
        }
        std::cerr << "robotloc: passive capture=" << options.camera << " robotd=" << options.socket
                  << " log=" << options.log_path;
        if (options.record_video) std::cerr << " video=" << options.video_path;
        std::cerr << '\n';
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
        robot::VisualOdometry visual_odometry;
        robot::FenceParticleFilter filter(options.particles, options.initial_x_m, options.initial_y_m,
                                          options.initial_yaw_deg * kDegreesToRadians, options.global_initialize);
        auto previous_time = std::chrono::steady_clock::now();
        int frame_count = 0;
        while (g_running && (options.max_frames == 0 || frame_count < options.max_frames)) {
            cv::Mat raw, rectified, small, gray;
            if (!capture.read(raw) || raw.empty()) throw std::runtime_error("camera capture failed");
            const auto capture_time = std::chrono::steady_clock::now();
            cv::remap(raw, rectified, map_x, map_y, cv::INTER_LINEAR);
            if (options.record_video) video.write(rectified);
            cv::resize(rectified, small, cv::Size(options.visual_width, options.visual_height), 0, 0, cv::INTER_AREA);
            cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
            const double dt_s = std::chrono::duration<double>(capture_time - previous_time).count();
            previous_time = capture_time;
            EspState state;
            if (!parse_state(request_robotd(options.socket, "state"), state)) throw std::runtime_error("invalid ESP state reply");
            (void)parse_telemetry_targets(request_robotd(options.socket, "telemetry"), state.targets);
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
            const double imu_yaw = state.gyro_z_degps * kDegreesToRadians;
            fused.yaw_radps = plausible_visual ? .8 * imu_yaw + .2 * visual.velocity.yaw_radps : imu_yaw;
            filter.predict(fused, std::min(dt_s, .2));
            const std::vector<cv::Point2d> fence = lower_fence_points(small, projector);
            filter.update(fence);
            const robot::PoseEstimate pose = filter.estimate();
            const std::uint64_t time_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(capture_time.time_since_epoch()).count());
            write_record(log, frame_count, time_ns, state, wheel, visual, fused, pose, fence.size());
            if ((frame_count++ % 30) == 0) {
                std::cerr << "frame=" << frame_count << " fps_dt=" << dt_s << " wheel=" << wheel.forward_mps << ','
                          << wheel.left_mps << " visual=" << (visual.valid ? "ok" : "warmup")
                          << " fence=" << fence.size() << " pose=" << pose.x_m << ',' << pose.y_m << '\n';
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "robotloc: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
