// Build on Cubie A7S:
// g++ -O3 -std=c++17 cubie_optical_flow.cpp -o cubie_optical_flow $(pkg-config --cflags --libs opencv4)
// Example:
// ./cubie_optical_flow --device /dev/video0 --output flow_visualization.avi --csv flow.csv

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

struct Options {
    std::string device = "/dev/video0";
    std::string output = "flow_visualization.avi";
    std::string csv = "flow.csv";
    int width = 1280;
    int height = 720;
    int port = 0;
    std::string preview = "live_preview.jpg";
    std::string raw_preview = "raw_preview.jpg";
    std::string calibration = "camera1_fisheye_1280x720_rectilinear_f400.yaml";
    bool rectify = true;
    bool manual_exposure = false;
    bool automatic_exposure = false;
    double exposure = 0.0;
    double auto_exposure_mode = 1.0;  // V4L2_EXPOSURE_MANUAL
    bool display = false;
};

static Options parse_options(int argc, char **argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--device" && i + 1 < argc) options.device = argv[++i];
        else if (arg == "--output" && i + 1 < argc) options.output = argv[++i];
        else if (arg == "--csv" && i + 1 < argc) options.csv = argv[++i];
        else if (arg == "--width" && i + 1 < argc) options.width = std::stoi(argv[++i]);
        else if (arg == "--height" && i + 1 < argc) options.height = std::stoi(argv[++i]);
        else if (arg == "--port" && i + 1 < argc) options.port = std::stoi(argv[++i]);
        else if (arg == "--preview" && i + 1 < argc) options.preview = argv[++i];
        else if (arg == "--raw-preview" && i + 1 < argc) options.raw_preview = argv[++i];
        else if (arg == "--calibration" && i + 1 < argc) options.calibration = argv[++i];
        else if (arg == "--no-rectify") options.rectify = false;
        else if (arg == "--auto-exposure") options.automatic_exposure = true;
        else if (arg == "--exposure" && i + 1 < argc) {
            options.exposure = std::stod(argv[++i]);
            options.manual_exposure = true;
        }
        else if (arg == "--auto-exposure-mode" && i + 1 < argc) {
            options.auto_exposure_mode = std::stod(argv[++i]);
        }
        else if (arg == "--display") options.display = true;
        else if (arg == "--help") {
            std::cout << "Usage: cubie_optical_flow [--device /dev/video0] [--output flow.avi]"
                         " [--csv flow.csv] [--width 1280] [--height 720]"
                         " [--calibration camera1_fisheye_1280x720_rectilinear_f400.yaml]"
                         " [--exposure VALUE|--auto-exposure] [--display]\n";
            std::exit(0);
        }
    }
    return options;
}

static bool initialise_rectification(const Options &options, const cv::Size &image_size,
                                     cv::Mat &map_x, cv::Mat &map_y) {
    cv::FileStorage storage(options.calibration, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        std::cerr << "Cannot open calibration file: " << options.calibration << "\n";
        return false;
    }
    cv::Mat K, D, rectified_K;
    int calibration_width = 0;
    int calibration_height = 0;
    storage["K"] >> K;
    storage["D"] >> D;
    storage["rectified_K"] >> rectified_K;
    storage["image_width"] >> calibration_width;
    storage["image_height"] >> calibration_height;
    if (K.empty() || D.empty() || rectified_K.empty()) {
        std::cerr << "Calibration file is missing K, D, or rectified_K\n";
        return false;
    }
    if (calibration_width != image_size.width || calibration_height != image_size.height) {
        std::cerr << "Camera frame is " << image_size.width << 'x' << image_size.height
                  << ", but calibration requires " << calibration_width << 'x' << calibration_height << "\n";
        return false;
    }
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, CV_64F), rectified_K,
                                         image_size, CV_16SC2, map_x, map_y);
    std::cout << "Fisheye rectification enabled: " << options.calibration
              << " (output focal length " << rectified_K.at<double>(0, 0) << " px)\n";
    return true;
}

static float median(std::vector<float> values) {
    if (values.empty()) return 0.0F;
    const auto middle = values.begin() + static_cast<long>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

static bool send_all(int fd, const void *data, size_t size) {
    const auto *bytes = static_cast<const char *>(data);
    while (size > 0) {
        const ssize_t sent = send(fd, bytes, size, MSG_NOSIGNAL);
        if (sent <= 0) return false;
        bytes += sent;
        size -= static_cast<size_t>(sent);
    }
    return true;
}

static void mjpeg_server(int port, std::mutex &mutex, std::vector<uchar> &latest_jpeg) {
    const int server = socket(AF_INET, SOCK_STREAM, 0);
    int reuse = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (bind(server, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 || listen(server, 2) != 0) {
        std::cerr << "Cannot start MJPEG server on port " << port << "\n";
        close(server);
        return;
    }
    std::cout << "Live view: http://0.0.0.0:" << port << "\n";
    while (true) {
        const int client = accept(server, nullptr, nullptr);
        if (client < 0) continue;
        const char header[] = "HTTP/1.0 200 OK\r\nCache-Control: no-cache\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
        if (!send_all(client, header, sizeof(header) - 1)) { close(client); continue; }
        while (true) {
            std::vector<uchar> jpeg;
            { std::lock_guard<std::mutex> lock(mutex); jpeg = latest_jpeg; }
            if (!jpeg.empty()) {
                const std::string part = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                         std::to_string(jpeg.size()) + "\r\n\r\n";
                if (!send_all(client, part.data(), part.size()) || !send_all(client, jpeg.data(), jpeg.size()) ||
                    !send_all(client, "\r\n", 2)) break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        close(client);
    }
}

int main(int argc, char **argv) {
    const Options options = parse_options(argc, argv);
    cv::VideoCapture camera(options.device, cv::CAP_V4L2);
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
    camera.set(cv::CAP_PROP_FPS, 30);
    if (!camera.isOpened()) {
        std::cerr << "Cannot open " << options.device << "\n";
        return 1;
    }
    if (options.automatic_exposure && options.manual_exposure) {
        std::cerr << "Use either --exposure or --auto-exposure, not both\n";
        return 1;
    }
    if (options.automatic_exposure) {
        const bool auto_mode_set = camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 3.0);  // V4L2_EXPOSURE_AUTO
        std::cout << "Automatic exposure request; V4L2 auto mode set=" << auto_mode_set << "\n";
    } else if (options.manual_exposure) {
        const bool auto_mode_set = camera.set(cv::CAP_PROP_AUTO_EXPOSURE, options.auto_exposure_mode);
        const bool exposure_set = camera.set(cv::CAP_PROP_EXPOSURE, options.exposure);
        std::cout << "Manual exposure request " << options.exposure
                  << "; V4L2 auto mode set=" << auto_mode_set
                  << ", exposure set=" << exposure_set
                  << ", driver reports " << camera.get(cv::CAP_PROP_EXPOSURE) << "\n";
    }

    cv::Mat previous, frame, raw_frame, map_x, map_y;
    if (!camera.read(raw_frame)) {
        std::cerr << "Cannot read initial camera frame\n";
        return 1;
    }
    const cv::Size size(raw_frame.cols, raw_frame.rows);
    std::cout << "Camera stream: " << size.width << 'x' << size.height
              << " at " << camera.get(cv::CAP_PROP_FPS) << " FPS\n";
    if (options.rectify) {
        if (!initialise_rectification(options, size, map_x, map_y)) return 1;
        cv::remap(raw_frame, previous, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    } else {
        previous = raw_frame;
    }
    cv::cvtColor(previous, previous, cv::COLOR_BGR2GRAY);
    cv::VideoWriter video(options.output, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                          30.0, size, true);
    if (!video.isOpened()) {
        std::cerr << "Cannot create visualisation video: " << options.output << "\n";
        return 1;
    }
    std::ofstream csv(options.csv);
    csv << "time_s,fps,flow_x_px_s,flow_y_px_s,valid_ratio\n";
    auto last_time = std::chrono::steady_clock::now();
    std::mutex jpeg_mutex;
    std::vector<uchar> latest_jpeg;
    if (options.port > 0) {
        std::thread(mjpeg_server, options.port, std::ref(jpeg_mutex), std::ref(latest_jpeg)).detach();
    }

    std::cout << "Running on " << options.device << ". Press Ctrl-C to stop.\n";
    int frame_number = 0;
    while (camera.read(raw_frame)) {
        if (options.rectify) {
            cv::remap(raw_frame, frame, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        } else {
            frame = raw_frame;
        }
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - last_time).count();
        if (dt <= 0.0F) continue;
        cv::Mat gray, flow;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::calcOpticalFlowFarneback(previous, gray, flow, 0.5, 3, 21, 3, 5, 1.2, 0);

        std::vector<float> dxs, dys;
        int sampled = 0;
        cv::Mat visual = frame.clone();
        for (int y = 10; y < flow.rows; y += 20) {
            for (int x = 10; x < flow.cols; x += 20) {
                const cv::Point2f movement = flow.at<cv::Point2f>(y, x);
                const float magnitude = cv::norm(movement);
                if (magnitude > 0.05F && magnitude < 30.0F) {
                    dxs.push_back(movement.x);
                    dys.push_back(movement.y);
                    ++sampled;
                    cv::arrowedLine(visual, {x, y},
                                    {cvRound(x + movement.x), cvRound(y + movement.y)},
                                    {0, 255, 0}, 1, cv::LINE_AA, 0, 0.3);
                }
            }
        }
        const float vx = median(dxs) / dt;
        const float vy = median(dys) / dt;
        const float fps = 1.0F / dt;
        const int sample_columns = (flow.cols - 11) / 20 + 1;
        const int sample_rows = (flow.rows - 11) / 20 + 1;
        const float quality = static_cast<float>(sampled) /
                              static_cast<float>(sample_columns * sample_rows);
        const std::string label = cv::format("%.1f FPS  flow: (%+.1f, %+.1f) px/s  quality: %.2f",
                                             fps, vx, vy, quality);
        cv::putText(visual, label, {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    {0, 0, 255}, 2, cv::LINE_AA);
        video.write(visual);
        std::vector<int> jpeg_parameters = {cv::IMWRITE_JPEG_QUALITY, 80};
        std::vector<uchar> jpeg;
        cv::imencode(".jpg", visual, jpeg, jpeg_parameters);
        { std::lock_guard<std::mutex> lock(jpeg_mutex); latest_jpeg = std::move(jpeg); }
        if (++frame_number % 5 == 0) {
            const std::string temporary_preview = options.preview + ".tmp.jpg";
            cv::imwrite(temporary_preview, visual, jpeg_parameters);
            std::rename(temporary_preview.c_str(), options.preview.c_str());
            const std::string temporary_raw = options.raw_preview + ".tmp.jpg";
            cv::imwrite(temporary_raw, raw_frame, jpeg_parameters);
            std::rename(temporary_raw.c_str(), options.raw_preview.c_str());
        }
        csv << std::chrono::duration<double>(now.time_since_epoch()).count() << ','
            << fps << ',' << vx << ',' << vy << ',' << quality << '\n';
        csv.flush();
        if (options.display) {
            cv::imshow("Cubie optical flow", visual);
            if ((cv::waitKey(1) & 0xFF) == 'q') break;
        }
        previous = gray;
        last_time = now;
    }
    return 0;
}
