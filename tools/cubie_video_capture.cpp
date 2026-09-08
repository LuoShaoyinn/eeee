// Headless calibrated USB-camera recorder for Cubie A7S.
#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>

struct Options {
    std::string device = "/dev/video0";
    std::string output;
    std::string calibration;
    int width = 1280;
    int height = 720;
    int fps = 30;
    int frames = 900;
};

static Options parse_options(int argc, char **argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--device" && index + 1 < argc) options.device = argv[++index];
        else if (argument == "--output" && index + 1 < argc) options.output = argv[++index];
        else if (argument == "--calibration" && index + 1 < argc) options.calibration = argv[++index];
        else if (argument == "--width" && index + 1 < argc) options.width = std::stoi(argv[++index]);
        else if (argument == "--height" && index + 1 < argc) options.height = std::stoi(argv[++index]);
        else if (argument == "--fps" && index + 1 < argc) options.fps = std::stoi(argv[++index]);
        else if (argument == "--frames" && index + 1 < argc) options.frames = std::stoi(argv[++index]);
        else if (argument == "--help") {
            std::cout << "Usage: cubie_video_capture --output SAMPLE.avi --calibration CAMERA.yaml"
                         " [--device /dev/video0] [--width 1280] [--height 720]"
                         " [--fps 30] [--frames 900]\n";
            std::exit(0);
        }
    }
    return options;
}

static bool initialise_rectifier(const Options &options, cv::Mat &map_x, cv::Mat &map_y) {
    cv::FileStorage storage(options.calibration, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        std::cerr << "Cannot open calibration file: " << options.calibration << '\n';
        return false;
    }
    cv::Mat camera_matrix, distortion, rectified_matrix;
    int calibration_width = 0;
    int calibration_height = 0;
    storage["K"] >> camera_matrix;
    storage["D"] >> distortion;
    storage["rectified_K"] >> rectified_matrix;
    storage["image_width"] >> calibration_width;
    storage["image_height"] >> calibration_height;
    if (camera_matrix.empty() || distortion.empty() || rectified_matrix.empty() ||
        calibration_width != options.width || calibration_height != options.height) {
        std::cerr << "Calibration must contain K, D, rectified_K and match "
                  << options.width << 'x' << options.height << '\n';
        return false;
    }
    cv::fisheye::initUndistortRectifyMap(camera_matrix, distortion,
                                         cv::Mat::eye(3, 3, CV_64F), rectified_matrix,
                                         cv::Size(options.width, options.height), CV_16SC2,
                                         map_x, map_y);
    return true;
}

int main(int argc, char **argv) {
    const Options options = parse_options(argc, argv);
    if (options.output.empty() || options.calibration.empty() || options.width <= 0 ||
        options.height <= 0 || options.fps <= 0 || options.frames <= 0) {
        std::cerr << "Output, calibration, dimensions, FPS, and frame count are required\n";
        return 2;
    }

    cv::Mat map_x, map_y;
    if (!initialise_rectifier(options, map_x, map_y)) return 1;

    cv::VideoCapture camera(options.device, cv::CAP_V4L2);
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
    camera.set(cv::CAP_PROP_FPS, options.fps);
    if (!camera.isOpened()) {
        std::cerr << "Cannot open " << options.device << '\n';
        return 1;
    }

    cv::VideoWriter writer(options.output, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                           options.fps, cv::Size(options.width, options.height));
    if (!writer.isOpened()) {
        std::cerr << "Cannot create " << options.output << '\n';
        return 1;
    }

    const auto started = std::chrono::steady_clock::now();
    cv::Mat raw, rectified;
    for (int frame_index = 0; frame_index < options.frames; ++frame_index) {
        if (!camera.read(raw) || raw.cols != options.width || raw.rows != options.height) {
            std::cerr << "Camera read failed or returned an unexpected frame size\n";
            return 1;
        }
        cv::remap(raw, rectified, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        writer.write(rectified);
    }
    const float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - started).count();
    std::cout << "saved " << options.frames << " rectified frames to " << options.output
              << " in " << elapsed << " s (" << options.frames / elapsed << " FPS)\n";
    return 0;
}
