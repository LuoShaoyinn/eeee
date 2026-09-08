#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <opencv2/videoio.hpp>

#include "npulib.h"
#include "yolo26_pipeline.h"

namespace {
double milliseconds(std::chrono::steady_clock::time_point begin,
                    std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s MODEL.nb VIDEO.avi [stride] [max_frames]\n", argv[0]);
        return 2;
    }
    const int stride = argc > 3 ? std::max(1, std::atoi(argv[3])) : 1;
    const int maximum = argc > 4 ? std::max(0, std::atoi(argv[4])) : 0;
    cv::VideoCapture video(argv[2]);
    if (!video.isOpened()) {
        std::fprintf(stderr, "cannot open video: %s\n", argv[2]);
        return 2;
    }
    NpuUint npu;
    if (npu.npu_init() != 0) return 3;
    NetworkItem network;
    if (network.network_create(argv[1], 0) != 0 || network.network_prepare() != 0) return 3;
    void* input = nullptr;
    unsigned int input_size = 0;
    network.get_network_input_buff_info(0, &input, &input_size);
    const int output_count = network.get_output_cnt();
    std::vector<float*> output(output_count, nullptr);
    std::vector<output_info_s> output_info(output_count);
    int source_index = 0;
    int processed = 0;
    std::size_t detections = 0;
    double capture_ms = 0, preprocess_ms = 0, inference_ms = 0, postprocess_ms = 0;
    cv::Mat frame;
    while (video.read(frame)) {
        const auto captured = std::chrono::steady_clock::now();
        capture_ms += milliseconds(captured, std::chrono::steady_clock::now());
        if (source_index++ % stride != 0) continue;
        const auto pre_begin = std::chrono::steady_clock::now();
        if (yolo26_preprocess_frame(frame, input, input_size) != 0) return 4;
        const auto pre_end = std::chrono::steady_clock::now();
        if (network.network_input_output_set() != 0) return 4;
        const auto infer_begin = std::chrono::steady_clock::now();
        if (network.network_run() != 0) return 4;
        const auto infer_end = std::chrono::steady_clock::now();
        network.get_output_fp_nocopy(output_info.data());
        for (int index = 0; index < output_count; ++index) output[index] = output_info[index].ptr;
        const auto post_begin = std::chrono::steady_clock::now();
        detections += yolo26_decode_frame(frame.size(), output.data()).size();
        const auto post_end = std::chrono::steady_clock::now();
        preprocess_ms += milliseconds(pre_begin, pre_end);
        inference_ms += milliseconds(infer_begin, infer_end);
        postprocess_ms += milliseconds(post_begin, post_end);
        if (++processed % 100 == 0) std::fprintf(stderr, "processed %d\n", processed);
        if (maximum && processed >= maximum) break;
    }
    const double count = std::max(1, processed);
    std::printf("frames=%d detections=%zu preprocess_ms=%.3f inference_ms=%.3f "
                "postprocess_ms=%.3f pipeline_fps=%.2f\n", processed, detections,
                preprocess_ms / count, inference_ms / count, postprocess_ms / count,
                1000.0 / ((preprocess_ms + inference_ms + postprocess_ms) / count));
    return 0;
}
