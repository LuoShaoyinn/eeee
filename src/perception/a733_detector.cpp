#include "robot/perception/a733_detector.hpp"

#include <stdexcept>
#include <vector>

#include "npulib.h"
#include "yolo26_pipeline.h"

namespace robot {
namespace {

class A733Detector final : public Detector {
public:
    explicit A733Detector(const std::string& model_path) : model_path_(model_path) {
        if (npu_.npu_init() != 0 || network_.network_create(model_path_.data(), 0) != 0 ||
            network_.network_prepare() != 0) {
            throw std::runtime_error("cannot initialize A733 NPU model: " + model_path);
        }
        network_.get_network_input_buff_info(0, &input_, &input_size_);
        output_info_.resize(network_.get_output_cnt());
        output_.resize(output_info_.size());
    }

    DetectionFrame detect(const cv::Mat& image, Timestamp timestamp,
                          std::uint64_t frame_sequence) override {
        if (yolo26_preprocess_frame(image, input_, input_size_) != 0 ||
            network_.network_input_output_set() != 0 || network_.network_run() != 0) {
            throw std::runtime_error("A733 NPU inference failed");
        }
        network_.get_output_fp_nocopy(output_info_.data());
        for (std::size_t index = 0; index < output_.size(); ++index) {
            output_[index] = output_info_[index].ptr;
        }
        DetectionFrame frame{.timestamp = timestamp, .frame_sequence = frame_sequence};
        for (const Yolo26Detection& detection : yolo26_decode_frame(image.size(), output_.data())) {
            ObjectClass object_class;
            switch (detection.class_id) {
            case 0: object_class = ObjectClass::opponent_robot; break;
            case 1: object_class = ObjectClass::red_cube; break;
            case 2: object_class = ObjectClass::yellow_cylinder; break;
            case 3: object_class = ObjectClass::home; break;
            default: continue;
            }
            frame.detections.push_back({
                .object_class = object_class,
                .confidence = detection.score,
                .box = {.left = static_cast<float>(detection.box.x),
                        .top = static_cast<float>(detection.box.y),
                        .right = static_cast<float>(detection.box.x + detection.box.width),
                        .bottom = static_cast<float>(detection.box.y + detection.box.height)},
            });
        }
        return frame;
    }

private:
    NpuUint npu_;
    NetworkItem network_;
    std::string model_path_;
    void* input_ = nullptr;
    unsigned int input_size_ = 0;
    std::vector<output_info_s> output_info_;
    std::vector<float*> output_;
};

}  // namespace

std::unique_ptr<Detector> make_a733_detector(const std::string& model_path) {
    return std::make_unique<A733Detector>(model_path);
}

}  // namespace robot
