#pragma once

#include <memory>
#include <string>

#include "robot/perception/detector.hpp"

namespace robot {

std::unique_ptr<Detector> make_a733_detector(const std::string& model_path);

}  // namespace robot
