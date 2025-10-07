#pragma once

#include <optional>

#include "board.hpp"
#include "calibration_data.hpp"
#include "detection_parameters.hpp"

namespace calib::marker
{
std::optional<ImageDecoding> detect_and_identify(cv::Mat1b &input, const DetectionParameters &parameters,
                                                 const BoardHexGrid &board, const int image_idx);
}  // namespace calib::marker
