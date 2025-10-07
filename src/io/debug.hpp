#pragma once

#include <opencv2/core.hpp>

namespace io::debug
{
void save_image(const cv::Mat1b &img, const std::string &name);
void save_image(const cv::Mat3b &img, const std::string &name);
}  // namespace io::debug