#pragma once

#include "calibration_data.hpp"

namespace calib::marker::debug
{
void save_inner_markers_and_unique(const cv::Mat1b &image, const std::vector<MarkerUnidentified> &markers_core,
                                   const std::vector<MarkerUnidentified> &ring_and_coding, const int image_idx,
                                   const size_t scale_idx);

void save_inner_markers_and_unique(const cv::Mat1b &image, const std::vector<MarkerCoding> &coding,
                                   const std::vector<MarkerRing> &ring, const int image_idx, const size_t scale_idx);

void save_marker_identification(const cv::Mat1b &image, const Eigen::Matrix<std::optional<int>, -1, -1> &ordering,
                                const std::vector<MarkerRing> &markers, const int image_idx);

void save_neighbors_edges(const cv::Mat1b &image, const std::vector<MarkerNeighborhood> &neighbors,
                          const std::vector<MarkerCoding> &coding, const std::vector<MarkerRing> &ring,
                          const int image_idx);
}  // namespace calib::marker::debug
