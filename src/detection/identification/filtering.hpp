#pragma once

#include <vector>

#include "detection/board.hpp"
#include "detection/calibration_data.hpp"

namespace calib::identification
{

void filter_out_flase_positives(std::vector<MarkerNeighborhood> &neighbors_graph);

std::vector<int> create_markers_order(const std::vector<MarkerNeighborhood> &neighbors_graph,
                                      const std::vector<MarkerRing> markers, const std::pair<int, int> coding_pair,
                                      const BoardHexGrid &board);

}  // namespace calib::identification
