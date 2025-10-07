#pragma once

#include <memory>

#include "detection/board.hpp"
#include "ordering.hpp"

namespace calib::identification
{
struct Decoded
{
    OrderingBoardHex ordering_hex;
    std::vector<MarkerRing> markers;
    int identified_markers;

    const Eigen::Matrix<std::optional<int>, -1, -1> &ordering() const;
    Eigen::Matrix<std::optional<int>, -1, -1> &ordering();
};

/**
 * @brief Given set of coding marker (unordered) we iterate all configurations of id assignment that can be made from
 * marker set and try to derive global indexing base on it.
 *
 * @param coding set of markers that we assume can be coding (unordered, can be more that 3)
 * @param rings set of markers that we assume are rings marker (unordered, can hold arbitrary large number [if not
 * decoded, they will not be assigned])
 * @param board definition of pattern
 */
std::pair<std::optional<identification::Decoded>, std::vector<MarkerNeighborhood>> assign_global_IDs(
    const std::vector<MarkerCoding> &coding, const std::vector<MarkerRing> &rings, const BoardHexGrid &board);

}  // namespace calib::identification
