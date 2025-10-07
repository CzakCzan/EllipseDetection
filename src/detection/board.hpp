#pragma once

#include <numbers>

#include <Eigen/Dense>

namespace calib
{

/**
 * @brief Define calibration board base on concentric circles.
 *
 * 0 1 are invisible markers
 *
 *  board in ordering of
 *  ------------>
 * | * * * * * *
 * |  * * * * * *
 * | * * * * * *
 * |  * 0 * 1 * *
 * | * * * * * *
 * V  * * * * * *
 *
 * board indexing
 * ------------>
 * | 000 001 002  *   *   028
 * |   029 030  *   *   *    *
 * |  *   *   *   *   *    *
 * |    *   *   *   *   *    *
 * V  *   *   *   *   *    *
 *
 * The single marker of the board:
 * |__________________10mm___________________|
 * |       |___________5mm___________|       |
 * |       |                         |       |
 * |       |      ooooooooooooo      |       |
 * |       |  ooooooooooooooooooooo  |       |
 * |       |ooooooooooooooooooooooooo|       |
 * |     oo|ooooooooooooooooooooooooo|oo     |
 * |   oooo|ooooooo           ooooooo|oooo   |
 * |  ooooo|oooo                 oooo|oooo#  |
 * | oooooo|oo                     oo|oooooo |
 * |ooooooo|o                       o|ooooooo|
 * |ooooooo|                         |ooooooo|
 * |ooooooo|                         |ooooooo|
 * |ooooooo|                         |ooooooo|
 * ooooooooo                         ooooooooo
 *  oooooooo                         oooooooo
 *  ooooooooo                       ooooooooo
 *   ooooooooo                     ooooooooo
 *    oooooooooo                 oooooooooo
 *     oooooooooooo           oooooooooooo
 *       ooooooooooooooooooooooooooooooo
 *         ooooooooooooooooooooooooooo
 *            ooooooooooooooooooooo
 *                ooooooooooooo
 */
class BoardHexGrid
{
   public:
    int cols_;
    int rows_;

    float inner_radius_;
    float outer_radius_;

    float spacing_cols_;
    float spacing_rows_;

    const int row_left_ = 13;
    const int col_left_ = 14;
    const int row_right_ = 13;
    const int col_right_ = 16;

    std::vector<Eigen::Vector3d> marker_centers_;

    Eigen::Vector3d top_left_;
    Eigen::Vector3d bottom_right_;

    Eigen::Vector2i id_to_row_and_col(const int id) const;
    int row_and_col_to_id(const int row, const int col) const;
    void apply_scale(const float factor);

    /**
     *  Even:
     * | * * * * * *
     * |  * L * R * *
     * | * * * * * *
     * |  * * * * * *
     *  Odd:
     * |  * * * * * *
     * | * L * R * *
     * |  * * * * * *
     * | * * * * * *
     */
    const bool is_even_ = true;

    Eigen::Vector3d padding_UL_cm_;
    Eigen::Vector3d padding_BR_cm_;

    int marker_distance_01() const;

    // return row/col locations of line between X-Y coding locations (exclude coding markers)
    std::vector<std::pair<int, int>> marker_lines_01_locations() const;

    BoardHexGrid();
};
}  // namespace calib
