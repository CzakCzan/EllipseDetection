#include "board.hpp"

namespace
{
Eigen::Vector3d marker_localization(const calib::BoardHexGrid& board, const int row, const int col)
{
    if (row % 2 == 0)
    {
        return board.top_left_ + Eigen::Vector3d(col * board.spacing_cols_, row * board.spacing_rows_, 0.0);
    }
    else
    {
        return board.top_left_ + Eigen::Vector3d((col + 0.5) * board.spacing_cols_, row * board.spacing_rows_, 0.0);
    }
}

}  // namespace

namespace calib
{

Eigen::Vector2i BoardHexGrid::id_to_row_and_col(const int id) const
{
    const int row = id / cols_;
    const int col = id - row * cols_;
    return {row, col};
}

int BoardHexGrid::row_and_col_to_id(const int row, const int col) const
{
    const int marker_id = row * cols_ + col;
    return marker_id;
}

void BoardHexGrid::apply_scale(const float factor)
{
    inner_radius_ *= factor;
    outer_radius_ *= factor;
    spacing_rows_ *= factor;
    spacing_cols_ *= factor;
    top_left_ *= factor;
    bottom_right_ *= factor;
    for (int row = 0; row < rows_; ++row)
    {
        for (int col = 0; col < cols_; ++col)
        {
            marker_centers_[col + row * cols_] *= factor;
        }
    }
}

int BoardHexGrid::marker_distance_01() const { return col_left_ - col_right_; }

std::vector<std::pair<int, int>> BoardHexGrid::marker_lines_01_locations() const
{
    std::vector<std::pair<int, int>> ordering_markers;
    for (int col = col_left_; col < col_right_; ++col)
    {
        ordering_markers.emplace_back(row_left_, col);
    }
    return ordering_markers;
}

BoardHexGrid::BoardHexGrid()
{
    static constexpr float kEquilateralTriangleHeight = std::sqrt(3.0f) / 2.f;
    padding_UL_cm_ = Eigen::Vector3d(1.0f, 1.149f, 0.f);  // cols, rows, 0
    padding_BR_cm_ = Eigen::Vector3d(0.5f, 0.5f, 0.f);    // cols, rows, 0

    rows_ = 27;
    cols_ = 33;
    inner_radius_ = 0.28f;
    outer_radius_ = 0.5f;
    spacing_cols_ = 1.2f;
    spacing_rows_ = kEquilateralTriangleHeight * spacing_cols_;
    top_left_ = padding_UL_cm_ + Eigen::Vector3d(outer_radius_, outer_radius_, 0);
    bottom_right_ =
        padding_UL_cm_ + padding_BR_cm_ + Eigen::Vector3d((cols_ + 0.5) * spacing_cols_, rows_ * spacing_rows_, 0.0);

    for (int row = 0; row < rows_; ++row)
    {
        for (int col = 0; col < cols_; ++col)
        {
            marker_centers_.emplace_back(marker_localization(*this, row, col));
        }
    }
}

}  // namespace calib