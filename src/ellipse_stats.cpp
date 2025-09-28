#include "ellipse_stats.hpp"

#include <opencv2/imgproc.hpp>

namespace
{
constexpr double kEnlargementScale = 1.3;
constexpr unsigned char kMaxOuter = 255;
constexpr unsigned char kTransition = 200;
constexpr unsigned char kRing = 150;
constexpr unsigned char kInner = 100;

ellipses::EllipseQuadric scale_quadric(const ellipses::EllipseQuadric &shape, const double scale,
                                       const double shift_row, const double shift_col)
{
    auto scalled = shape;
    scalled.parameters_.cy -= shift_row;
    scalled.parameters_.cx -= shift_col;
    scalled.parameters_.major *= scale;
    scalled.parameters_.minor *= scale;

    scalled.matrix_form_ = ellipses::parameters_to_matrix(scalled.parameters_);

    return scalled;
}

void draw_on_window(cv::Mat1b &window, const ellipses::EllipseQuadric &shape, const std::vector<double> &scales,
                    const std::vector<unsigned char> &colors, const int top_left_row, const int top_left_col)
{
    for (size_t idx = 0; idx < scales.size(); ++idx)
    {
        const auto scalled = scale_quadric(shape, scales[idx], top_left_row, top_left_col);
        const auto points_cv = scalled.points_on_ellipse_cv(2);

        cv::fillConvexPoly(window, points_cv, cv::Scalar(colors[idx]));
    }
}

struct Intensities
{
    int inner_sum, inner_count;
    int outer_sum, outer_count;
};

Intensities get_intensities(const cv::Mat1b &window, const cv::Mat1b &gray, const int top_left_row,
                            const int top_left_col, const unsigned char value_inner, const unsigned char value_outer)
{
    int outer_sum = 0;
    int outer_count = 0;

    int inner_sum = 0;
    int inner_count = 0;
    for (int row = top_left_row, row_window = 0; row < std::min(top_left_row + window.rows, gray.rows);
         ++row, ++row_window)
    {
        for (int col = top_left_col, col_window = 0; col < std::min(top_left_col + window.cols, gray.cols);
             ++col, ++col_window)
        {
            if (window(row_window, col_window) == value_outer)
            {
                outer_sum += gray(row, col);
                outer_count++;
            }
            else if (window(row_window, col_window) == value_inner)
            {
                inner_sum += gray(row, col);
                inner_count++;
            }
        }
    }
    return {inner_sum, inner_count, outer_sum, outer_count};
}

}  // namespace

namespace stats
{
std::pair<double, double> get_ellipse_inner_outer_white_inner(const cv::Mat1b &gray,
                                                              const ellipses::EllipseQuadric &shape)
{
    const int top_left_row = std::max(int(shape.parameters_.cy - shape.parameters_.major * kEnlargementScale), 0);
    const int top_left_col = std::max(int(shape.parameters_.cx - shape.parameters_.major * kEnlargementScale), 0);
    const int width = (shape.parameters_.major * 2) * kEnlargementScale;

    cv::Mat1b window = cv::Mat1b::zeros(width, width);

    draw_on_window(window, shape, {1.6, 1.3, 0.7}, {kMaxOuter, kTransition, kInner}, top_left_row, top_left_col);

    const auto intensities = get_intensities(window, gray, top_left_row, top_left_col, kInner, kMaxOuter);

    if (intensities.inner_count == 0 || intensities.outer_count == 0)
    {
        throw std::runtime_error("Should not happen!");
    }

    return {double(intensities.inner_sum) / intensities.inner_count,
            double(intensities.outer_sum) / intensities.outer_count};
}

std::pair<double, double> get_ellipse_inner_outer_ring(const cv::Mat1b &gray, const ellipses::EllipseQuadric &shape)
{
    const int top_left_row = std::max(int(shape.parameters_.cy - shape.parameters_.major * kEnlargementScale), 0);
    const int top_left_col = std::max(int(shape.parameters_.cx - shape.parameters_.major * kEnlargementScale), 0);

    const int width = (shape.parameters_.major * 2) * kEnlargementScale;

    cv::Mat1b window = cv::Mat1b::zeros(width, width);

    draw_on_window(window, shape, {1.3, 1.1, 0.8, 0.65}, {kMaxOuter, kTransition, kRing, kInner}, top_left_row,
                   top_left_col);

    const auto intensities = get_intensities(window, gray, top_left_row, top_left_col, kRing, kMaxOuter);

    if (intensities.inner_count == 0 || intensities.outer_count == 0)
    {
        throw std::runtime_error("Should not happen!");
    }

    return {double(intensities.inner_sum) / intensities.inner_count,
            double(intensities.outer_sum) / intensities.outer_count};
}
}  // namespace stats