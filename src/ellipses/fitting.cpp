#include "fitting.hpp"

namespace
{
struct Scalling
{
    Eigen::Vector2d mean;
    double scale;
};

double get_x(const cv::Point2i &pts) { return pts.x; }
double get_x(const Eigen::Vector2d &pts) { return pts.x(); }
double get_y(const cv::Point2i &pts) { return pts.y; }
double get_y(const Eigen::Vector2d &pts) { return pts.y(); }

template <class Point>
Scalling get_scalling(const std::vector<Point> &inner)
{
    Eigen::Vector2d mean(0, 0);
    double max_val = 0.0;
    for (size_t idx = 0; idx < inner.size(); ++idx)
    {
        mean(0) += get_x(inner[idx]);
        mean(1) += get_y(inner[idx]);

        max_val = std::max(get_x(inner[idx]), max_val);
        max_val = std::max(get_y(inner[idx]), max_val);
    }

    mean /= inner.size();

    return {mean, max_val};
}

void scale_quadric(ellipses::EllipseQuadric::ParameterForm &data, const Scalling &scalling)
{
    data.major *= scalling.scale;
    data.minor *= scalling.scale;

    data.cx = data.cx * scalling.scale + scalling.mean(0);
    data.cy = data.cy * scalling.scale + scalling.mean(1);
}

Eigen::Matrix3d ans_to_matrix(const Eigen::Matrix<double, 6, 1> &ans)
{
    Eigen::Matrix3d ellipse_matrix;

    ellipse_matrix(0, 0) = ans(0);
    ellipse_matrix(0, 1) = ans(1) / 2.0;
    ellipse_matrix(0, 2) = ans(3) / 2.0;

    ellipse_matrix(1, 0) = ans(1) / 2.0;
    ellipse_matrix(1, 1) = ans(2);
    ellipse_matrix(1, 2) = ans(4) / 2.0;

    ellipse_matrix(2, 0) = ans(3) / 2.0;
    ellipse_matrix(2, 1) = ans(4) / 2.0;
    ellipse_matrix(2, 2) = ans(5);

    return ellipse_matrix;
}

template <class Point>
ellipses::EllipseQuadric fit_ellipse_eig(const std::vector<Point> &contour)
{
    const auto scalling = get_scalling(contour);

    Eigen::Matrix<double, -1, 6> lhs(contour.size(), 6);
    for (size_t idx = 0; idx < contour.size(); ++idx)
    {
        const double scalled_x = (get_x(contour[idx]) - scalling.mean(0)) / scalling.scale;
        const double scalled_y = (get_y(contour[idx]) - scalling.mean(1)) / scalling.scale;

        lhs(idx, 0) = scalled_x * scalled_x;
        lhs(idx, 1) = scalled_x * scalled_y;
        lhs(idx, 2) = scalled_y * scalled_y;
        lhs(idx, 3) = scalled_x;
        lhs(idx, 4) = scalled_y;
        lhs(idx, 5) = 1.0;
    }

    const Eigen::Matrix<double, 6, 6> combined = lhs.transpose() * lhs;

    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(combined, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // note that said conic is scalled and shifted (for numericall purpose). We need to rescale and reshift it to it's
    // position, with is easiest in parameter (non matrix form)
    Eigen::Matrix3d ellipse_matrix = ans_to_matrix(svd.matrixV().col(5));

    auto parameters = ellipses::matrix_to_parameters(ellipse_matrix);
    scale_quadric(parameters, scalling);

    return ellipses::EllipseQuadric(parameters);
}
}  // namespace

namespace fitting
{
ellipses::EllipseQuadric fit_ellipse(const std::vector<cv::Point2i> &contour) { return fit_ellipse_eig(contour); }

ellipses::EllipseQuadric fit_ellipse(const std::vector<Eigen::Vector2d> &contour) { return fit_ellipse_eig(contour); }
}  // namespace fitting