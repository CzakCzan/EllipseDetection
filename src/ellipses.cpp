#include "ellipses.hpp"

#include "utilities.hpp"

namespace ellipses
{
EllipseQuadric::ParameterForm::ParameterForm(const double _major, const double _minor, const double _cx,
                                             const double _cy, const double _theta)
    : major(_major), minor(_minor), cx(_cx), cy(_cy), theta(_theta)
{
}

std::vector<Eigen::Vector2d> EllipseQuadric::points_on_ellipse_eig(const double degree_step) const
{
    const Eigen::Matrix2d rot_to = utils::to_rotation_matrix(-parameters_.theta).transpose();

    std::vector<Eigen::Vector2d> point_on_ellipse;
    point_on_ellipse.reserve(360.0 / degree_step);

    double t = 0.0;
    while (t < 360.0)
    {
        point_on_ellipse.emplace_back(rot_to * utils::to_point(parameters_.major, parameters_.minor, t * M_PI / 180.0) +
                                      Eigen::Vector2d(parameters_.cx, parameters_.cy));
        t += degree_step;
    }

    return point_on_ellipse;
}

std::vector<cv::Point2i> EllipseQuadric::points_on_ellipse_cv(const double degree_step) const
{
    const Eigen::Matrix2d rot_to = utils::to_rotation_matrix(-parameters_.theta).transpose();

    std::vector<cv::Point2i> point_on_ellipse;
    point_on_ellipse.reserve(360.0 / degree_step);

    double t = 0.0;
    while (t < 360.0)
    {
        const Eigen::Vector2d pts = rot_to * utils::to_point(parameters_.major, parameters_.minor, t * M_PI / 180.0) +
                                    Eigen::Vector2d(parameters_.cx, parameters_.cy);
        point_on_ellipse.emplace_back(std::round(pts(0)), std::round(pts(1)));
        t += degree_step;
    }

    return point_on_ellipse;
}

EllipseQuadric::EllipseQuadric(const Eigen::Matrix3d &matrix_form) : matrix_form_(matrix_form)
{
    parameters_ = matrix_to_parameters(matrix_form);
}

EllipseQuadric::EllipseQuadric(const ParameterForm &parameter_form) : parameters_(parameter_form)
{
    matrix_form_ = parameters_to_matrix(parameter_form);
}

EllipseQuadric::ParameterForm matrix_to_parameters(const Eigen::Matrix3d &matrix_form)
{
    EllipseQuadric::ParameterForm parameter_form;

    const double A = matrix_form(0, 0);
    const double B = 2.0 * matrix_form(0, 1);
    const double C = matrix_form(1, 1);
    const double D = 2.0 * matrix_form(0, 2);
    const double E = 2.0 * matrix_form(1, 2);
    const double F = matrix_form(2, 2);

    const double denom = std::pow(B, 2) - 4.0 * A * C;

    parameter_form.major =
        -(std::sqrt(2.0 * (A * std::pow(E, 2) * C * std::pow(D, 2) - B * D * E + (std::pow(B, 2) - 4.0 * A * C) * F) *
                    ((A + C) + std::sqrt(std::pow(A - C, 2) + std::pow(B, 2))))) /
        denom;
    parameter_form.minor =
        -(std::sqrt(2.0 * (A * std::pow(E, 2) * C * std::pow(D, 2) - B * D * E + (std::pow(B, 2) - 4.0 * A * C) * F) *
                    ((A + C) - std::sqrt(std::pow(A - C, 2) + std::pow(B, 2))))) /
        denom;

    parameter_form.cx = (2.0 * C * D - B * E) / denom;

    parameter_form.cy = (2.0 * A * E - B * D) / denom;

    parameter_form.theta = 0.5 * std::atan2(-B, C - A);

    return parameter_form;
}

Eigen::Matrix3d parameters_to_matrix(const EllipseQuadric::ParameterForm &parameter_form)
{
    const double major = parameter_form.major;
    const double minor = parameter_form.minor;
    const double cx = parameter_form.cx;
    const double cy = parameter_form.cy;
    const double theta = parameter_form.theta;

    const double A =
        std::pow(major, 2) * std::pow(std::sin(theta), 2) + std::pow(minor, 2) * std::pow(std::cos(theta), 2);

    const double B = 2 * (std::pow(minor, 2) - std::pow(major, 2)) * std::cos(theta) * std::sin(theta);

    const double C =
        std::pow(major, 2) * std::pow(std::cos(theta), 2) + std::pow(minor, 2) * std::pow(std::sin(theta), 2);

    const double D = -2 * A * cx - B * cy;

    const double E = -B * cx - 2 * C * cy;

    const double F = A * std::pow(cx, 2) + B * cx * cy + C * std::pow(cy, 2) - std::pow(major, 2) * std::pow(minor, 2);

    Eigen::Matrix3d matrix_form;

    matrix_form(0, 0) = A;
    matrix_form(0, 1) = B / 2;
    matrix_form(0, 2) = D / 2;

    matrix_form(1, 0) = B / 2;
    matrix_form(1, 1) = C;
    matrix_form(1, 2) = E / 2;

    matrix_form(2, 0) = D / 2;
    matrix_form(2, 1) = E / 2;
    matrix_form(2, 2) = F;

    return matrix_form;
}

}  // namespace ellipses