#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace ellipses
{
/**
 * @brief Describe ellipse as generic conic. Interpretation of parameters EXACTLY follow
 * https://en.wikipedia.org/wiki/Ellipse (theta sign, half axes etc)
 * Note that Opencv way of fitting ellipse DOES NOT FOLLOW WIKI CONVENTION!
 */
class EllipseQuadric
{
   public:
    struct ParameterForm
    {
        double major, minor, cx, cy, theta;

        ParameterForm(const double _major, const double _minor, const double _cx, const double _cy,
                      const double _theta);
        ParameterForm() = default;
    };

    Eigen::Matrix3d matrix_form_;
    ParameterForm parameters_;

    std::vector<Eigen::Vector2d> points_on_ellipse_eig(const double degree_step) const;
    std::vector<cv::Point2i> points_on_ellipse_cv(const double degree_step) const;

    EllipseQuadric() = default;
    EllipseQuadric(const Eigen::Matrix3d &matrix_form);
    EllipseQuadric(const ParameterForm &parameter_form);
};

EllipseQuadric::ParameterForm matrix_to_parameters(const Eigen::Matrix3d &matrix_form);
Eigen::Matrix3d parameters_to_matrix(const EllipseQuadric::ParameterForm &parameter_form);

}  // namespace ellipses
