#pragma once

#include "ellipses.hpp"

namespace fitting
{
/**
 * @brief Perform ALGEBRAIC fitting of ellipse using set of points. Does not enforce constraints on design matrix, and
 * it assumes that data will be of sufficient quality so it will define ellipse.
 *
 * Ellipse fitted is of form
 * Ax^2 + Bxy + Cy^2 + Dx+ Ey + F = 0
 */
ellipses::EllipseQuadric fit_ellipse(const std::vector<cv::Point2i>& contour);
ellipses::EllipseQuadric fit_ellipse(const std::vector<Eigen::Vector2d>& contour);
}  // namespace fitting