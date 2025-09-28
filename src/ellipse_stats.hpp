#pragma once

#include "ellipses.hpp"

/**
 * @brief Calculate inner and outer intensity of ellipse given information that it's outer ellipse of ring (with bright
 * spot inside) or it's most inner ellipse.
 */
namespace stats
{
std::pair<double, double> get_ellipse_inner_outer_white_inner(const cv::Mat1b &gray,
                                                              const ellipses::EllipseQuadric &shape);

std::pair<double, double> get_ellipse_inner_outer_ring(const cv::Mat1b &gray, const ellipses::EllipseQuadric &shape);
}  // namespace stats