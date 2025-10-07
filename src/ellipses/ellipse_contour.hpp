#pragma once

#include <optional>

#include "ellipses.hpp"

namespace ellipse_contour
{
/**
 * @brief Perform refitting of quadric given information that it's outermost ring, or innermost ellipse. Refit quadric
 * using assumption that we are observing some sort of ellipse, and it generate set of points on ellipse perimeter and
 * fit (for each of that points and it's neighborhood) sigmoid function and find it's crossing with mean intensity of
 * inner and outer part. Set of computed points are used to estimate ellipse and that process is repeated.
 */
std::optional<ellipses::EllipseQuadric> refit_quadric(const cv::Mat1b& image,
                                                      const ellipses::EllipseQuadric& initial_quadric,
                                                      const bool is_ring);
}  // namespace refit