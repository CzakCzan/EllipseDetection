#pragma once

#include "ellipses.hpp"

namespace utils
{
/**
 * @brief Given ellipse shape we can generate points in (or out) parametrized form. In case of generation of point "from
 * circle to transformed" said function could be called with NEGATIVE sign of ellipse theta (transform point from unit
 * circle to elongated ellipse)
 *
 * @param theta rotation angle in RADIANS
 */
Eigen::Matrix2d to_rotation_matrix(const double theta);

/**
 * @brief Given ellipse shape we can generate points in canonical form (X axis align with MAJOR ellipse axis, Y axis
 * align with MINOR).
 *
 * @param major axis of ellipse
 * @param minor axis of ellipse
 * @param t angle on UNIT CIRCLE for with we want to generate point (in RADIANS)
 */
Eigen::Vector2d to_point(const double major, const double minor, const double t);
}  // namespace utils