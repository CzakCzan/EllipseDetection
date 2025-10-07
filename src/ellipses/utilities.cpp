#include "utilities.hpp"

namespace utils
{
Eigen::Matrix2d to_rotation_matrix(const double theta)
{
    Eigen::Matrix2d in_plane_rot;
    in_plane_rot(0, 0) = std::cos(theta);
    in_plane_rot(0, 1) = -std::sin(theta);
    in_plane_rot(1, 0) = std::sin(theta);
    in_plane_rot(1, 1) = std::cos(theta);

    return in_plane_rot;
}

Eigen::Vector2d to_point(const double major, const double minor, const double t)
{
    return Eigen::Vector2d(major * std::cos(t), minor * sin(t));
}

}  // namespace utils