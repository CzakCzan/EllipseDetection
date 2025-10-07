#include "ellipse_contour.hpp"

#include <atomic>

#include "ellipse_stats.hpp"
#include "fitting.hpp"
#include "utilities.hpp"

namespace
{
constexpr bool kAssumeSigmoidZeroCrossing = true;

constexpr int kWindowSize = 3;
constexpr int kElementsCount = (kWindowSize * 2 + 1) * (kWindowSize * 2 + 1);
constexpr double kStep = 3.0;  // over 360 degrees we step to generate edge crosssections

constexpr int kReiterationCount = 2;
static_assert(kReiterationCount >= 1, "at least one iteration is needed");

constexpr int kMaxReducedIterations = 5;
constexpr int kMaxInnerIterations = 3;
constexpr int kMaxFullIterations = 50;

double mean_to_search(const cv::Mat1b &image, const ellipses::EllipseQuadric &initial_quadric, const bool is_ring)
{
    if (is_ring)
    {
        const auto inner_outer = stats::get_ellipse_inner_outer_ring(image, initial_quadric);
        return (inner_outer.first + inner_outer.second) / 2;
    }
    else
    {
        const auto inner_outer = stats::get_ellipse_inner_outer_white_inner(image, initial_quadric);
        return (inner_outer.first + inner_outer.second) / 2;
    }
}

struct EllipseLocalGeometry
{
    Eigen::Vector2d point_on;
    Eigen::Vector2d normal;
    Eigen::Vector2d tangent;
};

// {cx, cy, major, minor, theta}

EllipseLocalGeometry get_local_geometry(const Eigen::Matrix2d rot_to,
                                        const ellipses::EllipseQuadric::ParameterForm &shape, const double t)
{
    EllipseLocalGeometry local_geometry;

    // generate point on ellipse in ellipse centered coordinate space, and transform that point to image
    local_geometry.point_on =
        rot_to * utils::to_point(shape.major, shape.minor, t * M_PI / 180.0) + Eigen::Vector2d(shape.cx, shape.cy);

    const Eigen::Vector2d in_image_point_prev =
        rot_to * utils::to_point(shape.major, shape.minor, (t - 0.1) * M_PI / 180.0) +
        Eigen::Vector2d(shape.cx, shape.cy);
    const Eigen::Vector2d in_image_point_next =
        rot_to * utils::to_point(shape.major, shape.minor, (t + 0.1) * M_PI / 180.0) +
        Eigen::Vector2d(shape.cx, shape.cy);

    local_geometry.tangent = (in_image_point_next - in_image_point_prev).normalized();
    local_geometry.normal = local_geometry.tangent.unitOrthogonal();

    return local_geometry;
}

bool within_image(const EllipseLocalGeometry &local_geometry, const cv::Mat1b &gray)
{
    const int col_coord = local_geometry.point_on(0);
    const int row_coord = local_geometry.point_on(1);

    if (col_coord + kWindowSize < gray.cols && col_coord - kWindowSize >= 0 && row_coord + kWindowSize < gray.rows &&
        row_coord - kWindowSize >= 0)
    {
        return true;
    }
    return false;
}

struct EdgePoint
{
    const double x;
    const double observation;

    EdgePoint(const double x_in, const double observation_in) : x(x_in), observation(observation_in) {}
};

std::pair<std::vector<EdgePoint>, Eigen::VectorXd> get_observation_and_weight(
    const EllipseLocalGeometry &local_geometry, const cv::Mat1b &gray)
{
    // coordinates in marker image
    const int col_coord = local_geometry.point_on(0);
    const int row_coord = local_geometry.point_on(1);

    Eigen::VectorXd weights_vec(kElementsCount);
    std::vector<EdgePoint> edge_pts;
    edge_pts.reserve(kElementsCount);
    for (int row = row_coord - kWindowSize; row <= row_coord + kWindowSize; ++row)
    {
        for (int col = col_coord - kWindowSize; col <= col_coord + kWindowSize; ++col)
        {
            const Eigen::Vector2d to_vector = (Eigen::Vector2d(col, row) - local_geometry.point_on);
            const double projection = to_vector.dot(local_geometry.normal);

            const double distance = to_vector.dot(local_geometry.tangent);
            // compute weight in form of 1-X^2/(window_size ^2). By that calculation weight decay to 0 at maximal
            // distance
            weights_vec(edge_pts.size()) = (1.0 - distance * distance / ((kWindowSize + 1) * (kWindowSize + 1)));

            edge_pts.emplace_back(projection, double(gray(row, col)));
        }
    }

    return {edge_pts, weights_vec};
}

struct Sigmoid
{
    double A;
    double B;
    double C;
    double D;

    double solve_for_x(const double y) const
    {
        // note that y = <A;D> otherwise it's NaN
        const double nominator = y - A;
        const double denominator = D - y;

        return (-std::log(nominator / denominator) + B * C) / B;
    }
};

Sigmoid guess_sigmoid(const std::vector<EdgePoint> &data)
{
    double min = std::numeric_limits<double>::max();
    double max = 0.0;

    for (const auto &item : data)
    {
        min = std::min(min, item.observation);
        max = std::max(max, item.observation);
    }

    const double A = min - 0.1;
    double C;
    const double D = max + 0.1;

    if constexpr (kAssumeSigmoidZeroCrossing)
    {
        C = 0.0;
    }
    else
    {
        // find C guess as value of "r" closest to mean
        const double mean = (max + min) / 2.0;

        double smallest_distance = std::numeric_limits<double>::max();
        double closest_r = 0.0;

        for (const auto &item : data)
        {
            const double mean_distance = std::abs(item.observation - mean);
            if (mean_distance < smallest_distance)
            {
                smallest_distance = mean_distance;
                closest_r = item.x;
            }
        }
        C = closest_r;
    }

    // solve for B, for that method to work we need to quarantines that A & D are bounding set of observations (if not,
    // it will result in division by 0 or logarithm of negative numbers)
    std::vector<double> b_estimates;

    for (const auto &item : data)
    {
        const double log = std::log((item.observation - A) / (D - item.observation));
        const double divisor = (item.x - C);
        if (std::abs(divisor) > 0.01)
        {
            b_estimates.emplace_back(-log / divisor);
        }
    }

    if (b_estimates.empty())
    {
        return {min, 2.0, C, max};
    }
    else
    {
        std::nth_element(b_estimates.begin(), b_estimates.begin() + b_estimates.size() / 2, b_estimates.end());
        return {min, b_estimates[b_estimates.size() / 2], C, max};
    }
}

double compute_weighted_residual_and_evaluate_norm(const Sigmoid &sigmoid, const std::vector<EdgePoint> &data,
                                                   const Eigen::VectorXd &weights_vec, Eigen::VectorXd &residual)
{
    const double A = sigmoid.A;
    const double B = sigmoid.B;
    const double C = sigmoid.C;
    const double D = sigmoid.D;

    for (size_t element = 0; element < data.size(); ++element)
    {
        const double x = data[element].x;

        const double exponent = std::exp(-B * (x - C));

        residual(element) = (data[element].observation - (D + (A - D) / (1.0 + exponent))) * weights_vec(element);
    }

    return residual.norm();
}

void fill_reduced_jacobian_B_C_and_scaled_residuals(const Sigmoid &sigmoid, const std::vector<EdgePoint> &data,
                                                    Eigen::Matrix<double, -1, 2> &jacobian_reduced,
                                                    const Eigen::VectorXd &weights_vec, Eigen::VectorXd &residual)
{
    const double A = sigmoid.A;
    const double B = sigmoid.B;
    const double C = sigmoid.C;
    const double D = sigmoid.D;

    for (size_t element = 0; element < data.size(); ++element)
    {
        const double x = data[element].x;

        const double exponent = std::exp(-B * (x - C));
        const double exponent_plus_sq = (exponent + 1.0) * (exponent + 1.0);

        const double A_D = A - D;

        jacobian_reduced(element, 0) = -(A_D) * (C - x) * exponent / exponent_plus_sq;
        jacobian_reduced(element, 1) = -B * (A_D)*exponent / exponent_plus_sq;

        residual(element) = (data[element].observation - (D + (A_D) / (1.0 + exponent))) * weights_vec(element);
    }
}

void fill_reduced_jacobian_A_D_and_scaled_residuals(const Sigmoid &sigmoid, const std::vector<EdgePoint> &data,
                                                    Eigen::Matrix<double, -1, 2> &jacobian_reduced,
                                                    const Eigen::VectorXd &weights_vec, Eigen::VectorXd &residual)
{
    const double A = sigmoid.A;
    const double B = sigmoid.B;
    const double C = sigmoid.C;
    const double D = sigmoid.D;

    for (size_t element = 0; element < data.size(); ++element)
    {
        const double x = data[element].x;

        const double exponent = std::exp(-B * (x - C));

        // exploit that e^(-x) == 1/e^x to avoid calculating more exp functions
        const double negative_exponent = 1.0 / exponent;

        const double A_D = A - D;

        jacobian_reduced(element, 0) = 1.0 / (exponent + 1.0);
        jacobian_reduced(element, 1) = 1.0 / (negative_exponent + 1.0);

        residual(element) = (data[element].observation - (D + (A_D) / (1.0 + exponent))) * weights_vec(element);
    }
}

void fill_jacobian_and_scaled_residuals(const Sigmoid &sigmoid, const std::vector<EdgePoint> &data,
                                        Eigen::Matrix<double, -1, 4> &jacobian, const Eigen::VectorXd &weights_vec,
                                        Eigen::VectorXd &residual)
{
    const double A = sigmoid.A;
    const double B = sigmoid.B;
    const double C = sigmoid.C;
    const double D = sigmoid.D;

    for (size_t element = 0; element < data.size(); ++element)
    {
        const double x = data[element].x;

        const double exponent = std::exp(-B * (x - C));
        const double exponent_plus_sq = (exponent + 1.0) * (exponent + 1.0);

        // exploit that e^(-x) == 1/e^x to avoid calculating more exp functions
        const double negative_exponent = 1.0 / exponent;

        const double A_D = A - D;

        jacobian(element, 0) = 1.0 / (exponent + 1.0);
        jacobian(element, 1) = -(A_D) * (C - x) * exponent / exponent_plus_sq;
        jacobian(element, 2) = -B * (A_D)*exponent / exponent_plus_sq;
        jacobian(element, 3) = 1.0 / (negative_exponent + 1.0);

        residual(element) = (data[element].observation - (D + (A_D) / (1.0 + exponent))) * weights_vec(element);
    }
}

bool update_lambda_and_check_inner_iterations(const double current_error, double &previous_error, double &lambda)
{
    if (current_error > previous_error)
    {
        // increase lambda
        lambda *= 2;
        return false;
    }
    else
    {
        previous_error = current_error;
        lambda /= 10.0;
        return true;
    }
}

}  // namespace

namespace ellipse_contour
{
std::optional<ellipses::EllipseQuadric> refit_quadric(const cv::Mat1b &gray,
                                                      const ellipses::EllipseQuadric &initial_quadric,
                                                      const bool is_ring)
{
    const double mean_searched = mean_to_search(gray, initial_quadric, is_ring);

    ellipses::EllipseQuadric::ParameterForm quadric = initial_quadric.parameters_;

    // given shape is basis for search of proper edges
    const Eigen::Matrix2d rot_to = utils::to_rotation_matrix(-quadric.theta).transpose();

    Eigen::Matrix<double, -1, 4> jacobian(kElementsCount, 4);
    Eigen::Matrix<double, -1, 2> jacobian_reduced(kElementsCount, 2);
    Eigen::Matrix<double, 4, 4> jacobian_t_jacobian;
    Eigen::Matrix<double, 2, 2> jacobian_red_t_jacobian_red;
    Eigen::VectorXd weights_vec(kElementsCount);
    Eigen::VectorXd residual(kElementsCount);
    Eigen::Vector4d rhs;
    Eigen::Vector2d rhs_reduced;

    const int items = std::round(360.0 / kStep);

    // we need to create said control flow value as openMp does not allow for "break" statement
    std::atomic<bool> can_continue = true;

    for (int fit_reiteration = 0; fit_reiteration < kReiterationCount; ++fit_reiteration)
    {
        // we want to preserve order of point on ellipse due to drawing requirements
        std::vector<Eigen::Vector2d> contour_sub_pix(items, Eigen::Vector2d::Zero());
        std::vector<double> contour_weight(items, 0.0);

        // To that solver 2 improvements can be made.
        // 1) Lambda update handling -> there are papers how to do it
        // 2) Instead on sphere regularization, use diag(Lhs) as an replacement (implicit scalling between variables)
        // #pragma omp parallel for firstprivate(jacobian, jacobian_reduced, jacobian_t_jacobian,
        // jacobian_red_t_jacobian_red, \
//                                           weights_vec, residual, rhs, rhs_reduced)
        for (int idx = 0; idx < items; ++idx)
        {
            if (!can_continue)
            {
                continue;
            }

            const double t = double(idx) * kStep;

            const auto local_geometry = get_local_geometry(rot_to, quadric, t);

            if (!within_image(local_geometry, gray))
            {
                can_continue = false;
                continue;
            }

            const auto [obs, weight] = get_observation_and_weight(local_geometry, gray);

            auto sigmoid = guess_sigmoid(obs);
            auto sigmoid_old = sigmoid;
            double previous_error = compute_weighted_residual_and_evaluate_norm(sigmoid, obs, weight, residual);
            double lambda = obs.size() / 10.0;
            Eigen::Vector2d update_reduced;
            // first optimize ONLY B and C
            for (int iterations = 0; iterations < kMaxReducedIterations; ++iterations)
            {
                fill_reduced_jacobian_B_C_and_scaled_residuals(sigmoid, obs, jacobian_reduced, weight, residual);

                jacobian_red_t_jacobian_red.noalias() =
                    jacobian_reduced.transpose() * weight.asDiagonal() * jacobian_reduced;
                rhs_reduced.noalias() = jacobian_reduced.transpose() * residual;

                bool success_inner = false;
                for (int inner_iteration = 0; inner_iteration < kMaxInnerIterations; ++inner_iteration)
                {
                    update_reduced.noalias() =
                        (jacobian_red_t_jacobian_red + Eigen::Matrix2d::Identity() * lambda).ldlt().solve(rhs_reduced);

                    sigmoid.B = sigmoid_old.B + update_reduced(0);
                    sigmoid.C = sigmoid_old.C + update_reduced(1);

                    const double current_error =
                        compute_weighted_residual_and_evaluate_norm(sigmoid, obs, weight, residual);

                    if (update_lambda_and_check_inner_iterations(current_error, previous_error, lambda))
                    {
                        success_inner = true;
                        break;
                    }
                }

                if (success_inner)
                {
                    sigmoid_old = sigmoid;
                }
                else
                {
                    break;
                }

                if (update_reduced.norm() < std::pow(10.0, -4))
                {
                    break;
                }
            }

            // first optimize ONLY A and D
            for (int iterations = 0; iterations < kMaxReducedIterations; ++iterations)
            {
                fill_reduced_jacobian_A_D_and_scaled_residuals(sigmoid, obs, jacobian_reduced, weight, residual);

                jacobian_red_t_jacobian_red.noalias() =
                    jacobian_reduced.transpose() * weight.asDiagonal() * jacobian_reduced;
                rhs_reduced.noalias() = jacobian_reduced.transpose() * residual;

                bool success_inner = false;
                for (int inner_iteration = 0; inner_iteration < kMaxInnerIterations; ++inner_iteration)
                {
                    update_reduced.noalias() =
                        (jacobian_red_t_jacobian_red + Eigen::Matrix2d::Identity() * lambda).ldlt().solve(rhs_reduced);

                    sigmoid.A = sigmoid_old.A + update_reduced(0);
                    sigmoid.D = sigmoid_old.D + update_reduced(1);

                    const double current_error =
                        compute_weighted_residual_and_evaluate_norm(sigmoid, obs, weight, residual);
                    if (update_lambda_and_check_inner_iterations(current_error, previous_error, lambda))
                    {
                        success_inner = true;
                        break;
                    }
                }

                if (success_inner)
                {
                    sigmoid_old = sigmoid;
                }
                else
                {
                    break;
                }

                if (update_reduced.norm() < std::pow(10.0, -4))
                {
                    break;
                }
            }

            Eigen::Vector4d update;
            // now iterate using all values
            for (int iterations = 0; iterations < kMaxFullIterations; ++iterations)
            {
                fill_jacobian_and_scaled_residuals(sigmoid, obs, jacobian, weight, residual);

                jacobian_t_jacobian.noalias() = jacobian.transpose() * weight.asDiagonal() * jacobian;
                rhs.noalias() = jacobian.transpose() * residual;

                bool success_inner = false;
                for (int inner_iteration = 0; inner_iteration < kMaxInnerIterations; ++inner_iteration)
                {
                    update = (jacobian_t_jacobian + Eigen::Matrix4d::Identity() * lambda).ldlt().solve(rhs);

                    sigmoid.A = sigmoid_old.A + update(0);
                    sigmoid.B = sigmoid_old.B + update(1);
                    sigmoid.C = sigmoid_old.C + update(2);
                    sigmoid.D = sigmoid_old.D + update(3);

                    const double current_error =
                        compute_weighted_residual_and_evaluate_norm(sigmoid, obs, weight, residual);
                    if (update_lambda_and_check_inner_iterations(current_error, previous_error, lambda))
                    {
                        success_inner = true;
                        break;
                    }
                }

                if (success_inner)
                {
                    sigmoid_old = sigmoid;
                }
                else
                {
                    break;
                }

                if (update.norm() < std::pow(10.0, -4))
                {
                    break;
                }
            }

            if (!std::isfinite(sigmoid.C) || std::abs(sigmoid.C) > kWindowSize)
            {
                continue;
            }

            const double sigmoid_crossing = sigmoid.solve_for_x(mean_searched);

            if (!std::isfinite(sigmoid_crossing))
            {
                continue;
            }
            const Eigen::Vector2d crossed = local_geometry.point_on + local_geometry.normal * sigmoid_crossing;

#pragma omp critical
            {
                contour_sub_pix[idx] = crossed;
                contour_weight[idx] = (sigmoid.A - sigmoid.D);
            }
        }

        if (can_continue)
        {
            // collapse non found points and refit ellipses
            std::vector<Eigen::Vector2d> concentrated_points;
            concentrated_points.reserve(items);
            for (const auto &pts : contour_sub_pix)
            {
                if (pts(0) != 0.0)
                {
                    concentrated_points.emplace_back(pts);
                }
            }

            quadric = fitting::fit_ellipse(concentrated_points).parameters_;
        }
    }
    if (can_continue)
    {
        return {quadric};
    }
    return std::nullopt;
}
}  // namespace ellipse_contour