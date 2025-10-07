#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <ellipses/ellipse_contour.hpp>

int main(int argc, char* argv[])
{
    const int radius_on_example_data = 45;

    cv::Mat1b grayscale =
        cv::imread("/home/czak/ellipses/EllipseDetection/src/example_data/pattern_acircles.png", cv::IMREAD_GRAYSCALE);

    // for image that infinite response function (just binarized edge) method diverge, if real data is used, do not blur
    // image!
    cv::GaussianBlur(grayscale, grayscale, cv::Size(3, 3), 0.0);

    const cv::Size grid_size(4, 11);

    cv::SimpleBlobDetector::Params params;
    params.maxArea = 8000;
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::Point2f> centers;

    const bool success = cv::findCirclesGrid(grayscale, grid_size, centers, cv::CALIB_CB_ASYMMETRIC_GRID, detector);

    if (success)
    {
        // initialize quadric and refit them
        for (const auto& center : centers)
        {
            ellipses::EllipseQuadric quadric(ellipses::EllipseQuadric::ParameterForm(
                radius_on_example_data, radius_on_example_data, center.x, center.y, 0.0));

            const auto quad = ellipse_contour::refit_quadric(grayscale, quadric, false);
            if (quad.has_value())
            {
                quadric = quad.value();
            }
        }
    }
    else
    {
        return EXIT_FAILURE;
    }
}
