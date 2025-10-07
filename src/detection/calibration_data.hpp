#pragma once

#include <map>
#include <optional>
#include <set>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace calib
{

class ImageDecoding;

using PositionId = int;
using CameraKey = std::string;
using ProjectorKey = std::string;
using DecodingBoard = std::map<PositionId, ImageDecoding>;

enum class Difference
{
    INNER_BRIGHTER,
    OUTER_BRIGHTER,
    NO_DIFFERENCE
};

enum class Type
{
    INNER_CORE,
    RING,
    CODING
};

struct MarkerUnidentified
{
    int label_;
    float row_;
    float col_;

    Type type_;

    int width_;
    int height_;

    float black_value_;
    float white_value_;

    MarkerUnidentified(const int label, const float row, const float col, const Type type, const int width,
                       const int height, const float black_value, const float white_value);
};

struct MarkerCoding
{
    int label_ring_;
    float row_;
    float col_;

    int width_ring_;
    int height_ring_;

    float black_value_;
    float white_value_;

    MarkerCoding(const int label_ring, const float row, const float col, const int width_ring, const int height_ring,
                 const float black_value, const float white_value);
};

struct MarkerRing
{
    int global_id_ = -1;

    int lalbe_inner_;
    int label_ring_;

    float row_, col_;

    int width_inner_;
    int height_inner_;
    int width_ring_;
    int height_ring_;

    MarkerRing() = default;
    MarkerRing(const MarkerUnidentified &inner, const MarkerUnidentified &ring);
    MarkerRing(const MarkerCoding &coding);
};

class CodingMarkers
{
   public:
    Eigen::Matrix<std::optional<int>, -1, -1> ordering_;
    std::vector<MarkerRing> markers_;

    /**
     * @brief For projector calibration, fully black markers that has low modulation are producing errors, so we remove
     * them
     */
    CodingMarkers without_coding_markers() const;

    CodingMarkers(const Eigen::Matrix<std::optional<int>, -1, -1> &ordering, const std::vector<MarkerRing> &markers);
    CodingMarkers() = default;
};

/**
 * @brief Stores sensor size localization where detected markers are and what area of sensor is considered as calibrated
 */
class ValidityLocation
{
   public:
    const cv::Mat1b markers_location_;
    const cv::Mat1b calibrated_area_;

    ValidityLocation(const cv::Mat1b &markers_location, const cv::Mat1b &calibrated_area);
};

class ImageDecoding
{
   public:
    const cv::Mat1b linear_input_;

    const cv::Mat1b binary_;
    const cv::Mat1b inverted_binary_;

    const CodingMarkers coding_markers_;

    const ValidityLocation validity_;

    ImageDecoding(const cv::Mat1b &linear_input, const cv::Mat1b &binary, const cv::Mat1b &inverted_binary,
                  const Eigen::Matrix<std::optional<int>, -1, -1> &ordering, const std::vector<MarkerRing> &markers,
                  const cv::Mat1b &markers_location, const cv::Mat1b &calibrated_area);
};

struct MarkerNeighborhood
{
    std::vector<int> neighbors;
    std::vector<float> distances;
    std::vector<float> angles;
};

/**
 * @brief Holds calibration data.
 * Maps from "device name" to decoding data with can be coding for camera
 */
class CalibrationData
{
   public:
    std::set<std::string> device_list_;  // if device is calibrated, it's key will be added here

    // Map from "camera" name to global indexed observation of decoded calibration board
    std::map<CameraKey, DecodingBoard> cameras_decoding_;

    void add_camera_observation(const CameraKey &camera_key, const PositionId position_id,
                                const ImageDecoding &decoding);
};

}  // namespace calib