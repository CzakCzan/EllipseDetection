#include "debug.hpp"

#include <opencv2/imgcodecs.hpp>

#include "save_path.hpp"

namespace io::debug
{
void save_image(const cv::Mat1b &img, const std::string &name)
{
    const std::filesystem::path dir_path = debug_save_path();
    std::filesystem::create_directories(dir_path);

    cv::imwrite(dir_path / (name + ".png"), img);
}

void save_image(const cv::Mat3b &img, const std::string &name)
{
    const std::filesystem::path dir_path = debug_save_path();
    std::filesystem::create_directories(dir_path);

    cv::imwrite(dir_path / (name + ".png"), img);
}
}  // namespace io::debug