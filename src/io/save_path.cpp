#include "save_path.hpp"

#include <cstdlib>
#include <format>

namespace
{
static constexpr const char* const kSavePathEnvName = "DETECTION_SAVE_PATH";

std::filesystem::path default_save_path_unix()
{
    const char* const home = std::getenv("HOME");
    return std::filesystem::path(home) / ".detection";
}

std::filesystem::path default_save_path() { return default_save_path_unix(); }
}  // namespace

namespace io
{
// TODO: Use some singleton so that the path is evaluated only once and not on every call.
// Performance-wise this doesn't matter too much because we call this when saving files which takes much longer than
// getting ENV variables, but still, it would be cleaner.
std::filesystem::path save_path()
{
    std::filesystem::path save_path;

    const char* const env_val = std::getenv(kSavePathEnvName);
    if (env_val == nullptr)
    {
        save_path = default_save_path();
    }
    else
    {
        save_path = env_val;
    }

    std::filesystem::create_directories(save_path);
    return save_path;
}

std::filesystem::path debug_save_path()
{
    const std::filesystem::path debug_save_path = save_path() / "debug";
    std::filesystem::create_directories(debug_save_path);
    return debug_save_path;
}
}  // namespace io
