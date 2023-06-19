#include <rct_image_tools/parameter_loaders.h>
#include <rct_image_tools/exceptions.h>
#include <rct_optimizations/serialization/eigen.h>
#include <rct_optimizations/serialization/types.h>

#include <console_bridge/console.h>

namespace rct_ros_tools
{
template <typename T>
T load(const std::string& path)
{
  try
  {
    YAML::Node n = YAML::LoadFile(path);
    return n.as<T>();
  }
  catch (YAML::Exception &ex)
  {
    throw BadFileException(std::string("YAML failure: ") + ex.what());
  }
}

template <typename T>
bool load(const std::string& path, T& intrinsics)
{
  try
  {
    intrinsics = load<T>(path);
  }
  catch (BadFileException &ex)
  {
    std::stringstream ss;
    ss << "Failed to load intrinsics from file: " << ex.what();
    CONSOLE_BRIDGE_logError(ss.str().c_str());
    return false;
  }
  return true;
}

template rct_optimizations::CameraIntrinsics load(const std::string& path);
template bool load(const std::string& path, rct_optimizations::CameraIntrinsics& pose);

template Eigen::Isometry3d load(const std::string& path);
template bool load(const std::string& path, Eigen::Isometry3d& pose);

} // namespace rct_ros_tools

