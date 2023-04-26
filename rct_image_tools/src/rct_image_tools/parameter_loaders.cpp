#include <rct_image_tools/parameter_loaders.h>
#include <rct_image_tools/exceptions.h>
#include <rct_optimizations/serialization/eigen.h>
#include <rct_optimizations/serialization/types.h>

#include <console_bridge/console.h>

namespace rct_ros_tools
{
rct_optimizations::CameraIntrinsics loadIntrinsics(const std::string& path)
{
  try
  {
    YAML::Node n = YAML::LoadFile(path);
    return n.as<rct_optimizations::CameraIntrinsics>();
  }
  catch (YAML::Exception &ex)
  {
    throw BadFileException(std::string("YAML failure: ") + ex.what());
  }
}

bool loadIntrinsics(const std::string& path, rct_optimizations::CameraIntrinsics& intrinsics)
{
  try
  {
    intrinsics = loadIntrinsics(path);
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

Eigen::Isometry3d loadPose(const std::string& path)
{
  try
  {
    YAML::Node n = YAML::LoadFile(path);
    return n.as<Eigen::Isometry3d>();
  }
  catch (YAML::Exception &ex)
  {
    throw BadFileException(std::string("YAML failure: ") + ex.what());
  }
}

bool loadPose(const std::string& path, Eigen::Isometry3d& pose)
{
  try
  {
    pose = loadPose(path);
  }
  catch (BadFileException &ex)
  {
    std::stringstream ss;
    ss << "Failed to load pose from file: " << ex.what();
    CONSOLE_BRIDGE_logError(ss.str().c_str());
    return false;
  }
  return true;
}

} // namespace rct_ros_tools

