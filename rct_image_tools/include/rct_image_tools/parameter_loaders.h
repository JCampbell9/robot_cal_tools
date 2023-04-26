#pragma once

#include <Eigen/Geometry>
#include <rct_optimizations/types.h>

namespace rct_ros_tools
{
/**
 * \brief Load camera intrinsics from a YAML file
 * \throws rct_ros_tools::BadFileException
 */
rct_optimizations::CameraIntrinsics loadIntrinsics(const std::string& path);

/**
 * \brief Load camera intrinsics from a YAML file. Returns false if an error occurs.
 */
bool loadIntrinsics(const std::string& path, rct_optimizations::CameraIntrinsics& intrinsics);

/**
 * \brief Load a pose from a YAML file
 * \throws rct_ros_tools::BadFileException
 */
Eigen::Isometry3d loadPose(const std::string& path);

/**
 * \brief Load a pose from a YAML file. Returns false if an error occurs.
 */
bool loadPose(const std::string& path, Eigen::Isometry3d& pose);

} // namespace rct_ros_tools
