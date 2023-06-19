#pragma once

#include <Eigen/Geometry>
#include <rct_optimizations/types.h>

namespace rct_ros_tools
{
/**
 * \brief Load camera intrinsics from a YAML file
 * \throws rct_ros_tools::BadFileException
 */
template<typename T>
T load(const std::string& path);

/**
 * \brief Load camera intrinsics from a YAML file. Returns false if an error occurs.
 */
template<typename T>
bool load(const std::string& path, T& val);

} // namespace rct_ros_tools
