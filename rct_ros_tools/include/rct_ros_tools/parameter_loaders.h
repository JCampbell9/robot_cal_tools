#pragma once

#include <Eigen/Geometry>
#include <rct_optimizations/types.h>
#include <ros/node_handle.h>

namespace rct_ros_tools
{
/**
 * \brief Load camera intrinsics from a ROS parameter
 * \throws ros::InvalidParameterException
 */
rct_optimizations::CameraIntrinsics loadIntrinsics(const ros::NodeHandle& nh, const std::string& key);

/**
 * \brief Load a target from a ROS parameter. Returns false if an error occurs.
 */
bool loadIntrinsics(const ros::NodeHandle& nh, const std::string& key, rct_optimizations::CameraIntrinsics& intr);

/**
 * \brief Load a pose from a ROS parameter
 * \throws ros::InvalidParameterException
 */
Eigen::Isometry3d loadPose(const ros::NodeHandle& nh, const std::string& key);

/**
 * \brief Load a pose from a ROS parameter. Returns false if an error occurs.
 */
bool loadPose(const ros::NodeHandle& nh, const std::string& key, Eigen::Isometry3d& pose);

} // namespace rct_ros_tools
