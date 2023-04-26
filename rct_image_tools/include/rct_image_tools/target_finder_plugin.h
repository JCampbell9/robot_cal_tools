#pragma once

#include <rct_image_tools/target.h>
#include <rct_image_tools/target_finder.h>

#include <string>
#include <yaml-cpp/node/node.h>

namespace rct_image_tools
{
/**
 * @brief Target finder plugin that can be initialized by file or XmlRpcValue (i.e. from a ROS parameter)
 */
class TargetFinderPlugin : public rct_image_tools::TargetFinder
{
public:
  using Ptr = std::shared_ptr<TargetFinderPlugin>;
  using ConstPtr = std::shared_ptr<const TargetFinderPlugin>;
  using TargetFinder::TargetFinder;

  rct_image_tools::TargetFeatures findTargetFeatures(const cv::Mat& image) const override
  {
    return finder_->findTargetFeatures(image);
  }

  cv::Mat drawTargetFeatures(const cv::Mat& image,
                             const rct_image_tools::TargetFeatures& target_features) const override
  {
    return finder_->drawTargetFeatures(image, target_features);
  }

  const rct_image_tools::Target& target() const override
  {
    return finder_->target();
  }

  virtual void init(const YAML::Node& config) = 0;

protected:
  rct_image_tools::TargetFinder::ConstPtr finder_;
};

} // namespace rct_ros_tools
