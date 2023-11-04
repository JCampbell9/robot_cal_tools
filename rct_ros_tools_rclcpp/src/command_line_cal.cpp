#include <rct_image_tools/data_set.h>
#include <rct_image_tools/parameter_loaders.h>
#include <rct_image_tools/target_finder_plugin.h>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

template <typename T>
T declare_and_get(rclcpp::Node* node, const std::string& key)
{
  T val;
  node->declare_parameter(key, std::string());
  if (!node->get_parameter(key, val))
    throw(std::runtime_error("Failed to load '" + key + "' parameter"));
  return val;
}

class DataCollection : public rclcpp::Node
{
public:
  explicit DataCollection()
    : Node("command_line_cal_node")
    , clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
    , buffer_(clock_)
    , listener_(buffer_)
  {
    base_frame_ = declare_and_get<std::string>(this, "base_frame");
    tool_frame_ = declare_and_get<std::string>(this, "tool_frame");
    image_topic_ = declare_and_get<std::string>(this, "image_topic");
    save_dir_ = declare_and_get<std::string>(this, "save_dir");
    auto target_file = declare_and_get<std::string>(this, "target_file");

    it_sub_ = image_transport::create_subscription(
        this, image_topic_, std::bind(&DataCollection::onNewImage, this, std::placeholders::_1), "raw");
    it_pub_ = image_transport::create_publisher(this, image_topic_ + "_out");

    trigger_server_ = create_service<std_srvs::srv::Trigger>(
        "collect", std::bind(&DataCollection::onTrigger, this, std::placeholders::_1, std::placeholders::_2));
    save_server_ = create_service<std_srvs::srv::Trigger>(
        "save", std::bind(&DataCollection::onSave, this, std::placeholders::_1, std::placeholders::_2));

    // Configure the plugin loader
    loader_.search_libraries.insert(RCT_TARGET_PLUGINS);

    // Load the target finder plugin
    const YAML::Node target_config = YAML::LoadFile(target_file);
    finder_ = loader_.createInstance<rct_image_tools::TargetFinderPlugin>(target_config["type"].as<std::string>());
    finder_->init(target_config);
  }

private:
  void onNewImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      if (msg->encoding == "mono16")
      {
        cv_bridge::CvImagePtr temp_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);

        cv::Mat img_conv;
        cv::cvtColor(temp_ptr->image, img_conv, CV_GRAY2BGR);
        img_conv.convertTo(img_conv, CV_8UC1);
        cv_ptr = cv_bridge::CvImagePtr(
            new cv_bridge::CvImage(temp_ptr->header, sensor_msgs::image_encodings::BGR8, img_conv));
      }
      else
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      last_frame_ = cv_ptr;

      try
      {
        cv::Mat tmp_image = cv_ptr->image.clone();
        rct_image_tools::TargetFeatures image_observations = finder_->findTargetFeatures(tmp_image);
        cv::Mat modified_image = finder_->drawTargetFeatures(tmp_image, image_observations);
        cv_bridge::CvImagePtr ptr(new cv_bridge::CvImage(cv_ptr->header, cv_ptr->encoding, modified_image));
        it_pub_.publish(ptr->toImageMsg());
      }
      catch (const std::runtime_error& ex)
      {
        RCLCPP_ERROR(get_logger(), ex.what());
        it_pub_.publish(cv_ptr->toImageMsg());
      }
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
      return;
    }
  }

  void onTrigger(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Pose/Image capture triggered...");
    geometry_msgs::msg::TransformStamped pose;
    cv::Mat image;

    if (captureTransform(pose) && captureImage(image))
    {
      poses_.push_back(pose);
      images_.push_back(image);
      res->success = true;
      res->message = "Data collected successfully";
    }
    else
    {
      res->success = false;
      res->message = "Failed to capture pose/image pair";
    }
  }

  void onSave(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    rct_ros_tools::ExtrinsicDataSet data;
    for (std::size_t i = 0; i < poses_.size(); ++i)
    {
      cv::Mat image = images_[i];
      auto msg = poses_[i];

      Eigen::Isometry3d pose = tf2::transformToEigen(msg);
      data.images.push_back(image);
      data.tool_poses.push_back(pose);
    }

    res->success = rct_ros_tools::saveToDirectory(save_dir_, data);
    res->message = res->success ? "Success" : "Failed to save data";
  }

  bool captureImage(cv::Mat& frame)
  {
    if (last_frame_)
    {
      frame = last_frame_->image;
      return true;
    }
    return false;
  }

  bool captureTransform(geometry_msgs::msg::TransformStamped& out)
  {
    try
    {
      geometry_msgs::msg::TransformStamped t = buffer_.lookupTransform(base_frame_, tool_frame_, tf2::TimePointZero,
                                                                       tf2::Duration(std::chrono::seconds(10)));
      out = t;
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN_STREAM(get_logger(), "Failed to compute transfrom between " << base_frame_ << " and " << tool_frame_
                                                                              << ": " << ex.what());
      return false;
    }
  }
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_server_;

  image_transport::Subscriber it_sub_;
  image_transport::Publisher it_pub_;

  std::string base_frame_;
  std::string tool_frame_;
  std::string image_topic_;
  std::string save_dir_;

  std::vector<geometry_msgs::msg::TransformStamped> poses_;
  std::vector<cv::Mat> images_;

  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  cv_bridge::CvImagePtr last_frame_;

  // Target finder plugin
  boost_plugin_loader::PluginLoader loader_;
  rct_image_tools::TargetFinderPlugin::Ptr finder_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DataCollection>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
