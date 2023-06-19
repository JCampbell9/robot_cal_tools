#include <rct_image_tools/data_set.h>
#include <rct_image_tools/parameter_loaders.h>
#include <rct_image_tools/target_finder_plugin.h>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <yaml-cpp/yaml.h>

template <typename T>
T declare_and_get(rclcpp::Node* node, const std::string& key)
{
  T val;
  node->declare_parameter(key);
  if (!node->get_parameter(key, val))
    throw(std::runtime_error("Failed to load '" + key + "' parameter"));
  return val;
}

template <typename T>
T getLatestMessage(typename rclcpp::Subscription<T>::SharedPtr sub)
{
  rclcpp::WaitSet wait;
  wait.add_subscription(sub);
  auto ret = wait.wait(std::chrono::seconds(3));
  switch (ret.kind())
  {
    case rclcpp::WaitResultKind::Ready:
      break;
    default:
      throw std::runtime_error("Failed to get message within timeout");
  }

  T msg;
  rclcpp::MessageInfo info;
  if (!sub->take(msg, info))
  {
    std::stringstream ss;
    ss << "Failed to get latest message on topic '" << sub->get_topic_name() << "'";
    throw std::runtime_error(ss.str());
  }
  return msg;
}

std::vector<double> getJointValues(const sensor_msgs::msg::JointState& msg, const std::vector<std::string>& joint_names)
{
  std::vector<double> values(joint_names.size());
  for (const std::string& name : joint_names)
  {
    auto it = std::find(msg.name.begin(), msg.name.end(), name);
    if (it == msg.name.end())
      throw std::runtime_error("Failed to find joint '" + name + "' in joint state message");

    auto idx = std::distance(msg.name.begin(), it);
    values.push_back(msg.position.at(idx));
  }

  return values;
}

class DataCollection : public rclcpp::Node
{
public:
  explicit DataCollection()
    : Node("command_line_cal_node")
    , base_frame_(declare_and_get<std::string>(this, "base_frame"))
    , tool_frame_(declare_and_get<std::string>(this, "tool_frame"))
    , image_topic_(declare_and_get<std::string>(this, "image_topic"))
    , save_dir_(declare_and_get<std::string>(this, "save_dir"))
    , camera_chain_joints_(declare_and_get<std::vector<std::string>>(this, "camera_chain_joints"))
    , target_chain_joints_(declare_and_get<std::vector<std::string>>(this, "target_chain_joints"))
    , clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
    , buffer_(clock_)
    , listener_(buffer_)
  {
    auto target_file = declare_and_get<std::string>(this, "target_file");

    it_sub_ = image_transport::create_subscription(
        this, image_topic_, std::bind(&DataCollection::onNewImage, this, std::placeholders::_1), "raw");
    it_pub_ = image_transport::create_publisher(this, image_topic_ + "_out");

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, [](const sensor_msgs::msg::JointState::SharedPtr) {});

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
    try
    {
      RCLCPP_INFO_STREAM(get_logger(), "Pose/Image capture triggered...");

      // Wait for a new joint state message
      auto js = getLatestMessage<sensor_msgs::msg::JointState>(joint_state_sub_);
      std::vector<double> camera_joints = getJointValues(js, camera_chain_joints_);
      std::vector<double> target_joints = getJointValues(js, target_chain_joints_);

      // Check the latest image
      if (!last_frame_)
        throw std::runtime_error("No image yet acquired");

      if ((rclcpp::Time(last_frame_->header.stamp) - rclcpp::Time(js.header.stamp)) > rclcpp::Duration(1))
        throw std::runtime_error("Last image time stamp differs by more than 1 second from the last joint state "
                                 "message");

      // Look up the transform
      geometry_msgs::msg::TransformStamped t = buffer_.lookupTransform(
          base_frame_, tool_frame_, last_frame_->header.stamp, tf2::Duration(std::chrono::seconds(3)));

      // Save the data
      data_.images.push_back(last_frame_->image);
      data_.tool_poses.push_back(tf2::transformToEigen(t));
      data_.camera_chain_joints.push_back(camera_joints);
      data_.target_chain_joints.push_back(target_joints);

      res->success = true;
      res->message = "Data collected successfully";
    }
    catch (const std::exception& ex)
    {
      res->message = ex.what();
      res->success = false;
    }
  }

  void onSave(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    res->success = rct_ros_tools::saveToDirectory(save_dir_, data_);
    res->message = res->success ? "Success" : "Failed to save data";
  }

  // Parameters
  const std::string base_frame_;
  const std::string tool_frame_;
  const std::string image_topic_;
  const std::string save_dir_;
  const std::vector<std::string> camera_chain_joints_;
  const std::vector<std::string> target_chain_joints_;

  // Data
  cv_bridge::CvImagePtr last_frame_;
  rct_ros_tools::ExtrinsicDataSet data_;

  // Data subscribers
  image_transport::Subscriber it_sub_;
  image_transport::Publisher it_pub_;

  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // ROS interfaces
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_server_;

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
