// Utilities for loading data sets and calib parameters from YAML files via ROS
#include <rct_common/print_utils.h>
#include <rct_image_tools/data_set.h>
#include <rct_image_tools/parameter_loaders.h>
#include <rct_image_tools/target_finder_plugin.h>
// The calibration function for 'moving camera' on robot wrist
#include <rct_optimizations/extrinsic_hand_eye.h>
#include <rct_optimizations/serialization/problems.h>
#include <rct_optimizations/validation/homography_validation.h>
// Calibration analysis
#include <rct_image_tools/hand_eye_calibration_analysis.h>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <yaml-cpp/yaml.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;
using namespace rct_common;

const std::string WINDOW = "window";

template <typename T>
T declare_and_get(rclcpp::Node* node, const std::string& key)
{
  T val;
  node->declare_parameter(key);
  if (!node->get_parameter(key, val))
    throw(std::runtime_error("Failed to load '" + key + "' parameter"));
  return val;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hand_eye_calibration");

  try
  {
    // Create our calibration problem
    ExtrinsicHandEyeProblem2D3D problem;

    // Load the file location for the output YAML file
    auto output_file = declare_and_get<std::string>(node.get(), "output_file");

    // Load the homography threshold used to check detected target patterns
    auto homography_threshold = declare_and_get<double>(node.get(), "homography_threshold");

    // Load the camera intrinsics
    problem.intr = loadIntrinsics(declare_and_get<std::string>(node.get(), "intrinsics"));

    // Attempt to load the data set
    auto data_path = declare_and_get<std::string>(node.get(), "data_path");
    boost::optional<ExtrinsicDataSet> maybe_data_set = parseFromFile(data_path);
    if (!maybe_data_set)
      throw std::runtime_error("Failed to parse data set from path = " + data_path);

    // We know it exists, so define a helpful alias
    const ExtrinsicDataSet& data_set = *maybe_data_set;

    // Lets load a plugin that will search for the target in our raw images.
    TargetFinderPlugin::Ptr target_finder;
    boost_plugin_loader::PluginLoader loader;
    {
      // Configure the plugin loader
      loader.search_libraries.insert(RCT_TARGET_PLUGINS);

      auto target_file = declare_and_get<std::string>(node.get(), "target_file");
      const YAML::Node target_config = YAML::LoadFile(target_file);
      target_finder =
          loader.createInstance<rct_image_tools::TargetFinderPlugin>(target_config["type"].as<std::string>());
      target_finder->init(target_config);
    }

    // Load the calibration transform guesses using TF
    {
      tf2_ros::Buffer buffer(node->get_clock());
      tf2_ros::TransformListener listener(buffer);

      auto target_mount_frame = declare_and_get<std::string>(node.get(), "target_mount_frame");
      auto target_frame = declare_and_get<std::string>(node.get(), "target_frame");
      auto camera_mount_frame = declare_and_get<std::string>(node.get(), "camera_mount_frame");
      auto camera_frame = declare_and_get<std::string>(node.get(), "camera_frame");

      tf2::Duration timeout(std::chrono::seconds(5));
      problem.target_mount_to_target_guess =
          tf2::transformToEigen(buffer.lookupTransform(target_mount_frame, target_frame, tf2::TimePointZero, timeout));
      problem.camera_mount_to_camera_guess =
          tf2::transformToEigen(buffer.lookupTransform(camera_mount_frame, camera_frame, tf2::TimePointZero, timeout));
    }

    // Create a named OpenCV window for viewing the images
    cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);

    // Finally, we need to process our images into correspondence sets: for each dot in the
    // target this will be where that dot is in the target and where it was seen in the image.
    // Repeat for each image. We also tell where the wrist was when the image was taken.
    problem.observations.reserve(data_set.images.size());

    // The target may not be identified in all images, so let's keep track the indices of the images for which the
    // target was identified
    std::vector<cv::Mat> found_images;
    found_images.reserve(data_set.images.size());

    for (std::size_t i = 0; i < data_set.images.size(); ++i)
    {
      // For each image we need to:
      //// 1. Try to find the target features in this image:
      rct_image_tools::TargetFeatures target_features;
      try
      {
        target_features = target_finder->findTargetFeatures(data_set.images[i]);
        if (target_features.empty())
          throw std::runtime_error("Failed to find any target features");
        RCLCPP_INFO_STREAM(node->get_logger(), "Found " << target_features.size() << " target features");

        Observation2D3D obs;
        obs.correspondence_set.reserve(target_features.size());

        //// 2. Record the wrist position and target features
        obs.to_camera_mount = data_set.tool_poses[i];
        obs.to_target_mount = Eigen::Isometry3d::Identity();
        obs.correspondence_set = target_finder->target().createCorrespondences(target_features);

        //// 3. Check that a homography matrix can accurately reproject the observed points onto the expected target
        ///     points within a defined threshold
        rct_optimizations::RandomCorrespondenceSampler random_sampler(obs.correspondence_set.size(),
                                                                      obs.correspondence_set.size() / 3);
        Eigen::VectorXd homography_error =
            rct_optimizations::calculateHomographyError(obs.correspondence_set, random_sampler);
        if (homography_error.array().mean() > homography_threshold)
          throw std::runtime_error("Homography error exceeds threshold (" + std::to_string(homography_error.array().mean()) + ")");

        //// 3. And finally add that to the problem
        problem.observations.push_back(obs);
        found_images.push_back(data_set.images[i]);

        // Show the points we detected
        cv::imshow(WINDOW, target_finder->drawTargetFeatures(data_set.images[i], target_features));
        cv::waitKey();
      }
      catch (const std::runtime_error& ex)
      {
        RCLCPP_WARN_STREAM(node->get_logger(), "Image " << i << ": '" << ex.what() << "'");
        cv::imshow(WINDOW, data_set.images[i]);
        cv::waitKey();
        continue;
      }
    }

    // Now we have a defined problem, run optimization:
    ExtrinsicHandEyeResult opt_result = optimize(problem);

    // Report results
    printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
    printNewLine();

    printTransform(opt_result.camera_mount_to_camera, "Camera Mount", "Camera", "CAMERA MOUNT TO CAMERA");
    printNewLine();

    printTransform(opt_result.target_mount_to_target, "Target Mount", "Target", "TARGET MOUNT TO TARGET");
    printNewLine();

    std::cout << opt_result.covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;

    // Now let's compare the results of our extrinsic calibration with a PnP optimization for every observation.
    // The PnP optimization will give us an estimate of the camera to target transform using our input camera intrinsic
    // parameters We will then see how much this transform differs from the same transform calculated using the results
    // of the extrinsic calibration
    analyzeResults(problem, opt_result, found_images, WINDOW);

    // Save
    {
      std::ofstream fh(output_file);
      if (!fh)
        throw std::runtime_error("Failed to open file '" + output_file + "'");
      fh << YAML::Node(opt_result);
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), ex.what());
    return -1;
  }

  return 0;
}
