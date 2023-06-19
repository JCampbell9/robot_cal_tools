#include <rct_image_tools/data_set.h>
#include <rct_image_tools/parameter_loaders.h>
#include <rct_image_tools/image_utils.h>
#include <rct_optimizations/serialization/eigen.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <console_bridge/console.h>
#include <sys/stat.h>

static std::string rootPath(const std::string& filename)
{
  const auto last_idx = filename.find_last_of('/');
  assert(last_idx != std::string::npos);
  return filename.substr(0, last_idx);
}

static std::string combine(const std::string& dir, const std::string& rel_path)
{
  return dir + "/" + rel_path;
}

cv::Mat rct_ros_tools::readImageOpenCV(const std::string& path)
{
  cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
  if (image.data == nullptr)
    throw std::runtime_error("File failed to load or does not exist: '" + path + "'");
  return image;
}

rct_ros_tools::ExtrinsicDataSet parse(const YAML::Node& root, const std::string& root_path)
{
  rct_ros_tools::ExtrinsicDataSet data;

  for (std::size_t i = 0; i < root.size(); ++i)
  {
    // Each entry should have a pose and image path. This path is relative to the root_path directory!
    const auto img_path = root[i]["image"].as<std::string>();
    const auto pose_path = root[i]["pose"].as<std::string>();
    const auto camera_joints_path = root[i]["camera_joints"].as<std::string>();
    const auto target_joints_path = root[i]["target_joints"].as<std::string>();
    cv::Mat image = rct_ros_tools::readImageOpenCV(combine(root_path, img_path));

    if (image.empty())
    {
      CONSOLE_BRIDGE_logWarn("Failed to load image %i. Skipping...", i);
      continue;
    }

    Eigen::Isometry3d p;
    if (!rct_ros_tools::load(combine(root_path, pose_path), p))
    {
      CONSOLE_BRIDGE_logWarn("Failed to load pose %i. Skipping...", i);
      continue;
    }

    std::vector<double> camera_joints;
    if (!rct_ros_tools::load(camera_joints_path, camera_joints))
    {
      CONSOLE_BRIDGE_logWarn("Failed to load camera joints %i. Skipping...", i);
      continue;
    }

    std::vector<double> target_joints;
    if(!rct_ros_tools::load(target_joints_path, target_joints))
    {
      CONSOLE_BRIDGE_logWarn("Failed to load target joints %i. Skipping...", i);
      continue;
    }

    data.images.push_back(image);
    data.tool_poses.push_back(p);
    data.camera_chain_joints.push_back(camera_joints);
    data.target_chain_joints.push_back(target_joints);
  }

  return data;
}

boost::optional<rct_ros_tools::ExtrinsicDataSet> rct_ros_tools::parseFromFile(const std::string &path)
{
  try
  {
    YAML::Node root = YAML::LoadFile(path);
    const std::string root_path = rootPath(path);
    return parse(root, root_path);
  }
  catch (const YAML::Exception& ex)
  {
    CONSOLE_BRIDGE_logError("Error while parsing YAML file: %s", ex.what());
    return {};
  }
}

template<typename T>
void write(const std::string& path, const T& pose)
{
  YAML::Node root(pose);
  std::ofstream ofh (path);
  ofh << root;
}

void writeDirectory(const std::string& path, const rct_ros_tools::ExtrinsicDataSet& data)
{
  YAML::Node root;

  for (std::size_t i = 0; i < data.images.size(); ++i)
  {
    YAML::Node n;
    n["pose"] = "poses/" + std::to_string(i) + ".yaml";
    n["image"] = "images/" + std::to_string(i) + ".png";
    n["camera_joints"] = "camera_joints/" + std::to_string(i) + ".yaml";
    n["target_joints"] = "target_joints/" + std::to_string(i) + ".yaml";
    root.push_back(n);
  }

  std::ofstream ofh (path);
  ofh << root;
}

bool rct_ros_tools::saveToDirectory(const std::string& path, const rct_ros_tools::ExtrinsicDataSet& data)
{
  {
    auto boost_path = boost::filesystem::path(path);
    boost::filesystem::create_directories(boost_path);
    boost::filesystem::create_directories(boost_path / "images");
    boost::filesystem::create_directories(boost_path / "poses");
  }

  for (std::size_t i = 0; i < data.images.size(); ++i)
  {
    auto name = path + "/images/" + std::to_string(i) + ".png";
    cv::imwrite(name, data.images[i]);
  }

  for (std::size_t i = 0; i < data.tool_poses.size(); ++i)
  {
    auto name = path + "/poses/" + std::to_string(i) + ".yaml";
    write(name, data.tool_poses[i]);
  }

  for (std::size_t i = 0; i < data.camera_chain_joints.size(); ++i)
  {
    auto name = path + "/camera_joints/" + std::to_string(i) + ".yaml";
    write(name, data.camera_chain_joints[i]);
  }

  for (std::size_t i = 0; i < data.target_chain_joints.size(); ++i)
  {
    auto name = path + "/target_joints/" + std::to_string(i) + ".yaml";
    write(name, data.target_chain_joints[i]);
  }

  writeDirectory(path + "/data.yaml", data);

  return true;
}

rct_ros_tools::ExtrinsicCorrespondenceDataSet::ExtrinsicCorrespondenceDataSet(const std::vector<rct_ros_tools::ExtrinsicDataSet> &extrinsic_data_set,
                                                                              const rct_image_tools::TargetFinder &target_finder,
                                                                              bool debug)
{
  static const std::string WINDOW = "window";
  if (debug)
    cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);

  correspondences_.resize(extrinsic_data_set.size(), extrinsic_data_set[0].images.size());
  mask_.resize(extrinsic_data_set.size(), extrinsic_data_set[0].images.size());
  for (std::size_t c = 0; c < extrinsic_data_set.size(); ++c)
  {
    // We know it exists, so define a helpful alias
    const rct_ros_tools::ExtrinsicDataSet& data_set = extrinsic_data_set[c];

    // Finally, we need to process our images into correspondence sets: for each dot in the
    // target this will be where that dot is in the target and where it was seen in the image.
    // Repeat for each image. We also tell where the wrist was when the image was taken.
    for (std::size_t i = 0; i < data_set.images.size(); ++i)
    {
      // Try to find the circle grid in this image:
      try
      {
        mask_(c, i) = 1;

        rct_image_tools::TargetFeatures target_features = target_finder.findTargetFeatures(data_set.images[i]);
        if (target_features.empty())
          throw std::runtime_error("Failed to find any target features in image " + std::to_string(i));
        CONSOLE_BRIDGE_logInform("Found %i target features", target_features.size());

        correspondences_(c, i) = target_finder.target().createCorrespondences(target_features);

        if (debug)
        {
          // Show the points we detected
          std::vector<Eigen::Vector2d> observations(correspondences_(c, i).size());
          std::transform(correspondences_(c, i).begin(), correspondences_(c, i).end(), observations.begin(),
                         [](const rct_optimizations::Correspondence2D3D& o) { return o.in_image; });

          cv::imshow(WINDOW, target_finder.drawTargetFeatures(data_set.images[i], target_features));
          cv::waitKey();
        }
      }
      catch (const std::exception& ex)
      {
        mask_(c, i) = 0;
        CONSOLE_BRIDGE_logError(ex.what());
      }
    }
  }

  if (debug)
    cv::destroyWindow(WINDOW);
}

std::size_t rct_ros_tools::ExtrinsicCorrespondenceDataSet::getCameraCount() const
{
  return correspondences_.rows();
}

std::size_t rct_ros_tools::ExtrinsicCorrespondenceDataSet::getImageCount() const
{
  return correspondences_.cols();
}

std::size_t rct_ros_tools::ExtrinsicCorrespondenceDataSet::getImageCameraCount(std::size_t image_index) const
{
  return mask_.col(image_index).sum();
}

std::size_t rct_ros_tools::ExtrinsicCorrespondenceDataSet::getCameraImageCount(std::size_t camera_index) const
{
  return mask_.row(camera_index).sum();
}

bool rct_ros_tools::ExtrinsicCorrespondenceDataSet::foundCorrespondence(std::size_t camera_index, std::size_t image_index) const
{
  return static_cast<bool>(mask_(camera_index, image_index));
}

const rct_optimizations::Correspondence2D3D::Set&
rct_ros_tools::ExtrinsicCorrespondenceDataSet::getCorrespondenceSet(std::size_t camera_index, std::size_t image_index) const
{
  return correspondences_(camera_index, image_index);
}
