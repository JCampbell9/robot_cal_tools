#pragma once

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace YAML
{
template<typename FloatT>
struct convert<Eigen::Matrix<FloatT, 2, 1>>
{
  using T = Eigen::Matrix<FloatT, 2, 1>;

  static Node encode(const T &val)
  {
    YAML::Node node;
    node["x"] = val.x();
    node["y"] = val.y();
    return node;
  }

  static bool decode(const YAML::Node &node, T &val)
  {
    if (node.size() != 2)
      return false;

    val.x() = node["x"].as<FloatT>();
    val.y() = node["y"].as<FloatT>();

    return true;
  }
};

template<typename FloatT>
struct convert<Eigen::Matrix<FloatT, 3, 1>>
{
  using T = Eigen::Matrix<FloatT, 3, 1>;

  static Node encode(const T &val)
  {
    YAML::Node node;
    node["x"] = val.x();
    node["y"] = val.y();
    node["z"] = val.z();
    return node;
  }

  static bool decode(const YAML::Node &node, T &val)
  {
    if (node.size() != 3)
      return false;

    val.x() = node["x"].as<FloatT>();
    val.y() = node["y"].as<FloatT>();
    val.z() = node["z"].as<FloatT>();

    return true;
  }
};

template <typename FloatT>
struct convert<Eigen::Matrix<FloatT, Eigen::Dynamic, 1>>
{
  using T = Eigen::Matrix<FloatT, Eigen::Dynamic, 1>;

  static Node encode(const T& val)
  {
    YAML::Node node;
    for (Eigen::Index i = 0; i < val.size(); ++i)
    {
      node.push_back(val(i));
    }
    return node;
  }

  static bool decode(const YAML::Node& node, T& val)
  {
    val.resize(node.size());
    for (std::size_t i = 0; i < node.size(); ++i)
    {
      val(i) = node[i].as<FloatT>();
    }

    return true;
  }
};

template<typename FloatT>
struct convert<Eigen::Transform<FloatT, 3, Eigen::Isometry>>
{
  using T = Eigen::Transform<FloatT, 3, Eigen::Isometry>;

  static Node encode(const T &val)
  {
    YAML::Node node;
    node["x"] = val.translation().x();
    node["y"] = val.translation().y();
    node["z"] = val.translation().z();

    Eigen::Quaternion<FloatT> quat(val.linear());
    node["qw"] = quat.w();
    node["qx"] = quat.x();
    node["qy"] = quat.y();
    node["qz"] = quat.z();

    return node;
  }

  static bool decode(const YAML::Node &node, T &val)
  {
    if (node.size() != 7)
      return false;

    Eigen::Matrix<FloatT, 3, 1> trans;
    trans.x() = node["x"].as<FloatT>();
    trans.y() = node["y"].as<FloatT>();
    trans.z() = node["z"].as<FloatT>();

    Eigen::Quaternion<FloatT> quat;
    quat.w() = node["qw"].as<FloatT>();
    quat.x() = node["qx"].as<FloatT>();
    quat.y() = node["qy"].as<FloatT>();
    quat.z() = node["qz"].as<FloatT>();

    val = Eigen::Translation<FloatT, 3>(trans) * quat;

    return true;
  }
};

template <typename FloatT>
struct convert<Eigen::Matrix<FloatT, Eigen::Dynamic, 4>>
{
  using T = Eigen::Matrix<FloatT, Eigen::Dynamic, 4>;

  static Node encode(const T& rhs)
  {
    Node node;
    for (Eigen::Index row = 0; row < rhs.rows(); ++row)
    {
      Eigen::Matrix<FloatT, 4, 1> v = rhs.block(row, 0, 1, 4);
      node.push_back(std::vector<double>(v.data(), v.data() + v.size()));
    }
    return node;
  }

  static bool decode(const Node& node, T& lhs)
  {
    lhs.resize(node.size(), 4);
    for (std::size_t row = 0; row < node.size(); ++row)
    {
      const auto v = node[row].as<std::vector<double>>();
      lhs.row(row) = Eigen::Map<const Eigen::Vector4d>(v.data());
    }

    return true;
  }
};

} // namespace YAML
