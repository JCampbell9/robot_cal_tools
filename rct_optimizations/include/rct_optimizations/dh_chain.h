#pragma once

#include <Eigen/Geometry>
#include <memory>
#include <utility>
#include <vector>

namespace rct_optimizations
{
enum class DHJointType : unsigned
{
  LINEAR,
  REVOLUTE,
  FIXED
};

template<typename T>
using Isometry3 = Eigen::Transform<T, 3, Eigen::Isometry>;

template<typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template<typename T>
using Vector2 = Eigen::Matrix<T, 2, 1>;

/**
 * @brief Struct representing the DH parameters of a single transformation between adjacent links.
 * This struct follows the classical DH parameter convention: Trans[Zi-1](d) * Rot[Zi-1](theta) * Trans[Xi](r) * Rot[Xi](alpha)
 * See @link https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters for reference
 */
struct DHTransform
{
  using Ptr = std::unique_ptr<DHTransform>;

  DHTransform(std::array<double, 4> params_, DHJointType type_);
  virtual ~DHTransform() = default;

  /**
   *
   */
  template<typename T>
  Isometry3<T> createRelativeTransform(const T joint_value,
                                              const T* offsets) const;

  /**
   * @brief Creates the homogoneous transformation from the previous link to the current link
   * @param joint_value
   * @return
   */
  template<typename T>
  Isometry3<T> createRelativeTransform(const T joint_value) const;

  double createRandomJointValue() const;

  /** @brief DH parameters
   *  d: The linear offset in Z
   *  theta: The rotational offset about Z
   *  r: The linear offset in X
   *  alpha: The rotational offset about X
   */
  std::array<double, 4> params;
  DHJointType type; /** @brief The type of actuation of the joint */
  double max = M_PI; /** @brief Joint max */
  double min = -M_PI; /** @brief Joint min */
};

/**
 * @brief Robot representation using DH parameters
 */
class DHChain
{
public:
    DHChain(std::vector<DHTransform::Ptr> transforms);

  /**
   * @brief Calculates forward kinematics for the robot with the joints provided.
   * Note: the transform to the n-th link is calculated, where n is the size of @ref joint_values
   * @param joint_values - The joint values with which to calculate forward kinematics.
   * @return
   * @throws Exception if the size of joint values is larger than the number of DH transforms in the robot
   */
  template<typename T>
  Isometry3<T> getFK(const Eigen::Matrix<T, Eigen::Dynamic, 1> &joint_values) const;

  /**
   * @brief Override function of @ref getFK but using a data pointer for easier integration with Ceres
   * @param joint_values
   * @param offsets
   * @return
   */
  template<typename T>
  Isometry3<T> getFK(const Eigen::Matrix<T, Eigen::Dynamic, 1>& joint_values,
                            const T* const* offsets) const;

  /**
   * @brief Creates a random pose by choosing a random uniformly distributed joint value for each joint in the chain
   * @return
   */
  Eigen::Isometry3d createUniformlyRandomPose() const;

  inline std::size_t dof() const
  {
    return transforms_.size();
  }

protected:
  std::vector<DHTransform::Ptr> transforms_;
};

} // namespace rct_optimizations
