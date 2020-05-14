#include "drake/multibody/joints/quaternion_ball_joint.h"

#include <memory>
#include <random>
#include <utility>

#include "drake/math/random_rotation.h"

using Eigen::Vector4d;
using Eigen::VectorXd;

std::string QuaternionBallJoint::get_position_name(int index) const {
  switch (index) {
    case 0:
      return name_ + "_qw";
    case 1:
      return name_ + "_qx";
    case 2:
      return name_ + "_qy";
    case 3:
      return name_ + "_qz";
    default:
      throw std::runtime_error("bad index");
  }
}

std::string QuaternionBallJoint::get_velocity_name(int index) const {
  switch (index) {
    case 0:
      return name_ + "_wx";
    case 1:
      return name_ + "_wy";
    case 2:
      return name_ + "_wz";
    default:
      throw std::runtime_error("bad index");
  }
}

VectorXd QuaternionBallJoint::zeroConfiguration() const {
  // TODO(mitiguy) Consider updating this method to return Eigen::Quaterniond.
  return Vector4d(1, 0, 0, 0);
}

VectorXd QuaternionBallJoint::randomConfiguration(
    std::default_random_engine& generator) const {
  // TODO(mitiguy) Consider updating this method to return Eigen::Quaterniond.
  Eigen::Quaterniond q = drake::math::UniformlyRandomQuaternion(&generator);
  return Vector4d(q.w(), q.x(), q.y(), q.z());
}

std::unique_ptr<DrakeJoint> QuaternionBallJoint::DoClone() const {
  auto joint = std::make_unique<QuaternionBallJoint>(
      get_name(), get_transform_to_parent_body());
  return joint;
}
