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
      return name + "_qw";
    case 1:
      return name + "_qx";
    case 2:
      return name + "_qy";
    case 3:
      return name + "_qz";
    default:
      throw std::runtime_error("bad index");
  }
}

std::string QuaternionBallJoint::get_velocity_name(int index) const {
  switch (index) {
    case 0:
      return name + "_wx";
    case 1:
      return name + "_wy";
    case 2:
      return name + "_wz";
    default:
      throw std::runtime_error("bad index");
  }
}

VectorXd QuaternionBallJoint::zeroConfiguration() const {
  return Vector4d(1, 0, 0, 0);
}

VectorXd QuaternionBallJoint::randomConfiguration(
    std::default_random_engine& generator) const {
  return drake::math::UniformlyRandomQuat(generator);
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
std::string QuaternionBallJoint::getPositionName(int index) const {
  return get_position_name(index);
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
std::string QuaternionBallJoint::getVelocityName(int index) const {
  return get_velocity_name(index);
}

std::unique_ptr<DrakeJoint> QuaternionBallJoint::DoClone() const {
  auto joint = std::make_unique<QuaternionBallJoint>(
      get_name(), get_transform_to_parent_body());
  return std::move(joint);
}
