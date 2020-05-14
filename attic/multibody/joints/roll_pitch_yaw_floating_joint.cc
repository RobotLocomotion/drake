#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"

#include <memory>
#include <random>
#include <utility>

#include <Eigen/Dense>

#include "drake/math/random_rotation.h"

using Eigen::Map;
using Eigen::Vector3d;
using Eigen::VectorXd;

std::string RollPitchYawFloatingJoint::get_position_name(int index) const {
  switch (index) {
    case 0:
      return name_ + "_x";
    case 1:
      return name_ + "_y";
    case 2:
      return name_ + "_z";
    case 3:
      return name_ + "_roll";
    case 4:
      return name_ + "_pitch";
    case 5:
      return name_ + "_yaw";
    default:
      throw std::runtime_error("bad index");
  }
}

VectorXd RollPitchYawFloatingJoint::zeroConfiguration() const {
  return VectorXd::Zero(get_num_positions());
}

VectorXd RollPitchYawFloatingJoint::randomConfiguration(
    std::default_random_engine& generator) const {
  VectorXd q(6);
  std::normal_distribution<double> normal;

  Map<Vector3d> pos(&q[0]);
  for (int i = 0; i < drake::kSpaceDimension; i++) {
    pos(i) = normal(generator);
  }

  Map<Vector3d> rpy(&q[3]);
  rpy = drake::math::UniformlyRandomRPY(&generator);
  return q;
}

std::unique_ptr<DrakeJoint> RollPitchYawFloatingJoint::DoClone() const {
  auto joint = std::make_unique<RollPitchYawFloatingJoint>(
      get_name(),
      get_transform_to_parent_body());
  return joint;
}
