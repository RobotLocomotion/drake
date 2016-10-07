#include "drake/systems/plants/joints/RollPitchYawFloatingJoint.h"

#include <random>

#include <Eigen/Dense>

#include "drake/math/random_rotation.h"

using Eigen::Map;
using Eigen::Vector3d;
using Eigen::VectorXd;

std::string RollPitchYawFloatingJoint::get_position_name(int index) const {
  switch (index) {
    case 0:
      return name + "_x";
    case 1:
      return name + "_y";
    case 2:
      return name + "_z";
    case 3:
      return name + "_roll";
    case 4:
      return name + "_pitch";
    case 5:
      return name + "_yaw";
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
  rpy = drake::math::UniformlyRandomRPY(generator);
  return q;
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
std::string RollPitchYawFloatingJoint::getPositionName(int index)
    const {
  return get_position_name(index);
}
