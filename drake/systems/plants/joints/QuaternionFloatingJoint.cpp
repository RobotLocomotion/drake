#include "drake/systems/plants/joints/QuaternionFloatingJoint.h"

#include <random>

#include "drake/math/random_rotation.h"

using Eigen::Vector4d;
using Eigen::VectorXd;
using std::normal_distribution;

std::string QuaternionFloatingJoint::get_position_name(int index) const {
  switch (index) {
    case 0:
      return name + "_x";
    case 1:
      return name + "_y";
    case 2:
      return name + "_z";
    case 3:
      return name + "_qw";
    case 4:
      return name + "_qx";
    case 5:
      return name + "_qy";
    case 6:
      return name + "_qz";
    default:
      throw std::runtime_error("bad index");
  }
}

std::string QuaternionFloatingJoint::get_velocity_name(int index) const {
  switch (index) {
    case 0:
      return name + "_wx";
    case 1:
      return name + "_wy";
    case 2:
      return name + "_wz";
    case 3:
      return name + "_vx";
    case 4:
      return name + "_vy";
    case 5:
      return name + "_vz";
    default:
      throw std::runtime_error("bad index");
  }
}

VectorXd QuaternionFloatingJoint::zeroConfiguration() const {
  Eigen::VectorXd ret(get_num_positions());
  ret << 0, 0, 0, 1, 0, 0, 0;
  return ret;
}

VectorXd QuaternionFloatingJoint::randomConfiguration(
    std::default_random_engine& generator) const {
  VectorXd q(7);
  normal_distribution<double> normal;

  // position
  q[0] = normal(generator);
  q[1] = normal(generator);
  q[2] = normal(generator);

  // orientation
  Vector4d quat = drake::math::UniformlyRandomQuat(generator);
  q[3] = quat(0);
  q[4] = quat(1);
  q[5] = quat(2);
  q[6] = quat(3);
  return q;
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
std::string QuaternionFloatingJoint::getPositionName(int index) const {
  return get_position_name(index);
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
std::string QuaternionFloatingJoint::getVelocityName(int index) const {
  return get_velocity_name(index);
}
