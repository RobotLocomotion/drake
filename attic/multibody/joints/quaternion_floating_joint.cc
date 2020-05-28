#include "drake/multibody/joints/quaternion_floating_joint.h"

#include <memory>
#include <random>
#include <utility>

#include "drake/math/random_rotation.h"

using Eigen::Vector4d;
using Eigen::VectorXd;
using std::normal_distribution;

std::string QuaternionFloatingJoint::get_position_name(int index) const {
  switch (index) {
    case 0:
      return name_ + "_x";
    case 1:
      return name_ + "_y";
    case 2:
      return name_ + "_z";
    case 3:
      return name_ + "_qw";
    case 4:
      return name_ + "_qx";
    case 5:
      return name_ + "_qy";
    case 6:
      return name_ + "_qz";
    default:
      throw std::runtime_error("bad index");
  }
}

std::string QuaternionFloatingJoint::get_velocity_name(int index) const {
  switch (index) {
    case 0:
      return name_ + "_wx";
    case 1:
      return name_ + "_wy";
    case 2:
      return name_ + "_wz";
    case 3:
      return name_ + "_vx";
    case 4:
      return name_ + "_vy";
    case 5:
      return name_ + "_vz";
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
  Eigen::Quaterniond quat = drake::math::UniformlyRandomQuaternion(&generator);
  q[3] = quat.w();
  q[4] = quat.x();
  q[5] = quat.y();
  q[6] = quat.z();
  return q;
}

std::unique_ptr<DrakeJoint> QuaternionFloatingJoint::DoClone() const {
  auto joint = std::make_unique<QuaternionFloatingJoint>(get_name(),
      get_transform_to_parent_body());
  return joint;
}

