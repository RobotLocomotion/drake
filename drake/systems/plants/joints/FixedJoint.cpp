#include "drake/systems/plants/joints/FixedJoint.h"

std::string FixedJoint::get_position_name(int index) const {
  throw std::runtime_error("bad index");
}

Eigen::VectorXd FixedJoint::zeroConfiguration() const {
  return Eigen::VectorXd::Zero(0);
}

Eigen::VectorXd FixedJoint::randomConfiguration(
    std::default_random_engine& generator) const {
  return Eigen::VectorXd::Zero(0);
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
std::string FixedJoint::getPositionName(int index) const {
  return get_position_name(index);
}
