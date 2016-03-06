#include "drake/systems/plants/joints/FixedJoint.h"

std::string FixedJoint::getPositionName(int index) const { 
  throw std::runtime_error("bad index"); 
}

Eigen::VectorXd FixedJoint::zeroConfiguration() const {
  return Eigen::VectorXd::Zero(0);
}

Eigen::VectorXd FixedJoint::randomConfiguration(std::default_random_engine& generator) const {
  return Eigen::VectorXd::Zero(0);
}

