#include "FixedJoint.h"

std::string FixedJoint::getPositionName(int index) const { 
  throw std::runtime_error("bad index"); 
}

Eigen::VectorXd FixedJoint::randomConfiguration(std::default_random_engine& generator) const {
  return VectorXd::Zero(0);
}

