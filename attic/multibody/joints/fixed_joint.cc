#include "drake/multibody/joints/fixed_joint.h"

#include <memory>
#include <utility>

std::string FixedJoint::get_position_name(int) const {
  throw std::runtime_error("bad index");
}

Eigen::VectorXd FixedJoint::zeroConfiguration() const {
  return Eigen::VectorXd::Zero(0);
}

Eigen::VectorXd FixedJoint::randomConfiguration(
    std::default_random_engine&) const {
  return Eigen::VectorXd::Zero(0);
}

std::unique_ptr<DrakeJoint> FixedJoint::DoClone() const {
  auto joint = std::make_unique<FixedJoint>(get_name(),
                                            get_transform_to_parent_body());
  return joint;
}
