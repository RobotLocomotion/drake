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

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
std::string FixedJoint::getPositionName(int index) const {
  return get_position_name(index);
}

std::unique_ptr<DrakeJoint> FixedJoint::DoClone() const {
  auto joint = std::make_unique<FixedJoint>(get_name(),
                                            get_transform_to_parent_body());
  return std::move(joint);
}
