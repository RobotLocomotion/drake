#include "drake/multibody/joints/revolute_joint.h"

#include <memory>
#include <utility>

using Eigen::Vector3d;

std::unique_ptr<DrakeJoint> RevoluteJoint::Clone() const {
  auto joint = std::make_unique<RevoluteJoint>(get_name(),
                                               get_transform_to_parent_body(),
                                               rotation_axis_);
  InitializeClone(joint.get());
  return std::move(joint);
}

drake::TwistVector<double> RevoluteJoint::spatialJointAxis(
    const Vector3d& rotation_axis) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = rotation_axis;
  ret.bottomRows<3>() = Vector3d::Zero();
  return ret;
}
