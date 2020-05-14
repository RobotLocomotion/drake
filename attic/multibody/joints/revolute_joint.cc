#include "drake/multibody/joints/revolute_joint.h"

#include <memory>
#include <utility>

using Eigen::Vector3d;

drake::TwistVector<double> RevoluteJoint::spatialJointAxis(
    const Vector3d& rotation_axis) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = rotation_axis;
  ret.bottomRows<3>() = Vector3d::Zero();
  return ret;
}

std::unique_ptr<DrakeJoint> RevoluteJoint::DoClone() const {
  auto joint = std::make_unique<RevoluteJoint>(get_name(),
                                               get_transform_to_parent_body(),
                                               rotation_axis_);
  return joint;
}
