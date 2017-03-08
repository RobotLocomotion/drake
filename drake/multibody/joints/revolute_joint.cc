#include "drake/multibody/joints/revolute_joint.h"

#include <memory>
#include <utility>

using Eigen::Vector3d;

std::unique_ptr<DrakeJoint> RevoluteJoint::Clone() const {
  auto joint = std::make_unique<RevoluteJoint>(get_name(),
                                               get_transform_to_parent_body(),
                                               rotation_axis);
  return std::move(joint);
}

drake::TwistVector<double> RevoluteJoint::spatialJointAxis(
    const Vector3d& rotation_axis) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = rotation_axis;
  ret.bottomRows<3>() = Vector3d::Zero();
  return ret;
}

bool RevoluteJoint::CompareToClone(const DrakeJoint& other) const {
  if (!FixedAxisOneDoFJoint::CompareToClone(other)) return false;
  const RevoluteJoint* downcasted_joint =
      DrakeJoint::DowncastOrLog<RevoluteJoint>(&other);
  if (downcasted_joint == nullptr) {
    return false;
  }
  if (rotation_axis != downcasted_joint->rotation_axis) {
    drake::log()->debug(
        "RevoluteJoint::CompareToClone(): rotation_axis mismatch:\n"
        "  - this: {}\n"
        "  - other: {}",
        rotation_axis.transpose(),
        downcasted_joint->rotation_axis.transpose());
    return false;
  }
  return true;
}
