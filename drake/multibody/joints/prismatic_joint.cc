#include "drake/multibody/joints/prismatic_joint.h"

#include <memory>
#include <utility>

using Eigen::Vector3d;

std::unique_ptr<DrakeJoint> PrismaticJoint::Clone() const {
  auto joint = std::make_unique<PrismaticJoint>(get_name(),
                                                get_transform_to_parent_body(),
                                                translation_axis_);
  return std::move(joint);
}

drake::TwistVector<double> PrismaticJoint::spatialJointAxis(
    const Vector3d& translation_axis) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = Vector3d::Zero();
  ret.bottomRows<3>() = translation_axis;
  return ret;
}

bool PrismaticJoint::CompareToClonedJoint(const DrakeJoint& other) const {
  if (!FixedAxisOneDoFJoint::CompareToClonedJoint(other)) return false;
  const PrismaticJoint* downcasted_joint =
      dynamic_cast<const PrismaticJoint*>(&other);
  if (downcasted_joint == nullptr) {
    drake::log()->debug(
      "PrismaticJoint::CompareToClonedJoint(): other is not a PrismaticJoint.");
    return false;
  }
  if (translation_axis_ != downcasted_joint->translation_axis_) {
    drake::log()->debug(
        "PrismaticJoint::CompareToClonedJoint(): translation_axis mismatch:\n"
        "  - this: {}\n"
        "  - other: {}",
        translation_axis_.transpose(),
        downcasted_joint->translation_axis_.transpose());
    return false;
  }
  return true;
}

