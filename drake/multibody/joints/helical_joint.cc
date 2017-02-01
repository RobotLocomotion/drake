#include "drake/multibody/joints/helical_joint.h"

#include <memory>
#include <utility>

#include "drake/common/text_logging.h"

using Eigen::Vector3d;

std::unique_ptr<DrakeJoint> HelicalJoint::Clone() const {
  auto joint = std::make_unique<HelicalJoint>(get_name(),
                                              get_transform_to_parent_body(),
                                              axis_, pitch_);
  return std::move(joint);
}

drake::TwistVector<double> HelicalJoint::spatialJointAxis(const Vector3d& axis,
                                                          double pitch) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = axis;
  ret.bottomRows<3>() = pitch * axis;
  return ret;
}

bool HelicalJoint::CompareToClonedJoint(const DrakeJoint& other) const {
  if (!FixedAxisOneDoFJoint::CompareToClonedJoint(other)) return false;
  const HelicalJoint* downcasted_joint =
      dynamic_cast<const HelicalJoint*>(&other);
  if (downcasted_joint == nullptr) {
    drake::log()->debug(
      "HelicalJoint::CompareToClonedJoint(): other is not a HelicalJoint.");
    return false;
  }
  if (axis_ != downcasted_joint->axis_) {
    drake::log()->debug(
        "HelicalJoint::CompareToClonedJoint(): axis mismatch:\n"
        "  - this: {}\n"
        "  - other: {}",
        axis_,
        downcasted_joint->axis_);
    return false;
  }
  if (pitch_ != downcasted_joint->pitch_) {
      drake::log()->debug(
        "HelicalJoint::CompareToClonedJoint(): pitch mismatch:\n"
        "  - this: {}\n"
        "  - other: {}",
        pitch_,
        downcasted_joint->pitch_);
    return false;
  }
  return true;
}
