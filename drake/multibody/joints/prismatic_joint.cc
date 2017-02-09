#include "drake/multibody/joints/prismatic_joint.h"

#include <memory>
#include <utility>

using Eigen::Vector3d;

std::unique_ptr<DrakeJoint> PrismaticJoint::Clone() const {
  auto joint = std::make_unique<PrismaticJoint>(get_name(),
                                                get_transform_to_parent_body(),
                                                translation_axis_);
  FixedAxisOneDoFJoint::InitializeClone(joint.get());
  return std::move(joint);
}

drake::TwistVector<double> PrismaticJoint::spatialJointAxis(
    const Vector3d& translation_axis) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = Vector3d::Zero();
  ret.bottomRows<3>() = translation_axis;
  return ret;
}
