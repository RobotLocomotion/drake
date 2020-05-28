#include "drake/multibody/joints/prismatic_joint.h"

#include <memory>
#include <utility>

using Eigen::Vector3d;

drake::TwistVector<double> PrismaticJoint::spatialJointAxis(
    const Vector3d& translation_axis) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = Vector3d::Zero();
  ret.bottomRows<3>() = translation_axis;
  return ret;
}

std::unique_ptr<DrakeJoint> PrismaticJoint::DoClone() const {
  auto joint = std::make_unique<PrismaticJoint>(get_name(),
                                                get_transform_to_parent_body(),
                                                translation_axis_);
  return joint;
}
