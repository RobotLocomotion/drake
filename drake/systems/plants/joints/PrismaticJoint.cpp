#include "drake/systems/plants/joints/PrismaticJoint.h"

using Eigen::Vector3d;

drake::TwistVector<double> PrismaticJoint::spatialJointAxis(
    const Vector3d& translation_axis) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = Vector3d::Zero();
  ret.bottomRows<3>() = translation_axis;
  return ret;
}
