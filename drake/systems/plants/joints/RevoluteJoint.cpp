#include "drake/systems/plants/joints/RevoluteJoint.h"

using Eigen::Vector3d;

drake::TwistVector<double> RevoluteJoint::spatialJointAxis(
    const Vector3d& rotation_axis) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = rotation_axis;
  ret.bottomRows<3>() = Vector3d::Zero();
  return ret;
}
