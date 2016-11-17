#include "drake/multibody/joints/helical_joint.h"

using Eigen::Vector3d;

drake::TwistVector<double> HelicalJoint::spatialJointAxis(const Vector3d& axis,
                                                          double pitch) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = axis;
  ret.bottomRows<3>() = pitch * axis;
  return ret;
}
