#include "HelicalJoint.h"

using namespace Eigen;

drake::TwistVector<double> HelicalJoint::spatialJointAxis(const Vector3d& axis,
                                                          double pitch) {
  drake::TwistVector<double> ret;
  ret.topRows<3>() = axis;
  ret.bottomRows<3>() = pitch * axis;
  return ret;
}
