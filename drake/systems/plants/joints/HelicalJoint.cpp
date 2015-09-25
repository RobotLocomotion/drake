#include "HelicalJoint.h"

using namespace Eigen;

Matrix<double, TWIST_SIZE, 1> HelicalJoint::spatialJointAxis(const Vector3d& axis, double pitch)
{
  Matrix<double, TWIST_SIZE, 1> ret;
  ret.topRows<3>() = axis;
  ret.bottomRows<3>() = pitch * axis;
  return ret;
}
