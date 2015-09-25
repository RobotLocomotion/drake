#include "RevoluteJoint.h"

using namespace Eigen;

Matrix<double, TWIST_SIZE, 1> RevoluteJoint::spatialJointAxis(const Vector3d& rotation_axis)
{
  Matrix<double, TWIST_SIZE, 1> ret;
  ret.topRows<3>() = rotation_axis;
  ret.bottomRows<3>() = Vector3d::Zero();
  return ret;
}

