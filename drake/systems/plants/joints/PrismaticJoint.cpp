#include "PrismaticJoint.h"
#include <Eigen/Geometry>

using namespace Eigen;

Matrix<double, TWIST_SIZE, 1> PrismaticJoint::spatialJointAxis(const Vector3d& translation_axis)
{
  Matrix<double, TWIST_SIZE, 1> ret;
  ret.topRows<3>() = Vector3d::Zero();
  ret.bottomRows<3>() = translation_axis;
  return ret;
}

