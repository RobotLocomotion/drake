#include "HelicalJoint.h"

using namespace Eigen;

HelicalJoint::HelicalJoint(const std::string& name, const Isometry3d& transform_to_parent_body, const Vector3d& axis, double pitch) :
    FixedAxisOneDoFJoint(name, transform_to_parent_body, spatialJointAxis(axis, pitch)), axis(axis), pitch(pitch)
{
}

HelicalJoint::~HelicalJoint()
{
  // empty
}

Isometry3d HelicalJoint::jointTransform(double* const q) const
{
  Isometry3d ret;
  ret = AngleAxisd(q[0], axis) * Translation3d(q[0] * pitch * axis);
  ret.makeAffine();
  return ret;
}

Matrix<double, TWIST_SIZE, 1> HelicalJoint::spatialJointAxis(const Vector3d& axis, double pitch)
{
  Matrix<double, TWIST_SIZE, 1> ret;
  ret.topRows<3>() = axis;
  ret.bottomRows<3>() = pitch * axis;
  return ret;
}
