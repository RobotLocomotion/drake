#include "HelicalJoint.h"

using namespace Eigen;

HelicalJoint::HelicalJoint(const std::string& name, const RigidBody& parent_body, const AffineCompact3d& transform_to_parent_body, const Vector3d& axis, double pitch) :
    FixedAxisOneDoFJoint(name, parent_body, transform_to_parent_body, spatialJointAxis(axis, pitch)), axis(axis), pitch(pitch)
{
}

HelicalJoint::~HelicalJoint()
{
  // empty
}

AffineCompact3d HelicalJoint::jointTransform(double* const q) const
{
  return AngleAxisd(q[0], axis) * Translation3d(q[0] * pitch * axis);
}

Matrix<double, TWIST_SIZE, 1> HelicalJoint::spatialJointAxis(const Vector3d& axis, double pitch)
{
  Matrix<double, TWIST_SIZE, 1> ret;
  ret.topRows<3>() = axis;
  ret.bottomRows<3>() = pitch * axis;
  return ret;
}
