#include "HelicalJoint.h"

HelicalJoint::HelicalJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body, const e::Vector3d& axis, double pitch) :
    FixedAxisOneDoFJoint(parent_body, transform_to_parent_body, spatialJointAxis(axis, pitch)), axis(axis), pitch(pitch)
{
}

HelicalJoint::~HelicalJoint()
{
  // empty
}

e::AffineCompact3d HelicalJoint::jointTransform(double* const q) const
{
  return e::AngleAxisd(q[0], axis) * e::Translation3d(q[0] * pitch * axis);
}

e::Matrix<double, TWIST_SIZE, 1> HelicalJoint::spatialJointAxis(const e::Vector3d& axis, double pitch)
{
  e::Matrix<double, TWIST_SIZE, 1> ret;
  ret.topRows<3>() = axis;
  ret.bottomRows<3>() = pitch * axis;
  return ret;
}
