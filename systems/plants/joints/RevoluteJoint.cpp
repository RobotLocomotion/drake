#include "RevoluteJoint.h"
#include <Eigen/Geometry>

RevoluteJoint::RevoluteJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body, const e::Vector3d& rotation_axis) :
    FixedAxisOneDoFJoint(parent_body, transform_to_parent_body, spatialJointAxis(rotation_axis)), rotation_axis(rotation_axis)
{
}

RevoluteJoint::~RevoluteJoint()
{
  // empty
}

e::AffineCompact3d RevoluteJoint::jointTransform(double* const q) const
{
  return e::AffineCompact3d(e::AngleAxisd(q[0], rotation_axis));
}

e::Matrix<double, TWIST_SIZE, 1> RevoluteJoint::spatialJointAxis(const e::Vector3d& rotation_axis)
{
  Matrix<double, TWIST_SIZE, 1> ret;
  ret.topRows<3>() = rotation_axis;
  ret.bottomRows<3>() = Vector3d::Zero();
  return ret;
}
