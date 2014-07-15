#include "PrismaticJoint.h"
#include <Eigen/Geometry>

PrismaticJoint::PrismaticJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body, const e::Vector3d& translation_axis) :
  OneDoFJoint(parent_body, transform_to_parent_body, spatialJointAxis(translation_axis)), translation_axis(translation_axis)
{
}

PrismaticJoint::~PrismaticJoint()
{
  // empty
}

e::AffineCompact3d PrismaticJoint::jointTransform(double* const q) const
{
  e::AffineCompact3d ret;
  ret.linear().setIdentity();
  ret.translation() = q[0] * translation_axis;
  return ret;
}

e::Matrix<double, TWIST_SIZE, 1> PrismaticJoint::spatialJointAxis(e::Vector3d translation_axis)
{
  Matrix<double, TWIST_SIZE, 1> ret;
  ret.topRows<3>() = translation_axis;
  ret.bottomRows<3>() = Vector3d::Zero();
  return ret;
}
