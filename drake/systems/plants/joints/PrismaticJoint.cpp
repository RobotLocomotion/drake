#include "PrismaticJoint.h"
#include <Eigen/Geometry>

#include "RigidBodyManipulator.h" // todo: remove this when I remove setupOldKinematicTree

using namespace Eigen;

PrismaticJoint::PrismaticJoint(const std::string& name, const Isometry3d& transform_to_parent_body, const Vector3d& translation_axis) :
    FixedAxisOneDoFJoint(name, transform_to_parent_body, spatialJointAxis(translation_axis)), translation_axis(translation_axis)
{
  assert(std::abs(translation_axis.norm() - 1.0) < 1e-10);
}

PrismaticJoint::~PrismaticJoint()
{
  // empty
}

Isometry3d PrismaticJoint::jointTransform(const Eigen::Ref<const VectorXd>& q) const
{
  Isometry3d ret;
  ret.linear().setIdentity();
  ret.translation() = q[0] * translation_axis;
  ret.makeAffine();
  return ret;
}

Matrix<double, TWIST_SIZE, 1> PrismaticJoint::spatialJointAxis(const Vector3d& translation_axis)
{
  Matrix<double, TWIST_SIZE, 1> ret;
  ret.topRows<3>() = Vector3d::Zero();
  ret.bottomRows<3>() = translation_axis;
  return ret;
}

