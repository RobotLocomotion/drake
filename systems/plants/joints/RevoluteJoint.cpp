#include "RevoluteJoint.h"
#include <Eigen/Geometry>

using namespace Eigen;

RevoluteJoint::RevoluteJoint(const std::string& name, const Isometry3d& transform_to_parent_body, const Vector3d& rotation_axis) :
    FixedAxisOneDoFJoint(name, transform_to_parent_body, spatialJointAxis(rotation_axis)), rotation_axis(rotation_axis)
{
}

RevoluteJoint::~RevoluteJoint()
{
  // empty
}

Isometry3d RevoluteJoint::jointTransform(double* const q) const
{
  Isometry3d ret(AngleAxisd(q[0], rotation_axis));
  ret.makeAffine();
  return ret;
}

Matrix<double, TWIST_SIZE, 1> RevoluteJoint::spatialJointAxis(const Vector3d& rotation_axis)
{
  Matrix<double, TWIST_SIZE, 1> ret;
  ret.topRows<3>() = rotation_axis;
  ret.bottomRows<3>() = Vector3d::Zero();
  return ret;
}
