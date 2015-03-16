#include "PrismaticJoint.h"
#include <Eigen/Geometry>

#include "RigidBodyManipulator.h" // todo: remove this when I remove setupOldKinematicTree

using namespace Eigen;

PrismaticJoint::PrismaticJoint(const std::string& name, const Isometry3d& transform_to_parent_body, const Vector3d& translation_axis) :
    FixedAxisOneDoFJoint(name, transform_to_parent_body, spatialJointAxis(translation_axis)), translation_axis(translation_axis)
{
  assert(abs(translation_axis.norm()-1)<1e-10);
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

void PrismaticJoint::setupOldKinematicTree(RigidBodyManipulator* model, int body_ind, int position_num_start, int velocity_num_start) const
{
  FixedAxisOneDoFJoint::setupOldKinematicTree(model,body_ind,position_num_start,velocity_num_start);
  model->bodies[body_ind]->pitch = INF;

  Vector3d z_axis(0.0,0.0,1.0);
  if (translation_axis.dot(z_axis)<1-1e-4) {
    Vector4d a;
    a << translation_axis.cross(z_axis), acos(translation_axis.dot(z_axis));
    if ((std::abs(a(0))<1e-4) && (std::abs(a(1))<1e-4) && (std::abs(a(2))<1e-4))
      a.head(3) << 0.0, 1.0, 0.0;
    model->bodies[body_ind]->T_body_to_joint.topLeftCorner(3,3) = axis2rotmat(a);
  }

}

