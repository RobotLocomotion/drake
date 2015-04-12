#include "RevoluteJoint.h"
#include <Eigen/Geometry>

#include "RigidBodyManipulator.h" // todo: remove this when I remove setupOldKinematicTree

using namespace Eigen;

RevoluteJoint::RevoluteJoint(const std::string& name, const Isometry3d& transform_to_parent_body, const Vector3d& rotation_axis) :
    FixedAxisOneDoFJoint(name, transform_to_parent_body, spatialJointAxis(rotation_axis)), rotation_axis(rotation_axis)
{
  assert(abs(rotation_axis.norm()-1)<1e-10);
}

RevoluteJoint::~RevoluteJoint()
{
  // empty
}

Isometry3d RevoluteJoint::jointTransform(const Eigen::Ref<const VectorXd>& q) const
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

void RevoluteJoint::setupOldKinematicTree(RigidBodyManipulator* model, int body_ind, int position_num_start, int velocity_num_start) const
{
  FixedAxisOneDoFJoint::setupOldKinematicTree(model,body_ind,position_num_start,velocity_num_start);
  model->bodies[body_ind]->pitch = 0;

  Vector3d z_axis(0.0,0.0,1.0);
  if (rotation_axis.dot(z_axis)<1-1e-4) {
//    std::cout << "T_body_to_joint (before) = " << std::endl << model->bodies[body_ind]->T_body_to_joint << std::endl;
    Vector4d a;
    a << rotation_axis.cross(z_axis), acos(rotation_axis.dot(z_axis));
    if ((std::abs(a(0))<1e-4) && (std::abs(a(1))<1e-4) && (std::abs(a(2))<1e-4))
      a.head(3) << 0.0, 1.0, 0.0;
//    std::cout << "axis_angle = " << a.transpose() << std::endl;
    model->bodies[body_ind]->T_body_to_joint.topLeftCorner(3,3) = axis2rotmat(a);
//    std::cout << "T_body_to_joint (after) = " << std::endl << model->bodies[body_ind]->T_body_to_joint << std::endl;
  }
}

