#include "OneDoFJoint.h"
#include <cmath>
#include <Eigen/Core>

OneDoFJoint::OneDoFJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body,
    const Matrix<double, TWIST_SIZE, 1>& joint_axis) :
  DrakeJoint(parent_body, transform_to_parent_body, 1, 1), joint_axis(joint_axis)
{
  // empty
}

OneDoFJoint::~OneDoFJoint()
{
  // empty
}

void OneDoFJoint::motionSubspace(double* const q, MotionSubspaceType& motion_subspace, e::MatrixXd* dmotion_subspace) const
{
  motion_subspace = joint_axis;
  if (dmotion_subspace)
  {
    dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
  }
}
