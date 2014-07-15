#include "FixedAxisOneDoFJoint.h"
#include <cmath>
#include <Eigen/Core>

FixedAxisOneDoFJoint::FixedAxisOneDoFJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body, const Matrix<double, TWIST_SIZE, 1>& joint_axis) :
    DrakeJoint(parent_body, transform_to_parent_body, 1, 1), joint_axis(joint_axis)
{
  // empty
}

FixedAxisOneDoFJoint::~FixedAxisOneDoFJoint()
{
  // empty
}

void FixedAxisOneDoFJoint::motionSubspace(double* const q, MotionSubspaceType& motion_subspace, e::MatrixXd* dmotion_subspace) const
{
  motion_subspace = joint_axis;
  if (dmotion_subspace) {
    dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
  }
}

void FixedAxisOneDoFJoint::motionSubspaceDotTimesV(double* const q, double* const v, Vector6d& motion_subspace_dot_times_v, e::Matrix<double, TWIST_SIZE, e::Dynamic>* dmotion_subspace_dot_times_vdq, e::Matrix<double, TWIST_SIZE, e::Dynamic>* dmotion_subspace_dot_times_vdv) const
{
  motion_subspace_dot_times_v.setZero();

  if (dmotion_subspace_dot_times_vdq) {
    dmotion_subspace_dot_times_vdq->setZero(TWIST_SIZE, 1);
  }

  if (dmotion_subspace_dot_times_vdv) {
    dmotion_subspace_dot_times_vdv->setZero(TWIST_SIZE, 1);
  }
}
