#include "FixedAxisOneDoFJoint.h"
#include <cmath>
#include <Eigen/Core>

using namespace Eigen;

FixedAxisOneDoFJoint::FixedAxisOneDoFJoint(const std::string& name, const RigidBody& parent_body, const AffineCompact3d& transform_to_parent_body, const Matrix<double, TWIST_SIZE, 1>& joint_axis) :
    DrakeJoint(name, parent_body, transform_to_parent_body, 1, 1), joint_axis(joint_axis)
{
  // empty
}

FixedAxisOneDoFJoint::~FixedAxisOneDoFJoint()
{
  // empty
}

void FixedAxisOneDoFJoint::motionSubspace(double* const q, MotionSubspaceType& motion_subspace, MatrixXd* dmotion_subspace) const
{
  motion_subspace = joint_axis;
  if (dmotion_subspace) {
    dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
  }
}

void FixedAxisOneDoFJoint::motionSubspaceDotTimesV(double* const q, double* const v, Vector6d& motion_subspace_dot_times_v, Matrix<double, TWIST_SIZE, Dynamic>* dmotion_subspace_dot_times_vdq, Matrix<double, TWIST_SIZE, Dynamic>* dmotion_subspace_dot_times_vdv) const
{
  motion_subspace_dot_times_v.setZero();

  if (dmotion_subspace_dot_times_vdq) {
    dmotion_subspace_dot_times_vdq->setZero(TWIST_SIZE, 1);
  }

  if (dmotion_subspace_dot_times_vdv) {
    dmotion_subspace_dot_times_vdv->setZero(TWIST_SIZE, 1);
  }
}
