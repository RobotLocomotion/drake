#include "QuaternionFloatingJoint.h"

using namespace Eigen;

QuaternionFloatingJoint::QuaternionFloatingJoint(const std::string& name, const RigidBody& parent_body, const AffineCompact3d& transform_to_parent_body) :
    DrakeJoint(name, parent_body, transform_to_parent_body, 7, 6)
{
  // empty
}

QuaternionFloatingJoint::~QuaternionFloatingJoint()
{
  // empty
}

AffineCompact3d QuaternionFloatingJoint::jointTransform(double* const q) const
{
  return AffineCompact3d(Quaterniond(q[0], q[1], q[2], q[3]));
}

void QuaternionFloatingJoint::motionSubspace(double* const q, MotionSubspaceType& motion_subspace, MatrixXd* dmotion_subspace) const
{
}


void QuaternionFloatingJoint::motionSubspaceDotTimesV(double* const q, double* const v, Vector6d& motion_subspace_dot_times_v,
    Matrix<double, TWIST_SIZE, Dynamic>* dmotion_subspace_dot_times_vdq,
    Matrix<double, TWIST_SIZE, Dynamic>* dmotion_subspace_dot_times_vdv) const
{
}
