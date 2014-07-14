#include "OneDoFJoint.h"
#include <cmath>
#include <Eigen/Core>

OneDoFJoint::OneDoFJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body,
    const Matrix<double, DrakeJoint::TWIST_SIZE, 1>& joint_axis) :
  DrakeJoint(parent_body, transform_to_parent_body), joint_axis(joint_axis)
{
  // empty
}

OneDoFJoint::~OneDoFJoint()
{
  // empty
}

template <typename DerivedA, typename DerivedB>
void OneDoFJoint::motionSubspace(double* const q, MatrixBase<DerivedA>& motion_subspace, MatrixBase<DerivedB>* dmotion_subspace) const
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(motion_subspace);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(motion_subspace, DrakeJoint::TWIST_SIZE, NUM_VELOCITIES);

  motion_subspace = joint_axis;

  if (dmotion_subspace) {
    dmotion_subspace->setZero();
  }
}
