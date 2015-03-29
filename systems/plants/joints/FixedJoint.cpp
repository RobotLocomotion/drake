/*
 * FixedJoint.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: twan
 */

#include "FixedJoint.h"

FixedJoint::FixedJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body) :
  DrakeJoint(name, transform_to_parent_body, 0, 0)
{
  // empty
}

FixedJoint::~FixedJoint()
{
  // empty
}

Eigen::Isometry3d FixedJoint::jointTransform(const Eigen::Ref<const Eigen::VectorXd>& q) const
{
  return Eigen::Isometry3d::Identity();
}

void FixedJoint::motionSubspace(const Eigen::Ref<const Eigen::VectorXd>& q, MotionSubspaceType& motion_subspace, Eigen::MatrixXd* dmotion_subspace) const
{
  motion_subspace.resize(TWIST_SIZE, getNumVelocities());
  if (dmotion_subspace) {
    dmotion_subspace->resize(motion_subspace.size(), getNumPositions());
  }
}

void FixedJoint::motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v, Vector6d& motion_subspace_dot_times_v,
    Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq,
    Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv) const
{
  motion_subspace_dot_times_v.setZero();

  if (dmotion_subspace_dot_times_vdq) {
    dmotion_subspace_dot_times_vdq->setZero(TWIST_SIZE, 1);
  }

  if (dmotion_subspace_dot_times_vdv) {
    dmotion_subspace_dot_times_vdv->setZero(TWIST_SIZE, 1);
  }
}

void FixedJoint::randomConfiguration(Eigen::Ref<Eigen::VectorXd>& q, std::default_random_engine& generator) const
{
  // empty
}

void FixedJoint::qdot2v(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const
{
  qdot_to_v.resize(getNumVelocities(), getNumPositions());
  if (dqdot_to_v) {
    dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
  }
}

void FixedJoint::v2qdot(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const
{
  v_to_qdot.resize(getNumPositions(), getNumVelocities());
  if (dv_to_qdot) {
    dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());
  }
}
