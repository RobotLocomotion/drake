#include "RollPitchYawFloatingJoint.h"
#include <random>

using namespace Eigen;


// TODO: implement everything
RollPitchYawFloatingJoint::RollPitchYawFloatingJoint(const std::string& name, const RigidBody& parent_body, const Isometry3d& transform_to_parent_body) :
  DrakeJoint(name, parent_body, transform_to_parent_body, 6, 6)
{
  // empty
}

RollPitchYawFloatingJoint::~RollPitchYawFloatingJoint()
{
  // empty
}

Isometry3d RollPitchYawFloatingJoint::jointTransform(double* const q) const
{
  return Isometry3d(Quaterniond(q[3], q[4], q[5], q[6]) * Translation3d(q[0], q[1], q[2]));
}

void RollPitchYawFloatingJoint::motionSubspace(double* const q, MotionSubspaceType& motion_subspace, MatrixXd* dmotion_subspace) const
{
  motion_subspace.setIdentity(TWIST_SIZE, getNumVelocities());
  if (dmotion_subspace) {
    dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
  }
}

void RollPitchYawFloatingJoint::motionSubspaceDotTimesV(double* const q, double* const v, Vector6d& motion_subspace_dot_times_v,
    Matrix<double, TWIST_SIZE, Dynamic>* dmotion_subspace_dot_times_vdq,
    Matrix<double, TWIST_SIZE, Dynamic>* dmotion_subspace_dot_times_vdv) const
{
  motion_subspace_dot_times_v.setZero();
  if (dmotion_subspace_dot_times_vdq) {
    dmotion_subspace_dot_times_vdq->setZero(motion_subspace_dot_times_v.size(), getNumPositions());
  }
  if (dmotion_subspace_dot_times_vdv) {
    dmotion_subspace_dot_times_vdv->setZero(motion_subspace_dot_times_v.size(), getNumVelocities());
  }
}

void RollPitchYawFloatingJoint::randomConfiguration(double* q, std::default_random_engine& generator) const
{
  std::normal_distribution<double> normal;

  // position
  q[0] = normal(generator);
  q[1] = normal(generator);
  q[2] = normal(generator);

  // orientation
//  Quaterniond quat = uniformlyRandomQuat(generator);
//  q[3] = quat.w();
//  q[4] = quat.x();
//  q[5] = quat.y();
//  q[6] = quat.z();
}

void RollPitchYawFloatingJoint::qdotToV(double* q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const
{
  qdot_to_v.resize(getNumVelocities(), getNumPositions());

  Quaterniond quat(q[3], q[4], q[5], q[6]);
  const auto R = quat.toRotationMatrix();

  if (dqdot_to_v) {
    dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
  }
//  Matrix<double, num_velocities, num_positions> mat;
}

void RollPitchYawFloatingJoint::vToQdot(double* q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const
{
}
