#include "QuaternionFloatingJoint.h"
#include <random>

#define _USE_MATH_DEFINES
#include <cmath>

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
  return AffineCompact3d(Quaterniond(q[3], q[4], q[5], q[6]) * Translation3d(q[0], q[1], q[2]));
}

void QuaternionFloatingJoint::motionSubspace(double* const q, MotionSubspaceType& motion_subspace, MatrixXd* dmotion_subspace) const
{
  motion_subspace.setIdentity(TWIST_SIZE, getNumVelocities());
  if (dmotion_subspace) {
    dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
  }
}

void QuaternionFloatingJoint::motionSubspaceDotTimesV(double* const q, double* const v, Vector6d& motion_subspace_dot_times_v,
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

void QuaternionFloatingJoint::randomConfiguration(double* q, std::default_random_engine& generator) const
{
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(-M_PI, M_PI);

  // position
  q[0] = normal(generator);
  q[1] = normal(generator);
  q[2] = normal(generator);

  // orientation
  double angle = uniform(generator);
  Eigen::Vector3d axis = Vector3d(normal(generator), normal(generator), normal(generator));
  axis.normalize();
  Quaterniond quat;
  quat = AngleAxisd(angle, axis);
  q[3] = quat.w();
  q[4] = quat.x();
  q[5] = quat.y();
  q[6] = quat.z();
}
