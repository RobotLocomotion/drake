#include "FixedAxisOneDoFJoint.h"
#include "drakeFloatingPointUtil.h"
#include <cmath>
#include <Eigen/Core>
#include <limits>
#include <exception>
#include <stdexcept>



using namespace Eigen;

FixedAxisOneDoFJoint::FixedAxisOneDoFJoint(const std::string& name, const Isometry3d& transform_to_parent_body, const Matrix<double, TWIST_SIZE, 1>& joint_axis) :
    DrakeJoint(name, transform_to_parent_body, 1, 1),
    joint_axis(joint_axis),
    joint_limit_min(-std::numeric_limits<double>::infinity()),
    joint_limit_max(std::numeric_limits<double>::infinity())
{
  // empty
}

FixedAxisOneDoFJoint::~FixedAxisOneDoFJoint()
{
  // empty
}

void FixedAxisOneDoFJoint::setJointLimits(double joint_limit_min, double joint_limit_max)
{
  if (joint_limit_min > joint_limit_max) {
    throw std::logic_error("joint_limit_min cannot be larger than joint_limit_max");
  }

  this->joint_limit_min = joint_limit_min;
  this->joint_limit_max = joint_limit_max;
}

void FixedAxisOneDoFJoint::motionSubspace(double* const q, MotionSubspaceType& motion_subspace, MatrixXd* dmotion_subspace) const
{
  motion_subspace = joint_axis;
  if (dmotion_subspace) {
    dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
  }
}

void FixedAxisOneDoFJoint::motionSubspaceDotTimesV(double* const q, double* const v,
    Vector6d& motion_subspace_dot_times_v,
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

void FixedAxisOneDoFJoint::randomConfiguration(double* q, std::default_random_engine& generator) const
{
  if (isFinite(joint_limit_min) && isFinite(joint_limit_max)) {
    std::uniform_real_distribution<double> distribution(joint_limit_min, joint_limit_max);
    q[0] = distribution(generator);
  }
  else {
    std::normal_distribution<double> distribution;
    double stddev = 1.0;
    double joint_limit_offset = 1.0;
    if (isFinite(joint_limit_min)) {
      distribution = std::normal_distribution<double>(joint_limit_min + joint_limit_offset, stddev);
    }
    else if (isFinite(joint_limit_max)) {
      distribution = std::normal_distribution<double>(joint_limit_max - joint_limit_offset, stddev);
    }
    else {
      distribution = std::normal_distribution<double>();
    }

    q[0] = distribution(generator);
    if (q[0] < joint_limit_min) {
      q[0] = joint_limit_min;
    }
    if (q[0] > joint_limit_max) {
      q[0] = joint_limit_max;
    }
  }
}

void FixedAxisOneDoFJoint::qdot2v(double* q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const
{
  qdot_to_v.setIdentity(getNumVelocities(), getNumPositions());
  if (dqdot_to_v) {
    dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
  }
}

void FixedAxisOneDoFJoint::v2qdot(double* q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const
{
  v_to_qdot.setIdentity(getNumPositions(), getNumVelocities());
  if (dv_to_qdot) {
    dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());
  }
}
