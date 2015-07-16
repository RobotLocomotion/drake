#include "FixedAxisOneDoFJoint.h"
#include "drakeFloatingPointUtil.h"
#include <cmath>
#include <Eigen/Core>
#include <limits>
#include <exception>
#include <stdexcept>

#include "RigidBodyManipulator.h" // todo: remove this when I remove setupOldKinematicTree


using namespace Eigen;

FixedAxisOneDoFJoint::FixedAxisOneDoFJoint(const std::string& name, const Isometry3d& transform_to_parent_body, const Matrix<double, TWIST_SIZE, 1>& joint_axis) :
    DrakeJoint(name, transform_to_parent_body, 1, 1),
    joint_axis(joint_axis),
    joint_limit_min(-std::numeric_limits<double>::infinity()),
    joint_limit_max(std::numeric_limits<double>::infinity()),
		damping(0.0),
		coulomb_friction(0.0),
		coulomb_window(0.0)
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


void FixedAxisOneDoFJoint::motionSubspace(const Eigen::Ref<const VectorXd>& q, MotionSubspaceType& motion_subspace, MatrixXd* dmotion_subspace) const
{
  motion_subspace = joint_axis;
  if (dmotion_subspace) {
    dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
  }
}

void FixedAxisOneDoFJoint::motionSubspaceDotTimesV(const Eigen::Ref<const VectorXd>& q, const Eigen::Ref<const VectorXd>& v,
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

Eigen::VectorXd FixedAxisOneDoFJoint::randomConfiguration(std::default_random_engine& generator) const
{
	Eigen::VectorXd q(1);
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
	return q;
}

void FixedAxisOneDoFJoint::qdot2v(const Eigen::Ref<const VectorXd>& q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const
{
  qdot_to_v.setIdentity(getNumVelocities(), getNumPositions());
  if (dqdot_to_v) {
    dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
  }
}

void FixedAxisOneDoFJoint::v2qdot(const Eigen::Ref<const VectorXd>& q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const
{
  v_to_qdot.setIdentity(getNumPositions(), getNumVelocities());
  if (dv_to_qdot) {
    dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());
  }
}

void FixedAxisOneDoFJoint::setDynamics(double damping, double coulomb_friction, double coulomb_window)
{
	this->damping = damping;
	this->coulomb_friction = coulomb_friction;
	this->coulomb_window = coulomb_window;
}

GradientVar<double, Eigen::Dynamic, 1> FixedAxisOneDoFJoint::frictionTorque(const Eigen::Ref<const Eigen::VectorXd>& v, int gradient_order) const {
  GradientVar<double, Eigen::Dynamic, 1> ret(getNumVelocities(), 1, getNumVelocities(), gradient_order);
  ret.value()[0] = damping * v[0];
  ret.value()[0] += std::min(1.0, std::max(-1.0, v[0] / coulomb_window)) * coulomb_friction;
  if (gradient_order > 0) {
    ret.gradient().value()(0, 0) = damping;
    if (std::abs(v[0]) < coulomb_window)
      ret.gradient().value()(0, 0) += sign(v[0]) * (coulomb_friction / coulomb_window);
  }
  return ret;
}

