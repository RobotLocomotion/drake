#include "FixedAxisOneDoFJoint.h"
#include "RigidBodyManipulator.h" // todo: remove this when I remove setupOldKinematicTree

using namespace Eigen;

template <typename Derived>
void FixedAxisOneDoFJoint<Derived>::setJointLimits(double joint_limit_min, double joint_limit_max)
{
  if (joint_limit_min > joint_limit_max) {
    throw std::logic_error("joint_limit_min cannot be larger than joint_limit_max");
  }

  this->DrakeJoint::joint_limit_min[0] = joint_limit_min;
  this->DrakeJoint::joint_limit_max[0] = joint_limit_max;
}

template <typename Derived>
Eigen::VectorXd FixedAxisOneDoFJoint<Derived>::randomConfiguration(std::default_random_engine& generator) const
{
	Eigen::VectorXd q(1);
	if (isFinite(DrakeJoint::joint_limit_min.value()) && isFinite(DrakeJoint::joint_limit_max.value())) {
    std::uniform_real_distribution<double> distribution(DrakeJoint::joint_limit_min.value(), DrakeJoint::joint_limit_max.value());
    q[0] = distribution(generator);
  }
  else {
    std::normal_distribution<double> distribution;
    double stddev = 1.0;
    double joint_limit_offset = 1.0;
    if (isFinite(DrakeJoint::joint_limit_min.value())) {
      distribution = std::normal_distribution<double>(DrakeJoint::joint_limit_min.value() + joint_limit_offset, stddev);
    }
    else if (isFinite(DrakeJoint::joint_limit_max.value())) {
      distribution = std::normal_distribution<double>(DrakeJoint::joint_limit_max.value() - joint_limit_offset, stddev);
    }
    else {
      distribution = std::normal_distribution<double>();
    }

    q[0] = distribution(generator);
    if (q[0] < DrakeJoint::joint_limit_min.value()) {
      q[0] = DrakeJoint::joint_limit_min.value();
    }
    if (q[0] > DrakeJoint::joint_limit_max.value()) {
      q[0] = DrakeJoint::joint_limit_max.value();
    }
  }
	return q;
}

template <typename Derived>
void FixedAxisOneDoFJoint<Derived>::setDynamics(double damping, double coulomb_friction, double coulomb_window)
{
	this->damping = damping;
	this->coulomb_friction = coulomb_friction;
	this->coulomb_window = coulomb_window;
}

