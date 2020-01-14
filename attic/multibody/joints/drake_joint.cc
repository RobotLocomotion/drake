#include "drake/multibody/joints/drake_joint.h"

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

using Eigen::Isometry3d;
using Eigen::VectorXd;

DrakeJoint::DrakeJoint(const std::string& name,
                       const Isometry3d& transform_to_parent_body,
                       int num_positions, int num_velocities)
    : name_(name),
      joint_limit_min_(VectorXd::Constant(
          num_positions, -std::numeric_limits<double>::infinity())),
      joint_limit_max_(VectorXd::Constant(
          num_positions, std::numeric_limits<double>::infinity())),
      joint_limit_stiffness_(VectorXd::Constant(
          num_positions, 150. /* Historic default from RigidBodyPlant. */)),
      joint_limit_dissipation_(VectorXd::Constant(
          num_positions, 1. /* Arbitrary, reasonable default. */)),
      transform_to_parent_body_(transform_to_parent_body),
      num_positions_(num_positions),
      num_velocities_(num_velocities) {
  DRAKE_ASSERT(num_positions <= MAX_NUM_POSITIONS);
  DRAKE_ASSERT(num_velocities <= MAX_NUM_VELOCITIES);
}

DrakeJoint::~DrakeJoint() {
  // empty
}

std::unique_ptr<DrakeJoint> DrakeJoint::Clone() const {
  std::unique_ptr<DrakeJoint> clone = DoClone();
  InitializeClone(clone.get());
  return clone;
}

std::string DrakeJoint::get_velocity_name(int index) const {
  return get_position_name(index) + "dot";
}

const Eigen::VectorXd& DrakeJoint::getJointLimitMin() const {
  return joint_limit_min_;
}

const Eigen::VectorXd& DrakeJoint::getJointLimitMax() const {
  return joint_limit_max_;
}

const Eigen::VectorXd& DrakeJoint::get_joint_limit_stiffness() const {
  return joint_limit_stiffness_;
}

const Eigen::VectorXd& DrakeJoint::get_joint_limit_dissipation() const {
  return joint_limit_dissipation_;
}

void DrakeJoint::InitializeClone(DrakeJoint* clone) const {
  DoInitializeClone(clone);
  clone->joint_limit_min_ = joint_limit_min_;
  clone->joint_limit_max_ = joint_limit_max_;
  clone->joint_limit_stiffness_ = joint_limit_stiffness_;
  clone->joint_limit_dissipation_ = joint_limit_dissipation_;
}
