#include "drake/systems/plants/joints/DrakeJoint.h"

#include "drake/common/drake_assert.h"

using namespace Eigen;

DrakeJoint::DrakeJoint(const std::string& _name,
                       const Isometry3d& _transform_to_parent_body,
                       int _num_positions, int _num_velocities)
    : name(_name),
      joint_limit_min(VectorXd::Constant(
          _num_positions, -std::numeric_limits<double>::infinity())),
      joint_limit_max(VectorXd::Constant(
          _num_positions, std::numeric_limits<double>::infinity())),
      transform_to_parent_body(_transform_to_parent_body),
      num_positions(_num_positions),
      num_velocities(_num_velocities) {
  DRAKE_ASSERT(num_positions <= MAX_NUM_POSITIONS);
  DRAKE_ASSERT(num_velocities <= MAX_NUM_VELOCITIES);
}

DrakeJoint::~DrakeJoint() {
  // empty
}

const Isometry3d& DrakeJoint::getTransformToParentBody() const {
  return transform_to_parent_body;
}

int DrakeJoint::getNumPositions() const { return num_positions; }

int DrakeJoint::getNumVelocities() const { return num_velocities; }

const std::string& DrakeJoint::getName() const { return name; }

const Eigen::VectorXd& DrakeJoint::getJointLimitMin() const {
  return joint_limit_min;
}

const Eigen::VectorXd& DrakeJoint::getJointLimitMax() const {
  return joint_limit_max;
}
