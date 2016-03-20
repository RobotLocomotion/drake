#include "drake/systems/plants/joints/DrakeJoint.h"

#include <iostream>
#include "drake/util/testUtil.h" // for valueCheckMatrix

using namespace Eigen;

DrakeJoint::DrakeJoint(const std::string& _name,
                       const Isometry3d& _transform_to_parent_body,
                       int _num_positions, int _num_velocities)
    : name(_name),
      transform_to_parent_body(_transform_to_parent_body),
      num_positions(_num_positions),
      num_velocities(_num_velocities),
      joint_limit_min(VectorXd::Constant(
          _num_positions, -std::numeric_limits<double>::infinity())),
      joint_limit_max(VectorXd::Constant(
          _num_positions, std::numeric_limits<double>::infinity())) {
  assert(num_positions <= MAX_NUM_POSITIONS);
  assert(num_velocities <= MAX_NUM_VELOCITIES);
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

#define PRINT_STMT(x) std::cout << "DrakeJoint: EQUALS: " << x << std::endl;

bool operator==(const DrakeJoint & j1, const DrakeJoint & j2) {
  bool result = true;

  // Compare name
  if (j1.name.compare(j2.name) != 0) {
    PRINT_STMT("Joint names do not match! (" << j1.name << " vs. " << j2.name);
    result = false;
  }

  // Compare num_positions
  if (result && j1.num_positions != j2.num_positions) {
    PRINT_STMT("Joint " << j1.name << " num_positions do not match! (" << j1.num_positions << " vs. " << j2.num_positions);
    result = false;
  }

  // Compare num_velocities
  if (result && j1.num_velocities != j2.num_velocities) {
    PRINT_STMT("Joint " << j1.name << " num_velocities do not match! (" << j1.num_velocities << " vs. " << j2.num_velocities);
    result = false;
  }

  // Compare transform_to_body
  if (result) {
    try {
      valuecheckMatrix(j1.transform_to_parent_body.matrix(), j2.transform_to_parent_body.matrix(), std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      PRINT_STMT("Joint " << j1.name << " transform_to_body matrices do not match!\n" << re.what())
      result = false;
    }
  }

  // Compare joint_limit_min
  if (result) {
    try {
      valuecheckMatrix(j1.joint_limit_min, j2.joint_limit_min, std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      PRINT_STMT("Joint " << j1.name << " joint_limit_min vectors do not match!\n" << re.what())
      result = false;
    }
  }

  // Compare joint_limit_max
  if (result) {
    try {
      valuecheckMatrix(j1.joint_limit_max, j2.joint_limit_max, std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      PRINT_STMT("Joint " << j1.name << " joint_limit_max vectors do not match!\n" << re.what())
      result = false;
    }
  }

  return result;
}

#undef PRINT_STMT

bool operator!=(const DrakeJoint & j1, const DrakeJoint & j2) {
  return !operator==(j1, j2);
}