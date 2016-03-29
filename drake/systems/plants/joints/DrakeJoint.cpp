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

bool DrakeJoint::Compare(const DrakeJoint & jj, std::string * explanation) const {
  bool result = true;

  // Compare name
  if (name.compare(jj.name) != 0) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "Joint names do not match! (" << name << " vs. " << jj.name;
      *explanation = ss.str();
    }
    result = false;
  }

  // Compare num_positions
  if (result && num_positions != jj.num_positions) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "Joint " << name << " num_positions do not match! ("
         << num_positions << " vs. " << jj.num_positions;
      *explanation = ss.str();
    }
    result = false;
  }

  // Compare num_velocities
  if (result && num_velocities != jj.num_velocities) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "Joint " << name << " num_velocities do not match! ("
         << num_velocities << " vs. " << jj.num_velocities;
      *explanation = ss.str();
    }
    result = false;
  }

  // Compare transform_to_body
  if (result) {
    try {
      valuecheckMatrix(transform_to_parent_body.matrix(), jj.transform_to_parent_body.matrix(), std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "Joint " << name << " transform_to_body matrices do not match!\n"
           << re.what();
        *explanation = ss.str();
      }
      result = false;
    }
  }

  // Compare joint_limit_min
  if (result) {
    try {
      valuecheckMatrix(joint_limit_min, jj.joint_limit_min, std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "Joint " << name << " joint_limit_min vectors do not match!\n" << re.what();
        *explanation = ss.str();
      }
      result = false;
    }
  }

  // Compare joint_limit_max
  if (result) {
    try {
      valuecheckMatrix(joint_limit_max, jj.joint_limit_max, std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "Joint " << name << " joint_limit_max vectors do not match!\n" << re.what();
        *explanation = ss.str();
      }
      result = false;
    }
  }

  return result;
}

bool operator==(const DrakeJoint & j1, const DrakeJoint & j2) {
  return j1.Compare(j2);
}

bool operator!=(const DrakeJoint & j1, const DrakeJoint & j2) {
  return !operator==(j1, j2);
}