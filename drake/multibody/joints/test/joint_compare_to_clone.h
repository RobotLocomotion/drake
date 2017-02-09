#pragma once

#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/joints/fixed_axis_one_dof_joint.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/helical_joint.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"

namespace drake {
namespace multibody {

/// Compares joint `original` with joint `clone`. Since this method is intended
/// to compare a clone, an *exact* match is performed. This method will only
/// return `true` if the provided `clone` joint is exactly the same as the
/// provided `original` joint.
bool CompareDrakeJointToClone(
    const DrakeJoint& original,
    const DrakeJoint& clone);

/// Compares joint `original` with joint `clone`. Since this method is intended
/// to compare a clone, an *exact* match is performed. This method will only
/// return `true` if the provided `clone` joint is exactly the same as the
/// provided `original` joint.
template <typename Derived>
bool CompareFixedAxisOneDofJointToClone(
    const FixedAxisOneDoFJoint<Derived>& original,
    const FixedAxisOneDoFJoint<Derived>& clone) {
  if (!CompareDrakeJointToClone(original, clone)) {
    return false;
  }
  if (original.joint_axis() != clone.joint_axis()) {
    drake::log()->debug(
        "CompareFixedAxisOneDofJointToClone(): "
        "joint_axis mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.joint_axis(),
        clone.joint_axis());
    return false;
  }
  if (original.damping() != clone.damping()) {
      drake::log()->debug(
        "CompareFixedAxisOneDofJointToClone(): "
        "damping mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.damping(),
        clone.damping());
    return false;
  }
  if (original.coulomb_friction() != clone.coulomb_friction()) {
      drake::log()->debug(
        "CompareFixedAxisOneDofJointToClone(): "
        "coulomb_friction mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.coulomb_friction(),
        clone.coulomb_friction());
    return false;
  }
  if (original.coulomb_window() != clone.coulomb_window()) {
      drake::log()->debug(
        "CompareFixedAxisOneDofJointToClone(): "
        "coulomb_window mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.coulomb_window(),
        clone.coulomb_window());
    return false;
  }
  return true;
}

/// Compares joint `original` with joint `clone`. Since this method is intended
/// to compare a clone, an *exact* match is performed. This method will only
/// return `true` if the provided `clone` joint is exactly the same as the
/// provided `original` joint.
bool CompareFixedJointToClone(
    const FixedJoint& original,
    const FixedJoint& clone);

/// Compares joint `original` with joint `clone`. Since this method is intended
/// to compare a clone, an *exact* match is performed. This method will only
/// return `true` if the provided `clone` joint is exactly the same as the
/// provided `original` joint.
bool CompareHelicalJointToClone(
    const HelicalJoint& original,
    const HelicalJoint& clone);

/// Compares joint `original` with joint `clone`. Since this method is intended
/// to compare a clone, an *exact* match is performed. This method will only
/// return `true` if the provided `clone` joint is exactly the same as the
/// provided `original` joint.
bool ComparePrismaticJointToClone(
    const PrismaticJoint& original,
    const PrismaticJoint& clone);

/// Compares joint `original` with joint `clone`. Since this method is intended
/// to compare a clone, an *exact* match is performed. This method will only
/// return `true` if the provided `clone` joint is exactly the same as the
/// provided `original` joint.
bool CompareQuaternionFloatingJointToClone(
    const QuaternionFloatingJoint& original,
    const QuaternionFloatingJoint& clone);

/// Compares joint `original` with joint `clone`. Since this method is intended
/// to compare a clone, an *exact* match is performed. This method will only
/// return `true` if the provided `clone` joint is exactly the same as the
/// provided `original` joint.
bool CompareRevoluteJointToClone(
    const RevoluteJoint& original,
    const RevoluteJoint& clone);

/// Compares joint `original` with joint `clone`. Since this method is intended
/// to compare a clone, an *exact* match is performed. This method will only
/// return `true` if the provided `clone` joint is exactly the same as the
/// provided `original` joint.
bool CompareRollPitchYawFloatingJointToClone(
    const RollPitchYawFloatingJoint& original,
    const RollPitchYawFloatingJoint& clone);
}  // namespace multibody
}  // namespace drake
