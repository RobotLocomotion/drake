#pragma once

#include "drake/attic_warning.h"
#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/joints/fixed_axis_one_dof_joint.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/helical_joint.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/quaternion_ball_joint.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"

namespace drake {
namespace multibody {

/// @name Drake joint comparison methods.
///
/// These methods compare joint `original` with joint `clone`. Since these
/// methods are intended to compare a clone, an *exact* match is performed. This
/// method will only return `true` if the provided `clone` joint is exactly the
/// same as the provided `original` joint.
/// @{

bool CompareDrakeJointToClone(
    const DrakeJoint& original,
    const DrakeJoint& clone);

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

bool CompareFixedJointToClone(
    const FixedJoint& original,
    const FixedJoint& clone);

bool CompareHelicalJointToClone(
    const HelicalJoint& original,
    const HelicalJoint& clone);

bool ComparePrismaticJointToClone(
    const PrismaticJoint& original,
    const PrismaticJoint& clone);

bool CompareQuaternionBallJointToClone(
    const QuaternionBallJoint& original,
    const QuaternionBallJoint& clone);

bool CompareQuaternionFloatingJointToClone(
    const QuaternionFloatingJoint& original,
    const QuaternionFloatingJoint& clone);

bool CompareRevoluteJointToClone(
    const RevoluteJoint& original,
    const RevoluteJoint& clone);

bool CompareRollPitchYawFloatingJointToClone(
    const RollPitchYawFloatingJoint& original,
    const RollPitchYawFloatingJoint& clone);
/// @}

}  // namespace multibody
}  // namespace drake
