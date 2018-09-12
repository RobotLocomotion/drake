#include "drake/multibody/joints/test/joint_compare_to_clone.h"

#include <typeinfo>

#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {

bool CompareDrakeJointToClone(const DrakeJoint& original,
    const DrakeJoint& clone) {
  if (typeid(original) != typeid(clone)) {
    drake::log()->debug(
        "CompareDrakeJointToClone(): "
        "types mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        typeid(original).name(),
        typeid(clone).name());
    return false;
  }
  if (original.get_transform_to_parent_body().matrix() !=
      clone.get_transform_to_parent_body().matrix()) {
    drake::log()->debug(
        "CompareDrakeJointToClone(): "
        "transform_to_parent_body mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_transform_to_parent_body().matrix(),
        clone.get_transform_to_parent_body().matrix());
    return false;
  }
  if (original.get_num_positions() != clone.get_num_positions()) {
    drake::log()->debug(
        "CompareDrakeJointToClone(): get_num_positions mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_num_positions(),
        clone.get_num_positions());
    return false;
  }
  if (original.get_num_velocities() != clone.get_num_velocities()) {
    drake::log()->debug(
        "CompareDrakeJointToClone(): get_num_velocities mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_num_velocities(),
        clone.get_num_velocities());
    return false;
  }
  if (original.get_name() != clone.get_name()) {
    drake::log()->debug(
        "CompareDrakeJointToClone(): get_name mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_name(),
        clone.get_name());
    return false;
  }
  if (original.getJointLimitMin() != clone.getJointLimitMin()) {
    drake::log()->debug(
        "CompareDrakeJointToClone(): getJointLimitMin mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.getJointLimitMin(),
        clone.getJointLimitMin());
    return false;
  }
  if (original.getJointLimitMax() != clone.getJointLimitMax()) {
    drake::log()->debug(
        "CompareDrakeJointToClone(): getJointLimitMax mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.getJointLimitMax(),
        clone.getJointLimitMax());
    return false;
  }
  if (original.get_joint_limit_stiffness() !=
      clone.get_joint_limit_stiffness()) {
    drake::log()->debug(
        "CompareDrakeJointToClone(): "
        "get_joint_limit_stiffness mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_joint_limit_stiffness(),
        clone.get_joint_limit_stiffness());
    return false;
  }
  if (original.get_joint_limit_dissipation() !=
      clone.get_joint_limit_dissipation()) {
    drake::log()->debug(
        "CompareDrakeJointToClone(): "
        "get_joint_limit_dissipation mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_joint_limit_dissipation(),
        clone.get_joint_limit_dissipation());
    return false;
  }
  return true;
}

bool CompareFixedJointToClone(const FixedJoint& original,
    const FixedJoint& other) {
  return CompareDrakeJointToClone(original, other);
}


bool CompareHelicalJointToClone(const HelicalJoint& original,
    const HelicalJoint& clone) {
  if (!CompareFixedAxisOneDofJointToClone<HelicalJoint>(original, clone)) {
    return false;
  }
  if (original.axis() != clone.axis()) {
    drake::log()->debug(
        "CompareHelicalJointToClone(): axis mismatch:\n"
        "  - original: {}\n"
        "  - other: {}",
        original.axis(),
        clone.axis());
    return false;
  }
  if (original.pitch()  != clone.pitch()) {
      drake::log()->debug(
        "CompareHelicalJointToClone(): pitch mismatch:\n"
        "  - original: {}\n"
        "  - other: {}",
        original.pitch(),
        clone.pitch());
    return false;
  }
  return true;
}

bool ComparePrismaticJointToClone(const PrismaticJoint& original,
    const PrismaticJoint& clone) {
  if (!CompareFixedAxisOneDofJointToClone<PrismaticJoint>(original, clone)) {
    return false;
  }
  if (original.translation_axis() != clone.translation_axis()) {
    drake::log()->debug(
        "ComparePrismaticJointToClone(): translation_axis mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.translation_axis().transpose(),
        clone.translation_axis().transpose());
    return false;
  }
  return true;
}

bool CompareQuaternionBallJointToClone(
    const QuaternionBallJoint& original,
    const QuaternionBallJoint& clone) {
  return CompareDrakeJointToClone(original, clone);
}

bool CompareQuaternionFloatingJointToClone(
    const QuaternionFloatingJoint& original,
    const QuaternionFloatingJoint& clone) {
  return CompareDrakeJointToClone(original, clone);
}

bool CompareRevoluteJointToClone(const RevoluteJoint& original,
    const RevoluteJoint& clone) {
  if (!CompareFixedAxisOneDofJointToClone<RevoluteJoint>(original, clone)) {
    return false;
  }
  if (original.rotation_axis() != clone.rotation_axis()) {
    drake::log()->debug(
        "CompareRevoluteJointToClone(): rotation_axis mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.rotation_axis().transpose(),
        clone.rotation_axis().transpose());
    return false;
  }
  return true;
}

bool CompareRollPitchYawFloatingJointToClone(
    const RollPitchYawFloatingJoint& original,
    const RollPitchYawFloatingJoint& clone) {
  return CompareDrakeJointToClone(original, clone);
}

}  // namespace multibody
}  // namespace drake
