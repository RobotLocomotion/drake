#pragma once

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

constexpr int kIiwaArmNumJoints = 7;

/**
Returns the maximum joint velocities provided by Kuka.
@return Maximum joint velocities (rad/s).
*/
VectorX<double> get_iiwa_max_joint_velocities();

extern const double kIiwaLcmStatusPeriod;

/** Enumeration for control modes. */
enum class IiwaControlMode {
    kPosition = 0b01,
    kTorque = 0b10,
    // Position and Torque.
    kDefault = 0b01 | 0b10
};
/** Overload operator| for strongly-typed bitwise operations. */
inline IiwaControlMode operator|(IiwaControlMode lhs, IiwaControlMode rhs) {
  return static_cast<IiwaControlMode>(
      static_cast<int>(lhs) | static_cast<int>(rhs));
}
/** Overload operator& for strongly-typed bitwise operations. */
inline IiwaControlMode operator&(IiwaControlMode lhs, IiwaControlMode rhs) {
  return static_cast<IiwaControlMode>(
      static_cast<int>(lhs) & static_cast<int>(rhs));
}
/** Must specify position and/or torque. */
inline bool IsValid(IiwaControlMode control_mode) {
  return (
      static_cast<bool>(control_mode & IiwaControlMode::kPosition) ||
      static_cast<bool>(control_mode & IiwaControlMode::kTorque));
}

/**
Parses control mode, with string options of {"position", "torque"}.
*/
IiwaControlMode ParseIiwaControlMode(
    const std::vector<std::string>& control_mode);

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
