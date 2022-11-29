#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
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
    kPositionOnly,
    kTorqueOnly,
    kPositionAndTorque
};

/** Reports if the given control `mode` includes positions. */
constexpr inline bool has_position_mode(IiwaControlMode control_mode) {
  switch (control_mode) {
    case IiwaControlMode::kPositionOnly:
    case IiwaControlMode::kPositionAndTorque:
      return true;
    case IiwaControlMode::kTorqueOnly:
      return false;
    default:
      DRAKE_UNREACHABLE();
  }
}

/** Reports if the given control `mode` includes torques. */
constexpr inline bool has_torque_mode(IiwaControlMode control_mode) {
  switch (control_mode) {
    case IiwaControlMode::kTorqueOnly:
    case IiwaControlMode::kPositionAndTorque:
      return true;
    case IiwaControlMode::kPositionOnly:
      return false;
    default:
      DRAKE_UNREACHABLE();
  }
}

/**
Parses control mode with the following mapping:
- "position_only": kPositionOnly
- "torque_only": kTorqueOnly
- "position_and_torque": kPositionAndTorque
*/
IiwaControlMode ParseIiwaControlMode(const std::string& control_mode);

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
