#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

constexpr int kIiwaArmNumJoints = 7;

/** Returns the maximum joint velocities (rad/s) provided by Kuka. */
VectorX<double> get_iiwa_max_joint_velocities();

extern const double kIiwaLcmStatusPeriod;

/** Enumeration for control modes. */
enum class IiwaControlMode { kPositionOnly, kTorqueOnly, kPositionAndTorque };

/** Reports if the given control `mode` includes positions. */
constexpr inline bool position_enabled(IiwaControlMode control_mode) {
  return control_mode != IiwaControlMode::kTorqueOnly;
}

/** Reports if the given control `mode` includes torques. */
constexpr inline bool torque_enabled(IiwaControlMode control_mode) {
  return control_mode != IiwaControlMode::kPositionOnly;
}

/** Parses control mode with the following mapping:
- "position_only": kPositionOnly
- "torque_only": kTorqueOnly
- "position_and_torque": kPositionAndTorque */
IiwaControlMode ParseIiwaControlMode(const std::string& control_mode);

/** Formats control mode to a string using the mappping shown above. */
std::string FormatIiwaControlMode(IiwaControlMode mode);

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
