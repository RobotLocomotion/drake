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
    kPositionOnly,
    kTorqueOnly,
    kPositionAndTorque
};

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
