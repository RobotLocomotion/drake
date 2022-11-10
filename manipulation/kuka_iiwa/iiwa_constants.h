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

enum IiwaControlMode : int {
    kIiwaPositionMode = 0b01,
    kIiwaTorqueMode = 0b10
};

const int kIiwaDefaultMode = kIiwaPositionMode | kIiwaTorqueMode;

/**
Parses control mode, with string options of
{"position", "velocity", "torque"}.
*/
int ParseIiwaControlMode(const std::vector<std::string>& control_mode);

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
