#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

constexpr int kIiwaArmNumJoints = 7;

/// Returns the maximum joint velocities provided by Kuka.
/// @return Maximum joint velocities (rad/s).
VectorX<double> get_iiwa_max_joint_velocities();

extern const double kIiwaLcmStatusPeriod;

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
