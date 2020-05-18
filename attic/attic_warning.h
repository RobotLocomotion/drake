#pragma once

#include "drake/common/drake_deprecated.h"

namespace drake {
namespace internal {

// Displays a deprecation warning the first time we are included in a TU.
class DRAKE_DEPRECATED("2020-09-01",
    "All Drake code in the 'attic' is deprecated. This includes "
    "RigidBodyTree and RigidBodyPlant and their visualization, "
    "sensors for RigidBodyPlant such as accelerometer, gyro, camera, etc., "
    "inverse kinematics and inverse dynamics based on RigidBodyTree, and "
    "SimDiagramBuilder and WorldSimTreeBuilder based on RigidBodyTree. "
    "Developers should use the replacement MultibodyPlant family instead. "
    "See https://github.com/RobotLocomotion/drake/issues/12158 for details.")
AllDrakeCodeInTheAttic {};
inline constexpr AllDrakeCodeInTheAttic warning;

/** Prints a deprecation warning the first time it is called.  Subsequent calls
print no warning. */
void WarnOnceAboutAtticCode();

}  // namespace internal
}  // namespace drake
