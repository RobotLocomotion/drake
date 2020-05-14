#include "drake/attic_warning.h"

#include "drake/common/text_logging.h"

namespace drake {
namespace internal {

void WarnOnceAboutAtticCode() {
  static const logging::Warn log_once(
      "All Drake code in the 'attic' is deprecated and will be "
      "removed from Drake on or after 2020-09-01. This includes "
      "RigidBodyTree and RigidBodyPlant and their visualization, "
      "sensors for RigidBodyPlant such as accelerometer, gyro, camera, etc., "
      "inverse kinematics and inverse dynamics based on RigidBodyTree, and "
      "SimDiagramBuilder and WorldSimTreeBuilder based on RigidBodyTree. "
      "Developers should use the replacement MultibodyPlant family instead. "
      "See https://github.com/RobotLocomotion/drake/issues/12158 for details."
);
}

}  // namespace internal
}  // namespace drake
