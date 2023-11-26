#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

void MultibodyTreeTopology::RemoveJointActuator(
    JointActuatorIndex actuator_index) {
  DRAKE_DEMAND(actuator_index < ssize(joint_actuators_));
  if (is_valid()) {
    throw std::logic_error(
        "RemoveJointActuator() must be called pre-Finalize.");
  }

  if (!joint_actuators_[actuator_index].has_value()) {
    throw std::logic_error(
        fmt::format("RemoveJointActuator(): Actuator with index {} has "
                    "already been removed.",
                    actuator_index));
  }

  // Reduce the total number of actuated dofs.
  const int num_dofs = (*joint_actuators_[actuator_index]).num_dofs;
  DRAKE_ASSERT(num_actuated_dofs_ >= num_dofs);
  num_actuated_dofs_ -= num_dofs;

  // Mark the actuator as "removed".
  joint_actuators_[actuator_index] = std::nullopt;

  // Update the actuator_index_start for all joint actuators that come after
  // the one we just removed.
  for (JointActuatorIndex i(actuator_index); i < ssize(joint_actuators_); ++i) {
    if (joint_actuators_[i].has_value()) {
      (*joint_actuators_[i]).actuator_index_start -= num_dofs;
    }
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
