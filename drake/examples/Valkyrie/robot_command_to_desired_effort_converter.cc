#include "drake/examples/Valkyrie/robot_command_to_desired_effort_converter.h"

#include <algorithm>

#include "lcmtypes/bot_core/atlas_command_t.hpp"

namespace drake {
namespace systems {

RobotCommandToDesiredEffortConverter::RobotCommandToDesiredEffortConverter(
    const std::vector<const RigidBodyActuator*>& actuators)
    : robot_command_port_index_(DeclareAbstractInputPort().get_index()),
      desired_effort_port_indices_(DeclareDesiredEffortOutputPorts(actuators)) {
  set_name("RobotCommandToDesiredEffortConverter");
}

void RobotCommandToDesiredEffortConverter::OutputDesiredEffort(
    const systems::Context<double>& context,
    const RigidBodyActuator& actuator,
    systems::BasicVector<double>* output) const {
  using bot_core::atlas_command_t;

  // TODO(sherm1) Should use a cache entry to hold a joint name-to-index map
  // if each message can have unique joint ordering.
  const atlas_command_t* message =
      EvalInputValue<atlas_command_t>(context, robot_command_port_index_);
  DRAKE_DEMAND(message != nullptr);

  // Note: currently, all RigidBodyActuators are assumed to be one-dimensional.
  // The following code is written based on that assumption.

  // First, zero all the desired efforts to handle missing values in the
  // robot_command message. Notably, this happens on the first few ticks of a
  // simulation, before we've received a command message from the controller,
  // in which case the subscriber system returns a default-constructed message
  // (with empty std::vectors for the efforts and such).
  output->SetAtIndex(0, 0.0);

  // See if the name associated with this actuator is in the message.
  // TODO(sherm1) O(n) search repeated here for each port; see above.
  const auto& names = message->joint_names;
  auto entry = std::find(names.begin(), names.end(), actuator.name_);
  if (entry == names.end())
    return;  // Nothing for this port.

  const int index = static_cast<int>(entry - names.begin());
  const double& effort = message->effort[index];
  output->SetAtIndex(0, effort);
}

const OutputPort<double>&
RobotCommandToDesiredEffortConverter::desired_effort_output_port(
    const RigidBodyActuator& actuator) const {
  return get_output_port(desired_effort_port_indices_.at(&actuator));
}

std::map<const RigidBodyActuator*, OutputPortIndex>
RobotCommandToDesiredEffortConverter::DeclareDesiredEffortOutputPorts(
    const std::vector<const RigidBodyActuator*>& actuators) {
  // Currently, all RigidBodyActuators are assumed to be one-dimensional.
  const int desired_effort_length = 1;
  std::map<const RigidBodyActuator*, OutputPortIndex> ret;
  for (const RigidBodyActuator* actuator : actuators) {
    // Create a function with the signature of a vector output port calculator.
    auto calc_method = [this, actuator](const Context<double>& context,
                                        BasicVector<double>* output) {
      this->OutputDesiredEffort(context, *actuator, output);
    };
    ret[actuator] = DeclareVectorOutputPort(
                        BasicVector<double>(desired_effort_length), calc_method)
                        .get_index();
  }
  return ret;
}

}  // namespace systems
}  // namespace drake
