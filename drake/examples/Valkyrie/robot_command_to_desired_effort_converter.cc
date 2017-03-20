#include "drake/examples/Valkyrie/robot_command_to_desired_effort_converter.h"

#include "lcmtypes/bot_core/atlas_command_t.hpp"

namespace drake {
namespace systems {

RobotCommandToDesiredEffortConverter::RobotCommandToDesiredEffortConverter(
    const std::vector<const RigidBodyActuator*>& actuators)
    : robot_command_port_index_(DeclareAbstractInputPort().get_index()),
      desired_effort_port_indices_(DeclareDesiredEffortOutputPorts(actuators)),
      name_to_actuator_(CreateNameToActuatorMap(actuators)) {
  set_name("RobotCommandToDesiredEffortConverter");
}

void RobotCommandToDesiredEffortConverter::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  using bot_core::atlas_command_t;

  const AbstractValue* input =
      EvalAbstractInput(context, robot_command_port_index_);
  const atlas_command_t& message = input->GetValue<atlas_command_t>();

  // Note: currently, all RigidBodyActuators are assumed to be one-dimensional.
  // The following code is written based on that assumption.

  // First, zero all the desired efforts to handle missing values in the
  // robot_command message. Notably, this happens on the first few ticks of a
  // simulation, before we've received a command message from the controller,
  // in which case the subscriber system returns a default-constructed message
  // (with empty std::vectors for the efforts and such).
  for (const auto& actuator_and_port_index : desired_effort_port_indices_) {
    int port_index = actuator_and_port_index.second;
    output->get_mutable_port_value(port_index)
        ->GetMutableVectorData<double>()
        ->SetAtIndex(0, 0.0);
  }

  // Copy effort information from LCM message to appropriate output port.
  for (size_t i = 0; i < message.joint_names.size(); i++) {
    const std::string& joint_name = message.joint_names[i];
    const double& effort = message.effort[i];
    const RigidBodyActuator* actuator = name_to_actuator_.at(joint_name);
    int port_index = desired_effort_port_indices_.at(actuator);
    output->get_mutable_port_value(port_index)
        ->GetMutableVectorData<double>()
        ->SetAtIndex(0, effort);
  }
}

const OutputPortDescriptor<double>&
RobotCommandToDesiredEffortConverter::desired_effort_output_port(
    const RigidBodyActuator& actuator) const {
  return get_output_port(desired_effort_port_indices_.at(&actuator));
}

std::map<const RigidBodyActuator*, int>
RobotCommandToDesiredEffortConverter::DeclareDesiredEffortOutputPorts(
    const std::vector<const RigidBodyActuator*>& actuators) {
  // Currently, all RigidBodyActuators are assumed to be one-dimensional.
  const int desired_effort_length = 1;
  std::map<const RigidBodyActuator*, int> ret;
  for (const auto& actuator : actuators) {
    ret[actuator] =
        DeclareOutputPort(kVectorValued, desired_effort_length).get_index();
  }
  return ret;
}

std::map<std::string, const RigidBodyActuator*>
RobotCommandToDesiredEffortConverter::CreateNameToActuatorMap(
    const std::vector<const RigidBodyActuator*>& actuators) {
  std::map<std::string, const RigidBodyActuator*> ret;
  for (const auto& actuator : actuators) {
    ret[actuator->name_] = actuator;
  }
  return ret;
}

}  // namespace systems
}  // namespace drake
