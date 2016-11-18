#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/multibody/rigid_body_actuator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

// TODO(tkoolen): Currently just takes the effort part of the
// atlas_command_t message, without considering the PID control parts.
// This functionality exists in
// https://github.com/openhumanoids/valkyrie_translator/blob/7068e846c9595b0b518755692db63b5f75a10520/src/LCM2ROSControl.cpp
// and should probably be put in a library exported by Drake and used both here
// and in LCM2ROSControl.

/**
 * Converts an atlas_command_t message into desired efforts, presented on one
 * output port per actuator.
 *
 * Note that a RobotCommandToDesiredEffortConverter simply ignores commands for
 * actuators that it doesn't know about.
 */
class RobotCommandToDesiredEffortConverter
    : public LeafSystem<double> {
 public:
  RobotCommandToDesiredEffortConverter(
      const std::vector<const RigidBodyActuator*>& actuators);

  ~RobotCommandToDesiredEffortConverter() override {}

  // Disable copy and assign.
  RobotCommandToDesiredEffortConverter(
      const RobotCommandToDesiredEffortConverter&) = delete;

  RobotCommandToDesiredEffortConverter& operator=(
      const RobotCommandToDesiredEffortConverter&) = delete;

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override;

  /// Descriptor of output port that presents desired effort for @param
  /// actuator.
  const SystemPortDescriptor<double>& desired_effort_output_port(
      const RigidBodyActuator& actuator) const;

 private:
  int robot_command_port_index_;
  const std::map<const RigidBodyActuator*, int> desired_effort_port_indices_;
  const std::map<std::string, const RigidBodyActuator*> name_to_actuator_;

  // Declare one output port for each RigidBodyActuator and store their
  // descriptors in a map.
  std::map<const RigidBodyActuator*, int> DeclareDesiredEffortOutputPorts(
      const std::vector<const RigidBodyActuator*>& actuators);

  // Map from actuator name to actuator.
  std::map<std::string, const RigidBodyActuator*> CreateNameToActuatorMap(
      const std::vector<const RigidBodyActuator*>& actuators);
};

}  // namespace systems
}  // namespace drake
