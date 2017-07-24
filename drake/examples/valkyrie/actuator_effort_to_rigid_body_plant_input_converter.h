#pragma once

#include <map>
#include <vector>

#include "drake/multibody/rigid_body_actuator.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template <typename T>
class ActuatorEffortToRigidBodyPlantInputConverter : public LeafSystem<T> {
 public:
  ActuatorEffortToRigidBodyPlantInputConverter(
      const std::vector<const RigidBodyActuator*>& ordered_actuators);

  ~ActuatorEffortToRigidBodyPlantInputConverter() override {}

  // Disable copy and assign.
  ActuatorEffortToRigidBodyPlantInputConverter(
      const ActuatorEffortToRigidBodyPlantInputConverter&) = delete;

  ActuatorEffortToRigidBodyPlantInputConverter& operator=(
      const ActuatorEffortToRigidBodyPlantInputConverter&) = delete;


  /// Returns the descriptor of the effort input port corresponding for
  /// @param actuator
  const InputPortDescriptor<T>& effort_input_port(
      const RigidBodyActuator& actuator);

 private:
  // This is the calculator method for the output port.
  void OutputActuation(const Context<T>& context,
                       BasicVector<T>* output) const;

  std::vector<const RigidBodyActuator*> ordered_actuators_;
  const std::map<const RigidBodyActuator*, int> effort_ports_indices_;
  std::map<const RigidBodyActuator*, int> DeclareEffortInputPorts(
      const std::vector<const RigidBodyActuator*>& vector);
};

}  // namespace systems
}  // namespace drake
