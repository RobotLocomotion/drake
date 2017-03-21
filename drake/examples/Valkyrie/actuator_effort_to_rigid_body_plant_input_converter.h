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
  using System<T>::DeclareInputPort;
  using System<T>::DeclareOutputPort;
  using System<T>::set_name;
  using System<T>::EvalVectorInput;
  using System<T>::get_input_port;

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
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

  std::vector<const RigidBodyActuator*> ordered_actuators_;
  const std::map<const RigidBodyActuator*, int> effort_ports_indices_;
  int rigid_body_plant_input_port_index_;
  std::map<const RigidBodyActuator*, int> DeclareEffortInputPorts(
      const std::vector<const RigidBodyActuator*>& vector);
};

}  // namespace systems
}  // namespace drake
