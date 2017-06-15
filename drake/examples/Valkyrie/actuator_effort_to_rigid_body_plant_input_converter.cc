#include "drake/examples/Valkyrie/actuator_effort_to_rigid_body_plant_input_converter.h"

namespace drake {
namespace systems {

template <typename T>
ActuatorEffortToRigidBodyPlantInputConverter<T>::
    ActuatorEffortToRigidBodyPlantInputConverter(
        const std::vector<const RigidBodyActuator*>& ordered_actuators)
    : ordered_actuators_(ordered_actuators),
      effort_ports_indices_(DeclareEffortInputPorts(ordered_actuators)) {
  this->DeclareVectorOutputPort(
      BasicVector<T>(static_cast<int>(ordered_actuators.size())),
      &ActuatorEffortToRigidBodyPlantInputConverter::OutputActuation);
  this->set_name("ActuatorEffortToRigidBodyPlantInputConverter");
}

template <typename T>
void ActuatorEffortToRigidBodyPlantInputConverter<T>::OutputActuation(
    const Context<T>& context, BasicVector<T>* output) const {
  int index = 0;
  for (const auto& actuator : ordered_actuators_) {
    int port_index = effort_ports_indices_.at(actuator);
    T effort = this->EvalVectorInput(context, port_index)->get_value()[0];
    output->SetAtIndex(index++, effort);
  }
}

template <typename T>
std::map<const RigidBodyActuator*, int>
ActuatorEffortToRigidBodyPlantInputConverter<T>::DeclareEffortInputPorts(
    const std::vector<const RigidBodyActuator*>& ordered_actuators) {
  std::map<const RigidBodyActuator*, int> ret;

  // Currently, all RigidBodyActuators are assumed to be one-dimensional.
  const int effort_length = 1;
  for (const auto& actuator : ordered_actuators) {
    ret[actuator] =
        this->DeclareInputPort(kVectorValued, effort_length).get_index();
  }
  return ret;
}

template <typename T>
const InputPortDescriptor<T>& ActuatorEffortToRigidBodyPlantInputConverter<
    T>::effort_input_port(const RigidBodyActuator& actuator) {
  return this->get_input_port(effort_ports_indices_.at(&actuator));
}

// Explicit instantiations.
template class ActuatorEffortToRigidBodyPlantInputConverter<double>;

}  // namespace systems
}  // namespace drake
