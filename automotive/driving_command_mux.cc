#include "drake/automotive/driving_command_mux.h"

#include <numeric>
#include <utility>

#include "drake/common/default_scalars.h"

namespace drake {
namespace automotive {

template <typename T>
DrivingCommandMux<T>::DrivingCommandMux()
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::DrivingCommandMux>{}),
      steering_port_index_(
          this->DeclareInputPort(systems::kVectorValued, 1).get_index()),
      acceleration_port_index_(
          this->DeclareInputPort(systems::kVectorValued, 1).get_index()) {
  this->DeclareVectorOutputPort(DrivingCommand<T>(),
                                &DrivingCommandMux<T>::CombineInputsToOutput);
}

template <typename T>
template <typename U>
DrivingCommandMux<T>::DrivingCommandMux(const DrivingCommandMux<U>&)
    : DrivingCommandMux<T>() {}

template <typename T>
const systems::InputPort<T>& DrivingCommandMux<T>::steering_input()
    const {
  return systems::System<T>::get_input_port(steering_port_index_);
}

template <typename T>
const systems::InputPort<T>&
DrivingCommandMux<T>::acceleration_input() const {
  return systems::System<T>::get_input_port(acceleration_port_index_);
}

template <typename T>
void DrivingCommandMux<T>::CombineInputsToOutput(
    const systems::Context<T>& context, DrivingCommand<T>* output) const {
  const auto& steering = steering_input().Eval(context);
  DRAKE_DEMAND(steering.size() == 1);
  output->set_steering_angle(steering[0]);
  const auto& acceleration = acceleration_input().Eval(context);
  DRAKE_DEMAND(acceleration.size() == 1);
  output->set_acceleration(acceleration[0]);
}

}  // namespace automotive
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::automotive::DrivingCommandMux)
