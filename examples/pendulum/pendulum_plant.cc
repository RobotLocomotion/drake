#include "drake/examples/pendulum/pendulum_plant.h"

#include <cmath>

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace pendulum {

template <typename T>
PendulumPlant<T>::PendulumPlant()
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<pendulum::PendulumPlant>{}) {
  this->DeclareVectorInputPort("tau", PendulumInput<T>());
  this->DeclareVectorOutputPort("state", &PendulumPlant::CopyStateOut,
                                {this->all_state_ticket()});
  this->DeclareContinuousState(PendulumState<T>(), 1 /* num_q */, 1 /* num_v */,
                               0 /* num_z */);
  this->DeclareNumericParameter(PendulumParams<T>());
}

template <typename T>
template <typename U>
PendulumPlant<T>::PendulumPlant(const PendulumPlant<U>&) : PendulumPlant() {}

template <typename T>
PendulumPlant<T>::~PendulumPlant() = default;

template <typename T>
const systems::InputPort<T>& PendulumPlant<T>::get_input_port() const {
  DRAKE_DEMAND(systems::LeafSystem<T>::num_input_ports() == 1);
  return systems::LeafSystem<T>::get_input_port(0);
}

template <typename T>
const systems::OutputPort<T>& PendulumPlant<T>::get_state_output_port() const {
  DRAKE_DEMAND(systems::LeafSystem<T>::num_output_ports() == 1);
  return systems::LeafSystem<T>::get_output_port(0);
}

template <typename T>
void PendulumPlant<T>::CopyStateOut(const systems::Context<T>& context,
                                    PendulumState<T>* output) const {
  *output = get_state(context);
}

template <typename T>
T PendulumPlant<T>::CalcTotalEnergy(const systems::Context<T>& context) const {
  using std::pow;
  const PendulumState<T>& state = get_state(context);
  const PendulumParams<T>& params = get_parameters(context);
  // Kinetic energy = 1/2 m l² θ̇ ².
  const T kinetic_energy =
      0.5 * params.mass() * pow(params.length() * state.thetadot(), 2);
  // Potential energy = -mgl cos θ.
  const T potential_energy =
      -params.mass() * params.gravity() * params.length() * cos(state.theta());
  return kinetic_energy + potential_energy;
}

// Compute the actual physics.
template <typename T>
void PendulumPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const PendulumState<T>& state = get_state(context);
  const PendulumParams<T>& params = get_parameters(context);
  PendulumState<T>& derivative_vector = get_mutable_state(derivatives);

  derivative_vector.set_theta(state.thetadot());
  derivative_vector.set_thetadot(
      (get_tau(context) -
       params.mass() * params.gravity() * params.length() * sin(state.theta()) -
       params.damping() * state.thetadot()) /
      (params.mass() * params.length() * params.length()));
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::pendulum::PendulumPlant)
