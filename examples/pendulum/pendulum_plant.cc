#include "drake/examples/pendulum/pendulum_plant.h"

#include <cmath>

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace pendulum {

template <typename T>
PendulumPlant<T>::PendulumPlant()
    : systems::LeafSystem<T>(systems::SystemTypeTag<PendulumPlant>{}) {
  this->DeclareNumericParameter(PendulumParams<T>());
  this->DeclareVectorInputPort("tau", PendulumInput<T>());
  auto state_index = this->DeclareContinuousState(
      PendulumState<T>(), 1 /* num_q */, 1 /* num_v */, 0 /* num_z */);
  this->DeclareStateOutputPort("state", state_index);
}

template <typename T>
template <typename U>
PendulumPlant<T>::PendulumPlant(const PendulumPlant<U>&) : PendulumPlant() {}

template <typename T>
PendulumPlant<T>::~PendulumPlant() = default;

template <typename T>
const systems::OutputPort<T>& PendulumPlant<T>::get_state_output_port() const {
  DRAKE_DEMAND(systems::LeafSystem<T>::num_output_ports() == 1);
  return systems::LeafSystem<T>::get_output_port(0);
}

template <typename T>
T PendulumPlant<T>::CalcTotalEnergy(const systems::Context<T>& context) const {
  return DoCalcPotentialEnergy(context) + DoCalcKineticEnergy(context);
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

template <typename T>
T PendulumPlant<T>::DoCalcPotentialEnergy(const systems::Context<T>& context)
const {
  using std::cos;
  const PendulumState<T>& state = get_state(context);
  const PendulumParams<T>& params = get_parameters(context);
  // Potential energy = -mgl cos θ.
  return -params.mass() * params.gravity() * params.length() *
         cos(state.theta());
}

template <typename T>
T PendulumPlant<T>::DoCalcKineticEnergy(const systems::Context<T>& context)
const {
  using std::pow;
  const PendulumState<T>& state = get_state(context);
  const PendulumParams<T>& params = get_parameters(context);
  // Kinetic energy = 1/2 m l² θ̇ ².
  return 0.5 * params.mass() * pow(params.length() * state.thetadot(), 2);
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::pendulum::PendulumPlant)
