#include "drake/examples/pendulum/pendulum_plant.h"

#include <cmath>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace examples {
namespace pendulum {

template <typename T>
PendulumPlant<T>::PendulumPlant()
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<pendulum::PendulumPlant>{}) {
  this->DeclareVectorInputPort(PendulumInput<T>());
  this->DeclareVectorOutputPort(PendulumState<T>(),
                                &PendulumPlant::CopyStateOut);
  this->DeclareContinuousState(PendulumState<T>(), 1 /* num_q */, 1 /* num_v */,
                               0 /* num_z */);
  this->DeclareNumericParameter(PendulumParams<T>());
}

template <typename T>
template <typename U>
PendulumPlant<T>::PendulumPlant(const PendulumPlant<U>&) : PendulumPlant() {}

template <typename T>
PendulumPlant<T>::~PendulumPlant() {}

template <typename T>
const systems::InputPortDescriptor<T>& PendulumPlant<T>::get_tau_port() const {
  return this->get_input_port(0);
}

template <typename T>
const systems::OutputPort<T>& PendulumPlant<T>::get_output_port() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
void PendulumPlant<T>::CopyStateOut(const systems::Context<T>& context,
                                    PendulumState<T>* output) const {
  output->set_value(get_state(context).get_value());
}

template <typename T>
T PendulumPlant<T>::CalcTotalEnergy(const systems::Context<T>& context) const {
  using std::pow;
  const PendulumState<T>& state = get_state(context);
  const PendulumParams<T>& params =
      this->template GetNumericParameter<PendulumParams>(context, 0);
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
  const PendulumParams<T>& params =
      this->template GetNumericParameter<PendulumParams>(context, 0);
  PendulumState<T>* derivative_vector = get_mutable_state(derivatives);

  derivative_vector->set_theta(state.thetadot());
  using std::pow;
  derivative_vector->set_thetadot(
      (get_tau(context) -
       params.mass() * params.gravity() * params.length() * sin(state.theta()) -
       params.damping() * state.thetadot()) /
      (params.mass() * pow(params.length(), 2)));
}

template class PendulumPlant<double>;
template class PendulumPlant<AutoDiffXd>;
template class PendulumPlant<symbolic::Expression>;

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
