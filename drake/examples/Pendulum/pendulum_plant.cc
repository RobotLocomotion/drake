#include "drake/examples/Pendulum/pendulum_plant.h"

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace examples {
namespace pendulum {

template <typename T>
PendulumPlant<T>::PendulumPlant() {
  this->DeclareInputPort(systems::kVectorValued, 1);
  this->DeclareVectorOutputPort(PendulumStateVector<T>());
  this->DeclareContinuousState(
      PendulumStateVector<T>(),
      1 /* num_q */, 1 /* num_v */, 0 /* num_z */);
  static_assert(PendulumStateVectorIndices::kNumCoordinates == 1 + 1, "");
}

template <typename T>
PendulumPlant<T>::~PendulumPlant() {}

template <typename T>
const systems::InputPortDescriptor<T>&
PendulumPlant<T>::get_tau_port() const {
  return this->get_input_port(0);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
PendulumPlant<T>::get_output_port() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
void PendulumPlant<T>::DoCalcOutput(const systems::Context<T>& context,
                                    systems::SystemOutput<T>* output) const {
  get_mutable_output(output)->set_value(get_state(context).get_value());
}

// Compute the actual physics.
template <typename T>
void PendulumPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const PendulumStateVector<T>& state = get_state(context);
  PendulumStateVector<T>* derivative_vector = get_mutable_state(derivatives);

  derivative_vector->set_theta(state.thetadot());
  // Pendulum formula from Section 2.2 of Russ Tedrake. Underactuated
  // Robotics: Algorithms for Walking, Running, Swimming, Flying, and
  // Manipulation (Course Notes for MIT 6.832). Downloaded on
  // 2016-09-30 from
  // http://underactuated.csail.mit.edu/underactuated.html?chapter=2
  derivative_vector->set_thetadot(
      (get_tau(context) - m_ * g_ * lc_ * sin(state.theta()) -
       b_ * state.thetadot()) / I_);
}

// PendulumPlant has no constructor arguments, so there's no work to do here.
template <typename T>
PendulumPlant<AutoDiffXd>* PendulumPlant<T>::DoToAutoDiffXd() const {
  return new PendulumPlant<AutoDiffXd>();
}

template class PendulumPlant<double>;
template class PendulumPlant<AutoDiffXd>;

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
