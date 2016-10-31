#include "drake/examples/Pendulum/pendulum_plant.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace examples {
namespace pendulum {

namespace {
constexpr int kStateSize = 2;  // position, velocity
}

template <typename T>
PendulumPlant<T>::PendulumPlant() {
  this->DeclareInputPort(
      systems::kVectorValued, 1, systems::kContinuousSampling);
  this->DeclareOutputPort(
      systems::kVectorValued, kStateSize, systems::kContinuousSampling);
}

template <typename T>
PendulumPlant<T>::~PendulumPlant() {}

template <typename T>
const systems::SystemPortDescriptor<T>&
PendulumPlant<T>::get_tau_port() const {
  return this->get_input_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
PendulumPlant<T>::get_output_port() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
PendulumPlant<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  DRAKE_THROW_UNLESS(descriptor.get_size() == kStateSize);
  return std::make_unique<PendulumStateVector<T>>();
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
PendulumPlant<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<PendulumStateVector<T>>(),
      1 /* num_q */, 1 /* num_v */, 0 /* num_z */);
  static_assert(kStateSize == 1 + 1, "State size has changed");
}

template <typename T>
void PendulumPlant<T>::EvalOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  get_mutable_output(output)->set_value(get_state(context).get_value());
}

// Compute the actual physics.
template <typename T>
void PendulumPlant<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

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

template class DRAKE_EXPORT PendulumPlant<double>;
template class DRAKE_EXPORT PendulumPlant<AutoDiffXd>;

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
