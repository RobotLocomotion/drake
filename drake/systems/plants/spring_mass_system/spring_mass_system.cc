#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

#include <utility>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

namespace {
constexpr int kStateSize = 3;  // position, velocity, power integral
}  // namespace

template <typename T>
SpringMassStateVector<T>::SpringMassStateVector(const T& initial_position,
                                                const T& initial_velocity)
    : BasicVector<T>(kStateSize) {
  set_position(initial_position);
  set_velocity(initial_velocity);
  set_conservative_work(0);
}

template <typename T>
SpringMassStateVector<T>::SpringMassStateVector()
    : SpringMassStateVector(0.0, 0.0) {}

template <typename T>
SpringMassStateVector<T>::~SpringMassStateVector() {}

// Order matters: Position (q) precedes velocity (v) precedes misc. (z) in
// ContinuousState.
template <typename T>
T SpringMassStateVector<T>::get_position() const {
  return this->GetAtIndex(0);
}
template <typename T>
T SpringMassStateVector<T>::get_velocity() const {
  return this->GetAtIndex(1);
}
template <typename T>
T SpringMassStateVector<T>::get_conservative_work() const {
  return this->GetAtIndex(2);
}
template <typename T>
void SpringMassStateVector<T>::set_position(const T& q) {
  this->SetAtIndex(0, q);
}
template <typename T>
void SpringMassStateVector<T>::set_velocity(const T& v) {
  this->SetAtIndex(1, v);
}
template <typename T>
void SpringMassStateVector<T>::set_conservative_work(const T& work) {
  this->SetAtIndex(2, work);
}

template <typename T>
SpringMassStateVector<T>* SpringMassStateVector<T>::DoClone() const {
  auto state = new SpringMassStateVector<T>(get_position(), get_velocity());
  state->set_conservative_work(get_conservative_work());
  return state;
}

template <typename T>
SpringMassSystem<T>::SpringMassSystem(const T& spring_constant_N_per_m,
                                      const T& mass_kg, bool system_is_forced)
    : spring_constant_N_per_m_(spring_constant_N_per_m),
      mass_kg_(mass_kg),
      system_is_forced_(system_is_forced) {
  // Declares input port for forcing term.
  if (system_is_forced_) this->DeclareInputPort(kVectorValued, 1);

  // Declares output port for q, qdot, Energy.
  this->DeclareVectorOutputPort(SpringMassStateVector<T>());

  this->DeclareContinuousState(
      std::make_unique<SpringMassStateVector<T>>(),
      1 /* num_q */, 1 /* num_v */, 1 /* num_z */);
}

template <typename T>
const InputPortDescriptor<T>& SpringMassSystem<T>::get_force_port() const {
  if (system_is_forced_) {
    return this->get_input_port(0);
  } else {
    throw std::runtime_error(
        "Attempting to access input force port when this SpringMassSystem was "
            "instantiated with no input ports.");
  }
}

template <typename T>
const OutputPortDescriptor<T>& SpringMassSystem<T>::get_output_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
T SpringMassSystem<T>::EvalSpringForce(const MyContext& context) const {
  const T& k = spring_constant_N_per_m_, x = get_position(context);
  T x0 = 0;  // TODO(david-german-tri) should be a parameter.
  T stretch = x - x0, f = -k * stretch;
  return f;
}

template <typename T>
T SpringMassSystem<T>::DoCalcPotentialEnergy(const MyContext& context) const {
  const T& k = spring_constant_N_per_m_, x = get_position(context),
          x0 = 0.,  // TODO(david-german-tri) should be a parameter.
     stretch = x - x0, pe = k * stretch * stretch / 2;
  return pe;
}

template <typename T>
T SpringMassSystem<T>::DoCalcKineticEnergy(const MyContext& context) const {
  const T& m = mass_kg_, v = get_velocity(context), ke = m * v * v / 2;
  return ke;
}

template <typename T>
T SpringMassSystem<T>::DoCalcConservativePower(const MyContext& context) const {
  const T& power_c = EvalSpringForce(context) * get_velocity(context);
  return power_c;
}

template <typename T>
T SpringMassSystem<T>::DoCalcNonConservativePower(const MyContext&) const {
  const T& power_nc = 0.;
  return power_nc;
}

// Assign the state to the output.
template <typename T>
void SpringMassSystem<T>::DoCalcOutput(const Context<T>& context,
                                       SystemOutput<T>* output) const {
  const SpringMassStateVector<T>& state = get_state(context);
  SpringMassStateVector<T>* output_vector = get_mutable_output(output);
  output_vector->set_position(state.get_position());
  output_vector->set_velocity(state.get_velocity());
}

// Compute the actual physics.
template <typename T>
void SpringMassSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context,
    ContinuousState<T>* derivatives) const {

  // TODO(david-german-tri): Cache the output of this function.
  const SpringMassStateVector<T>& state = get_state(context);

  SpringMassStateVector<T>* derivative_vector = get_mutable_state(derivatives);

  // The derivative of position is velocity.
  derivative_vector->set_position(state.get_velocity());

  const T external_force = get_input_force(context);

  // By Newton's 2nd law, the derivative of velocity (acceleration) is f/m where
  // f is the force applied to the body by the spring, and m is the mass of the
  // body.
  const T force_applied_to_body = EvalSpringForce(context) + external_force;
  derivative_vector->set_velocity(force_applied_to_body / mass_kg_);

  // We are integrating conservative power to get the work done by conservative
  // force elements, that is, the net energy transferred between the spring and
  // the mass over time.
  derivative_vector->set_conservative_work(
      this->CalcConservativePower(context));
}

template class SpringMassStateVector<double>;
template class SpringMassStateVector<AutoDiffXd>;
template class SpringMassSystem<double>;
template class SpringMassSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
