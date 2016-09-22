#include "drake/systems/framework/examples/spring_mass_system.h"

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
const T& SpringMassStateVector<T>::get_position() const { return GetAtIndex(0); }
template <typename T>
const T& SpringMassStateVector<T>::get_velocity() const { return GetAtIndex(1); }
template <typename T>
const T& SpringMassStateVector<T>::get_conservative_work() const {
  return GetAtIndex(2);
}
template <typename T>
void SpringMassStateVector<T>::set_position(const T& q) { SetAtIndex(0, q); }
template <typename T>
void SpringMassStateVector<T>::set_velocity(const T& v) { SetAtIndex(1, v); }
template <typename T>
void SpringMassStateVector<T>::set_conservative_work(const T& work) {
  SetAtIndex(2, work);
}

template <typename T>
SpringMassStateVector<T>* SpringMassStateVector<T>::DoClone() const {
  auto state = new SpringMassStateVector<T>(get_position(), get_velocity());
  state->set_conservative_work(get_conservative_work());
  return state;
}

SpringMassSystem::SpringMassSystem(double spring_constant_N_per_m,
                                   double mass_kg, bool system_is_forced)
    : spring_constant_N_per_m_(spring_constant_N_per_m),
      mass_kg_(mass_kg),
      system_is_forced_(system_is_forced) {
  // Declares input port for forcing term.
  if (system_is_forced_)
    DeclareInputPort(kVectorValued, 1, kContinuousSampling);

  // Declares output port for q, qdot, Energy.
  DeclareOutputPort(kVectorValued, 3, kContinuousSampling);
}

const SystemPortDescriptor<double>& SpringMassSystem::get_force_port() const {
  if (system_is_forced_) {
    return get_input_port(0);
  } else {
    throw std::runtime_error(
        "Attempting to access input force port when this SpringMassSystem was "
            "instantiated with no input ports.");
  }
}

const SystemPortDescriptor<double>& SpringMassSystem::get_output_port() const {
  return System<double>::get_output_port(0);
}

double SpringMassSystem::EvalSpringForce(const MyContext& context) const {
  const double k = spring_constant_N_per_m_, x = get_position(context),
               x0 = 0.,  // TODO(david-german-tri) should be a parameter.
      stretch = x - x0, f = -k * stretch;
  return f;
}

double SpringMassSystem::EvalPotentialEnergy(const MyContext& context) const {
  const double k = spring_constant_N_per_m_, x = get_position(context),
               x0 = 0.,  // TODO(david-german-tri) should be a parameter.
      stretch = x - x0, pe = k * stretch * stretch / 2;
  return pe;
}

double SpringMassSystem::EvalKineticEnergy(const MyContext& context) const {
  const double m = mass_kg_, v = get_velocity(context), ke = m * v * v / 2;
  return ke;
}

double SpringMassSystem::EvalConservativePower(const MyContext& context) const {
  DRAKE_ASSERT_VOID(CheckValidContext(context));
  const double power_c = EvalSpringForce(context) * get_velocity(context);
  return power_c;
}

double SpringMassSystem::EvalNonConservativePower(const MyContext&) const {
  const double power_nc = 0.;
  return power_nc;
}

std::unique_ptr<SystemOutput<double>> SpringMassSystem::AllocateOutput(
    const Context<double>& context) const {
  std::unique_ptr<LeafSystemOutput<double>> output(
      new LeafSystemOutput<double>);
  {
    std::unique_ptr<BasicVector<double>> data(new SpringMassStateVector());
    std::unique_ptr<OutputPort> port(new OutputPort(std::move(data)));
    output->get_mutable_ports()->push_back(std::move(port));
  }
  return std::unique_ptr<SystemOutput<double>>(output.release());
}

std::unique_ptr<ContinuousState<double>>
SpringMassSystem::AllocateContinuousState() const {
  return std::make_unique<ContinuousState<double>>(
      std::make_unique<SpringMassStateVector>(),
      1 /* num_q */, 1 /* num_v */, 1 /* num_z */);
}

// Assign the state to the output.
void SpringMassSystem::EvalOutput(const Context<double>& context,
                                  SystemOutput<double>* output) const {
  // TODO(david-german-tri): Cache the output of this function.
  const SpringMassStateVector& state = get_state(context);
  SpringMassStateVector* output_vector = get_mutable_output(output);
  output_vector->set_position(state.get_position());
  output_vector->set_velocity(state.get_velocity());
}

// Compute the actual physics.
void SpringMassSystem::EvalTimeDerivatives(
    const Context<double>& context,
    ContinuousState<double>* derivatives) const {
  DRAKE_ASSERT_VOID(CheckValidContext(context));

  // TODO(david-german-tri): Cache the output of this function.
  const SpringMassStateVector& state = get_state(context);

  SpringMassStateVector* derivative_vector = get_mutable_state(derivatives);

  // The derivative of position is velocity.
  derivative_vector->set_position(state.get_velocity());

  double external_force = get_input_force(context);

  // By Newton's 2nd law, the derivative of velocity (acceleration) is f/m where
  // f is the force applied to the body by the spring, and m is the mass of the
  // body.
  const double force_applied_to_body =
      EvalSpringForce(context) + external_force;
  derivative_vector->set_velocity(force_applied_to_body / mass_kg_);

  // We are integrating conservative power to get the work done by conservative
  // force elements, that is, the net energy transferred between the spring and
  // the mass over time.
  derivative_vector->set_conservative_work(EvalConservativePower(context));
}

}  // namespace systems
}  // namespace drake
