#include "drake/examples/spring_mass/spring_mass_system.h"

namespace drake {

using systems::BasicVector;
using systems::Context;
using systems::ContinuousState;
using systems::OutputPort;
using systems::StateVector;
using systems::SystemOutput;
using systems::VectorInterface;

namespace examples {

namespace {
constexpr int64_t kStateSize = 2;
}  // namespace

SpringMassStateVector::SpringMassStateVector(double initial_position,
                                             double initial_velocity)
    : BasicStateVector<double>(kStateSize) {
  set_position(initial_position);
  set_velocity(initial_velocity);
}

SpringMassStateVector::~SpringMassStateVector() {}

// Order matters: Position (q) precedes velocity (v) in ContinuousState.
double SpringMassStateVector::get_position() const { return GetAtIndex(0); }
double SpringMassStateVector::get_velocity() const { return GetAtIndex(1); }
void SpringMassStateVector::set_position(double q) { SetAtIndex(0, q); }
void SpringMassStateVector::set_velocity(double v) { SetAtIndex(1, v); }

SpringMassStateVector* SpringMassStateVector::DoClone() const {
  return new SpringMassStateVector(get_position(), get_velocity());
}

SpringMassOutputVector::SpringMassOutputVector()
    : BasicVector<double>(kStateSize) {}

SpringMassOutputVector::~SpringMassOutputVector() {}

double SpringMassOutputVector::get_position() const { return get_value()[0]; }
double SpringMassOutputVector::get_velocity() const { return get_value()[1]; }

void SpringMassOutputVector::set_position(double q) {
  get_mutable_value()[0] = q;
}

void SpringMassOutputVector::set_velocity(double v) {
  get_mutable_value()[1] = v;
}

SpringMassOutputVector* SpringMassOutputVector::DoClone() const {
  SpringMassOutputVector* clone(new SpringMassOutputVector());
  clone->get_mutable_value() = get_value();
  return clone;
}

SpringMassSystem::SpringMassSystem(const std::string& name,
                                   double spring_constant_N_per_m,
                                   double mass_kg)
    : name_(name),
      spring_constant_N_per_m_(spring_constant_N_per_m),
      mass_kg_(mass_kg) {}

SpringMassSystem::~SpringMassSystem() {}

std::string SpringMassSystem::get_name() const { return name_; }

// Reserve a context with no input, and a SpringMassStateVector state.
std::unique_ptr<Context<double>> SpringMassSystem::CreateDefaultContext()
    const {
  std::unique_ptr<Context<double>> context(new Context<double>);
  std::unique_ptr<SpringMassStateVector> state(new SpringMassStateVector(0, 0));
  context->get_mutable_state()->continuous_state.reset(
      new ContinuousState<double>(std::move(state), 1 /* size of q */,
                                  1 /* size of v */, 0 /* size of z */));
  return context;
}

std::unique_ptr<SystemOutput<double>> SpringMassSystem::AllocateOutput() const {
  std::unique_ptr<SystemOutput<double>> output(new SystemOutput<double>);
  {
    OutputPort<double> port;
    port.vector_output.reset(new SpringMassOutputVector());
    output->ports.push_back(std::move(port));
  }
  return output;
}

std::unique_ptr<StateVector<double>>
SpringMassSystem::AllocateStateDerivatives() const {
  return std::unique_ptr<StateVector<double>>(new SpringMassStateVector(0, 0));
}

// Assign the state to the output.
void SpringMassSystem::Output(const Context<double>& context,
                              SystemOutput<double>* output) const {
  // TODO(david-german-tri): Add a cast that is dynamic_cast in Debug mode,
  // and static_cast in Release mode.
  // TODO(david-german-tri): Cache the output of this function.
  const SpringMassStateVector& state =
      dynamic_cast<const SpringMassStateVector&>(
          context.get_state().continuous_state->get_state());
  SpringMassOutputVector* output_vector = dynamic_cast<SpringMassOutputVector*>(
      output->ports[0].vector_output.get());
  output_vector->set_position(state.get_position());
  output_vector->set_velocity(state.get_velocity());
}

// Compute the actual physics.
void SpringMassSystem::Dynamics(const Context<double>& context,
                                StateVector<double>* derivatives) const {
  // TODO(david-german-tri): Cache the output of this function.
  const SpringMassStateVector& state =
      dynamic_cast<const SpringMassStateVector&>(
          context.get_state().continuous_state->get_state());
  SpringMassStateVector* derivative_vector =
      dynamic_cast<SpringMassStateVector*>(derivatives);

  // The derivative of position is velocity.
  derivative_vector->set_position(state.get_velocity());

  // The derivative of velocity is spring force divided by mass.
  const double spring_force = -spring_constant_N_per_m_ * state.get_position();
  derivative_vector->set_velocity(spring_force / mass_kg_);
}

}  // namespace examples
}  // namespace drake
