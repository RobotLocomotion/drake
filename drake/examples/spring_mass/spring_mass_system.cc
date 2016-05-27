#include "drake/examples/spring_mass/spring_mass_system.h"

namespace drake {

using systems::BasicVector;
using systems::Context;
using systems::ContinuousState;
using systems::OutputPort;
using systems::StateVectorInterface;
using systems::SystemOutput;
using systems::VectorInterface;
using systems::VectorX;

namespace examples {

namespace {
const size_t kStateSize = 2;
}  // namespace

SpringMassStateVector::SpringMassStateVector(double initial_position,
                                             double initial_velocity)
    : BasicVector(kStateSize) {
  // Order matters: Position (q) precedes velocity (v).
  get_mutable_value()[0] = initial_position;
  get_mutable_value()[1] = initial_velocity;
}

SpringMassStateVector::~SpringMassStateVector() {}

size_t SpringMassStateVector::size() const { return 2; }

const double SpringMassStateVector::GetAtIndex(size_t index) const {
  return get_value()[index];
}

void SpringMassStateVector::SetAtIndex(size_t index, const double& value) {
  get_mutable_value()[index] = value;
}

void SpringMassStateVector::SetFromVector(const VectorX<double>& value) {
  set_value(value);
}

double SpringMassStateVector::get_position() const { return get_value()[0]; }
double SpringMassStateVector::get_velocity() const { return get_value()[1]; }
void SpringMassStateVector::set_position(double q) { SetAtIndex(0, q); }
void SpringMassStateVector::set_velocity(double v) { SetAtIndex(1, v); }

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
                                  1 /* size of v */));
  return context;
}

std::unique_ptr<SystemOutput<double>> SpringMassSystem::CreateDefaultOutput()
    const {
  std::unique_ptr<SystemOutput<double>> output(new SystemOutput<double>);
  {
    OutputPort<double> port;
    port.vector_output.reset(new SpringMassStateVector(0, 0));
    output->ports.push_back(std::move(port));
  }
  return output;
}

std::unique_ptr<StateVectorInterface<double>>
SpringMassSystem::AllocateStateDerivatives() const {
  return std::unique_ptr<StateVectorInterface<double>>(
      new SpringMassStateVector(0, 0));
}

// Assign the state to the output.
void SpringMassSystem::Output(const Context<double>& context,
                              SystemOutput<double>* output) const {
  /// TODO(david-german-tri): Add a cast that is dynamic_cast in Debug mode,
  /// and static_cast in Release mode.
  const SpringMassStateVector& state =
      dynamic_cast<const SpringMassStateVector&>(
          context.get_state().continuous_state->get_state());
  SpringMassStateVector* output_vector = dynamic_cast<SpringMassStateVector*>(
      output->ports[0].vector_output.get());
  output_vector->set_value(state.get_value());
}

// Compute the actual physics.
void SpringMassSystem::Dynamics(
    const Context<double>& context,
    StateVectorInterface<double>* derivatives) const {
  const SpringMassStateVector& state =
      dynamic_cast<const SpringMassStateVector&>(
          context.get_state().continuous_state->get_state());
  SpringMassStateVector* output_vector =
      dynamic_cast<SpringMassStateVector*>(derivatives);

  // The derivative of position is velocity.
  output_vector->set_position(state.get_velocity());

  // The derivative of velocity is spring force divided by mass.
  const double spring_force = -spring_constant_N_per_m_ * state.get_position();
  output_vector->set_velocity(spring_force / mass_kg_);
}

void SpringMassSystem::MapVelocityToConfigurationDerivatives(
    const Context<double>& context,
    StateVectorInterface<double>* derivatives) const {
  const SpringMassStateVector& state =
      dynamic_cast<const SpringMassStateVector&>(
          context.get_state().continuous_state->get_state());
  SpringMassStateVector* output_vector =
      dynamic_cast<SpringMassStateVector*>(derivatives);
  output_vector->get_mutable_value() = VectorX<double>::Zero(kStateSize);
  // The derivative of configuration is velocity.
  output_vector->set_position(state.get_velocity());
}

}  // namespace examples
}  // namespace drake
