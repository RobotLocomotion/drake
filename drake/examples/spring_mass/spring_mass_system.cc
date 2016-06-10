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
const ptrdiff_t kStateSize = 2;
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

SpringMassSystem::SpringMassSystem(const std::string& name,
                                   double spring_constant_N_per_m,
                                   double mass_kg)
    : name_(name),
      spring_constant_N_per_m_(spring_constant_N_per_m),
      mass_kg_(mass_kg) {}

double SpringMassSystem::GetSpringForce(const MyContext& context) const {
  const double k = spring_constant_N_per_m_,
               x = get_position(context),
               x0 = 0.,  // TODO(david-german-tri) should be a parameter.
               stretch = x - x0, f = -k * stretch;
  return f;
}

double SpringMassSystem::GetPotentialEnergy(const MyContext& context) const {
  const double k = spring_constant_N_per_m_,
               x = get_position(context),
               x0 = 0.,  // TODO(david-german-tri) should be a parameter.
               stretch = x - x0, pe = k * stretch * stretch / 2;
  return pe;
}

double SpringMassSystem::GetKineticEnergy(const MyContext& context) const {
  const double m = mass_kg_,
               v = get_velocity(context),
               ke = m * v * v / 2;
  return ke;
}

// TODO(sherm1) Make this more interesting.
double SpringMassSystem::GetPower(const MyContext&) const {
  const double power = 0.;
  return power;
}

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

std::unique_ptr<ContinuousState<double>> SpringMassSystem::AllocateDerivatives()
    const {
  std::unique_ptr<SpringMassStateVector> derivs(
      new SpringMassStateVector(0, 0));
  return std::unique_ptr<ContinuousState<double>>(
      new ContinuousState<double>(std::move(derivs), 1 /* size of q */,
                                  1 /* size of v */, 0 /* size of z */));
}

// Assign the state to the output.
void SpringMassSystem::GetOutput(const Context<double>& context,
                                 SystemOutput<double>* output) const {
  // TODO(david-german-tri): Cache the output of this function.
  const SpringMassStateVector& state = get_state(context);

  SpringMassOutputVector* output_vector = get_mutable_output(output);
  output_vector->set_position(state.get_position());
  output_vector->set_velocity(state.get_velocity());
}

// Compute the actual physics.
void SpringMassSystem::GetDerivatives(
    const Context<double>& context,
    ContinuousState<double>* derivatives) const {
  // TODO(david-german-tri): Cache the output of this function.
  const SpringMassStateVector& state = get_state(context);

  SpringMassStateVector* derivative_vector = get_mutable_state(derivatives);

  // The derivative of position is velocity.
  derivative_vector->set_position(state.get_velocity());

  // By Newton's 2nd law, the derivative of velocity (acceleration) is f/m where
  // f is the force applied to the body by the spring, and m is the mass of the
  // body.
  const double force_applied_to_body = GetSpringForce(context);
  derivative_vector->set_velocity(force_applied_to_body / mass_kg_);
}

}  // namespace examples
}  // namespace drake
