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
constexpr int kStateSize = 3;  // position, velocity, power integral
}  // namespace

SpringMassStateVector::SpringMassStateVector(double initial_position,
                                             double initial_velocity)
    : BasicStateVector<double>(kStateSize) {
  set_position(initial_position);
  set_velocity(initial_velocity);
  set_conservative_work(0);
}

SpringMassStateVector::~SpringMassStateVector() {}

// Order matters: Position (q) precedes velocity (v) precedes misc. (z) in
// ContinuousState.
double SpringMassStateVector::get_position() const { return GetAtIndex(0); }
double SpringMassStateVector::get_velocity() const { return GetAtIndex(1); }
double SpringMassStateVector::get_conservative_work() const {
  return GetAtIndex(2);
}
void SpringMassStateVector::set_position(double q) { SetAtIndex(0, q); }
void SpringMassStateVector::set_velocity(double v) { SetAtIndex(1, v); }
void SpringMassStateVector::set_conservative_work(double work) {
  SetAtIndex(2, work);
}

SpringMassStateVector* SpringMassStateVector::DoClone() const {
  auto state = new SpringMassStateVector(get_position(), get_velocity());
  state->set_conservative_work(get_conservative_work());
  return state;
}

SpringMassOutputVector::SpringMassOutputVector()
    : BasicVector<double>(kStateSize-1) {}  // don't output conservative energy

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

double SpringMassSystem::EvalSpringForce(const MyContext& context) const {
  const double k = spring_constant_N_per_m_,
               x = get_position(context),
               x0 = 0.,  // TODO(david-german-tri) should be a parameter.
               stretch = x - x0, f = -k * stretch;
  return f;
}

double SpringMassSystem::EvalPotentialEnergy(const MyContext& context) const {
  const double k = spring_constant_N_per_m_,
               x = get_position(context),
               x0 = 0.,  // TODO(david-german-tri) should be a parameter.
               stretch = x - x0, pe = k * stretch * stretch / 2;
  return pe;
}

double SpringMassSystem::EvalKineticEnergy(const MyContext& context) const {
  const double m = mass_kg_,
               v = get_velocity(context),
               ke = m * v * v / 2;
  return ke;
}

double SpringMassSystem::EvalConservativePower(const MyContext& context) const {
  const double power_c = EvalSpringForce(context) * get_velocity(context);
  return power_c;
}

double SpringMassSystem::EvalNonConservativePower(const MyContext&) const {
  const double power_nc = 0.;
  return power_nc;
}

std::string SpringMassSystem::get_name() const { return name_; }

// Reserve a context with no input, and a SpringMassStateVector state.
std::unique_ptr<Context<double>> SpringMassSystem::CreateDefaultContext()
    const {
  std::unique_ptr<Context<double>> context(new Context<double>);
  std::unique_ptr<SpringMassStateVector> state(new SpringMassStateVector(0, 0));
  context->get_mutable_state()->continuous_state.reset(
      new ContinuousState<double>(std::move(state), 1 /* size of q */,
                                  1 /* size of v */, 1 /* size of z */));
  return context;
}

std::unique_ptr<SystemOutput<double>> SpringMassSystem::AllocateOutput() const {
  std::unique_ptr<SystemOutput<double>> output(new SystemOutput<double>);
  {
    std::unique_ptr<VectorInterface<double>> data(new SpringMassOutputVector());
    std::unique_ptr<OutputPort<double>> port(
        new OutputPort<double>(std::move(data)));
    output->ports.push_back(std::move(port));
  }
  return output;
}

std::unique_ptr<ContinuousState<double>>
SpringMassSystem::AllocateTimeDerivatives() const {
  std::unique_ptr<SpringMassStateVector> derivs(
      new SpringMassStateVector(0, 0));
  return std::unique_ptr<ContinuousState<double>>(
      new ContinuousState<double>(std::move(derivs), 1 /* size of q */,
                                  1 /* size of v */, 1 /* size of z */));
}

// Assign the state to the output.
void SpringMassSystem::EvalOutput(const Context<double>& context,
                                  SystemOutput<double>* output) const {
  // TODO(david-german-tri): Cache the output of this function.
  const SpringMassStateVector& state = get_state(context);
  SpringMassOutputVector* output_vector = get_mutable_output(output);
  output_vector->set_position(state.get_position());
  output_vector->set_velocity(state.get_velocity());
}

// Compute the actual physics.
void SpringMassSystem::EvalTimeDerivatives(
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
  const double force_applied_to_body = EvalSpringForce(context);
  derivative_vector->set_velocity(force_applied_to_body / mass_kg_);

  // We are integrating conservative power to get the work done by conservative
  // force elements, that is, the net energy transferred between the spring and
  // the mass over time.
  derivative_vector->set_conservative_work(EvalConservativePower(context));
}

}  // namespace examples
}  // namespace drake
