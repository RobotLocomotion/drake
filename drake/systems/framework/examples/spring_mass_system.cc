#include "drake/systems/framework/examples/spring_mass_system.h"

#include "drake/systems/framework/basic_state_vector.h"

namespace drake {
namespace systems {

namespace {
constexpr int kStateSize = 3;  // position, velocity, power integral
}  // namespace

SpringMassStateVector::SpringMassStateVector(double initial_position,
                                             double initial_velocity)
    : BasicStateAndOutputVector<double>(kStateSize) {
  set_position(initial_position);
  set_velocity(initial_velocity);
  set_conservative_work(0);
}

SpringMassStateVector::SpringMassStateVector()
    : SpringMassStateVector(0.0, 0.0) {}

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

SpringMassSystem::SpringMassSystem(double spring_constant_N_per_m,
                                   double mass_kg, bool system_is_forced)
    : spring_constant_N_per_m_(spring_constant_N_per_m),
      mass_kg_(mass_kg),
      system_is_forced_(system_is_forced) {
  System<double>::set_name("SpringMassSystem");
  // Declare input port for forcing term.
  if(system_is_forced) {
    this->DeclareInputPort(kVectorValued, 1, kContinuousSampling);
  }
  // Output port of q, qdot, Energy.
  this->DeclareOutputPort(kVectorValued, 3, kContinuousSampling);
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
  const double power_c = EvalSpringForce(context) * get_velocity(context);
  return power_c;
}

double SpringMassSystem::EvalNonConservativePower(const MyContext&) const {
  const double power_nc = 0.;
  return power_nc;
}

std::unique_ptr<ContinuousState<double>>
SpringMassSystem::AllocateContinuousState() const {
  return std::make_unique<ContinuousState<double>>(
      std::make_unique<BasicStateVector<double>>(3), 1, 1, 1);
}

// Assign the state to the output.
void SpringMassSystem::EvalOutput(const ContextBase<double>& context,
                                  SystemOutput<double>* output) const {
  // TODO(david-german-tri): Cache the output of this function.
  this->GetMutableOutputVector(output, 0) =
      this->CopyContinuousStateVector(context);
}

// Compute the actual physics.
void SpringMassSystem::EvalTimeDerivatives(
    const ContextBase<double>& context,
    ContinuousState<double>* derivatives) const {
  DRAKE_ASSERT_VOID(CheckValidContext(context));

  auto state_vec = this->CopyContinuousStateVector(context);

  // The derivative of position is velocity.
  derivatives->get_mutable_state()->SetAtIndex(0, state_vec[1]);

  double external_force = get_input_force(context);

  // By Newton's 2nd law, the derivative of velocity (acceleration) is f/m where
  // f is the force applied to the body by the spring, and m is the mass of the
  // body.
  const double force_applied_to_body =
      EvalSpringForce(context) + external_force;
  derivatives->get_mutable_state()->SetAtIndex(1, force_applied_to_body / mass_kg_);

  // We are integrating conservative power to get the work done by conservative
  // force elements, that is, the net energy transferred between the spring and
  // the mass over time.
  derivatives->get_mutable_state()->SetAtIndex(2, EvalConservativePower(context));
}

}  // namespace systems
}  // namespace drake
