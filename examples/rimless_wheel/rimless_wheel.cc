#include "drake/examples/rimless_wheel/rimless_wheel.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace rimless_wheel {

template <typename T>
RimlessWheel<T>::RimlessWheel()
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<examples::rimless_wheel::RimlessWheel>{}) {
  this->DeclareContinuousState(RimlessWheelState<T>(), 1, 1, 1);

  // The minimal state of the system.
  this->DeclareVectorOutputPort(systems::BasicVector<T>(2),
                                &RimlessWheel::MinimalStateOut);

  // The floating-base (RPY) state of the system (useful for visualization).
  this->DeclareVectorOutputPort(systems::BasicVector<T>(12),
                                &RimlessWheel::FloatingBaseStateOut);

  this->DeclareNumericParameter(RimlessWheelParams<T>());

  // Create the witness functions.
  // TODO(edrumwri): Accept MySystem::* functions (to avoid the need for the
  // lambda).
  systems::UnrestrictedUpdateEvent<T> backward_event(
      systems::Event<T>::TriggerType::kWitness,
      [this](const systems::Context<T>& c,
             const systems::UnrestrictedUpdateEvent<T>& e,
             systems::State<T>* s) {
        return this->StepBackwardReset(c, e, s);
      });
  step_backward_ = this->DeclareWitnessFunction(
      "step backward",
      systems::WitnessFunctionDirection::kPositiveThenNonPositive,
      &RimlessWheel::StepBackwardGuard, backward_event);

  systems::UnrestrictedUpdateEvent<T> forward_event(
      systems::Event<T>::TriggerType::kWitness,
      [this](const systems::Context<T>& c,
             const systems::UnrestrictedUpdateEvent<T>& e,
             systems::State<T>* s) { return this->StepForwardReset(c, e, s); });
  step_forward_ = this->DeclareWitnessFunction(
      "step forward",
      systems::WitnessFunctionDirection::kPositiveThenNonPositive,
      &RimlessWheel::StepForwardGuard, forward_event);
}

template <typename T>
T RimlessWheel<T>::CalcTotalEnergy(const systems::Context<T>& context) const {
  using std::pow;
  const RimlessWheelState<T>& rw_state = get_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);
  // Kinetic energy = 1/2 m l² θ̇ ².
  const T kinetic_energy =
      0.5 * params.mass() * pow(params.length() * rw_state.thetadot(), 2);
  // Potential energy = mgl cos θ.
  const T potential_energy = params.mass() * params.gravity() *
                             params.length() * cos(rw_state.theta());
  return kinetic_energy + potential_energy;
}

template <typename T>
T RimlessWheel<T>::StepForwardGuard(const systems::Context<T>& context) const {
  const RimlessWheelState<T>& rw_state = get_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);

  // Triggers when θ = slope + α.
  return params.slope() + calc_alpha(params) - rw_state.theta();
}

template <typename T>
void RimlessWheel<T>::StepForwardReset(
    const systems::Context<T>& context,
    const systems::UnrestrictedUpdateEvent<T>&,
    systems::State<T>* state) const {
  const RimlessWheelState<T>& rw_state = get_state(context);
  RimlessWheelState<T>& next_state =
      get_mutable_state(&(state->get_mutable_continuous_state()));
  const RimlessWheelParams<T>& params = get_parameters(context);
  const T alpha = calc_alpha(params);

  // Stance foot changes to the new support leg.
  // θ ⇐ slope - α.
  next_state.set_theta(params.slope() - alpha);

  // θ̇ ⇐ θ̇ cos(2α)
  next_state.set_thetadot(rw_state.thetadot() * cos(2. * alpha));

  // toe += stance length.
  next_state.set_toe(rw_state.toe() + 2. * params.length() * sin(alpha));
}

template <typename T>
T RimlessWheel<T>::StepBackwardGuard(const systems::Context<T>& context) const {
  const RimlessWheelState<T>& rw_state = get_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);

  // Triggers when θ = slope - α.
  return rw_state.theta() - params.slope() + calc_alpha(params);
}

template <typename T>
void RimlessWheel<T>::StepBackwardReset(
    const systems::Context<T>& context,
    const systems::UnrestrictedUpdateEvent<T>&,
    systems::State<T>* state) const {
  const RimlessWheelState<T>& rw_state = get_state(context);
  RimlessWheelState<T>& next_state =
      get_mutable_state(&(state->get_mutable_continuous_state()));
  const RimlessWheelParams<T>& params = get_parameters(context);
  const T alpha = calc_alpha(params);

  // Stance foot changes to the new support leg.
  // θ ⇐ slope + α.
  next_state.set_theta(params.slope() + alpha);

  // θ̇ ⇐ θ̇ cos(2α)
  next_state.set_thetadot(rw_state.thetadot() * cos(2. * alpha));

  // toe -= stance length.
  next_state.set_toe(rw_state.toe() - 2. * params.length() * sin(alpha));
}

template <typename T>
void RimlessWheel<T>::MinimalStateOut(const systems::Context<T>& context,
                                      systems::BasicVector<T>* output) const {
  const RimlessWheelState<T>& rw_state = get_state(context);

  output->SetAtIndex(0, rw_state.theta());
  output->SetAtIndex(1, rw_state.thetadot());
}

template <typename T>
void RimlessWheel<T>::FloatingBaseStateOut(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  const RimlessWheelState<T>& rw_state = get_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);

  // x, y, z.
  output->SetAtIndex(0, rw_state.toe() * cos(params.slope()) +
                            params.length() * sin(rw_state.theta()));
  output->SetAtIndex(1, 0.);
  output->SetAtIndex(2, -rw_state.toe() * sin(params.slope()) +
                            params.length() * cos(rw_state.theta()));

  // roll, pitch, yaw.
  output->SetAtIndex(3, 0.);
  output->SetAtIndex(4, rw_state.theta());
  output->SetAtIndex(5, 0.);

  // x, y, z derivatives.
  output->SetAtIndex(
      6, -rw_state.thetadot() * params.length() * cos(rw_state.theta()));
  output->SetAtIndex(7, 0.);
  output->SetAtIndex(
      8, rw_state.thetadot() * params.length() * sin(rw_state.theta()));

  // roll, pitch, yaw derivatives.
  output->SetAtIndex(9, 0.);
  output->SetAtIndex(10, rw_state.thetadot());
  output->SetAtIndex(11, 0.);
}

namespace {

template <typename T>
bool isDoubleSupport(const RimlessWheel<T>& rw,
                     const RimlessWheelState<T>& rw_state,
                     const RimlessWheelParams<T>& params,
                     const double theta_tol, const double thetadot_tol) {
  const T alpha = rw.calc_alpha(params);

  return ((rw_state.theta() > params.slope() + alpha - theta_tol) &&
          (rw_state.thetadot() >= 0.0) &&
          (rw_state.thetadot() < thetadot_tol)) ||
         ((rw_state.theta() < params.slope() - alpha + theta_tol) &&
          (rw_state.thetadot() <= 0.0) &&
          (rw_state.thetadot() > -thetadot_tol));
}

}  // namespace

template <typename T>
void RimlessWheel<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const RimlessWheelState<T>& rw_state = get_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);
  RimlessWheelState<T>& rw_derivatives = get_mutable_state(derivatives);

  rw_derivatives.set_toe(0.0);

  // Handle double-support (to avoid simulating at the Zeno fixed point).
  const double theta_tol = 1e-4, thetadot_tol = 1e-2;
  if (isDoubleSupport(*this, rw_state, params, theta_tol, thetadot_tol)) {
    // Then I'm *almost* at the fixed point and approaching it.  Time to stop.
    rw_derivatives.set_theta(0.0);
    rw_derivatives.set_thetadot(0.0);
  } else {
    rw_derivatives.set_theta(rw_state.thetadot());
    rw_derivatives.set_thetadot(sin(rw_state.theta()) * params.gravity() /
                                params.length());
  }
}

template <typename T>
void RimlessWheel<T>::DoGetWitnessFunctions(
    const systems::Context<T>&,
    std::vector<const systems::WitnessFunction<T>*>* witnesses) const {
  // The signed distance witness functions are always active and, hence, always
  // returned.
  witnesses->push_back(step_backward_.get());
  witnesses->push_back(step_forward_.get());
}

}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::rimless_wheel::RimlessWheel)
