#include "drake/examples/rimless_wheel/rimless_wheel.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace rimless_wheel {

template <typename T>
RimlessWheel<T>::RimlessWheel()
    : systems::LeafSystem<T>(systems::SystemTypeTag<RimlessWheel>{}) {
  this->DeclareContinuousState(RimlessWheelContinuousState<T>(), 1, 1, 0);

  // Discrete state for stance toe distance along the ramp.
  this->DeclareDiscreteState(1);

  // Abstract state for indicating that we are in "double support" mode (e.g.
  // two feet/spokes are touching the ground, and angular velocity is
  // approximately zero).
  const bool double_support = false;
  this->DeclareAbstractState(Value<bool>(double_support));

  // The minimal state of the system.
  this->DeclareVectorOutputPort(systems::kUseDefaultName,
                                RimlessWheelContinuousState<T>(),
                                &RimlessWheel::MinimalStateOut);

  // The floating-base (RPY) state of the system (useful for visualization).
  this->DeclareVectorOutputPort(systems::kUseDefaultName, 12,
                                &RimlessWheel::FloatingBaseStateOut);

  this->DeclareNumericParameter(RimlessWheelParams<T>());

  // Create the witness functions.
  step_backward_ = this->MakeWitnessFunction(
      "step backward",
      systems::WitnessFunctionDirection::kPositiveThenNonPositive,
      &RimlessWheel::StepBackwardGuard, &RimlessWheel::StepBackwardReset);

  step_forward_ = this->MakeWitnessFunction(
      "step forward",
      systems::WitnessFunctionDirection::kPositiveThenNonPositive,
      &RimlessWheel::StepForwardGuard, &RimlessWheel::StepForwardReset);
}

template <typename T>
T RimlessWheel<T>::CalcTotalEnergy(const systems::Context<T>& context) const {
  using std::pow;
  const RimlessWheelContinuousState<T>& rw_state =
      get_continuous_state(context);
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
  const RimlessWheelContinuousState<T>& rw_state =
      get_continuous_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);

  // Triggers when θ = slope + α.
  return params.slope() + calc_alpha(params) - rw_state.theta();
}

template <typename T>
void RimlessWheel<T>::StepForwardReset(
    const systems::Context<T>& context,
    const systems::UnrestrictedUpdateEvent<T>&,
    systems::State<T>* state) const {
  const RimlessWheelContinuousState<T>& rw_state =
      get_continuous_state(context);
  RimlessWheelContinuousState<T>& next_state =
      get_mutable_continuous_state(&(state->get_mutable_continuous_state()));
  const RimlessWheelParams<T>& params = get_parameters(context);
  T& toe = get_mutable_toe_position(state);
  const T alpha = calc_alpha(params);

  // Stance foot changes to the new support leg.
  next_state.set_theta(rw_state.theta() - 2.*alpha);
  // Event isolation guarantees that the penetration was positive (though
  // small).  The post-impact state needs to be above the ground, so that the
  // witness function evaluation that occurs immediately after the impact has
  // a positive value (we have declared the WitnessFunction with
  // kPositiveThenNonPositive).  The failure mode before this improved
  // implementation was the robot falling through the ground after an impact.
  DRAKE_DEMAND(next_state.theta() > params.slope() - alpha);

  // θ̇ ⇐ θ̇ cos(2α)
  next_state.set_thetadot(rw_state.thetadot() * cos(2. * alpha));

  // toe += stance length.
  toe += 2. * params.length() * sin(alpha);

  // If thetadot is very small, then transition to double support (this is
  // only for efficiency, to avoid simulation at the Zeno).
  // Note: I already know that thetadot > 0 since the guard triggered.
  DRAKE_ASSERT(next_state.thetadot() >= 0.);
  // The threshold value below only impacts early termination.  Setting it
  // closer to zero will not cause the wheel to miss an event, but will cause
  // the simulator to perform arbitrarily more event detection calculations.
  // The threshold is multiplied by sqrt(g/l) to make it dimensionless.
  if (next_state.thetadot() < 0.01*sqrt(params.gravity()/params.length())) {
    bool& double_support = get_mutable_double_support(state);
    double_support = true;
    next_state.set_thetadot(0.0);
  }
}

template <typename T>
T RimlessWheel<T>::StepBackwardGuard(const systems::Context<T>& context) const {
  const RimlessWheelContinuousState<T>& rw_state =
      get_continuous_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);

  // Triggers when θ = slope - α.
  return rw_state.theta() - params.slope() + calc_alpha(params);
}

template <typename T>
void RimlessWheel<T>::StepBackwardReset(
    const systems::Context<T>& context,
    const systems::UnrestrictedUpdateEvent<T>&,
    systems::State<T>* state) const {
  const RimlessWheelContinuousState<T>& rw_state =
      get_continuous_state(context);
  RimlessWheelContinuousState<T>& next_state =
      get_mutable_continuous_state(&(state->get_mutable_continuous_state()));
  const RimlessWheelParams<T>& params = get_parameters(context);
  T& toe = get_mutable_toe_position(state);
  const T alpha = calc_alpha(params);

  // Stance foot changes to the new support leg.
  next_state.set_theta(rw_state.theta() + 2.*alpha);
  // Event isolation guarantees that the penetration was positive (though
  // small).  The post-impact state needs to be above the ground, so that the
  // witness function evaluation that occurs immediately after the impact has
  // a positive value (we have declared the WitnessFunction with
  // kPositiveThenNonPositive).  The failure mode before this improved
  // implementation was the robot falling through the ground after an impact.
  DRAKE_DEMAND(next_state.theta() < params.slope() + alpha);

  // θ̇ ⇐ θ̇ cos(2α)
  next_state.set_thetadot(rw_state.thetadot() * cos(2. * alpha));

  // toe -= stance length.
  toe -= 2. * params.length() * sin(alpha);

  // If thetadot is very small, then transition to double support.
  // Note: I already know that thetadot < 0 since the guard triggered.
  DRAKE_ASSERT(next_state.thetadot() <= 0.);
  // The threshold value below only impacts early termination.  Setting it
  // closer to zero will not cause the wheel to miss an event, but will cause
  // the simulator to perform arbitrarily more event detection calculations.
  // The threshold is multiplied by sqrt(g/l) to make it dimensionless.
  if (next_state.thetadot() > -0.01*sqrt(params.gravity()/params.length())) {
    bool& double_support = get_mutable_double_support(state);
    double_support = true;
    next_state.set_thetadot(0.0);
  }
}

template <typename T>
void RimlessWheel<T>::MinimalStateOut(
    const systems::Context<T>& context,
    RimlessWheelContinuousState<T>* output) const {
  output->SetFromVector(get_continuous_state(context).CopyToVector());
}

template <typename T>
void RimlessWheel<T>::FloatingBaseStateOut(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  const RimlessWheelContinuousState<T>& rw_state =
      get_continuous_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);
  const T toe = get_toe_position(context);

  // x, y, z.
  output->SetAtIndex(
      0, toe * cos(params.slope()) + params.length() * sin(rw_state.theta()));
  output->SetAtIndex(1, 0.);
  output->SetAtIndex(
      2, -toe * sin(params.slope()) + params.length() * cos(rw_state.theta()));

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

template <typename T>
void RimlessWheel<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const RimlessWheelContinuousState<T>& rw_state =
      get_continuous_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);
  RimlessWheelContinuousState<T>& rw_derivatives =
      get_mutable_continuous_state(derivatives);
  const bool double_support = get_double_support(context);

  // Handle double-support (to avoid simulating at the Zeno fixed point).
  if (double_support) {
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
    const systems::Context<T>& context,
    std::vector<const systems::WitnessFunction<T>*>* witnesses) const {
  const bool double_support = get_double_support(context);

  if (!double_support) {
    witnesses->push_back(step_backward_.get());
    witnesses->push_back(step_forward_.get());
  }
}

}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::rimless_wheel::RimlessWheel)
