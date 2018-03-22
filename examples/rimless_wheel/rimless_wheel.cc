#include "drake/examples/rimless_wheel/rimless_wheel.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace rimless_wheel {

template <typename T>
RimlessWheel<T>::RimlessWheel() {
  // Two state variables: q and v.
  this->DeclareContinuousState(RimlessWheelState<T>(), 1, 1, 1);

  // The minimal state of the system.
  this->DeclareVectorOutputPort(systems::BasicVector<T>(2),
                                &RimlessWheel::MinimalStateOut);

  // The floating-base (RPY) state of the system (useful for visualization).
  this->DeclareVectorOutputPort(systems::BasicVector<T>(12),
                                &RimlessWheel::FloatingBaseStateOut);

  this->DeclareNumericParameter(RimlessWheelParams<T>());

  // Create the witness functions.
  // TODO(russt): Hide the lambdas.
  step_backward_ = std::make_unique<MyWitnessFunction>(
      this,
      [this](const systems::Context<T>& c) {
        return this->StepBackwardGuard(c);
      },
      [this](const systems::Context<T>& c,
             const systems::UnrestrictedUpdateEvent<T>& e,
             systems::State<T>* s) {
        return this->StepBackwardReset(c, e, s);
      });
  step_backward_->set_name("step backward");
  step_forward_ = std::make_unique<MyWitnessFunction>(
      this,
      [this](const systems::Context<T>& c) {
        return this->StepForwardGuard(c);
      },
      [this](const systems::Context<T>& c,
             const systems::UnrestrictedUpdateEvent<T>& e,
             systems::State<T>* s) { return this->StepForwardReset(c, e, s); });

  step_forward_->set_name("step forward");
}

template <typename T>
T RimlessWheel<T>::StepForwardGuard(const systems::Context<T>& context) const {
  const RimlessWheelState<T>& rwstate = get_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);
  const T alpha = M_PI / params.number_of_spokes();

  return rwstate.theta() - params.slope() + alpha;
}

template <typename T>
void RimlessWheel<T>::StepForwardReset(
    const systems::Context<T>& context,
    const systems::UnrestrictedUpdateEvent<T>&,
    systems::State<T>* state) const {
  const RimlessWheelState<T>& rwstate = get_state(context);
  RimlessWheelState<T>& next_state =
      get_mutable_state(&(state->get_mutable_continuous_state()));
  const RimlessWheelParams<T>& params = get_parameters(context);
  const T alpha = M_PI / params.number_of_spokes();

  next_state.set_theta(params.slope() + alpha);
  next_state.set_thetadot(rwstate.thetadot() * cos(2. * alpha));
  next_state.set_toe(rwstate.toe() - 2. * params.length() * sin(alpha));
}

template <typename T>
T RimlessWheel<T>::StepBackwardGuard(const systems::Context<T>& context) const {
  const RimlessWheelState<T>& rwstate = get_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);
  const T alpha = M_PI / params.number_of_spokes();

  return params.slope() + alpha - rwstate.theta();
}

template <typename T>
void RimlessWheel<T>::StepBackwardReset(
    const systems::Context<T>& context,
    const systems::UnrestrictedUpdateEvent<T>&,
    systems::State<T>* state) const {
  const RimlessWheelState<T>& rwstate = get_state(context);
  RimlessWheelState<T>& next_state =
      get_mutable_state(&(state->get_mutable_continuous_state()));
  const RimlessWheelParams<T>& params = get_parameters(context);
  const T alpha = M_PI / params.number_of_spokes();

  next_state.set_theta(params.slope() - alpha);
  next_state.set_thetadot(rwstate.thetadot() * cos(2. * alpha));
  next_state.set_toe(rwstate.toe() + 2. * params.length() * sin(alpha));
}

template <typename T>
void RimlessWheel<T>::MinimalStateOut(const systems::Context<T>& context,
                                      systems::BasicVector<T>* output) const {
  const RimlessWheelState<T>& rwstate = get_state(context);

  output->SetAtIndex(0, rwstate.theta());
  output->SetAtIndex(1, rwstate.thetadot());
}

template <typename T>
void RimlessWheel<T>::FloatingBaseStateOut(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  const RimlessWheelState<T>& rwstate = get_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);

  // x, y, z.
  output->SetAtIndex(0, rwstate.toe() * cos(params.slope()) +
                            params.length() * sin(rwstate.theta()));
  output->SetAtIndex(1, 0.);
  output->SetAtIndex(2, -rwstate.toe() * sin(params.slope()) +
                            params.length() * cos(rwstate.theta()));

  // roll, pitch, yaw.
  output->SetAtIndex(3, 0.);
  output->SetAtIndex(4, rwstate.theta());
  output->SetAtIndex(5, 0.);

  // x, y, z derivatives.
  output->SetAtIndex(
      6, -rwstate.thetadot() * params.length() * cos(rwstate.theta()));
  output->SetAtIndex(7, 0.);
  output->SetAtIndex(
      8, rwstate.thetadot() * params.length() * sin(rwstate.theta()));

  // roll, pitch, yaw derivatives.
  output->SetAtIndex(9, 0.);
  output->SetAtIndex(10, rwstate.thetadot());
  output->SetAtIndex(11, 0.);
}

template <typename T>
void RimlessWheel<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const RimlessWheelState<T>& rwstate = get_state(context);
  const RimlessWheelParams<T>& params = get_parameters(context);
  RimlessWheelState<T>& rwderivatives = get_mutable_state(derivatives);

  rwderivatives.set_theta(rwstate.thetadot());
  rwderivatives.set_thetadot(sin(rwstate.theta()) * params.gravity() /
                             params.length());
  rwderivatives.set_toe(0.);
}

template <typename T>
void RimlessWheel<T>::DoGetWitnessFunctions(
    const systems::Context<T>&,
    std::vector<const systems::WitnessFunction<T>*>* witnesses) const {
  // The signed distance witness function is always active and, hence, always
  // returned.
  witnesses->push_back(step_backward_.get());
  witnesses->push_back(step_forward_.get());
}

}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::rimless_wheel::RimlessWheel)
