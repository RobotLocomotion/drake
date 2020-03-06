#pragma once

#include <memory>
#include <vector>

#include "drake/examples/rimless_wheel/gen/rimless_wheel_continuous_state.h"
#include "drake/examples/rimless_wheel/gen/rimless_wheel_params.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace examples {
namespace rimless_wheel {

/// Dynamical representation of the idealized hybrid dynamics of a "rimless
/// wheel", as described in
///   http://underactuated.mit.edu/underactuated.html?chapter=simple_legs
/// In addition, this model has two additional (discrete) state variables that
/// are not required in the mathematical model:
///
/// - the position of the stance toe along the ramp (helpful for outputting
///   a floating-base model coordinate, e.g. for visualization),
/// - a boolean indicator for "double support" (to avoid the numerical
///   challenges of simulation around the Zeno phenomenon at the standing
///   fixed point).
///
/// @system{RimlessWheel, ,
///   @output_port{minimal_state (theta and thetadot only)}
///   @output_port{floating_base_state}
/// }
///
/// Continuous States: theta, thetadot.
/// Discrete States: stance toe position, double support indicator.
/// Parameters: mass, length, number of spokes, etc, are all set as Context
///   parameters using RimlessWheelParams.
///
/// @tparam_nonsymbolic_scalar
template <typename T>
class RimlessWheel final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RimlessWheel);

  /// Constructs the plant.
  RimlessWheel();

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit RimlessWheel(const RimlessWheel<U>&) : RimlessWheel<T>() {}

  /// Return reference to the output port that publishes only [theta,
  /// thetatdot].
  const systems::OutputPort<T>& get_minimal_state_output_port() {
    return this->get_output_port(0);
  }

  /// Returns reference to the output port that provides a 12 dimensional state
  /// (FloatingBaseType::kRollPitchYaw positions then velocities).  This is
  /// useful, e.g., for visualization.  θ of the rimless wheel is the pitch
  /// of the floating base (rotation around global y), and downhill moves toward
  /// positive x.  As always, we use vehicle coordinates (x-y on the ground, z
  /// is up).
  const systems::OutputPort<T>& get_floating_base_state_output_port() {
    return this->get_output_port(1);
  }

  /// Access the RimlessWheelContinuousState.
  static const RimlessWheelContinuousState<T>& get_continuous_state(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const RimlessWheelContinuousState<T>&>(
        cstate.get_vector());
  }

  /// Access the RimlessWheelContinuousState.
  static const RimlessWheelContinuousState<T>& get_continuous_state(
      const systems::Context<T>& context) {
    return get_continuous_state(context.get_continuous_state());
  }

  /// Access the mutable RimlessWheelContinuousState.
  static RimlessWheelContinuousState<T>& get_mutable_continuous_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<RimlessWheelContinuousState<T>&>(
        cstate->get_mutable_vector());
  }

  /// Access the mutable RimlessWheelContinuousState.
  static RimlessWheelContinuousState<T>& get_mutable_continuous_state(
      systems::Context<T>* context) {
    return get_mutable_continuous_state(
        &context->get_mutable_continuous_state());
  }

  static const T& get_toe_position(const systems::Context<T>& context) {
    return context.get_discrete_state(0).GetAtIndex(0);
  }

  static T& get_mutable_toe_position(systems::State<T>* state) {
    return state->get_mutable_discrete_state().get_mutable_vector(0).GetAtIndex(
        0);
  }

  static bool get_double_support(const systems::Context<T>& context) {
    return context.template get_abstract_state<bool>(0);
  }

  static bool& get_mutable_double_support(systems::State<T>* state) {
    return state->template get_mutable_abstract_state<bool>(0);
  }

  /// Access the RimlessWheelParams.
  const RimlessWheelParams<T>& get_parameters(
      const systems::Context<T>& context) const {
    return this->template GetNumericParameter<RimlessWheelParams>(context, 0);
  }

  /// Alpha is half the interleg angle, and is used frequently.
  static T calc_alpha(const RimlessWheelParams<T>& params) {
    return M_PI / params.number_of_spokes();
  }

  /// Calculates the kinetic + potential energy.
  T CalcTotalEnergy(const systems::Context<T>& context) const;

 private:
  // Check when the foot hits the ramp (rolling downhill).  This occurs when
  //   θ = slope + the interleg angle (α).
  T StepForwardGuard(const systems::Context<T>& context) const;

  // Handles the impact dynamics, including resetting theta to the angle of the
  // new stance leg, and updating the toe position by the step length.
  void StepForwardReset(const systems::Context<T>& context,
                        const systems::UnrestrictedUpdateEvent<T>&,
                        systems::State<T>* state) const;

  // Check when the foot hits the ramp (rolling uphill).  This occurs when
  //   θ = slope - the interleg angle (α).
  T StepBackwardGuard(const systems::Context<T>& context) const;

  // Handles the impact dynamics, including resetting theta to the angle of the
  // new stance leg, and updating the toe position by the step length.
  void StepBackwardReset(const systems::Context<T>& context,
                         const systems::UnrestrictedUpdateEvent<T>&,
                         systems::State<T>* state) const;

  void MinimalStateOut(const systems::Context<T>& context,
                       RimlessWheelContinuousState<T>* output) const;

  void FloatingBaseStateOut(const systems::Context<T>& context,
                            systems::BasicVector<T>* output) const;

  // Implements the simple pendulum dynamics during stance.
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void DoGetWitnessFunctions(const systems::Context<T>&,
                             std::vector<const systems::WitnessFunction<T>*>*
                                 witnesses) const override;

  // The system stores its witness functions internally.
  std::unique_ptr<systems::WitnessFunction<T>> step_backward_;
  std::unique_ptr<systems::WitnessFunction<T>> step_forward_;
};

}  // namespace rimless_wheel
}  // namespace examples

// Explicitly disable symbolic::Expression (for now).
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<examples::rimless_wheel::RimlessWheel>
    : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake
