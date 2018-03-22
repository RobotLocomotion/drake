#pragma once

#include <memory>
#include <vector>

#include "drake/examples/rimless_wheel/gen/rimless_wheel_params.h"
#include "drake/examples/rimless_wheel/gen/rimless_wheel_state.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace examples {
namespace rimless_wheel {

/// Dynamical representation of the idealized hybrid dynamics of a "rimless
/// wheel", as described in
///   http://underactuated.mit.edu/underactuated.html?chapter=simple_legs
/// In addition, this model has one additional state variable to keep track
/// of the toe position along the ramp - this is updated with the step length
/// at each collision event.
///
/// Inputs: None.
/// States: Minimal state (theta and thetadot).
/// Outputs:
///   0) minimal state output
///   1) floating-base state output
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
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
  /// of the floating base, and downhill moves toward positive x.  As always,
  /// we use vehicle coordinates (x-y on the ground, z is up).
  const systems::OutputPort<T>& get_floating_base_state_output_port() {
    return this->get_output_port(1);
  }

  /// Access the RimlessWheelState.
  static const RimlessWheelState<T>& get_state(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const RimlessWheelState<T>&>(cstate.get_vector());
  }

  /// Access the RimlessWheelState.
  static const RimlessWheelState<T>& get_state(
      const systems::Context<T>& context) {
    return get_state(context.get_continuous_state());
  }

  /// Access the mutable RimlessWheelState.
  static RimlessWheelState<T>& get_mutable_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<RimlessWheelState<T>&>(cstate->get_mutable_vector());
  }

  /// Access the mutable RimlessWheelState.
  static RimlessWheelState<T>& get_mutable_state(systems::Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  /// Access the RimlessWheelParams.
  const RimlessWheelParams<T>& get_parameters(
      const systems::Context<T>& context) const {
    return this->template GetNumericParameter<RimlessWheelParams>(context, 0);
  }

  static T calc_alpha(const RimlessWheelParams<T>& params) {
    return M_PI / params.number_of_spokes();
  }

  /// Calculates the kinetic + potential energy.
  T CalcTotalEnergy(const systems::Context<T>& context) const;

 private:
  // Check when the foot hits the ramp (rolling downhill).  This occurs when
  //   θ = slope + the interleg angle (α).
  T StepForwardGuard(const systems::Context<T>& context) const;

  // Handles the impact dynamics, including reseting theta to the angle of the
  // new stance leg, and updating the toe position by the step length.
  void StepForwardReset(const systems::Context<T>& context,
                        const systems::UnrestrictedUpdateEvent<T>&,
                        systems::State<T>* state) const;

  // Check when the foot hits the ramp (rolling uphill).  This occurs when
  //   θ = slope - the interleg angle (α).
  T StepBackwardGuard(const systems::Context<T>& context) const;

  // Handles the impact dynamics, including reseting theta to the angle of the
  // new stance leg, and updating the toe position by the step length.
  void StepBackwardReset(const systems::Context<T>& context,
                         const systems::UnrestrictedUpdateEvent<T>&,
                         systems::State<T>* state) const;

  void MinimalStateOut(const systems::Context<T>& context,
                       systems::BasicVector<T>* output) const;

  void FloatingBaseStateOut(const systems::Context<T>& context,
                            systems::BasicVector<T>* output) const;

  // Implements the simple pendulum dynamics during stance.
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // The signed distance witness functions are always active and, hence, always
  // returned.
  void DoGetWitnessFunctions(const systems::Context<T>&,
                             std::vector<const systems::WitnessFunction<T>*>*
                                 witnesses) const override;

  // The system stores its witness functions internally.
  std::unique_ptr<systems::WitnessFunction<T>> step_backward_;
  std::unique_ptr<systems::WitnessFunction<T>> step_forward_;
};

}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake
