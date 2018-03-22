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
class RimlessWheel : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RimlessWheel);

  /// Constructs the plant.
  RimlessWheel();

  /// Return reference to the output port that publishes only [theta,
  /// thetatdot].
  const systems::OutputPort<T>& get_minimal_state_output_port() {
    return this->get_output_port(0);
  }

  /// Returns reference to the output port that provides a full 6-dof (with an
  /// FloatingBaseType::kRollPitchYaw) state.  This is useful for, e.g.,
  /// visualization.
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

 private:
  // TODO(russt): Consider generalizing this as sugar for any system class.
  class MyWitnessFunction : public systems::WitnessFunction<T> {
   public:
    typedef std::function<T(const systems::Context<T>&)> CalcWitnessCallback;

    explicit MyWitnessFunction(
        const systems::System<T>* system, CalcWitnessCallback guard,
        typename systems::UnrestrictedUpdateEvent<T>::UnrestrictedUpdateCallback
            reset)
        : systems::WitnessFunction<T>(
              system,
              systems::WitnessFunctionDirection::kPositiveThenNonPositive,
              std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
                  systems::Event<T>::TriggerType::kWitness, reset)),
          guard_(guard) {}
    ~MyWitnessFunction() override {}

   private:
    T DoCalcWitnessValue(const systems::Context<T>& context) const override {
      return guard_(context);
    }

    const CalcWitnessCallback guard_;
  };

  // Check when the foot hits the ramp (rolling downhill).
  T StepForwardGuard(const systems::Context<T>& context) const;

  // Handles the impact dynamics.
  void StepForwardReset(const systems::Context<T>& context,
                        const systems::UnrestrictedUpdateEvent<T>&,
                        systems::State<T>* state) const;

  // Check when the foot hits the ramp (rolling uphill).
  T StepBackwardGuard(const systems::Context<T>& context) const;

  // Handles the impact dynamics.
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

  // The signed distance witness function is always active and, hence, always
  // returned.
  void DoGetWitnessFunctions(const systems::Context<T>&,
                             std::vector<const systems::WitnessFunction<T>*>*
                                 witnesses) const override;

  // The system stores its witness functions internally.
  std::unique_ptr<MyWitnessFunction> step_backward_;
  std::unique_ptr<MyWitnessFunction> step_forward_;
};

}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake
