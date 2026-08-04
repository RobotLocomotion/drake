#pragma once

#include "drake/examples/pendulum/pendulum_input.h"
#include "drake/examples/pendulum/pendulum_params.h"
#include "drake/examples/pendulum/pendulum_state.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace pendulum {

/// A model of a simple pendulum
/// @f[ ml^2 \ddot\theta + b\dot\theta + mgl\sin\theta = \tau @f]
///
/// @system
/// name: PendulumPlant
/// input_ports:
/// - tau (optional)
/// output_ports:
/// - state
/// @endsystem
///
/// Note: If the tau input port is not connected, then the torque is
/// taken to be zero.
///
/// @tparam_default_scalar
template <typename T>
class PendulumPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PendulumPlant);

  /// Constructs a default plant.
  PendulumPlant();

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit PendulumPlant(const PendulumPlant<U>&);

  ~PendulumPlant() final;

  /// Returns the port to output state.
  const systems::OutputPort<T>& get_state_output_port() const;

  /// Calculates the kinetic + potential energy.
  T CalcTotalEnergy(const systems::Context<T>& context) const;

  /// Evaluates the input port and returns the scalar value of the commanded
  /// torque. If the input port is not connected, then the torque is taken to
  /// be zero.
  T get_tau(const systems::Context<T>& context) const {
    const systems::BasicVector<T>* u_vec = this->EvalVectorInput(context, 0);
    return u_vec ? u_vec->GetAtIndex(0) : 0.0;
  }

  static const PendulumState<T>& get_state(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const PendulumState<T>&>(cstate.get_vector());
  }

  static const PendulumState<T>& get_state(const systems::Context<T>& context) {
    return get_state(context.get_continuous_state());
  }

  static PendulumState<T>& get_mutable_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<PendulumState<T>&>(cstate->get_mutable_vector());
  }

  static PendulumState<T>& get_mutable_state(systems::Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  const PendulumParams<T>& get_parameters(
      const systems::Context<T>& context) const {
    return this->template GetNumericParameter<PendulumParams>(context, 0);
  }

  PendulumParams<T>& get_mutable_parameters(
      systems::Context<T>* context) const {
    return this->template GetMutableNumericParameter<PendulumParams>(context,
                                                                     0);
  }

 private:
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const final;

  T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

  T DoCalcKineticEnergy(const systems::Context<T>& context) const override;
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
