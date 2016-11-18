#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// LinearCar -- model a car operating in a single lane using a double
/// integrator with an acceleration input.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::TaylorVarXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in libdrakeAutomotive.
///
/// @ingroup automotive_systems
///
/// Inputs:
///   0: @p vdot_ego linear acceleration of the ego car (scalar) [m/s^2].
/// Outputs:
///   0: @p x position (scalar) [m]
///   1: @p v velocity (scalar) [m/s].
template <typename T>
class LinearCar : public systems::LeafSystem<T> {
 public:
  /// @p x_init initial position.
  /// @p v_init initial velocity.
  explicit LinearCar(const T& x_init, const T& v_init);
  ~LinearCar() override;

  /// Returns the input port.
  const systems::SystemPortDescriptor<T>& get_input_port() const;

  /// Returns the output port.
  const systems::SystemPortDescriptor<T>& get_output_port() const;

  /// Sets the continuous states in @p context to default values.
  void SetDefaultState(systems::Context<T>* context) const;

  // System<T> overrides.
  // Declare that the outputs are all algebraically isolated from the input.
  bool has_any_direct_feedthrough() const override { return false; }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // Disable copy and assignment.
  LinearCar(const LinearCar<T>&) = delete;
  LinearCar& operator=(const LinearCar<T>&) = delete;
  LinearCar(LinearCar<T>&&) = delete;
  LinearCar& operator=(LinearCar<T>&&) = delete;

 private:
  const T x_init_;
  const T v_init_;
};

}  // namespace automotive
}  // namespace drake
