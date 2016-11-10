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
/// Input: linear acceleration of the ego car (scalar) [m/s^2].
/// Outputs: position (scalar) [m], velocity (scalar)
/// [m/s].
template <typename T>
class LinearCar : public systems::LeafSystem<T> {
 public:
  /// @param x_init initial position.
  /// @param v_init initial velocity.
  explicit LinearCar(const T& x_init, const T& v_init);
  ~LinearCar() override;

  /// Declare that the outputs are all algebraically isolated from the input.
  bool has_any_direct_feedthrough() const override { return false; }

  /// Returns the input port.
  const systems::SystemPortDescriptor<T>& get_input_port() const;

  /// Returns the output port.
  const systems::SystemPortDescriptor<T>& get_output_port() const;

  // System<T> overrides.
  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

 protected:
  // LeafSystem<T> overrides.
  std::unique_ptr<systems::ContinuousState<T>> AllocateContinuousState()
      const override;

 private:
  const T x_init_;
  const T v_init_;

  // Disable copy and assignment.
  LinearCar(const LinearCar<T>&) = delete;
  LinearCar& operator=(const LinearCar<T>&) = delete;
  LinearCar(LinearCar<T>&&) = delete;
  LinearCar& operator=(LinearCar<T>&&) = delete;
};

}  // namespace automotive
}  // namespace drake
