#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// LinearCar models a car operating in a single lane using a double integrator
/// with an acceleration input.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::TaylorVarXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearCar)

  /// @p x_init initial position.
  /// @p v_init initial velocity.
  explicit LinearCar(const T& x_init, const T& v_init);
  ~LinearCar() override;

  /// Returns the input port.
  const systems::InputPortDescriptor<T>& get_input_port() const;

  /// Returns the output port.
  const systems::OutputPortDescriptor<T>& get_output_port() const;

  /// Sets the continuous states in @p context to default values.
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

 protected:
  // Declares that the outputs are all algebraically isolated from the input.
  bool DoHasDirectFeedthrough(const systems::SparsityMatrix* sparsity,
                              int input_port, int output_port) const override {
    return false;
  }

 private:
  // System<T> overrides.
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  const T x_init_;
  const T v_init_;
};

}  // namespace automotive
}  // namespace drake
