#pragma once

//#include "drake/automotive/gen/linear_car_input.h"
//#include "drake/automotive/gen/linear_car_state.h"
#include "drake/systems/framework/leaf_system.h"
//#include "drake/systems/framework/system_output.h"

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
template <typename T>
class LinearCar : public systems::LeafSystem<T> {
 public:
  LinearCar();
  ~LinearCar() override;

  /// Declare that the outputs are all algebraically isolated from the input.
  bool has_any_direct_feedthrough() const override { return false; }

  /// Returns the input port.
  const systems::SystemPortDescriptor<T>& get_input_port() const;

  /// Returns the output port.
  const systems::SystemPortDescriptor<T>& get_output_port() const;

  // System<T> overrides
  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

 protected:
  // LeafSystem<T> overrides
  std::unique_ptr<systems::ContinuousState<T>> AllocateContinuousState()
      const override;

  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;
};

}  // namespace automotive
}  // namespace drake
