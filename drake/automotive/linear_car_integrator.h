#pragma once
 
#include <cstdint>
#include <memory>

#include "drake/automotive/gen/linear_car_input.h"
#include "drake/automotive/gen/linear_car_state.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

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
  /// Constructs an %LinearCar system.
  /// @param size number of elements in the signal to be processed.
  explicit LinearCar();
  ~LinearCar() override;

  /// Sets the value of the state in the context.
  /// @p value must be a column vector of the appropriate size.
  void set_states(systems::Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

  // System<T> overrides
  bool has_any_direct_feedthrough() const override;

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

 protected:
  // LeafSystem<T> overrides
  std::unique_ptr<systems::ContinuousState<T>> AllocateContinuousState()
    const override;
};

}  // namespace automotive
}  // namespace drake
