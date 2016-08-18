#pragma once

#include <cstdint>
#include <memory>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// An integrator for a continuous vector input.
/// @tparam T The type being integrated. Must be a valid Eigen scalar.
template <typename T>
class Integrator : public LeafSystem<T> {
 public:
  /// @param length is the size of the input port.
  explicit Integrator(int length);
  ~Integrator() override {}

  /// Integrator's output is not a direct feedthrough of its input.
  // TODO(amcastro-tri): we should be able to express that initial conditions
  // feed through an integrator but the dynamic signal during simulation
  // does not.
  bool has_any_direct_feedthrough() const override { return false;}

  /// Integrates the input ports into the output port. If the input ports are
  /// not of the length specified in the constructor, std::runtime_error will
  /// be thrown.
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override;

  void EvalTimeDerivatives(const ContextBase<T>& context,
                           ContinuousState<T>* derivatives) const override;

 private:
  void ReserveState(Context<T>* context) const override;
};

}  // namespace systems
}  // namespace drake
