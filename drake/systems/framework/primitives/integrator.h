#pragma once

#include <cstdint>
#include <memory>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// An integrator for a continuous vector input.
/// @tparam T The type being integrated. Must be a valid Eigen scalar.
template <typename T>
class Integrator : public LeafSystem<T> {
 public:
  /// @param length is the size of the input port.
  explicit Integrator(int length);
  ~Integrator() override;

 public:
  // System<T> overrides
  bool has_any_direct_feedthrough() const override;
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;
  void EvalTimeDerivatives(const ContextBase<T>& context,
                           ContinuousState<T>* derivatives) const override;

 protected:
  // LeafSystem<T> override
  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override;
};

}  // namespace systems
}  // namespace drake
