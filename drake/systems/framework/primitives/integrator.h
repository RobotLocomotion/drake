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

  void ReserveState(Context<T>* context) const override;

  /// Integrates the input ports into the output port. If the input ports are
  /// not of the length specified in the constructor, std::runtime_error will
  /// be thrown.
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override;

  void EvalTimeDerivatives(const ContextBase<T>& context,
                           ContinuousState<T>* derivatives) const override;

 private:
  int length_{0};
};

}  // namespace systems
}  // namespace drake
