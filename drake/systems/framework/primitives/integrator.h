#pragma once

#include <cstdint>
#include <memory>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// An integrator for a continuous vector input.
/// @tparam T The type being integrated. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
/// @ingroup primitive_systems

template <typename T>
class Integrator : public LeafSystem<T> {
 public:
  /// Constructs an %Integrator system.
  /// @param size number of elements in the signal to be processed.
  explicit Integrator(int size);
  ~Integrator() override;

  /// Sets the value of the integral modifying the state in the context.
  /// @p value must be a column vector of the appropriate size.
  void set_integral_value(Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

  // System<T> overrides
  bool has_any_direct_feedthrough() const override;
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;
  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override;

 protected:
  // LeafSystem<T> override
  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override;
};

}  // namespace systems
}  // namespace drake
