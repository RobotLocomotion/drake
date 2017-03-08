#pragma once

#include <cstdint>
#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic_expression.h"
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
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
/// @ingroup primitive_systems

template <typename T>
class Integrator : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Integrator)

  /// Constructs an %Integrator system.
  /// @param size number of elements in the signal to be processed.
  explicit Integrator(int size);
  ~Integrator() override;

  /// Sets the value of the integral modifying the state in the context.
  /// @p value must be a column vector of the appropriate size.
  void set_integral_value(Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

  // System<T> override.
  bool has_any_direct_feedthrough() const override;

  // Returns an Integrator<AutoDiffXd> with the same dimensions as this
  // Integrator.
  std::unique_ptr<Integrator<AutoDiffXd>> ToAutoDiffXd() const {
    return std::unique_ptr<Integrator<AutoDiffXd>>(DoToAutoDiffXd());
  }

 protected:
  // System<T> overrides.

  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;
  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

  // Returns an Integrator<AutoDiffXd> with the same dimensions as this
  // Integrator.
  Integrator<AutoDiffXd>* DoToAutoDiffXd() const override;
  // Returns an Integrator<symbolic::Expression> with the same dimensions as
  // this Integrator.
  Integrator<symbolic::Expression>* DoToSymbolic() const override;

  // LeafSystem<T> override
  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override;
};

}  // namespace systems
}  // namespace drake
