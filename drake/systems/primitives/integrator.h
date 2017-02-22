#pragma once

#include <cstdint>
#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/siso_vector_system.h"

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
class Integrator : public SisoVectorSystem<T> {
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

  // Returns an Integrator<AutoDiffXd> with the same dimensions as this
  // Integrator.
  std::unique_ptr<Integrator<AutoDiffXd>> ToAutoDiffXd() const {
    return std::unique_ptr<Integrator<AutoDiffXd>>(DoToAutoDiffXd());
  }

 protected:
  // System<T> override.  Returns an Integrator<AutoDiffXd> with the same
  // dimensions as this Integrator.
  Integrator<AutoDiffXd>* DoToAutoDiffXd() const override;

  // System<T> override.  Returns an Integrator<symbolic::Expression> with the
  // same dimensions as this Integrator.
  Integrator<symbolic::Expression>* DoToSymbolic() const override;

  // SisoVectorSystem<T> override.
  void DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const override;

  // SisoVectorSystem<T> override.
  void DoCalcVectorTimeDerivatives(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const override;
};

}  // namespace systems
}  // namespace drake
