#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace systems {

/// A continuous-time integrator for a vector input.
///
/// @system
/// name: Integrator
/// input_ports:
/// - u0
/// output_ports:
/// - y0
/// @endsystem
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class Integrator final : public VectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Integrator);

  /// Constructs an %Integrator system. The initial output value will be zero.
  /// @param size number of elements in the signal to be processed.
  explicit Integrator(int size) : Integrator(VectorX<double>::Zero(size)) {}

  /// Constructs an %Integrator system with a particular initial output value.
  /// The size of both input and output are inferred from the given
  /// `initial_value`.
  /// @param initial_value the initial output value.
  explicit Integrator(const VectorX<double>& initial_value);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit Integrator(const Integrator<U>&);

  ~Integrator() final;

  /// Sets the value of the integral modifying the state in the context.
  /// @p value must be a column vector of the appropriate size.
  void set_integral_value(Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

 private:
  template <typename U>
  friend class Integrator;

  // VectorSystem<T> override.
  void DoCalcVectorOutput(const Context<T>& context,
                          const Eigen::VectorBlock<const VectorX<T>>& input,
                          const Eigen::VectorBlock<const VectorX<T>>& state,
                          Eigen::VectorBlock<VectorX<T>>* output) const final;

  // VectorSystem<T> override.
  void DoCalcVectorTimeDerivatives(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const final;

  VectorX<double> initial_value_;  // Save for use during scalar conversion.
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Integrator);
