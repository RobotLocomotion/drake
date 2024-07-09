#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** A discrete-time integrator for a vector input, using explicit Euler
integration.

@system
name: DiscreteTimeIntegrator
input_ports:
- u
output_ports:
- y
@endsystem

The discrete state space dynamics of %DiscreteTimeIntegrator with
time step `h` is:
```
xₙ₊₁ = xₙ + h uₙ  // update
yₙ = xₙ           // output
x₀ = xᵢₙᵢₜ        // initialize
```
where xᵢₙᵢₜ = 0 by default. Use set_integral_value() to set xₙ in the context.
The output at time `t` is `xₙ` where `n = ceil(t/h)`.  See @ref
discrete_systems.

@tparam_default_scalar
@ingroup primitive_systems
*/
template <typename T>
class DiscreteTimeIntegrator final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteTimeIntegrator);

  /// Constructs an %DiscreteTimeIntegrator system.
  /// @param size number of elements in the signal to be processed.
  /// @param time_step the discrete time step.
  /// @pre size > 0. time_step > 0.
  DiscreteTimeIntegrator(int size, double time_step);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteTimeIntegrator(const DiscreteTimeIntegrator<U>&);

  ~DiscreteTimeIntegrator() final;

  /// Sets the value of the integral modifying the state in the context.
  /// @p value must be a column vector of the appropriate size.
  void set_integral_value(Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

  /// Returns the time_step used by the integrator.
  double time_step() const { return time_step_; }

 private:
  void Update(const Context<T>& context, DiscreteValues<T>* next_state) const;

  double time_step_{};
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteTimeIntegrator);
