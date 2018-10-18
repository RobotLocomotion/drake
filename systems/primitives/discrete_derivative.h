#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// System that outputs the discrete-time derivative of the input:
///   @f[ y(t) = (u[n] - u[n-1])/h, @f]
/// where @f$ n = floor(t/h), @f$ where @f$ h @f$ is the time period.
///
/// This is implemented as the linear system
///   @f[ x₀[n+1] = u[n], @f]
///   @f[ x₁[n+1] = x₀[n] = u[n-1], @f]
///   @f[ y(t) = (x₀[n]-x₁[n])/h. @f]
///
/// Note: The reason for this specific implementation is a little subtle.
/// Since drake systems do not control the evaluation time of their output
/// ports, a downstream system may ask for the output at any continuous time.
/// The estimate @f$ y(t) = (u(t) - u[n])/(t-n*h) @f$, which is a more
/// continuous-time derivative, would require fewer state variables, and
/// introduce less delay.  But the inconsistent time interval (which could be
/// arbitrarily close to zero) makes it numerically unreliable.  Prefer the
/// discrete-time derivative implemented here.
///
/// @system{ DiscreteDerivative, @input_port{u}, @output_port{dudt} }
///
/// @tparam T The type of mathematical object being added.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// @ingroup primitive_systems
template <class T>
class DiscreteDerivative final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteDerivative)

  /// Constructor taking @p num_inputs, the size of the vector to be
  // differentiated, and @p time_step, the sampling interval.
  DiscreteDerivative(int num_inputs, double time_step);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteDerivative(const DiscreteDerivative<U>& other)
      : DiscreteDerivative<T>(other.get_input_port().size(),
                              other.time_step()) {}

  /// Convenience method to set the entire input history to a constant
  /// vector value (resulting in a derivative = 0).
  void set_history(const Eigen::Ref<const VectorX<T>>& q,
                   systems::Context<T>* context) const;

  const systems::InputPort<T>& get_input_port() const {
    return System<T>::get_input_port(0);
  }

  const systems::OutputPort<T>& get_output_port() const {
    return System<T>::get_output_port(0);
  }

  double time_step() const { return time_step_; }

 private:
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>&,
      DiscreteValues<T>* discrete_state) const final;

  void CalcOutput(const Context<T>& context,
                  BasicVector<T>* output_vector) const;

  const int N_;  // The size of the input (and output) ports.
  const double time_step_;
};

}  // namespace systems
}  // namespace drake
