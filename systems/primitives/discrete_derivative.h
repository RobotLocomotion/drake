#pragma once

#include "drake/systems/primitives/linear_system.h"

using Eigen::MatrixXd;

namespace drake {
namespace systems {

/// System that outputs the discrete-time derivative of the input:
///   @f$ y[n] = (u[n] - u[n-1])/h @f$
///
/// This is implemented as the linear system
///   @f[ x[n+1] = u[n], @f]
///   @f[ y[n] = (u[n]-x[n])/h, @f]
/// where @f$ h @f$ is the time period.
///
/// @system{ DiscreteDerivative, @input_port{u(t)}, @output_port{du(t)/dt} }
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
class DiscreteDerivative final : public LinearSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteDerivative)

  DiscreteDerivative(int num_inputs, double time_period)
      : LinearSystem<T>(
            SystemTypeTag<systems::DiscreteDerivative>{},
            MatrixXd::Zero(num_inputs, num_inputs),
            MatrixXd::Identity(num_inputs, num_inputs),
            -MatrixXd::Identity(num_inputs, num_inputs) / time_period,
            MatrixXd::Identity(num_inputs, num_inputs) / time_period,
            time_period) {
    DRAKE_DEMAND(time_period > 0.0);
  }

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteDerivative(const DiscreteDerivative<U>& other)
      : DiscreteDerivative<T>(other.A().cols(), 1. / other.D()(0, 0)) {}
};

}  // namespace systems
}  // namespace drake
