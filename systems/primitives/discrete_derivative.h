#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// System that outputs the discrete-time derivative of its input:
///   y(t) = (u[n] - u[n-1])/h,
/// where n = floor(t/h), where h is the time period.
///
/// This is implemented as the linear system
/// <pre>
///   x₀[n+1] = u[n],
///   x₁[n+1] = x₀[n],
///   y(t) = (x₀[n]-x₁[n])/h.
///   x₀[0] and x₁[0] are initialized in the Context (default is zeros).
/// </pre>
///
/// @note For dynamical systems, a derivative should not be computed in
/// continuous-time, i.e. `y(t) = (u(t) - u[n])/(t-n*h)`. This is numerically
/// unstable since the time interval `t-n*h` could be arbitrarily close to
/// zero. Prefer the discrete-time implementation for robustness.
///
/// @system{ DiscreteDerivative, @input_port{u}, @output_port{dudt} }
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
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
  /// differentiated, and @p time_step, the sampling interval.
  DiscreteDerivative(int num_inputs, double time_step);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteDerivative(const DiscreteDerivative<U>& other)
      : DiscreteDerivative<T>(other.get_input_port().size(),
                              other.time_step()) {}

  /// Set the input history so that the initial output is fully specified.
  /// This is useful during initialization to avoid large derivative outputs if
  /// u[0] ≠ 0.  @p u_n and @ u_n_minus_1 must be the same size as the
  /// input/output ports.
  void set_input_history(systems::State<T>* state,
                         const Eigen::Ref<const VectorX<T>>& u_n,
                         const Eigen::Ref<const VectorX<T>>& u_n_minus_1) const;

  /// Set the input history so that the initial output is fully specified.
  /// This is useful during initialization to avoid large derivative outputs if
  /// u[0] ≠ 0.  @p u_n and @ u_n_minus_1 must be the same size as the
  /// input/output ports.
  void set_input_history(systems::Context<T>* context,
                         const Eigen::Ref<const VectorX<T>>& u_n,
                         const Eigen::Ref<const VectorX<T>>& u_n_minus_1)
                         const {
    set_input_history(&context->get_mutable_state(), u_n, u_n_minus_1);
  }

  /// Convenience method to set the entire input history to a constant
  /// vector value (x₀ = x₁ = u,resulting in a derivative = 0).  This is
  /// useful during initialization to avoid large derivative outputs if
  /// u[0] ≠ 0.  @p u must be the same size as the input/output ports.
  void set_input_history(systems::Context<T>* context,
                         const Eigen::Ref<const VectorX<T>>& u) const {
    set_input_history(&context->get_mutable_state(), u, u);
  }

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

  const int n_;  // The size of the input (and output) ports.
  const double time_step_;
};

/// Supports the common pattern of combining a (feed-through) position with
/// a velocity estimated with the DiscreteDerivative into a single output
/// vector with positions and velocities stacked.  This assumes that the
/// number of positions == the number of velocities.
///
///                                  ┌─────┐
/// position ───┬───────────────────>│     │
///             │                    │ Mux ├──> state
///             │   ┌────────────┐   │     │
///             └──>│  Discrete  ├──>│     │
///                 │ Derivative │   └─────┘
///                 └────────────┘
///
/// @system{ StateInterpolatorWithDiscreteDerivative,
///          @input_port{position},
///          @output_port{state}
/// }
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// @ingroup primitive_systems
template <typename T>
class StateInterpolatorWithDiscreteDerivative final : public Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateInterpolatorWithDiscreteDerivative)

  /// Constructor taking @p num_positions, the size of the position vector
  /// to be differentiated, and @p time_step, the sampling interval.
  StateInterpolatorWithDiscreteDerivative(int num_positions, double time_step);

  const systems::InputPort<T>& get_input_port() const {
    return System<T>::get_input_port(0);
  }

  const systems::OutputPort<T>& get_output_port() const {
    return System<T>::get_output_port(0);
  }

  /// Convenience method to set the entire position history for the
  /// discrete-time derivative to a constant vector value (resulting in
  /// velocity estimate of zero). This is useful during initialization to
  /// avoid large derivative outputs.  @p position must be the same
  /// size as the input/output ports.
  void set_initial_position(systems::State<T>* state,
                            const Eigen::Ref<const VectorX<T>>& position) const;

  /// Convenience method to set the entire position history for the
  /// discrete-time derivative as if the most recent input was @p position,
  /// and the input before that was whatever was required to produce the
  /// output velocity @p velocity.  @p position and @p velocity must be the
  /// same size as the input/output ports.
  void set_initial_state(systems::State<T>* state,
                         const Eigen::Ref<const VectorX<T>>& position,
                         const Eigen::Ref<const VectorX<T>>& velocity) const;

  /// Convenience method to set the entire position history for the
  /// discrete-time derivative to a constant vector value (resulting in
  /// velocity estimate of zero). This is useful during initialization to
  /// avoid large derivative outputs.  @p position must be the same
  /// size as the input/output ports.
  void set_initial_position(systems::Context<T>* context,
                            const Eigen::Ref<const VectorX<T>>& position)
                            const {
    set_initial_position(&context->get_mutable_state(), position);
  }

  /// Convenience method to set the entire position history for the
  /// discrete-time derivative as if the most recent input was @p position,
  /// and the input before that was whatever was required to produce the
  /// output velocity @p velocity.  @p position and @p velocity must be the
  /// same size as the input/output ports.
  void set_initial_state(systems::Context<T>* context,
                         const Eigen::Ref<const VectorX<T>>& position,
                         const Eigen::Ref<const VectorX<T>>& velocity) const {
    set_initial_state(&context->get_mutable_state(), position, velocity);
  }

 private:
  DiscreteDerivative<T>* derivative_;
};

}  // namespace systems
}  // namespace drake
