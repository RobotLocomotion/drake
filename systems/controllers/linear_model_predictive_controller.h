#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {

/// Implements a basic Model Predictive Controller that linearizes the system
/// about an equilibrium condition and regulates to the same point by solving an
/// optimal control problem over a finite time horizon.  In particular, MPC
/// solves, at each time step k, the following problem to find an optimal u(k)
/// as a function of x(k):
///
///   @f[ \min_{u(k),\ldots,u(k+N),x(k+1),\ldots,x(k+N)}
///          \sum_{i=k}^{k+N} ((x(i) - xd(i))ᵀQ(x(i) - xd(i)) +
///                            (u(i) - ud(i))ᵀR(u(i) - ud(i))) @f]
///   @f[ \mathrm{s.t. } x(k+1) = A(k)x(k) + B(k)u(k) @f]
///
/// and subject to linear inequality constraints on the inputs and states, where
/// N is the horizon length, Q and R are cost matrices, and xd and ud are the
/// desired states and inputs, respectively.  Note that the present
/// implementation solves the QP in whole at every time step, discarding any
/// information between steps.
///
/// @tparam_double_only
/// @ingroup control_systems
template <typename T>
class LinearModelPredictiveController : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearModelPredictiveController)

  // TODO(jadecastro) Implement a version that regulates to an arbitrary
  // trajectory.
  /// Constructor for an unconstrained MPC formulation with linearization
  /// occurring about the provided base_context.  Since this formulation is
  /// devoid of any externally-imposed input/state constraints, the controller
  /// essentially behaves the same as a finite-time LQR.
  ///
  /// @param model The plant model of the System to be controlled.
  /// @param base_context The fixed base point about which to linearize of the
  /// system and regulate the system.  To be valid, @p base_context must
  /// correspond to an equilibrium point of the system.
  /// @param Q A symmetric positive semi-definite state cost matrix of size
  /// (num_states x num_states).
  /// @param R A symmetric positive definite control effort cost matrix of size
  /// (num_inputs x num_inputs).
  /// @param time_period The discrete time period (in seconds) at which
  /// controller updates occur.
  /// @param time_horizon The prediction time horizon (seconds).
  ///
  /// @pre model must have discrete states of dimension num_states and inputs
  /// of dimension num_inputs.
  /// @pre base_context must have discrete states set as appropriate for the
  /// given @p model.  The input must also be initialized via
  /// `input_port.FixValue(base_context, u0)`, or otherwise initialized via
  /// Diagram.
  LinearModelPredictiveController(
      std::unique_ptr<systems::System<double>> model,
      std::unique_ptr<systems::Context<double>> base_context,
      const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, double time_period,
      double time_horizon);
    // TODO(jadecastro) Get time_period directly from the plant model.

  const InputPort<T>& get_state_port() const {
    return this->get_input_port(state_input_index_);
  }
  const OutputPort<T>& get_control_port() const {
    return this->get_output_port(control_output_index_);
  }

 private:
  void CalcControl(const Context<T>& context, BasicVector<T>* control) const;

  // Sets up a DirectTranscription problem and solves for the current control
  // input.
  VectorX<T> SetupAndSolveQp(const Context<T>& base_context,
                             const VectorX<T>& current_state) const;

  const int state_input_index_{-1};
  const int control_output_index_{-1};

  const std::unique_ptr<systems::System<double>> model_;
  // The base context that contains the reference point to regulate.
  const std::unique_ptr<systems::Context<double>> base_context_;

  const int num_states_{};
  const int num_inputs_{};

  const Eigen::MatrixXd Q_;
  const Eigen::MatrixXd R_;

  const double time_period_{};
  const double time_horizon_{};

  // Descrption of the linearized plant model.
  std::unique_ptr<LinearSystem<double>> linear_model_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
