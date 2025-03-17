#pragma once

#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/planning/trajectory_optimization/multiple_shooting.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

/// DirectCollocation implements the approach to trajectory optimization as
/// described in
///   C. R. Hargraves and S. W. Paris. Direct trajectory optimization using
///    nonlinear programming and collocation. J Guidance, 10(4):338-342,
///    July-August 1987.
/// It assumes a first-order hold on the input trajectory and a cubic spline
/// representation of the state trajectory, and adds dynamic constraints (and
/// running costs) to the midpoints as well as the breakpoints in order to
/// achieve a 3rd order integration accuracy.
///
/// Note: This algorithm only works with the continuous states of a system.
/// @ingroup planning_trajectory
class DirectCollocation : public MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectCollocation);

  /// Constructs the %MathematicalProgram% and adds the collocation
  /// constraints.
  ///
  /// @param system A dynamical system to be used in the dynamic constraints.
  /// This system must support System::ToAutoDiffXd. Note that this is aliased
  /// for the lifetime of this object.
  /// @param context Required to describe any parameters of the system.  The
  /// values of the state in this context do not have any effect.  This context
  /// will also be "cloned" by the optimization; changes to the context after
  /// calling this method will NOT impact the trajectory optimization.
  /// @param num_time_samples The number of breakpoints in the trajectory.
  /// @param minimum_time_step Minimum spacing between sample times.
  /// @param maximum_time_step Maximum spacing between sample times.
  /// @param input_port_index A valid input port index for @p system or
  /// InputPortSelection.  All other inputs on the system will be left
  /// disconnected (if they are disconnected in @p context) or will be fixed to
  /// their current values (if they are connected/fixed in @p context).
  /// @default kUseFirstInputIfItExists.
  /// @param assume_non_continuous_states_are_fixed Boolean which, if true,
  /// allows this algorithm to optimize without considering the dynamics of any
  /// non-continuous states. This is helpful for optimizing systems that might
  /// have some additional book-keeping variables in their state. Only use this
  /// if you are sure that the dynamics of the additional state variables
  /// cannot impact the dynamics of the continuous states. @default false.
  /// @param prog (optional).  If non-null, then additional decision variables,
  /// costs, and constraints will be added into the existing
  /// MathematicalProgram. This can be useful for, e.g., combining multiple
  /// trajectory optimizations into a single program, coupled by a few
  /// constraints.  If nullptr, then a new MathematicalProgram will be
  /// allocated.
  /// @throws std::exception if `system` is not supported by this direct
  /// collocation method.
  DirectCollocation(
      const systems::System<double>* system,
      const systems::Context<double>& context, int num_time_samples,
      double minimum_time_step, double maximum_time_step,
      std::variant<systems::InputPortSelection, systems::InputPortIndex>
          input_port_index =
              systems::InputPortSelection::kUseFirstInputIfItExists,
      bool assume_non_continuous_states_are_fixed = false,
      solvers::MathematicalProgram* prog = nullptr);

  // NOTE: The fixed-time-step constructor, which would avoid adding h as
  // decision variables, has been (temporarily) removed since it complicates
  // the API and code.

  ~DirectCollocation() override;

  trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory(
      const solvers::MathematicalProgramResult& result) const override;

  trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory(
      const solvers::MathematicalProgramResult& result) const override;

 private:
  // Implements a running cost at all time steps using trapezoidal integration.
  void DoAddRunningCost(const symbolic::Expression& e) override;

  // Store system-relevant data for e.g. computing the derivatives during
  // trajectory reconstruction.
  const systems::System<double>* system_{nullptr};
  const std::unique_ptr<systems::Context<double>> context_;
  const std::variant<systems::InputPortSelection, systems::InputPortIndex>
      input_port_index_;

  std::unique_ptr<systems::System<AutoDiffXd>> system_ad_;
  std::unique_ptr<systems::Context<AutoDiffXd>> context_ad_;
  std::vector<std::unique_ptr<systems::Context<AutoDiffXd>>> sample_contexts_;
};

/// Implements the direct collocation constraints for a first-order hold on
/// the input and a cubic polynomial representation of the state trajectories.
///
/// Note that the DirectCollocation implementation allocates only ONE of
/// these constraints, but binds that constraint multiple times (with
/// different decision variables, along the trajectory).
/// @ingroup solver_evaluators
class DirectCollocationConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectCollocationConstraint);

 public:
  /// @see DirectCollocation constructor for a description of the parameters.
  /// @throws std::exception if `system` is not supported by this direct
  /// collocation method.
  /// @pydrake_mkdoc_identifier{double}
  DirectCollocationConstraint(
      const systems::System<double>& system,
      const systems::Context<double>& context,
      std::variant<systems::InputPortSelection, systems::InputPortIndex>
          input_port_index =
              systems::InputPortSelection::kUseFirstInputIfItExists,
      bool assume_non_continuous_states_are_fixed = false);

  /// Constructor which supports passing different mutable contexts for the
  /// different evaluation times. This can be used to facilitate caching (for
  /// instance, if the `context_segment_start` of one constraint uses the
  /// `context_segment_end` of the previous constraint).
  ///
  /// @see DirectCollocation constructor for a description of the remaining
  /// parameters.
  ///
  /// @throws std::exception if `system` is not supported by this direct
  /// collocation method.
  /// @pydrake_mkdoc_identifier{autodiff}
  DirectCollocationConstraint(
      const systems::System<AutoDiffXd>& system,
      systems::Context<AutoDiffXd>* context_sample,
      systems::Context<AutoDiffXd>* context_next_sample,
      systems::Context<AutoDiffXd>* context_collocation,
      std::variant<systems::InputPortSelection, systems::InputPortIndex>
          input_port_index =
              systems::InputPortSelection::kUseFirstInputIfItExists,
      bool assume_non_continuous_states_are_fixed = false);

  ~DirectCollocationConstraint() override;

  int num_states() const { return num_states_; }
  int num_inputs() const { return num_inputs_; }

 protected:
  DirectCollocationConstraint(
      std::pair<std::unique_ptr<systems::System<AutoDiffXd>>,
                std::unique_ptr<systems::Context<AutoDiffXd>>>
          owned_pair,
      const systems::System<AutoDiffXd>* system,
      systems::Context<AutoDiffXd>* context_sample,
      systems::Context<AutoDiffXd>* context_next_sample,
      systems::Context<AutoDiffXd>* context_collocation, int num_states,
      int num_inputs,
      std::variant<systems::InputPortSelection, systems::InputPortIndex>
          input_port_index,
      bool assume_non_continuous_states_are_fixed);

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

 private:
  void CalcDynamics(const AutoDiffVecXd& state, const AutoDiffVecXd& input,
                    systems::Context<AutoDiffXd>* context,
                    AutoDiffVecXd* xdot) const;

  // Note: owned_system_ and owned_context_ can be nullptr.
  std::unique_ptr<systems::System<AutoDiffXd>> owned_system_;
  std::unique_ptr<systems::Context<AutoDiffXd>> owned_context_;

  const systems::System<AutoDiffXd>& system_;
  systems::Context<AutoDiffXd>* context_sample_;
  systems::Context<AutoDiffXd>* context_next_sample_;
  systems::Context<AutoDiffXd>* context_collocation_;
  const systems::InputPort<AutoDiffXd>* input_port_;

  const int num_states_{0};
  const int num_inputs_{0};
};

// Note: The order of arguments is a compromise between GSG and the desire to
// match the AddConstraint interfaces in MathematicalProgram.
/// Helper method to add a DirectCollocationConstraint to the @p prog,
/// ensuring that the order of variables in the binding matches the order
/// expected by the constraint.
solvers::Binding<solvers::Constraint> AddDirectCollocationConstraint(
    std::shared_ptr<DirectCollocationConstraint> constraint,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& time_step,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& state,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& next_state,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& input,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& next_input,
    solvers::MathematicalProgram* prog);

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
