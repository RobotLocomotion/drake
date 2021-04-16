#pragma once

#include <memory>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

namespace drake {
namespace systems {
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
/// @ingroup planning
class DirectCollocation : public MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectCollocation)

  /// Constructs the %MathematicalProgram% and adds the collocation constraints.
  ///
  /// @param system A dynamical system to be used in the dynamic constraints.
  /// This system must support System::ToAutoDiffXd. Note that this is aliased
  /// for the lifetime of this object.
  /// @param context Required to describe any parameters of the system.  The
  /// values of the state in this context do not have any effect.  This context
  /// will also be "cloned" by the optimization; changes to the context after
  /// calling this method will NOT impact the trajectory optimization.
  /// @param num_time_samples The number of breakpoints in the trajectory.
  /// @param minimum_timestep Minimum spacing between sample times.
  /// @param maximum_timestep Maximum spacing between sample times.
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
  DirectCollocation(const System<double>* system,
                    const Context<double>& context, int num_time_samples,
                    double minimum_timestep, double maximum_timestep,
                    std::variant<InputPortSelection, InputPortIndex>
                        input_port_index =
                    InputPortSelection::kUseFirstInputIfItExists,
                    bool assume_non_continuous_states_are_fixed = false);

  // NOTE: The fixed timestep constructor, which would avoid adding h as
  // decision variables, has been removed since it complicates the API and code.
  // Unlike other trajectory optimization transcriptions, direct collocation
  // will not be a convex optimization even if the sample times are fixed, so
  // there is little advantage to actually removing the variables.  Setting
  // minimum_timestep == maximum_timestep should be essentially just as good.

  ~DirectCollocation() override {}

  trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory(
      const solvers::MathematicalProgramResult& result) const override;

  trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory(
      const solvers::MathematicalProgramResult& result) const override;

 private:
  // Implements a running cost at all timesteps using trapezoidal integration.
  void DoAddRunningCost(const symbolic::Expression& e) override;

  // Store system-relevant data for e.g. computing the derivatives during
  // trajectory reconstruction.
  const System<double>* system_{nullptr};
  const std::unique_ptr<Context<double>> context_;
  const std::unique_ptr<ContinuousState<double>> continuous_state_;
  const InputPort<double>* input_port_{nullptr};
  FixedInputPortValue* input_port_value_{nullptr};
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectCollocationConstraint)

 public:
  /// @see DirectCollocation constructor for a description of the parameters.
  DirectCollocationConstraint(
      const System<double>& system, const Context<double>& context,
      std::variant<InputPortSelection, InputPortIndex> input_port_index =
          InputPortSelection::kUseFirstInputIfItExists,
      bool assume_non_continuous_states_are_fixed = false);

  ~DirectCollocationConstraint() override = default;

  int num_states() const { return num_states_; }
  int num_inputs() const { return num_inputs_; }

 protected:
  DirectCollocationConstraint(
      const System<double>& system, const Context<double>& context,
      int num_states, int num_inputs,
      std::variant<InputPortSelection, InputPortIndex> input_port_index,
      bool assume_non_continuous_states_are_fixed);


  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

 private:
  void dynamics(const AutoDiffVecXd& state, const AutoDiffVecXd& input,
                AutoDiffVecXd* xdot) const;

  const std::unique_ptr<System<AutoDiffXd>> system_;
  std::unique_ptr<Context<AutoDiffXd>> context_;
  const InputPort<AutoDiffXd>* input_port_{nullptr};
  FixedInputPortValue* input_port_value_{nullptr};
  std::unique_ptr<ContinuousState<AutoDiffXd>> derivatives_;

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
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& timestep,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& state,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& next_state,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& input,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& next_input,
    solvers::MathematicalProgram* prog);

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
