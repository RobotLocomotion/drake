#pragma once

#include <memory>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

// Helper struct holding a time-step value for continuous-time
// DirectTranscription.
struct TimeStep {
  double value{-1};
  explicit TimeStep(double step) : value(step) {}
};

/// DirectTranscription is perhaps the simplest implementation of a multiple
/// shooting method, where we have decision variables representing the
/// control and input at every sample time in the trajectory, and one-step
/// of numerical integration provides the dynamic constraints between those
/// decision variables.
/// @ingroup planning
class DirectTranscription : public MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectTranscription)

  /// Constructs the MathematicalProgram and adds the dynamic constraints.
  /// This version of the constructor is only for simple discrete-time systems
  /// (with a single periodic timestep update).  Continuous-time systems
  /// must call one of the constructors that takes bounds on the timestep as
  /// an argument.
  ///
  /// @param system A dynamical system to be used in the dynamic constraints.
  ///    This system must support System::ToAutoDiffXd.
  ///    Note that this is aliased for the lifetime of this object.
  /// @param context Required to describe any parameters of the system.  The
  ///    values of the state in this context do not have any effect.  This
  ///    context will also be "cloned" by the optimization; changes to the
  ///    context after calling this method will NOT impact the trajectory
  ///    optimization.
  /// @param num_time_samples The number of breakpoints in the trajectory.
  /// @param input_port_index A valid input port index or valid
  /// InputPortSelection for @p system.  All other inputs on the system will be
  /// left disconnected (if they are disconnected in @p context) or will be set
  /// to their current values (if they are connected/fixed in @p context).
  /// @default kUseFirstInputIfItExists.
  DirectTranscription(
      const System<double>* system, const Context<double>& context,
      int num_time_samples,
      const std::variant<InputPortSelection, InputPortIndex>& input_port_index =
          InputPortSelection::kUseFirstInputIfItExists);

  // TODO(russt): Generalize the symbolic short-cutting to handle this case,
  //  and remove the special-purpose constructor (unless we want it for
  //  efficiency).
  /// Constructs the MathematicalProgram and adds the dynamic constraints.  This
  /// version of the constructor is only for *linear time-varying* discrete-time
  /// systems (with a single periodic timestep update).  This constructor adds
  /// value because the symbolic short-cutting does not yet support systems
  /// that are affine in state/input, but not time.
  ///
  /// @param system A linear time-varying system to be used in the dynamic
  ///    constraints. Note that this is aliased for the lifetime of this object.
  /// @param context Required to describe any parameters of the system.  The
  ///    values of the state in this context do not have any effect.  This
  ///    context will also be "cloned" by the optimization; changes to the
  ///    context after calling this method will NOT impact the trajectory
  ///    optimization.
  /// @param num_time_samples The number of breakpoints in the trajectory.
  /// @param input_port_index A valid input port index or valid
  /// InputPortSelection for @p system.  All other inputs on the system will be
  /// left disconnected (if they are disconnected in @p context) or will be set
  /// to their current values (if they are connected/fixed in @p context).
  /// @default kUseFirstInputIfItExists.
  ///
  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.  When
  /// we do bind it, we should probably rename `system` to tv_linear_system` or
  /// similar, so that kwargs determine which overload is suggested, instead of
  /// hoping that type checking does the right thing.}
  DirectTranscription(
      const TimeVaryingLinearSystem<double>* system,
      const Context<double>& context, int num_time_samples,
      const std::variant<InputPortSelection, InputPortIndex>& input_port_index =
          InputPortSelection::kUseFirstInputIfItExists);

  // TODO(russt): Support more than just forward Euler integration
  //  (perhaps by taking IntegratorBase as an optional parameter?).
  /// Constructs the MathematicalProgram and adds the dynamic constraints.
  /// This version of the constructor is only for continuous-time systems;
  /// the dynamics constraints use explicit forward Euler integration.
  ///
  /// @param system A dynamical system to be used in the dynamic constraints.
  ///    This system must support System::ToAutoDiffXd.
  ///    Note that this is aliased for the lifetime of this object.
  /// @param context Required to describe any parameters of the system.  The
  ///    values of the state in this context do not have any effect.  This
  ///    context will also be "cloned" by the optimization; changes to the
  ///    context after calling this method will NOT impact the trajectory
  ///    optimization.
  /// @param num_time_samples The number of breakpoints in the trajectory.
  /// @param fixed_timestep The spacing between sample times.
  /// @param input_port_index A valid input port index or valid
  /// InputPortSelection for @p system.  All other inputs on the system will be
  /// left disconnected (if they are disconnected in @p context) or will be set
  /// to their current values (if they are connected/fixed in @p context).
  /// @default kUseFirstInputIfItExists.
  DirectTranscription(
      const System<double>* system, const Context<double>& context,
      int num_time_samples, TimeStep fixed_timestep,
      const std::variant<InputPortSelection, InputPortIndex>& input_port_index =
          InputPortSelection::kUseFirstInputIfItExists);

  // TODO(russt):  Implement constructor for continuous time systems with
  // time as a decision variable; and perhaps add support for mixed
  // discrete-/continuous- systems.

  ~DirectTranscription() override {}

  /// Get the input trajectory at the solution as a PiecewisePolynomial.  The
  /// order of the trajectory will be determined by the integrator used in
  /// the dynamic constraints.
  trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory(
      const solvers::MathematicalProgramResult& result) const override;

  /// Get the state trajectory at the solution as a PiecewisePolynomial.  The
  /// order of the trajectory will be determined by the integrator used in
  /// the dynamic constraints.
  trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory(
      const solvers::MathematicalProgramResult& result) const override;

 private:
  // Implements a running cost at all timesteps.
  void DoAddRunningCost(const symbolic::Expression& e) override;

  // Attempts to create a symbolic version of the plant, and to add linear
  // constraints to impose the dynamics if possible.  Returns true iff the
  // constraints are added.
  bool AddSymbolicDynamicConstraints(
      const System<double>* system, const Context<double>& context,
      const std::variant<InputPortSelection, InputPortIndex>& input_port_index);

  // Attempts to create an autodiff version of the plant, and to impose
  // the generic (nonlinear) constraints to impose the dynamics.
  // Aborts if the conversion ToAutoDiffXd fails.
  void AddAutodiffDynamicConstraints(
      const System<double>* system, const Context<double>& context,
      const std::variant<InputPortSelection, InputPortIndex>& input_port_index);

  // Constrain the final input to match the penultimate, otherwise the final
  // input is unconstrained.
  // (Note that it might be more ideal to have fewer decision variables
  // allocated, but this is a reasonable work-around).
  //
  // TODO(jadecastro) Allow MultipleShooting to take on N-1 inputs, and remove
  // this constraint.
  void ConstrainEqualInputAtFinalTwoTimesteps();

  // Ensures that the MultipleShooting problem is well-formed and that the
  // provided @p system and @p context have only one group of discrete states
  // and only one (possibly multidimensional) input.
  void ValidateSystem(
      const System<double>& system, const Context<double>& context,
      const std::variant<InputPortSelection, InputPortIndex>& input_port_index);

  // AutoDiff versions of the System components (for the constraints).
  // These values are allocated iff the dynamic constraints are allocated
  // as DirectTranscriptionConstraint, otherwise they are nullptr.
  std::unique_ptr<const System<AutoDiffXd>> system_;
  std::unique_ptr<Context<AutoDiffXd>> context_;
  std::unique_ptr<IntegratorBase<AutoDiffXd>> integrator_;
  const InputPort<AutoDiffXd>* input_port_{nullptr};
  FixedInputPortValue* input_port_value_{nullptr};  // Owned by the context.

  const bool discrete_time_system_{false};
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
