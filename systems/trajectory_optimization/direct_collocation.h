#pragma once

#include <memory>

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
/// running costs) to the midpoints as well as the knot points in order to
/// achieve a 3rd order integration accuracy.
class DirectCollocation : public MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectCollocation)

  /// Constructs the %MathematicalProgram% and adds the collocation constraints.
  ///
  /// @param system A dynamical system to be used in the dynamic constraints.
  ///    This system must support System::ToAutoDiffXd.
  ///    Note that this is aliased for the lifetime of this object.
  /// @param context Required to describe any parameters of the system.  The
  ///    values of the state in this context do not have any effect.  This
  ///    context will also be "cloned" by the optimization; changes to the
  ///    context after calling this method will NOT impact the trajectory
  ///    optimization.
  /// @param num_time_samples The number of knot points in the trajectory.
  /// @param minimum_timestep Minimum spacing between sample times.
  /// @param maximum_timestep Maximum spacing between sample times.
  DirectCollocation(const System<double>* system,
                    const Context<double>& context, int num_time_samples,
                    double minimum_timestep, double maximum_timestep);

  // NOTE: The fixed timestep constructor, which would avoid adding h as
  // decision variables, has been removed since it complicates the API and code.
  // Unlike other trajectory optimization transcriptions, direct collocation
  // will not be a convex optimization even if the sample times are fixed, so
  // there is little advantage to actually removing the variables.  Setting
  // minimum_timestep == maximum_timestep should be essentially just as good.

  ~DirectCollocation() override {}

  /// Get the input trajectory at the solution as a
  /// %PiecewisePolynomialTrajectory%.
  PiecewisePolynomialTrajectory ReconstructInputTrajectory() const override;

  /// Get the state trajectory at the solution as a
  /// %PiecewisePolynomialTrajectory%.
  PiecewisePolynomialTrajectory ReconstructStateTrajectory() const override;

 private:
  // Implements a running cost at all timesteps using trapezoidal integration.
  void DoAddRunningCost(const symbolic::Expression& e) override;

  // Store system-relevant data for e.g. computing the derivatives during
  // trajectory reconstruction.
  const System<double>* system_{nullptr};
  const std::unique_ptr<Context<double>> context_{nullptr};
  const std::unique_ptr<ContinuousState<double>> continuous_state_{nullptr};
  FreestandingInputPortValue* input_port_value_{nullptr};
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
