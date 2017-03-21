#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/trajectory_optimization/direct_trajectory_optimization.h"

namespace drake {
namespace systems {

/**
 * DircolTrajectoryOptimization implements a direct colocation
 * approach to trajectory optimization, as described in
 *   C. R. Hargraves and S. W. Paris. Direct trajectory optimization using
 *    nonlinear programming and collocation. J Guidance, 10(4):338-342,
 *    July-August 1987.
 * It assumes a first-order hold on the input trajectory and a cubic spline
 * representation of the state trajectory, and adds dynamic constraints (and
 * running costs) to the midpoints as well as the knot points in order to
 * achieve a 3rd order integration accuracy.
 *
 * @param system A dynamical system to be used in the dynamic constraints.
 *    This system must implement DoToAutoDiffXd. Note that this is aliased for
 *    the lifetime of this object.
 * @param context Required to describe any parameters of the system.  The
 *    values of the state in this context do not have any effect.  This
 *    context will also be "cloned" by the optimization; changes to the context
 *    after calling this method will NOT impact the trajectory optimization.
 * @param num_time_samples The number of knot points in the trajectory.
 * @param trajectory_time_lower_bound Constrains the minimum duration for the
 *    entire trajectory.  Required to prevent trivial solutions (like total
 *    time = 0).
 * @param trajectory_time_upper_bound Constrains the maximum duration for the
 *    entire trajectory.  Required to prevent trivial solutions and solutions
 *    with unbounded/unreasonable numerical integration errors.
 */
class DircolTrajectoryOptimization : public DirectTrajectoryOptimization {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DircolTrajectoryOptimization)

  DircolTrajectoryOptimization(const System<double>* system,
                               const Context<double>& context,
                               int num_time_samples,
                               double trajectory_time_lower_bound,
                               double trajectory_time_upper_bound);

  ~DircolTrajectoryOptimization() override {}

  void AddRunningCost(std::shared_ptr<solvers::Constraint> constraint) override;

  PiecewisePolynomialTrajectory ReconstructStateTrajectory() const override;
  // TODO(Lucy-tri) According to @siyuanfeng-tri, the current calculation of
  // derivatives is not correct for floating base joints. More strongly, we
  // can't use independent splines for joints with coupled degrees of freedom.

 private:
  const System<double>* system_{nullptr};
  const std::unique_ptr<Context<double>> context_{nullptr};
  const std::unique_ptr<ContinuousState<double>> continuous_state_{nullptr};
  FreestandingInputPortValue* input_port_value_{nullptr};
};

}  // namespace systems
}  // namespace drake
