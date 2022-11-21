#pragma once

#include <array>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

/** Optimizes a trajectory, q(t) subject to costs and constraints on the
trajectory and its derivatives. This is accomplished using a `path`, r(s),
represented as a BsplineTrajectory on the interval s∈[0,1], and a separate
duration, T, which maps [0,1] => [0,T].

The q(t) trajectory is commonly associated with, for instance, the generalized
positions of a MultibodyPlant by adding multibody costs and constraints; in
this case take note that the velocities in this optimization are q̇(t), not
v(t).

Use solvers::Solve to solve the problem. A typical use case could look like:
@verbatim
  KinematicTrajectoryOptimization trajopt(2, 10);
  // add costs and constraints
  trajopt.SetInitialGuess(...);
  auto result = solvers::Solve(trajopt.prog());
  auto traj = trajopt.ReconstructTrajectory(result);
@endverbatim

When possible this class attempts to formulate convex forms of the costs and
constraints.
*/
class KinematicTrajectoryOptimization {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KinematicTrajectoryOptimization);

  // TODO(russt): Change the "approximately" in the description of the
  // time_scaling initialization to "exactly" by creating methods in
  // BsplineTrajectory to initialize the splines (analogous to
  // PiecewisePolynomial::Cubic, etc).

  /** Constructs an optimization problem for a position trajectory represented
  as a B-spline. The initial guess is the zero trajectory over the time
  interval [0, T].
  @param num_positions       The number of rows in the B-spline.
  @param num_control_points  The number of B-spline control points.
  @param spline_order        The order of the B-spline.
  @param duration            The duration (in seconds) of the initial guess.
  */
  KinematicTrajectoryOptimization(int num_positions, int num_control_points,
                                  int spline_order = 4, double duration = 1.0);

  /** Constructs an optimization problem for a trajectory represented by a
  B-spline with the same order and number of control points as `trajectory`.
  Additionally sets `trajectory` as the initial guess for the optimization. */
  explicit KinematicTrajectoryOptimization(
      const trajectories::BsplineTrajectory<double>& trajectory);

  /** Returns the number of position variables. */
  int num_positions() const { return num_positions_; }

  /** Returns the number of control points used for the path. */
  int num_control_points() const { return num_control_points_; }

  /** Returns the basis used to represent the path, r(s), over s∈[0,1]. */
  const math::BsplineBasis<double>& basis() const { return basis_; }

  /** Returns the control points defining the path as an M-by-N matrix, where M
  is the number of positions and N is the number of control points. */
  const solvers::MatrixXDecisionVariable& control_points() const {
    return control_points_;
  }

  /** Returns the decision variable defining the time duration of the
  trajectory. */
  const symbolic::Variable& duration() const { return duration_; }

  /** Getter for the optimization program. */
  const solvers::MathematicalProgram& prog() const { return prog_; }

  /** Getter for a mutable pointer to the optimization program. */
  solvers::MathematicalProgram& get_mutable_prog() { return prog_; }

  /** Sets the initial guess for the path and duration to match `trajectory`.
  @pre trajectory.rows() == num_positions()
  @pre trajectory.columns() == 1
  */
  void SetInitialGuess(
      const trajectories::BsplineTrajectory<double>& trajectory);

  /** Returns the trajectory q(t) from the `result` of solving `prog()`. */
  trajectories::BsplineTrajectory<double> ReconstructTrajectory(
      const solvers::MathematicalProgramResult& result) const;

  /** Adds a linear constraint on the value of the path, `lb` ≤ r(s) ≤ `ub`.
  @pre 0 <= `s` <= 1. */
  void AddPathPositionConstraint(const Eigen::Ref<const Eigen::VectorXd>& lb,
                                 const Eigen::Ref<const Eigen::VectorXd>& ub,
                                 double s);

  /** Adds a (generic) constraint on path. The constraint will be evaluated
  as if it is bound with variables corresponding to `r(s)`.
  @pre constraint.num_vars() == num_positions()
  @pre 0 <= `s` <= 1. */
  void AddPathPositionConstraint(
      const std::shared_ptr<solvers::Constraint>& constraint, double s);

  /** Adds a linear constraint on the derivative of the path, `lb` ≤ ṙ(s) ≤
  `ub`. Note that this does NOT directly constrain q̇(t).
  @pre 0 <= `s` <= 1. */
  void AddPathVelocityConstraint(const Eigen::Ref<const Eigen::VectorXd>& lb,
                                 const Eigen::Ref<const Eigen::VectorXd>& ub,
                                 double s);

  /** Adds a (generic) constraint on trajectory velocity `q̇(t)`, evaluated at
  `s`. The constraint will be evaluated as if it is bound with variables
  corresponding to `[q(T*s), q̇(T*s)]`.

  This is a potentially confusing mix of `s` and `t`, but it is important in
  practice. For instance if you want to constrain the true (trajectory)
  velocity at the final time, one would naturally want to write
  AddVelocityConstraint(constraint, s=1).
  @pre constraint.num_vars() == num_positions()
  @pre 0 <= `s` <= 1. */
  void AddVelocityConstraintAtNormalizedTime(
      const std::shared_ptr<solvers::Constraint>& constraint, double s);

  /** Adds a linear constraint on the second derivative of the path,
  `lb` ≤ r̈(s) ≤ `ub`. Note that this does NOT directly constrain q̈(t).
  @pre 0 <= `s` <= 1. */
  void AddPathAccelerationConstraint(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub, double s);

  /** Adds upper and lower bounds on the duration of the trajectory. */
  void AddDurationConstraint(std::optional<double> lb,
                             std::optional<double> ub);

  /** Adds bounding box constraints to enforce upper and lower bounds on the
  positions trajectory, q(t). These bounds will be respected at all times,
  t∈[0,T]. This also implies the constraints are satisfied for r(s), for all
  s∈[0,1]. */
  void AddPositionBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                         const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Adds linear constraints to enforce upper and lower bounds on the velocity
  trajectory, q̇(t). These bounds will be respected at all times. Note this
  does NOT directly constrain ṙ(s). */
  void AddVelocityBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                         const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Adds generic (nonlinear) constraints to enforce the upper and lower
  bounds to the acceleration trajectory, q̈(t).  These constraints will be
  respected at all times.  Note that this does NOT directly constrain r̈(s). */
  void AddAccelerationBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                             const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Adds generic (nonlinear) constraints to enforce the upper and lower bounds
  to the jerk trajectory, d³qdt³(t).  These constraints will be respected at all
  times.  Note that this does NOT directly constrain d³rds³(s). */
  void AddJerkBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                             const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Adds a linear cost on the duration of the trajectory. */
  void AddDurationCost(double weight = 1.0);

  /** Adds a cost on an upper bound on length of the path, ∫₀ᵀ |q̇(t)|₂ dt, by
  summing the distance between the path control points. If `use_conic_constraint
  = false`, then costs are added via MathematicalProgram::AddL2NormCost;
  otherwise they are added via
  MathematicalProgram::AddL2NormCostUsingConicConstraint. */
  void AddPathLengthCost(double weight = 1.0,
                         bool use_conic_constraint = false);

  /* TODO(russt):
  - Support a cost on an upper bound on "energy" of the path, ∫₀ᵀ |q̇(t)|₂² dt.
      void AddEnergyCost(double weight = 1.0);
  - Support additional (non-convex) costs/constraints on q(t) directly.
  */

 private:
  solvers::MathematicalProgram prog_{};
  int num_positions_{};
  int num_control_points_{};

  math::BsplineBasis<double> basis_;
  solvers::MatrixXDecisionVariable control_points_;
  symbolic::Variable duration_;

  /* TODO(russt): Minimize the use of symbolic to construct the constraints.
  This is inefficient, and the B-spline math should all have closed-form
  solutions for most everything we need.*/

  // r(s) is the path.
  copyable_unique_ptr<trajectories::BsplineTrajectory<symbolic::Expression>>
      sym_r_{};
  copyable_unique_ptr<trajectories::BsplineTrajectory<symbolic::Expression>>
      sym_rdot_{};
  copyable_unique_ptr<trajectories::BsplineTrajectory<symbolic::Expression>>
      sym_rddot_{};
  copyable_unique_ptr<trajectories::BsplineTrajectory<symbolic::Expression>>
      sym_rdddot_{};
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
