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

  /** Constructs an optimization problem for a `num_positions`-element position
  trajectory represented as a `spline_order`-order B-spline with
  `num_control_points` control_points. The initial guess of the `path` will be
  set to the trajectory near zero (we use 1e-3 instead to avoid the local
  minima that often occur with zero). The initial guess for the duration will
  be set to `duration`. */
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

  /** Returns the BsplineBasis used to represent both the path over s∈[0,1]. */
  const math::BsplineBasis<double>& basis() const { return basis_; }

  /** Returns the control points defining the path, with control points arrange
  in the columns. */
  const solvers::MatrixXDecisionVariable& control_points() const {
    return control_points_;
  }

  /** Returns the decision variable defining the time duration of the
   * trajectory. */
  const symbolic::Variable& duration() const { return duration_; }

  /** Getter for the optimization program owned by `this`. */
  const solvers::MathematicalProgram& prog() const { return prog_; }

  /** Getter for the optimization program owned by `this`. */
  solvers::MathematicalProgram* get_mutable_prog() { return &prog_; }

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
  `ub`. Note that this is NOT directly constraint q̇(t).
  @pre 0 <= `s` <= 1. */
  void AddPathVelocityConstraint(const Eigen::Ref<const Eigen::VectorXd>& lb,
                                 const Eigen::Ref<const Eigen::VectorXd>& ub,
                                 double s);

  /** Adds a linear constraint on the second derivative of the path,
  `lb` ≤ r̈(s) ≤ `ub`. Note that this is NOT directly constraint q̈(t).
  @pre 0 <= `s` <= 1. */
  void AddPathAccelerationConstraint(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub, double s);

  /** Adds upper and lower bounds on the duration of the trajectory. */
  void AddDurationConstraint(std::optional<double> lb,
                             std::optional<double> ub);

  /** Adds upper and lower bounds on the position trajectory. These bounds will
  be respected at all times. */
  void AddPositionBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                         const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Adds upper and lower bounds on the velocity trajectory. These bounds will
  be respected at all times. */
  void AddVelocityBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
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

  // r is the path, and h is the time_scaling.
  copyable_unique_ptr<trajectories::BsplineTrajectory<symbolic::Expression>>
      sym_r_{};
  copyable_unique_ptr<trajectories::BsplineTrajectory<symbolic::Expression>>
      sym_rdot_{};
  copyable_unique_ptr<trajectories::BsplineTrajectory<symbolic::Expression>>
      sym_rddot_{};
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
