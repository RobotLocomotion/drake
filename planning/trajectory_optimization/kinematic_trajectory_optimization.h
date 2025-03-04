#pragma once

#include <array>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace planning {
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

@ingroup planning_trajectory
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
  const math::BsplineBasis<double>& basis() const { return bspline_.basis(); }

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
  solvers::Binding<solvers::LinearConstraint> AddPathPositionConstraint(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub, double s);

  /** Adds a (generic) constraint on path. The constraint will be evaluated
  as if it is bound with variables corresponding to `r(s)`.
  @pre constraint.num_vars() == num_positions()
  @pre 0 <= `s` <= 1. */
  solvers::Binding<solvers::Constraint> AddPathPositionConstraint(
      const std::shared_ptr<solvers::Constraint>& constraint, double s);

  /** Adds a linear constraint on the derivative of the path, `lb` ≤ ṙ(s) ≤
  `ub`. Note that this does NOT directly constrain q̇(t).
  @pre 0 <= `s` <= 1. */
  solvers::Binding<solvers::LinearConstraint> AddPathVelocityConstraint(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub, double s);

  /** Adds a (generic) constraint on trajectory velocity `q̇(t)`, evaluated at
  `s`. The constraint will be evaluated as if it is bound with variables
  corresponding to `[q(T*s), q̇(T*s)]`.

  This is a potentially confusing mix of `s` and `t`, but it is important in
  practice. For instance if you want to constrain the true (trajectory)
  velocity at the final time, one would naturally want to write
  AddVelocityConstraint(constraint, s=1).

  This method should be compared with AddPathVelocityConstraint, which only
  constrains ṙ(s) because it does not reason about the time scaling, T.
  However, AddPathVelocityConstraint adds convex constraints, whereas this
  method adds nonconvex generic constraints.

  @pre constraint.num_vars() == num_positions()
  @pre 0 <= `s` <= 1. */
  solvers::Binding<solvers::Constraint> AddVelocityConstraintAtNormalizedTime(
      const std::shared_ptr<solvers::Constraint>& constraint, double s);

  /** Adds a linear constraint on some (or all) of the placeholder variables
   qdot, evaluated at a normalized time s.

   @pre binding Can only associate with qdot().
   @pre 0 <= `s` <= 1.

   @code{cpp}
   Binding<LinearConstraint> b(LinearConstraint(A, b), trajopt.qdot());
   trajopt.AddVelocityConstraintAtNormalizedTime(b, 0);
   @endcode
  */
  std::vector<solvers::Binding<solvers::LinearConstraint>>
  AddVelocityConstraintAtNormalizedTime(
      const solvers::Binding<solvers::LinearConstraint>& binding, double s);

  /** Adds a linear constraint on the second derivative of the path,
  `lb` ≤ r̈(s) ≤ `ub`. Note that this does NOT directly constrain q̈(t).
  @pre 0 <= `s` <= 1. */
  solvers::Binding<solvers::LinearConstraint> AddPathAccelerationConstraint(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub, double s);

  /** Adds bounding box constraints for upper and lower bounds on the duration
   * of the trajectory. */
  solvers::Binding<solvers::BoundingBoxConstraint> AddDurationConstraint(
      std::optional<double> lb, std::optional<double> ub);

  /** Adds bounding box constraints to enforce upper and lower bounds on the
  positions trajectory, q(t). These bounds will be respected at all times,
  t∈[0,T]. This also implies the constraints are satisfied for r(s), for all
  s∈[0,1].

  @returns A vector of bindings with the ith element adding a constraint to the
  ith control point. However, this specific interpretation of these constraints
  may change without deprecation.
  */
  std::vector<solvers::Binding<solvers::BoundingBoxConstraint>>
  AddPositionBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                    const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Adds linear constraints to enforce upper and lower bounds on the velocity
  trajectory, q̇(t). By leveraging the convex hull property of B-splines, these
  bounds are applied at the (derivative) control points, but will be respected
  for all times, t ∈ [0,T]. Note this does NOT directly constrain ṙ(s).

  @returns A vector of bindings with interleaved lower and then upper bounds,
  constraining all of the control points for one element of q̇ (e.g. the velocity
  of the ith joint at all times). However, this specific interpretation of these
  constraints may change without deprecation..*/
  std::vector<solvers::Binding<solvers::LinearConstraint>> AddVelocityBounds(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Adds generic (nonlinear) constraints to enforce the upper and lower
  bounds to the acceleration trajectory, q̈(t).  By leveraging the convex hull
  property of B-splines, these bounds are applied at the (derivative) control
  points, but will be respected for all times, t ∈ [0,T].  Note that this does
  NOT directly constrain r̈(s).

  @returns A vector of bindings with interleaved lower and then upper bounds,
  constraining all of the control points for one element of q̈ (e.g. the
  acceleration of the ith joint at all times). However, this specific
  interpretation of these constraints may change without deprecation. */
  std::vector<solvers::Binding<solvers::Constraint>> AddAccelerationBounds(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Adds generic (nonlinear) constraints to enforce the upper and lower
  bounds to the jerk trajectory, d³qdt³(t).  By leveraging the convex hull
  property of B-splines, these bounds are applied at the (derivative) control
  points, but will be respected for all times, t ∈ [0,T]. Note that this does
  NOT directly constrain d³rds³(s).

  @returns A vector of bindings with interleaved lower and then upper bounds,
  constraining all of the control points for one element of d³qdt³ (e.g. the
  jerk of the ith joint at all times). However, this specific interpretation of
  these constraints may change without deprecation. */
  std::vector<solvers::Binding<solvers::Constraint>> AddJerkBounds(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Adds generic (nonlinear) constraints to enforce the effort limits defined
  in the plant at a sequence of normalized times, `s`:
  @verbatim
  B lb ≤ M(q)v̇ + C(q, v)v - τ_g(q) - τ_app ≤ B ub
  @endverbatim
  where q, v, and v̇ are evaluated at s. B is the plant's actuation matrix, and
  M, C, τ_g, and τ_app are the plant's mass matrix, Coriolis force, gravity,
  and applied force, respectively. `ub` and `lb` are the upper and lower effort
  bounds, respectively; if they are not provided then
  plant.GetEffortLowerLimits() and plant.GetEffortUpperLimits() are used.

  Pass `plant_context` if you have non-default parameters in the context. Note
  that there are no lifetime requirements on `plant` nor `plant_context`.

  Note that the convex hull property of the B-splines is not guaranteed to hold
  here -- effort limits maybe be violated away from the normalized times `s`.

  @pre plant.is_finalized()
  @pre plant.num_positions() == num_positions()
  @pre plant.IsVelocityEqualToQDot() == true
  @pre s[i] ∈ [0, 1] for all i
  @pre B lb ≤ B ub

  @returns A vector of bindings with one effort limit constraint for each `s`.
  */
  std::vector<solvers::Binding<solvers::Constraint>>
  AddEffortBoundsAtNormalizedTimes(
      const multibody::MultibodyPlant<double>& plant,
      const Eigen::Ref<const Eigen::VectorXd>& s,
      const std::optional<Eigen::Ref<const Eigen::VectorXd>>& lb = std::nullopt,
      const std::optional<Eigen::Ref<const Eigen::VectorXd>>& ub = std::nullopt,
      const systems::Context<double>* plant_context = nullptr);

  /** Adds a linear cost on the duration of the trajectory. */
  solvers::Binding<solvers::LinearCost> AddDurationCost(double weight = 1.0);

  /** Adds a cost on an upper bound of the length of the path, ∫₀ᵀ |q̇(t)|₂ dt,
  or equivalently ∫₀¹ |ṙ(s)|₂ ds, by summing the distance between the path
  control points. If `use_conic_constraint = false`, then costs are added via
  MathematicalProgram::AddL2NormCost; otherwise they are added via
  MathematicalProgram::AddL2NormCostUsingConicConstraint.

  @returns A vector of bindings with the ith element adding a cost to the
  ith control point of the velocity trajectory. However, this specific
  interpretation of these constraints may change without deprecation. */
  std::vector<solvers::Binding<solvers::Cost>> AddPathLengthCost(
      double weight = 1.0, bool use_conic_constraint = false);

  /** Adds a convex quadratic cost on an upper bound on the energy of the path,
  ∫₀¹ |ṙ(s)|₂² ds, by summing the squared distance between the path control
  points. In the limit of infinitely many control points, minimizers for
  AddPathLengthCost and AddPathEnergyCost will follow the same path, but
  potentially with different timing. They may have different values if
  additional costs and constraints are imposed. This cost yields simpler
  gradients than AddPathLengthCost, and biases the control points towards being
  evenly spaced.

  @returns A vector of bindings with the ith element adding a cost to the
  ith control point of the velocity trajectory. However, this specific
  interpretation of these constraints may change without deprecation. */
  std::vector<solvers::Binding<solvers::Cost>> AddPathEnergyCost(
      double weight = 1.0);

  /** Returns the placeholder variable for generalized position q. Note these
   are NOT decision variables in the MathematicalProgram. These variables will
   be substituted for the real decision variables at particular times. Passing
   these variables directily into objective/constraints for the
   MathematicalProgram will result in an error.
   */
  const VectorX<symbolic::Variable>& q() const { return placeholder_q_vars_; }

  /** Returns the placeholder variable for the time derivative of generalized
   position. Note these are NOT decision variables in the MathematicalProgram.
   These variables will be substituted for the real decision variables at
   particular times. Passing these variables directily into
   objective/constraints for the MathematicalProgram will result in an error.
   */
  const VectorX<symbolic::Variable>& qdot() const {
    return placeholder_qdot_vars_;
  }

  /** Returns the placeholder variable for the second time derivative of
   generalized position. Note these are NOT decision variables in the
   MathematicalProgram. These variables will be substituted for the real
   decision variables at particular times. Passing these variables directily
   into objective/constraints for the MathematicalProgram will result in an
   error.
   */
  const VectorX<symbolic::Variable>& qddot() const {
    return placeholder_qddot_vars_;
  }

  /* TODO(russt):
  - Support additional (non-convex) costs/constraints on q(t) directly.
  - AddMultibodyPlantConstraints which adds joint, velocity, acceleration, and
    effort limits + multibody constraints. Presumably we can't have quaternions
    in a fully-actuated system.
  */

 private:
  solvers::MathematicalProgram prog_{};
  int num_positions_{};
  int num_control_points_{};

  trajectories::BsplineTrajectory<double> bspline_;
  solvers::MatrixXDecisionVariable control_points_;
  symbolic::Variable duration_;
  VectorX<symbolic::Variable> placeholder_q_vars_;
  VectorX<symbolic::Variable> placeholder_qdot_vars_;
  VectorX<symbolic::Variable> placeholder_qddot_vars_;
};

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
