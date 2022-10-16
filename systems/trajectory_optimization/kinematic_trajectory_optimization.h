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
#include "drake/solvers/solution_result.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

/** Optimizes a position trajectory, q(t), represented as a B-form spline,
subject to constraints on the trajectory and its derivatives.

The q(t) trajectory is commonly associated with, for instance, the generalized
positions of a MultibodyPlant by adding multibody costs and constraints; in
this case take note that the velocities in this optimization are q̇(t), not
v(t). */
class KinematicTrajectoryOptimization {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(KinematicTrajectoryOptimization);

  /** Constructs an optimization problem for a `num_positions`-element position
  trajectory represented as a `spline_order`-order B-form spline with
  `num_control_points` control_points. The initial guess used in solving the
  optimization problem will be the zero-trajectory. */
  KinematicTrajectoryOptimization(int num_positions, int num_control_points,
                                  int spline_order = 4);

  /** Constructs an optimization problem for a position trajectory represented
  by a B-form spline with the same order and number of control points as the
  initial guess, `position_curve_seed`. */
  explicit KinematicTrajectoryOptimization(
      const trajectories::BsplineTrajectory<double>& position_curve_seed);

  /** Resets the position curve. `position_curve` will be rescaled in time to
  run from 0 to 1.
  @pre position_curve.cols == 1
  @pre position_curve.rows() == this->num_positions() */
  void SetPositionCurve(
      const trajectories::BsplineTrajectory<double>& position_curve);

  /** Adds a linear equality constraint on the value of the position trajectory
  at `plan_time`, where `plan_time` represents a fraction of the trajectory's
  duration.
  @pre `plan_time` is between 0 and 1. */
  void AddFixedPositionConstraint(
      const VectorX<double>& desired_position, double plan_time);

  /** Adds a linear equality constraint on the value of the velocity trajectory
  at `plan_time`, where `plan_time` represents a fraction of the trajectory's
  duration.
  @pre `plan_time` is between 0 and 1. */
  void AddFixedVelocityConstraint(
      const VectorX<double>& desired_velocity, double plan_time);

  /** Adds a linear equality constraint on the value of the acceleration
  trajectory at `plan_time`, where `plan_time` represents a fraction of the
  trajectory's duration.
  @pre `plan_time` is between 0 and 1. */
  void AddFixedAccelerationConstraint(
      const VectorX<double>& desired_acceleration, double plan_time);

  /** Adds upper and lower bounds on the duration of the trajectory. */
  void AddDurationBounds(std::optional<double> lower_bound,
                         std::optional<double> upper_bound);

  /** Adds upper and lower bounds on the position trajectory. These bounds will
  be respected at all times. */
  void AddPositionBounds(const VectorX<double>& lower_bound,
                         const VectorX<double>& upper_bound);

  /** Adds upper and lower bounds on the velocity trajectory. These bounds will
  be respected at all times. */
  void AddVelocityBounds(const VectorX<double>& lower_bound,
                         const VectorX<double>& upper_bound);

  /** Adds a linear cost on the duration of the trajectory. */
  void AddDurationCost(double weight = 1.0);

  /** Adds a quadratic cost on the control points of the velocity curve. */
  void AddVelocityCost(double weight = 1.0);

  /** Adds a quadratic cost on the control points of the acceleration curve. */
  void AddAccelerationCost(double weight = 1.0);

  /** Adds a quadratic cost on the control points of the jerk curve. */
  void AddJerkCost(double weight = 1.0);

  // TODO(avalenzu): Provide a form that takes a tolerance as input.
  /** Adds a generic constraint on position, `constraint`, that should be
  satisfied over `plan_interval`. If `validation_constraint` is provided, it
  will be checked at `initial_num_evaluation_points` evenly spaced points
  within `plan_interval`. Otherwise, `constraint` will be checked with a
  tolerance of 5e-3.

  @pre plan_interval[0] <= plan_interval[1].
  */
  void AddGenericPositionConstraint(
      const std::shared_ptr<solvers::Constraint>& constraint,
      const std::array<double, 2>& plan_interval,
      const std::shared_ptr<solvers::Constraint>& validation_constraint =
          nullptr);

  /** Sets an option for a particular solver. See the documentation of
  solvers::MathematicalProgram for more details. Note: Currently only
  SnoptSolver is used by this class. */
  void SetSolverOption(const solvers::SolverId& solver_id,
                       const std::string& solver_option, double option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  /** Sets an option for a particular solver. See the documentation of
  solvers::MathematicalProgram for more details. Note: Currently only
  SnoptSolver is used by this class. */
  void SetSolverOption(const solvers::SolverId& solver_id,
                       const std::string& solver_option, int option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  /** Sets an option for a particular solver. See the documentation of
  solvers::MathematicalProgram for more details. Note: Currently only
  SnoptSolver is used by this class. */
  void SetSolverOption(const solvers::SolverId& solver_id,
                       const std::string& solver_option,
                       const std::string& option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  // TODO(russt): Support passing a solver or using ChooseBestSolver instead of
  // only using Snopt.
  /** Solves the optimization problem. If `always_update_curve` is false (the
  default), the stored position trajectory is updated only when a solution is
  found. If `always_update_curve` is true, the stored position trajectory will
  be updated based on the result of the optimization regardless of whether a
  solution was found. */
  solvers::SolutionResult Solve(bool always_update_curve = false);

  /** Attempts to solve multiple kinematic trajectory optimization problems.
  @returns The solution trajectory for the first successfully solved problem or
  std::nullopt if no solution is found for any problem. */
  static std::optional<trajectories::BsplineTrajectory<double>>
  ComputeFirstSolution(
      std::vector<KinematicTrajectoryOptimization>* programs,
      std::optional<double> min_duration = std::nullopt);

  /** Returns the position trajectory as a B-form spline. */
  trajectories::BsplineTrajectory<double> GetPositionCurve(
      double time_scaling = 1) const;

  /** Returns the number of control points in the position B-spline curve. */
  int num_control_points() const {
    return position_curve_.num_control_points();
  }

  /** Returns the spline order of the position B-spline curve. */
  int spline_order() const { return position_curve_.basis().order(); }

  /** Returns the number of position variables. */
  int num_positions() const { return num_positions_; }

  /** Returns the number of evenly spaced time-points that will be used to
  validate constraints over their respective plan intervals. */
  int num_validation_points() const { return num_validation_points_; }

  /** Returns the maximum number of control points allowed. */
  double max_num_control_points() const { return max_num_control_points_; }

  /** Returns the number of points at which newly-added generic constraints will
  be applied. */
  int initial_num_evaluation_points() const {
    return initial_num_evaluation_points_;
  }

  /** Sets the number of evenly spaced time-points that will be used to
  validate constraints over their respective plan intervals. */
  void set_num_validation_points(int num_evaluation_points) {
    num_validation_points_ = num_evaluation_points;
  }

  /** Sets the maximum number of control points allowed. */
  void set_max_num_control_points(int max_num_control_points) {
    max_num_control_points_ = max_num_control_points;
  }

  /** Sets the number of points at which newly-added generic constraints will
  be applied. */
  void set_initial_num_evaluation_points(int initial_num_evaluation_points) {
    initial_num_evaluation_points_ = initial_num_evaluation_points;
  }

  /** Returns a placeholder decision variable (not actually declared as a
  decision variable in the MathematicalProgram) associated with the generalized
  position vector. */
  const solvers::VectorXDecisionVariable& position() const {
    return placeholder_q_vars_;
  }

  /** Returns a placeholder decision variable (not actually declared as a
  decision variable in the MathematicalProgram) associated with the generalized
  velocity vector. */
  const solvers::VectorXDecisionVariable& velocity() const {
    return placeholder_v_vars_;
  }

  /** Adds a linear constraint on the position and/or velocity over
  `plan_interval`. Only the placeholder variables returned by the `position()`
  and `velocity()` functions can be used in `f`.

  @pre plan_interval[0] <= plan_interval[1]. */
  void AddLinearConstraint(const symbolic::Formula& f,
                           const std::array<double, 2>& plan_interval = {
                               {0.0, 1.0}});

 private:
  struct FormulaWrapper {
    symbolic::Formula formula;
    std::array<double, 2> plan_interval;
  };

  struct ExpressionWrapper {
    symbolic::Expression expression;
    std::array<double, 2> plan_interval;
  };

  struct ConstraintWrapper {
    std::shared_ptr<solvers::Constraint> constraint;
    std::array<double, 2> plan_interval;
    std::shared_ptr<solvers::Constraint> validation_constraint;
    std::set<double> evaluation_times;
  };

  /* Returns a placeholder decision variable (not actually declared as a
  decision variable in the MathematicalProgram) associated with the generalized
  acceleration vector. */
  const solvers::VectorXDecisionVariable& acceleration() const {
    return placeholder_a_vars_;
  }

  /* Returns a placeholder decision variable (not actually declared as a
  decision variable in the MathematicalProgram) associated with the generalized
  jerk vector. */
  const solvers::VectorXDecisionVariable& jerk() const {
    return placeholder_j_vars_;
  }

  const symbolic::Variable& duration() const {
    return placeholder_duration_var_;
  }

  void AddQuadraticCost(const symbolic::Expression& expression);

  void AddLinearCost(const symbolic::Expression& expression);

  void AddLinearConstraintToProgram(
      const FormulaWrapper& constraint,
      solvers::MathematicalProgram* prog) const;

  void AddQuadraticCostToProgram(
      const ExpressionWrapper& cost,
      solvers::MathematicalProgram* prog) const;

  void AddLinearCostToProgram(const ExpressionWrapper& cost,
                              solvers::MathematicalProgram* prog) const;

  void AddGenericPositionConstraintToProgram(
      const ConstraintWrapper& constraint,
      solvers::MathematicalProgram* prog) const;

  void AddPositionPointConstraintToProgram(
      const ConstraintWrapper& constraint, double evaluation_time,
      solvers::MathematicalProgram* prog) const;

  std::vector<symbolic::Substitution>
  ConstructPlaceholderVariableSubstitution(
      const std::vector<solvers::MatrixXDecisionVariable>&
          control_points,
      const std::array<double, 2>& plan_interval) const;

  std::vector<symbolic::Formula> SubstitutePlaceholderVariables(
      const symbolic::Formula& f,
      const std::vector<solvers::MatrixXDecisionVariable>&
          control_points,
      const std::array<double, 2>& plan_interval) const;

  std::vector<symbolic::Expression> SubstitutePlaceholderVariables(
      const symbolic::Expression& expression,
      const std::vector<solvers::MatrixXDecisionVariable>&
          control_points,
      const std::array<double, 2>& plan_interval) const;

  void SetupMathematicalProgram();

  bool ContainsPlaceholders(const symbolic::Variables& vars) const;

  /* Add evaluation points to generic constraints if necessary.
  @returns true if constraints have been modified. */
  bool UpdateGenericConstraints();

  /* Add knots to the position curve.
  @returns true if knots were added. */
  bool AddKnots();

  int num_positions_{};

  /* See description of the public time(), position(), velocity(),
  acceleration() and jerk() accessor functions for details about the
  placeholder variables. */
  solvers::VectorXDecisionVariable placeholder_q_vars_;
  solvers::VectorXDecisionVariable placeholder_v_vars_;
  solvers::VectorXDecisionVariable placeholder_a_vars_;
  solvers::VectorXDecisionVariable placeholder_j_vars_;
  symbolic::Variable placeholder_duration_var_;

  trajectories::BsplineTrajectory<double> position_curve_;

  double duration_{1};

  std::vector<FormulaWrapper> formula_linear_constraints_;

  std::vector<ExpressionWrapper> expression_quadratic_costs_;

  std::vector<ExpressionWrapper> expression_linear_costs_;

  std::vector<ConstraintWrapper> generic_position_constraints_;

  std::vector<solvers::MatrixXDecisionVariable> control_point_variables_;

  solvers::VectorDecisionVariable<1> duration_variable_;

  int num_validation_points_{100};

  int max_num_control_points_{100};

  int initial_num_evaluation_points_{3};

  solvers::SolverOptions solver_options_;
  copyable_unique_ptr<solvers::MathematicalProgram> prog_{};
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
