#pragma once

#include <optional>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"

/**
 * Solves a nonlinear program via the projected gradient descent algorithm. The
 * gradient is determined by differentiation of the costs, or the user can
 * supply a custom gradient function. The projection step is itself an
 * optimization problem in general, but the user can supply a custom projection
 * function. The user can also specify a specific solver interface to be used
 * to solve the projection problem.
 *
 * The solver terminates if
 * - the projection step fails to find a feasible solution,
 * - the norm of the gradient step is less than a user-specified threshold
 * (default 1e-4), or
 * - a maximum number of iterations have been run (default 100).
 */
namespace drake {
namespace solvers {

class ProjectedGradientDescentSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ProjectedGradientDescentSolver);

  ProjectedGradientDescentSolver();
  ~ProjectedGradientDescentSolver() final;

  /** Specify a custom gradient function. Otherwise, this solver will
   * differentiate through the costs in the MathematicalProgram it's used to
   * solve. */
  void SetCustomGradientFunction(
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
          custom_gradient_function) {
    custom_gradient_function_ = custom_gradient_function;
  }

  /** Specify a custom projection function. Otherwise, this solver will attempt
   * to solve the L2 projection onto the feasible set of the MathematicalProgram
   * it's used to solve. The projection function should return a boolean value
   * indicating success or failure. It should take in two arguments: the point
   * we are trying to stay close to, and an output argument where the projected
   * value will be placed. */
  void SetCustomProjectionFunction(
      const std::function<bool(const Eigen::VectorXd&, Eigen::VectorXd*)>&
          custom_projection_function) {
    custom_projection_function_ = custom_projection_function;
  }

  /** Specify a solver interface to be used when solving the L2 projection onto
   * the feasible set of the MathematicalProgram it's being used to solve. */
  void SetProjectionSolverInterface(
      const SolverInterface* projection_solver_interface) {
    projection_solver_interface_ = projection_solver_interface;
  }

  /**
   * @returns string key for SolverOptions to set the threshold used to
   * determine convergence. It must be positive.
   */
  static std::string ConvergenceTolOptionName();

  /**
   * @returns string key for SolverOptions to set the threshold used to
   * determine feasibility of the projection step. It must be positive.
   */
  static std::string FeasibilityTolOptionName();

  /**
   * @returns string key for SolverOptions to set the maximum number of
   * iterations. It must be a postive integer.
   */
  static std::string MaxIterationsOptionName();

  /**
   * @returns string key for SolverOptions to set the value of c to use for the
   * backtracking line search. Must be between 0 and 1.
   */
  static std::string BacktrackingCOptionName();

  /**
   * @returns string key for SolverOptions to set the value of tau to use for
   * the backtracking line search. Must be between 0 and 1.
   */
  static std::string BacktrackingTauOptionName();

  /**
   * @returns string key for SolverOptions to set the value of alpha_0 to use
   * for the backtracking line search. Must be positive.
   */
  static std::string BacktrackingAlpha0OptionName();

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  //@}

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve2(const MathematicalProgram&, const Eigen::VectorXd&,
                internal::SpecificOptions*,
                MathematicalProgramResult*) const final;

  std::optional<std::function<Eigen::VectorXd(const Eigen::VectorXd&)>>
      custom_gradient_function_;
  std::optional<std::function<bool(const Eigen::VectorXd&, Eigen::VectorXd*)>>
      custom_projection_function_;
  const SolverInterface* projection_solver_interface_;
};

}  // namespace solvers
}  // namespace drake
