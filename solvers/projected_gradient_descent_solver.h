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

  /**
   * @returns string key for SolverOptions to set the custom gradient function.
   */
  static std::string CustomGradientFunctionOptionName();

  /**
   * @returns string key for SolverOptions to set the custom projection
   * function.
   */
  static std::string CustomProjectionFunctionOptionName();

  /**
   * @returns string key for SolverOptions to set the solver interface used to
   * solve the projection program. This options is ignored if a custom
   * projection function has been specified.
   */
  static std::string ProjectionSolverInterfaceOptionName();

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
};

}  // namespace solvers
}  // namespace drake
