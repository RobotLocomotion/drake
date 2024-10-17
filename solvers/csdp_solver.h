#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/solvers/sdpa_free_format.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {
/**
 * The CSDP solver details after calling Solve() function. The user can call
 * MathematicalProgramResult::get_solver_details<CsdpSolver>() to obtain the
 * details.
 */
struct CsdpSolverDetails {
  /** Refer to the Return Codes section of CSDP 6.2.0 User's Guide for
   * explanation on the return code. Some of the common return codes are
   *
   *     0 Problem is solved to optimality.
   *     1 Problem is primal infeasible.
   *     2 Problem is dual infeasible.
   *     3 Problem solved to near optimality.
   *     4 Maximum iterations reached.
   *     5 Stuck at edge of primal feasibility.
   *     6 Stuck at edge of dual feasibility.
   *     7 Lack of progress.
   *     8 X, Z, or O is singular.
   *     9 NaN or Inf values encountered.
   */
  int return_code{};
  /** The primal objective value. */
  double primal_objective{};
  /** The dual objective value. */
  double dual_objective{};
  /**
   * CSDP solves a primal problem of the form
   *
   *     max tr(C*X)
   *     s.t tr(Aᵢ*X) = aᵢ
   *         X ≽ 0
   *
   * The dual form is
   *
   *     min aᵀy
   *     s.t ∑ᵢ yᵢAᵢ - C = Z
   *         Z ≽ 0
   *
   * y, Z are the variables for the dual problem.
   * y_val, Z_val are the solutions to the dual problem.
   */
  Eigen::VectorXd y_val;
  Eigen::SparseMatrix<double> Z_val;
};

/**
 * Wrap CSDP solver such that it can solve a
 * drake::solvers::MathematicalProgram.
 * @note CSDP doesn't accept free variables, while
 * drake::solvers::MathematicalProgram does. In order to convert
 * MathematicalProgram into CSDP format, we provide several approaches to remove
 * free variables. You can set the approach through
 * @code{cc}
 * SolverOptions solver_options;
 * solver_options.SetOption(CsdpSolver::id(),
 *    "drake::RemoveFreeVariableMethod",
 *    static_cast<int>(RemoveFreeVariableMethod::kNullspace));
 * CsdpSolver solver;
 * auto result = solver.Solve(prog, std::nullopt, solver_options);
 * @endcode
 * For more details, check out RemoveFreeVariableMethod.
 */
class CsdpSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CsdpSolver);

  /** Default constructor */
  CsdpSolver();

  ~CsdpSolver() final;

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  //@}

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

  using Details = CsdpSolverDetails;

 private:
  void DoSolve2(const MathematicalProgram&, const Eigen::VectorXd&,
                internal::SpecificOptions*,
                MathematicalProgramResult*) const final;
};
}  // namespace solvers
}  // namespace drake
