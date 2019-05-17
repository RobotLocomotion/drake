#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
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
   * <pre>
   * 0 Problem is solved to optimality.
   * 1 Problem is primal infeasible.
   * 2 Problem is dual infeasible.
   * 3 Problem solved to near optimality.
   * 4 Maximum iterations reached.
   * </pre>
   */
  int return_code;
  /** The primal objective value. */
  double primal_objective;
  /** The dual objective value. */
  double dual_objective;
  /**
   * CSDP solves a primal problem of the form
   * <pre>
   * max tr(C*X)
   * s.t tr(Aᵢ*X) = aᵢ
   *     X ≽ 0
   * </pre>
   * The dual form is
   * <pre>
   * min aᵀy
   * s.t ∑ᵢ yᵢAᵢ - C = Z
   *     Z ≽ 0
   * </pre>
   * y, Z are the variables for the dual problem.
   * y_val, Z_val are the solutions to the dual problem.
   */
  Eigen::VectorXd y_val;
  Eigen::SparseMatrix<double> Z_val;
};

class CsdpSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CsdpSolver)


  CsdpSolver();

  ~CsdpSolver() final;

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  //@}

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

  using Details = CsdpSolverDetails;

 private:
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;
};
}  // namespace solvers
}  // namespace drake
