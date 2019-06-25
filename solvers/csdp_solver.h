#pragma once

#include "drake/common/drake_copyable.h"
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

class CsdpSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CsdpSolver)

  /**
   * CSDP doesn't accept free variables, namely the problem it solves is in this
   * form P1
   *
   *     max tr(C * X)
   *     s.t tr(Aᵢ*X) = aᵢ
   *         X ≽ 0.
   *
   * Notice that the decision variable X has to be in the proper cone X ≽ 0, and
   * it doesn't accept free variable (without the conic constraint). On the
   * other hand, most real-world applications require free variables, namely
   * problems in this form P2
   *
   *     max tr(C * X) + dᵀs
   *     s.t tr(Aᵢ*X) + bᵢᵀs = aᵢ
   *         X ≽ 0
   *         s is free.
   *
   * In order to remove the free variables, we consider two approaches.
   * 1. Replace a free variable s with two variables s = p - q, p ≥ 0, q ≥ 0.
   * 2. First write the dual of the problem P2 as D2
   *
   *        min aᵀy
   *        s.t ∑ᵢ yᵢAᵢ - C = Z
   *            Z ≽ 0
   *            Bᵀ * y = d,
   *
   *    where bᵢᵀ is the i'th row of B.
   *    The last constraint Bᵀ * y = d means y = ŷ + Nt, where Bᵀ * ŷ = d, and N
   *    is the null space of Bᵀ. Hence, D2 is equivalent to the following
   *    problem, D3
   *
   *        min aᵀNt + aᵀŷ
   *        s.t ∑ᵢ tᵢFᵢ - (C -∑ᵢ ŷᵢAᵢ) = Z
   *            Z ≽ 0,
   *
   *    where Fᵢ  = ∑ⱼ NⱼᵢAⱼ. D3 is the dual of the following primal problem P3
   *    without free variables
   *
   *        max tr((C-∑ᵢ ŷᵢAᵢ)*X̂) + aᵀŷ
   *        s.t tr(FᵢX̂) = (Nᵀa)(i)
   *            X̂ ≽ 0.
   *
   *    Then (X, s) = (X̂, B⁻¹(a - tr(Aᵢ X̂))) is the solution to the original
   *    problem P2.
   *
   */
  enum RemoveFreeVariableMethod {
    kTwoSlackVariables,  ///< Approach 1, replace a free variable s as
                         ///< s = y⁺ - y⁻, y⁺ ≥ 0, y⁻ ≥ 0.
    kNullspace,  ///< Approach 2, reformulate the dual problem by considering
                 ///< the nullspace of the linear constraint in the dual.
  };

  explicit CsdpSolver(RemoveFreeVariableMethod method = kNullspace);

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

  RemoveFreeVariableMethod method_;
};
}  // namespace solvers
}  // namespace drake
