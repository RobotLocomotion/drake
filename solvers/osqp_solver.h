#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {
/**
 * The OSQP solver details after calling Solve() function. The user can call
 * MathematicalProgramResult::get_solver_details<OsqpSolver>() to obtain the
 * details.
 */
struct OsqpSolverDetails {
  /// Number of iterations taken.
  int iter{};
  /// Status of the solver at termination. Please refer to
  /// https://github.com/oxfordcontrol/osqp/blob/master/include/constants.h
  int status_val{};
  /// Norm of primal residue.
  double primal_res{};
  /// Norm of dual residue.
  double dual_res{};
  /// Time taken for setup phase (seconds).
  double setup_time{};
  /// Time taken for solve phase (seconds).
  double solve_time{};
  /// Time taken for polish phase (seconds).
  double polish_time{};
  /// Total OSQP time (seconds).
  double run_time{};
  /// y contains the solution for the Lagrangian multiplier associated with
  /// l <= Ax <= u. The Lagrangian multiplier is set only when OSQP solves
  /// the problem. Notice that the order of the linear constraints are linear
  /// inequality first, and then linear equality constraints.
  Eigen::VectorXd y{};
};

class OsqpSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OsqpSolver)

  /// Type of details stored in MathematicalProgramResult.
  using Details = OsqpSolverDetails;

  OsqpSolver();
  ~OsqpSolver() final;

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  static std::string UnsatisfiedProgramAttributes(const MathematicalProgram&);
  //@}

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;
};
}  // namespace solvers
}  // namespace drake
