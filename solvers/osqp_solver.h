#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {
/**
 * The OSQP solver details after calling Solve function in OsqpSolver. The user
 * can obtain the details from
 * MathematicalProgramResult.get_solver_details().GetValue<OsqpSolverDetails>();
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
};

class OsqpSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OsqpSolver)

  OsqpSolver() = default;
  ~OsqpSolver() override = default;

  // This solver is implemented in various pieces depending on if Osqp was
  // available during compilation.
  bool available() const override { return is_available(); };

  static bool is_available();

  SolutionResult Solve(MathematicalProgram& prog) const override;

  void Solve(const MathematicalProgram&, const optional<Eigen::VectorXd>&,
             const optional<SolverOptions>&,
             MathematicalProgramResult*) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

  bool AreProgramAttributesSatisfied(
      const MathematicalProgram& prog) const override;

  static bool ProgramAttributesSatisfied(const MathematicalProgram& prog);
};
}  // namespace solvers
}  // namespace drake
