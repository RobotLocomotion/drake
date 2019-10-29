#pragma once

#define DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Classname)      \
  Classname(const Classname&) = delete;                 \
  void operator=(const Classname&) = delete;            \
  Classname(Classname&&) = delete;                      \
  void operator=(Classname&&) = delete;

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
};

struct SolverId {};
struct MathematicalProgram {};
struct MathematicalProgramResult {};
struct VectorXd {};
struct SolverOptions {};

class OsqpSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OsqpSolver)

  /// Type of details stored in MathematicalProgramResult.
  using Details = OsqpSolverDetails;

  OsqpSolver();
  ~OsqpSolver();

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  //@}

 private:
  void DoSolve(const MathematicalProgram&, const VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const;
};
}  // namespace solvers

namespace dev {
// Must be ignored
struct OsqpSolverDetails1 {
  int iter{};
  int status_val{};
  double primal_res{};
};

}
}  // namespace drake
