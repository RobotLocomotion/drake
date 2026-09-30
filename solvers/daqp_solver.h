#pragma once

#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

/// The DAQP solver details after calling the Solve() function. The user can
/// call MathematicalProgramResult::get_solver_details<DaqpSolver>() to obtain
/// the details.
struct DaqpSolverDetails {
  /// The exit flag returned by DAQP; 1 means that an optimal solution was
  /// found. Refer to DAQP_EXIT_* in
  /// https://github.com/darnstrom/daqp/blob/master/include/constants.h
  int exitflag{};
  /// Number of active-set iterations.
  int iterations{};
  /// Time spent in DAQP's setup phase (seconds).
  double setup_time{};
  /// Time spent in DAQP's solve phase (seconds).
  double solve_time{};
  /// The Lagrange multipliers computed by DAQP, in DAQP's sign convention
  /// (positive at an active upper bound). The first MathematicalProgram::
  /// num_vars() entries are for the variable bounds, followed by the rows of
  /// the linear inequality constraints, and then the rows of the linear
  /// equality constraints. Set only when DAQP solves the problem.
  Eigen::VectorXd multipliers{};
};

/** A wrapper to call [DAQP](https://github.com/darnstrom/daqp) using Drake's
MathematicalProgram.

DAQP is a dual active-set solver for convex quadratic programs. It works on
dense matrices, so it is well suited to small and medium sized QPs (e.g., up
to a few hundred variables), such as those in differential inverse kinematics
and model predictive control, where it is typically much faster and more
accurate than first-order methods. It does not exploit sparsity, so for large
sparse QPs prefer a sparse solver such as ClarabelSolver or OsqpSolver.

DaqpSolver accepts quadratic and linear costs, linear constraints, linear
equality constraints, and bounding box constraints. A quadratic cost is
required, and it must be convex; positive semidefinite Hessians are
supported. The solver performs a fresh setup on every call to Solve(); the
initial guess is ignored, and variable scaling is ignored (with a warning).

<b>Solver options</b>

DAQP's options may be set with SolverOptions using the field names of
DAQPSettings:

- `primal_tol` (double)
- `dual_tol` (double)
- `zero_tol` (double)
- `pivot_tol` (double)
- `progress_tol` (double)
- `sing_tol` (double)
- `refactor_tol` (double)
- `cycle_tol` (int)
- `iter_limit` (int)
- `time_limit` (double, seconds)
- `fval_bound` (double)
- `eps_prox` (double)
- `eta_prox` (double)
- `eq_reduction` (int)

Refer to https://darnstrom.github.io/daqp/parameters/ for their meanings and
default values. An unrecognized option name throws an exception. DaqpSolver
does not print to the console, so CommonSolverOption::kPrintToConsole and
CommonSolverOption::kPrintFileName have no effect; DAQP is single-threaded,
so CommonSolverOption::kMaxThreads has no effect.

<b>Solution result</b>

When DAQP reaches its iteration limit (`iter_limit`) or time limit
(`time_limit`), the result is SolutionResult::kIterationLimit. Note that when
the Hessian is only positive semidefinite, DAQP does not detect an unbounded
problem; it reports SolutionResult::kIterationLimit after `iter_limit`
proximal-point iterations instead of SolutionResult::kUnbounded. */
class DaqpSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DaqpSolver);

  /// Type of details stored in MathematicalProgramResult.
  using Details = DaqpSolverDetails;

  DaqpSolver();
  ~DaqpSolver() final;

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
  void DoSolve2(const MathematicalProgram&, const Eigen::VectorXd&,
                internal::SpecificOptions*,
                MathematicalProgramResult*) const final;
};

}  // namespace solvers
}  // namespace drake
