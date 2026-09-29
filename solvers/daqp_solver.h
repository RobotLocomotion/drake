#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

/** Details returned by DaqpSolver. */
struct DaqpSolverDetails {
  /// DAQP exit flag; 1 means an optimal solution was found.
  int exitflag{};
  /// Active-set iterations.
  int iterations{};
  /// Setup and solution times in seconds (zero when profiling is disabled).
  double setup_time{};
  double solve_time{};
  /// DAQP multipliers: variable bounds first, then linear and equality rows.
  Eigen::VectorXd multipliers{};
};

/** Solves convex quadratic programs using the dense DAQP active-set solver.

Accepts quadratic and linear costs, linear constraints, linear equality
constraints, and bounding boxes. This wrapper performs a fresh setup for every
Solve() call. It does not cache DAQP's workspace between calls. The solver is
selected explicitly; Drake's automatic QP solver preference is unchanged.
The initial guess argument is currently ignored, and variable scaling is not
supported.

Options may be supplied using the DAQPSettings field names `primal_tol`,
`dual_tol`, `iter_limit`, `eps_prox`, `eq_reduction`, and `time_limit`.
See https://darnstrom.github.io/daqp/parameters/ for their meanings.
*/
class DaqpSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DaqpSolver);

  using Details = DaqpSolverDetails;

  DaqpSolver();
  ~DaqpSolver() final;

  static SolverId id();
  static bool is_available();
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  static std::string UnsatisfiedProgramAttributes(const MathematicalProgram&);

  using SolverBase::Solve;

 private:
  void DoSolve2(const MathematicalProgram&, const Eigen::VectorXd&,
                internal::SpecificOptions*,
                MathematicalProgramResult*) const final;
};

}  // namespace solvers
}  // namespace drake
