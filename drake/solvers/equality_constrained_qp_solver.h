#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class EqualityConstrainedQPSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EqualityConstrainedQPSolver)

  EqualityConstrainedQPSolver() = default;
  ~EqualityConstrainedQPSolver() override = default;

  bool available() const override;

  /**
   * Solve the qudratic program with equality constraint.
   * The user can set the following options
   *  FeasibilityTol. The feasible solution (both primal and dual
   *  variables) should satisfy their constraints, with error no
   *  larger than this value. The default is Eigen::dummy_precision().
   */
  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();
};

}  // namespace solvers
}  // namespace drake
