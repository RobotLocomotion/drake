#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class SnoptSolver : public MathematicalProgramSolverInterface  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SnoptSolver)

  SnoptSolver() = default;
  ~SnoptSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // SNOPT was available during compilation.
  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

  /// @return if the solver is thread safe. SNOPT f2c interface uses global
  /// variables, hence it is not thread safe. SNOPT fortran interface is thread
  /// safe.
  static bool is_thread_safe();

  /// For some reason, when I use SNOPT 7.4, the solver fails to detect a simple
  /// LP being unbounded.
  static bool is_bounded_lp_broken();
};

}  // namespace solvers
}  // namespace drake
