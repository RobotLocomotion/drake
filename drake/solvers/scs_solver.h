#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class ScsSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScsSolver)

  ScsSolver() = default;
  ~ScsSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // SCS was available during compilation.
  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

  // If the user want the solver to print out debug message, then set this to
  // true; otherwise set it to false. The default is false.
  // TODO(hongkai.dai): add function to set the option through
  // MathematicalProgram.
  void SetVerbose(bool verbose) { verbose_ = verbose; }

 private:
  bool verbose_ = false;
};

}  // namespace solvers
}  // namespace drake
