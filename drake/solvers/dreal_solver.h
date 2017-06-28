#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class DrealSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrealSolver)

  DrealSolver() = default;
  ~DrealSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // Dreal was available during compilation.
  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverType solver_type() const override  { return SolverType::kDReal; }

  std::string SolverName() const override { return id().name(); }

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();
};

}  // namespace solvers
}  // namespace drake
