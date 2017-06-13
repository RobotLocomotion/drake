#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class LinearSystemSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystemSolver)

  LinearSystemSolver() = default;
  ~LinearSystemSolver() override = default;

  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverType solver_type() const override { return SolverType::kLinearSystem; }

  std::string SolverName() const override { return "Linear system"; }
};

}  // namespace solvers
}  // namespace drake
