#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class GurobiSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GurobiSolver)

  GurobiSolver() : MathematicalProgramSolverInterface(Solver::kGurobi) {}

  // This solver is implemented in various pieces depending on if
  // Gurobi was available during compilation.
  bool available() const override;

  static std::string SolverName() {return "Gurobi"; }

  SolutionResult Solve(MathematicalProgram& prog) const override;
};

}  // end namespace solvers
}  // end namespace drake
