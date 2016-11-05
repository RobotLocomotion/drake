#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class GurobiSolver : public MathematicalProgramSolverInterface {
 public:
  // This solver is implemented in various pieces depending on if
  // Gurobi was available during compilation.
  bool available() const override;

  std::string SolverName() const override {return "Gurobi"; }

  SolutionResult Solve(MathematicalProgram& prog) const override;
};

}  // end namespace solvers
}  // end namespace drake
