#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class EqualityConstrainedQPSolver : public MathematicalProgramSolverInterface {
 public:
  bool available() const override;

  std::string SolverName() const override {
    return "Equality Constrained QP Solver";
  }

  SolutionResult Solve(MathematicalProgram& prog) const override;
};

}  // namespace solvers
}  // namespace drake
