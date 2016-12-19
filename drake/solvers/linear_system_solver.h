#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
class LinearSystemSolverResult : public MathematicalProgramSolverResult {
 public:
  explicit LinearSystemSolverResult(SolutionSummary summary)
      : MathematicalProgramSolverResult(summary) {}
};

class LinearSystemSolver : public MathematicalProgramSolverInterface {
 public:
  bool available() const { return available_impl(); }

  std::string SolverName() const { return SolverName_impl(); }

  std::unique_ptr<LinearSystemSolverResult> Solve(
      MathematicalProgram& prog) const {
    return std::unique_ptr<LinearSystemSolverResult>(Solve_impl(prog));
  }

 private:
  bool available_impl() const override;

  std::string SolverName_impl() const override {
    return "Linear System Solver";
  }

  LinearSystemSolverResult* Solve_impl(
      MathematicalProgram& prog) const override;
};

}  // namespace solvers
}  // namespace drake
