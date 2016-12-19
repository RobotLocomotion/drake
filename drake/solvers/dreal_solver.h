#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
class DrealSolverResult : public MathematicalProgramSolverResult {
 public:
  explicit DrealSolverResult(SolutionSummary summary)
      : MathematicalProgramSolverResult(summary) {}
};

class DrealSolver : public MathematicalProgramSolverInterface {
 public:
  // This solver is implemented in various pieces depending on if
  // Dreal was available during compilation.
  bool available() const { return available_impl(); }

  std::string SolverName() const { return SolverName_impl(); }

  std::unique_ptr<DrealSolverResult> Solve(
      MathematicalProgram* const prog) const {
    return std::unique_ptr<DrealSolverResult>(Solve_impl(prog));
  }

 private:
  bool available_impl() const override;

  std::string SolverName_impl() const override { return "dReal"; }

  DrealSolverResult* Solve_impl(MathematicalProgram* const prog) const override;
};

}  // namespace solvers
}  // namespace drake
