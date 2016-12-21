#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
class NloptSolverResult : public MathematicalProgramSolverResult {
 public:
  explicit NloptSolverResult(SolutionSummary summary)
      : MathematicalProgramSolverResult(summary) {}
  // TODO(hongkai.dai): Add nlopt solver status.
};

class NloptSolver : public MathematicalProgramSolverInterface {
 public:
  // This solver is implemented in various pieces depending on if
  // NLOpt was available during compilation.
  bool available() const { return available_impl(); }

  std::string SolverName() const { return SolverName_impl(); }

  std::unique_ptr<NloptSolverResult> Solve(
      MathematicalProgram* const prog) const {
    return std::unique_ptr<NloptSolverResult>(Solve_impl(prog));
  }

 private:
  bool available_impl() const override;

  std::string SolverName_impl() const override { return "NLopt"; }

  NloptSolverResult* Solve_impl(MathematicalProgram* const prog) const override;
};

}  // namespace solvers
}  // namespace drake
