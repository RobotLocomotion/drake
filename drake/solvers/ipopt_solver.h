#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
class IpoptSolverResult : public MathematicalProgramSolverResult {
 public:
  explicit IpoptSolverResult(SolutionSummary summary)
      : MathematicalProgramSolverResult(summary) {}
  // TODO(hongkai.dai): Add Ipopt solver status.
};

class IpoptSolver : public MathematicalProgramSolverInterface {
 public:
  // This solver is implemented in various pieces depending on if
  // Ipopt was available during compilation.
  bool available() const { return available_impl(); }

  std::string SolverName() const { return SolverName_impl(); }

  std::unique_ptr<IpoptSolverResult> Solve(
      MathematicalProgram* const prog) const {
    return std::unique_ptr<IpoptSolverResult>(Solve_impl(prog));
  }

 private:
  bool available_impl() const override;

  std::string SolverName_impl() const override { return "IPOPT"; }

  IpoptSolverResult* Solve_impl(MathematicalProgram* const prog) const override;
};

}  // namespace solvers
}  // namespace drake
