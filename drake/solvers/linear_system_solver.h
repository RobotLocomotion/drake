#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
class LinearSystemSolver : public MathematicalProgramSolverInterface {
 public:
  bool available() const { return available_impl(); }

  std::string SolverName() const { return SolverName_impl(); }

  std::unique_ptr<MathematicalProgramSolverResult> Solve(
      MathematicalProgram* const prog) const {
    return std::unique_ptr<MathematicalProgramSolverResult>(Solve_impl(prog));
  }

 private:
  bool available_impl() const override;

  std::string SolverName_impl() const override {
    return "Linear System Solver";
  }

  MathematicalProgramSolverResult* Solve_impl(
      MathematicalProgram* const prog) const override;
};

}  // namespace solvers
}  // namespace drake
