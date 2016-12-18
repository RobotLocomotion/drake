#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
class EqualityConstrainedQPSolverResult : public MathematicalProgramSolverResult {
 public:
  EqualityConstrainedQPSolverResult(SolutionSummary summary) : MathematicalProgramSolverResult(summary) {};

 private:
  // TODO(hongkai.dai) return the Lagrangian.
};

class EqualityConstrainedQPSolver : public MathematicalProgramSolverInterface {
 public:
  bool available() const {return available_impl();}

  std::string SolverName() const { return SolverName_impl();}

  std::unique_ptr<EqualityConstrainedQPSolverResult> Solve(MathematicalProgram& prog) const {
    return std::unique_ptr<EqualityConstrainedQPSolverResult>(Solve_impl(prog));
  };

 private:
  bool available_impl() const override;

  std::string SolverName_impl() const override { return "Equality Constrained QP Solver";}

  EqualityConstrainedQPSolverResult* Solve_impl(MathematicalProgram& prog) const override;
};

}  // namespace solvers
}  // namespace drake
