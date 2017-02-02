#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class EqualityConstrainedQPSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EqualityConstrainedQPSolver)

  EqualityConstrainedQPSolver() = default;

  bool available() const override;

  Solver solver_type() const override { return Solver::kEqualityConstrainedQP; }

  static std::string SolverName() { return "Equality Constrained QP Solver"; }

  SolutionResult Solve(MathematicalProgram& prog) const override;
};

}  // namespace solvers
}  // namespace drake
