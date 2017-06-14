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
  ~EqualityConstrainedQPSolver() override = default;

  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverType solver_type() const override {
    return SolverType::kEqualityConstrainedQP;
  }

  std::string SolverName() const override {
    return "Equality constrained QP";
  }
};

}  // namespace solvers
}  // namespace drake
