#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class NloptSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NloptSolver)

  NloptSolver() = default;
  ~NloptSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // NLOpt was available during compilation.
  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverType solver_type() const override { return SolverType::kNlopt; }

  std::string SolverName() const override { return "NLopt"; }
};

}  // namespace solvers
}  // namespace drake
