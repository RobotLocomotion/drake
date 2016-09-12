#pragma once

#include "drake/drakeOptimization_export.h"

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class DRAKEOPTIMIZATION_EXPORT EqualityConstrainedQPSolver
    : public MathematicalProgramSolverInterface {
 public:
  bool available() const override;
  SolutionResult Solve(MathematicalProgram& prog) const override;
};

}  // namespace solvers
}  // namespace drake
