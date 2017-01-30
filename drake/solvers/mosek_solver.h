#pragma once

#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class MosekSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MosekSolver)

  MosekSolver() = default;

  /**
   * Defined true if Mosek was included during compilation, false otherwise.
   */
  bool available() const override;

  std::string SolverName() const override { return "Mosek";}

  SolutionResult Solve(MathematicalProgram& prog) const override;
};

}  // namespace solvers
}  // namespace drake
